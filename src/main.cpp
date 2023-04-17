
/*

TESTER AVEC LIB : https://github.com/Valar-Systems/FastAccelStepper-ESP32 ???

controle de moteur pas à pas selon commande DMX

code par Clément SAILLANT pour Hémisphère - 2023
c.saillant@gmail.com
0625334420

Un canal DMX est utilisé pour la commande de la position du moteur
Un autre canal DMX est utilisé pour la commande de la vitesse du moteur
Une fonction affine est utilisée pour convertir la valeur DMX en position ou vitesse
La position et la vitesse sont stockées dans des variables globales
Une fonction est utilisée pour la sécurité si un obstacle est détecté par la perte de pas du moteur

moteur : 34HS59-6004D-E1000
Angle de pas: 1.8 deg
Résolution de l'encodeur: 1000PPR(4000CPR)
https://www.omc-stepperonline.com/download/34HS59-6004D-E1000_Torque_Curve.pdf
https://www.omc-stepperonline.com/download/34HS59-6004D-E1000.pdf

Controleur CL86T V4.0
https://www.omc-stepperonline.com/index.php?route=product/product/get_file&file=389/CL86T%20(V4.0).pdf

SW1 0
SW2 0
SW3 1
SW4 1

SW5 On  CCW
SW6 Off Close loop
SW7 Off Pul/Dir
SW8 Off Brk
*/

// #define dev_mode // mode developpeur

#include "PIN_mapping.h"    // fichier contenant les variables globales et le mapping des pins
bool emergencyStop = false; // variable pour stocker l'état de l'arrêt d'urgence

#include <Arduino.h>
// #include <Arduino_FreeRTOS.h>
// #include "esp_attr.h"

#include <esp_dmx.h>
// #include "esp_task_wdt.h"     // librairie pour le watchdog
#include <ESP_FlexyStepper.h> // librairie pour le controle des moteurs pas à pas
#include <ESP32Encoder.h>     // librairie pour le controle des encodeurs

ESP_FlexyStepper stepper; // création de l'ojet motor
ESP32Encoder encoder;     // création de l'objet encoder

#include "limit.h"    // fichier contenant les fonctions de limites
#include "dmx_data.h" // fichier contenant les fonctions dmx
#include "moving.h"   // fichier contenant les fonctions de mouvement
#include "security.h" // fichier contenant les fonctions de sécurité type emergency stop

void setup()
{
  delay(1000);
  Serial.begin(115200);
  Serial.println("started");

  // configuration DMX
  dmx_set_pin(dmx_num, tx_pin, rx_pin, rts_pin);
  dmx_driver_install(dmx_num, DMX_DEFAULT_INTR_FLAGS);

  // configuration encoder pour suivre la position du moteur et la perte de pas
  encoder.attachHalfQuad(EB_plus, EA_plus);
  encoder.setCount(0);

  // configuration ENABLE motor
  pinMode(MOTOR_ENABLE_PIN, OUTPUT);
  // configuration BCD Coder
  pinMode(q1, INPUT); // thumbwheel '1'
  pinMode(q2, INPUT); // thumbwheel '2'
  pinMode(q4, INPUT); // thumbwheel '4'
  pinMode(q8, INPUT); // thumbwheel '8'

  /*
  #ifndef dev_mode
    // attach an interrupt to the IO pin of the ermegency stop switch and specify the handler function
    pinMode(EMERGENCY_STOP_PIN, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(EMERGENCY_STOP_PIN), emergencySwitchHandler, RISING);
  #endif
  */
  pinMode(MOTOR_ENABLE_PIN, OUTPUT);
  pinMode(LIMIT_SWITCH_PIN, INPUT);
  pinMode(HOME_SWITCH_PIN, INPUT);


  // ???????????????? Error : attachInterrupt with ADC enabled :'(  ???????????????? 

 // attach an interrupt to the IO pin of the home switch and specify the handler function
  attachInterrupt(digitalPinToInterrupt(HOME_SWITCH_PIN), homeSwitchHandler, CHANGE);
  stepper.registerHomeReachedCallback(homeReachedCallback);

  // attach an interrupt to the IO pin of the limit switch and specify the handler function
  attachInterrupt(digitalPinToInterrupt(LIMIT_SWITCH_PIN), limitSwitchHandler, CHANGE);
  stepper.registerLimitReachedCallback(limitReachedCallback);

  // connect and configure the stepper motor to its IO pins
  stepper.connectToPins(MOTOR_STEP_PIN, MOTOR_DIRECTION_PIN);
  // set the speed and acceleration rates for the stepper motor
  stepper.setSpeedInStepsPerSecond(SPEED_IN_STEPS_PER_SECOND);
  stepper.setAccelerationInStepsPerSecondPerSecond(ACCELERATION_IN_STEPS_PER_SECOND);
  stepper.setDecelerationInStepsPerSecondPerSecond(DECELERATION_IN_STEPS_PER_SECOND);
  stepper.setStepsPerRevolution(STEP_PER_REV);
  stepper.registerTargetPositionReachedCallback(targetPositionReachedCallback);

  // start the stepper instance as a service in the "background" as a separate task
  stepper.startAsService(); // at core 1

  init_range(); // set home and limit position of the motor

  Serial.print("number of tasks is ");
  Serial.println(uxTaskGetNumberOfTasks());
  Serial.print("home position is ");
  Serial.println(min_steps);
  Serial.print("limit position is ");
  Serial.println(max_steps);
  Serial.println("----------------------------------------");
}

void loop()
{
  receiveDMX(); // reception des données DMX

  // mise à jour de la vitesse du moteur si la valeur DMX a changé
  if (dataChanged[speed_array])
  {
    dataChanged[speed_array] = false;
    Serial.printf("dmx speed receive => %ld\n", DMX_data[speed_array]);
    speed_map(); // fonction pour convertir la valeur DMX en vitesse/acceleration/deceleration
  }

  // mise à jour de la position du moteur si la valeur DMX a changée
  if (dataChanged[pos_channel] == true && emergencyStop == false)
  {
    // stepper.setTargetPositionToStop();
    dataChanged[pos_channel] = false;
    int pos_in_step = map(DMX_data[pos_array], 0, 255, min_steps, max_steps);

    // adaptation de la vitesse d'acceleration en fonction de la distance à parcourir
    if (abs(DMX_data_old[pos_array] - DMX_data[pos_array]) < 25)
    {
      speed_in_accel = 50;
    }
    else
    {
      speed_in_accel = 500;
    }
    stepper.setAccelerationInStepsPerSecondPerSecond(speed_in_accel);
    stepper.setTargetPositionInSteps(pos_in_step);
    Serial.printf("dmx pos to step => %ld\n", pos_in_step);
    DMX_data_old[pos_array] = DMX_data[pos_array];
  }

  // motor_follower(emergency_stop_loss_step); // fonction pour suivre la position du moteur et la perte de pas
  emergency_check(); // fonction pour la sécurité si un obstacle est détecté par la perte de pas du moteur
  limit_check();     // fonction pour la sécurité si un obstacle est détecté par le bouton poussoir
}