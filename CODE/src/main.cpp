
/*
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

*/

#include "PIN_mapping.h"    // fichier contenant les variables globales et le mapping des pins
bool emergencyStop = false; // variable pour stocker l'état de l'arrêt d'urgence

#include <Arduino.h>
#include <esp_dmx.h>
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
  // delay(1000);
  Serial.begin(115200);
  Serial.println("started");

  dmx_set_pin(dmx_num, tx_pin, rx_pin, rts_pin);
dmx_driver_install(dmx_num, &config, DMX_INTR_FLAGS_DEFAULT);

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
    // attach an interrupt to the IO pin of the ermegency stop switch and specify the handler function
    pinMode(EMERGENCY_STOP_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(EMERGENCY_STOP_PIN), emergencySwitchHandler, RISING);
  */

  // lecture des potentiomètres pour définir les offset de pas a ajouter aux positions home et limit
  limit_steps = map((analogRead(limit_set_pot)), 4095, 0, min_offset, max_offset);
  home_steps = map((analogRead(home_set_pot)), 4095, 0, min_offset, max_offset);

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
  stepper.startAsService(1); // core 1 sinon bug avec le dmx et déplacement aléatoire du moteur

  init_range(); // set home and limit position of the motor

  DMX_start_channel = readSwitch(); // lecture de la position du codeurs BCD pour définir le canal DMX de départ
  // DMX_start_channel = 2;            // pour test avec adresse fixe
  pos_channel = DMX_start_channel;
  speed_channel = DMX_start_channel + 1;

  Serial.println("----------------------------------------");
  Serial.printf("number of tasks is %i\n", uxTaskGetNumberOfTasks());
  Serial.printf("home position is %i\n", min_steps);
  Serial.printf("limit position is %i\n", max_steps);
  Serial.printf("Limit setting is %i\n", limit_steps);
  Serial.printf("Home setting is %i\n", home_steps);
  Serial.printf("DMX start channel is %i\n", DMX_start_channel);
  Serial.printf("DMX position channel is %i\n", pos_channel);
  Serial.printf("DMX speed channel is %i\n", speed_channel);
  Serial.println("----------------------------------------");
}

void loop()
{
  receiveDMX(); // reception des données DMX

  // mise à jour de la vitesse du moteur si la valeur DMX a changée
  if (dataChanged[speed_array] == true)
  {
    dataChanged[speed_array] = false;
    speed_map(); // fonction pour convertir la valeur DMX en vitesse/acceleration/deceleration
  }

  // mise à jour de la position du moteur si la valeur DMX a changée
  // if (dataChanged[pos_array] == true && emergencyStop == false && digitalRead(MOTOR_ENABLE_PIN) == LOW)
  if (dataChanged[pos_array] == true)
  {
    // stepper.setTargetPositionToStop();
    dataChanged[pos_array] = false;
    pos_in_step = map(DMX_data[pos_array], 255, 0, min_steps, max_steps);
    // pos_map(); // fonction pour convertir la valeur DMX en position
    stepper.setTargetPositionInSteps(pos_in_step);
    DMX_data_old[pos_array] = DMX_data[pos_array];
    last_dmx_change_pos = millis();
    Serial.printf("dmx pos to step => %ld\n", pos_in_step);
  }
  // gestion de la position du moteur si la valeur DMX n'a pas changée mais que le moteur n'est pas à la position demandée
  if (millis() - last_dmx_change_pos > 1000)
  {
    int codeur_pos = map(encoder.getCount(), min_count, max_count, min_steps, max_steps);
    if (pos_in_step <= min_steps && ConfirmedHomeSwitchState != switch_active && stepper.getDirectionOfMotion() == 0)
    {
      Serial.println("!!!!!!!!!!!!! home not reached");
      Serial.printf("codeur pos => %i\n", codeur_pos);
      // stepper.setTargetPositionInSteps(min_steps - codeur_pos); // home_steps
      int step_add = nb_microstep * 2;
      stepper.setTargetPositionRelativeInSteps(-step_add);
      delay(100);
    }
    else if (pos_in_step >= max_steps && ConfirmedLimitSwitchState != switch_active && stepper.getDirectionOfMotion() == 0)
    {
      Serial.println("!!!!!!!!!!!!! limit not reached");
      Serial.printf("codeur pos => %i\n", codeur_pos);
      // stepper.setTargetPositionInSteps(max_steps + (max_steps - codeur_pos)); // limit_steps
      int step_add = nb_microstep * 2;
      stepper.setTargetPositionRelativeInSteps(step_add);
      delay(100);
    }
    else if (abs(codeur_pos - pos_in_step) > 100 && stepper.getDirectionOfMotion() == 0)
    {
      Serial.println("!!!!!!!!!!!!! Not in position");
      Serial.printf("codeur pos => %i\n", codeur_pos);
      Serial.printf("pos in step => %i\n", pos_in_step);
      stepper.setTargetPositionInSteps(pos_in_step);
    }
  }
  // motor_follower(emergency_stop_loss_step); // fonction pour suivre la position du moteur et la perte de pas
  emergency_check(); // fonction pour la sécurité si un obstacle est détecté par la perte de pas du moteur
  limit_check();     // fonction pour la sécurité si un obstacle est détecté par le bouton poussoir
}