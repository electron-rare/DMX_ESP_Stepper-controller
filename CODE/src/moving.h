
#define nb_microstep 8         // nombre de microstep configuré sur le driver moteur
#define STEP_PER_REV_MOTOR 200 // nombre de pas par tour du moteur
#define CPR_ENCODER 4000       // nombre de pas par tour du codeur optique
/*
Nb              SW1     SW2     SW3     SW4
1      200      on      on      on      on
4      800      off     on      on      on
8     1600      on      off     on      on
16    3200      off     off     on      on
32    6400      on      on      off     on
64   12800      off     on      off     on
128  25600      on      off     off     on
256  51200      off     off     off     on
5     1000      on      on      on      off
10    2000      off     on      on      off
20    4000      on      off     on      off
25    5000      off     off     on      off
40    8000      on      on      off     off
50   10000      off     on      off     off
100  20000      on      off     off     off
200  40000      off     off     off     off
*/
#define SECOND_PER_REV 2                                                                       // nombre de seconde pour faire un tour
const int STEP_PER_REV = nb_microstep * STEP_PER_REV_MOTOR;                                    // nombre de pas par tour du moteur
const int MIN_SPEED = 1;                                                                       // vitesse minimum du moteur
const int MAX_SPEED = ((STEP_PER_REV / 2) / SECOND_PER_REV) * nb_microstep;                    // vitesse maximum du moteur (1/2 révolution par SECOND_PER_REV seconde)
//const int MAX_SPEED = ((STEP_PER_REV / 2) / SECOND_PER_REV) * nb_microstep;                    // vitesse maximum du moteur (1/2 révolution par SECOND_PER_REV seconde)

const int SPEED_IN_STEPS_PER_SECOND = STEP_PER_REV;                                           // vitesse initiale de déplacement du moteur = 1/2 de la vitesse maximum
const int ACCELERATION_IN_STEPS_PER_SECOND = nb_microstep * 2;                                 // accélération initiale du moteur
const int DECELERATION_IN_STEPS_PER_SECOND = nb_microstep;                                     // décélération initiale du moteur
int ACC_DEC_REMOVE_STEP = ACCELERATION_IN_STEPS_PER_SECOND + DECELERATION_IN_STEPS_PER_SECOND; // nombre de pas à enlever à la position pour l'accélération et la décélération ?
int min_steps = 0;                                                                             // position de départ du moteur
int max_steps = STEP_PER_REV / 2;                                                              // position de fin du moteur (1600 en 16 microstep) (400 en 1 microstep)
int pos_in_step = max_steps / 2;                                                               // position du moteur en pas pour le mappage de la position

int min_count = 0;                       // valeur minimum du compteur du codeur optique
int max_count = CPR_ENCODER / 4;         // valeur maximum du compteur du codeur optique avec uniquement les fronts montants du codeur
const int min_offset = 0;                // min offset pour le potentiomètre de réglage de la position min/max
const int max_offset = nb_microstep * 5; // max offset pour le potentiomètre de réglage de la position min/max

volatile unsigned long last_dmx_change_pos;

volatile int speed_in_accel = MAX_SPEED / 10;                   // variable pour la vitesse d'acceleration
volatile int speed_in_decel = DECELERATION_IN_STEPS_PER_SECOND; // variable pour la vitesse de deceleration

// fonction
void targetPositionReachedCallback(long position); // callback function as the target position is reached
void speed_map();                                  // fonction de mapping de la vitesse du moteur avec accélération et décélération
void pos_map();                                    // fonction de mapping de la position du moteur
void set_min();                                    // fonction pour position minimum de la plage de déplacement du moteur
void set_max();                                    // fonction pour position maximum de la plage de déplacement du moteurF
void init_range();                                 // fonction d'initialisation de la plage de déplacement du moteur

void targetPositionReachedCallback(long position) // callback function as the target position is reached
{
  Serial.printf("Stepper reached target position %ld\n", position);
  Serial.printf("Encoder map count is %ld\n", map(encoder.getCount(), min_count, max_count, min_steps, max_steps));
  Serial.printf("Encoder count is %ld\n", encoder.getCount());
}

void speed_map() // fonction de mapping de la vitesse du moteur avec accélération et décélération
{
  int speed_in_step = map(DMX_data[speed_array], 0, 255, MIN_SPEED, MAX_SPEED);
  stepper.setSpeedInStepsPerSecond(speed_in_step);
  Serial.printf("dmx speed receive => %ld\n", DMX_data[speed_array]);
  Serial.printf("speed in step => %ld\n", speed_in_step);
}

void pos_map() // fonction de mapping de la position du moteur
{
  //int pos_in_step = map(DMX_data[pos_array], 255, 0, min_steps, max_steps);
  //int pos_in_step = map(DMX_data[pos_array], 0, 255, min_steps - home_steps, max_steps + limit_steps); // fait dans la fonction limit_check
  //stepper.setTargetPositionInSteps(pos_in_step);
  Serial.printf("dmx pos to step => %ld\n", pos_in_step);
}

void set_min() // fonction pour position minimum de la plage de déplacement du moteur
{
  home_steps = map((analogRead(home_set_pot)), 4095, 0, min_offset, max_offset);
  stepper.setCurrentPositionAsHomeAndStop(); // set the current position as the home position and stop the stepper
  // min_steps = stepper.getCurrentPositionInSteps() - home_steps; // position min du moteur en pas
  min_steps = stepper.getCurrentPositionInSteps();
  encoder.setCount(0);            // remise à zéro de l'encodeur
  min_count = encoder.getCount(); // valeur minimum du compteur du codeur en position min
}

void set_max() // fonction pour position maximum de la plage de déplacement du moteur
{
  limit_steps = map((analogRead(limit_set_pot)), 4095, 0, min_offset, max_offset);
  // max_steps = stepper.getCurrentPositionInSteps() + limit_steps; // position max du moteur en pas
  max_steps = stepper.getCurrentPositionInSteps();
  max_count = encoder.getCount(); // valeur maximum du compteur du codeur en position max
}

void init_range() // fonction d'initialisation de la plage de déplacement du moteur
{
  int direction = -1;
  bool limit_OK = false;
  int position_init = max_steps / 2;
  Serial.println("starting homing");
  digitalWrite(MOTOR_ENABLE_PIN, LOW); // enable motor
  stepper.setDirectionToHome(direction);
  if (digitalRead(HOME_SWITCH_PIN) == switch_active) // si position est déjà à home
  {
    Serial.println("already in home position");
    stepper.setCurrentPositionAsHomeAndStop(); // set the current position as the home position and stop the stepper
    stepper.setTargetPositionInSteps(position_init);
    delay(2000);
  }
  stepper.goToLimitAndSetAsHome(); // go to the limit switch and set the current position as the home position

  while (!limit_OK)
  {
    home_sw_check();
    if (ConfirmedHomeSwitchState == switch_active)
    {
      stepper.setLimitSwitchActive(stepper.LIMIT_SWITCH_BEGIN); // this will cause to stop any motion that is currently going on and block further movement in the same direction as long as the switch is active
      stepper.setTargetPositionToStop();
      limit_OK = true;
    }
    else
    {
      stepper.clearLimitSwitchActive(); // clear the limit switch flag to allow movement in both directions again
    }
  }
  set_min();
  Serial.println("home OK");
  stepper.clearLimitSwitchActive(); // clear the limit switch flag to allow movement in both directions again
  limit_OK = false;
  Serial.println("waiting for limit switch");
  stepper.setTargetPositionInSteps(position_init); // set the target position to the middle of the range
  while (!limit_OK)
  {
    limit_sw_check();
    if (ConfirmedLimitSwitchState == switch_active)
    {
      stepper.setLimitSwitchActive(stepper.LIMIT_SWITCH_END); // this will cause to stop any motion that is currently going on and block further movement in the same direction as long as the switch is active
      stepper.setTargetPositionToStop();
      limit_OK = true;
    }
    else
    {
      stepper.clearLimitSwitchActive(); // clear the limit switch flag to allow movement in both directions again
      if (stepper.getCurrentPositionInSteps() < (max_steps - (max_steps / 4)))
      {
        position_init = stepper.getCurrentPositionInSteps() + (max_steps / 8);
      }
      else
      {
        position_init = stepper.getCurrentPositionInSteps() + nb_microstep;
      }
      stepper.setTargetPositionInSteps(position_init);
    }
  }
  Serial.println("limit init OK");
  stepper.setTargetPositionToStop();
  position_init = max_steps / 2;
  stepper.clearLimitSwitchActive(); // clear the limit switch flag to allow movement in both directions again
  set_max();
  stepper.setTargetPositionInSteps(position_init); // set the target position to the middle of the range
  delay(500);
  buttonStateChangeDetected = false; // remise à zéro du flag de changement d'état des switchs
}