#define STEP_PER_REV 3200 // nombre de pas par tour du moteur

const int MIN_SPEED[3] = {0, 0, 0};               // vitesse, acceleration et deceleration minimum du moteur
const int MAX_SPEED[3] = {1000, 5000, 5000};      // vitesse, acceleration et deceleration maximum du moteur
const int SPEED_IN_STEPS_PER_SECOND = 500;        // vitesse de déplacement du moteur
const int ACCELERATION_IN_STEPS_PER_SECOND = 500; // accélération du moteur
const int DECELERATION_IN_STEPS_PER_SECOND = 100; // décélération du moteur
int min_steps = 0;                                // position de départ du moteur
int max_steps = 1600;                             // position de fin du moteur

volatile int speed_in_accel = 100;
volatile int speed_in_decel = 100;

// fonction
void targetPositionReachedCallback(long position); // callback function as the target position is reached
// void limitSwitchHandler();                         // interrupt handler for the limit switch
// void homeSwitchHandler();                          // interrupt handler for the home switch
void speed_map();  // fonction de mapping de la vitesse du moteur avec accélération et décélération
void pos_map();    // fonction de mapping de la position du moteur
void set_min();    // fonction pour position minimum de la plage de déplacement du moteur
void set_max();    // fonction pour position maximum de la plage de déplacement du moteurF
void init_range(); // fonction d'initialisation de la plage de déplacement du moteur

void targetPositionReachedCallback(long position) // callback function as the target position is reached
{
  Serial.printf("Stepper reached target position %ld\n", position);
  Serial.printf("Encoder count is %ld\n", encoder.getCount());
}

// fonction de mapping de la vitesse du moteur avec accélération et décélération
void speed_map() // fonction de mapping de la vitesse du moteur avec accélération et décélération
{
  int speed_in_step = map(DMX_data[speed_array], 0, 255, MIN_SPEED[0], MAX_SPEED[0]);
  // int speed_in_accel = map(DMX_data[speed_array], 0, 255, MIN_SPEED[1], MAX_SPEED[1]);
  // int speed_in_decel = map(DMX_data[speed_array], 0, 255, MIN_SPEED[2], MAX_SPEED[2]);
  stepper.setSpeedInStepsPerSecond(speed_in_step);
  // set the speed and acceleration rates for the stepper motor
  stepper.setAccelerationInStepsPerSecondPerSecond(speed_in_accel);
  stepper.setDecelerationInStepsPerSecondPerSecond(speed_in_decel);
}

void pos_map() // fonction de mapping de la position du moteur
{
  int pos_in_step = map(DMX_data[pos_array], 0, 255, min_steps, max_steps);
  stepper.setTargetPositionInSteps(pos_in_step);
}

void set_min() // fonction pour position minimum de la plage de déplacement du moteur
{
  min_steps = stepper.getCurrentPositionInSteps() - home_steps;
  encoder.setCount(0); // remise à zéro de l'encodeur
}

void set_max() // fonction pour position maximum de la plage de déplacement du moteur
{
  max_steps = stepper.getCurrentPositionInSteps() + limit_steps;
}

void init_range() // fonction d'initialisation de la plage de déplacement du moteur
{
#ifndef dev_mode
  int direction = -1;
  bool limit_OK = false;
  int position_limit = 500;
  Serial.println("starting homing");
  // delay(2000);
  digitalWrite(MOTOR_ENABLE_PIN, LOW); // enable motor
  stepper.setDirectionToHome(direction);
  if (digitalRead(HOME_SWITCH_PIN) == switch_active) // si position est déjà à home
  {
    Serial.println("already in home position");
    stepper.setCurrentPositionAsHomeAndStop(); // set the current position as the home position and stop the stepper
    stepper.setTargetPositionInSteps(position_limit);
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

  stepper.setCurrentPositionAsHomeAndStop(); // set the current position as the home position and stop the stepper
  set_min();
  encoder.setCount(0); // remise à zéro de l'encodeur
  Serial.println("home OK");
  stepper.clearLimitSwitchActive(); // clear the limit switch flag to allow movement in both directions again
  limit_OK = false;
  position_limit = 0;
  Serial.println("waiting for limit switch");
  stepper.setTargetPositionInSteps(160);
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
      if (stepper.getCurrentPositionInSteps() < 1350)
      {
        position_limit = stepper.getCurrentPositionInSteps() + 200;
      }
      else
      {
        position_limit = stepper.getCurrentPositionInSteps() + 5;
      }
      stepper.setTargetPositionInSteps(position_limit);
    }
  }
  Serial.println("limit init OK");
  stepper.setTargetPositionToStop();
  set_max();
  stepper.clearLimitSwitchActive(); // clear the limit switch flag to allow movement in both directions again
  stepper.setTargetPositionInSteps(max_steps / 2);
  delay(2000);
#endif
}