
#define emergency_stop_bounce_time 1000 // time in ms to wait before disabling the motor in emergency when step as been lost
#define emergency_release_time 2000     // time in ms to wait before releasing the emergency stop
#define emergency_stop_loss_step 5      // number of step to wait before disabling the motor in emergency when step as been lost
unsigned long emercency_stop_timer;     // temps en ms depuis lequel le moteur est en arrêt d'urgence
unsigned long lastStepTime;             // variable pour stocker le temps de la dernière mise à jour des données encoder
long old_count;                         // variable des précedentes données de l'encoder
long newPosition;                       // variable des données de l'encoder

// fonction :
void motor_follower(int loss_step); // fonction pour suivre la position du moteur et la perte de pas
void emergency_check();             // fonction pour la sécurité si un obstacle est détecté par la perte de pas du moteur
void limit_check();                 // fonction pour la sécurité si une limite est détectée par le bouton poussoir

#ifndef dev_mode
/*
void IRAM_ATTR emergencySwitchHandler() // interrupt handler for the emergency stop switch
{

  // we do not realy need to debounce here, since we only want to trigger a stop, no matter what.
  // So even triggering mutliple times does not realy matter at the end
  if (digitalRead(EMERGENCY_STOP_PIN) == LOW) // Switch is configured in active low configuration
  {
    // the boolean true in the following command tells the stepper to hold the emergency stop until reaseEmergencyStop() is called explicitly.
    // If ommitted or "false" is given, the function call would only stop the current motion and then instanlty would allow for new motion commands to be accepted
    stepper.emergencyStop(true);
  }
  else
  {
    // release a previously enganed emergency stop when the emergency stop button is released
    stepper.releaseEmergencyStop();
  }

}
  */
#endif

void motor_follower(int loss_step) // fonction pour suivre la position du moteur et la perte de pas
{
  // Quand le moteur est en mouvement, on vérifie si la position a changé de plus de 5 pas
  // mise à jour de la position de l'encodeur
  newPosition = encoder.getCount();
  if (stepper.getDirectionOfMotion() != 0)
  {
    // Serial.printf("new position is %ld\n", newPosition);
    if (newPosition > old_count || newPosition < old_count)
    {
      lastStepTime = millis();
      emergencyStop = false;
    }
    if (newPosition >= old_count + loss_step || newPosition <= old_count - loss_step)
    {
      //  Serial.printf("new position => %ld\n", newPosition);
      //  Serial.printf("old position => %ld\n", old_count);
      old_count = newPosition;
    }
    // dans le else : newPosition == old_count &&
    else if (millis() - lastStepTime > emergency_stop_bounce_time / (DMX_data[speed_array] + 1) && emergencyStop == false)
    {
      Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!! emergency stop");
      stepper.setTargetPositionToStop();
      emercency_stop_timer = millis();
      emergencyStop = true;
    }
  }
}

void emergency_check() // fonction pour la sécurité si un obstacle est détecté par la perte de pas du moteur
{
  if (emergencyStop)
  {
    if (millis() - emercency_stop_timer > emergency_release_time && emergencyStop == true)
    {
      digitalWrite(MOTOR_ENABLE_PIN, LOW); // enable motor
      Serial.println("======================> emergency stop released");
      emergencyStop = false;
    }
    else if (digitalRead(MOTOR_ENABLE_PIN) == LOW && emergencyStop == true)
    {
      Serial.println("======================> emergency stop engaged");
      digitalWrite(MOTOR_ENABLE_PIN, HIGH); // disable and free motor
    }
  }
}

void limit_check() // fonction pour la sécurité si une limite est détectée par le bouton poussoir
{
  const int limit_step_lost = 5;
  limit_sw_check();
  home_sw_check();
  // active high switch configuration (NC connection with internal pull up)
  if (limitSwitchState == switch_active && buttonStateChangeDetected == true)
  {
    buttonStateChangeDetected = false;
    // limit_step = analogRead(limit_set_pot) / divider_pot;
    ConfirmedLimitSwitchState = limitSwitchState;
    Serial.printf("Limit switch change detected. New state is %i\n", limitSwitchState);
    Serial.printf("Limit setting is %i\n", limit_steps);
    // set_max();
    motor_follower(limit_step_lost);
    if (emergencyStop)
    {
      stepper.setLimitSwitchActive(stepper.LIMIT_SWITCH_END); // this will cause to stop any motion that is currently going on and block further movement in the same direction as long as the switch is agtive
    }
    else
    {
      stepper.setTargetPositionInSteps(max_steps + limit_steps);
      Serial.printf("go to Limit set : %i\n", max_steps + limit_steps);
    }
  }
  else if (buttonStateChangeDetected == true)
  {
    buttonStateChangeDetected = false;
  }
  else
  {
    stepper.clearLimitSwitchActive(); // clear the limit switch flag to allow movement in both directions again
  }

  if (homeSwitchState == switch_active && buttonStateChangeDetected == true)
  {
    buttonStateChangeDetected = false;
    // home_step = analogRead(home_set_pot) / divider_pot;
    ConfirmedHomeSwitchState = homeSwitchState;
    Serial.printf("Home switch change detected. New state is %i\n", homeSwitchState);
    Serial.printf("Home setting is %i\n", home_steps);
    stepper.setCurrentPositionAsHomeAndStop(); // set the current position as the home position and stop the stepper
    // set_min();
    motor_follower(limit_step_lost);
    if (emergencyStop)
    {
      stepper.setLimitSwitchActive(stepper.LIMIT_SWITCH_BEGIN); // this will cause to stop any motion that is currently going on and block further movement in the same direction as long as the switch is agtive
    }
    else
    {
      stepper.setTargetPositionInSteps(min_steps - home_steps);
      Serial.printf("go to Home set : %i\n", min_steps - home_steps);
    }
  }
  else if (buttonStateChangeDetected == true)
  {
    buttonStateChangeDetected = false;
  }
  else
  {
    stepper.clearLimitSwitchActive(); // clear the limit switch flag to allow movement in both directions again
  }
}
