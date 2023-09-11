
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

void motor_follower(int loss_step) // fonction pour suivre la position du moteur et la perte de pas
{
  // Quand le moteur est en mouvement, on vérifie si la position a changé de plus de 5 pas
  // mise à jour de la position de l'encodeur
  newPosition = encoder.getCount();
  if (stepper.getDirectionOfMotion() != 0)
  {
    if (newPosition > old_count || newPosition < old_count)
    {
      lastStepTime = millis();
      emergencyStop = false;
    }
    if (newPosition >= old_count + loss_step || newPosition <= old_count - loss_step)
    {
      old_count = newPosition;
    }
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
    Serial.printf("emergency stop timer : %i\n", millis() - emercency_stop_timer);
    if (millis() - emercency_stop_timer > emergency_release_time && emergencyStop == true)
    {
      digitalWrite(MOTOR_ENABLE_PIN, LOW); // enable motor
      Serial.println("======================> emergency stop released");
      lastStepTime = millis();
      emergencyStop = false;
      last_dmx_change_pos = millis();
      // stepper.setTargetPositionInSteps(pos_in_step);
    }
    else if (digitalRead(MOTOR_ENABLE_PIN) == LOW && emergencyStop == true)
    {
      Serial.println("======================> emergency stop engaged");
      digitalWrite(MOTOR_ENABLE_PIN, HIGH); // disable and free motor
    }
  }
  else if (digitalRead(MOTOR_ENABLE_PIN) == HIGH && emergencyStop == false)
  {
    digitalWrite(MOTOR_ENABLE_PIN, LOW); // enable motor
    Serial.println("======================> emergency HARD stop released");
    lastStepTime = millis();
    // last_dmx_change_pos = millis();
    // stepper.setTargetPositionInSteps(pos_in_step);
  }
}

void limit_check() // fonction pour la sécurité si une limite est détectée par le bouton poussoir
{
  const int limit_step_lost = nb_microstep; // nombre de pas à perdre avant de déclencher l'arrêt d'urgence en cas de détection de limite
  limit_sw_check();                         // check limit switch
  home_sw_check();                          // check home switch

  // si aucun bouton n'est appuyé
  if (ConfirmedHomeSwitchState != switch_active && ConfirmedLimitSwitchState != switch_active && buttonStateChangeDetected == true)
  {
    buttonStateChangeDetected = false;
    stepper.clearLimitSwitchActive(); // clear the limit switch flag to allow movement in both directions again
    return;
  }

  // si les deux boutons sont appuyés il y a un problème
  if (ConfirmedHomeSwitchState == switch_active && ConfirmedLimitSwitchState == switch_active)
  {
    Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! ERROOOOORRRRRREEE !!!!!!!!");
    stepper.emergencyStop(true);
    emergencyStop = true;
  }

  // si le bouton de début de course est appuyé
  if (ConfirmedHomeSwitchState == switch_active)
  {
    home_steps = map((analogRead(home_set_pot)), 4095, 0, min_offset, max_offset); // mise à jour de la valeur de l'offset de la position de départ
    // motor_follower(limit_step_lost);
    if (emergencyStop)
    {
      stepper.setLimitSwitchActive(stepper.LIMIT_SWITCH_BEGIN); // this will cause to stop any motion that is currently going on and block further movement in the same direction as long as the switch is agtive
      stepper.setCurrentPositionAsHomeAndStop();
    }
    // si le bouton vient d'être appuyé on va à la position de départ selon la config d'offset par le potentiomètre
    else if (buttonStateChangeDetected == true)
    {
      Serial.printf("===========> Home <===========\n");
      buttonStateChangeDetected = false;
      home_steps = map((analogRead(home_set_pot)), 4095, 0, min_offset, max_offset);
      // set_min(); // on enregistre la position actuelle comme position de départ
      min_count = encoder.getCount();                           // valeur minimum du compteur du codeur en position min
      stepper.setCurrentPositionAsHomeAndStop();                // set the current position as the home position and stop the stepper
      stepper.setCurrentPositionInSteps(min_steps);             // mise à jour de la position pour compenser un décalage éventuel
      stepper.setTargetPositionInSteps(min_steps - home_steps); // on va à la position de départ selon la config d'offset par le potentiomètre
      Serial.printf("go to Home set : %i\n", min_steps - home_steps);
      Serial.printf("min_steps : %i\n", min_steps);
      Serial.printf("add config step : %i\n", home_steps);
      Serial.printf("pos_in_step : %i\n", pos_in_step);
    }
  }

  // si le bouton de fin de course est appuyé
  if (ConfirmedLimitSwitchState == switch_active)
  {
    limit_steps = map((analogRead(limit_set_pot)), 4095, 0, min_offset, max_offset); // mise à jour de la valeur de l'offset de la position de fin
    // motor_follower(limit_step_lost);
    if (emergencyStop)
    {
      stepper.setLimitSwitchActive(stepper.LIMIT_SWITCH_END); // this will cause to stop any motion that is currently going on and block further movement in the same direction as long as the switch is agtive
    }
    // si le bouton vient d'être appuyé on va à la position de fin selon la config d'offset par le potentiomètre
    else if (buttonStateChangeDetected == true)
    {
      Serial.printf("===========> Limit <===========\n");
      buttonStateChangeDetected = false;
      limit_steps = map((analogRead(limit_set_pot)), 4095, 0, min_offset, max_offset);
      // set_max(); // on enregistre la position actuelle comme position de fin
      max_count = encoder.getCount();                            // valeur maximum du compteur du codeur en position max
      stepper.setCurrentPositionInSteps(max_steps);              // mise à jour de la position pour compenser un décalage éventuel
      stepper.setTargetPositionInSteps(max_steps);               // mise à jour de la position pour compenser un décalage éventuel
      stepper.setTargetPositionInSteps(max_steps + limit_steps); // on va à la position de fin selon la config d'offset par le potentiomètre
      Serial.printf("go to Limit set : %i\n", max_steps + limit_steps);
      Serial.printf("max_steps : %i\n", max_steps);
      Serial.printf("add config step : %i\n", limit_steps);
    }
  }
}
