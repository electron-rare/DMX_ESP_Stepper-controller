
// ********************** variables for software debouncing of the limit switches **********************
#define switch_active 1                  // active low switch configuration (NO connection with internal pull up) or HIGH for active high switch configuration (NC connection with internal pull up)

volatile unsigned long lastDebounceTime = 0;     // the last time the limit switch input pin was toggled
volatile unsigned long debounceDelay = 100;      // the minimum delay in milliseconds to check for bouncing of the switch. Increase this slighlty if you switches tend to bounce a lot
bool buttonStateChangeDetected = false;          // flag to indicate that the button state has changed
volatile byte limitSwitchState = !switch_active; // the current reading from the limit switch input pin
byte ConfirmedLimitSwitchState = !switch_active; // the last confirmed reading from the limit switch input pin
volatile byte homeSwitchState = !switch_active;  // the current reading from the home switch input pin
byte ConfirmedHomeSwitchState = !switch_active;  // the last confirmed reading from the home switch input pin

volatile int home_steps;  // nombre de pas pour bien positionner le moteur à la position de départ
volatile int limit_steps; // nombre de pas pour bien positionner le moteur à la position de fin

void homeReachedCallback();          // callback function that will be called when the home position is reached
void limitReachedCallback();         // callback function that will be called when the limit position is reached
void IRAM_ATTR limitSwitchHandler(); // interrupt handler for the limit switch
void IRAM_ATTR homeSwitchHandler();  // interrupt handler for the home switch
void limit_sw_check();               // fonction pour la sécurité si une limite est détectée par le bouton poussoir
void home_sw_check();                // fonction pour la sécurité si une limite est détectée par le bouton poussoir

void homeReachedCallback() // callback function that will be called when the home position is reached
{
  Serial.println("Home reached");
}

void limitReachedCallback() // callback function that will be called when the limit position is reached
{
  Serial.println("Limit reached");
}

void IRAM_ATTR limitSwitchHandler() // interrupt handler for the limit switch
{
  lastDebounceTime = millis();
  limitSwitchState = digitalRead(LIMIT_SWITCH_PIN);
}

void IRAM_ATTR homeSwitchHandler() // interrupt handler for the home switch
{
  lastDebounceTime = millis();
  homeSwitchState = digitalRead(HOME_SWITCH_PIN);
}

void limit_sw_check() // fonction pour la sécurité si une limite est détectée par le bouton poussoir
{

  if (limitSwitchState != ConfirmedLimitSwitchState && (millis() - lastDebounceTime) > debounceDelay)
  {
    ConfirmedLimitSwitchState = limitSwitchState;
    Serial.printf("Limit switch change detected. New state is %i\n", limitSwitchState);
    Serial.printf("Limit setting is %i\n", limit_steps);
    buttonStateChangeDetected = true;
  }
}

void home_sw_check() // fonction pour la sécurité si une limite est détectée par le bouton poussoir
{
  if (homeSwitchState != ConfirmedHomeSwitchState && (millis() - lastDebounceTime) > debounceDelay)
  {

    ConfirmedHomeSwitchState = homeSwitchState;
    Serial.printf("Home switch change detected. New state is %i\n", homeSwitchState);
    Serial.printf("Home setting is %i\n", home_steps);
    buttonStateChangeDetected = true;
  }
}
