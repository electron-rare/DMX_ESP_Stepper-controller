/// ********************** PIN ASSIGNEMENTS **********************
// DMX
const int tx_pin = 16; // define the IO pin the DMX TX pin is connected to
const int rx_pin = 17; // define the IO pin the DMX RX pin is connected to
const int rts_pin = 4; // define the IO pin the DMX RTS pin is connected to

// capteur BCD pour adressage DMX
#define q1 21 // define the IO pin for the thumbwheel '1' is connected to
#define q2 19 // define the IO pin for the thumbwheel '2' is connected to
#define q4 18 // define the IO pin for the thumbwheel '4' is connected to
#define q8 5 // define the IO pin for the thumbwheel '8' is connected to

// driver moteur
const int MOTOR_ENABLE_PIN = 27; // define the IO pin the motor enable pin is connected to
const int MOTOR_STEP_PIN = 26; // define the IO pin the motor step pin is connected to
const int MOTOR_DIRECTION_PIN = 25; // define the IO pin the motor direction pin is connected to
#ifndef dev_mode
const int EMERGENCY_STOP_PIN = 13; // define the IO pin the emergency stop switch is connected to
#endif
const int HOME_SWITCH_PIN = 35; // define the IO pin where the home switches are connected to (switches in series in normally closed setup against ground)
const int LIMIT_SWITCH_PIN = 34; // define the IO pin where the limit switches are connected to (switches in series in normally closed setup against ground)

//  encoder moteur pas à pas
#define EB_plus 22 // define the IO pin for the encoder B is connected to
#define EA_plus 23  // define the IO pin for the encoder A is connected to

// potentiomètre pour le réglage des fins de course
#define home_set_pot 36 // define the IO pin for the home set potentiometer is connected to
#define limit_set_pot 39 // define the IO pin for the limit set potentiometer is connected to
#define divider_pot 1 // define the divider potentiometer value


