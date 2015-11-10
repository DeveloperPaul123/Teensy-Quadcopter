/**
* Miscellaneous defines
*/
//define to print serial data. 
#define DEBUG

//define to print MPU data
// #define DEBUG_MPU

//define bluetooth.
#define BLUETOOTH Serial1

//define to debug controls only.
// #define DEBUG_CONTROLS

//define to test from serial monitor, won't write PWM values to motors. 
// #define DISABLE_MOTORS

//define to see the biases. 
#define FIRST_RUN

//define if using a radio
//#define HAS_RC

//define to view debug data from RC
//#define DEBUG_RC

//define if should read bluetooth
#define BLUETOOTH_CONTROL

//define if you want to compute a bias curve vs temperature. Use this if you want to 
//compute a line to use for correcting bias on sensors due to temperature. 
//#define COMPUTE_BIAS_CURVE