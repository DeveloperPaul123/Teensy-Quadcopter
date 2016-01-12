
/**
* PWM resolution for driving brushed motors.
**/
#define PWM_RESOLUTION 16

/**
* Define max value as 2^PWM_RESOLUTION - 1
**/
#define PWM_MAX ((1<<PWM_RESOLUTION) - 1)

/*
* PID stuff.
*/
#define PITCH_MIN -35
#define PITCH_MAX 35
#define ROLL_MIN -35
#define ROLL_MAX 35
#define YAW_MIN -150
#define YAW_MAX 150

//max rotational rates in degrees per second. 
#define RATE_PITCH_MIN -150
#define RATE_PITCH_MAX 150
#define RATE_ROLL_MIN -150
#define RATE_ROLL_MAX 150

#define PID_INFLUENCE (PWM_MAX / 5)

/**
* These two values should be different because they never directly interface with the motors. 
* so outputs need to match the user inputs when flying acro mode.
**/ 
#define PID_PITCH_INFLUENCE RATE_PITCH_MAX
#define PID_ROLL_INFLUENCE RATE_ROLL_MAX

/**
* Always directly controls the motors so larger value. 
**/
#define PID_YAW_INFLUENCE PID_INFLUENCE

#define RATE_PITCH_PID_INFLUENCE PID_INFLUENCE
#define RATE_ROLL_PID_INFLUENCE PID_INFLUENCE

#define MAX_THROTTLE (PWM_MAX - PID_INFLUENCE)
#define MIN_THROTTLE (PID_INFLUENCE)


// Modulo definitions (integer remainder) processing events
// at a fixed rate.
#define TASK_50HZ 2
#define TASK_10HZ 10
#define TASK_1HZ 100

/**
* RC Stuff
**/
#define RC_HIGH_CH1 1000
#define RC_LOW_CH1 2000
#define RC_HIGH_CH2 1000
#define RC_LOW_CH2 2000
#define RC_HIGH_CH3 1000
#define RC_LOW_CH3 2000
#define RC_HIGH_CH4 1000
#define RC_LOW_CH4 2000
#define RC_HIGH_CH5 1000
#define RC_LOW_CH5 2000
#define RC_HIGH_CH6 1000
#define RC_LOW_CH6 2000
#define RC_ERROR_BAND 100

