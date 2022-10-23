/* Header to define Drive Calibration Constants
- Define Hard coded parameters for each joint
*/

#define DEG2RAD 0.01745329251994329576923690768489
#define RAD2DEG 57.295779513082320876798154814105

/* ##########################################################################
############## ----Constant Drive System Parameters----####################
############################################################################* /


/* --- Timing Constants --- */
#define DRVSYS_FOC_PERIOD_US 200 //us -> 5kHz
//Encoder Processing
#define DRVSYS_PROCESS_ENCODERS_PERIOD_US 500 //us -> 3kHz
#define DRVSYS_PROCESS_ENCODERS_FREQU 2000 //Hz
// Torque Target Control 
#define DRVSYS_CONTROL_TORQUE_PERIOD_US 400 // us 
#define DRVSYS_CONTROL_TORQUE_FREQU 2000 // Hz
// PID Controller
#define DRVSYS_CONTROL_POS_PERIOD_US 2000 //us 2000Hz
#define DRVSYS_CONTROL_POS_FREQ 500 //Hz

#define DRVSYS_CONTROL_VEL_PERIOD_US 500 //us 2000Hz
#define DRVSYS_CONTROL_VEL_FREQ 2000 //Hz
// Torque Sensor Processing
#define DRVSYS_PROCESS_TORQUE_SENSOR_PERIOD_MS 3

// Closed Loop Stepper Controller
#define DRVSYS_CONTROL_STEPPER_PERIOD_US 500
#define DRVSYS_CONTROL_STEPPER_FREQU 2000 //Hz

//Admittance Controller
#define DRVSYS_CONTROL_ADMITTANCE_PERIOD_MS 5

//Learning System Dynamics
#define DRVSYS_LEARNING_PERIOD_MS 2

//Monitoring System 
#define DRVSYS_MONITOR_PERIOD_MS 1000

/* --- Hardware-Timer-Constants --- */
#define DRVSYS_TIMER_PRESCALER_DIV 80 // with 80MHz Clock, makes the timer tick every 1us
#define DRVSYS_TIMER_ALARM_RATE_US 50 //generate timer alarm every 50us

/* Allow Calibrations */

#define ALLOW_ENCODER_CALIBRATION_ROUTINE

#define ALLOW_AXIS_ALIGN_CALIBRATION

//#define ALLOW_ELECTRIC_ANGLE_CALIBRATION // DANGEROUS IN ASSEMBLED ROBOT AXIS
// ----- DANGEROUS !!! ----- 
// USE ONLY WITH MOTOR THAT IS NOT ATTACHED TO ROBOT GEAR SYSTEM
// MOTOR CALIBRATION COULD DAMAGE ROBOT
// INTENDED TO USE TO OBTAIN MOTOR SPECIFIC ELECTRIC ANGLE OFFSET FOR FOC CONTROL
// is required, since foc control of 50 Pole Stepper motor and noisy 14 bit encoder with delay 
// is pushing the limits. Calibrating via Coil energizing does not deliver accurat enough results
// and slight offset diminishes control so severely that the use of FOC control is rendered useless
// since the magnetic field will be off that much, that for higher velocities makes the motor cog,
// inefficient 


/*########################################################################
############## --- Joint Independent Standard Settings --- ###############
######################################################################## */


//Learning System 
#define DRVSYS_PIDNN_LEARNING_REGULARIZATION 1e-2 // (the higher this value, the more conservative (smaller Gains) the adaptive Tuning will be)


// Task Stack Sizes
#define DRVSYS_STACKSIZE_FOC_CONTROL 2000
#define DRVSYS_STACKSIZE_PROCESS_ENCODER_TASK 5000
#define DRVSYS_STACKSIZE_TORQUE_CONTROL_TASK 3000
#define DRVSYS_STACKSIZE_PID_CONTROLLER_TASK 3000
#define DRVSYS_STACKSIZE_ADMITTANCE_CONTROLLER_TASK 2000
#define DRVSYS_STACKSIZE_PROCESS_TORQUE_SENSOR_TASK 1000
#define DRVSYS_STACKSIZE_LEARN_DYNAMICS_TASK 20000
#define DRVSYS_STACKSIZE_LEARN_PID_GAINS_TASK 10000
#define DRVSYS_STACKSIZE_STEPPER_CONTROL_TASK 3000
#define DRVSYS_STACKSIZE_TORQUE_SENSE_TASK 2000
#define DRVSYS_STACKSIZE_MONITOR_TASK 2000

//Pin Tasks to Cores
#define DRVSYS_FOC_CORE 0
#define DRVSYS_ENCODER_CORE 0
#define DRVSYS_TORQUE_CONTROL_CORE 0
#define DRVSYS_PID_CORE 0
#define DRVSYS_ADMITTANCE_CORE 1
#define DRVSYS_PROCESS_TORQUE_CORE 1
#define DRVSYS_LEARNING_CORE 1
#define DRVSYS_STEPPER_CORE 0
#define DRVSYS_TORQUE_SENSE_CORE 1
#define DRVSYS_MONITOR_CORE 1


//Calibration Routine
#define DRVSYS_CAL_LARGE_STEPCOUNT 10000
#define DRVSYS_CAL_SMALL_STEPCOUNT 50
#define DRVSYS_CAL_ANGLE_LIMIT 120.0

#define DRVSYS_CAL_REPEAT 1

#define DRVSYS_LIMITS_ENABLED 1

#define DRVSYS_SENSOR_MAX_DIFFERENCE_DEG 45

#define TEMP_SENSOR_AVAILABLE 1
#define FAN_AVAILABLE 1
#define FAN_12V 0
#define FAN_24V 1
#define N_LEDS 1


#define TORQUE_SENSOR_0
#define TORQUE_SENSE_INT_OFFSET -729016.02
#define TORQUE_SENSE_SLOPE -5.87523e-6




/*#########################################################################
####################### --- Select Drive Configuration --- ###########################
######################################################################## */
#define DRVSYS_DRIVE_TESTBUILD
//#define DRVSYS_DRIVE_6
//#define DRVSYS_DRIVE_5
//#define DRVSYS_DRIVE_4
//#define DRVSYS_DRIVE_3
//#define DRVSYS_DRIVE_2
//#define DRVSYS_DRIVE_1
//#define DRVSYS_DRIVE_0

/* ############################################# */

#ifdef DRVSYS_DRIVE_TESTBUILD 
/*#########################################################################
#################### --- Testbuild Configuration --- ######################
######################################################################## */

#define JOINT_ID 5


// --- Motor Parameters --- */
#define DRVSYS_PHASE_CURRENT_NOMINAL_mA 1500
#define DRVSYS_TRANSMISSION_RATIO 90.0
#define DRVSYS_TORQUE_CONSTANT 0.45 //max nominal torque
#define DRVSYS_TORQUE_LIMIT 0.54
#define DRVSYS_PHASE_CURRENT_MAX_mA 1800
// IMPORTANT ONLY CHANGE AFTER CALIBRATION
#define FOC_EMPIRIC_PHASE_ANGLE_OFFSET 7860

// --- Kinematic Constraints --- //
#define DRVSYS_POS_LIMIT_HIGH 125.0*DEG2RAD
#define DRVSYS_POS_LIMIT_LOW -125.0*DEG2RAD

#define DRVSYS_VEL_MAX 90.0*DEG2RAD // deg/s
#define DRVSYS_ACC_MAX 100.0*DEG2RAD//deg/s^


// Origin Location relative to Hall Sensor Locaion 
#define DRVSYS_ANGLE_ORIGIN_2_HALL_DEG 180.0
// Encoders Offsets (raw integer 14 bit 0 - 16383)
#define DRVSYS_RAW_JOINT_ENC_OFFSET 0
#define DRVSYS_RAW_MOTOR_ENC_OFFSET 0

#define DRVSYS_TORQUE_SENSOR_TYPE dms

// Alignment of Axis
#define DRVSYS_GLOB_DIR_FLIP 0
#define DRVSYS_TORQUE_ALIGN_DIR 1.0
#define DRVSYS_JOINT_ENC_ALIGN_DIR 1.0
#define DRVSYS_MOTOR_ENC_ALIGN_DIR 1.0

//Kalman Filter Noise Assumptions
#define DRVSYS_KIN_KALMAN_MOTOR_ACCELERATION_STD 100*DEG2RAD
#define DRVSYS_KIN_KALMAN_JOINT_ACCELERATION_STD 10*DEG2RAD


// Notch Filter Settings
#define DRVSYS_NOTCH_ACTIVE 1
#define DRVSYS_NOTCH_FREQU 320

#define DRVSYS_AXIS_ALIGNED_FLAG 1
#define DRVSYS_ENCODERS_CALIBRATED_FLAG 0

// PID Settings
#define DRVSYS_POS_PID_FILTER_DERIVATIVE 1
#define DRVSYS_POS_PID_FILTER_DERIVATIVE_ALPHA 0.0001 // corresponds to ~ 500 Hz aka 2ms Time Constant alpha = (1-exp(-T/tau))
#define DRVSYS_POS_PID_DERIVATIVE_ON_MEASUREMENT 0
#define DRVSYS_POS_PID_DEADBAND 0.025*DEG2RAD
#define DRVSYS_VEL_PID_FILTER_DERIVATIVE 1
#define DRVSYS_VEL_PID_FILTER_DERIVATIVE_ALPHA 0.001 // corresponds to ~ 1 Hz aka 2ms Time Constant alpha = (1-exp(-T/tau))
#define DRVSYS_VEL_PID_DERIVATIVE_ON_MEASUREMENT 0
#define DRVSYS_VEL_PID_DEADBAND 0
#define DRVSYS_VEL_PID_INPUT_FILTER_ALPHA 0.001 // 200Hz/90 ~ 1Hz

// PID Gains
#define PID_POS_GAIN_P 1.00
#define PID_POS_GAIN_I 0.1
#define PID_POS_GAIN_D 0.75

#define PID_VEL_GAIN_P 0.5
#define PID_VEL_GAIN_I 75
#define PID_VEL_GAIN_D 0

#define STEPPER_POS_GAIN_P 0.5
#define STEPPER_POS_GAIN_I 0.0
#define STEPPER_POS_GAIN_D 0.0


// Constand Feedforward Gains
#define DRVSYS_VEL_FF_GAIN 1*0.15
#define DRVSYS_ACC_FF_GAIN 1*5.4e-6

#define DRVSYS_NN_CONTROL_AUTO_ACTIVE 0


#define DRVSYS_NN_CONTROL_BANDWIDTH 30




// Stepper Settings
#define DRVSYS_MICROSTEPS 8
#define FLIP_DIR_VEL 1
#define FLIP_DIR_POS 0



// Additional Info
//...
#endif


#ifdef DRVSYS_DRIVE_6 
/*#########################################################################
#################### --- Drive 6 Joint Configuration --- ######################
######################################################################## */

#define JOINT_ID 6

// --- Motor Parameters --- */
#define DRVSYS_PHASE_CURRENT_NOMINAL_mA 400
#define DRVSYS_TRANSMISSION_RATIO 1
#define DRVSYS_TORQUE_CONSTANT 0.26
#define DRVSYS_TORQUE_LIMIT 0.3
#define DRVSYS_PHASE_CURRENT_MAX_mA 500

#define FOC_EMPIRIC_PHASE_ANGLE_OFFSET 14680//15715 //

// --- Kinematic Constraints --- //
#define DRVSYS_POS_LIMIT_HIGH 180.0
#define DRVSYS_POS_LIMIT_LOW -180.0

#define DRVSYS_VEL_MAX 300.0 // deg/s

// Origin Location relative to Hall Sensor Locaion 
#define DRVSYS_ANGLE_ORIGIN_2_HALL_DEG 0
// Encoders Offsets (raw integer 14 bit 0 - 16383)
#define DRVSYS_RAW_JOINT_ENC_OFFSET 0
#define DRVSYS_RAW_MOTOR_ENC_OFFSET 0

// Alignment of Axis
#define DRVSYS_GLOB_DIR_FLIP 0
#define DRVSYS_TORQUE_ALIGN_DIR 1.0
#define DRVSYS_JOINT_ENC_ALIGN_DIR 1.0
#define DRVSYS_MOTOR_ENC_ALIGN_DIR 1.0


// Additional Info
//...
#endif


#ifdef DRVSYS_DRIVE_5 
/*#########################################################################
#################### --- Drive 5 Joint Configuration --- ######################
######################################################################## */

#define JOINT_ID 4

// --- Motor Parameters --- */
#define DRVSYS_PHASE_CURRENT_NOMINAL_mA 1500
#define DRVSYS_TRANSMISSION_RATIO 1
#define DRVSYS_TORQUE_CONSTANT 0.45
#define DRVSYS_TORQUE_LIMIT 0.6
#define DRVSYS_PHASE_CURRENT_MAX_mA 1800

#define FOC_EMPIRIC_PHASE_ANGLE_OFFSET 7703 //2625//15715 //

// --- Kinematic Constraints --- //
#define DRVSYS_POS_LIMIT_HIGH 180.0
#define DRVSYS_POS_LIMIT_LOW -180.0

#define DRVSYS_VEL_MAX 300.0 // deg/s

// Origin Location relative to Hall Sensor Locaion 
#define DRVSYS_ANGLE_ORIGIN_2_HALL_DEG 0
// Encoders Offsets (raw integer 14 bit 0 - 16383)
#define DRVSYS_RAW_JOINT_ENC_OFFSET 0
#define DRVSYS_RAW_MOTOR_ENC_OFFSET 0

// Alignment of Axis
#define DRVSYS_GLOB_DIR_FLIP 0
#define DRVSYS_TORQUE_ALIGN_DIR 1.0
#define DRVSYS_JOINT_ENC_ALIGN_DIR 1.0
#define DRVSYS_MOTOR_ENC_ALIGN_DIR 1.0


// Additional Info
//...
#endif

#ifdef DRVSYS_DRIVE_4 
/*#########################################################################
#################### --- Drive 4 Joint Configuration --- ######################
######################################################################## */

#define JOINT_ID 4

// --- Motor Parameters --- */
#define DRVSYS_PHASE_CURRENT_NOMINAL_mA 1680
#define DRVSYS_TRANSMISSION_RATIO 1
#define DRVSYS_TORQUE_CONSTANT 0.52
#define DRVSYS_TORQUE_LIMIT 0.6
#define DRVSYS_PHASE_CURRENT_MAX_mA 1800

#define FOC_EMPIRIC_PHASE_ANGLE_OFFSET 12187// 20073//15715 //

// --- Kinematic Constraints --- //
#define DRVSYS_POS_LIMIT_HIGH 180.0
#define DRVSYS_POS_LIMIT_LOW -180.0

#define DRVSYS_VEL_MAX 300.0 // deg/s

// Origin Location relative to Hall Sensor Locaion 
#define DRVSYS_ANGLE_ORIGIN_2_HALL_DEG 0
// Encoders Offsets (raw integer 14 bit 0 - 16383)
#define DRVSYS_RAW_JOINT_ENC_OFFSET 0
#define DRVSYS_RAW_MOTOR_ENC_OFFSET 0

// Alignment of Axis
#define DRVSYS_GLOB_DIR_FLIP 0
#define DRVSYS_TORQUE_ALIGN_DIR 1.0
#define DRVSYS_JOINT_ENC_ALIGN_DIR 1.0
#define DRVSYS_MOTOR_ENC_ALIGN_DIR 1.0


// Additional Info
//...
#endif

#ifdef DRVSYS_DRIVE_3 
/*#########################################################################
#################### --- Drive 3 Joint Configuration --- ######################
######################################################################## */

#define JOINT_ID 3

// --- Motor Parameters --- */
#define DRVSYS_PHASE_CURRENT_NOMINAL_mA 1500
#define DRVSYS_TRANSMISSION_RATIO 1
#define DRVSYS_TORQUE_CONSTANT 0.45
#define DRVSYS_TORQUE_LIMIT 0.6
#define DRVSYS_PHASE_CURRENT_MAX_mA 1800

#define FOC_EMPIRIC_PHASE_ANGLE_OFFSET 6744 //29046//15715 //

// --- Kinematic Constraints --- //
#define DRVSYS_POS_LIMIT_HIGH 180.0
#define DRVSYS_POS_LIMIT_LOW -180.0

#define DRVSYS_VEL_MAX 300.0 // deg/s

// Origin Location relative to Hall Sensor Locaion 
#define DRVSYS_ANGLE_ORIGIN_2_HALL_DEG 0
// Encoders Offsets (raw integer 14 bit 0 - 16383)
#define DRVSYS_RAW_JOINT_ENC_OFFSET 0
#define DRVSYS_RAW_MOTOR_ENC_OFFSET 0

// Alignment of Axis
#define DRVSYS_GLOB_DIR_FLIP 0
#define DRVSYS_TORQUE_ALIGN_DIR 1.0
#define DRVSYS_JOINT_ENC_ALIGN_DIR 1.0
#define DRVSYS_MOTOR_ENC_ALIGN_DIR 1.0


// Additional Info
//...
#endif

#ifdef DRVSYS_DRIVE_2 
/*#########################################################################
#################### --- Drive 2 Joint Configuration --- ######################
######################################################################## */

#define JOINT_ID 2

// --- Motor Parameters --- */
#define DRVSYS_PHASE_CURRENT_NOMINAL_mA 2100
#define DRVSYS_TRANSMISSION_RATIO 1
#define DRVSYS_TORQUE_CONSTANT 0.65
#define DRVSYS_TORQUE_LIMIT 0.8
#define DRVSYS_PHASE_CURRENT_MAX_mA 2500

#define FOC_EMPIRIC_PHASE_ANGLE_OFFSET 16256 //18534//15715 //

// --- Kinematic Constraints --- //
#define DRVSYS_POS_LIMIT_HIGH 180.0
#define DRVSYS_POS_LIMIT_LOW -180.0

#define DRVSYS_VEL_MAX 300.0 // deg/s

// Origin Location relative to Hall Sensor Locaion 
#define DRVSYS_ANGLE_ORIGIN_2_HALL_DEG 0
// Encoders Offsets (raw integer 14 bit 0 - 16383)
#define DRVSYS_RAW_JOINT_ENC_OFFSET 0
#define DRVSYS_RAW_MOTOR_ENC_OFFSET 0

// Alignment of Axis
#define DRVSYS_GLOB_DIR_FLIP 0
#define DRVSYS_TORQUE_ALIGN_DIR 1.0
#define DRVSYS_JOINT_ENC_ALIGN_DIR 1.0
#define DRVSYS_MOTOR_ENC_ALIGN_DIR 1.0


// Additional Info
//...
#endif

#ifdef DRVSYS_DRIVE_1 
/*#########################################################################
#################### --- Drive 1 Joint Configuration --- ######################
######################################################################## */

#define JOINT_ID 1

// --- Motor Parameters --- */
#define DRVSYS_PHASE_CURRENT_NOMINAL_mA 2800
#define DRVSYS_TRANSMISSION_RATIO 1
#define DRVSYS_TORQUE_CONSTANT 1.92
#define DRVSYS_TORQUE_LIMIT 2.12
#define DRVSYS_PHASE_CURRENT_MAX_mA 3000

#define FOC_EMPIRIC_PHASE_ANGLE_OFFSET 492// 16386//15715 //
// --- Kinematic Constraints --- //
#define DRVSYS_POS_LIMIT_HIGH 180.0
#define DRVSYS_POS_LIMIT_LOW -180.0

#define DRVSYS_VEL_MAX 300.0 // deg/s

// Origin Location relative to Hall Sensor Locaion 
#define DRVSYS_ANGLE_ORIGIN_2_HALL_DEG 0
// Encoders Offsets (raw integer 14 bit 0 - 16383)
#define DRVSYS_RAW_JOINT_ENC_OFFSET 0
#define DRVSYS_RAW_MOTOR_ENC_OFFSET 0

// Alignment of Axis
#define DRVSYS_GLOB_DIR_FLIP 0
#define DRVSYS_TORQUE_ALIGN_DIR 1.0
#define DRVSYS_JOINT_ENC_ALIGN_DIR 1.0
#define DRVSYS_MOTOR_ENC_ALIGN_DIR 1.0


// Additional Info
//...
#endif

#ifdef DRVSYS_DRIVE_0
/*#########################################################################
#################### --- Drive 0 Joint Configuration --- ######################
######################################################################## */

#define JOINT_ID 0

// --- Motor Parameters --- */
#define DRVSYS_PHASE_CURRENT_NOMINAL_mA 1500
#define DRVSYS_TRANSMISSION_RATIO 1
#define DRVSYS_TORQUE_CONSTANT 0.45
#define DRVSYS_TORQUE_LIMIT 0.6
#define DRVSYS_PHASE_CURRENT_MAX_mA 1800

#define FOC_EMPIRIC_PHASE_ANGLE_OFFSET 0//15715 //

// --- Kinematic Constraints --- //
#define DRVSYS_POS_LIMIT_HIGH 180.0
#define DRVSYS_POS_LIMIT_LOW -180.0

#define DRVSYS_VEL_MAX 300.0 // deg/s

// Origin Location relative to Hall Sensor Locaion 
#define DRVSYS_ANGLE_ORIGIN_2_HALL_DEG 0
// Encoders Offsets (raw integer 14 bit 0 - 16383)
#define DRVSYS_RAW_JOINT_ENC_OFFSET 0
#define DRVSYS_RAW_MOTOR_ENC_OFFSET 0

// Alignment of Axis
#define DRVSYS_GLOB_DIR_FLIP 0
#define DRVSYS_TORQUE_ALIGN_DIR 1.0
#define DRVSYS_JOINT_ENC_ALIGN_DIR 1.0
#define DRVSYS_MOTOR_ENC_ALIGN_DIR 1.0


// Additional Info
//...
#endif
