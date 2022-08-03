/* Header to define Drive Calibration Constants
- Define Hard coded parameters for each joint
*/

#define DEG2RAD 0.01745329251994329576923690768489
#define RAD2DEG 57.295779513082320876798154814105

/*########################################################################
############## --- Joint Independent Standard Settings --- ###############
######################################################################## */


//Learning System 
#define DRVSYS_MAX_LEARNING_ITERATION_PER_STEP 10
#define DRVSYS_INV_DYN_ACTIVATION_ERROR_THRESHOLD 0.05

#define DRVSYS_INV_DYN_LEARNING_ACTIVE 0
#define DRVSYS_PID_GAIN_LEARNING_ACTIVE 0


// Task Stack Sizes
#define DRVSYS_STACKSIZE_FOC_CONTROL 2000
#define DRVSYS_STACKSIZE_PROCESS_ENCODER_TASK 5000
#define DRVSYS_STACKSIZE_TORQUE_CONTROL_TASK 3000
#define DRVSYS_STACKSIZE_PID_CONTROLLER_TASK 3000
#define DRVSYS_STACKSIZE_ADMITTANCE_CONTROLLER_TASK 2000
#define DRVSYS_STACKSIZE_PROCESS_TORQUE_SENSOR_TASK 1000
#define DRVSYS_STACKSIZE_LEARN_DYNAMICS_TASK 10000
#define DRVSYS_STACKSIZE_LEARN_PID_GAINS_TASK 10000
#define DRVSYS_STACKSIZE_STEPPER_CONTROL_TASK 3000

//Pin Tasks to Cores
#define DRVSYS_FOC_CORE 0
#define DRVSYS_ENCODER_CORE 0
#define DRVSYS_TORQUE_CONTROL_CORE 0
#define DRVSYS_PID_CORE 1
#define DRVSYS_ADMITTANCE_CORE 1
#define DRVSYS_PROCESS_TORQUE_CORE 1
#define DRVSYS_LEARNING_CORE 1
#define DRVSYS_STEPPER_CORE 0


//Calibration Routine
#define DRVSYS_CAL_LARGE_STEPCOUNT 10000
#define DRVSYS_CAL_SMALL_STEPCOUNT 50
#define DRVSYS_CAL_ANGLE_LIMIT 120.0

#define DRVSYS_CAL_REPEAT 1

#define DRVSYS_LIMITS_ENABLED 1

#define DRVSYS_SENSOR_MAX_DIFFERENCE_DEG 15


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


// --- Motor Parameters --- */
#define DRVSYS_PHASE_CURRENT_NOMINAL_mA 1500
#define DRVSYS_TRANSMISSION_RATIO 90.0
#define DRVSYS_TORQUE_CONSTANT 0.45 //max nominal torque
#define DRVSYS_TORQUE_LIMIT 0.6
#define DRVSYS_PHASE_CURRENT_MAX_mA 1800
// IMPORTANT ONLY CHANGE AFTER CALIBRATION
#define FOC_EMPIRIC_PHASE_ANGLE_OFFSET 7860

// --- Kinematic Constraints --- //
#define DRVSYS_POS_LIMIT_HIGH 180.0*DEG2RAD
#define DRVSYS_POS_LIMIT_LOW -180.0*DEG2RAD

#define DRVSYS_VEL_MAX 30.0*DEG2RAD // deg/s
#define DRVSYS_ACC_MAX 100.0*DEG2RAD//deg/s^


// Origin Location relative to Hall Sensor Locaion 
#define DRVSYS_ANGLE_ORIGIN_2_HALL_DEG 180.0
// Encoders Offsets (raw integer 14 bit 0 - 16383)
#define DRVSYS_RAW_JOINT_ENC_OFFSET 0
#define DRVSYS_RAW_MOTOR_ENC_OFFSET 0

// Alignment of Axis
#define DRVSYS_GLOB_DIR_FLIP 0
#define DRVSYS_TORQUE_ALIGN_DIR 1.0
#define DRVSYS_JOINT_ENC_ALIGN_DIR 1.0
#define DRVSYS_MOTOR_ENC_ALIGN_DIR 1.0

//Kalman Filter Noise Assumptions
#define DRVSYS_KIN_KALMAN_MOTOR_ACCELERATION_STD 10*DEG2RAD
#define DRVSYS_KIN_KALMAN_JOINT_ACCELERATION_STD 1*DEG2RAD


// Notch Filter Settings
#define DRVSYS_NOTCH_ACTIVE 1
#define DRVSYS_NOTCH_FREQU 320

#define DRVSYS_AXIS_ALIGNED_FLAG 1
#define DRVSYS_ENCODERS_CALIBRATED_FLAG 0

// PID Settings
#define DRVSYS_POS_PID_FILTER_DERIVATIVE 1
#define DRVSYS_POS_PID_FILTER_DERIVATIVE_ALPHA 0.05 // corresponds to ~ 500 Hz aka 2ms Time Constant alpha = (1-exp(-T/tau))
#define DRVSYS_POS_PID_DERIVATIVE_ON_MEASUREMENT 0
#define DRVSYS_POS_PID_DEADBAND 0

// PID Gains
#define PID_GAIN_P 0.75
#define PID_GAIN_I 10
#define PID_GAIN_D 0.05

// Constand Feedforward Gains
#define DRVSYS_VEL_FF_GAIN 0.01
#define DRVSYS_ACC_FF_GAIN 5.4e-6


#define DRVSYS_NN_CONTROL_BANDWIDTH 100


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
