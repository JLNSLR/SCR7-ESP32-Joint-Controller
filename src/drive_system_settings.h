/* Header to define Drive Calibration Constants
- Define Hard coded parameters for each joint
*/

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
#define DRVSYS_TORQUE_CONSTANT 0.45
#define DRVSYS_TORQUE_LIMIT 0.6
#define DRVSYS_PHASE_CURRENT_MAX_mA 1800

#define FOC_EMPIRIC_PHASE_ANGLE_OFFSET 7860

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


// Notch Filter Settings
#define DRVSYS_NOTCH_0_ACTIVE 1
#define DRVSYS_NOTCH_0_FREQU 300
#define DRVSYS_NOTCH_0_BW 20

#define DRVSYS_NOTCH_1_ACTIVE 0
#define DRVSYS_NOTCH_1_FREQU 300
#define DRVSYS_NOTCH_1_BW 20

#define DRVSYS_AXIS_ALIGNED_FLAG 0
#define DRVSYS_ENCODERS_CALIBRATED_FLAG 0


// Additional Info
//...
#endif


#ifdef DRVSYS_DRIVE_6 
/*#########################################################################
#################### --- Drive 6 Joint Configuration --- ######################
######################################################################## */

// --- Motor Parameters --- */
#define DRVSYS_PHASE_CURRENT_NOMINAL_mA 1500
#define DRVSYS_TRANSMISSION_RATIO 1
#define DRVSYS_TORQUE_CONSTANT 0.45
#define DRVSYS_TORQUE_LIMIT 0.6
#define DRVSYS_PHASE_CURRENT_MAX_mA 1800

#define FOC_EMPIRIC_PHASE_ANGLE_OFFSET 17359//15715 //

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

#define FOC_EMPIRIC_PHASE_ANGLE_OFFSET 2625//15715 //

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
#define DRVSYS_PHASE_CURRENT_NOMINAL_mA 1500
#define DRVSYS_TRANSMISSION_RATIO 1
#define DRVSYS_TORQUE_CONSTANT 0.45
#define DRVSYS_TORQUE_LIMIT 0.6
#define DRVSYS_PHASE_CURRENT_MAX_mA 1800

#define FOC_EMPIRIC_PHASE_ANGLE_OFFSET 20073//15715 //

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

#define FOC_EMPIRIC_PHASE_ANGLE_OFFSET 29046//15715 //

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
#define DRVSYS_PHASE_CURRENT_NOMINAL_mA 1500
#define DRVSYS_TRANSMISSION_RATIO 1
#define DRVSYS_TORQUE_CONSTANT 0.45
#define DRVSYS_TORQUE_LIMIT 0.6
#define DRVSYS_PHASE_CURRENT_MAX_mA 1800

#define FOC_EMPIRIC_PHASE_ANGLE_OFFSET 18534//15715 //

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
#define DRVSYS_PHASE_CURRENT_NOMINAL_mA 1500
#define DRVSYS_TRANSMISSION_RATIO 1
#define DRVSYS_TORQUE_CONSTANT 0.45
#define DRVSYS_TORQUE_LIMIT 0.6
#define DRVSYS_PHASE_CURRENT_MAX_mA 1800

#define FOC_EMPIRIC_PHASE_ANGLE_OFFSET 16386//15715 //

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
