#ifndef DRIVE_SYSTEM_H
#define DRIVE_SYSTEM_H

#include <Arduino.h>
#include <FreeRTOS.h>
#include <joint_control_global_def.h>
#include <foc_controller_tmc2160.h>
#include <CircularBuffer.h>
#include <AS5048A.h>
#include <TMCStepper.h>
#include <signal_processing/differentiator.h>
#include <signal_processing/IIRFilter.h>
#include <PID/PIDController.h>
#include <drive_system_settings.h>
#include <kinematic_kalman_filter.h>

//#include <drive_dynamic_kalman_filter.h>

#include <Preferences.h>

#define DEG2RAD 0.01745329251994329576923690768489
#define RAD2DEG 57.295779513082320876798154814105

/* ##########################################################################
############## ---- Constant Drive System Parameters ----####################
############################################################################*/


/* --- Timing Constants --- */

#define DRVSYS_FOC_PERIOD_US 200 //us -> 5kHz
#define DRVSYS_PROCESS_ENCODERS_PERIOD_US 250 //us -> 3kHz
#define DRVSYS_PROCESS_ENCODERS_FREQU 4000 //Hz
#define DRVSYS_CONTROL_TORQUE_PERIOD_US 333 // us 
#define DRVSYS_CONTROL_TORQUE_FREQU 3000 // Hz
#define DRVSYS_CONTROL_VEL_PERIOD_US 333 //us //600Hz
#define DRVSYS_CONTROL_VEL_FREQ 3000    //Hz
#define DRVSYS_CONTROL_POS_PERIOD_US 3333 //ms //1000Hz
#define DRVSYS_CONTROL_POS_FREQ 3000

#define DRVSYS_CONTROL_ADMITTANCE_PERIOD_MS 10
#define DRVSYS_PROCESS_TORQUE_SENSOR_PERIOD_MS 333

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



/* DEBUG COMMAND */
#define DRV_SYS_DEBUG

/*########################################################################
################ Drive System Data Types & Enums #########################
#########################################################################*/

/* Drive System State Flag */
enum drvSys_StateFlag { error, not_ready, ready, foc_direct_torque, closed_loop_control_active, closed_loop_control_inactive };
/*Drive System Control Mode Variables */
enum drvSys_controlMode { direct_torque, dual_control, admittance_control, impedance_control, hybrid_control };

extern drvSys_controlMode drvSys_mode;
extern drvSys_StateFlag drvSys_state_flag;

struct drvSys_PID_Gains {
    float K_p;
    float K_i;
    float K_d;

};

struct drvSys_admittance_parameters {
    float virtual_spring;
    float virtual_damping;
    float virtual_inertia;
};


/*#########################################################################
################# Drive System Data Structures ############################
##########################################################################*/

struct drvSys_driveState {
    float joint_pos;
    float joint_vel;
    float joint_acc;
    float joint_torque;
    float motor_torque;
};

struct drvSys_extendedDriveState {
    float joint_pos;
    float joint_vel;
    float joint_acc;
    float joint_torque;
    float motor_torque;
    float motor_pos;
    float motor_vel;
    float motor_acc;
};

struct drvSys_driveTargets {
    float pos_target;
    float vel_target;
    float motor_torque_target;
    float ref_torque;
};


struct drvSys_parameters {
    int max_current_mA;
    float max_torque_Nm;
    float max_vel;
    drvSys_PID_Gains vel_pid_gains;
    drvSys_PID_Gains pos_pid_gains;
    float joint_pos_P_gain;
    float vel_ff_gain;
    drvSys_admittance_parameters admittance_gains;
    float limit_high_deg;
    float limit_low_deg;
};

struct drvSys_notch_filter_params {
    bool notch_0_active;
    bool notch_1_active;
    float notch_frequ_0;
    float notch_bw_0;
    float notch_frequ_1;
    float notch_bw_1;
};

extern drvSys_parameters drvSys_parameter_config;

struct drvSys_Constants {
    const int nominal_current_mA;
    const float transmission_ratio;
    const int joint_id;
    const float motor_torque_constant;
};

struct drvSys_controllerState {
    enum drvSys_controlMode control_mode;
    enum drvSys_StateFlag state_flag;
    bool calibrated;
    bool overtemperature;
    bool temperature_warning;
    int temperature;
};


/* ##########################################################################
################### ---- Interface Functions ---- ###########################
############################################################################*/

/**
 * @brief Initializes the components of the Drive System
 *
 */
void drvSys_initialize();
/**
 * @brief starts motor encoding processing. Reads position sensors, calculates velocity and acceleration and starts FOC controller.
 *
 */
int32_t drvSys_start_foc_processing();
/**
 * @brief starts the motion controllers depending on the control mode
 *
 */
int32_t drvSys_start_motion_control(drvSys_controlMode = dual_control);
/**
 * @brief obtain the current state of the drive system
 *
 * @return +1 if started, -1 not able to start
 */
 //drvSys_State_t drvSys_get_current_state();

void drvSys_start_debug_output();

void drvSys_stop_controllers();

void drvSys_set_notch_filters(int filter_id, float notch_frequ, float bandwidth_f = 10, bool activate = false);

void drvSys_remove_notch_filter(int filter_id = 0);


/* Interface Functions */

/**
 * @brief
 *
 * @return drvSys_parameters
 */
drvSys_parameters drvSys_get_parameters();
/**
 * @brief
 *
 * @return drvSys_controllerState
 */
drvSys_controllerState drvSys_get_controllerState();


/**
 * @brief sets torque target to the FOC controller - directly
 *
 * @param target_torque
 */
void drvSys_set_target_motor_torque(float target_torque);
/**
 * @brief sets torque target to the torque controller - checks for torque limits
 *
 * @param torque
 */
void drvSys_set_target_torque(float torque);

void drvSys_set_target_velocity(float vel);

void drvSys_set_target_pos(float angle);

void drvSys_set_kalman_filter_acc_noise(float acc_noise, bool joint);
drvSys_driveTargets drvSys_get_targets();

void drvSys_set_feed_forward_torque(float torque_ff);

void drvSys_set_feed_forward_velocity(float vel_ff);

/**
 * @brief checks wether the input is valid w.r.t. the joint limit
 * acts as software endstop;
 *
 * @param input - input command (torque, velocity)
 * @return float - input, or 0.0 if joint limit is reached
 */
float _drvSys_check_joint_limit(float input);

/* Functions to change persistent parameters (PID Gains etc.) */

void _drvSys_load_parameters_from_Flash();

// Calibration
void drvSys_save_encoder_offsets_to_Flash();
bool drvSys_read_encoder_offsets_from_Flash();
void drvSys_reset_encoder_offset_data_on_Flash();
// Alignment
void drvSys_save_alignment_to_Flash();
bool drvSys_read_alignment_from_Flash();
void drvSys_reset_alignment_data_on_Flash();
//Controller Gains
void drvSys_set_pos_PID_gains(float Kp, float Ki, float Kd, bool save = true);
void drvSys_save_pos_PID_gains();

/**
 * @brief
 *
 * @param type 0 - pos, 1 - vel
 * @param filter_alpha - alpha of exponential output filter 0.0 = inactive
 * @param deadzone - error deadzone
 */
void drvSys_set_advanced_PID_settings(int type = 0, float filter_alpha = 0.0, float deadzone = 0.0, bool active = true);

void drvSys_set_vel_PID_gains(float Kp, float Ki, float Kd, bool save = true);
void drvSys_save_vel_PID_gains();

void drvSys_set_admittance_params(float virtual_spring, float virtual_damping, float virtual_inertia, bool save = true);
void drvSys_save_admittance_params();


bool _drvSys_read_pos_PID_gains_from_flash();
bool _drvSys_read_vel_PID_gains_from_flash();
bool _drvSys_read_admittanceGains_from_flash();

void drvSys_save_angle_offset(float angle_offset);

/* ###################################################
############ Internal Drive System functions #########
###################################################### */

/**
 * @brief sets up the Motor Encoder and the FOC Controller
 *
 */
void _drvSys_setup_FOC_Driver();
/**
 * @brief creates Interrupt Timers for Processing & Controllers
 *
 */
void _drvSys_setup_interrupts();

void _drvSys_calibrate_with_hallsensor();

void _drvSys_align_axis();


drvSys_driveState drvSys_get_drive_state();

drvSys_extendedDriveState drvSys_get_extended_drive_state();

drvSys_controllerState drvSys_get_controllerState();

drvSys_parameters drvSys_get_parameters();

drvSys_Constants drvSys_get_constants();

drvSys_driveTargets drvSys_get_targets();

/* Interrupt Handler */

void IRAM_ATTR _drvSys_on_foc_timer();

/* ############################
########### RTOS TASKS ########
###############################*/

/* FOC-Control */
void _drvSys_foc_controller_task(void* parameters);

void _drvSys_set_empiric_phase_shift(float phase_shift_factor);

/* Process Sensor Tasks */
void _drvSys_process_torque_sensor_task(void* parameters);

void _drvSys_process_encoders_task(void* parameters);



/* --- RTOS Controller Tasks --- */
void _drvSys_PID_dual_controller_task(void* parameters);

void _drvSys_velocity_controller_task(void* parameters);

void _drvSys_torque_controller_task(void* parameters);

void _drvSys_admittance_controller_task(void* parameters);


/* Debug & Test Tasks */
void _drvSys_debug_print_position_task(void* parameters);

void _drvSys_read_serial_commands_task(void* parameters);

void _drvSys_test_signal_task(void* parameters);

void _drvSys_process_peripheral_sensors_task(void* parameters);

/**
 * @brief Starts a test signal task for debugging and controller tuning
 *
 * @param signal_target 0 - pos, 1 - vel, 2 - m_torque, 3 - output_torque
 * @param shape 0 - square, 1 - sine wave, 2 - ramp function
 * @param max maximum value (deg, deg/s, Nm)
 * @param min minimum value (deg, deg/s, Nm)
 * @param period (ms)
 */
void drvSys_start_test_signal(int signal_target, int shape, float max, float min, float period);

void drvSys_stop_test_signal();

/* internal Funvtions */

void _drvSys_setup_dual_controller();

void _drvSys_setup_direct_controller();

void _drvSys_setup_admittance_controller();

//void _drvSys_setup_impedance_controller();

//void _drvSys_setup_hybrid_controoller();

void drvSys_calibrate_FOC();








#endif //DRIVE_SYSTEM_H