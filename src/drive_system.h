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
#include <drive_system_types.h>
#include <neural_controller.h>
#include <closed_loop_stepper_control.h>

#include <Preferences.h>


/* ##########################################################################
############## ---- Constant Drive System Parameters ----####################
############################################################################*/


/* --- Timing Constants --- */
#define DRVSYS_FOC_PERIOD_US 200 //us -> 5kHz
//Encoder Processing
#define DRVSYS_PROCESS_ENCODERS_PERIOD_US 500 //us -> 3kHz
#define DRVSYS_PROCESS_ENCODERS_FREQU 2000 //Hz
// Torque Target Control 
#define DRVSYS_CONTROL_TORQUE_PERIOD_US 400 // us 
#define DRVSYS_CONTROL_TORQUE_FREQU 2500 // Hz
// PID Controller
#define DRVSYS_CONTROL_POS_PERIOD_US 1000 //us 2000Hz
#define DRVSYS_CONTROL_POS_FREQ 1000 //Hz
// Torque Sensor Processing
#define DRVSYS_PROCESS_TORQUE_SENSOR_PERIOD_MS 333

// Closed Loop Stepper Controller
#define DRVSYS_CONTROL_STEPPER_PERIOD_US 500
#define DRVSYS_CONTROL_STEPPER_FREQU 2000 //Hz

//Admittance Controller
#define DRVSYS_CONTROL_ADMITTANCE_PERIOD_MS 5

//Learning System Dynamics
#define DRVSYS_LEARNING_PERIOD_MS 2

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

void drvSys_initialize_foc_based_control();

void drvSys_initialize_stepper_based_control();

int32_t drvSys_start_foc_processing();
/**
 * @brief starts the motion controllers depending on the control mode
 *
 */
int32_t drvSys_start_motion_control(drvSys_controlMode mode = closed_loop_foc);


void _drvSys_start_stepper_controller();
void _drvSys_stop_stepper_controller();

void drvSys_stop_controllers();


/* Interface Functions */

drvSys_parameters& drvSys_get_parameters();
const drvSys_controllerCondition drvSys_get_controllerState();
const drvSys_driveState drvSys_get_drive_state();
const drvSys_FullDriveState drvSys_get_full_drive_state();
const drvSys_Constants drvSys_get_constants();
const drvSys_driveTargets drvSys_get_targets();
drvSys_FullDriveStateTimeSample drvSys_get_full_drive_state_time_samples();

/**
 * @brief sets the target values for the joint controller including
 * position (rad), velocity (rad/s), acceleration (rad/s^2), motor torque (Nm), joint torque (Nm)
 * motor torque acts as feed forward term
 * @param targets
 */
void drvSys_set_target(drvSys_driveTargets targets);


/**
 * @brief sets motor torque target to the torque controller - checks for torque limits
 *
 * @param torque
 */
void _drvSys_set_target_torque(float torque_Nm);

/**
 * @brief sets velocity target
 *
 * @param vel_rad
 */
void _drvSys_set_target_velocity(float vel_rad);

void _drvSys_set_target_pos(float angle_rad);


// Used for direct torque control
void drvSys_set_feed_forward_torque(float torque_ff);

/**
 * @brief checks wether the input is valid w.r.t. the joint limit
 * acts as software endstop;
 *
 * @param input - input command (torque, velocity)
 * @return float - input, or 0.0 if joint limit is reached
 */
float _drvSys_check_joint_limit(float input);


/* Adapt Filter Functions */
void drvSys_set_notch_filter(float notch_frequ, bool activate = false);

void drvSys_set_kalman_filter_acc_noise(float acc_noise, bool joint);

/* Functions to change persistent parameters (PID Gains etc.) */

void _drvSys_load_parameters_from_Flash();

// Calibration
void drvSys_save_encoder_offsets_to_Flash();
bool drv_Sys_check_if_joint_encoder_is_calibrated();
void drvSys_reset_encoder_offset_data_on_Flash();
// Alignment
void drvSys_save_alignment_to_Flash();
bool drvSys_read_alignment_from_Flash();
void drvSys_reset_alignment_data_on_Flash();

void drvSys_setOffsets(float motor_offset, float joint_offset, bool save = true, bool reset = false);
void drvSys_loadOffsets();
//Controller Gains
void drvSys_update_PID_gains(drvSys_PID_Gains gains);
void drvSys_set_pos_PID_gains(float Kp, float Ki, float Kd, bool save = true);
void drvSys_set_ff_gains(float gain);
void drvSys_save_pos_PID_gains();

void drvSys_limit_torque(float torque_limit);

void drvSys_set_admittance_params(float virtual_spring, float virtual_damping, float virtual_inertia, bool save = true);
void drvSys_save_admittance_params();


bool _drvSys_read_pos_PID_gains_from_flash();
bool _drvSys_read_admittanceGains_from_flash();


void _drvSys_save_angle_offset(float angle_offset);

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


/* Inverse Dynamics Learner */

void _drvSys_neural_controller_setup();
void _drvSys_learn_neural_control_task(void* parameters);
void drvSys_neural_control_activate(bool active);
void drvSys_neural_control_save_nets(bool reset = false);
/**
 * @brief Adapts parameters of NN online
 *
 * @param nn_type 0 - Emulator, 1 - Controller
 * @param parameter_type 0 - lr, 1 - min_lr,  2 - lr_error_factor, 3 - max_lr, 4 -
 * @param value
 */
void drvSys_adapt_nn_parameter(int nn_type, int parameter_type, float value);
/**
 * @brief
 *
 * @param nn_type 0 - Emulator, 1-Controller
 * @param error_type 0 error, 1 filtered error
 * @return float
 */
float drvSys_get_neural_control_error(int nn_type, int error_type);

float drvSys_neural_control_error();
float _drvSys_neural_control_predict_torque();
float drvSys_neural_control_read_predicted_torque();
drvSys_driveState drvSys_get_emulator_pred();

float drvSys_get_pid_torque();

float _drvSys_compute_notch_FIR_filter(float input, float* b_coef);


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

void _drvSys_torque_controller_task(void* parameters);

void _drvSys_admittance_controller_task(void* parameters);


void _drvSys_monitor_system_task(void* parameters);

void _drvSys_closed_loop_stepper_task(void* parameters);


/* internal Functions */

void _drvSys_setup_dual_controller();

void _drvSys_setup_direct_controller();

void _drvSys_setup_admittance_controller();

//void _drvSys_setup_impedance_controller();

//void _drvSys_setup_hybrid_controoller();

void drvSys_calibrate_FOC();








#endif //DRIVE_SYSTEM_H