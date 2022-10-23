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
#include <torqueSensor.h>
#include <Temperature_LM75_Derived.h>
#include <led_control.h>

#include <Preferences.h>

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

void drvSys_set_position_control(bool active);

void drvSys_set_ff_control(bool active);

void drvSys_set_velocity_control(bool active);

/* Interface Functions */

drvSys_parameters& drvSys_get_parameters();
const drvSys_controllerCondition drvSys_get_controllerState();
const drvSys_driveState drvSys_get_drive_state();
const drvSys_FullDriveState drvSys_get_full_drive_state();
const drvSys_Constants drvSys_get_constants();
const drvSys_driveTargets drvSys_get_targets();
drvSys_FullDriveStateTimeSample drvSys_get_full_drive_state_time_samples();

TorqueSensor& drvSys_get_torqueSensor();

/**
 * @brief sets the target values for the joint controller including
 * position (rad), velocity (rad/s), acceleration (rad/s^2), motor torque (Nm), joint torque (Nm)
 * motor torque acts as feed forward term
 * @param targets
 */
void drvSys_set_target(drvSys_driveTargets targets);

void drvSys_set_control_targets(drvSys_driveControlTargets targets);


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

/**
 * @brief returns torque sensor output, also outputs if the torque sensor is uncalibrated
 *
 * @return float  torque sensor value
 */
float drvSys_get_torque(bool raw = false);

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
/**
 * @brief sets PID gains
 *
 * @param pos - bool true -> position controller, false -> velocity controller
 * @param Kp
 * @param Ki
 * @param Kd
 * @param save - if true saves new gains on flash
 */


drvSys_cascade_gains drvSys_get_controller_gains();
drvSys_PID_Gains drvSys_get_PID_gains(bool pos_controller = true);
void drvSys_set_PID_gains(bool pos, float Kp, float Ki, float Kd, bool save = true);
void drvSys_set_ff_gains(float vel_ff_gain, float acc_ff_gain);
void drvSys_save_PID_gains();

/**
 * @brief adapts advanced PID settings
 *
 * @param pos  - bool true -> position controller, false -> velocity controller
 * @param type 0 - input filter alpha; 1 - deadzone
 * @param value alpha/deadzone
 */
void drvSys_adv_PID_settings(bool pos, int type, float value);

void drvSys_limit_torque(float torque_limit);


bool _drvSys_read_PID_gains_from_flash();

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
/**
 * @brief sets neural PID settings
 * @param type - int: 0 - learning rate, 1 - gain_regularization
 * @param parameter_val - float
 */
void drvSys_neural_PID_settings(int type, float parameter_val);
void drvSys_neural_control_activate(bool active);
void drvSys_neural_control_save_nets(bool reset = false);
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

drvSys_cascade_gains _drvSys_predict_pid_gains();

float drvSys_pid_nn_error(bool average = false);

/**
 * @brief if torqueboost is active, the maximum phase current is used in foc control instead of the
 *         nominal phase current. Activated as standard, since FOC control is very efficient.
 *         Should be turned off if the driver or the motor gets increasingly hot
 *
 * @param active
 */
void drvSys_set_torque_boost_active(bool active);

float drvSys_get_pid_torque();

float _drvSys_compute_notch_FIR_filter(float input, float* b_coef);

float drvSys_get_delta_angle();


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

void _drvSys_monitor_system_task(void* parameters);

void _drvSys_closed_loop_stepper_task(void* parameters);


/* internal Functions */

void _drvSys_setup_dual_controller();

void _drvSys_setup_direct_controller();

void drvSys_calibrate_FOC();








#endif //DRIVE_SYSTEM_H