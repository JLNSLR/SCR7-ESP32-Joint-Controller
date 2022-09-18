#ifndef JCTRL_CLI_INTERFACE_FUNCTIONS_H 
#define JCTRL_CLI_INTERFACE_FUNCTIONS_H

#include <Arduino.h>
extern bool jcrtl_cli_feedback;

extern bool cli_output_active;
enum cli_output_mode { all, load_side, extended, tune_pos, nn_inv, nn_em, nn_pid, load_control, tune_vel, torque };

#define N_MAX_ARGS 64

bool jctrl_cli_process_torque_command(char(*cli_arg)[N_MAX_ARGS]);

bool jctrl_cli_process_position_command(char(*cli_arg)[N_MAX_ARGS]);

bool jctrl_cli_process_pid_command(char(*cli_arg)[N_MAX_ARGS]);

bool jctrl_cli_process_drive_sys_command(char(*cli_arg)[N_MAX_ARGS]);

bool jctrl_cli_process_output_command(char(*cli_arg)[N_MAX_ARGS]);

bool jctrl_cli_process_controller_state_command(char(*cli_arg)[N_MAX_ARGS]);

bool jctrl_cli_process_adapt_kalman(char(*cli_arg)[N_MAX_ARGS]);

bool jctrl_cli_process_nn_commands(char(*cli_arg)[N_MAX_ARGS]);

bool jctrl_cli_process_motion_planner_commands(char(*cli_arg)[N_MAX_ARGS]);

bool jctrl_cli_process_motion_plan_constraints_commands(char(*cli_arg)[N_MAX_ARGS]);

bool jctrl_cli_process_trajectory_command(char(*cli_arg)[N_MAX_ARGS]);

bool jctrl_cli_manage_calibration_command(char(*cli_arg)[N_MAX_ARGS]);

bool jctrl_cli_limit_command(char(*cli_arg)[N_MAX_ARGS]);

void _jctrl_cli_feedback_output(String output);

void _jctrl_cli_output_periodic();


#endif //JCTRL_INTERFACE_FUNCTIONS_H
