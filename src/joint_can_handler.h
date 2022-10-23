#ifndef JOINT_CAN_HANDLER_H
#define JOINT_CAN_HANDLER_H

#include <drive_system.h>
#include <drv_can_utils.h>
#include <led_control.h>
#include <motion_interface.h>

#include <CAN.h>

#define CAN_PERIOD_MS 1
#define CAN_RATE_HZ 1000

#define CAN_HANDLER_STACK_SIZE 2000
#define CAN_HANDLER_PRIO 8

#define CAN_DRIVE_STATE_RATE 250 //Hz
#define CAN_CTRL_STATE_RATE 10



const int can_drive_state_divier = CAN_RATE_HZ / CAN_DRIVE_STATE_RATE;
const int can_ctrl_state_divier = CAN_RATE_HZ / CAN_CTRL_STATE_RATE;

void can_handle_task(void* params);


void can_init();
void can_start_interface();
void can_send_controller_state(drvSys_controllerCondition controller_state);
//void can_send_drive_state(drvSys_driveState drive_state);
void can_send_drive_state(drvSys_FullDriveState drive_state);
void can_send_load_state(float joint_torque, float delta_pos, bool torque_calibrated);

void can_send_pid_data(drvSys_parameters params);

void can_parse_pid_data();

void can_read_incoming_CAN_commands();
void can_parse_drive_target();

void can_parse_parameter_command(drvComm_paramsCmd param_cmd);
void can_parse_controller_command(drvComm_controllerCmd control_cmd);







#endif // !JOINT_CAN_HANDLER_H