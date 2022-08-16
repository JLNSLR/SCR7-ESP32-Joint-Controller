#ifndef JOINT_CAN_HANDLER_H
#define JOINT_CAN_HANDLER_H

#include <drive_system.h>
#include <drv_can_utils.h>
#include <led_control.h>

#include <CAN.h>

#define CAN_PERIOD_MS 1
#define CAN_HANDLER_STACK_SIZE 2000
#define CAN_HANDLER_PRIO 8


void can_handle_task(void* params);


void can_init();
void can_start_interface();
void can_send_controller_state(drvSys_controllerCondition controller_state);
void can_send_drive_state(drvSys_driveState drive_state);

void can_send_pid_data(drvSys_parameters params);

void can_parse_pid_data();

void can_read_incoming_CAN_commands();
void can_parse_drive_target();







#endif // !JOINT_CAN_HANDLER_H