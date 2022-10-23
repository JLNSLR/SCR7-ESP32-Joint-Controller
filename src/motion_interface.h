#ifndef MOTION_INTERFACE_H
#define MOTION_INTERFACE_H

#include <drive_system.h>
#include <motion_planner.h>

#define MOTION_MAX_JERK 1000.0 //rad/s^3
#define MOTION_MAX_ACC 1200.0*DEG2RAD
#define MOTION_MAX_VEL 140.0*DEG2RAD

#define MOTION_SEQUENCER_CORE 1
#define MOTION_SEQUENCER_PRIO 8
#define MOTION_SEQUENCER_STACK_SIZE 2000

#define MOTION_SEQUENCER_PERIOD_MS 2

extern MotionPlanner motion_planner;

enum motion_control_mode { position, trajectory };

void start_motion_interface();

void handle_motion_command(float target_pos, float travel_vel = 0, float travel_acc = 0);

void handle_motion_command(drvSys_driveTargets target_traj_point);

void handle_motion_control_command(drvSys_driveControlTargets target_data);

void _motion_sequencer_task(void* parameters);

void set_motion_mode(motion_control_mode);

void set_motion_planner_constraints(float max_vel, float max_acc, float max_jerk);

void start_goto_test_signal();

void stop_test_signal();

void start_sinusoidal_test_signal(float max_acc, float max_frequ);

void _output_test_sinusoid_signal();



#endif // !MOTION_INTERFACE_H
