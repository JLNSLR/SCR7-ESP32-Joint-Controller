#include<motion_interface.h>
#include<CircularBuffer.h>

TaskHandle_t motion_sequencer_th;

MotionPlanner motion_planner(MOTION_MAX_VEL, MOTION_MAX_ACC, MOTION_MAX_ACC, MOTION_MAX_JERK, MOTION_MAX_JERK);

motion_control_mode motion_mode = position;

struct extendedPositionCommand {
    float target_pos;
    float travel_vel = 0;
    float travel_acc = 0;
};

CircularBuffer<extendedPositionCommand, 4> position_command_buffer;
CircularBuffer<drvSys_driveTargets, 10> trajectory_command_buffer;


void start_motion_interface() {

    Serial.println("MOTION_INTERFACE: Started Motion Interface");

    xTaskCreatePinnedToCore(
        _motion_sequencer_task,   // function name
        "Motion Sequencer Task", // task name
        MOTION_SEQUENCER_STACK_SIZE,      // Stack size (bytes)
        NULL,      // task parameters
        MOTION_SEQUENCER_PRIO,         // task priority
        &motion_sequencer_th,
        MOTION_SEQUENCER_CORE // task handle
    );


}

void handle_motion_command(float target_pos) {
    if (motion_planner.executing_traj_flag) {

        float start = drvSys_get_drive_state().joint_pos;
        motion_planner.planMotion(start, target_pos);
    }
    else {
        extendedPositionCommand command;
        command.target_pos = target_pos;
        command.travel_acc = 0;
        command.travel_vel = 0;

        position_command_buffer.push(command);
    }
}

void handle_motion_command(float target_pos, float travel_vel, float travel_acc) {
    motion_mode = position;
    if (motion_planner.executing_traj_flag) {

        float start = drvSys_get_drive_state().joint_pos;
        motion_planner.planMotion(start, target_pos, travel_vel, travel_acc);
    }
    else {
        extendedPositionCommand command;
        command.target_pos = target_pos;
        command.travel_acc = travel_acc;
        command.travel_vel = travel_vel;

        position_command_buffer.push(command);
    }
}

void handle_motion_command(drvSys_driveTargets target_traj_point) {

    motion_mode = trajetory;
    drvSys_set_target(target_traj_point);
}

void _motion_sequencer_task(void* parameters) {

    const TickType_t motion_delay = MOTION_SEQUENCER_PERIOD_MS / portTICK_PERIOD_MS;

    static long last_time_us = 0;

    while (true) {

        if (motion_mode == position) {

            if (!position_command_buffer.isEmpty() && !motion_planner.executing_traj_flag) {
                extendedPositionCommand command = position_command_buffer.pop();
                float start = drvSys_get_drive_state().joint_pos;
                motion_planner.planMotion(start, command.target_pos, command.travel_vel, command.travel_vel);
            }

            long this_time_t_us = micros();
            int delta_t_us = this_time_t_us - last_time_us;
            float delta_t = float(delta_t_us) * 1e-6;
            last_time_us = this_time_t_us;

            trajectory_point target_point = motion_planner.sequenceMotion(delta_t);

            drvSys_driveTargets target;
            target.acc_target = target_point.acceleration;
            target.vel_target = target_point.velocity;
            target.pos_target = target_point.position;
            target.motor_torque_target = 0;
            target.ref_torque = 0;

            drvSys_set_target(target);
        }



    }


    vTaskDelay(motion_delay);
}