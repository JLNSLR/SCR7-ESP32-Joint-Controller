#include<motion_interface.h>
#include<CircularBuffer.h>

TaskHandle_t motion_sequencer_th;
extern MotionPlanner motion_planner(MOTION_MAX_VEL, MOTION_MAX_ACC, MOTION_MAX_ACC, MOTION_MAX_JERK, MOTION_MAX_JERK);

motion_control_mode motion_mode = position;

bool reset_test_sig = true;

struct extendedPositionCommand {
    float target_pos;
    float travel_vel = 0;
    float travel_acc = 0;
};

struct test_sinusoid {
    float max_acc;
    float max_frequ;
};

test_sinusoid test_sine_data;

CircularBuffer<extendedPositionCommand, 4> position_command_buffer;
CircularBuffer<drvSys_driveTargets, 10> trajectory_command_buffer;

bool command_given = false;

bool test_sinusoid_active = false;
bool test_goto_active = false;

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

void handle_motion_control_command(drvSys_driveControlTargets target_data) {

    motion_mode = trajectory;
    drvSys_set_control_targets(target_data);
    command_given = true;

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
    command_given = true;
}


void handle_motion_command(float target_pos, float travel_vel, float travel_acc) {
    motion_mode = position;
    if (!motion_planner.executing_traj_flag) {

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
    command_given = true;
}

void handle_motion_command(drvSys_driveTargets target_traj_point) {

    motion_mode = trajectory;
    drvSys_set_target(target_traj_point);
    command_given = true;
}



void _motion_sequencer_task(void* parameters) {

    const TickType_t motion_delay = MOTION_SEQUENCER_PERIOD_MS / portTICK_PERIOD_MS;

    static long last_time_us = 0;

    static long counter = 0;

    while (true) {

        if (motion_mode == position && command_given) {

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
            target.motor_torque_ff = 0;

            drvSys_set_target(target);
        }

        counter++;

        if (test_goto_active) {
            if (counter % 500 == 0) {

                static float dir = 1.0;
                float target = 0;
                float acc = 0;
                static int no_traj_counter = 0;
                if (!motion_planner.executing_traj_flag) {

                    no_traj_counter++;

                    if (no_traj_counter % 10 == 0) {
                        no_traj_counter = 0;
                        target = (float(rand()) / float(RAND_MAX)) * 90;
                        if (float(rand()) / float(RAND_MAX) > 0.5) {
                            dir = -dir;
                        }
                        float vel = (float(rand()) / float(RAND_MAX)) * 140.0 + 3.0;
                        acc = (float(rand()) / float(RAND_MAX)) * 100.0 + 5.0;

                        handle_motion_command(target * DEG2RAD * dir, vel * DEG2RAD, acc * DEG2RAD);
                    }
                }


            }
        }
        else if (test_sinusoid_active) {
            _output_test_sinusoid_signal();
        }



        vTaskDelay(motion_delay);



    }



}

void set_motion_planner_constraints(float max_vel, float max_acc, float max_jerk) {

    motion_planner.constraints.max_accel = max_acc;
    motion_planner.constraints.max_deccel = max_acc;
    motion_planner.constraints.max_vel = max_vel;
    motion_planner.constraints.max_jerk = max_jerk;
    motion_planner.constraints.max_jerk_deccel = max_jerk;

}

void start_sinusoidal_test_signal(float max_acc, float max_frequ) {

    test_sine_data.max_acc = max_acc;
    test_sine_data.max_frequ = max_frequ;

    test_sinusoid_active = true;
}

void stop_test_signal() {

    test_goto_active = false;
    test_sinusoid_active = false;

    reset_test_sig = true;
}


void start_goto_test_signal() {

    test_goto_active = true;
}



void _output_test_sinusoid_signal() {

    static int n_counter = 0;

    static float delta_frequ = 0.01;
    static int f_counter = 0;
    static float frequ = 0.0001;

    static bool frequ_change = false;

    if (reset_test_sig) {
        frequ = 0.0001;
        n_counter = 0;
        reset_test_sig = false;
    }

    float delta_t = 1e-3;

    float t = n_counter * delta_t;

    test_sine_data.max_acc = 1000;

    float pos_amplitude = 90.0 * DEG2RAD;
    float acc_amplitude = pos_amplitude * (2.0 * PI * frequ) * (2.0 * PI * frequ);
    float vel_amplitude = pos_amplitude * (2.0 * PI * frequ);

    /*
    if (acc_amplitude > test_sine_data.max_acc) {
        pos_amplitude = test_sine_data.max_acc * (1.0 / (2.0 * PI * frequ));
        vel_amplitude = test_sine_data.max_acc * (1.0 / (2.0 * PI * frequ));
        acc_amplitude = test_sine_data.max_acc;
    }
    */

    float acc = -acc_amplitude * sin(2.0 * PI * frequ * t);
    float vel = vel_amplitude * cos(2.0 * PI * frequ * t);
    float pos = pos_amplitude * sin(2.0 * PI * frequ * t);
    /*
    Serial.println("---");
    Serial.println(acc * RAD2DEG);
    Serial.println(vel * RAD2DEG);
    Serial.println(pos * RAD2DEG);
    Serial.println("--");
    */

    n_counter++;


    if (pos < 0.001 && pos > 0 && frequ_change == false) {
        frequ = frequ + delta_frequ;
        frequ_change = true;
    }

    if (abs(pos) > 0.3) {
        frequ_change = false;
    }

    if (frequ > test_sine_data.max_frequ) {
        frequ = 0.0001;
    }


    drvSys_driveTargets target;
    target.acc_target = acc;
    target.vel_target = vel;
    target.pos_target = pos;
    target.motor_torque_ff = 0;

    drvSys_set_target(target);


}