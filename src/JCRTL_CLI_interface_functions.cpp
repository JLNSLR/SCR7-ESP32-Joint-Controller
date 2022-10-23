#include <JCRTL_CLI_interface_functions.h>
#include <drive_system.h>
#include <motion_interface.h>

float inv_transmission;

cli_output_mode cli_out_mode = all;

bool jcrtl_cli_feedback = true;

bool cli_output_active = false;

void _jctrl_cli_feedback_output(String output) {

    if (jcrtl_cli_feedback) {
        Serial.print("DRVSYS_CLI: ");
        Serial.println(output);
    }

}

bool jctrl_cli_process_torque_command(char(*cli_arg)[N_MAX_ARGS]) {

    char* keyword = cli_arg[0];

    bool error = false;

    bool processed = false;

    if (strcmp(keyword, "torque") == 0 || strcmp(keyword, "t") == 0) {
        processed = true;

        float target_torque;
        if (strcmp(cli_arg[1], "ff") == 0) {
            target_torque = atof(cli_arg[2]);
        }
        target_torque = atof(cli_arg[1]);

        _jctrl_cli_feedback_output("Torque command: " + String(target_torque) + " Nm.");
        _drvSys_set_target_torque(target_torque);
        return processed;
    }

    return processed;

}


bool jctrl_cli_process_position_command(char(*cli_arg)[N_MAX_ARGS]) {

    char* keyword = cli_arg[0];

    bool processed = false;

    if (strcmp(keyword, "pos") == 0) {
        processed = true;
        float pos_target = atof(cli_arg[1]);
        _drvSys_set_target_pos(pos_target * DEG2RAD);

    }
    return processed;

}

bool jctrl_cli_process_motion_planner_commands(char(*cli_arg)[N_MAX_ARGS]) {

    char* keyword = cli_arg[0];
    char* value_0 = cli_arg[1];
    char* value_1 = cli_arg[2];
    char* value_2 = cli_arg[3];
    bool processed = false;

    if (strcmp(keyword, "mov") == 0) {
        processed = true;
        float position_target = atof(value_0) * DEG2RAD;
        float travel_vel = atof(value_1) * DEG2RAD;
        float travel_acc = atof(value_2) * DEG2RAD;

        handle_motion_command(position_target, travel_vel, travel_acc);


    }

    if (strcmp(keyword, "mov_test") == 0) {
        processed = true;
        bool test_sig_goto = atoi(value_0);

        if (test_sig_goto == true) {
            start_goto_test_signal();
            return processed;
        }

        if (test_sig_goto == false) {

            start_sinusoidal_test_signal(DRVSYS_ACC_MAX, 1);
        }

    }
    if (strcmp(keyword, "mov_stop") == 0) {
        processed = true;
        stop_test_signal();
    }


    return processed;

}

bool jctrl_cli_process_pid_command(char(*cli_arg)[N_MAX_ARGS]) {

    char* keyword = cli_arg[0];
    char* keyword_2 = cli_arg[1];
    char* keyword_3 = cli_arg[2];
    bool processed = false;

    if (strcmp(keyword, "pid") == 0 || strcmp(keyword, "PID") == 0) {
        processed = true;

        bool pos = true;

        if (strcmp(keyword_2, "p") == 0 || strcmp(keyword_2, "pos") == 0) {
            pos = true;
        }

        if (strcmp(keyword_2, "v") == 0 || strcmp(keyword_2, "vel") == 0) {
            pos = false;
        }


        if (strcmp(keyword_3, "gains") == 0 || strcmp(keyword_3, "g") == 0) {

            float p_val = atof(cli_arg[3]);
            float i_val = atof(cli_arg[4]);
            float d_val = atof(cli_arg[5]);

            bool save = false;

            if (strcmp(cli_arg[5], "s") == 0) {
                save = true;
                _jctrl_cli_feedback_output("Saved position PID gains to: " + String(p_val) + ", " + String(i_val) + ", " + String(d_val) + ".");
            }
            else {
                if (pos) {
                    _jctrl_cli_feedback_output("Position Controller: ");
                }
                else {
                    _jctrl_cli_feedback_output("Velocity Controller: ");
                }
                _jctrl_cli_feedback_output("Set PID gains to: " + String(p_val) + ", " + String(i_val) + ".");
            }

            drvSys_set_PID_gains(pos, p_val, i_val, d_val, save);
        }

        if (strcmp(keyword_3, "adv") == 0) {

            int type = atoi(cli_arg[3]);
            float val = atof(cli_arg[4]);

            drvSys_adv_PID_settings(pos, type, val);

            _jctrl_cli_feedback_output("Changed PID settings.");
        }


    }

    return processed;


}

bool jctrl_cli_process_drive_sys_command(char(*cli_arg)[N_MAX_ARGS]) {

    char* keyword = cli_arg[0];

    bool processed = false;

    if (strcmp(keyword, "start") == 0) {
        processed = true;

        int32_t mode = atoi(cli_arg[1]);

        drvSys_controllerCondition state = drvSys_get_controllerState();

        if (state.state_flag == closed_loop_control_inactive) {

            if (mode == 0) {
                drvSys_start_motion_control(closed_loop_foc);

                _jctrl_cli_feedback_output("Started motion control in dual control mode.");
                return processed;
            }
            if (mode == 1) {
                drvSys_start_motion_control(direct_torque);
                _jctrl_cli_feedback_output("Started motion control in direct torque mode.");
                return processed;
            }
            if (mode == 2) {
                drvSys_start_motion_control(stepper_mode);
                _jctrl_cli_feedback_output("Started motion control in stepper mode.");
                return processed;
            }

        }
    }

    if (strcmp(keyword, "stop") == 0) {
        drvSys_stop_controllers();
        _jctrl_cli_feedback_output("Stopped motion controllers.");
        return processed;
    }


    

    return processed;

}

bool jctrl_cli_process_output_command(char(*cli_arg)[N_MAX_ARGS]) {

    char* keyword = cli_arg[0];
    bool processed = false;



    inv_transmission = 1.0 / drvSys_get_constants().transmission_ratio;

    if (strcmp(keyword, "out") == 0) {
        processed = true;
        char* mode = cli_arg[1];

        if (strcmp(mode, "all") == 0) {
            cli_output_active = true;
            cli_out_mode = all;

            Serial.print("keyword: ");
            Serial.println(keyword);
            Serial.print("mode: ");
            Serial.println(mode);

            return processed;;
        }

        if (strcmp(mode, "load") == 0) {
            cli_output_active = true;
            cli_out_mode = load_side;
            return processed;;
        }
        if (strcmp(mode, "full") == 0) {
            cli_output_active = true;
            cli_out_mode = extended;
            return processed;;
        }

        if (strcmp(mode, "stop") == 0) {
            cli_output_active = false;
            return processed;;
        }
        if (strcmp(mode, "tune_pos") == 0) {
            cli_output_active = true;
            cli_out_mode = tune_pos;
            return processed;
        }
        if (strcmp(mode, "tune_vel") == 0) {
            cli_output_active = true;
            cli_out_mode = tune_vel;
            return processed;
        }
        if (strcmp(mode, "torque") == 0) {
            cli_output_active = true;
            cli_out_mode = torque;
            return processed;
        }
        if (strcmp(mode, "nn_inv") == 0) {
            cli_output_active = true;
            cli_out_mode = nn_inv;
            return processed;
        }
        if (strcmp(mode, "nn_em") == 0) {
            cli_output_active = true;
            cli_out_mode = nn_em;
            return processed;
        }
        if (strcmp(mode, "load_nn") == 0) {
            cli_output_active = true;
            cli_out_mode = load_control;
            return processed;
        }
        if (strcmp(mode, "nn_pid") == 0) {
            cli_output_active = true;
            cli_out_mode = nn_pid;
            return processed;
        }


    }
    return processed;
}

bool jctrl_cli_process_controller_state_command(char(*cli_arg)[N_MAX_ARGS]) {

    char* keyword = cli_arg[0];
    bool processed = false;

    if (strcmp(keyword, "state") == 0) {
        processed = true;
        drvSys_controllerCondition state = drvSys_get_controllerState();

        Serial.print("Control Mode: ");
        String control_mode;
        if (state.control_mode == direct_torque) {
            control_mode = "Direct Torque";
        }
        if (state.control_mode == closed_loop_foc) {
            control_mode = "Closed Loop FOC";
        }
        if (state.control_mode == stepper_mode) {
            control_mode = "Stepper";
        }

        Serial.println(control_mode);

        Serial.print("Controller State: ");
        String state_flag;
        if (state.state_flag == error) {
            state_flag = "Error";
        }
        if (state.state_flag == not_ready) {
            state_flag = "Not ready";
        }
        if (state.state_flag == closed_loop_control_active) {
            state_flag = "Closed Loop Control Active";
        }
        if (state.state_flag == closed_loop_control_inactive) {
            state_flag = "Closed Loop Control Inactive";
        }

        Serial.print("Position Control: ");
        Serial.println(state.position_control);

        Serial.print("Velocity Control: ");
        Serial.print(state.velocity_control);

        Serial.println(state_flag);

        Serial.print("Calibrated: ");
        Serial.println(state.calibrated);

        Serial.print("Hit negative limit: ");
        Serial.println(state.hit_neg_limit);
        Serial.print("Hit positive limit: ");
        Serial.println(state.hit_positive_limit);

        Serial.print("Overtemperature: ");
        Serial.println(state.overtemperature);

        Serial.print("Temperature Warning ");
        Serial.println(state.temperature_warning);

        Serial.print("Driver Temperature: ");
        Serial.println(state.temperature);

        Serial.print("Fan Level: ");
        Serial.println(state.fan_level);

        Serial.print("Position Control: ");
        Serial.println(state.position_control);

        Serial.print("Velocity Control: ");
        Serial.print(state.velocity_control);

    }
    return processed;
}

bool jctrl_cli_process_adapt_kalman(char(*cli_arg)[N_MAX_ARGS]) {

    char* keyword = cli_arg[0];
    bool processed = false;

    if (strcmp(keyword, "kalman") == 0) {
        processed = true;
        char* acc = cli_arg[1];
        char* type = cli_arg[2];

        float acc_noise = atof(acc);

        if (strcmp(type, "0") == 0) {
            drvSys_set_kalman_filter_acc_noise(acc_noise, false);
        }
        else {
            drvSys_set_kalman_filter_acc_noise(acc_noise, true);
        }

    }
    return processed;
}


bool jctrl_cli_process_nn_commands(char(*cli_arg)[N_MAX_ARGS]) {
    char* keyword = cli_arg[0];
    bool processed = false;

    if (strcmp(keyword, "nn") == 0) {
        processed = true;
        char* command = cli_arg[1];
        float value = atof(cli_arg[3]);

        if (strcmp(command, "s") == 0) {

            drvSys_neural_control_save_nets(false);

        }
        if (strcmp(command, "res") == 0 || strcmp(command, "r") == 0) {

            drvSys_neural_control_save_nets(true);

        }
        if (strcmp(command, "act") == 0) {
            int a = atoi(cli_arg[3]);
            drvSys_neural_control_activate(a);
        }
    }

    return processed;
}


bool jctrl_cli_process_trajectory_command(char(*cli_arg)[N_MAX_ARGS]) {
    char* keyword = cli_arg[0];
    char* value_0 = cli_arg[1];
    char* value_1 = cli_arg[2];
    char* value_2 = cli_arg[3];
    char* value_3 = cli_arg[4];
    char* value_4 = cli_arg[5];


    bool processed = false;

    if (strcmp(keyword, "tr") == 0) {
        processed = true;
        float position_target = atof(value_0);
        float vel_target = atof(value_1);
        float acc_target = atof(value_2);
        float torque_ff = atof(value_3);

        drvSys_driveTargets target;
        target.acc_target = acc_target;
        target.vel_target = vel_target;
        target.pos_target = position_target;
        target.motor_torque_ff = torque_ff;
        handle_motion_command(target);

    }
    return processed;
}
bool jctrl_cli_manage_calibration_command(char(*cli_arg)[N_MAX_ARGS]) {
    char* keyword = cli_arg[0];
    char* keyword_0 = cli_arg[1];
    char* value_1 = cli_arg[2];


    bool processed = false;

    if (strcmp(keyword, "cal") == 0) {
        processed = true;

        if (strcmp(keyword_0, "del") == 0) {
            drvSys_reset_encoder_offset_data_on_Flash();
            _jctrl_cli_feedback_output("Deleted Calibration Data on Flash");

        }

    }

    if (strcmp(keyword, "offset") == 0) {
        processed = true;

        if (strcmp(keyword_0, "m") == 0) {
            float m_offset = atof(value_1);
            drvSys_setOffsets(m_offset, 0, true, false);
        }
        if (strcmp(keyword_0, "j") == 0) {
            float j_offset = atof(value_1);
            drvSys_setOffsets(0, j_offset, true, false);
        }
        if (strcmp(keyword_0, "reset") == 0) {
            drvSys_setOffsets(0, 0, false, true);
        }

    }
    return processed;
}
bool jctrl_cli_process_motion_plan_constraints_commands(char(*cli_arg)[N_MAX_ARGS]) {

}
bool jctrl_cli_limit_command(char(*cli_arg)[N_MAX_ARGS]) {
    char* keyword = cli_arg[0];
    char* type = cli_arg[1];
    char* value_1 = cli_arg[2];
    char* value_2 = cli_arg[3];
    char* value_3 = cli_arg[4];
    char* value_4 = cli_arg[5];


    bool processed = false;

    if (strcmp(keyword, "limit") == 0) {
        processed = true;
        if (strcmp(keyword, "p") == 0) {
            float limit_pos = atof(value_1);
            float limit_neg = atof(value_2);
        }
        if (strcmp(keyword, "t") == 0) {
            float max_torque = atof(value_1);
            drvSys_limit_torque(max_torque);
        }
        if (strcmp(keyword, "v") == 0) {
            float max_vel = atof(value_1);

        }
        if (strcmp(keyword, "i") == 0) {
            float current_max = atof(value_1);

        }

    }
}

bool jctrl_cli_save_command(char(*cli_arg)[N_MAX_ARGS]) {

}


void _jctrl_cli_output_periodic() {



    if (cli_output_active) {
        if (cli_out_mode == all) {

            drvSys_driveTargets targets = drvSys_get_targets();
            drvSys_FullDriveState state = drvSys_get_full_drive_state();

            Serial.print(state.joint_pos * RAD2DEG);
            Serial.print("\t");
            Serial.print(state.joint_vel * RAD2DEG);
            Serial.print("\t");
            Serial.print(state.joint_acc * RAD2DEG);
            Serial.print("\t");
            Serial.print(state.motor_pos * RAD2DEG);
            Serial.print("\t");
            Serial.print(state.motor_vel * RAD2DEG);
            Serial.print("\t");
            Serial.print(state.motor_acc * RAD2DEG);
            Serial.print("\t");
            Serial.print(state.motor_torque);
            Serial.print("\t");
            Serial.print(state.joint_torque);
            Serial.print("\t");
            Serial.print(targets.pos_target * RAD2DEG);
            Serial.print("\t");
            Serial.print(targets.vel_target * RAD2DEG);
            Serial.print("\t");
            Serial.print(targets.acc_target * RAD2DEG);
            Serial.print("\t");
            Serial.println(targets.motor_torque_ff);
            return;
        }
        if (cli_out_mode == load_side) {

            drvSys_driveTargets targets = drvSys_get_targets();
            drvSys_FullDriveState state = drvSys_get_full_drive_state();

            Serial.print(state.joint_pos * RAD2DEG);
            Serial.print("\t");
            Serial.print(state.joint_vel * RAD2DEG);
            Serial.print("\t");
            Serial.print(state.joint_acc * RAD2DEG);
            Serial.print("\t");
            Serial.print(state.motor_torque);
            Serial.print("\t");
            Serial.print(state.joint_torque);
            Serial.print("\t");
            Serial.print(targets.pos_target * RAD2DEG);
            Serial.print("\t");
            Serial.print(targets.vel_target * RAD2DEG);
            Serial.print("\t");
            Serial.println(targets.motor_torque_ff);


        }
        if (cli_out_mode == load_control) {

            drvSys_driveTargets targets = drvSys_get_targets();
            drvSys_FullDriveState state = drvSys_get_full_drive_state();

            float current_error = drvSys_get_neural_control_error(1, 0);
            float average_error = drvSys_get_neural_control_error(1, 1);

            Serial.print(state.joint_pos * RAD2DEG);
            Serial.print("\t");
            Serial.print(state.joint_vel * RAD2DEG);
            Serial.print("\t");
            Serial.print(state.joint_acc * RAD2DEG);
            Serial.print("\t");
            Serial.print(state.motor_torque);
            Serial.print("\t");
            Serial.print(state.joint_torque);
            Serial.print("\t");
            Serial.print(targets.pos_target * RAD2DEG);
            Serial.print("\t");
            Serial.print(targets.vel_target * RAD2DEG);
            Serial.print("\t");
            Serial.print(targets.motor_torque_ff);
            Serial.print("\t");
            Serial.print(current_error);
            Serial.print("\t");
            Serial.println(average_error);


        }
        if (cli_out_mode == extended) {

            drvSys_driveTargets targets = drvSys_get_targets();
            drvSys_FullDriveState state = drvSys_get_full_drive_state();

            drvSys_controllerCondition cond = drvSys_get_controllerState();


            Serial.print(state.joint_pos * RAD2DEG);
            Serial.print("\t");
            Serial.print(state.joint_vel * RAD2DEG);
            Serial.print("\t");
            Serial.print(state.joint_acc * RAD2DEG);
            Serial.print("\t");
            Serial.print(state.motor_torque);
            Serial.print("\t");
            Serial.print(state.joint_torque);
            Serial.print("\t");
            Serial.print(state.motor_pos * RAD2DEG);
            Serial.print("\t");
            Serial.print(state.motor_vel * RAD2DEG);
            Serial.print("\t");
            Serial.print(state.motor_acc * RAD2DEG);
            Serial.print("\t");
            Serial.print(targets.pos_target * RAD2DEG);
            Serial.print("\t");
            Serial.print(targets.vel_target * RAD2DEG);
            Serial.print("\t");
            Serial.print(targets.motor_torque_ff);
            Serial.print("\t");
            Serial.print(cond.temperature); // Motor Temperature
            Serial.print("\t");
            Serial.println(0); // Hall Sensor


        }

    }
    if (cli_out_mode == tune_pos) {
        drvSys_driveTargets targets = drvSys_get_targets();
        drvSys_FullDriveState state = drvSys_get_full_drive_state();

        Serial.print(state.joint_pos * RAD2DEG);
        Serial.print("\t");
        Serial.println(targets.pos_target * RAD2DEG);

    }
    if (cli_out_mode == tune_vel) {
        drvSys_driveTargets targets = drvSys_get_targets();
        drvSys_FullDriveState state = drvSys_get_full_drive_state();

        Serial.print(state.motor_vel * RAD2DEG);
        Serial.print("\t");
        Serial.print(state.joint_vel * RAD2DEG);
        Serial.print("\t");
        Serial.println(targets.vel_target * RAD2DEG);

    }
    if (cli_out_mode == torque) {
        drvSys_FullDriveState state = drvSys_get_full_drive_state();
        float torque = drvSys_get_torque();
        float raw_torque = drvSys_get_torque(true);
        float delta_angle = drvSys_get_delta_angle();
        Serial.print(torque);
        Serial.print("\t");
        Serial.print(raw_torque);
        Serial.print("\t");
        Serial.print(state.joint_pos * RAD2DEG);
        Serial.print("\t");
        Serial.println(delta_angle * RAD2DEG);

    }


    if (cli_out_mode == nn_inv) {

        drvSys_driveState state = drvSys_get_drive_state();
        float torque_pred = drvSys_neural_control_read_predicted_torque();

        float current_error = drvSys_get_neural_control_error(1, 0);
        float average_error = drvSys_get_neural_control_error(1, 1);

        float pid_torque = drvSys_get_pid_torque();

        Serial.print(state.motor_torque * 100);
        Serial.print("\t");
        Serial.print(torque_pred * 100);
        Serial.print("\t");
        Serial.print(pid_torque * 100);
        Serial.print("\t");
        Serial.print(current_error * 100);
        Serial.print("\t");
        Serial.println(average_error * 100);
    }
    if (cli_out_mode == nn_em) {

        drvSys_driveState state_pred = drvSys_get_emulator_pred();


        const drvSys_driveState actual_state = drvSys_get_drive_state();

        float current_error = drvSys_get_neural_control_error(0, 0);
        float average_error = drvSys_get_neural_control_error(0, 1);

        Serial.print(state_pred.joint_pos * RAD2DEG);
        Serial.print("\t");
        Serial.print(state_pred.joint_vel * RAD2DEG);
        Serial.print("\t");
        Serial.print(state_pred.joint_acc * RAD2DEG);
        Serial.print("\t");
        Serial.print(actual_state.joint_pos * RAD2DEG);
        Serial.print("\t");
        Serial.print(actual_state.joint_vel * RAD2DEG);
        Serial.print("\t");
        Serial.print(actual_state.joint_acc * RAD2DEG);
        Serial.print("\t");
        Serial.print(current_error * 100);
        Serial.print("\t");
        Serial.println(average_error * 100);



    }

    if (cli_out_mode == nn_pid) {

        drvSys_driveTargets targets = drvSys_get_targets();
        drvSys_FullDriveState state = drvSys_get_full_drive_state();

        drvSys_cascade_gains gains = drvSys_get_controller_gains();

        float current_error = drvSys_pid_nn_error(false);
        float average_error = drvSys_pid_nn_error(true);

        Serial.print(state.joint_pos * RAD2DEG);
        Serial.print("\t");
        Serial.print(state.joint_vel * RAD2DEG);
        Serial.print("\t");
        Serial.print(state.joint_acc * RAD2DEG);
        Serial.print("\t");
        Serial.print(state.motor_torque);
        Serial.print("\t");
        Serial.print(state.joint_torque);
        Serial.print("\t");
        Serial.print(targets.pos_target * RAD2DEG);
        Serial.print("\t");
        Serial.print(targets.vel_target * RAD2DEG);
        Serial.print("\t");
        Serial.print(gains.pos_Kp, 8);
        Serial.print("\t");
        Serial.print(gains.vel_Kp, 8);
        Serial.print("\t");
        Serial.print(gains.vel_Ki, 8);
        Serial.print("\t");
        Serial.print(gains.vel_ff_gain, 8);
        Serial.print("\t");
        Serial.print(gains.acc_ff_gain, 8);
        Serial.print("\t");
        Serial.print(current_error, 4);
        Serial.print("\t");
        Serial.println(average_error, 4);

    }
}








