#include <JCRTL_CLI_interface_functions.h>
#include <drive_system.h>

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

        if (strcmp(cli_arg[1], "notch") == 0) {

            bool active = atoi(cli_arg[2]);

            int filter_id = atoi(cli_arg[3]);

            float frequ = atof(cli_arg[4]);

            float bandwidth = atof(cli_arg[5]);

            if (bandwidth == 0 || frequ == 0) {
                error = true;
            }

            if (filter_id > 1) {

                _jctrl_cli_feedback_output("Notch Filer ID not valid.");

                error = true;
            }
            if (frequ > 1000.0 || frequ < 0) {
                _jctrl_cli_feedback_output("Notch Filer ID not valid.");
                error = true;
            }
            if (bandwidth > 1000) {
                _jctrl_cli_feedback_output("Notch Filter bandwidth not valid");
                error = true;
            }

            if (!error) {
                drvSys_set_notch_filters(filter_id, frequ, bandwidth, active);

                if (active) {
                    _jctrl_cli_feedback_output("Notch Filter activated.");
                }
                else {
                    _jctrl_cli_feedback_output("Notch Filter deactivated.");
                }
                return processed;
            }
        }
        else {

            float target_torque;
            if (strcmp(cli_arg[1], "ff") == 0) {
                target_torque = atof(cli_arg[2]);
            }
            target_torque = atof(cli_arg[1]);


            _jctrl_cli_feedback_output("Torque command: " + String(target_torque) + " Nm.");
            drvSys_set_target_torque(target_torque);
            return processed;
        }
    }

    return processed;

}

bool jctrl_cli_process_velocity_command(char(*cli_arg)[N_MAX_ARGS]) {

    char* keyword = cli_arg[0];

    bool processed = false;
    if (strcmp(keyword, "vel") == 0) {
        processed = true;

        if (strcmp(cli_arg[1], "ff") == 0) {
            float vel_ff = atof(cli_arg[2]);
            drvSys_set_feed_forward_velocity(vel_ff);
        }
        float vel_target = atof(cli_arg[1]);
        drvSys_set_target_velocity(vel_target);

    }

    return processed;



}

bool jctrl_cli_process_position_command(char(*cli_arg)[N_MAX_ARGS]) {

    char* keyword = cli_arg[0];

    bool processed = false;

    if (strcmp(keyword, "pos") == 0) {
        processed = true;
        float pos_target = atof(cli_arg[1]);
        drvSys_set_target_pos(pos_target);

    }
    return processed;

}

bool jctrl_cli_process_pid_command(char(*cli_arg)[N_MAX_ARGS]) {

    char* keyword = cli_arg[0];
    char* keyword_2 = cli_arg[1];
    char* keyword_3 = cli_arg[2];
    bool processed = false;

    bool error = false;

    if (strcmp(keyword, "pid") == 0 || strcmp(keyword, "PID") == 0) {
        processed = true;
        if (strcmp(keyword_2, "pos") == 0) {

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
                    _jctrl_cli_feedback_output("Set position PID gains to: " + String(p_val) + ", " + String(i_val) + ", " + String(d_val) + ".");
                }

                drvSys_set_pos_PID_gains(p_val, i_val, d_val, save);
            }

            if (strcmp(keyword_3, "adv") == 0) {

                bool active = atoi(cli_arg[3]);
                bool filter_alpha = atoi(cli_arg[4]);
                float deadzone = atoi(cli_arg[5]);

                if (filter_alpha < 0 || deadzone < 0 || filter_alpha > 1.0) {
                    error = true;
                    return processed;
                    _jctrl_cli_feedback_output("Invalid PID settings.");
                }

                drvSys_set_advanced_PID_settings(0, filter_alpha, deadzone, active);

            }

        }
        if (strcmp(keyword_2, "vel") == 0) {

            if (strcmp(keyword_3, "gains") == 0 || strcmp(keyword_3, "g") == 0) {

                float p_val = atof(cli_arg[3]);
                float i_val = atof(cli_arg[4]);
                float d_val = atof(cli_arg[5]);

                bool save = false;

                if (strcmp(cli_arg[6], "s") == 0) {
                    save = true;
                    _jctrl_cli_feedback_output("Saved velocity PID gains to: " + String(p_val) + ", " + String(i_val) + ", " + String(d_val) + ".");
                }
                else {
                    _jctrl_cli_feedback_output("Set velocity PID gains to: " + String(p_val) + ", " + String(i_val) + ", " + String(d_val) + ".");
                }

                drvSys_set_vel_PID_gains(p_val, i_val, d_val, save);
                return processed;
            }

            if (strcmp(keyword_3, "adv") == 0) {

                bool active = atoi(cli_arg[3]);
                bool filter_alpha = atoi(cli_arg[4]);
                float deadzone = atoi(cli_arg[5]);

                if (filter_alpha < 0 || deadzone < 0 || filter_alpha > 1.0) {
                    error = true;

                    _jctrl_cli_feedback_output("Invalid PID settings.");
                    return processed;
                }

                drvSys_set_advanced_PID_settings(1, filter_alpha, deadzone, active);
                return processed;

            }

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

        drvSys_controllerState state = drvSys_get_controllerState();

        if (state.state_flag == foc_direct_torque || state.state_flag == closed_loop_control_inactive) {

            if (mode == 0) {
                drvSys_start_motion_control(dual_control);

                _jctrl_cli_feedback_output("Started motion control in dual control mode.");
                return processed;
            }
            if (mode == 1) {
                drvSys_start_motion_control(direct_torque);
                _jctrl_cli_feedback_output("Started motion control in direct torque mode.");
                return processed;
            }
            if (mode == 2) {
                drvSys_start_motion_control(admittance_control);
                _jctrl_cli_feedback_output("Started motion control in admittance control mode.");
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
        if (strcmp(mode, "extended") == 0) {
            cli_output_active = true;
            cli_out_mode = extended;
            return processed;;
        }

        if (strcmp(mode, "stop") == 0) {
            cli_output_active = false;
            return processed;;
        }
        if (strcmp(mode, "tune_vel") == 0) {
            cli_output_active = true;
            cli_out_mode = tune_vel;
            return processed;
        }
        if (strcmp(mode, "tune_pos") == 0) {
            cli_output_active = true;
            cli_out_mode = tune_pos;
            return processed;
        }

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


void _jctrl_cli_output_periodic() {



    if (cli_output_active) {
        if (cli_out_mode == all) {

            drvSys_driveTargets targets = drvSys_get_targets();
            drvSys_extendedDriveState state = drvSys_get_extended_drive_state();

            Serial.print(state.joint_pos);
            Serial.print("\t");
            Serial.print(state.joint_vel);
            Serial.print("\t");
            Serial.print(state.joint_acc);
            Serial.print("\t");
            Serial.print(state.motor_pos);
            Serial.print("\t");
            Serial.print(state.motor_vel);
            Serial.print("\t");
            Serial.print(state.motor_acc);
            Serial.print("\t");
            Serial.print(state.motor_torque);
            Serial.print("\t");
            Serial.print(state.joint_torque);
            Serial.print("\t");
            Serial.print(targets.pos_target);
            Serial.print("\t");
            Serial.print(targets.vel_target);
            Serial.print("\t");
            Serial.println(targets.motor_torque_target);
            return;
        }
        if (cli_out_mode == load_side) {

            drvSys_driveTargets targets = drvSys_get_targets();
            drvSys_extendedDriveState state = drvSys_get_extended_drive_state();

            Serial.print(state.joint_pos);
            Serial.print("\t");
            Serial.print(state.joint_vel);
            Serial.print("\t");
            Serial.print(state.joint_acc);
            Serial.print("\t");
            Serial.print(state.motor_torque);
            Serial.print("\t");
            Serial.print(state.joint_torque);
            Serial.print("\t");
            Serial.print(targets.pos_target);
            Serial.print("\t");
            Serial.print(targets.vel_target);
            Serial.print("\t");
            Serial.println(targets.motor_torque_target);


        }
        if (cli_out_mode == extended) {

            drvSys_driveTargets targets = drvSys_get_targets();
            drvSys_extendedDriveState state = drvSys_get_extended_drive_state();

            Serial.print(state.joint_pos);
            Serial.print("\t");
            Serial.print(state.joint_vel);
            Serial.print("\t");
            Serial.print(state.joint_acc);
            Serial.print("\t");
            Serial.print(state.motor_torque);
            Serial.print("\t");
            Serial.print(state.joint_torque);
            Serial.print("\t");
            Serial.print(state.motor_pos);
            Serial.print("\t");
            Serial.print(state.motor_vel);
            Serial.print("\t");
            Serial.print(state.motor_acc);
            Serial.print("\t");
            Serial.print(targets.pos_target);
            Serial.print("\t");
            Serial.print(targets.vel_target);
            Serial.print("\t");
            Serial.print(targets.motor_torque_target);
            Serial.print("\t");
            Serial.print(0); // Motor Temperature
            Serial.print("\t");
            Serial.println(0); // Hall Sensor


        }

        if (cli_out_mode == tune_vel) {
            drvSys_driveTargets targets = drvSys_get_targets();
            drvSys_extendedDriveState state = drvSys_get_extended_drive_state();

            Serial.print(state.joint_vel);
            Serial.print("\t");
            Serial.print(state.motor_vel * inv_transmission);
            Serial.print("\t");
            Serial.println(targets.vel_target);

        }
        if (cli_out_mode == tune_pos) {
            drvSys_driveTargets targets = drvSys_get_targets();
            drvSys_extendedDriveState state = drvSys_get_extended_drive_state();

            Serial.print(state.joint_pos);
            Serial.print("\t");
            Serial.println(targets.pos_target);

        }
    }



}




