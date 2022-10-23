#include <joint_can_handler.h>

TaskHandle_t can_task_th;

const int can_sys_id = 0;
const int can_joint_id = JOINT_ID;


void can_init() {


    Serial.println("JCTRL_INFO: Init CAN Communication.");
    CAN.setPins(CAN_RX, CAN_TX);

    if (!CAN.begin(1000E3)) {
        Serial.println("JCTRL_INFO: Starting CAN failed!");
    }
    else {
        Serial.println("JCTRL_INFO: Starting CAN successful!");
    }

    union {
        struct {
            int sys : 3;
            int joint_id : 4;
            int msg_type : 4;
        };
        int val;
    }filter_id;

    filter_id.sys = 0;
    filter_id.joint_id = can_joint_id;
    filter_id.msg_type = 0;


    union {
        struct {
            int sys : 3;
            int joint_id : 4;
            int msg_type : 4;
        };
        int val;
    }mask;

    mask.sys = 0b111;
    mask.joint_id = 0xF;
    mask.msg_type = 0x0;


    CAN.filter(filter_id.val, mask.val);



};

void can_start_interface() {

    xTaskCreate(
        can_handle_task,   // function name
        "can_handle_task", // task name
        CAN_HANDLER_STACK_SIZE,      // Stack size (bytes)
        NULL,      // task parameters
        CAN_HANDLER_PRIO,         // task priority
        &can_task_th
        // task handle
    );

    Serial.println("CAN_INTERFACE: Started CAN Interface");
}


void can_handle_task(void* params) {

    const TickType_t can_delay = CAN_PERIOD_MS / portTICK_PERIOD_MS;

    static long counter = 0;

    while (true) {
        Serial.println("doing can things");
        if (counter % can_drive_state_divier == 0) { // 500 Hz
            can_send_drive_state(drvSys_get_full_drive_state());
        }

        if (counter % can_ctrl_state_divier == 0) { // 10 Hz
            can_send_controller_state(drvSys_get_controllerState());
        }

        can_read_incoming_CAN_commands();

        counter++;



        vTaskDelay(can_delay);
    }
}

void can_send_controller_state(drvSys_controllerCondition controller_state) {

    float max_motor_torque = drvSys_get_parameters().max_torque_Nm;
    drvComm_ControllerState state_data = drvComm_pack_controllerState(controller_state, can_joint_id, max_motor_torque);

    union {
        drvComm_ControllerState state_data;
        uint8_t bytes[DRVCOMM_CONTROLLER_STATE_LENGTH];
    }state_data_packed;

    state_data_packed.state_data = state_data;


    drvComm_CANID can_id;
    can_id.joint_id = can_joint_id;
    can_id.sys_id = 0;
    can_id.msg_type = controller_state_msg;

    CAN.beginPacket(can_id.msg_id);
    CAN.write(state_data_packed.bytes, DRVCOMM_CONTROLLER_STATE_LENGTH);
    CAN.endPacket();
}

void can_send_drive_state(drvSys_FullDriveState drive_state) {

    // Send Drive Data

    drvSys_driveState drive_state_output;
    drive_state_output.joint_pos = drive_state_output.joint_pos;
    drive_state_output.joint_vel = drive_state_output.joint_vel;
    drive_state_output.joint_acc = drive_state_output.joint_acc;
    drive_state_output.motor_torque = drive_state_output.motor_torque;

    drvComm_DriveStatePacket drive_state_data = drvComm_pack_drive_state(drive_state_output, can_joint_id);

    union {
        drvComm_DriveStatePacket data;
        uint8_t bytes[DRVCOMM_DRIVE_STATE_LENGTH];
    }drive_state_data_packed;


    drive_state_data_packed.data = drive_state_data;


    drvComm_CANID can_id;
    can_id.joint_id = can_joint_id;
    can_id.sys_id = 0;
    can_id.msg_type = drive_state_msg;

    CAN.beginPacket(can_id.msg_id);
    CAN.write(drive_state_data_packed.bytes, DRVCOMM_DRIVE_STATE_LENGTH);
    CAN.endPacket();


    // Send Load Data

    bool torque_calibrated = drvSys_get_torqueSensor().calibrated;

    float torque_sensor_val;

    if (torque_calibrated) {
        torque_sensor_val = drive_state.joint_torque;
    }
    else {
        torque_sensor_val = drvSys_get_torqueSensor().raw_sensor_val;
    }


    drvComm_loadStatePacket load_state_msg = drvComm_pack_load_state(torque_sensor_val,
        drive_state.joint_pos - drive_state.motor_pos, torque_calibrated, can_joint_id);

    union {
        drvComm_loadStatePacket data;
        uint8_t bytes[DRVCOMM_LOAD_MSG_LENGTH];
    }load_state_msg_packed;

    drvComm_CANID can_id_load = can_id;
    can_id_load.msg_type = drive_load_state_msg;

    CAN.beginPacket(can_id_load.msg_id);
    CAN.write(load_state_msg_packed.bytes, DRVCOMM_DRIVE_STATE_LENGTH);
    CAN.endPacket();


}


/*
void can_send_pid_data(drvSys_parameters params) {

    drvComm_PID_Gains_Paket paket_0;
    paket_0 = drvComm_pack_gain_packet(params.pos_pid_gains, true);

    drvComm_CANID can_id;
    can_id.joint_id = JOINT_ID;
    can_id.sys_id = 0;
    can_id.msg_type = drive_pid_data;

    union {
        drvComm_PID_Gains_Paket data;
        uint8_t bytes[DRVCOMM_GAIN_LENGTH];
    }data_packed;

    CAN.beginPacket(can_id.msg_id);
    CAN.write(data_packed.bytes, DRVCOMM_GAIN_LENGTH);
    CAN.endPacket();

    paket_0 = drvComm_pack_gain_packet(params.vel_pid_gains, false);

    data_packed.data = paket_0;

    CAN.beginPacket(can_id.msg_id);
    CAN.write(data_packed.bytes, DRVCOMM_GAIN_LENGTH);
    CAN.endPacket();

}
*/

void can_read_incoming_CAN_commands() {

    int packet_size = CAN.parsePacket();

    Serial.println(packet_size);

    if (packet_size) {

        drvComm_CANID can_id;
        can_id.msg_id = CAN.packetId();

        if (can_id.joint_id == JOINT_ID) {

            if (can_id.msg_type == drive_traj_target) {

                union {
                    drvComm_MotionCmd_traj_target traj_cmd;
                    uint8_t bytes[DRVCOMM_TRAJ_CMD_LENGTH];
                }input_paket;

                CAN.readBytes(input_paket.bytes, size_t(DRVCOMM_TRAJ_CMD_LENGTH));

                drvSys_driveControlTargets drive_control_target = drvComm_unpack_traj_command(input_paket.traj_cmd, can_joint_id);


                Serial.print("Parsed Drive Target Traj command ");
                Serial.print(drive_control_target.targets.pos_target);
                Serial.print(" ");
                Serial.print(drive_control_target.targets.vel_target);
                Serial.print(" ");
                Serial.print(drive_control_target.targets.acc_target);
                Serial.print(" ");
                Serial.println(drive_control_target.targets.motor_torque_ff);


                handle_motion_control_command(drive_control_target);

            }

            if (can_id.msg_type == go_to_target) {

                union {
                    drvComm_MotionCmd_goTo_target_packet go_to_cmd;
                    uint8_t bytes[DRVCOMM_GOTO_CMD_LENGTH];
                }input_paket;

                CAN.readBytes(input_paket.bytes, size_t(DRVCOMM_GOTO_CMD_LENGTH));

                drvComm_goTo_target goto_target = drvComm_unpack_go_to_command(input_paket.go_to_cmd);

                Serial.println("Parsed Drive go to command");

                handle_motion_command(goto_target.target_pos, goto_target.vel_max, goto_target.acc_max);


            }

            if (can_id.msg_type == mode_command) {


                union {
                    drvComm_modeCmd controller_cmd;
                    uint8_t bytes[DRVCOMM_CONTROLLER_CMD_LENGTH];
                }input_packet;


                CAN.readBytes(input_packet.bytes, size_t(DRVCOMM_CONTROLLER_CMD_LENGTH));


                if (input_packet.controller_cmd.start) {
                    drvSys_start_motion_control(input_packet.controller_cmd.mode);
                }
                else if (input_packet.controller_cmd.stop) {
                    drvSys_stop_controllers();
                }


            }

            if (can_id.msg_type == drive_direct_torque_command) {

                union {
                    drvComm_MotionCmd_direct_motor_torque_target direct_cmd;
                    uint8_t bytes[DRVCOMM_DIRECT_TORQUE_CMD_LENGTH];
                }input_packet;

                CAN.readBytes(input_packet.bytes, size_t(DRVCOMM_DIRECT_TORQUE_CMD_LENGTH));

                float target_torque = drvComm_unpack_direct_torque_command(input_packet.direct_cmd, can_joint_id);

                if (drvSys_get_controllerState().control_mode == direct_torque) {
                    _drvSys_set_target_torque(target_torque);
                }

            }

            if (can_id.msg_type == drive_gain_data) {
                union {
                    drvComm_gains_Packet gains_packet;
                    uint8_t bytes[DRVCOMM_GAIN_LENGTH];
                }input_packet;

                CAN.readBytes(input_packet.bytes, size_t(DRVCOMM_GAIN_LENGTH));

                drvComm_gain_cmd gains = drvComm_unpack_gain_packet(input_packet.gains_packet);

                if (gains.type == pos_gain) {
                    // Position PID gains

                    drvSys_set_PID_gains(true, gains.val0, gains.val1, gains.val2);

                }
                else if (gains.type == vel_gain) {
                    // Velocity PID gains
                    drvSys_set_PID_gains(false, gains.val0, gains.val1, gains.val2);
                }
                else if (gains.type == ff_gain) {
                    drvSys_set_ff_gains(gains.val0, gains.val1);
                }

            }

            if (can_id.msg_type == drive_sys_light_command) {
                union {
                    drvComm_LightCmd light_cmd;
                    uint8_t bytes[DRVCOMM_LIGHT_CMD_LENGTH];
                }input_packet;

                CAN.readBytes(input_packet.bytes, size_t(DRVCOMM_LIGHT_CMD_LENGTH));

                bool rgb = input_packet.light_cmd.rgb_hsv;

                set_leds(input_packet.light_cmd.rh, input_packet.light_cmd.gs, input_packet.light_cmd.rh, rgb, input_packet.light_cmd.period_ms);

            }

            if (can_id.msg_type == drive_param_command) {

                union {
                    drvComm_paramsCmd param_cmd;
                    uint8_t bytes[DRVCOMM_PARAMS_CMD_LENGTH];
                }input_packet;

                CAN.readBytes(input_packet.bytes, size_t(DRVCOMM_PARAMS_CMD_LENGTH));

                can_parse_parameter_command(input_packet.param_cmd);
            }

            if (can_id.msg_type == drive_controller_command) {

                union {
                    drvComm_controllerCmd controller_cmd;
                    uint8_t bytes[DRVCOMM_CONTROLLER_CMD_LENGTH];
                }input_packet;

                CAN.readBytes(input_packet.bytes, size_t(DRVCOMM_CONTROLLER_CMD_LENGTH));

                can_parse_controller_command(input_packet.controller_cmd);
            }

        }



    }
}

void can_parse_parameter_command(drvComm_paramsCmd param_cmd) {

    // Limits
    if (param_cmd.type == torque_limit) {

        portable_float_type parameter;
        parameter.binary = param_cmd.param_data;
        float torque_limit = parameter.value;
        drvSys_limit_torque(torque_limit);
    }

    if (param_cmd.type == pos_limit_high) {

        portable_float_type parameter;
        parameter.binary = param_cmd.param_data;

        drvSys_get_parameters().limit_high_rad = parameter.value;;
    }
    if (param_cmd.type == pos_limit_low) {

        portable_float_type parameter;
        parameter.binary = param_cmd.param_data;
        drvSys_get_parameters().limit_low_rad = parameter.value;
    }

    if (param_cmd.type == max_vel) {
        portable_float_type parameter;
        parameter.binary = param_cmd.param_data;

        drvSys_get_parameters().max_vel = parameter.value;
    }


    // Torque Sensor Parameters
    if (param_cmd.type == torque_sensor_int_offset) {
        portable_float_type parameter;
        parameter.binary = param_cmd.param_data;

        //drvSys_get_torqueSensor().internal_offset = parameter.value;
        drvSys_get_torqueSensor().set_save_calibration_data(1, parameter.value);

    }
    if (param_cmd.type == torque_sensor_ext_offset) {
        portable_float_type parameter;
        parameter.binary = param_cmd.param_data;

        //drvSys_get_torqueSensor().external_offset = parameter.value;
        drvSys_get_torqueSensor().set_save_calibration_data(2, parameter.value);

    }
    if (param_cmd.type == torque_sensor_conversion_factor) {
        portable_float_type parameter;
        parameter.binary = param_cmd.param_data;

        //drvSys_get_torqueSensor().conversion_factor = parameter.value;
        drvSys_get_torqueSensor().set_save_calibration_data(0, parameter.value);
    }

    //Neural Control

    if (param_cmd.type == nn_pid_learning_rate) {
        portable_float_type parameter;
        parameter.binary = param_cmd.param_data;

        drvSys_neural_PID_settings(1, parameter.value);
    }

    if (param_cmd.type == nn_pid_max_learning_rate) {
        portable_float_type parameter;
        parameter.binary = param_cmd.param_data;

        drvSys_neural_PID_settings(2, parameter.value);
    }

    if (param_cmd.type == nn_pid_gain_penalty) {
        portable_float_type parameter;
        parameter.binary = param_cmd.param_data;

        drvSys_neural_PID_settings(0, parameter.value);
    }






}
void can_parse_controller_command(drvComm_controllerCmd controller_cmd) {

    if (controller_cmd.type == reset) {
        ESP.restart();
    }

    if (controller_cmd.type == delete_calibration) {
        drvSys_reset_encoder_offset_data_on_Flash();
    }

    if (controller_cmd.type == save_nn) {
        drvSys_neural_control_save_nets();
    }

    if (controller_cmd.type == reset_nn) {
        drvSys_neural_control_save_nets(true); // resets nets
    }

    if (controller_cmd.type == set_torque_sensor_calibrated) {
        drvSys_get_torqueSensor().calibrated = true;
    }
    if (controller_cmd.type == reset_torque_sensor) {
        drvSys_get_torqueSensor().calibrated = false;
        drvSys_get_torqueSensor().internal_offset = 0;
        drvSys_get_torqueSensor().external_offset = 0;
    }

    if (controller_cmd.type == set_nn_ff_active) {

        drvSys_set_ff_control(true);
    }

    if (controller_cmd.type == set_nn_ff_inactive) {
        drvSys_set_ff_control(false);
    }

    if (controller_cmd.type == set_position_control_active) {
        drvSys_set_position_control(true);
    }
    if (controller_cmd.type == set_position_control_inactive) {
        drvSys_set_position_control(false);
    }



}