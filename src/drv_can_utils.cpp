#include <drv_can_utils.h>
#include <math.h>


int32_t convert_float_to_integer(const unsigned int n_bits, const float max_val, const float value) {

    const float conversion_factor = powf(2.0, n_bits - 1) / max_val;

    return  value * conversion_factor;

}

float convert_integer_to_float(const unsigned int n_bits, const float max_val, const int32_t value) {

    const float conversion_factor = max_val / powf(2.0, n_bits);

    return value * conversion_factor;
}

drvComm_DriveStatePacket drvComm_pack_drive_state(drvSys_driveState state, const int joint_id) {

    drvComm_DriveStatePacket packet;

    packet.pos = int16_t(state.joint_pos * pos_to_14bit_data);
    packet.vel = int16_t(state.joint_vel * vel_to_14bit_data);
    packet.acc = int16_t(state.joint_acc * acc_to_13bit_data);
    packet.m_torque = int16_t(state.motor_torque * float(1.0 / motor_torque_conversion_factor_arr_9bit[joint_id]));

    return packet;
}

drvComm_loadStatePacket  drvComm_pack_load_state(float torque_sensor_val, float delta_pos, bool calibrated, const int joint_id) {

    drvComm_loadStatePacket packet;

    if (calibrated) {
        packet.torque_sensor_val = torque_sensor_val * sensor_torque_conversion_factor_24bit[joint_id];
    }
    else {
        packet.torque_sensor_val = torque_sensor_val;
    }

    packet.position_delta = delta_pos * pos_to_14bit_data;
    packet.torque_sensor_calibrated = int(calibrated);

    return packet;
}

drvComm_load_state drvComm_unpack_load_state(drvComm_loadStatePacket paket, int joint_id) {

    drvComm_load_state data;
    data.torque_sensor_val_Nm = paket.torque_sensor_val * (1.0 / sensor_torque_conversion_factor_24bit[joint_id]);
    data.torque_sensor_raw = paket.torque_sensor_val;
    data.calibrated = paket.torque_sensor_calibrated;

    return data;
}



drvSys_driveState drvComm_unpack_drive_state(drvComm_DriveStatePacket state_packet, const int joint_id) {

    drvSys_driveState state;
    state.joint_pos = float(state_packet.pos) * pos_data_14bit_to_val;
    state.joint_vel = float(state_packet.vel) * vel_data_14bit_to_val;
    state.joint_acc = float(state_packet.acc) * acc_data_13bit_to_val;
    state.motor_torque = float(state_packet.m_torque) * motor_torque_conversion_factor_arr_9bit[joint_id];

    return state;

}

drvSys_driveControlTargets drvComm_unpack_traj_command(drvComm_MotionCmd_traj_target targets, const int joint_id) {

    drvSys_driveControlTargets drive_control_targets;

    drvSys_driveTargets drive_targets;
    drive_targets.pos_target = float(targets.pos) * pos_data_14bit_to_val;
    drive_targets.vel_target = float(targets.vel) * vel_data_14bit_to_val;
    drive_targets.acc_target = float(targets.acc) * acc_data_13bit_to_val;


    drive_targets.motor_torque_ff = float(targets.torque_ff) * motor_torque_conversion_factor_arr_9bit[joint_id];



    drive_control_targets.ff_control = targets.feedforward;
    drive_control_targets.position_control = targets.position_control;
    drive_control_targets.velocity_control = targets.velocity_control;

    drive_control_targets.targets = drive_targets;


    return drive_control_targets;



}

drvComm_MotionCmd_traj_target drvComm_pack_traj_command(drvSys_driveTargets targets, const int joint_id) {

    drvComm_MotionCmd_traj_target target_cmd;
    target_cmd.pos = targets.pos_target * pos_to_14bit_data;
    target_cmd.vel = targets.vel_target * vel_to_14bit_data;
    target_cmd.acc = targets.acc_target * acc_to_13bit_data;
    target_cmd.torque_ff = targets.motor_torque_ff * (1.0 / motor_torque_conversion_factor_arr_9bit[joint_id]);

    return target_cmd;
}



drvComm_goTo_target drvComm_unpack_go_to_command(drvComm_MotionCmd_goTo_target_packet go_to_packet) {

    drvComm_goTo_target goTo;
    goTo.target_pos = go_to_packet.pos * pos_data_14bit_to_val;
    goTo.vel_max = go_to_packet.vel_max * vel_data_14bit_to_val;
    goTo.acc_max = go_to_packet.acc_max * acc_data_13bit_to_val;

    return goTo;
}

drvComm_MotionCmd_goTo_target_packet drvComm_pack_go_to_command(drvComm_goTo_target goto_target) {
    drvComm_MotionCmd_goTo_target_packet go_to_packet;

    go_to_packet.pos = goto_target.target_pos * pos_to_14bit_data;
    go_to_packet.vel_max = goto_target.vel_max * vel_to_14bit_data;
    go_to_packet.acc_max = goto_target.acc_max * acc_to_13bit_data;

    return go_to_packet;
}

float drvComm_unpack_direct_torque_command(drvComm_MotionCmd_direct_motor_torque_target direct_torque, const int joint_id) {

    return direct_torque.m_torque_target * motor_torque_conversion_factor_arr_9bit[joint_id];
}

drvComm_MotionCmd_direct_motor_torque_target drvComm_pack_direct_torque_command(float motor_torque, const int joint_id) {
    drvComm_MotionCmd_direct_motor_torque_target direct_motor_torque_msg;
    direct_motor_torque_msg.m_torque_target = motor_torque * (1.0 / float(motor_torque_conversion_factor_arr_9bit[joint_id]));

    return direct_motor_torque_msg;
}




drvComm_ControllerState drvComm_pack_controllerState(drvSys_controllerCondition controller_state, const int joint_id, float torque_limit) {
    drvComm_ControllerState state_data;
    state_data.calibrated = int(controller_state.calibrated);
    state_data.mode = int(controller_state.control_mode);
    state_data.stateFlag = int(controller_state.state_flag);
    state_data.overtemperature = int(controller_state.overtemperature);
    state_data.temperature = int(controller_state.temperature * motor_temp_val_to_12bit);
    state_data.overtemp_warn = int(controller_state.temperature_warning);
    state_data.neural_control = int(controller_state.neural_control_active);
    state_data.fan_level = int(controller_state.fan_level);
    state_data.feed_forward_control = int(controller_state.feed_forward_control);
    state_data.position_control = int(controller_state.position_control);


    if (controller_state.hit_neg_limit) {
        state_data.hit_endstop = 0b01;
    }
    else if (controller_state.hit_positive_limit) {
        state_data.hit_endstop = 0b10;
    }
    else {
        state_data.hit_endstop = 0b00;
    }

    state_data.torque_limit = uint8_t(torque_limit * (1.0 / motor_torque_conversion_factor_arr_9bit[joint_id]));


    return state_data;
}

drvSys_controllerCondition drvComm_unpack_controllerState(drvComm_ControllerState state_data, const int joint_id) {

    drvSys_controllerCondition controller_state;
    controller_state.calibrated = bool(state_data.calibrated);
    controller_state.control_mode = drvSys_controlMode(state_data.mode);
    controller_state.temperature_warning = bool(state_data.overtemp_warn);
    controller_state.overtemperature = bool(state_data.overtemperature);
    controller_state.temperature = motor_temp_12bit_to_val * state_data.temperature;
    controller_state.neural_control_active = bool(state_data.neural_control);
    controller_state.fan_level = state_data.fan_level;
    controller_state.position_control = state_data.position_control;
    controller_state.feed_forward_control = state_data.feed_forward_control;


    if (state_data.hit_endstop == 0b00) {
        controller_state.hit_neg_limit = false;
        controller_state.hit_positive_limit = false;
    }
    else if (state_data.hit_endstop == 0b10) {
        controller_state.hit_positive_limit = true;
        controller_state.hit_neg_limit = false;
    }
    else if (state_data.hit_endstop == 0b01) {
        controller_state.hit_positive_limit = false;
        controller_state.hit_neg_limit = true;
    }

    return controller_state;
}

drvComm_modeCmd drvComm_gen_controller_command(bool start, bool stop, drvSys_controlMode control_mode) {

    drvComm_modeCmd controller_cmd;

    controller_cmd.mode = control_mode;
    controller_cmd.start = start;
    controller_cmd.stop = stop;

    return controller_cmd;
}


//PID Parameter update
drvComm_gain_cmd drvComm_unpack_gain_packet(drvComm_gains_Packet gains_packet) {

    drvComm_gain_cmd gains;
    gains.type = gains_packet.type;
    gains.val0 = gains_packet.val0 * gain_data_16bit_to_val;
    gains.val1 = gains_packet.val1 * gain_data_16bit_to_val;
    gains.val2 = gains_packet.val2 * gain_data_16bit_to_val;

    return gains;
}
drvComm_gains_Packet drvComm_pack_gain_packet(drvComm_gain_cmd gains) {

    drvComm_gains_Packet gains_packet;
    gains_packet.type = gains.type;

    gains_packet.val0 = gains.val0 * gain_val_to_16bit;
    gains_packet.val1 = gains.val1 * gain_val_to_16bit;
    gains_packet.val2 = gains.val2 * gain_val_to_16bit;

    return gains_packet;

}


drvComm_LightCmd drvComm_gen_light_command(bool rgb, uint8_t rh, uint8_t gs, uint8_t bv, int period_ms) {

    drvComm_LightCmd cmd_paket;
    cmd_paket.rgb_hsv = rgb;
    cmd_paket.rh = rh;
    cmd_paket.gs = gs;
    cmd_paket.bv = bv;
    cmd_paket.period_ms = period_ms;

    return cmd_paket;
}

// Parameter command generation
drvComm_paramsCmd drvComm_gen_params_command(drvComm_parameterType type, float param_value) {
    drvComm_paramsCmd cmd_paket;
    cmd_paket.type = type;

    portable_float_type parameter;
    parameter.value = param_value;

    cmd_paket.param_data = parameter.binary;



    return cmd_paket;

}