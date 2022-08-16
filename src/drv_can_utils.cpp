#include <drv_can_utils.h>

#ifdef DRVSYS_VERSION
drvComm_DriveStatePaket drvComm_pack_drive_state(drvSys_driveState state) {

    drvComm_DriveStatePaket paket;

    paket.pos = int16_t(state.joint_pos * pos_to_14bit_data);
    paket.vel = int16_t(state.joint_vel * vel_to_13bit_data);
    paket.acc = int16_t(state.joint_acc * acc_to_13bit_data);
    paket.m_torque = int16_t(state.motor_torque * motor_torque_to_9bit_data);
    paket.joint_torque = int16_t(state.joint_torque * joint_torque_to_15bit_data);

    return paket;
}
#endif


#ifdef DRVSYS_VERSION
drvSys_driveState drvComm_unpack_drive_state(drvComm_DriveStatePaket state_packet) {

    drvSys_driveState state;
    state.joint_pos = float(state_packet.pos) * pos_data_14bit_to_val;
    state.joint_vel = float(state_packet.vel) * vel_data_13bit_to_val;
    state.joint_acc = float(state_packet.acc) * acc_data_13bit_to_val;
    state.motor_torque = float(state_packet.m_torque) * motor_torque_data_9bit_to_val;
    state.joint_torque = float(state_packet.joint_torque) * joint_torque_data_15bit_to_val;

    return state;

}
#endif
drvSys_driveState drvComm_unpack_drive_state(drvComm_DriveStatePaket state_packet, float max_motor_torque) {

    drvSys_driveState state;
    state.joint_pos = float(state_packet.pos) * pos_data_14bit_to_val;
    state.joint_vel = float(state_packet.vel) * vel_data_13bit_to_val;
    state.joint_acc = float(state_packet.acc) * acc_data_13bit_to_val;
    state.motor_torque = float(state_packet.m_torque) * (max_motor_torque / 255.0);
    state.joint_torque = float(state_packet.joint_torque) * joint_torque_data_15bit_to_val;

    return state;

}

#ifdef DRVSYS_VERSION
drvSys_driveTargets drvComm_unpack_traj_command(drvComm_MotionCmd_traj_target targets) {

    drvSys_driveTargets drive_targets;
    drive_targets.pos_target = float(targets.pos) * pos_data_14bit_to_val;
    drive_targets.vel_target = float(targets.vel) * vel_data_13bit_to_val;
    drive_targets.acc_target = float(targets.acc) * acc_data_13bit_to_val;

    drive_targets.motor_torque_ff = float(targets.torque_ff) * motor_torque_data_9bit_to_val;
    drive_targets.ref_torque = float(targets.ref_torque) * joint_torque_data_15bit_to_val;

    return drive_targets;
}
#endif
drvSys_driveTargets drvComm_unpack_traj_command(drvComm_MotionCmd_traj_target targets, float max_motor_torque) {

    drvSys_driveTargets drive_targets;
    drive_targets.pos_target = float(targets.pos) * pos_data_14bit_to_val;
    drive_targets.vel_target = float(targets.vel) * vel_data_13bit_to_val;
    drive_targets.acc_target = float(targets.acc) * acc_data_13bit_to_val;

    drive_targets.motor_torque_ff = float(targets.torque_ff) * (max_motor_torque / 255.0);
    drive_targets.ref_torque = float(targets.ref_torque) * joint_torque_data_15bit_to_val;

    return drive_targets;
}

#ifdef DRVSYS_VERSION
drvComm_MotionCmd_traj_target drvComm_pack_traj_command(drvSys_driveTargets targets) {

    drvComm_MotionCmd_traj_target target_cmd;
    target_cmd.pos = targets.pos_target * pos_to_14bit_data;
    target_cmd.vel = targets.vel_target * vel_to_13bit_data;
    target_cmd.acc = targets.acc_target * acc_to_13bit_data;
    target_cmd.torque_ff = targets.motor_torque_ff * motor_torque_to_9bit_data;
    target_cmd.ref_torque = targets.ref_torque * joint_torque_to_15bit_data;

    return target_cmd;
}
#endif

drvComm_MotionCmd_traj_target drvComm_pack_traj_command(drvSys_driveTargets targets, float max_motor_torque) {
    drvComm_MotionCmd_traj_target target_cmd;
    target_cmd.pos = targets.pos_target * pos_to_14bit_data;
    target_cmd.vel = targets.vel_target * vel_to_13bit_data;
    target_cmd.acc = targets.acc_target * acc_to_13bit_data;
    target_cmd.torque_ff = targets.motor_torque_ff * (255.0 / max_motor_torque);
    target_cmd.ref_torque = targets.ref_torque * joint_torque_to_15bit_data;

    return target_cmd;
}

drvComm_goTo_target drvComm_unpack_go_to_command(drvComm_MotionCmd_goTo_target_packet go_to_packet) {

    drvComm_goTo_target goTo;
    goTo.target_pos = go_to_packet.pos * pos_data_14bit_to_val;
    goTo.vel_max = go_to_packet.vel_max * vel_data_13bit_to_val;
    goTo.acc_max = go_to_packet.acc_max * acc_data_13bit_to_val;

    return goTo;
}

drvComm_MotionCmd_goTo_target_packet drvComm_pack_go_to_command(drvComm_goTo_target goto_target) {
    drvComm_MotionCmd_goTo_target_packet go_to_packet;

    go_to_packet.pos = goto_target.target_pos * pos_to_14bit_data;
    go_to_packet.vel_max = goto_target.vel_max * vel_to_13bit_data;
    go_to_packet.acc_max = goto_target.acc_max * acc_to_13bit_data;

    return go_to_packet;
}
#ifdef DRVSYS_VERSION
float drvComm_unpack_direct_torque_command(drvComm_MotionCmd_direct_motor_torque_target direct_torque) {

    return direct_torque.m_torque_target * motor_torque_data_9bit_to_val;
}
#endif

drvComm_MotionCmd_direct_motor_torque_target drvComm_pack_direct_torque_command(float target_torque, float max_torque) {

    drvComm_MotionCmd_direct_motor_torque_target direct_command;
    direct_command.m_torque_target = int(float(target_torque * (255.0 / max_torque)));

    return direct_command;
}


drvComm_ControllerState drvComm_pack_controllerState(drvSys_controllerCondition controller_state, float max_motor_torque) {
    drvComm_ControllerState state_data;
    state_data.calibrated = int(controller_state.calibrated);
    state_data.mode = int(controller_state.control_mode);
    state_data.stateFlag = int(controller_state.state_flag);
    state_data.overtemperature = int(controller_state.overtemperature);
    state_data.temperature = int(controller_state.temperature * motor_temp_val_to_12bit);
    state_data.overtemp_warn = int(controller_state.temperature_warning);
    state_data.neural_control = int(controller_state.neural_control_active);
    state_data.fan_level = int(controller_state.fan_level);


    if (controller_state.hit_neg_limit) {
        state_data.hit_endstop = 0b01;
    }
    else if (controller_state.hit_positive_limit) {
        state_data.hit_endstop = 0b10;
    }
    else {
        state_data.hit_endstop = 0b00;
    }

    state_data.max_motor_torque = int(max_motor_torque * max_motor_torque_val_to_12bit_data);


    return state_data;
}

drvSys_controllerCondition drvComm_unpack_contollerState(drvComm_ControllerState state_data) {

    drvSys_controllerCondition controller_state;
    controller_state.calibrated = bool(state_data.calibrated);
    controller_state.control_mode = drvSys_controlMode(state_data.mode);
    controller_state.temperature_warning = bool(state_data.overtemperature);
    controller_state.overtemperature = bool(state_data.overtemperature);
    controller_state.temperature = motor_temp_12bit_to_val * state_data.temperature;
    controller_state.overtemperature = bool(state_data.overtemp_warn);
    controller_state.neural_control_active = bool(state_data.neural_control);
    controller_state.fan_level = state_data.fan_level;

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

drvComm_controllerCmd drvComm_gen_controller_command(bool start, bool stop, drvSys_controlMode control_mode) {

    drvComm_controllerCmd controller_cmd;

    controller_cmd.mode = control_mode;
    controller_cmd.start = start;
    controller_cmd.stop = stop;

    return controller_cmd;
}


drvSys_PID_Gains drvComm_unpack_pid_packet(drvComm_PID_Gains_Paket pid_packet) {

    drvSys_PID_Gains gains;
    gains.K_p = pid_packet.p * pid_gain_data_20bit_to_val;
    gains.K_i = pid_packet.i * pid_gain_data_20bit_to_val;
    gains.K_d = pid_packet.d * pid_gain_data_20bit_to_val;

    return gains;
}

drvComm_PID_Gains_Paket drvComm_pack_pid_packet(drvSys_PID_Gains gains, bool controller_type) {


    drvComm_PID_Gains_Paket paket;
    paket.type = controller_type;
    paket.p = gains.K_p * pid_gain_val_to_20bit;
    paket.i = gains.K_i * pid_gain_val_to_20bit;
    paket.d = gains.K_d * pid_gain_val_to_20bit;

    return paket;
}