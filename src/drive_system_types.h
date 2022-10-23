#ifndef DRIVE_SYS_TYPES_H
#define DRIVE_SYS_TYPES_H

/* Drive System State Flag */
enum drvSys_StateFlag { error, not_ready, ready, closed_loop_control_active, closed_loop_control_inactive };
/*Drive System Control Mode Variables */
enum drvSys_controlMode { direct_torque, closed_loop_foc, stepper_mode };

/* Drive System Priority constants */
enum drvSys_priorities {
    foc_prio = 10, process_sensor_prio = 10, torque_control_prio = 10,
    pid_dual_control_prio = 10, learn_dynamics_prio = 5, stepper_control_prio = 10, torque_sense_prio = 8, monitor_prio = 7
};

struct drvSys_PID_Gains {
    float K_p;
    float K_i;
    float K_d;

};

struct drvSys_feedforwardGains {
    float vel_ff;
    float acc_ff;
};

struct drvSys_cascade_gains {
    float pos_Kp;
    float pos_Ki;
    float pos_Kd;
    float vel_Kp;
    float vel_Ki;

    float vel_ff_gain;
    float acc_ff_gain;
};


/*#########################################################################
################# Drive System Data Structures ############################
##########################################################################*/

struct drvSys_driveState {
    float joint_pos;
    float joint_vel;
    float joint_acc;
    float joint_torque;
    float motor_torque;
};

struct drvSys_FullDriveState {
    float joint_pos;
    float joint_vel;
    float joint_acc;
    float joint_torque;
    float motor_torque;
    float motor_pos;
    float motor_vel;
    float motor_acc;
};

struct drvSys_FullDriveStateTimeSample {
    drvSys_FullDriveState state;
    drvSys_FullDriveState state_prev;

    float drvSys_feedback_torque;
};

struct drvSys_driveTargets {
    float pos_target = 0;
    float vel_target = 0;
    float acc_target = 0;
    float motor_torque_ff = 0;
};

struct drvSys_driveControlTargets {
    drvSys_driveTargets targets;

    bool position_control = true;
    bool velocity_control = true;
    bool ff_control = true;
};

struct drvSys_parameters {
    int max_current_mA;
    float max_torque_Nm;
    float max_vel;
    float limit_high_rad;
    float limit_low_rad;
    bool endStops_enabled;
    drvSys_cascade_gains gains;
    drvSys_PID_Gains closed_loop_stepper_gains;

};

struct drvSys_notch_filter_params {
    bool notch_active;
    float notch_frequ;
};

extern drvSys_parameters drvSys_parameter_config;

struct drvSys_Constants {
    const int nominal_current_mA;
    const float transmission_ratio;
    const int joint_id;
    const float motor_torque_constant;
};



struct drvSys_controllerCondition {
    enum drvSys_controlMode control_mode;
    enum drvSys_StateFlag state_flag;
    bool calibrated;
    bool position_control;
    bool velocity_control;
    bool feed_forward_control;
    bool hit_neg_limit;
    bool hit_positive_limit;
    bool overtemperature;
    bool temperature_warning;
    float temperature;
    int fan_level;
    bool neural_control_active;
};





#endif//