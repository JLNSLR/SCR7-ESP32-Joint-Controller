#ifndef DRIVE_SYS_TYPES_H
#define DRIVE_SYS_TYPES_H

/* Drive System State Flag */
enum drvSys_StateFlag { error, not_ready, ready, closed_loop_control_active, closed_loop_control_inactive };
/*Drive System Control Mode Variables */
enum drvSys_controlMode { direct_torque, closed_loop_foc, admittance_control, stepper_mode };

/* Drive System Priority constants */
enum drvSys_priorities {
    foc_prio = 10, process_sensor_prio = 10, torque_control_prio = 9,
    pid_dual_control_prio = 9, admittance_control_prio = 6, learn_dynamics_prio = 7, stepper_control_prio = 10
};

struct drvSys_PID_Gains {
    float K_p;
    float K_i;
    float K_d;
    float K_vel_ff;

};

struct drvSys_admittance_parameters {
    float virtual_spring;
    float virtual_damping;
    float virtual_inertia;
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
    float pos_target;
    float vel_target;
    float acc_target;
    float motor_torque_ff = 0;
    float ref_torque = 0;

};


struct drvSys_parameters {
    int max_current_mA;
    float max_torque_Nm;
    float max_vel;
    drvSys_PID_Gains pid_gains;
    drvSys_admittance_parameters admittance_gains;
    float limit_high_deg;
    float limit_low_deg;
    bool endStops_enabled;
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
    bool hit_neg_limit;
    bool hit_positive_limit;
    bool overtemperature;
    bool temperature_warning;
    float temperature;
};



#endif//