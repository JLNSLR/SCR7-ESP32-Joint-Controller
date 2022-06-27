#ifndef DRIVE_DYNAMIC_KALMAN_FILTER_H
#define DRIVE_DYNAMIC_KALMAN_FILTER_H


#include <BasicLinearAlgebra.h>
#include <Arduino.h>


#define DEG2RAD 0.01745329251994329576923690768489
#define RAD2DEG 57.295779513082320876798154814105


#define JOINT_POS_ID 0
#define JOINT_VEL_ID 1
#define JOINT_ACC_ID 2
#define MOTOR_POS_ID 3
#define MOTOR_VEL_ID 4
#define MOTOR_ACC_ID 5
#define TORQUE_ID 6
#define INV_INERTIA_JOINT_ID 7
#define INV_INERTIA_MOTOR_ID 8
#define STIFFNESS_ID 9
#define BACKLASH_ANGLE_ID 10
#define VISC_FRIC_ID 11
#define COLOUMB_FRIC_ID 12

#define N_STATES 13
#define DYNAMIC_STATES 7

struct KinematicStateVector {
    float pos;
    float vel;
    float acc;
};

struct DynamicStateVector {
    float joint_pos;
    float joint_vel;
    float joint_acc;

    float motor_pos;
    float motor_vel;
    float motor_acc;

    float joint_torque;
};

struct ExtendedDynamicStateVector {
    float joint_pos;
    float joint_vel;
    float joint_acc;

    float motor_pos;
    float motor_vel;
    float motor_acc;

    float joint_torque;

    float inertia_joint;
    float inertia_motor;
    float stiffness;
    float backlash_angle;
    float viscous_fric_coef;
    float coloumb_fric_coef;

};

struct KinematicNoiseVector {
    float sys_noise_pos;
    float sys_noise_vel;
    float sys_noise_acc;
    float meas_noise_pos;
};

struct DynamicParameters {
    float inertia_joint;
    float inertia_motor;
    float stiffness;
    float backlash_angle;
    float viscous_fric_coef;
    float coloumb_fric_coef;
};

struct backlash_vector {
    float delta_angle;
    bool in_deadzone;
};



class DriveDynamicKalmanFilter {

public:
    DriveDynamicKalmanFilter();
    DriveDynamicKalmanFilter(float delta_t = 250e-6, float transmission_ratio_N = 80);

    void init(float delta_t);

    void predictionStep(float motor_torque);
    void correctionStep_motor_pos(float sensor_motor_pos);
    void correctionStep_joint_pos(float sensor_joint_pos);
    void correctionStep_load_torque(float sensor_torque);

    KinematicStateVector getEstimatedMotorState();
    KinematicStateVector getEstimatedJointState();
    DynamicStateVector getEstimatedState();
    ExtendedDynamicStateVector getEstimatedExtendedState();

    DynamicParameters getParameters();

    bool initialized = false;


    // Fixed Joint Simulation Parameters
    float delta_t = 250e-6;

    float transmission_ratio_N = 80;
    float inv_transmission_ratio_N = 1 / transmission_ratio_N;

    // Initial values of extended States

    float inertia_joint = 0.00025;
    float inertia_motor = 54e-7;

    float inv_inertia_joint = 1.0 / inertia_joint;
    float inv_inertia_motor = 1.0 / inertia_motor;

    float visc_fric_coef = 0.0;
    float coloumb_fric_coef = 0.01 / coloumb_fric_threshold_vel;

    float backlash_angle = 1.0 * DEG2RAD;
    float stiffness = 100.0;



    // Sensor inputs
    float position_sensor_val = 0.0;
    float torque_sensor_val = 0.0;
    float motor_torque_val = 0.0;


    // Sensor Noise values
    float sensor_noise_encoder = 0.05 * DEG2RAD;
    float sensor_noise_torque_sensor = 0.5;



private:

    BLA::Matrix<N_STATES> x_pred;
    BLA::Matrix<N_STATES> x_corrected;

    BLA::Matrix<N_STATES> x_current;

    BLA::Matrix<N_STATES, N_STATES> system_matrix_A;
    BLA::Matrix<N_STATES, N_STATES> kalman_Gain;
    BLA::Matrix<N_STATES, N_STATES> errorCovariance;
    BLA::Matrix<N_STATES, N_STATES> system_noise_matrix;

    BLA::Matrix<N_STATES, N_STATES> observer_matrix_H_motor_pos;
    BLA::Matrix<N_STATES, N_STATES> observer_matrix_H_joint_pos;
    BLA::Matrix<N_STATES, N_STATES> observer_matrix_H_load_torque;

    BLA::Matrix<N_STATES, N_STATES> identity_matrix;

    BLA::Matrix<N_STATES> deterministic_input;

    float coloumb_fric_threshold_vel = 0.05 * DEG2RAD;

    backlash_vector angle_difference_with_backlash(float motor_pos, float joint_angle, float backlash_angle);

    float get_friction_coefficient(float vel, float viscous_fric_coef, float coloumb_fric_coef);


};




#endif // !DRIVE_DYNAMIC_KALMAN_FILTER_H

