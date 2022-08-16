#ifndef KINEMATIC_KALMAN_FILTER_H
#define KINEMATIC_KALMAN_FILTER_H

#include <BasicLinearAlgebra.h>
#include <Arduino.h>


#define DEG2RAD 0.01745329251994329576923690768489
#define RAD2DEG 57.295779513082320876798154814105

struct KinematicStateVector {
    float pos;
    float vel;
    float acc;
};

struct KinematicNoiseVector {
    float sys_noise_pos;
    float sys_noise_vel;
    float sys_noise_acc;
    float meas_noise_pos;
};



class KinematicKalmanFilter {

public:
    KinematicKalmanFilter();
    KinematicKalmanFilter(float delta_t);

    void init(float delta_t);

    void predictionStep();
    void predictionStep_u(float motor_torque);
    void correctionStep();
    void correctionStep_vel(float vel);

    KinematicStateVector getEstimatedState();
    KinematicStateVector estimateStates(float position_sensor_val);
    KinematicStateVector estimateStates(float position_sensor_val, float delta_t);

    KinematicNoiseVector noise;

    void setAccelChange(float accel_change);

    float delta_t = 250e-6;

    float position_sensor_val;

    float accel_change = 5.0 * DEG2RAD;

    float sensor_noise;

private:

    BLA::Matrix<3> x_pred;
    BLA::Matrix<3> x_corrected;

    BLA::Matrix<3> x_current;

    BLA::Matrix<3, 3> system_matrix_A;
    BLA::Matrix<3, 3> kalman_Gain;
    BLA::Matrix<3, 3> errorCovariance;
    BLA::Matrix<3, 3> system_noise_matrix;
    BLA::Matrix<3, 3> system_noise_matrix_prototype;

    BLA::Matrix<3, 3> observer_matrix_H;
    BLA::Matrix<3, 3> observer_matrix_H_vel;

    BLA::Matrix<3, 3> identity_matrix;

    BLA::Matrix<3> b_vec;
    BLA::Matrix<3> z_n;

    int n_iterations = 0;




};


#endif // !KINEMATIC_KALMAN_FILTER_H

