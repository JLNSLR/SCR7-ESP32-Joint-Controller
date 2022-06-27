#include "kinematic_kalman_filter.h"


KinematicKalmanFilter::KinematicKalmanFilter() {


};

KinematicKalmanFilter::KinematicKalmanFilter(float delta_t) {
    this->delta_t = delta_t;

    KinematicKalmanFilter();
};

void KinematicKalmanFilter::init(float delta_t) {


    this->delta_t = delta_t;
    //setup system matrix
    system_matrix_A.Fill(0);
    system_matrix_A(0, 0) = 1;
    system_matrix_A(0, 1) = delta_t;
    system_matrix_A(0, 2) = 0.5 * delta_t * delta_t;
    system_matrix_A(1, 1) = 1;
    system_matrix_A(1, 2) = delta_t;
    system_matrix_A(2, 2) = 1;

    x_current.Fill(0);

    observer_matrix_H.Fill(0);
    observer_matrix_H(0, 0) = 1;


    //initial error covariance_matrix;

    errorCovariance.Fill(0);
    errorCovariance(0, 0) = 1e-2;
    errorCovariance(1, 1) = 1;
    errorCovariance(2, 2) = 10;


    // System_noise matrix


    // assume piece white noise model for acceleration


    system_noise_matrix(0, 0) = delta_t * delta_t * delta_t * delta_t / 4.0;
    system_noise_matrix(0, 1) = delta_t * delta_t * delta_t / 2.0;
    system_noise_matrix(0, 2) = delta_t * delta_t / 2.0;

    system_noise_matrix(1, 0) = delta_t * delta_t * delta_t / 2.0;
    system_noise_matrix(1, 1) = delta_t * delta_t;
    system_noise_matrix(1, 2) = delta_t;

    system_noise_matrix(2, 0) = delta_t * delta_t / 2.0;
    system_noise_matrix(2, 1) = delta_t;
    system_noise_matrix(2, 2) = 1;


    float acc_value = accel_change; //DEG

    system_noise_matrix = system_noise_matrix * acc_value * acc_value;


    /*

    system_noise_matrix.Fill(0);
    system_noise_matrix(0, 0) = 1e-3;
    system_noise_matrix(1, 1) = 5 * DEG2RAD;
    system_noise_matrix(2, 2) = 100 * DEG2RAD;
    */



    identity_matrix.Fill(0);
    identity_matrix(0, 0) = 1;
    identity_matrix(1, 1) = 1;
    identity_matrix(2, 2) = 1;

    sensor_noise = 0.05;


    /*
    b_vec.Fill(0);
    b_vec(3) = 1 / inertia;

    // add damping

    system_matrix_A(2, 1) = -0.1 / inertia;
    */

}


void KinematicKalmanFilter::predictionStep() {

    x_pred = system_matrix_A * x_current;

    errorCovariance = system_matrix_A * errorCovariance * (~system_matrix_A) + system_noise_matrix;

    x_current = x_pred;

};

void KinematicKalmanFilter::predictionStep(float motor_torque) {
    x_pred = system_matrix_A * x_current; // + b_vec * motor_torque;

    errorCovariance = system_matrix_A * errorCovariance * (~system_matrix_A) + system_noise_matrix;

    x_current = x_pred;
}

void KinematicKalmanFilter::correctionStep() {

    //Estimate Kalman Gain

    float divisor = 1.0 / (errorCovariance(0, 0) + sensor_noise);

    kalman_Gain = errorCovariance * (~observer_matrix_H) * divisor;

    BLA::Matrix<3> z_n;
    z_n.Fill(0);
    z_n(0) = position_sensor_val;

    x_corrected = x_current + kalman_Gain * (z_n - observer_matrix_H * x_current);

    errorCovariance = (identity_matrix - kalman_Gain * observer_matrix_H) * errorCovariance;

    x_current = x_corrected;

};

KinematicStateVector KinematicKalmanFilter::getEstimatedState() {
    KinematicStateVector state;

    state.acc = x_current(2);
    state.vel = x_current(1);
    state.pos = x_current(0);

    return state;

};

KinematicStateVector KinematicKalmanFilter::estimateStates(float position_sensor_val) {

    this->position_sensor_val = position_sensor_val;

    predictionStep();

    correctionStep();

    return getEstimatedState();


};