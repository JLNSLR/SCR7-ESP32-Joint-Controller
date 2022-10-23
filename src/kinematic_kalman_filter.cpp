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
    x_pred.Fill(0);
    x_corrected.Fill(0);

    observer_matrix_H.Fill(0);
    observer_matrix_H(0, 0) = 1;
    observer_matrix_H_vel.Fill(0);
    observer_matrix_H_vel(1, 1) = 1;

    z_n.Fill(0);

    n_iterations = 0;


    //initial error covariance_matrix;



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

    system_noise_matrix_prototype = system_noise_matrix;


    float acc_value = accel_change; //DEG

    system_noise_matrix = system_noise_matrix * acc_value * acc_value;



    errorCovariance.Fill(0);
    errorCovariance = system_noise_matrix * 1e2;


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

    sensor_noise = 0.1 * DEG2RAD; // DEG


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

void KinematicKalmanFilter::predictionStep_u(float motor_torque) {
    x_pred = system_matrix_A * x_current; // + b_vec * motor_torque;

    errorCovariance = system_matrix_A * errorCovariance * (~system_matrix_A) + system_noise_matrix;

    x_current = x_pred;
}

void KinematicKalmanFilter::correctionStep() {

    //Estimate Kalman Gain

    float divisor = 1.0 / (errorCovariance(0, 0) + sensor_noise);

    kalman_Gain = errorCovariance * (~observer_matrix_H) * divisor;

    z_n(0) = position_sensor_val;

    x_corrected = x_current + kalman_Gain * (z_n - observer_matrix_H * x_current);

    errorCovariance = (identity_matrix - kalman_Gain * observer_matrix_H) * errorCovariance;

    x_current = x_corrected;

};

void KinematicKalmanFilter::correctionStep_vel(float velocity_derivative) {

    //Estimate Kalman Gain
    float derivative_noise = 3.0;

    float divisor = 1.0 / (errorCovariance(1, 1) + derivative_noise);

    BLA::Matrix<3, 3> observer_matrix_H_vel;
    observer_matrix_H_vel.Fill(0);

    kalman_Gain = errorCovariance * (~observer_matrix_H_vel) * divisor;

    z_n(1) = velocity_derivative;

    x_corrected = x_current + kalman_Gain * (z_n - observer_matrix_H_vel * x_current);

    errorCovariance = (identity_matrix - kalman_Gain * observer_matrix_H_vel) * errorCovariance;

    x_current = x_corrected;

};

KinematicStateVector KinematicKalmanFilter::getEstimatedState() {
    KinematicStateVector state;

    if (n_iterations < 100) {
        x_current(1) = 0;
        x_current(2) = 0;
    }

    state.acc = x_current(2);
    state.vel = x_current(1);
    state.pos = x_current(0);

    return state;

};

KinematicStateVector KinematicKalmanFilter::estimateStates(float position_sensor_val) {

    this->position_sensor_val = position_sensor_val;

    predictionStep();

    correctionStep();

    if (n_iterations <= 100) {
        n_iterations++;
    }


    return getEstimatedState();


};

void KinematicKalmanFilter::setAccelChange(float accel_change) {

    this->accel_change = accel_change;

    system_noise_matrix = system_noise_matrix_prototype * accel_change * accel_change;

}