#include "drive_dynamic_kalman_filter.h"


DriveDynamicKalmanFilter::DriveDynamicKalmanFilter() {


};

DriveDynamicKalmanFilter::DriveDynamicKalmanFilter(float delta_t, float transmission_ratio_N) {
    this->delta_t = delta_t;
    this->transmission_ratio_N = transmission_ratio_N;
    this->inv_transmission_ratio_N = 1.0 / transmission_ratio_N;

};

void DriveDynamicKalmanFilter::init(float delta_t) {


    this->delta_t = delta_t;
    //setup system matrix
    system_matrix_A.Fill(0);

    // initialize as Unit Matrix
    for (int i = 0; i < N_STATES; i++) {
        system_matrix_A(i, i) = 1.0;
    }

    // set up Kinematic relations
    system_matrix_A(JOINT_POS_ID, JOINT_POS_ID) = 1;
    system_matrix_A(JOINT_POS_ID, JOINT_VEL_ID) = delta_t;
    system_matrix_A(JOINT_POS_ID, JOINT_ACC_ID) = 0.5 * delta_t * delta_t;
    system_matrix_A(JOINT_VEL_ID, JOINT_VEL_ID) = 1;
    system_matrix_A(JOINT_VEL_ID, JOINT_ACC_ID) = delta_t;

    system_matrix_A(MOTOR_POS_ID, MOTOR_POS_ID) = 1;
    system_matrix_A(MOTOR_POS_ID, MOTOR_VEL_ID) = delta_t;
    system_matrix_A(MOTOR_POS_ID, MOTOR_ACC_ID) = 0.5 * delta_t * delta_t;
    system_matrix_A(MOTOR_VEL_ID, MOTOR_VEL_ID) = 1;
    system_matrix_A(MOTOR_VEL_ID, MOTOR_ACC_ID) = delta_t;


    // setup acceleration dynamic models with backlash
    backlash_vector backlash = angle_difference_with_backlash(x_current(MOTOR_POS_ID), x_current(JOINT_POS_ID), backlash_angle);

    float torsion_factor = 1.0;

    if (backlash.in_deadzone) {
        torsion_factor = 0.0;
    }

    system_matrix_A(JOINT_ACC_ID, JOINT_POS_ID) = torsion_factor * stiffness * inv_inertia_joint;
    system_matrix_A(JOINT_ACC_ID, MOTOR_POS_ID) = -torsion_factor * stiffness * inv_transmission_ratio_N * inv_inertia_joint;

    system_matrix_A(MOTOR_ACC_ID, MOTOR_POS_ID) = inv_inertia_motor * torsion_factor * stiffness * inv_transmission_ratio_N * inv_transmission_ratio_N;
    system_matrix_A(MOTOR_ACC_ID, JOINT_POS_ID) = -torsion_factor * inv_inertia_motor * stiffness * inv_transmission_ratio_N;

    system_matrix_A(JOINT_ACC_ID, TORQUE_ID) = -inv_inertia_joint;
    system_matrix_A(MOTOR_ACC_ID, MOTOR_VEL_ID) = -inv_inertia_motor * coloumb_fric_coef;

    // Initial Params

    transmission_ratio_N = 80;
    inv_transmission_ratio_N = 1 / transmission_ratio_N;

    // Initial values of extended States

    inertia_joint = 0.00025;
    inertia_motor = 54e-7;

    inv_inertia_joint = 1.0 / inertia_joint;
    inv_inertia_motor = 1.0 / inertia_motor;

    visc_fric_coef = 0.1;
    coloumb_fric_coef = 0.1 / coloumb_fric_threshold_vel;

    backlash_angle = 3.0 * DEG2RAD;
    stiffness = 1.0;



    // Sensor inputs
    position_sensor_val = 0.0;
    torque_sensor_val = 0.0;
    motor_torque_val = 0.0;


    // Sensor Noise values
    sensor_noise_encoder = 0.05 * DEG2RAD;
    sensor_noise_torque_sensor = 0.5;


    // Setup initial states;;
    x_current.Fill(0);
    x_current(INV_INERTIA_JOINT_ID) = inv_inertia_joint;
    x_current(INV_INERTIA_MOTOR_ID) = inv_inertia_motor;
    x_current(VISC_FRIC_ID) = visc_fric_coef;
    x_current(COLOUMB_FRIC_ID) = coloumb_fric_coef;
    x_current(BACKLASH_ANGLE_ID) = backlash_angle;
    x_current(STIFFNESS_ID) = stiffness;

    x_current(MOTOR_POS_ID) = 0.64;
    x_current(JOINT_POS_ID) = 0.64 / transmission_ratio_N;
    x_current(MOTOR_ACC_ID) = 0.0;
    x_current(MOTOR_VEL_ID) = 0.0;

    observer_matrix_H_motor_pos.Fill(0);
    observer_matrix_H_motor_pos(MOTOR_POS_ID, MOTOR_POS_ID) = 1.0;

    observer_matrix_H_joint_pos.Fill(0);
    observer_matrix_H_joint_pos(JOINT_POS_ID, JOINT_POS_ID) = 1.0;

    observer_matrix_H_load_torque.Fill(0);
    observer_matrix_H_load_torque(TORQUE_ID, TORQUE_ID) = 1.0;

    deterministic_input.Fill(0);
    deterministic_input(MOTOR_ACC_ID) = inv_inertia_motor;

    kalman_Gain.Fill(0);


    //initial error covariance_matrix;

    errorCovariance.Fill(0);
    errorCovariance(JOINT_POS_ID, JOINT_POS_ID) = 1e-3;
    errorCovariance(JOINT_VEL_ID, JOINT_VEL_ID) = 100 * DEG2RAD;
    errorCovariance(JOINT_ACC_ID, JOINT_ACC_ID) = 1000 * DEG2RAD;

    errorCovariance(MOTOR_POS_ID, MOTOR_POS_ID) = 1e-3;
    errorCovariance(MOTOR_VEL_ID, MOTOR_VEL_ID) = 100 * DEG2RAD;
    errorCovariance(MOTOR_ACC_ID, MOTOR_ACC_ID) = 1000 * DEG2RAD;

    errorCovariance(INV_INERTIA_JOINT_ID, INV_INERTIA_JOINT_ID) = 10000;
    errorCovariance(INV_INERTIA_MOTOR_ID, INV_INERTIA_MOTOR_ID) = 10000;

    errorCovariance(TORQUE_ID, TORQUE_ID) = 10;
    errorCovariance(STIFFNESS_ID, STIFFNESS_ID) = 10;
    errorCovariance(VISC_FRIC_ID, VISC_FRIC_ID) = 10.0;
    errorCovariance(COLOUMB_FRIC_ID, COLOUMB_FRIC_ID) = 10.0;
    errorCovariance(BACKLASH_ANGLE_ID, BACKLASH_ANGLE_ID) = 10.0 * DEG2RAD;

    // System_noise matrix

    // assume piece white noise model for acceleration (motor acc, joint angle acc)

    system_noise_matrix.Fill(0);

    system_noise_matrix(JOINT_POS_ID, JOINT_POS_ID) = delta_t * delta_t * delta_t * delta_t / 4.0;
    system_noise_matrix(JOINT_POS_ID, JOINT_VEL_ID) = delta_t * delta_t * delta_t / 2.0;
    system_noise_matrix(JOINT_POS_ID, JOINT_ACC_ID) = delta_t * delta_t / 2.0;

    system_noise_matrix(JOINT_VEL_ID, JOINT_POS_ID) = delta_t * delta_t * delta_t / 2.0;
    system_noise_matrix(JOINT_VEL_ID, JOINT_VEL_ID) = delta_t * delta_t;
    system_noise_matrix(JOINT_VEL_ID, JOINT_ACC_ID) = delta_t;

    system_noise_matrix(JOINT_ACC_ID, JOINT_POS_ID) = delta_t * delta_t / 2.0;
    system_noise_matrix(JOINT_ACC_ID, JOINT_VEL_ID) = delta_t;
    system_noise_matrix(JOINT_ACC_ID, JOINT_ACC_ID) = 1;

    system_noise_matrix(MOTOR_POS_ID, MOTOR_POS_ID) = delta_t * delta_t * delta_t * delta_t / 4.0;
    system_noise_matrix(MOTOR_POS_ID, MOTOR_VEL_ID) = delta_t * delta_t * delta_t / 2.0;
    system_noise_matrix(MOTOR_POS_ID, MOTOR_ACC_ID) = delta_t * delta_t / 2.0;

    system_noise_matrix(MOTOR_VEL_ID, MOTOR_POS_ID) = delta_t * delta_t * delta_t / 2.0;
    system_noise_matrix(MOTOR_VEL_ID, MOTOR_ACC_ID) = delta_t * delta_t;
    system_noise_matrix(MOTOR_VEL_ID, MOTOR_ACC_ID) = delta_t;

    system_noise_matrix(MOTOR_ACC_ID, MOTOR_POS_ID) = delta_t * delta_t / 2.0;
    system_noise_matrix(MOTOR_ACC_ID, MOTOR_ACC_ID) = delta_t;
    system_noise_matrix(MOTOR_ACC_ID, MOTOR_ACC_ID) = 1;


    float acc_value = 100.0 * DEG2RAD;
    system_noise_matrix = system_noise_matrix * acc_value * acc_value;


    // Add system uncertainty 
    system_noise_matrix(TORQUE_ID, TORQUE_ID) = 0.25;
    system_noise_matrix(STIFFNESS_ID, STIFFNESS_ID) = 100.0;
    system_noise_matrix(INV_INERTIA_JOINT_ID) = 1000.0;
    system_noise_matrix(INV_INERTIA_MOTOR_ID) = 1000;
    system_noise_matrix(BACKLASH_ANGLE_ID) = 1 * DEG2RAD;
    system_noise_matrix(VISC_FRIC_ID) = 1.0;
    system_noise_matrix(COLOUMB_FRIC_ID) = 10.0;

    system_noise_matrix = system_noise_matrix * 1e-1;


    /*
    system_noise_matrix.Fill(0);
    system_noise_matrix(0, 0) = 1e-3;
    system_noise_matrix(1, 1) = 5;
    system_noise_matrix(2, 2) = 100;
    */



    // Sensor Noise values
    sensor_noise_encoder = 0.05 * DEG2RAD;
    sensor_noise_torque_sensor = 0.5;


    identity_matrix.Fill(0);

    for (int i = 0; i < N_STATES; i++) {
        identity_matrix(i, i) = 1.0;
    }

    Serial.println("DRVSYS_KALMAN_INFO: Kalman Filter initialized");
    initialized = true;

}


void DriveDynamicKalmanFilter::predictionStep(float motor_torque) {

    this->motor_torque_val = motor_torque;
    //Serial.print("torque: ");
    //Serial.println(motor_torque);

    // obtain extended state parameters
    inv_inertia_joint = x_current(INV_INERTIA_JOINT_ID);
    inv_inertia_motor = x_current(INV_INERTIA_MOTOR_ID);

    visc_fric_coef = x_current(VISC_FRIC_ID);
    coloumb_fric_coef = x_current(COLOUMB_FRIC_ID);

    float fric_coef = get_friction_coefficient(x_current(MOTOR_VEL_ID), visc_fric_coef, coloumb_fric_coef);

    deterministic_input(MOTOR_ACC_ID) = inv_inertia_motor;

    //Serial.print("X_curr before ");
    //Serial.println(x_current(MOTOR_POS_ID));



    // Make Prediction

    // handle nonlinear backlash and linear torsion
    backlash_vector backlash = angle_difference_with_backlash(x_current(MOTOR_POS_ID), x_current(JOINT_POS_ID), backlash_angle);

    float torsion_torque = stiffness * backlash.delta_angle;

    //torsion_torque = 0.0;

    float torsion_factor = 0.0;

    if (backlash.in_deadzone) {
        torsion_factor = 0.0;
    }

    x_pred = x_current; // assume states to be static

    /*
    Serial.print("Predict:0 ");
    Serial.println(x_pred(MOTOR_POS_ID));
    */

    // predict all dynamic states
    x_pred(JOINT_POS_ID) = x_current(JOINT_POS_ID) + delta_t * x_current(JOINT_VEL_ID) + delta_t * delta_t * 0.5 * x_current(JOINT_ACC_ID);
    x_pred(JOINT_VEL_ID) = x_current(JOINT_VEL_ID) + delta_t * x_current(JOINT_ACC_ID);
    x_pred(JOINT_ACC_ID) = inv_inertia_joint * (torsion_torque - x_current(TORQUE_ID));
    x_pred(MOTOR_POS_ID) = x_current(MOTOR_POS_ID) + delta_t * x_current(MOTOR_VEL_ID) + delta_t * delta_t * 0.5 * x_current(MOTOR_ACC_ID);
    x_pred(MOTOR_VEL_ID) = x_current(MOTOR_VEL_ID) + delta_t * x_current(MOTOR_ACC_ID);
    x_pred(MOTOR_ACC_ID) = inv_inertia_motor * (-torsion_torque * inv_transmission_ratio_N - x_current(MOTOR_VEL_ID) * fric_coef + motor_torque_val);

    /*Serial.print("Predict ");

    for (int i = 0; i < N_STATES; i++) {
        Serial.print(x_pred(i));
        Serial.print(", ");
    }
    Serial.println("");
    */


    x_current = x_pred;
    //update system matrix (get Jacobian at current state)
    system_matrix_A(JOINT_ACC_ID, JOINT_POS_ID) = torsion_factor * stiffness * inv_inertia_joint;
    system_matrix_A(JOINT_ACC_ID, MOTOR_POS_ID) = -torsion_factor * stiffness * inv_transmission_ratio_N * inv_inertia_joint;

    system_matrix_A(MOTOR_ACC_ID, MOTOR_POS_ID) = inv_inertia_motor * torsion_factor * stiffness * inv_transmission_ratio_N;
    system_matrix_A(MOTOR_ACC_ID, JOINT_POS_ID) = -torsion_factor * inv_inertia_motor * stiffness * inv_transmission_ratio_N;

    system_matrix_A(JOINT_ACC_ID, TORQUE_ID) = -inv_inertia_joint;
    system_matrix_A(MOTOR_ACC_ID, MOTOR_VEL_ID) = -inv_inertia_motor * fric_coef;

    //update error covariance matrix
    errorCovariance = system_matrix_A * errorCovariance * (~system_matrix_A) + system_noise_matrix;

};

void DriveDynamicKalmanFilter::correctionStep_motor_pos(float sensor_motor_pos) {

    //Estimate Kalman Gain
    float divisor = 1.0 / (errorCovariance(MOTOR_POS_ID, MOTOR_POS_ID) + sensor_noise_encoder);

    kalman_Gain = errorCovariance * (~observer_matrix_H_motor_pos) * divisor;

    BLA::Matrix<N_STATES> z_n;
    z_n.Fill(0);
    z_n(MOTOR_POS_ID) = sensor_motor_pos;

    //Serial.print("sensor input: ");
    //Serial.println(sensor_motor_pos);
    x_corrected = x_current + kalman_Gain * (z_n - observer_matrix_H_motor_pos * x_current);

    /*
    Serial.print("X_corr: ");
    for (int i = 0; i < N_STATES; i++) {
        Serial.print(x_corrected(i));
        Serial.print(", ");
    }
    Serial.println("");
    */


    errorCovariance = (identity_matrix - kalman_Gain * observer_matrix_H_motor_pos) * errorCovariance;

    x_current = x_corrected;

};

void DriveDynamicKalmanFilter::correctionStep_joint_pos(float sensor_joint_pos) {
    //Estimate Kalman Gain
    float divisor = 1.0 / (errorCovariance(JOINT_POS_ID, JOINT_POS_ID) + sensor_noise_encoder);

    kalman_Gain = errorCovariance * (~observer_matrix_H_joint_pos) * divisor;

    BLA::Matrix<N_STATES> z_n;
    z_n.Fill(0);
    z_n(JOINT_POS_ID) = sensor_joint_pos;

    x_corrected = x_current + kalman_Gain * (z_n - observer_matrix_H_joint_pos * x_current);

    errorCovariance = (identity_matrix - kalman_Gain * observer_matrix_H_joint_pos) * errorCovariance;

    x_current = x_corrected;
};

void DriveDynamicKalmanFilter::correctionStep_load_torque(float sensor_torque) {

    //Estimate Kalman Gain

    float divisor = 1.0 / (errorCovariance(TORQUE_ID, TORQUE_ID) + sensor_noise_torque_sensor);

    kalman_Gain = errorCovariance * (~observer_matrix_H_load_torque) * divisor;

    BLA::Matrix<N_STATES> z_n;
    z_n.Fill(0);
    z_n(TORQUE_ID) = torque_sensor_val;

    x_corrected = x_current + kalman_Gain * (z_n - observer_matrix_H_load_torque * x_current);

    errorCovariance = (identity_matrix - kalman_Gain * observer_matrix_H_load_torque) * errorCovariance;

    x_current = x_corrected;

};

backlash_vector DriveDynamicKalmanFilter::angle_difference_with_backlash(float motor_pos, float joint_angle, float backlash_angle) {

    float delta = joint_angle - motor_pos * inv_transmission_ratio_N;

    backlash_vector output;
    output.in_deadzone = false;


    if (delta > backlash_angle) {
        output.delta_angle = delta - backlash_angle;
    }
    if (abs(delta) <= backlash_angle) {
        output.delta_angle = 0.0;
        output.in_deadzone = true;
    }
    if (delta < -backlash_angle) {
        output.delta_angle = delta + backlash_angle;
    }

    return output;
}

float DriveDynamicKalmanFilter::get_friction_coefficient(float motor_vel, float visc_fric_coef, float coloumb_fric_coef) {

    if (abs(motor_vel) > coloumb_fric_threshold_vel) {
        return visc_fric_coef;
    }
    else {
        return coloumb_fric_coef;
    }
}

KinematicStateVector DriveDynamicKalmanFilter::getEstimatedMotorState() {
    KinematicStateVector state;

    state.pos = x_current(MOTOR_POS_ID);
    state.vel = x_current(MOTOR_VEL_ID);
    state.acc = x_current(MOTOR_ACC_ID);

    return state;

}

DynamicStateVector DriveDynamicKalmanFilter::getEstimatedState() {
    DynamicStateVector state;

    state.joint_pos = x_current(JOINT_POS_ID);
    state.joint_vel = x_current(JOINT_VEL_ID);
    state.joint_acc = x_current(JOINT_ACC_ID);

    state.joint_torque = x_current(TORQUE_ID);

    state.motor_pos = x_current(MOTOR_POS_ID);
    state.motor_vel = x_current(MOTOR_VEL_ID);
    state.motor_acc = x_current(MOTOR_ACC_ID);

    return state;

};

ExtendedDynamicStateVector DriveDynamicKalmanFilter::getEstimatedExtendedState() {

    ExtendedDynamicStateVector state;

    state.joint_pos = x_current(JOINT_POS_ID);
    state.joint_vel = x_current(JOINT_VEL_ID);
    state.joint_acc = x_current(JOINT_ACC_ID);

    state.joint_torque = x_current(TORQUE_ID);

    state.motor_pos = x_current(MOTOR_POS_ID);
    state.motor_vel = x_current(MOTOR_VEL_ID);
    state.motor_acc = x_current(MOTOR_ACC_ID);

    state.inertia_motor = 1.0 / x_current(INV_INERTIA_MOTOR_ID);
    state.inertia_joint = 1.0 / x_current(INV_INERTIA_MOTOR_ID);
    state.backlash_angle = x_current(backlash_angle);
    state.stiffness = x_current(STIFFNESS_ID);
    state.viscous_fric_coef = x_current(VISC_FRIC_ID);
    state.coloumb_fric_coef = x_current(COLOUMB_FRIC_ID);

    return state;
}

DynamicParameters DriveDynamicKalmanFilter::getParameters() {

    DynamicParameters params;

    params.inertia_motor = 1.0 / x_current(INV_INERTIA_MOTOR_ID);
    params.inertia_joint = 1.0 / x_current(INV_INERTIA_MOTOR_ID);
    params.backlash_angle = x_current(backlash_angle);
    params.stiffness = x_current(STIFFNESS_ID);
    params.viscous_fric_coef = x_current(VISC_FRIC_ID);
    params.coloumb_fric_coef = x_current(COLOUMB_FRIC_ID);

    return params;


}

