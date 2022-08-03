#ifndef TORQUE_SENSOR_H
#define TORQUE_SENSOR_H

#include <Arduino.h>
#include <BasicLinearAlgebra.h>
#include <SparkFun_Qwiic_Scale_NAU7802_Arduino_Library.h>
#include <CAP/FDC2214.h>
#include <joint_control_global_def.h>

#define TORQUE_SENSOR_SAMPLE_TIME 3.125e-3
#define CAPACITIVE_CHANNELS 4

enum torque_sensor_type { dms, cap };

class TorqueSensor {

public:
    TorqueSensor();
    TorqueSensor(torque_sensor_type type);

    bool init(torque_sensor_type type, float sample_time_s = TORQUE_SENSOR_SAMPLE_TIME);
    float get_torque_measurement();

    torque_sensor_type type = dms;

    float delta_t = 1.0 / 320;

    // Calibration Data
    float internal_offset = 0;
    float external_offset = 0;
    float conversion_factor = 0.001; // integer value to Nm

    // Drift Compensation Parameters
    float drift_noise_val = 1e-9;
    float drift_measurement_noise = 30000;
    float initial_drift_val = 0.0;

    float damping = 5;



private:

    float compensate_drift(float input);

    bool init_dms();
    bool init_capacitive_sensors();
    float read_raw_dms();
    float read_raw_capacitive_sensors();

    NAU7802 dms_sensor;
    FDC2214* cap_sensor;


    // Drift Compensation
    BLA::Matrix<2, 2> drift_sys_A;
    BLA::Matrix<2, 2> sys_noise;
    BLA::Matrix<2, 2> errorCov;
    BLA::Matrix<2, 2> kalman_gain;
    BLA::Matrix<2, 2> observer_mat;
    BLA::Matrix<2> x_pred;
    BLA::Matrix<2> x_drift;
    BLA::Matrix<2> z_n;
    BLA::Matrix<2, 2> identity_mat;

    //Capacitor Measurements
    int32_t capacitor_baselines[CAPACITIVE_CHANNELS] = { 0 };
    int32_t capacitor_values[CAPACITIVE_CHANNELS] = { 0 };

    float prev_val = 0;

    float delta_data_prev = 0;


};

#endif // !TORQUE_SENSOR_H
