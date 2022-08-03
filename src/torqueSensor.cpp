#include <torqueSensor.h>


TorqueSensor::TorqueSensor() {

    cap_sensor = new FDC2214(FDC2214_I2C_ADDR_0);
}

TorqueSensor::TorqueSensor(torque_sensor_type type) {

    cap_sensor = new FDC2214(FDC2214_I2C_ADDR_0);
}


bool TorqueSensor::init(torque_sensor_type type, float sample_time_s) {


    Wire.begin(SDA, SCL);
    bool init_success = false;

    delta_t = sample_time_s;

    if (type == dms) {
        if (init_dms()) {
            Serial.println("DRVSYS: DMS based torque sensor initialized.");
            init_success = true;
        }
        else {
            Serial.println("DRVSYS: DMS bases torque sensor failed.");
            init_success = false;
        }
    }

    if (type == dms) {
        if (init_capacitive_sensors()) {
            Serial.println("DRVSYS: Capacitive torque sensor initialized.");
            init_success = true;
        }
        else {
            Serial.println("DRVSYS: Capacitive torque sensor failed.");
            init_success = false;
        }
    }


    // Drift Compensator init

    drift_sys_A.Fill(0);
    drift_sys_A(0, 0) = 1;
    drift_sys_A(0, 1) = 1;
    drift_sys_A(1, 1) = 0;

    sys_noise.Fill(0);
    sys_noise(0, 0) = drift_noise_val;
    sys_noise(1, 1) = drift_noise_val;

    errorCov = sys_noise * 1000;

    x_drift.Fill(0);
    x_drift(1) = initial_drift_val;

    observer_mat.Fill(0);
    observer_mat(0, 0) = 1;

    z_n.Fill(0);

    identity_mat.Fill(0);
    identity_mat(0, 0) = 1;
    identity_mat(1, 1) = 1;


    return init_success;

}

bool TorqueSensor::init_dms() {

    bool sensor_ok = false;

    if (dms_sensor.begin()) {
        sensor_ok = true;
        dms_sensor.setSampleRate(NAU7802_SPS_320);
        dms_sensor.setGain(NAU7802_GAIN_32);
        dms_sensor.beginCalibrateAFE();
        dms_sensor.waitForCalibrateAFE();
    }

    return sensor_ok;

}

bool TorqueSensor::init_capacitive_sensors() {

    // Start FDC2214 with 4 channels init
    bool sensor_ok = cap_sensor->begin(0xF, 0x6, 0b101, false);
    //setup all four channels, autoscan with 4 channels, deglitch at 10MHz, external oscillator 

    // set baselines
    return sensor_ok;
}

float TorqueSensor::read_raw_capacitive_sensors() {

    for (int i = 0; i < CAPACITIVE_CHANNELS; i++) { // for each channel
        // ### read 28bit data
        capacitor_values[i] = cap_sensor->getReading28(i) - capacitor_baselines[i];
        // ### Transmit data to serial in simple format readable by SerialPlot application.
    }


    float prop_torque = (capacitor_values[0] - capacitor_values[1]) + (capacitor_values[2] - capacitor_values[3]);

    // process capacitances further

    return prop_torque;


}

float TorqueSensor::read_raw_dms() {

    float data = 0;
    if (dms_sensor.available()) {
        data = dms_sensor.getReading() - internal_offset;

        Serial.print(data);

        float delta_data = (data - prev_val) * 0.01 + 0.99 * delta_data_prev;
        prev_val = data;
        delta_data_prev = delta_data;


        data = data + damping * delta_data;
        Serial.print("\t");
        Serial.println(data);


    }

    return data;
}

float TorqueSensor::get_torque_measurement() {

    float sensor_val = 0;

    if (type == dms) {

        sensor_val = read_raw_dms();

        //Serial.print(sensor_val);
        sensor_val = compensate_drift(sensor_val);
        //Serial.print("\t");
        //Serial.println(sensor_val);


        sensor_val = sensor_val * conversion_factor - external_offset;

    }

    //Implement Capacitive Sensing here

    if (type == cap) {
        sensor_val = read_raw_capacitive_sensors();

        sensor_val = compensate_drift(sensor_val);

        sensor_val = sensor_val * conversion_factor - external_offset;
    }


    return sensor_val;
}

float TorqueSensor::compensate_drift(float input) {


    sys_noise(0, 0) = drift_noise_val;
    sys_noise(1, 1) = drift_noise_val;
    // Prediction Step

    x_pred = drift_sys_A * x_drift;

    errorCov = drift_sys_A * errorCov * (~drift_sys_A) + sys_noise;

    // Correction Step

    float divisor = 1.0 / (errorCov(0, 0) + drift_measurement_noise);

    kalman_gain = errorCov * (~observer_mat) * divisor;

    z_n(0) = input;

    x_drift = x_pred + kalman_gain * (z_n - observer_mat * x_pred);
    errorCov = (identity_mat - kalman_gain * observer_mat) * errorCov;


    float corrected_value = input - x_drift(0);
    return corrected_value;


}


