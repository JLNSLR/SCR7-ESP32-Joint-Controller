#include <torqueSensor.h>
#include <drive_system_settings.h>


TorqueSensor::TorqueSensor() {

    cap_sensor = new FDC2214(FDC2214_I2C_ADDR_0);
}

TorqueSensor::TorqueSensor(torque_sensor_type type) {

    cap_sensor = new FDC2214(FDC2214_I2C_ADDR_0);
}


bool TorqueSensor::init(torque_sensor_type type, float sample_time_s) {

    xSemaphoreTake(glob_I2C_mutex, portMAX_DELAY);
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

    if (type == cap) {
        if (init_capacitive_sensors()) {
            Serial.println("DRVSYS: Capacitive torque sensor initialized.");
            init_success = true;
        }
        else {
            Serial.println("DRVSYS: Capacitive torque sensor failed.");
            init_success = false;
        }
    }
    xSemaphoreGive(glob_I2C_mutex);

    // check flash storage for calibration data

    read_calibration_data();



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
        dms_sensor.setLDO(NAU7802_LDO_3V0);
        dms_sensor.setSampleRate(NAU7802_SPS_320);
        dms_sensor.setGain(NAU7802_GAIN_64);
        dms_sensor.beginCalibrateAFE();
        dms_sensor.waitForCalibrateAFE();
    }

    conversion_factor = TORQUE_SENSE_SLOPE;
    internal_offset = TORQUE_SENSE_INT_OFFSET;


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
    xSemaphoreTake(glob_I2C_mutex, portMAX_DELAY);
    for (int i = 0; i < CAPACITIVE_CHANNELS; i++) { // for each channel
        // ### read 28bit data
        capacitor_values[i] = cap_sensor->getReading28(i) - capacitor_baselines[i];
        // ### Transmit data to serial in simple format readable by SerialPlot application.
    }
    xSemaphoreGive(glob_I2C_mutex);


    float prop_torque = (capacitor_values[0] - capacitor_values[1]) + (capacitor_values[2] - capacitor_values[3]);

    // process capacitances further

    return prop_torque;


}

float TorqueSensor::read_raw_dms() {

    static float data = 0;
    static float data_prev = 0;

    xSemaphoreTake(glob_I2C_mutex, portMAX_DELAY);
    if (dms_sensor.available()) {
        raw_sensor_val = dms_sensor.getReading();
        data = raw_sensor_val - internal_offset;
        xSemaphoreGive(glob_I2C_mutex);

        //Serial.print(data);
        /*
        float delta_data = (data - prev_val) * 0.01 + 0.99 * delta_data_prev;
        prev_val = data;
        delta_data_prev = delta_data;


        data = data + damping * delta_data;
        */
        //Serial.print("\t");
        //Serial.println(data);

        if (raw_sensor_val == 0) {
            data = data_prev;
        }
        else {
            const float alpha = 0.32; // ~125Hz

            data = data * alpha + data_prev * (1 - alpha);
            data_prev = data;
        }



    }
    else {
        data = data_prev;
    }
    xSemaphoreGive(glob_I2C_mutex);

    return data;
}

float TorqueSensor::get_torque_measurement() {

    float sensor_val = 0;

    if (type == dms) {

        sensor_val = read_raw_dms();

        //Serial.print(sensor_val);
        //sensor_val = compensate_drift(sensor_val);
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


/**
 * @brief Set the save calibration data object
 *
 * @param type 0 - conversion factor, 1 - internal offset, 2 - external offset
 * @param value value
 */
void TorqueSensor::set_save_calibration_data(int type, float value) {

    torque_preferences.begin(torque_sensor_calibration_ns, false);
    if (type == 0) {
        this->conversion_factor = value;
        torque_preferences.putFloat(conv_factor_key, value);
        torque_preferences.putBool(conv_factor_set_key, true);
    }
    else if (type == 1) {
        this->internal_offset = value;
        torque_preferences.putFloat(int_offset_key, value);
        torque_preferences.putBool(int_offset_set_key, true);
    }
    else if (type == 2) {
        this->external_offset = value;
        torque_preferences.putFloat(ext_offset_key, value);
    }

    bool torque_conv_set = torque_preferences.getBool(conv_factor_set_key, false);
    bool torque_int_offset_set = torque_preferences.getBool(int_offset_set_key, false);

    if (torque_conv_set && torque_int_offset_set) {

        this->calibrated = true;
    }

    torque_preferences.end();
}

void TorqueSensor::remove_calibration_data() {

    torque_preferences.begin(torque_sensor_calibration_ns, false);

    torque_preferences.clear();

    torque_preferences.end();

}


void TorqueSensor::read_calibration_data() {

    torque_preferences.begin(torque_sensor_calibration_ns, false);

    if (torque_preferences.getBool(conv_factor_set_key, false)) {
        this->conversion_factor = torque_preferences.getFloat(conv_factor_key, 1.0);

        if (torque_preferences.getBool(int_offset_set_key, false)) {
            this->internal_offset = torque_preferences.getFloat(int_offset_key, 0);

            this->calibrated = true;

            this->external_offset = torque_preferences.getFloat(ext_offset_key, 0);

        }
        else {
            this->calibrated = false;
        }
    }
    torque_preferences.end();


}


