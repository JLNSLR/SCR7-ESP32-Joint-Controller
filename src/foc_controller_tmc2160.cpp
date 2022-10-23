#include <foc_controller_tmc2160.h>
#define EN_PIN 15  // Enable

FOCController::FOCController() {
    init_sine_quart();
}
FOCController::FOCController(AS5048A* m_encoder, TMC2160Stepper* driver,
    int16_t max_current_mA, float max_torque, SemaphoreHandle_t SPI_mutex) :
    motor_encoder(m_encoder), driver(driver), max_current_mA(max_current_mA),
    max_torque(max_torque), foc_spi_mutex(SPI_mutex)
{

    this->foc_output_const = (255.0 / 8191.0) * 1.0 / max_torque;

    this->foc_torque_command_spinlock = portMUX_INITIALIZER_UNLOCKED;

    init_sine_quart();
}

void FOCController::setup_driver() {

    Serial.println("DRVSYS_FOC_INFO: Initialize TMC2160 Stepper Driver");
    pinMode(EN_PIN, OUTPUT);
    digitalWrite(EN_PIN, LOW);

    xSemaphoreTake(foc_spi_mutex, portMAX_DELAY);

    driver->begin();           //  SPI: Init CS pins and possible SW SPI pins

    Serial.print("DRVSYS_FOC_INFO: DRV_STATUS=0b");
    Serial.println(driver->DRV_STATUS(), BIN);

    driver->toff(5);           // Enables driver in software
    driver->rms_current(max_current_mA); // Set motor RMS current
    driver->microsteps(256);   // Set microsteps to 1/16th

    driver->en_pwm_mode(true); // Toggle stealthChop on TMC2130/2160/5130/5160
    driver->pwm_autoscale(true); // Needed for stealthChop

    driver->direct_mode(true);

    xSemaphoreGive(foc_spi_mutex);

    init_sine_quart();

}

void FOCController::set_max_current(int16_t max_current_mA, float max_torque) {
    xSemaphoreTake(foc_spi_mutex, portMAX_DELAY);
    driver->rms_current(max_current_mA); // Set motor RMS current
    xSemaphoreGive(foc_spi_mutex);
    this->foc_output_const = (255.0 / 8191.0) * 1.0 / max_torque;
}


void FOCController::calibrate_phase_angle(uint32_t phase_angle_null) {

    if (phase_angle_null != 0) {
        phase_null_angle = phase_angle_null;
    }
    else {
        Serial.println("DRVSYS_FOC_INFO: Starting electric angle initial calibration.");
        xSemaphoreTake(foc_spi_mutex, portMAX_DELAY);
        SPI.begin();
        driver->coil_A(0);
        driver->coil_B(0);
        xSemaphoreGive(foc_spi_mutex);

        vTaskDelay(500 / portTICK_PERIOD_MS);

        xSemaphoreTake(foc_spi_mutex, portMAX_DELAY);
        driver->coil_A(255);
        driver->coil_B(0);
        xSemaphoreGive(foc_spi_mutex);

        Serial.print("DRVSYS_FOC_INFO: Energize Coils to Position Rotor.");

        vTaskDelay(250 / portTICK_PERIOD_MS);

        uint32_t null_angle_temp = 0.0;
        int count = 100000;


        xSemaphoreTake(foc_spi_mutex, portMAX_DELAY);
        for (int i = 0; i < count; i++) {

            null_angle_temp = null_angle_temp + motor_encoder->getRotationPos();

        }
        xSemaphoreGive(foc_spi_mutex);

        null_angle_temp = null_angle_temp / count;

        phase_null_angle = null_angle_temp;
    }


    Serial.print("DRVSYS_FOC_INFO: Found initial electric angle offset ");
    Serial.println(phase_null_angle);
    vTaskDelay(1000 / portTICK_PERIOD_MS);

}

void FOCController::set_target_torque(float torque_target) {

    if (torque_target > max_torque) {
        torque_target = max_torque;
    }
    else if (torque_target < -max_torque) {
        torque_target = -max_torque;
    }

    portENTER_CRITICAL(&foc_torque_command_spinlock);
    this->target_torque = torque_target;
    portEXIT_CRITICAL(&foc_torque_command_spinlock);

    if (target_torque == 0.0) {
        driver->freewheel(0b01);
    }
}

int32_t sign(int32_t value) {
    if (value >= 0) {
        return 1;
    }
    else {
        return -1;
    }
}

void FOCController::foc_control() {
    // variables for velocity compensation

    //static long last_micros = 0;

    //static float delay_normalizer_us = 1.0 / 100.0; // 1/200us

    // long current_micros = micros();

     //long passed_micros = current_micros - last_micros;


     //float  delta_angle_factor = passed_micros * delay_normalizer_us;

     //Serial.println(passed_micros);
     //last_micros = current_micros;

    static int32_t prev_delta_angle = 0;
    static int32_t prev_angle = 0;

    // obtain current motor angle

    xSemaphoreTake(foc_spi_mutex, portMAX_DELAY);
    int32_t motor_angle = motor_encoder->getRotation(false);
    xSemaphoreGive(foc_spi_mutex);

    int32_t delta_angle = motor_angle - prev_angle;
    int32_t delta_delta_angle = delta_angle - prev_delta_angle;
    prev_delta_angle = delta_angle;
    prev_angle = motor_angle;

    delta_angle = 1.0 * (delta_angle + 0.5 * delta_delta_angle);

    // Filter velocity compensation
    delta_angle = 0.5 * delta_angle + 0.5 * prev_delta_angle;
    prev_delta_angle = delta_angle;


    // anticipate phase shift based on previous shift
    int32_t empiric_phase_shift = 4* delta_angle;

    empiric_phase_shift = empiric_phase_shift;

    int32_t electric_angle_int = (int32_t(motor_encoder->last_sample_raw) - int32_t(phase_null_angle) + empiric_phase_shift) * N_pole_pairs;

    // calculate desired phase currents

    int add_phase = 0;
    
    if (target_torque > 0) {
        add_phase = 512;
    }
    else {
        add_phase = -512;
    }
    


    int16_t i_a_simp = double(-foc_output_const * target_torque * sine_lookup(electric_angle_int + add_phase));
    int16_t i_b_simp = double(foc_output_const * target_torque * sine_lookup(4096 + electric_angle_int + add_phase));

    // create phase current register values
    xdirect_tmc direct;
    direct.values.coil_A = i_a_simp;
    direct.values.coil_B = i_b_simp;


    // write into current register of the driver
    xSemaphoreTake(foc_spi_mutex, portMAX_DELAY);
    driver->XDIRECT(direct.reg);
    xSemaphoreGive(foc_spi_mutex);

}


void FOCController::set_target_torque_9bit(int16_t torque_target) {
    static const float inv_max_torque = 1.0 / max_torque;
    if (torque_target > 255) {
        torque_target = 255;
    }
    else if (torque_target < -255) {
        torque_target = -255;
    }
    portENTER_CRITICAL(&foc_torque_command_spinlock);
    this->target_torque = float(torque_target / 255.0) * inv_max_torque;
    portEXIT_CRITICAL(&foc_torque_command_spinlock);
}

void FOCController::init_sine_quart() {

    const double sine_res = FOC_SINE_LOOKUP_RES;

    for (int i = 0; i < FOC_SINE_LOOKUP_SIZE; i++) {
        sineQuart_14bit[i] = sin(sine_res * (PI / 180.0) * double(i)) * 8192;
        //Serial.println(sineQuart[i]);
        //Serial.println(sineQuart_14bit[i]);
        //Serial.println(i);

    }
}

void FOCController::set_empiric_phase_shift_factor(float factor) {
    this->empiric_phase_shift_factor = factor;
}


int32_t FOCController::sine_lookup(int32_t val) {
    static const double divisor = FOC_SINE_LOOKUP_DIVISOR_disc;
    int32_t sign = 1;
    if (val < 0) {
        val = -val;
        sign = -1;
    }

    while (val > 16384) {
        val = val - 16384;
    }

    if (val <= 4096) {
        uint16_t index = round(val * divisor);
        return sineQuart_14bit[index] * sign;
    }
    else if (val > 4096 && val <= 8192) {
        uint16_t index = round((val - 4096) * divisor);
        return sineQuart_14bit[FOC_SINE_LOOKUP_SIZE - 1 - index] * sign;
    }
    else if (val > 8192 && val <= 12288) {
        uint16_t index = round((val - 8192) * divisor);
        return (-1) * sineQuart_14bit[index] * sign;
    }
    else if (val > 12288) {
        uint16_t index = round((val - 12288) * divisor);
        return -sineQuart_14bit[FOC_SINE_LOOKUP_SIZE - 1 - index] * sign;
    }
    return 0;

}