#include <closed_loop_stepper_control.h>

ClosedLoopStepperController* controller_pointer;

void IRAM_ATTR stepper_interrupt() {

    controller_pointer->generatePulse();


}


ClosedLoopStepperController::ClosedLoopStepperController(TMC2160Stepper* stepper) {

    this->stepper_driver = stepper;

};


void ClosedLoopStepperController::setup(TMC2160Stepper* stepper) {


    controller_pointer = this;

    this->stepper_driver = stepper;

    //stepper_driver->begin();
    stepper_driver->direct_mode(false);
    stepper_driver->toff(5);
    stepper_driver->rms_current(DRVSYS_PHASE_CURRENT_NOMINAL_mA);
    stepper_driver->microsteps(DRVSYS_MICROSTEPS);
    stepper_driver->en_pwm_mode(true);
    stepper_driver->pwm_autoscale(true);
    stepper_driver->dedge(true);


    pinMode(TMC_DIR, OUTPUT);
    pinMode(TMC_STEP, OUTPUT);



    interrupt_counter = 0;


    // Timer prescaler tick every 100ns (80MHz/8 -> 10 MHz)
    timer = timerBegin(1, 80, true);

    timerAttachInterrupt(timer, &stepper_interrupt, true);
    timerAlarmWrite(timer, 10, true);


    output_pulse = false;

    setup_done = true;


};

void ClosedLoopStepperController::start() {

    if (setup_done) {
        Serial.println("DRVSYS: Start Stepper Timer");
        timerAlarmEnable(timer);
    }
}

void ClosedLoopStepperController::stop() {


    timerAlarmDisable(timer);
    stepper_driver->direct_mode(true);
    stepper_driver->rms_current(DRVSYS_PHASE_CURRENT_MAX_mA);


}

volatile void IRAM_ATTR ClosedLoopStepperController::generatePulse() {

    volatile static uint64_t tickCount = 0;
    tickCount++;


    portENTER_CRITICAL_ISR(&timerMux);
    if (output_pulse) {
        if (tickCount % output_pulse_ticks == 0) {
            step = !step;
            digitalWrite(TMC_STEP, step);

            // output stuff
        }
    }
    portEXIT_CRITICAL_ISR(&timerMux);



}

void ClosedLoopStepperController::set_target_vel(float vel) {

    static const float const_factor = (1.8 / (DRVSYS_MICROSTEPS)) * (1.0 / DRVSYS_TRANSMISSION_RATIO);

    float vel_deg = vel * RAD2DEG;

    // Rate Limiter
    const float rate_limit_vel_change_deg = 1000;

    if (vel_deg - current_vel_deg > rate_limit_vel_change_deg) {
        current_vel_deg = current_vel_deg + rate_limit_vel_change_deg;
    }
    else if (vel_deg - current_vel_deg < -rate_limit_vel_change_deg) {
        current_vel_deg = current_vel_deg - rate_limit_vel_change_deg;
    }
    else {
        current_vel_deg = vel_deg;
    }



    if (vel == 0.0) {
        output_pulse = false;
    }
    else {
        output_pulse = true;

        float t_period_10us = (const_factor / current_vel_deg) * (1.0 / 10e-6);

        int ticks = abs(int(t_period_10us));

        if (ticks == 0) {
            ticks = 1;
        }

        portENTER_CRITICAL(&timerMux);
        output_pulse_ticks = ticks;
        portEXIT_CRITICAL(&timerMux);

    }

    if (vel < 0) {
        dir = 0;
    }
    else {
        dir = 1;
    }

    digitalWrite(TMC_DIR, dir);


}