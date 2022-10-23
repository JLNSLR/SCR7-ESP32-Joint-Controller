#ifndef CLOSED_LOOP_STEPPER_CONTROL_H
#define CLOSED_LOOP_STEPPER_CONTROL_H

#include <Arduino.h>
#include <FreeRTOS.h>
#include <TMCStepper.h>
#include <joint_control_global_def.h>
#include <drive_system_settings.h>
#include <drive_system_types.h>







class ClosedLoopStepperController {

public:
    ClosedLoopStepperController(TMC2160Stepper* stepper_driver);

    void setup(TMC2160Stepper* stepper_driver);

    void set_target_vel(float vel);

    TMC2160Stepper* stepper_driver;

    volatile void IRAM_ATTR generatePulse();

    void stop();
    void start();


private:


    volatile long interrupt_counter = 0;
    volatile int step = 1;

    hw_timer_t* timer = NULL;

    bool setup_done = false;

    float current_vel_deg = 0;


    int prescaler = 80; // Timer then hits every 100ns

    portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;


    volatile int output_pulse_ticks = 100000000;
    volatile bool output_pulse = false;

    volatile int dir = 0;
};

void IRAM_ATTR stepper_interrupt();

#endif // !CLOSED_LOOP_STEPPER_CONTROL_H