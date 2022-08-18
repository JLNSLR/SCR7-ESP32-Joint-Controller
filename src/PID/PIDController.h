#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

#include "Arduino.h"

#define PID_MODE_INACTIVE 0
#define PID_MODE_ACTIVE 1

#define PID_DIR_DIRECT 0
#define PID_DIR_REVERSE 1

/*
PID Controller class using the fix point math in signed 32-bit arithmetics
*/
class PIDController
{
public:
    PIDController();
    PIDController(float kp, float ki, float kd);

    void compute();
    void setTuning(float kp, float ki, float kd);

    void setVelocityDTerm(float vel_error);

    // set sampleTime in us
    void setSampleTime(int newSampleTime_us);

    void setOutputLimits(float min, float max);
    //activates or deactives the controller
    void setMode(int Mode);

    void Initialize();

    void SetControllerDirection(int Direction);

    float* getGains();
    // sets the filter for D-Term
    void setDifferentialFilter(bool isActive, float alpha = 0.5);
    // sets the filter for output
    void setInputFilter(bool isActive, float alpha = 0.5);

    bool getMode();

    void setErrorDeadBand(float const deadband);

    void setSetPoint(float setpoint, bool reset_I = false);

    void backlash_compensator(float backlash_thres, float kp, float ki);

    float get_sample_time();

    void get_internal_gains(float gains[]);

    float double_exp_filter(float value);

    void setInputFilter_double_exp(bool double_exp, float alpha, float gamma);


    /* ---  working variables --- */

    float input, output, setpoint;
    float outMin, outMax;

    float error = 0;
    float prev_error = 0;
    float prev_error_t2 = 0;
    float dError = 0;
    float iTerm = 0;

    float velError = 0;

    bool derivative_on_measurement = true;

    float gains[3] = { 0 };

    bool d_term_vel = false;

private:
    float kp, ki, kd;
    float lastInput = 0;

    float gains_int[3] = { 0 };

    // sample time in microseconds
    int sampleTime = 1; // microseconds

    bool isActive = true;
    bool filterDerivative = false;
    bool outputFilter = false;

    float errorNoise = 0;
    float errorDeadBand = 0;

    int controllerDirection = PID_DIR_DIRECT;

    float d_filter_alpha = 0.5;
    float input_exp_filter_alpha = 0.5;
    bool double_exp = false;

    float gamma_d_exp_filter = 0.5;
};

#endif // PIDCONTROLLER_H