#include "PIDController.h"

PIDController::PIDController() {

    input = 0;
    output = 0;
    setpoint = 0;
    error = 0;
    prev_error = 0;
    prev_error_t2 = 0;
    dError = 0;
    iTerm = 0;
    kp = 0;
    kd = 0;
    ki = 0;


};

PIDController::PIDController(float kp, float ki, float kd) : PIDController()
{
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;

    error = 0;
    prev_error = 0;
    prev_error_t2 = 0;
    dError = 0;
    iTerm = 0;
    lastInput = 0;
};

void PIDController::compute()
{

    if (!isActive)
    {
        return;
    }

    /* Compute error variables */

    error = setpoint - input;

    float error_raw;

    /* Compute Output Filter */
    if (outputFilter)
    {
        if (double_exp) {
            error = double_exp_filter(error);
        }
        else {

            error = error * input_exp_filter_alpha + prev_error * (1 - input_exp_filter_alpha);
        }


    }


    iTerm += error;

    if (fabs(error) < errorDeadBand)
    {
        error = 0.0;
    }

    // Clamp iTerm against windup
    if (iTerm > outMax) {
        iTerm = outMax;
    }
    else if (iTerm < outMin) {
        iTerm = outMin;
    }

    static float prev_dInput = 0;

    if (derivative_on_measurement) {
        dError = (-1.0) * (input - lastInput);
    }
    else {
        dError = error - prev_error;
    }

    prev_error_t2 = prev_error;
    prev_error = error;


    if (fabs(dError) < errorDeadBand)
    {
        dError = 0;
    }

    if (d_term_vel) {
        dError = velError;
    }


    /* Compute Derivative Filter */
    if (filterDerivative)
    {
        dError = dError * d_filter_alpha + prev_dInput * (1 - d_filter_alpha);
        prev_dInput = dError;
    }

    /* Compute PID Output */

    output = kp * error + ki * iTerm + kd * dError;


    // Clamp output against windup
    if (output > outMax)
        output = outMax;
    else if (output < outMin)
        output = outMin;

    /* Save Variables for next step */
};


void PIDController::setSetPoint(float setpoint, bool reset_I) {

    static float prev_setpoint = 0;
    if (reset_I) {

        if (prev_setpoint != setpoint) {
            iTerm = 0.0;
        }
        prev_setpoint = setpoint;

    }

    this->setpoint = setpoint;
}

void PIDController::setTuning(float kp, float ki, float kd)
{

    if (kp < 0 || ki < 0 || kd < 0)
        return;

    float sampleTimeInSec = ((float)sampleTime / (1000 * 1000));


    gains[0] = kp;
    gains[1] = ki;
    gains[2] = kd;

    this->kp = kp;
    this->ki = ki * sampleTimeInSec;
    this->kd = kd / sampleTimeInSec;

    if (controllerDirection == PID_DIR_REVERSE)
    {
        this->kp = (0 - kp);
        this->ki = (0 - ki);
        this->kd = (0 - kd);
    }
}

void PIDController::setSampleTime(int newSampleTime)
{

    if (newSampleTime > 0)
    {

        float ratio = (float)newSampleTime / (float)sampleTime;

        ki = ((float)ki * ratio);
        kd = ((float)kd / ratio);

        sampleTime = newSampleTime;
    }
}

void PIDController::setOutputLimits(float min, float max)
{

    if (min > max)
        return;
    outMin = min;
    outMax = max;

    if (output > outMax)
        output = outMax;
    else if (output < outMin)
        output = outMin;

    if (iTerm > outMax)
        iTerm = outMax;
    else if (iTerm < outMin)
        iTerm = outMin;
}

void PIDController::setMode(int Mode)
{
    bool newMode = (Mode == PID_MODE_ACTIVE);
    if (newMode && !isActive)
    { /*we just went from manual to auto*/
        Initialize();
    }
    isActive = newMode;
}

void PIDController::Initialize()
{
    lastInput = input;
    iTerm = output;
    if (iTerm > outMax)
        iTerm = outMax;
    else if (iTerm < outMin)
        iTerm = outMin;
}

void PIDController::SetControllerDirection(int Direction)
{
    controllerDirection = Direction;
}

float* PIDController::getGains()
{

    float sampleTimeInSec = ((float)sampleTime * 1e-6);
    gains[0] = kp;
    gains[1] = ki / sampleTimeInSec;
    gains[2] = kd * sampleTimeInSec;

    return gains;
}

void PIDController::get_internal_gains(float gains[])
{

    gains[0] = kp;
    gains[1] = ki;
    gains[2] = kd;

}

void PIDController::setDifferentialFilter(bool isActive, float alpha)
{
    this->filterDerivative = isActive;
    this->d_filter_alpha = alpha;
}

void PIDController::setInputFilter(bool isActive, float alpha)
{
    this->outputFilter = isActive;
    this->input_exp_filter_alpha = alpha;
}

void PIDController::setErrorDeadBand(float const deadband) {
    this->errorDeadBand = deadband;
}

bool PIDController::getMode()
{
    return this->isActive;
}

float PIDController::get_sample_time() {

    return float(sampleTime) * 1e-6;
}


float PIDController::double_exp_filter(float value) {

    static float output = 0;
    static float b = 0;
    float new_output = value * input_exp_filter_alpha + (1 - input_exp_filter_alpha) * (output + b);

    b = gamma_d_exp_filter * (new_output - output) + (1 - gamma_d_exp_filter) * b;

    output = new_output;

    return output;
}


void PIDController::setInputFilter_double_exp(bool double_exp, float alpha, float gamma) {

    this->double_exp = double_exp;

    input_exp_filter_alpha = alpha;

    gamma_d_exp_filter = gamma;


}
