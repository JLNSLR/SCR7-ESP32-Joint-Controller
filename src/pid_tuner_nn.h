#ifndef PID_TUNER_NN_H
#define PID_TUNER_NNH

#include <drive_system_settings.h>
#include <drive_system_types.h>
#include <NN/NeuralNetwork.h>
#include <NN/nn_utils.h>

#define PID_TUNER_NN_DEPTH 4
#define PID_TUNER_INPUTS 6
#define PID_TUNER_OUTPUTS 5
#define PID_TUNER_NN_HIDDEN_LAYER_1_SIZE 8
#define PID_TUNER_NN_HIDDEN_LAYER_2_SIZE 5

#define PID_TUNER_DEBUG



struct PID_Tuner_sample {
    float joint_pos;
    float joint_vel;
    float motor_pos;
    float motor_vel;

    float pos_target;
    float vel_target;

    float error;
    float dError;
    float ITermError;

    float motor_torque_u;

    float joint_pos_t_plus;
    float joint_vel_t_plus;

};

struct nn_ref_output_derivative {
    float dpos_du;
    float dvel_du;
};


class PID_Tuner_NN {

public:
    PID_Tuner_NN();
    void init();
    void set_scaling(float max_vel, float max_motor_torque);

    bool learning_step();

    drvSys_PID_Gains predictOptimalGains(const drvSys_FullDriveState state, const drvSys_driveTargets targets);

    NeuralNetwork* nn;

    NeuralNetwork* nn_reference;

    void addSample(const PID_Tuner_sample sample);

    long low_error_iterations = 0;

    void collectSampleTimeStep(drvSys_FullDriveState state);
    void collectSamplePIDState(drvSys_driveTargets target, drvSys_FullDriveState state, float error, float dError, float ITermError);

    bool saved_weights = false;


private:

    const float max_angle = 180.0 * DEG2RAD;
    float max_vel = 300 * DEG2RAD;
    float max_motor_torque = 0.45;

    float inv_max_vel = 1.0 / (300 * DEG2RAD);
    const float inv_max_angle = 1.0 / (180.0 * DEG2RAD);
    float inv_max_motor_torque = 1.0 / 0.45;

    const int max_buffer_size = 50;

    PID_Tuner_sample sample_buffer[50];

    int current_buffer_size = 0;

    float train_reference_nn(const PID_Tuner_sample sample);

    const nn_ref_output_derivative estimateOutputDerivative(const PID_Tuner_sample);

    SemaphoreHandle_t mutex_pid_tuner_net = xSemaphoreCreateBinary();


    int sampling_part = 0;
    PID_Tuner_sample input_sample;





};


#endif // !PID_TUNER_NN_H


