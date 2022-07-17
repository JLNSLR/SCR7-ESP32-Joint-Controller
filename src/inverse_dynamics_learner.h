#ifndef INVERSE_DYNAMICS_LEARNER_H
#define INVERSE_DYNAMICS_LEARNER_H

#include <drive_system.h>
#include <Arduino.h>
#include <drive_system_types.h>
#include <NN/nn_utils.h>
#include <NN/NeuralNetwork.h>
#include <drive_system_settings.h>

struct inv_dynamics_sample {
    float input_vector[7];
    float output;
};

class InverseDynamicsLearner {

public:
    InverseDynamicsLearner();
    void init();
    float predict_torque(drvSys_FullDriveState state);
    void set_scale(float max_motor_torque, float max_vel, float max_acc, float max_joint_torque);
    void add_data(const drvSys_FullDriveState state);
    bool learning_step();

    NeuralNetwork* nn;

    bool activate_automatic = true;

    const int n_inputs = 7;
    const int n_outputs = 1;

    float prediction_error = 100.0;
    float delta_prediction_error = 100.0;

    bool control_active = false;
    float prediction_error_threshold = 0.01;
    bool saved_weights = false;



private:
    static const int buffer_size = 50;
    inv_dynamics_sample sample_vector[buffer_size];
    int samples_in_buffer = 0;


    float max_motor_torque = 1.92; //Nm
    float max_vel = 300 * DEG2RAD; //rad/s
    float max_acc = 2000 * DEG2RAD; //rad/s
    float max_joint_torque = 100;

    float inv_max_motor_torque;
    float inv_max_vel;
    float inv_acc;
    float inv_max_joint_torque;

    long low_change_iterations = 0;
    bool low_learning_mode = false;
    long low_error_iterations = 0;

    SemaphoreHandle_t mutex_inverse_dyn_network = xSemaphoreCreateBinary();;

    const float max_angle = 180 * DEG2RAD;
    const float inv_max_angle = 1.0 / max_angle;

    void addSampleToBuffer(inv_dynamics_sample sample);
    inv_dynamics_sample takeSampleFromBuffer();

    void randomize(inv_dynamics_sample arr[], int n);
    void swap(inv_dynamics_sample* a, inv_dynamics_sample* b);

};

#endif // !INVERSE_DYNAMICS_LEARNER_H
