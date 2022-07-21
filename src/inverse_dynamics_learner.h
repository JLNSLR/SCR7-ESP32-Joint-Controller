#ifndef INVERSE_DYNAMICS_LEARNER_H
#define INVERSE_DYNAMICS_LEARNER_H

#include <drive_system.h>
#include <Arduino.h>
#include <drive_system_types.h>
#include <NN/nn_utils.h>
#include <NN/NeuralNetwork.h>
#include <drive_system_settings.h>
#include <CircularBuffer.h>

struct inv_dynamics_sample {
    float joint_pos_norm;
    float joint_vel_norm;
    float joint_acc_norm;
    float motor_pos_norm;
    float motor_vel_norm;
    float motor_acc_norm;
    float joint_torque;
    float output_motor_torque_norm;
};

class InverseDynamicsLearner {

public:
    InverseDynamicsLearner();
    void init();
    float predict_torque(drvSys_FullDriveState state);
    void set_scale(float max_motor_torque, float max_vel, float max_acc, float max_joint_torque);
    void add_data(const drvSys_FullDriveState state);
    bool learning_step();

    static const int depth = 4;
    int width[depth] = { 7,7,5,1 };
    nn_activation_f activation[depth - 1] = { leakyReLu, leakyReLu, Linear };

    NeuralNetwork* nn;

    bool activate_automatic = true;

    const int n_inputs = 7;
    const int n_outputs = 1;

    float prediction_error = 100.0;
    float delta_prediction_error = 100.0;

    float prediction_error_threshold = 0.01;
    bool saved_weights = false;


private:
    static const int buffer_size = 50;

    CircularBuffer<inv_dynamics_sample, buffer_size> buffer;

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

    SemaphoreHandle_t mutex_inverse_dyn_network;
    SemaphoreHandle_t mutex_training_buffer;

    const float max_angle = 180 * DEG2RAD;
    const float inv_max_angle = 1.0 / max_angle;



};

#endif // !INVERSE_DYNAMICS_LEARNER_H
