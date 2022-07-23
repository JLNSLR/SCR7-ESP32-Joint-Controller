#ifndef INVERSE_DYNAMICS_LEARNER_H
#define INVERSE_DYNAMICS_LEARNER_H

#include <drive_system.h>
#include <Arduino.h>
#include <drive_system_types.h>
#include <NN/nn_utils.h>
#include <NN/NeuralNetwork.h>
#include <drive_system_settings.h>
#include <CircularBuffer.h>

struct dynamics_sample {

    // t-1
    float joint_pos_prev_norm;
    float joint_vel_prev_norm;
    float joint_acc_prev_norm;
    float motor_pos_prev_norm;
    float motor_vel_prev_norm;
    float motor_acc_prev_norm;
    float motor_torque_prev_norm;
    float joint_torque_prev_norm;
    // t
    float joint_pos_norm;
    float joint_vel_norm;
    float joint_acc_norm;
    float motor_pos_norm;
    float motor_vel_norm;
    float motor_acc_norm;
    float motor_torque_norm;
    float joint_torque_norm;

    // targets
    float joint_target_pos_norm;
    float joint_target_vel_norm;
    float joint_target_acc_norm;

};

class InverseDynamicsLearner {

public:
    InverseDynamicsLearner();
    void init();
    float predict_torque(const drvSys_FullDriveState state_rad, const drvSys_driveTargets targets);
    void set_scale(float max_motor_torque, float max_vel, float max_acc, float max_joint_torque);
    void add_data(const drvSys_FullDriveStateExt states, const drvSys_driveTargets targets);
    bool forward_dyn_learning_step();
    bool inverse_dyn_learning_step();
    drvSys_driveState predict_joint_kinematic(const drvSys_FullDriveStateExt states);

    void save_parameters_on_flash();


    static const int depth = 4;
    int width[depth] = { 10,8,6,1 };
    nn_activation_f activation[depth - 1] = { leakyReLu, leakyReLu, Linear };

    NeuralNetwork* nn;

    NeuralNetwork* forward_model_nn;
    static const int depth_forward = 4;
    int width_forward[depth_forward] = { 8,12,8,3 };
    nn_activation_f activation_forward[depth_forward - 1] = { leakyReLu, leakyReLu, Linear };

    float control_error;





private:
    static const int buffer_size = 50;

    CircularBuffer<dynamics_sample, buffer_size> buffer;

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
    SemaphoreHandle_t mutex_forward_dyn_network;
    SemaphoreHandle_t mutex_training_buffer;

    const float max_angle = 180 * DEG2RAD;
    const float inv_max_angle = 1.0 / max_angle;





};

#endif // !INVERSE_DYNAMICS_LEARNER_H
