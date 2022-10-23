#ifndef NEURAL_CONTROLLER_H
#define NEURAL_CONTROLLER_H


#include <NN/NeuralNetwork.h>
#include <NN/nn_utils.h>
#include <drive_system_types.h>
#include <Arduino.h>
#include <CircularBuffer.h>
#include <drive_system_settings.h>
#include <drive_system.h>


#define BUFFERSIZE 20

//#define NN_CONTROL_DEBUG

union emulator_sample {
    struct {
        float joint_pos;
        float joint_vel;
        float joint_acc;
        float motor_pos;
        float motor_vel;
        float motor_acc;
        float motor_torque;
        float joint_torque;

        float joint_pos_next;
        float joint_vel_next;
        float joint_acc_next;
    }data;
    struct {
        float inputs[8];
        float outputs[3];
    }arrays;

};


union inverse_dyn_control_sample {
    struct {
        float joint_pos;
        float joint_vel;
        float joint_acc;
        float motor_pos;
        float motor_vel;
        float motor_acc;
        float joint_torque;

        float joint_pos_target;
        float joint_vel_target;
        float joint_acc_target;

        float control_error;
        float control_error_derivative;

    }data;
    float inputs[10];
};

struct pid_tune_sample {

    float joint_pos;
    float joint_vel;
    float joint_acc;
    float motor_pos;
    float motor_vel;
    float motor_acc;
    float joint_torque;

    float joint_pos_target;
    float joint_vel_target;
    float joint_acc_target;

    float pos_prev_error;
    float pos_iTerm;
    float vel_iTerm;

};

struct full_neural_control_sample {
    drvSys_FullDriveStateTimeSample state_sample;
    drvSys_driveTargets target_sample;
};



class NeuralController {

public:
    NeuralController();
    void init();
    void add_sample(drvSys_FullDriveStateTimeSample sample, drvSys_driveTargets targets);
    void learning_step_emulator();
    drvSys_driveState emulator_predict_next_state(drvSys_FullDriveState current_state);
    float predict_control_torque(drvSys_FullDriveState current_state, drvSys_driveTargets targets);
    void learning_step_error_fb_network();

    void save_emulator_network();
    void save_error_fb_network();
    void save_control_network();
    void reset_emulator_network();
    void reset_error_fb_network();
    void reset_control_network();

    void add_pid_sample(drvSys_FullDriveState current_state, drvSys_driveTargets targets, float pos_err_sum, float pos_prev_err, float vel_err_sum);
    void learning_step_pid_tuner();
    drvSys_cascade_gains predict_gains(drvSys_FullDriveState current_state, drvSys_driveTargets targets);


    NeuralNetwork* emulator_nn;
    NeuralNetwork* error_feedback_nn;
    NeuralNetwork* controller_nn;

    NeuralNetwork* inverse_dynamics_nn;

    bool pid_ff_active = true;
    bool pid_position_control_active = true;
    bool pid_velocity_control_active = true;

    float emulator_error = 0;
    float average_emulator_error = 1;
    float average_control_error = 1;
    float control_error = 0;
    bool error_fb_net_pretrained = false;
    bool pid_net_pretrained = false;
    bool emu_net_pretrained = false;
    float control_effort_penalty = 1.0;

    float pid_nn_regularization = DRVSYS_PIDNN_LEARNING_REGULARIZATION;

    float pid_nn_max_learning_rate = 8e-3;
    float pid_nn_min_learning_rate = 0.5e-6;

    float pid_nn_learning_rate_scale = 10e-3;


    float pid_control_error = 0;
    float average_pid_control_error = 0;

    float inv_dyn_error = 0;


private:

    long emulator_counter = 0;
    static const int emulator_depth = 4;
    int emulator_width[emulator_depth] = { 8,8,5,3 };
    nn_activation_f emulator_act[emulator_depth - 1] = { leakyReLu,leakyReLu,Linear };

    CircularBuffer<full_neural_control_sample, BUFFERSIZE > training_buffer_input;
    CircularBuffer<full_neural_control_sample, BUFFERSIZE> training_buffer;
    CircularBuffer<pid_tune_sample, BUFFERSIZE> pid_training_buffer;
    CircularBuffer<pid_tune_sample, BUFFERSIZE> pid_training_buffer_input;

    emulator_sample get_emulator_sample(drvSys_FullDriveStateTimeSample data);

    SemaphoreHandle_t mutex_training_buffer;
    SemaphoreHandle_t mutex_emulator;
    SemaphoreHandle_t mutex_ff_controller;

    SemaphoreHandle_t mutex_pid_training_buffer;
    SemaphoreHandle_t mutex_control_net;

    char emulator_name[7] = "emu_nn";
    char error_fb_nn_name[9] = "ff_nn";
    char controller_nn_name[7] = "c_nn";


    static const int error_fb_depth = 3;
    int error_fb_width[error_fb_depth] = { 8,16,1 };
    nn_activation_f error_fb_act[error_fb_depth - 1] = { leakyReLu,Linear };


    static const int controller_nn_depth = 3;
    int controller_nn_width[controller_nn_depth] = { 10,12,5 };
    nn_activation_f controller_nn_act[controller_nn_depth - 1] = { leakyReLu,Linear };


    static const int inv_nn_depth = 3;
    int inv_nn_width[inv_nn_depth] = { 10,12,1 };
    nn_activation_f inv_nn_act[inv_nn_depth - 1] = { leakyReLu,Linear };


    pid_tune_sample current_pid_sample;
    emulator_sample current_sample;

    const float max_angle = 180.0 * DEG2RAD;
    const float inv_max_angle = 1.0 / max_angle;
    const float max_vel = 180.0 * DEG2RAD;
    const float inv_max_vel = 1.0 / max_vel;
    const float max_acc = 1000 * DEG2RAD;
    const float inv_max_acc = 1.0 / max_acc;
    const float max_motor_torque = DRVSYS_TORQUE_CONSTANT;
    const float inv_max_motor_torque = 1.0 / max_motor_torque;
    const float max_joint_torque = 100;
    const float inv_max_joint_torque = 1 / max_joint_torque;

    float pos_sample_time_s = DRVSYS_CONTROL_POS_PERIOD_US * 1e-6;
    float vel_sample_time_s = DRVSYS_CONTROL_VEL_PERIOD_US * 1e-6;

    float vel_input_filter_alpha = DRVSYS_VEL_PID_INPUT_FILTER_ALPHA;

    float abs_grad(float x);



};



#endif // NEURAL_CONTROLLER_H