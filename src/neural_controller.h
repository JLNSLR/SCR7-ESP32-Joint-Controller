#ifndef NEURAL_CONTROLLER_H
#define NEURAL_CONTROLLER_H


#include <NN/NeuralNetwork.h>
#include <NN/nn_utils.h>
#include <drive_system_types.h>
#include <Arduino.h>
#include <CircularBuffer.h>
#include <drive_system_settings.h>
#include <drive_system.h>


#define BUFFERSIZE 10
#define EMULATOR
#define INVERSE_DYN


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

union pid_tune_sample {
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

        float pos_prev_error;
        float pos_iTerm;
        float vel_iTerm;
    }data;
    float inputs[10];
};

struct full_neural_control_sample {
    drvSys_FullDriveStateTimeSample state_sample;
    drvSys_driveTargets target_sample;
};

struct cascade_gains {
    float pos_Kp;
    float pos_Ki;
    float pos_Kd;
    float vel_Kp;
    float vel_Ki;
};


class NeuralController {

public:
    NeuralController();
    void init();
    void add_sample(drvSys_FullDriveStateTimeSample sample, drvSys_driveTargets targets);
    void learning_step_emulator();
    drvSys_driveState emulator_predict_next_state(drvSys_FullDriveState current_state);

    float predict_control_torque(drvSys_FullDriveState current_state, drvSys_driveTargets targets);
    float inverse_dynamics_predict_torque(drvSys_FullDriveState current_state, drvSys_driveTargets targets);
    void learning_step_controller();

    void learning_step_inverse_dyn();

    void save_emulator_network();
    void save_controller_network();
    void reset_emulator_network();
    void reset_controller_network();

    void init_pid_learner(const PIDController pos_controller, const PIDController vel_controller);
    void add_pid_sample(drvSys_FullDriveState current_state, drvSys_driveTargets targets, float pos_err_sum, float pos_prev_err, float vel_err_sum);
    void learning_step_pid_tuner();
    cascade_gains predict_gains(drvSys_FullDriveState current_state, drvSys_driveTargets targets);



    NeuralNetwork* emulator_nn;
    NeuralNetwork* controller_nn;
    NeuralNetwork* inverse_dyn_nn;
    NeuralNetwork* adaptive_pid_nn;
    float emulator_error = 0;
    float average_emulator_error = 1e1;
    float average_control_error = 1e1;
    float control_error = 0;
    bool control_net_pretrained = false;
    bool pid_net_pretrained = false;
    bool emu_net_pretrained = false;
    float control_effort_penalty = 1.0;

    float pid_control_error = 0;
    float average_pid_control_error = 0;


    float inv_dyn_error = 0;


private:

    long emulator_counter = 0;
    static const int emulator_depth = 4;
    int emulator_width[emulator_depth] = { 8,12,5,3 };
    nn_activation_f emulator_act[emulator_depth - 1] = { leakyReLu,leakyReLu,Linear };

    CircularBuffer<full_neural_control_sample, BUFFERSIZE> training_buffer;
    CircularBuffer<pid_tune_sample, 10> pid_training_buffer;

    emulator_sample get_emulator_sample(drvSys_FullDriveStateTimeSample data);

    SemaphoreHandle_t mutex_training_buffer;
    SemaphoreHandle_t mutex_emulator;
    SemaphoreHandle_t mutex_controller;
    SemaphoreHandle_t mutex_inv_dyn;

    SemaphoreHandle_t mutex_pid_training_buffer;
    SemaphoreHandle_t mutex_pid_net;

    char emulator_name[7] = "emu_nn";
    char controller_name[9] = "contr_nn";
    char pid_tuner_name[7] = "pid_nn";
    char inv_name[7] = "inv_nn";



    static const int controller_depth = 4;
    int controller_width[controller_depth] = { 10,12,6,1 };
    nn_activation_f controller_act[controller_depth - 1] = { leakyReLu,leakyReLu,Linear };

    static const int inv_depth = 4;
    int inv_width[inv_depth] = { 10,12,6,1 };
    nn_activation_f inv_act[inv_depth - 1] = { leakyReLu,leakyReLu,Linear };

    static const int pid_adapt_depth = 4;
    int pid_adapt_width[pid_adapt_depth] = { 10,10,5,5 };
    nn_activation_f pid_adapt_act[pid_adapt_depth - 1] = { leakyReLu,leakyReLu,ReLu };

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

    PIDController pos_controller;
    PIDController vel_controller;


};



#endif // NEURAL_CONTROLLER_H