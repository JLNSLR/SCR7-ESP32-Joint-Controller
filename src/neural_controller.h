#ifndef NEURAL_CONTROLLER_H
#define NEURAL_CONTROLLER_H
#include <NN/NeuralNetwork.h>
#include <NN/nn_utils.h>
#include <drive_system_types.h>
#include <Arduino.h>
#include <CircularBuffer.h>
#include <drive_system_settings.h>


#define BUFFERSIZE 10


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
    float inputs[8];
    float outputs[3];
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
struct full_neural_control_sample {
    drvSys_FullDriveStateTimeSample state_sample;
    drvSys_driveTargets target_sample;
};

struct pid_tune_sample {
    drvSys_FullDriveState state;
    drvSys_driveTargets;
    float error;
    float error_sum;
    float dError;
};

class NeuralController {

public:
    NeuralController();
    void init();
    void add_sample(drvSys_FullDriveStateTimeSample sample, drvSys_driveTargets targets);
    void learning_step_emulator();
    drvSys_driveState emulator_predict_next_state(drvSys_FullDriveState current_state);

    float predict_control_torque(drvSys_FullDriveState current_state, drvSys_driveTargets targets);
    void learning_step_controller();

    void save_emulator_network();
    void save_controller_network();
    NeuralNetwork* emulator_nn;
    NeuralNetwork* controller_nn;
    float emulator_error = 0;
    float average_emulator_error = 0;

    float average_control_error = 0;
    float control_error = 0;


    NeuralNetwork* inverse_dynamic_control_nn;



private:

    long emulator_counter = 0;
    static const int emulator_depth = 4;
    int emulator_width[emulator_depth] = { 8,12,5,3 };
    nn_activation_f emulator_act[emulator_depth - 1] = { leakyReLu,leakyReLu,Linear };

    CircularBuffer<full_neural_control_sample, BUFFERSIZE> training_buffer;

    emulator_sample get_emulator_sample(drvSys_FullDriveStateTimeSample data);

    SemaphoreHandle_t mutex_training_buffer;
    SemaphoreHandle_t mutex_emulator;
    SemaphoreHandle_t mutex_controller;

    char emulator_name[7] = "emu_nn";
    char controller_name[9] = "contr_nn";

    static const int controller_depth = 4;
    int controller_width[controller_depth] = { 10,12,6,1 };
    nn_activation_f controller_act[controller_depth - 1] = { leakyReLu,leakyReLu,Linear };


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

    // PID Tuner

    CircularBuffer<pid_tune_sample, 5> tuning_buffer;






};



#endif // NEURAL_CONTROLLER