#include "neural_controller.h"


NeuralController::NeuralController() {

};

void NeuralController::init() {

    emulator_nn = new NeuralNetwork(emulator_depth, emulator_width, emulator_act);
    emulator_nn->loss_type = MSE;
    emulator_nn->learning_rate = 1e-3;
    emulator_nn->lr_schedule = error_adaptive_filtered;
    emulator_nn->update_rule = adam;
    emulator_nn->lr_error_factor = 5e-4;
    emulator_nn->minimal_learning_rate = 5e-5;
    emulator_nn->init_weights_randomly(0.5, -0.3);
    emulator_nn->max_gradient_abs = 10.0;
    emulator_nn->regularization = lasso;
    emulator_nn->reg_penalty_factor = 1e-4;


    mutex_training_buffer = xSemaphoreCreateBinary();
    mutex_emulator = xSemaphoreCreateBinary();

    xSemaphoreGive(mutex_emulator);
    xSemaphoreGive(mutex_training_buffer);

    //load weights from Flash
    nn_model_weights emulator_weights = nn_load_model_weights_from_flash(emulator_name, emulator_nn->n_weights);

    if (emulator_weights.n_weights == emulator_nn->n_weights) {
        emulator_nn->load_model_weights(emulator_weights);
        Serial.println("DRVSYS_NN_INFO: Loaded Weights for Emulator Network from Flash");
    }


    // Initializing neural controller

    controller_nn = new NeuralNetwork(controller_depth, controller_width, controller_act);
    controller_nn->loss_type = MSE;
    controller_nn->learning_rate = 1e-3;
    controller_nn->lr_schedule = error_adaptive_filtered;
    controller_nn->update_rule = adam;
    controller_nn->lr_error_factor = 5e-4;
    controller_nn->minimal_learning_rate = 5e-5;
    controller_nn->init_weights_randomly(0.5, -0.3);
    controller_nn->max_gradient_abs = 10.0;
    controller_nn->regularization = lasso;
    controller_nn->reg_penalty_factor = 1e-4;


    mutex_controller = xSemaphoreCreateBinary();

    xSemaphoreGive(mutex_emulator);
    xSemaphoreGive(mutex_training_buffer);

    //load weights from Flash
    nn_model_weights controller_weights = nn_load_model_weights_from_flash(controller_name, emulator_nn->n_weights);

    if (controller_weights.n_weights == controller_nn->n_weights) {
        controller_nn->load_model_weights(emulator_weights);
        Serial.println("DRVSYS_NN_INFO: Loaded Weights for Controller Network from Flash");
    }


    xSemaphoreGive(mutex_controller);



}


void NeuralController::add_sample(drvSys_FullDriveStateTimeSample sample, drvSys_driveTargets target) {

    full_neural_control_sample sample_data;
    sample_data.state_sample = sample;
    sample_data.target_sample = target;

    xSemaphoreTake(mutex_training_buffer, portMAX_DELAY);
    training_buffer.push(sample_data);
    xSemaphoreGive(mutex_training_buffer);
}

void NeuralController::learning_step_emulator() {

    if (training_buffer.isEmpty()) {
        return;
    }
    else {

        emulator_sample sample = get_emulator_sample(training_buffer.last().state_sample);

        xSemaphoreTake(mutex_emulator, portMAX_DELAY);
        emulator_error = emulator_nn->train_SGD(sample.inputs, sample.outputs);
        xSemaphoreGive(mutex_emulator);
        emulator_counter++;

        average_emulator_error = emulator_error * (0.1) + average_emulator_error * (1 - 0.1);



    }
}


emulator_sample NeuralController::get_emulator_sample(drvSys_FullDriveStateTimeSample data) {

    emulator_sample emulator_data;

    emulator_data.data.joint_pos = data.state_prev.joint_pos;
    emulator_data.data.joint_vel = data.state_prev.joint_vel;
    emulator_data.data.joint_acc = data.state_prev.joint_acc;
    emulator_data.data.motor_pos = data.state_prev.motor_pos;
    emulator_data.data.motor_vel = data.state_prev.motor_vel;
    emulator_data.data.motor_acc = data.state_prev.motor_acc;

    emulator_data.data.joint_pos_next = data.state.joint_pos;
    emulator_data.data.joint_vel_next = data.state.joint_vel;
    emulator_data.data.joint_acc_next = data.state.joint_acc;

#ifdef NN_CONTROL_DEBUG
    Serial.println(emulator_data.inputs[0]);
    Serial.println(emulator_data.data.joint_pos);
    Serial.println(emulator_data.data.joint_vel);
    Serial.println(emulator_data.inputs[1]);
    Serial.println("vel data");
    Serial.println(emulator_data.data.joint_vel_next);
    Serial.println(emulator_data.outputs[1]);
    Serial.println(emulator_data.data.motor_vel);
    Serial.println(emulator_data.inputs[4]);
    Serial.println("..");

    Serial.println(emulator_data.outputs[2]);
    Serial.println(emulator_data.data.joint_acc_next);
#endif // NN_CONTROL_DEBUG

    return emulator_data;

}

drvSys_driveState NeuralController::emulator_predict_next_state(drvSys_FullDriveState current_state) {


    drvSys_driveState pred_state;


    float inputs[8];
    inputs[0] = current_state.joint_pos;
    inputs[1] = current_state.joint_vel;
    inputs[2] = current_state.joint_acc;
    inputs[3] = current_state.motor_pos;
    inputs[4] = current_state.motor_vel;
    inputs[5] = current_state.motor_acc;
    inputs[6] = current_state.motor_torque;
    inputs[7] = current_state.joint_torque;
    xSemaphoreTake(mutex_emulator, portMAX_DELAY);

    float* outputs = emulator_nn->predict(inputs);

    xSemaphoreGive(mutex_emulator);


    pred_state.joint_pos = outputs[0];
    pred_state.joint_vel = outputs[1];
    pred_state.joint_acc = outputs[2];

    return pred_state;
}

void NeuralController::save_emulator_network() {

    nn_model_weights current_weights = emulator_nn->get_model_weights();

    nn_save_model_weights_on_flash(current_weights, emulator_name);

    Serial.println("DRVSYS_NN: Saved Emulator Network Weights on Flash");
}


void NeuralController::learning_step_controller() {

    if (training_buffer.isEmpty()) {
        return;
    }
    else {
        xSemaphoreTake(mutex_training_buffer, portMAX_DELAY);
        full_neural_control_sample sample = training_buffer.pop();
        xSemaphoreGive(mutex_training_buffer);


        float pos_error = sample.state_sample.state.joint_pos - sample.target_sample.pos_target;
        float vel_error = sample.state_sample.state.joint_vel - sample.target_sample.vel_target;

        control_error = (pos_error * pos_error + vel_error * vel_error) * 0.5;

        // simulate system output  

        float epsilon = 1e-3;

        drvSys_FullDriveState state_plus_epsilon = sample.state_sample.state_prev;
        state_plus_epsilon.motor_torque += epsilon;

        drvSys_FullDriveState state_min_epsilon = sample.state_sample.state_prev;
        state_min_epsilon.motor_torque -= epsilon;

        drvSys_driveState output_state_plus_epsilon = emulator_predict_next_state(state_plus_epsilon);
        drvSys_driveState output_state_min_epsilon = emulator_predict_next_state(state_min_epsilon);

        // Use Finite Differences to obtain error derivative

        float dpos_du = (output_state_plus_epsilon.joint_pos - output_state_min_epsilon.joint_pos) / (2.0 * epsilon);
        float dvel_du = (output_state_plus_epsilon.joint_vel - output_state_min_epsilon.joint_vel) / (2.0 * epsilon);

        float derror_du = pos_error * dpos_du + vel_error * dvel_du;

        float input_vector[10];
        input_vector[0] = sample.state_sample.state_prev.joint_pos;
        input_vector[1] = sample.state_sample.state_prev.joint_vel;
        input_vector[2] = sample.state_sample.state_prev.joint_acc;
        input_vector[3] = sample.state_sample.state_prev.motor_pos;
        input_vector[4] = sample.state_sample.state_prev.motor_vel;
        input_vector[5] = sample.state_sample.state_prev.motor_acc;
        input_vector[6] = sample.state_sample.state_prev.joint_torque;
        input_vector[7] = sample.target_sample.pos_target;
        input_vector[8] = sample.target_sample.vel_target;
        input_vector[9] = sample.target_sample.acc_target;

        xSemaphoreTake(mutex_controller, portMAX_DELAY);
        controller_nn->train_SGD_ext_loss(input_vector, control_error, &derror_du);
        xSemaphoreGive(mutex_controller);

        average_control_error = control_error * (0.1) + average_control_error * (1 - 0.1);

    }
}

float NeuralController::predict_control_torque(drvSys_FullDriveState current_state, drvSys_driveTargets targets) {



    float input[10];
    input[0] = current_state.joint_pos;
    input[1] = current_state.joint_vel;
    input[2] = current_state.joint_acc;
    input[3] = current_state.motor_pos;
    input[4] = current_state.motor_vel;
    input[5] = current_state.motor_acc;
    input[6] = current_state.joint_torque;
    input[7] = targets.pos_target;
    input[8] = targets.vel_target;
    input[9] = targets.acc_target;

    xSemaphoreTake(mutex_controller, portMAX_DELAY);
    float pred_torque = *controller_nn->predict(input);
    xSemaphoreGive(mutex_controller);

    return pred_torque;


}

void NeuralController::save_controller_network() {


    nn_model_weights current_weights = controller_nn->get_model_weights();

    nn_save_model_weights_on_flash(current_weights, controller_name);

    Serial.println("DRVSYS_NN: Saved Controller Network Weights on Flash");

}




