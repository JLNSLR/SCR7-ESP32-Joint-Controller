#include "neural_controller.h"


NeuralController::NeuralController() {

};

void NeuralController::init() {
mutex_training_buffer = xSemaphoreCreateBinary();
xSemaphoreGive(mutex_training_buffer);

#ifdef EMULATOR
    emulator_nn = new NeuralNetwork(emulator_depth, emulator_width, emulator_act);
    emulator_nn->loss_type = huber_loss;
    emulator_nn->learning_rate = 1e-4;
    emulator_nn->lr_schedule = no_schedule;
    emulator_nn->update_rule = adam;
    emulator_nn->lr_error_factor = 5e-4;
    emulator_nn->minimal_learning_rate = 1e-4;
    emulator_nn->maximal_learning_rate = 1e-2;
    emulator_nn->init_weights_randomly(0.3, -0.3);
    emulator_nn->max_gradient_abs = 0.1;
    emulator_nn->regularization = none;
    emulator_nn->reg_penalty_factor = 1e-4;


    
    mutex_emulator = xSemaphoreCreateBinary();

    //load weights from Flash
    nn_model_weights emulator_weights = nn_load_model_weights_from_flash(emulator_name, emulator_nn->n_weights);

    if (emulator_weights.n_weights == emulator_nn->n_weights) {
        emulator_nn->load_model_weights(emulator_weights);
        Serial.println("DRVSYS_NN_INFO: Loaded Weights for Emulator Network from Flash");
    }

#endif


    // Initializing neural controller

    controller_nn = new NeuralNetwork(controller_depth, controller_width, controller_act);
    controller_nn->loss_type = MSE;
    controller_nn->learning_rate = 0.5e-3;
    controller_nn->lr_schedule = error_adaptive;
    controller_nn->update_rule = adam;
    controller_nn->lr_error_factor = 1e-4;
    controller_nn->minimal_learning_rate = 1e-5;
    controller_nn->maximal_learning_rate = 0.5e-2;
    controller_nn->init_weights_randomly(0.005, -0.005);
    controller_nn->max_gradient_abs = 1.0;
    controller_nn->regularization = none;
    controller_nn->reg_penalty_factor = 1e-4;


    mutex_controller = xSemaphoreCreateBinary();

    xSemaphoreGive(mutex_emulator);
    xSemaphoreGive(mutex_training_buffer);

    //load weights from Flash
    nn_model_weights controller_weights = nn_load_model_weights_from_flash(controller_name, controller_nn->n_weights);

    if (controller_weights.n_weights == controller_nn->n_weights) {
        controller_nn->load_model_weights(controller_weights);
        Serial.println("DRVSYS_NN_INFO: Loaded Weights for Controller Network from Flash");
        control_net_pretrained = true;
    }


#ifdef INVERSE_DYN
    xSemaphoreGive(mutex_controller);

    // Initializing neural controller

    inverse_dyn_nn = new NeuralNetwork(inv_depth, inv_width, inv_act);
    inverse_dyn_nn->loss_type = huber_loss;
    inverse_dyn_nn->learning_rate = 1e-4;
    inverse_dyn_nn->lr_schedule = no_schedule;
    inverse_dyn_nn->update_rule = adam;
    inverse_dyn_nn->lr_error_factor = 1e-4;
    inverse_dyn_nn->minimal_learning_rate = 1e-4;
    inverse_dyn_nn->maximal_learning_rate = 1e-2;
    inverse_dyn_nn->init_weights_randomly(0.05, -0.05);
    inverse_dyn_nn->max_gradient_abs = 0.1;
    inverse_dyn_nn->regularization = none;
    inverse_dyn_nn->reg_penalty_factor = 1e-4;


    mutex_inv_dyn = xSemaphoreCreateBinary();



    //load weights from Flash
    /*
    nn_model_weights inv_dyn_weights = nn_load_model_weights_from_flash(inv_name, inverse_dyn_nn->n_weights);

    if (inv_dyn_weights.n_weights == inverse_dyn_nn->n_weights) {
        inverse_dyn_nn->load_model_weights(inv_dyn_weights);
        Serial.println("DRVSYS_NN_INFO: Loaded Weights for Inverse Dynamics from Flash");
    }

    xSemaphoreGive(mutex_inv_dyn);
    */
#endif

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

#ifndef EMULATOR
    return;

#endif // !EMULATOR

    if (training_buffer.isEmpty()) {
        return;
    }
    else {

        current_sample = get_emulator_sample(training_buffer.last().state_sample);



        float input[8];
        input[0] = current_sample.data.joint_pos;
        input[1] = current_sample.data.joint_vel;
        input[2] = current_sample.data.joint_acc;
        input[3] = current_sample.data.motor_pos;
        input[4] = current_sample.data.motor_vel;
        input[5] = current_sample.data.motor_acc;
        input[6] = current_sample.data.motor_torque;
        input[7] = current_sample.data.joint_torque;

        float output[3];
        output[0] = current_sample.data.joint_pos_next;
        output[1] = current_sample.data.joint_vel_next;
        output[2] = current_sample.data.joint_acc_next;

        xSemaphoreTake(mutex_emulator, portMAX_DELAY);
        emulator_error = emulator_nn->train_SGD(input, output);
        xSemaphoreGive(mutex_emulator);
        emulator_counter++;

        average_emulator_error = emulator_error * (1e-3) + average_emulator_error * (1 - 1e-3);



    }
}


emulator_sample NeuralController::get_emulator_sample(drvSys_FullDriveStateTimeSample data) {

    emulator_sample emulator_data;

    emulator_data.data.joint_pos = data.state_prev.joint_pos * inv_max_angle;
    emulator_data.data.joint_vel = data.state_prev.joint_vel * inv_max_vel;
    emulator_data.data.joint_acc = data.state_prev.joint_acc * inv_max_acc;
    emulator_data.data.motor_pos = data.state_prev.motor_pos * inv_max_angle;
    emulator_data.data.motor_vel = data.state_prev.motor_vel * inv_max_vel;
    emulator_data.data.motor_acc = data.state_prev.motor_acc * inv_max_acc;

    emulator_data.data.motor_torque = data.state_prev.motor_torque * inv_max_motor_torque;
    emulator_data.data.joint_torque = data.state_prev.joint_torque * inv_max_joint_torque;

    emulator_data.data.joint_pos_next = data.state.joint_pos * inv_max_angle;
    emulator_data.data.joint_vel_next = data.state.joint_vel * inv_max_vel;
    emulator_data.data.joint_acc_next = data.state.joint_acc * inv_max_acc;
#ifdef NN_CONTROL_DEBUG
    Serial.println(emulator_data.arrays.inputs[0]);
    Serial.println(emulator_data.data.joint_pos);
    Serial.println(emulator_data.data.joint_vel);
    Serial.println(emulator_data.arrays.inputs[1]);
    Serial.println("vel data");
    Serial.println(emulator_data.data.joint_pos_next);
    Serial.println(emulator_data.arrays.outputs[0]);
    Serial.println(emulator_data.data.motor_vel);
    Serial.println(emulator_data.arrays.inputs[4]);
    Serial.println("..");

    Serial.println(emulator_data.arrays.outputs[2]);
    Serial.println(emulator_data.data.joint_acc_next);
#endif // NN_CONTROL_DEBUG

    return emulator_data;

}

drvSys_driveState NeuralController::emulator_predict_next_state(drvSys_FullDriveState current_state) {


    drvSys_driveState pred_state;


    float inputs[8];
    inputs[0] = current_state.joint_pos * inv_max_angle;
    inputs[1] = current_state.joint_vel * inv_max_vel;
    inputs[2] = current_state.joint_acc * inv_max_acc;
    inputs[3] = current_state.motor_pos * inv_max_angle;
    inputs[4] = current_state.motor_vel * inv_max_vel;
    inputs[5] = current_state.motor_acc * inv_max_acc;
    inputs[6] = current_state.motor_torque * inv_max_motor_torque;
    inputs[7] = current_state.joint_torque * inv_max_joint_torque;
    /*
    Serial.println(inputs[0]);
    Serial.println(inputs[1]);
    Serial.println(inputs[2]);
    Serial.println(inputs[3]);
    Serial.println(inputs[4]);
    Serial.println(inputs[5]);
    Serial.println(inputs[6]);
    Serial.println(inputs[6]);
    */


    xSemaphoreTake(mutex_emulator, portMAX_DELAY);

    float* outputs = emulator_nn->predict(inputs);
    pred_state.joint_pos = outputs[0] * max_angle;
    pred_state.joint_vel = outputs[1] * max_vel;
    pred_state.joint_acc = outputs[2] * max_acc;



    xSemaphoreGive(mutex_emulator);




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


        float input_vector[10];
        input_vector[0] = sample.state_sample.state_prev.joint_pos * inv_max_angle;
        input_vector[1] = sample.state_sample.state_prev.joint_vel * inv_max_vel;
        input_vector[2] = sample.state_sample.state_prev.joint_acc * inv_max_acc;
        input_vector[3] = sample.state_sample.state_prev.motor_pos * inv_max_angle;
        input_vector[4] = sample.state_sample.state_prev.motor_vel * inv_max_vel;
        input_vector[5] = sample.state_sample.state_prev.motor_acc * inv_max_acc;
        input_vector[6] = sample.state_sample.state_prev.joint_torque * inv_max_joint_torque;
        input_vector[7] = sample.target_sample.pos_target * inv_max_angle;
        input_vector[8] = sample.target_sample.vel_target * inv_max_vel;
        input_vector[9] = sample.target_sample.acc_target * inv_max_acc;

        control_error = abs((sample.state_sample.state.joint_pos - sample.target_sample.pos_target)) +
            abs((sample.state_sample.state.joint_vel - sample.target_sample.vel_target));
        xSemaphoreTake(mutex_controller, portMAX_DELAY);

        float output_torque_norm = *controller_nn->predict(input_vector);

        float derror_du = -10000*sample.state_sample.drvSys_feedback_torque * inv_max_motor_torque + output_torque_norm * control_effort_penalty + -control_error*10 ;

        controller_nn->backpropagation(input_vector, control_error, &derror_du);

        controller_nn->apply_gradient_descent(adam);

        xSemaphoreGive(mutex_controller);

        average_control_error = average_control_error * (1 - 1e-4) + control_error * 1e-4;


    }
}

float NeuralController::predict_control_torque(drvSys_FullDriveState current_state, drvSys_driveTargets targets) {

    static float last_pred = 0;
    float pred_torque = 0;

    float input[10];
    input[0] = current_state.joint_pos * inv_max_angle;
    input[1] = current_state.joint_vel * inv_max_vel;
    input[2] = current_state.joint_acc * inv_max_acc;
    input[3] = current_state.motor_pos * inv_max_angle;
    input[4] = current_state.motor_vel * inv_max_vel;
    input[5] = current_state.motor_acc * inv_max_acc;
    input[6] = current_state.joint_torque * inv_max_joint_torque;
    input[7] = targets.pos_target * inv_max_angle;
    input[8] = targets.vel_target * inv_max_vel;
    input[9] = targets.acc_target * inv_max_acc;

    if (xSemaphoreTake(mutex_controller, (TickType_t)1) == pdTRUE) {
        pred_torque = *controller_nn->predict(input) * max_motor_torque;

        last_pred = pred_torque;
        xSemaphoreGive(mutex_controller);

    }
    else {
        pred_torque = last_pred;
    }

    return pred_torque;


}

void NeuralController::learning_step_inverse_dyn() {

#ifndef INVERSE_DYNAMICS
    return;
#endif // !INVERSE_DYNAMICS
    if (training_buffer.isEmpty()) {
        return;
    }
    else {
        xSemaphoreTake(mutex_training_buffer, portMAX_DELAY);
        full_neural_control_sample sample = training_buffer.last();
        xSemaphoreGive(mutex_training_buffer);


        float input_vector[10];
        input_vector[0] = sample.state_sample.state_prev.joint_pos * inv_max_angle;
        input_vector[1] = sample.state_sample.state_prev.joint_vel * inv_max_vel;
        input_vector[2] = sample.state_sample.state_prev.joint_acc * inv_max_acc;
        input_vector[3] = sample.state_sample.state_prev.motor_pos * inv_max_angle;
        input_vector[4] = sample.state_sample.state_prev.motor_vel * inv_max_vel;
        input_vector[5] = sample.state_sample.state_prev.motor_acc * inv_max_acc;
        input_vector[6] = sample.state_sample.state_prev.joint_torque * inv_max_joint_torque;
        // take next state data
        input_vector[7] = sample.state_sample.state.joint_pos * inv_max_angle;
        input_vector[8] = sample.state_sample.state.joint_vel * inv_max_vel;
        input_vector[9] = sample.state_sample.state.joint_acc * inv_max_acc;

        float output = sample.state_sample.state_prev.motor_torque * inv_max_motor_torque;

        xSemaphoreTake(mutex_inv_dyn, portMAX_DELAY);
        inv_dyn_error = inverse_dyn_nn->train_SGD(input_vector, &output);
        xSemaphoreGive(mutex_inv_dyn);


    }
}

float NeuralController::inverse_dynamics_predict_torque(drvSys_FullDriveState current_state, drvSys_driveTargets targets) {
    static float last_pred = 0;

    float pred_torque = 0;

    float input[10];
    input[0] = current_state.joint_pos * inv_max_angle;
    input[1] = current_state.joint_vel * inv_max_vel;
    input[2] = current_state.joint_acc * inv_max_acc;
    input[3] = current_state.motor_pos * inv_max_angle;
    input[4] = current_state.motor_vel * inv_max_vel;
    input[5] = current_state.motor_acc * inv_max_acc;
    input[6] = current_state.joint_torque * inv_max_joint_torque;
    input[7] = targets.pos_target * inv_max_angle;
    input[8] = targets.vel_target * inv_max_vel;
    input[9] = targets.acc_target * inv_max_acc;

    if (xSemaphoreTake(mutex_inv_dyn, (TickType_t)1) == pdTRUE) {
        pred_torque = *inverse_dyn_nn->predict(input) * max_motor_torque;

        last_pred = pred_torque;
        xSemaphoreGive(mutex_inv_dyn);

    }
    else {
        pred_torque = last_pred;
    }

    return pred_torque;
}

void NeuralController::save_controller_network() {


    nn_model_weights current_weights = controller_nn->get_model_weights();

    nn_save_model_weights_on_flash(current_weights, controller_name);

    Serial.println("DRVSYS_NN: Saved Controller Network Weights on Flash");

}

void NeuralController::reset_controller_network() {

    nn_clear_data_on_flash(controller_name);
    Serial.println("DRVSYS_NN: Reset Controller Network Weights on Flash");
}

void NeuralController::reset_emulator_network() {
    nn_clear_data_on_flash(emulator_name);
    Serial.println("DRVSYS_NN: Reset Emulator Network Weights on Flash");
}






