#include "neural_controller.h"


NeuralController::NeuralController() {

};

void NeuralController::init() {

    emulator_nn = new NeuralNetwork(emulator_depth, emulator_width, emulator_act);
    emulator_nn->loss_type = MSE;
    emulator_nn->learning_rate = 1e-3;
    emulator_nn->lr_schedule = no_schedule;
    emulator_nn->update_rule = adam;
    emulator_nn->lr_error_factor = 5e-4;
    emulator_nn->minimal_learning_rate = 1e-4;
    emulator_nn->init_weights_randomly(0.5, -0.3);
    emulator_nn->max_gradient_abs = 10.0;
    emulator_nn->regularization = none;
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
    controller_nn->learning_rate = 1e-2;
    controller_nn->lr_schedule = no_schedule;
    controller_nn->update_rule = adam;
    controller_nn->lr_error_factor = 5e-4;
    controller_nn->minimal_learning_rate = 1e-4;
    controller_nn->init_weights_randomly(0.001, -0.001);
    controller_nn->max_gradient_abs = 0.1;
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
    }


    xSemaphoreGive(mutex_controller);


    // Initialize PID Tuner Network

    pid_tuner_nn = new NeuralNetwork(pid_tune_depth, pid_tune_width, pid_tune_act);
    pid_tuner_nn->loss_type = MSE;
    pid_tuner_nn->learning_rate = 1e-3;
    pid_tuner_nn->lr_schedule = error_adaptive;
    pid_tuner_nn->update_rule = adam;
    pid_tuner_nn->lr_error_factor = 5e-4;
    pid_tuner_nn->minimal_learning_rate = 1e-3;
    pid_tuner_nn->init_weights_randomly(1.5, -0.5);
    pid_tuner_nn->max_gradient_abs = 10.0;
    pid_tuner_nn->regularization = none;
    pid_tuner_nn->reg_penalty_factor = 1e-4;


    mutex_pid_tuner_nn = xSemaphoreCreateBinary();
    mutex_pid_sample_buffer = xSemaphoreCreateBinary();



    //load weights from Flash
    nn_model_weights tuner_weights = nn_load_model_weights_from_flash(pid_tuner_name, pid_tuner_nn->n_weights);

    if (tuner_weights.n_weights == pid_tuner_nn->n_weights) {
        pid_tuner_nn->load_model_weights(tuner_weights);
        Serial.println("DRVSYS_NN_INFO: Loaded Weights for PID Tuner Network from Flash");
    }

    init_gains.K_p = PID_GAIN_P;
    init_gains.K_i = PID_GAIN_I;
    init_gains.K_d = PID_GAIN_D;
    init_gains.K_vel_ff = PID_VEL_FF;

    xSemaphoreGive(mutex_pid_tuner_nn);
    xSemaphoreGive(mutex_pid_sample_buffer);



}


void NeuralController::add_sample(drvSys_FullDriveStateTimeSample sample, drvSys_driveTargets target) {

    full_neural_control_sample sample_data;
    sample_data.state_sample = sample;
    sample_data.target_sample = target;

    xSemaphoreTake(mutex_training_buffer, portMAX_DELAY);
    training_buffer.push(sample_data);
    xSemaphoreGive(mutex_training_buffer);
}

void NeuralController::add_pid_sample(drvSys_FullDriveState state, drvSys_driveTargets target, float error, float Iterm, float dError) {

    pid_tune_sample sample;
    sample.state = state;
    sample.error = error;
    sample.error_sum = Iterm;
    sample.dError = dError;
    sample.targets = target;

    xSemaphoreTake(mutex_pid_sample_buffer, portMAX_DELAY);
    tuning_buffer.push(sample);
    xSemaphoreGive(mutex_pid_sample_buffer);
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
    inputs[0] = current_state.joint_pos * inv_max_angle;
    inputs[1] = current_state.joint_vel * inv_max_vel;
    inputs[2] = current_state.joint_acc * inv_max_acc;
    inputs[3] = current_state.motor_pos * inv_max_angle;
    inputs[4] = current_state.motor_vel * inv_max_vel;
    inputs[5] = current_state.motor_acc * inv_max_acc;
    inputs[6] = current_state.motor_torque * inv_max_motor_torque;
    inputs[7] = current_state.joint_torque * inv_max_joint_torque;
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


        // simulate system output  

        float epsilon = 1e-3;


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

        xSemaphoreTake(mutex_controller, portMAX_DELAY);

        float output_torque = *controller_nn->predict(input_vector) * max_motor_torque;

        drvSys_FullDriveState state_plus_epsilon = sample.state_sample.state_prev;
        state_plus_epsilon.motor_torque = output_torque + epsilon;

        drvSys_FullDriveState state_min_epsilon = sample.state_sample.state_prev;
        state_min_epsilon.motor_torque = output_torque - epsilon;

        drvSys_driveState output_state_plus_epsilon = emulator_predict_next_state(state_plus_epsilon);
        drvSys_driveState output_state_min_epsilon = emulator_predict_next_state(state_min_epsilon);

        float dpos_du = (output_state_plus_epsilon.joint_pos - output_state_min_epsilon.joint_pos) / (2.0 * epsilon);
        float dvel_du = (output_state_plus_epsilon.joint_vel - output_state_min_epsilon.joint_vel) / (2.0 * epsilon);

        drvSys_FullDriveState state_sim = sample.state_sample.state_prev;
        state_sim.motor_torque = output_torque;
        drvSys_driveState emulator_out = emulator_predict_next_state(state_sim);

        float pos_error = emulator_out.joint_pos - sample.target_sample.pos_target;
        float vel_error = emulator_out.joint_vel - sample.target_sample.vel_target;

        control_error = (pos_error * pos_error + vel_error * vel_error) * 0.5;

        float derror_du = pos_error * dpos_du * inv_max_angle + vel_error * dvel_du * inv_max_vel + 2.0 * output_torque * inv_max_motor_torque;

        controller_nn->backpropagation(input_vector, control_error, &derror_du);

        controller_nn->apply_gradient_descent();

        xSemaphoreGive(mutex_controller);
        average_control_error = control_error * (0.1) + average_control_error * (1 - 0.1);

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
        xSemaphoreGive(mutex_controller);
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


drvSys_PID_Gains NeuralController::predict_optimal_gains(drvSys_FullDriveState state, drvSys_driveTargets targets) {

    float input[10];
    input[0] = state.joint_pos * inv_max_angle;
    input[1] = state.joint_vel * inv_max_vel;
    input[2] = state.joint_acc * inv_max_acc;
    input[3] = state.motor_pos * inv_max_angle;
    input[4] = state.motor_vel * inv_max_vel;
    input[5] = state.motor_acc * inv_max_acc;
    input[6] = state.joint_torque * inv_max_joint_torque;
    input[7] = targets.pos_target * inv_max_angle;
    input[8] = targets.vel_target * inv_max_vel;
    input[9] = targets.acc_target * inv_max_acc;

    //Serial.println("want to pred gains");

    xSemaphoreTake(mutex_pid_tuner_nn, portMAX_DELAY);
    float* outputs = pid_tuner_nn->predict(input);

    //Serial.println("try to read gains");

    opt_gains.K_p = outputs[0];
    opt_gains.K_i = outputs[1];
    opt_gains.K_d = outputs[2];
    //Serial.println("got gains");

    opt_gains.K_vel_ff = init_gains.K_vel_ff;

    init_gains = opt_gains;
    xSemaphoreGive(mutex_pid_tuner_nn);



    return opt_gains;

}

void NeuralController::learning_step_pid_tuner() {

    if (tuning_buffer.isEmpty()) {
        return;
    }
    else {
        pid_tune_sample sample = tuning_buffer.pop();


        float input[10];
        input[0] = sample.state.joint_pos * inv_max_angle;
        input[1] = sample.state.joint_vel * inv_max_vel;
        input[2] = sample.state.joint_acc * inv_max_acc;
        input[3] = sample.state.motor_pos * inv_max_angle;
        input[4] = sample.state.motor_vel * inv_max_vel;
        input[5] = sample.state.motor_acc * inv_max_acc;
        input[6] = sample.state.joint_torque * inv_max_joint_torque;
        input[7] = sample.targets.pos_target * inv_max_angle;
        input[8] = sample.targets.vel_target * inv_max_vel;
        input[9] = sample.targets.acc_target * inv_max_acc;

        xSemaphoreTake(mutex_pid_tuner_nn, portMAX_DELAY);
        float* gains = pid_tuner_nn->predict(input);


        // construct pid output torque

        float output_torque = gains[0] * sample.error + gains[1] * sample.error_sum * pid_sample_time_in_sec + gains[2] * sample.dError * (1.0 / pid_sample_time_in_sec);

        drvSys_FullDriveState sim_state;
        sim_state = sample.state;
        sim_state.motor_torque = output_torque;

        float epsilon = 1e-3;

        drvSys_FullDriveState sim_state_plus_eps = sim_state;
        drvSys_FullDriveState sim_state_min_eps = sim_state;
        sim_state_plus_eps.motor_torque = output_torque + epsilon;
        sim_state_min_eps.motor_torque = output_torque - epsilon;

        drvSys_driveState state_sim_output = emulator_predict_next_state(sim_state);

        drvSys_driveState state_plus_eps = emulator_predict_next_state(sim_state_plus_eps);
        drvSys_driveState state_min_eps = emulator_predict_next_state(sim_state_min_eps);

        float pos_error = state_sim_output.joint_pos - sample.targets.pos_target;
        float vel_error = state_sim_output.joint_vel - sample.targets.vel_target;

        pid_tune_control_error = (pos_error * pos_error + vel_error * vel_error) * 0.5;

        float dpos_du = (state_plus_eps.joint_pos - state_min_eps.joint_pos) / (2.0 * epsilon);
        float dvel_du = (state_plus_eps.joint_vel - state_min_eps.joint_vel) / (2.0 * epsilon);

        float du_dKp = sample.error;
        float du_dKi = sample.error_sum * pid_sample_time_in_sec;
        float du_dKd = sample.dError * (1.0 / pid_sample_time_in_sec);


        float derror_dKp = du_dKp * (pos_error * dpos_du + vel_error * dvel_du);
        float derror_dKi = du_dKi * (pos_error * dpos_du + vel_error * dvel_du);
        float derror_dKd = du_dKd * (pos_error * dpos_du + vel_error * dvel_du);

        float error_derivatives[3] = { derror_dKp, derror_dKi, derror_dKd };

        pid_tuner_nn->backpropagation(input, pid_tune_control_error, error_derivatives);

        pid_tuner_nn->apply_gradient_descent();

        xSemaphoreGive(mutex_pid_tuner_nn);

        average_pid_tune_control_error = pid_tune_control_error * (0.1) + average_pid_tune_control_error * (1 - 0.1);

    }

}




