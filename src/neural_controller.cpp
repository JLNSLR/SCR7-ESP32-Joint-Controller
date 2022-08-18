#include "neural_controller.h"

//#define NN_CONTROL_DEBUG


NeuralController::NeuralController() {

};

void NeuralController::init() {
    mutex_training_buffer = xSemaphoreCreateBinary();
    xSemaphoreGive(mutex_training_buffer);

#ifdef EMULATOR
    emulator_nn = new NeuralNetwork(emulator_depth, emulator_width, emulator_act);
    emulator_nn->loss_type = MSE;
    emulator_nn->learning_rate = 1e-2;
    emulator_nn->lr_schedule = no_schedule;
    emulator_nn->update_rule = sgd;
    emulator_nn->lr_error_factor = 5e-4;
    emulator_nn->minimal_learning_rate = 1e-4;
    emulator_nn->maximal_learning_rate = 1e-2;
    emulator_nn->init_weights_randomly(0.5, -0.5);
    emulator_nn->max_gradient_abs = 10.0;
    emulator_nn->regularization = lasso;
    emulator_nn->reg_penalty_factor = 1e-4;



    mutex_emulator = xSemaphoreCreateBinary();

    //load weights from Flash
    nn_model_weights emulator_weights = nn_load_model_weights_from_flash(emulator_name, emulator_nn->n_weights);

    if (emulator_weights.n_weights == emulator_nn->n_weights) {
        emulator_nn->load_model_weights(emulator_weights);
        Serial.println("DRVSYS_NN_INFO: Loaded Weights for Emulator Network from Flash");
        emu_net_pretrained = true;
    }


#endif


    // Initializing neural controller

    controller_nn = new NeuralNetwork(controller_depth, controller_width, controller_act);
    controller_nn->loss_type = MSE;
    controller_nn->learning_rate = 1e-3;
    controller_nn->lr_schedule = no_schedule;
    controller_nn->update_rule = sgd;
    controller_nn->lr_error_factor = 1e-3;
    controller_nn->minimal_learning_rate = 1e-5;
    controller_nn->maximal_learning_rate = 0.5e-2;
    controller_nn->init_weights_randomly(0.03, -0.02);
    controller_nn->max_gradient_abs = 10;
    controller_nn->regularization = none;
    controller_nn->reg_penalty_factor = 1e-3;


    mutex_controller = xSemaphoreCreateBinary();

    xSemaphoreGive(mutex_emulator);
    xSemaphoreGive(mutex_training_buffer);

    //load weights from Flash
    nn_model_weights controller_weights = nn_load_model_weights_from_flash(controller_name, controller_nn->n_weights);

    /*
    if (controller_weights.n_weights == controller_nn->n_weights) {
        controller_nn->load_model_weights(controller_weights);
        Serial.println("DRVSYS_NN_INFO: Loaded Weights for Controller Network from Flash");
        control_net_pretrained = true;
    }
    */

    xSemaphoreGive(mutex_controller);


    /*#ifdef INVERSE_DYN


            // Initializing neural controller

            inverse_dyn_nn = new NeuralNetwork(inv_depth, inv_width, inv_act);
            inverse_dyn_nn->loss_type = huber_loss;
            inverse_dyn_nn->learning_rate = 1e-4;
            inverse_dyn_nn->lr_schedule = no_schedule;
            inverse_dyn_nn->update_rule = adam;
            inverse_dyn_nn->lr_error_factor = 1e-4;
            inverse_dyn_nn->minimal_learning_rate = 1e-4;
            inverse_dyn_nn->maximal_learning_rate = 1e-2;
            inverse_dyn_nn->init_weights_randomly(0.005, -0.05);
            inverse_dyn_nn->max_gradient_abs = 0.1;
            inverse_dyn_nn->regularization = lasso;
            inverse_dyn_nn->reg_penalty_factor = 1e-2;


            mutex_inv_dyn = xSemaphoreCreateBinary();



            //load weights from Flash

            nn_model_weights inv_dyn_weights = nn_load_model_weights_from_flash(inv_name, inverse_dyn_nn->n_weights);

            if (inv_dyn_weights.n_weights == inverse_dyn_nn->n_weights) {
                inverse_dyn_nn->load_model_weights(inv_dyn_weights);
                Serial.println("DRVSYS_NN_INFO: Loaded Weights for Inverse Dynamics from Flash");
            }

            xSemaphoreGive(mutex_inv_dyn);

        #endif
        */

}


void NeuralController::add_sample(drvSys_FullDriveStateTimeSample sample, drvSys_driveTargets target) {

    full_neural_control_sample sample_data;
    sample_data.state_sample = sample;
    sample_data.target_sample = target;

    training_buffer_input.push(sample_data);

    // select random sample from input

    int index = random(0, training_buffer_input.size() - 1);

    xSemaphoreTake(mutex_training_buffer, portMAX_DELAY);
    training_buffer.push(training_buffer_input[index]);
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

        xSemaphoreTake(mutex_training_buffer, portMAX_DELAY);
        current_sample = get_emulator_sample(training_buffer.last().state_sample);
        xSemaphoreGive(mutex_training_buffer);



        float input[8];
        input[0] = current_sample.data.joint_pos;
        input[1] = current_sample.data.joint_vel;
        input[2] = current_sample.data.joint_acc;
        input[3] = current_sample.data.motor_pos;
        input[4] = current_sample.data.motor_vel;
        input[5] = current_sample.data.motor_acc;
        input[6] = current_sample.data.motor_torque;
        input[7] = current_sample.data.joint_torque;

        /*
        Serial.println(input[0]);
        Serial.println(input[1]);
        Serial.println(input[2]);
        Serial.println(input[3]);
        Serial.println(input[4]);
        Serial.println(input[5]);
        Serial.println(input[6]);
        Serial.println(input[7]);
        */

        float output[3];
        output[0] = current_sample.data.joint_pos_next;
        output[1] = current_sample.data.joint_vel_next;
        output[2] = current_sample.data.joint_acc_next;

        /*
        Serial.println(output[0]);
        Serial.println(output[1]);
        Serial.println(output[2]);
        */

        xSemaphoreTake(mutex_emulator, portMAX_DELAY);
        emulator_error = emulator_nn->train_SGD(input, output);

        emulator_counter++;

        average_emulator_error = emulator_error * (1e-3) + average_emulator_error * (1 - 1e-3);
        xSemaphoreGive(mutex_emulator);



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


    static float inputs[8];
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
    Serial.println(inputs[7]);
    */




    xSemaphoreTake(mutex_emulator, portMAX_DELAY);

    static float outputs[3] = { 0 };
    emulator_nn->predict(inputs, outputs);
    pred_state.joint_pos = outputs[0] * max_angle;
    pred_state.joint_vel = outputs[1] * max_vel;
    pred_state.joint_acc = outputs[2] * max_acc;

    /*
    Serial.println(pred_state.joint_pos);
    Serial.println(pred_state.joint_vel);
    Serial.println(pred_state.joint_acc);
    */


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

        float derror_du = -sample.state_sample.drvSys_feedback_torque * inv_max_motor_torque + output_torque_norm;// +2 * output_torque_norm * control_effort_penalty; ;

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

#ifndef INVERSE_DYN
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

        xSemaphoreTake(mutex_controller, portMAX_DELAY);
        inv_dyn_error = controller_nn->train_SGD(input_vector, &output);
        xSemaphoreGive(mutex_controller);


    }
}

/*
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

        //Serial.println("predicted torque");

    }
    else {
        //Serial.println("taking last torque");
        pred_torque = last_pred;
    }


    return pred_torque;
}
*/

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


void NeuralController::init_pid_learner(const PIDController pos_controller, const PIDController vel_controller) {
    // Initializing neural controller

    adaptive_pid_nn = new NeuralNetwork(pid_adapt_depth, pid_adapt_width, pid_adapt_act);
    adaptive_pid_nn->loss_type = MSE;
    adaptive_pid_nn->learning_rate = 1e-4;
    adaptive_pid_nn->lr_schedule = no_schedule;
    adaptive_pid_nn->update_rule = adam;
    adaptive_pid_nn->lr_error_factor = 1e-4;
    adaptive_pid_nn->minimal_learning_rate = 1e-5;
    adaptive_pid_nn->maximal_learning_rate = 0.5e-2;
    adaptive_pid_nn->init_weights_randomly(0.25, 0.0);
    adaptive_pid_nn->max_gradient_abs = 0.1;
    adaptive_pid_nn->regularization = lasso;
    adaptive_pid_nn->reg_penalty_factor = 1e-5;


    mutex_pid_net = xSemaphoreCreateBinary();
    mutex_pid_training_buffer = xSemaphoreCreateBinary();

    xSemaphoreGive(mutex_pid_net);
    xSemaphoreGive(mutex_pid_training_buffer);


    //load weights from Flash
    nn_model_weights adaptive_pid_weights = nn_load_model_weights_from_flash(pid_tuner_name, adaptive_pid_nn->n_weights);

    /*
    if (adaptive_pid_weights.n_weights == adaptive_pid_nn->n_weights) {
        adaptive_pid_nn->load_model_weights(adaptive_pid_weights);
        Serial.println("DRVSYS_NN_INFO: Loaded Weights for PID Network from Flash");
        pid_net_pretrained = true;
    }
    */

}
void NeuralController::add_pid_sample(drvSys_FullDriveState current_state, drvSys_driveTargets targets, float pos_err_sum, float pos_prev_err, float vel_err_sum) {

    pid_tune_sample sample;

    sample.joint_acc = current_state.joint_acc * inv_max_acc;
    sample.joint_pos = current_state.joint_pos * inv_max_angle;
    sample.joint_vel = current_state.joint_vel * inv_max_vel;
    sample.motor_vel = current_state.motor_vel * inv_max_vel;
    sample.motor_pos = current_state.motor_pos * inv_max_angle;
    sample.motor_acc = current_state.motor_acc * inv_max_acc;
    sample.joint_torque = current_state.joint_pos * inv_max_joint_torque;

    sample.joint_pos_target = targets.pos_target * inv_max_angle;
    sample.joint_vel_target = targets.vel_target * inv_max_vel;
    sample.joint_acc_target = targets.acc_target * inv_max_acc;


    sample.pos_prev_error = pos_prev_err;
    sample.pos_iTerm = pos_err_sum;
    sample.vel_iTerm = vel_err_sum;


    // select random sample from input

    //Serial.println("adding PID samples");

    pid_training_buffer_input.push(sample);

    int index = random(0, pid_training_buffer_input.size() - 1);

    xSemaphoreTake(mutex_pid_training_buffer, portMAX_DELAY);
    pid_training_buffer.push(pid_training_buffer_input[index]);
    xSemaphoreGive(mutex_pid_training_buffer);

}
void NeuralController::learning_step_pid_tuner() {

    if (pid_training_buffer.isEmpty()) {
        return;
    }


    //#define PID_LEARN_DEBUG

    xSemaphoreTake(mutex_pid_training_buffer, portMAX_DELAY);
    current_pid_sample = pid_training_buffer.pop();
    xSemaphoreGive(mutex_pid_training_buffer);




    static float gains_arr[5] = { 0 };

    static float inputs[10] = { 0 };
    inputs[0] = current_pid_sample.joint_pos * inv_max_angle;
    inputs[1] = current_pid_sample.joint_vel * inv_max_vel;
    inputs[2] = current_pid_sample.joint_acc * inv_max_acc;
    inputs[3] = current_pid_sample.motor_pos * inv_max_angle;
    inputs[4] = current_pid_sample.motor_vel * inv_max_vel;
    inputs[5] = current_pid_sample.motor_acc * inv_max_acc;
    inputs[6] = current_pid_sample.joint_torque * inv_max_joint_torque;
    inputs[7] = current_pid_sample.joint_pos_target * inv_max_angle;
    inputs[8] = current_pid_sample.joint_vel_target * inv_max_vel;
    inputs[9] = current_pid_sample.joint_acc_target * inv_max_acc;

    xSemaphoreTake(mutex_pid_net, portMAX_DELAY);
    adaptive_pid_nn->predict(inputs, gains_arr);
    xSemaphoreGive(mutex_pid_net);

#ifdef PID_LEARN_DEBUG
    Serial.println("Learning Steps");
    Serial.println("Sample Inputs");
    for (int i = 0; i < 10; i++) {
        Serial.println(inputs[i]);
    }
    Serial.println("Sample Outputs");
    for (int i = 0; i < 5; i++) {
        Serial.println(gains_arr[i]);
    }
#endif


    // simulate PID Control with these parameters



    float pos_Kp = gains_arr[0];
    float pos_Ki = gains_arr[1];
    float pos_Kd = gains_arr[2];

    float vel_Kp = gains_arr[3];
    float vel_Ki = gains_arr[4];


    float pos_kp = pos_Kp;
    float pos_ki = pos_Ki * pos_sample_time_s;
    float pos_kd = pos_Kd * (1.0 / pos_sample_time_s);

    float vel_kp = vel_Kp;
    float vel_ki = vel_Ki * vel_sample_time_s;


    float prev_error_pos = current_pid_sample.pos_prev_error;
    float iTerm_error_pos = current_pid_sample.pos_iTerm;

    float iTerm_error_vel = current_pid_sample.vel_iTerm;



#ifdef PID_LEARN_DEBUG
    Serial.println("load Controller errors");
    Serial.println(current_pid_sample.pos_prev_error, 4);
    Serial.println(current_pid_sample.pos_iTerm, 4);
    Serial.println(current_pid_sample.vel_iTerm, 4);
#endif

    float pos_target = current_pid_sample.joint_pos_target * max_angle;
    float vel_target = current_pid_sample.joint_vel_target * max_vel;

#ifdef PID_LEARN_DEBUG
    Serial.println("load target values");

    Serial.println(pos_target, 4);
    Serial.println(vel_target, 4);
#endif

    // Simulate Position-Controller
    float pos_error_in = pos_target - current_pid_sample.joint_pos * max_angle;
    float pos_dError = pos_error_in - prev_error_pos;
    float pos_controller_output = pos_error_in * pos_kp + (iTerm_error_pos + pos_error_in) * pos_ki + pos_dError * pos_kd;


    float target_vel_m = pos_controller_output + vel_target;

    // Simulate Velocity Controller

    float vel_error_in = target_vel_m - current_pid_sample.motor_vel * max_vel;

    float motor_torque = vel_error_in * vel_kp + (iTerm_error_vel + vel_error_in) * vel_ki;

    if (motor_torque > DRVSYS_TORQUE_LIMIT) {
        motor_torque = DRVSYS_TORQUE_LIMIT;
    }

    if (motor_torque < -DRVSYS_TORQUE_LIMIT) {
        motor_torque = -DRVSYS_TORQUE_LIMIT;
    }

#ifdef PID_LEARN_DEBUG
    Serial.println("Controller Inputs");
    Serial.println(pos_error_in, 4);
    Serial.println(vel_error_in, 4);

    Serial.println("Controller Setpoints");
    Serial.println(pos_target, 4);
    Serial.println(target_vel_m, 4);

    Serial.println("Controller Outputs");
    Serial.println(pos_controller_output, 4);
    Serial.println(motor_torque, 4);
#endif





    // simulate Plant

    drvSys_FullDriveState current_state;
    current_state.joint_pos = current_pid_sample.joint_pos * max_angle;
    current_state.joint_vel = current_pid_sample.joint_vel * max_vel;
    current_state.joint_acc = current_pid_sample.joint_acc * max_acc;
    current_state.motor_pos = current_pid_sample.motor_pos * max_angle;
    current_state.motor_vel = current_pid_sample.motor_vel * max_vel;
    current_state.motor_acc = current_pid_sample.motor_acc * max_acc;
    current_state.motor_torque = motor_torque;
    current_state.joint_torque = current_pid_sample.joint_torque * max_joint_torque;

    drvSys_driveState state_pred = emulator_predict_next_state(current_state);

    float eps = 1e-3;

    drvSys_FullDriveState current_state_plus_eps = current_state;
    drvSys_FullDriveState current_state_min_eps = current_state;

    current_state_plus_eps.motor_torque = motor_torque + eps;
    current_state_min_eps.motor_torque = motor_torque - eps;

    drvSys_driveState state_pred_plus_eps = emulator_predict_next_state(current_state_plus_eps);
    drvSys_driveState state_pred_min_eps = emulator_predict_next_state(current_state_min_eps);


    // obtain pid control error
    float pos_error = (state_pred.joint_pos - pos_target);
    float vel_error = (current_state.motor_vel - target_vel_m) + (state_pred.joint_vel - vel_target);
#ifdef PID_LEARN_DEBUG
    Serial.println("simulated error");
    Serial.println(pos_error, 4);
    Serial.println(vel_error, 4);
#endif

    pid_control_error = 0.5 * (pos_error * pos_error + vel_error * vel_error);

    // Backpropagate pid control error to Controller gains
    float delta_pos_du = (state_pred_plus_eps.joint_pos - state_pred_min_eps.joint_pos) / (2.0 * eps);
    float delta_vel_du = (state_pred_plus_eps.joint_vel - state_pred_min_eps.joint_vel) / (2.0 * eps);
#ifdef PID_LEARN_DEBUG
    Serial.println("Estimated output derivatives");
    Serial.println(delta_pos_du, 4);
    Serial.println(delta_vel_du, 4);
#endif


    float du_dKp_vel = vel_error_in;
    float du_dKi_vel = (vel_error_in + iTerm_error_vel) * vel_sample_time_s;
#ifdef PID_LEARN_DEBUG
    Serial.print("du_dKp_vel: ");
    Serial.println(du_dKp_vel, 4);
    Serial.print("du_dKi_vel: ");
    Serial.println(du_dKi_vel, 4);
#endif

#ifdef PID_LEARN_DEBUG
    Serial.println("Internal vel gains");
    Serial.println(vel_kp, 4);
    Serial.println(vel_ki, 4);
#endif


    float d_traj_error_du = (pos_error * delta_pos_du + vel_error * delta_vel_du);
#ifdef PID_LEARN_DEBUG
    Serial.print("d_traj_error_du: ");
    Serial.println(d_traj_error_du, 4);
#endif

    float delta_Kp_vel = d_traj_error_du * du_dKp_vel;
    float delta_Ki_vel = d_traj_error_du * du_dKi_vel;
#ifdef PID_LEARN_DEBUG
    Serial.print("delta_Kp_vel: ");
    Serial.println(delta_Kp_vel, 4);
    Serial.print("delta_Ki_vel: ");
    Serial.println(delta_Ki_vel, 4);
#endif

    float du_dvel_error = (vel_kp + vel_ki);
#ifdef PID_LEARN_DEBUG
    Serial.print("du_dvel_error: ");
    Serial.println(du_dvel_error, 4);
#endif

    float dvel_error_dKp_pos = pos_error_in;
    float dvel_error_dKi_pos = (iTerm_error_pos + pos_error_in) * pos_sample_time_s;
    float dvel_error_dKd_pos = pos_dError * (1.0 / pos_sample_time_s);


#ifdef PID_LEARN_DEBUG
    Serial.print("dvel_error_dKp_pos: ");
    Serial.println(dvel_error_dKp_pos, 4);
    Serial.print("dvel_error_dKi_pos: ");
    Serial.println(dvel_error_dKi_pos, 4);
    Serial.print("dvel_error_dKd_pos: ");
    Serial.println(dvel_error_dKd_pos, 4);
#endif


    float delta_Kp_pos = d_traj_error_du * du_dvel_error * dvel_error_dKp_pos;
    float delta_Ki_pos = d_traj_error_du * du_dvel_error * dvel_error_dKi_pos;
    float delta_Kd_pos = d_traj_error_du * du_dvel_error * dvel_error_dKd_pos;
#ifdef PID_LEARN_DEBUG
    Serial.print("delta_Kp_pos: ");
    Serial.println(delta_Kp_pos, 4);
    Serial.print("delta_Ki_pos: ");
    Serial.println(delta_Ki_pos, 4);
    Serial.print("delta_Kd_pos: ");
    Serial.println(delta_Kd_pos, 4);
#endif

    float reg_penalty = 1e-2;


    static float loss_gains[5] = { 0 };
    loss_gains[0] = delta_Kp_pos + reg_penalty * (pos_Kp * pos_Kp * pos_Kp);
    loss_gains[1] = delta_Ki_pos + reg_penalty * (pos_Ki * pos_Ki * pos_Ki);
    loss_gains[2] = delta_Kd_pos + reg_penalty * (pos_Kd * pos_Kd * pos_Kd);
    loss_gains[3] = delta_Kp_vel + reg_penalty * (vel_Kp * vel_Kp * vel_Kp);
    loss_gains[4] = delta_Ki_vel + reg_penalty * (vel_Ki * vel_Ki * vel_Ki);
#ifdef PID_LEARN_DEBUG
    Serial.println("losses");
#endif

    const float grad_limit = 1.0;
    for (int i = 0; i < 5; i++) {

        /*
        if (loss_gains[i] > grad_limit) {
            loss_gains[i] = grad_limit;
        }
        if (loss_gains[i] < -grad_limit) {
            loss_gains[i] = -grad_limit;
        }---
        */
#ifdef PID_LEARN_DEBUG
        Serial.println(loss_gains[i], 4);
#endif
    }
    xSemaphoreTake(mutex_pid_net, portMAX_DELAY);
    adaptive_pid_nn->backpropagation(inputs, pid_control_error, loss_gains);

    adaptive_pid_nn->apply_gradient_descent(adam);

    xSemaphoreGive(mutex_pid_net);


    average_pid_control_error = average_pid_control_error * (1 - 1e-2) + pid_control_error * 1e-2;


    adaptive_pid_nn->learning_rate = 5e-3 * average_pid_control_error;

    if (adaptive_pid_nn->learning_rate > 1e-4) {
        adaptive_pid_nn->learning_rate = 1e-4;
    }

    if (adaptive_pid_nn->learning_rate < 1e-7) {
        adaptive_pid_nn->learning_rate = 1e-7;
    }




}
cascade_gains NeuralController::predict_gains(drvSys_FullDriveState current_state, drvSys_driveTargets targets) {

    static float input[10];
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

    static float gains_pointer[5] = { 0 };


    if (xSemaphoreTake(mutex_pid_net, (TickType_t)0) == pdTRUE) {
        adaptive_pid_nn->predict(input, gains_pointer);

        xSemaphoreGive(mutex_pid_net);

    }
    xSemaphoreGive(mutex_pid_net);

    cascade_gains gains;
    gains.pos_Kp = gains_pointer[0];
    gains.pos_Ki = gains_pointer[1];
    gains.pos_Kd = gains_pointer[2];
    gains.vel_Kp = gains_pointer[3];
    gains.vel_Ki = gains_pointer[4];

    return gains;

}





