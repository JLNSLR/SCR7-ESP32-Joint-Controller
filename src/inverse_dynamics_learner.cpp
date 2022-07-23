#include <inverse_dynamics_learner.h>
#include <drive_system_types.h>


InverseDynamicsLearner::InverseDynamicsLearner() {

}

void InverseDynamicsLearner::init() {
    Serial.println("DRVSYS_INFO: Initializing NN for Inverse Dynamics.");

    nn = new NeuralNetwork(depth, width, activation);

    nn->init_weights_randomly(0.02, -0.01);

    // Setup hyperparameters
    nn->learning_rate = 5e-3;
    nn->loss_type = MSE;
    nn->update_rule = adam;

    nn->regularization = none;
    nn->reg_penalty_factor = 1e-4;

    nn->decay_factor = 1e6;
    nn->lr_schedule = error_adaptive;
    nn->minimal_learning_rate = 1e-3;
    nn->lr_error_factor = 5e-4;
    nn->maximal_learning_rate = 1e-2;

    // setup scalers
    inv_acc = 1.0 / max_acc;
    inv_max_motor_torque = 1.0 / max_motor_torque;
    inv_max_vel = 1.0 / max_vel;
    inv_max_joint_torque = 1.0 / max_joint_torque;

    nn->max_gradient_abs = 1.0;


    // load weights from the flash if available
    nn_model_weights weights = nn_load_model_weights_from_flash("inv_dyn", nn->n_weights);

    if (weights.n_weights == nn->n_weights) {
        nn->load_model_weights(weights);
    }


    mutex_inverse_dyn_network = xSemaphoreCreateBinary();
    mutex_training_buffer = xSemaphoreCreateBinary();


    xSemaphoreGive(mutex_inverse_dyn_network);
    xSemaphoreGive(mutex_training_buffer);


    // setup forward dynamics learner

    forward_model_nn = new NeuralNetwork(depth_forward, width_forward, activation_forward);

    forward_model_nn->init_weights_randomly(0.001, -0.01);

    forward_model_nn->learning_rate = 1e-3;
    forward_model_nn->update_rule = adam;
    forward_model_nn->loss_type = MSE;
    forward_model_nn->regularization = none;
    forward_model_nn->lr_schedule = no_schedule;
    forward_model_nn->minimal_learning_rate = 1e-3;
    forward_model_nn->lr_error_factor = 1e-4;
    forward_model_nn->maximal_learning_rate = 5e-3;
    forward_model_nn->max_gradient_abs = 0.1;

    // load weights from the flash if available
    nn_model_weights weights_forward = nn_load_model_weights_from_flash("forw_nn", forward_model_nn->n_weights);

    if (weights_forward.n_weights == forward_model_nn->n_weights) {
        forward_model_nn->load_model_weights(weights_forward);
    }


    mutex_forward_dyn_network = xSemaphoreCreateBinary();

    xSemaphoreGive(mutex_forward_dyn_network);




}
float InverseDynamicsLearner::predict_torque(const drvSys_FullDriveState state_rad, const drvSys_driveTargets targets) {

    static float last_prediction = 0.0;
    float torque_pred = 0.0;
    float input_vector[10];
    input_vector[0] = state_rad.joint_pos * inv_max_angle;
    input_vector[1] = state_rad.joint_vel * inv_max_vel;
    input_vector[2] = state_rad.joint_acc * inv_acc;
    input_vector[3] = state_rad.motor_pos * inv_max_angle;
    input_vector[4] = state_rad.motor_vel * inv_max_vel;
    input_vector[5] = state_rad.motor_acc * inv_acc;
    input_vector[6] = state_rad.joint_torque * inv_max_joint_torque;
    input_vector[7] = targets.pos_target * inv_max_angle;
    input_vector[8] = targets.vel_target * inv_max_vel;
    input_vector[9] = targets.acc_target * inv_acc;
    if (xSemaphoreTake(mutex_inverse_dyn_network, (TickType_t)1) == pdTRUE) {
        torque_pred = *nn->predict(input_vector) * max_motor_torque;
        xSemaphoreGive(mutex_inverse_dyn_network);
    }
    else {
        torque_pred = last_prediction;
    }


    return torque_pred;

}

drvSys_driveState InverseDynamicsLearner::predict_joint_kinematic(const drvSys_FullDriveStateExt states) {


    float input_vector[8] = { 0 };
    input_vector[0] = states.state_prev.joint_pos * inv_max_angle;
    input_vector[1] = states.state_prev.joint_vel * inv_max_vel;
    input_vector[2] = states.state_prev.joint_acc * inv_acc;
    input_vector[3] = states.state_prev.motor_pos * inv_max_angle;
    input_vector[4] = states.state_prev.motor_vel * inv_max_vel;
    input_vector[5] = states.state_prev.motor_acc * inv_acc;
    input_vector[6] = states.state_prev.motor_torque * inv_max_motor_torque;
    input_vector[7] = states.state_prev.joint_torque * inv_max_joint_torque;

    /*
    Serial.println(input_vector[0]);
    Serial.println(input_vector[1]);
    Serial.println(input_vector[2]);
    Serial.println(input_vector[3]);
    Serial.println(input_vector[4]);
    Serial.println(input_vector[5]);
    Serial.println(input_vector[6]);
    Serial.println(input_vector[7]);
    */




    float* output_vec = new float[3];

    output_vec = forward_model_nn->predict(input_vector);

    drvSys_driveState state_output;
    state_output.joint_pos = output_vec[0] * max_angle;
    state_output.joint_vel = output_vec[1] * max_vel;
    state_output.joint_acc = output_vec[2] * max_acc;

    /*
    Serial.println(output_vec[0]);
    Serial.println(output_vec[1]);
    Serial.println(output_vec[2]);
    */


    delete output_vec;


    return state_output;

}

void InverseDynamicsLearner::add_data(const drvSys_FullDriveStateExt states, const drvSys_driveTargets targets) {


    dynamics_sample sample;
    sample.joint_acc_norm = states.state.joint_acc * inv_acc;
    sample.joint_vel_norm = states.state.joint_vel * inv_max_vel;
    sample.joint_pos_norm = states.state.joint_pos * inv_max_angle;
    sample.motor_acc_norm = states.state.motor_acc * inv_acc;
    sample.motor_vel_norm = states.state.motor_vel * inv_max_vel;
    sample.motor_pos_norm = states.state.motor_pos * inv_max_angle;

    sample.joint_torque_norm = states.state.joint_torque * inv_max_joint_torque;
    sample.motor_torque_norm = states.state.motor_torque * inv_max_motor_torque;

    sample.joint_acc_prev_norm = states.state_prev.joint_acc * inv_acc;
    sample.joint_vel_prev_norm = states.state_prev.joint_vel * inv_max_vel;
    sample.joint_pos_prev_norm = states.state_prev.joint_pos * inv_max_angle;
    sample.motor_acc_prev_norm = states.state_prev.motor_acc * inv_acc;
    sample.motor_vel_prev_norm = states.state_prev.motor_vel * inv_max_vel;
    sample.motor_pos_prev_norm = states.state_prev.motor_pos * inv_max_angle;

    sample.joint_torque_prev_norm = states.state_prev.joint_torque * inv_max_joint_torque;
    sample.motor_torque_prev_norm = states.state_prev.motor_torque * inv_max_motor_torque;

    sample.joint_target_acc_norm = targets.acc_target * inv_acc;
    sample.joint_target_pos_norm = targets.pos_target * inv_max_angle;
    sample.joint_target_vel_norm = targets.vel_target * inv_max_vel;

    xSemaphoreTake(mutex_training_buffer, portMAX_DELAY);
    buffer.push(sample);
    xSemaphoreGive(mutex_training_buffer);


}

bool InverseDynamicsLearner::forward_dyn_learning_step() {
    bool learned = false;

    if (buffer.isEmpty() == false) {
        xSemaphoreTake(mutex_training_buffer, portMAX_DELAY);
        dynamics_sample sample = buffer.pop();
        xSemaphoreGive(mutex_training_buffer);

        float input_vector[8] = { 0 };
        input_vector[0] = sample.joint_pos_prev_norm;
        input_vector[1] = sample.joint_vel_prev_norm;
        input_vector[2] = sample.joint_acc_prev_norm;
        input_vector[3] = sample.motor_pos_prev_norm;
        input_vector[4] = sample.motor_vel_prev_norm;
        input_vector[5] = sample.motor_acc_prev_norm;
        input_vector[6] = sample.motor_torque_prev_norm;
        input_vector[7] = sample.joint_torque_prev_norm;

        /*
        Serial.println("---");

        Serial.println(input_vector[0] * max_angle * RAD2DEG);
        Serial.println(input_vector[1] * max_vel * RAD2DEG);
        Serial.println(input_vector[2] * max_acc * RAD2DEG);
        Serial.println(input_vector[3] * max_angle * RAD2DEG);
        Serial.println(input_vector[4] * max_vel * RAD2DEG);
        Serial.println(input_vector[5] * max_acc * RAD2DEG);
        Serial.println(input_vector[6] * max_motor_torque);
        Serial.println(input_vector[7] * max_joint_torque);
        */

        float output_vector[3] = { 0 };
        output_vector[0] = sample.joint_pos_norm;
        output_vector[1] = sample.joint_vel_norm;
        output_vector[2] = sample.joint_acc_norm;
        /*
        Serial.println("...");

        Serial.println(output_vector[0] * max_angle * RAD2DEG);
        Serial.println(output_vector[1] * max_vel * RAD2DEG);
        Serial.println(output_vector[2] * max_acc * RAD2DEG);
        */



        xSemaphoreTake(mutex_forward_dyn_network, portMAX_DELAY);
        float error = forward_model_nn->train_SGD(input_vector, output_vector);
        xSemaphoreGive(mutex_forward_dyn_network);

        learned = true;

    }
    return learned;
}

bool InverseDynamicsLearner::inverse_dyn_learning_step() {
    bool learned = false;

    if (buffer.isEmpty() == false) {
        xSemaphoreTake(mutex_training_buffer, portMAX_DELAY);
        dynamics_sample sample = buffer.pop();
        xSemaphoreGive(mutex_training_buffer);

        float input_vector[10];
        input_vector[0] = sample.joint_pos_prev_norm;
        input_vector[1] = sample.joint_vel_prev_norm;
        input_vector[2] = sample.joint_acc_prev_norm;
        input_vector[3] = sample.motor_pos_prev_norm;
        input_vector[4] = sample.motor_vel_prev_norm;
        input_vector[5] = sample.motor_acc_prev_norm;
        input_vector[6] = sample.joint_torque_prev_norm;
        input_vector[7] = sample.joint_target_pos_norm;
        input_vector[8] = sample.joint_target_vel_norm;
        input_vector[9] = sample.joint_target_acc_norm;


        float output_motor_torque = sample.motor_torque_prev_norm;


        //calculate output error 
        float pos_error = sample.joint_target_pos_norm - sample.joint_pos_prev_norm;
        float vel_error = sample.joint_target_vel_norm - sample.joint_vel_prev_norm;
        control_error = vel_error * vel_error + pos_error * pos_error;

        // calculate error derivative with respect to motor torque output u

        //using finite differences method
        float forward_dyn_input_vector[8];
        forward_dyn_input_vector[0] = sample.joint_pos_prev_norm;
        forward_dyn_input_vector[1] = sample.joint_vel_prev_norm;
        forward_dyn_input_vector[2] = sample.joint_acc_prev_norm;
        forward_dyn_input_vector[3] = sample.motor_pos_prev_norm;
        forward_dyn_input_vector[4] = sample.motor_vel_prev_norm;
        forward_dyn_input_vector[5] = sample.motor_acc_prev_norm;
        forward_dyn_input_vector[6] = sample.motor_torque_prev_norm;
        forward_dyn_input_vector[7] = sample.joint_torque_prev_norm;

        // plus epsilon
        float epsilon = 1e-3;
        float pos_output_plus_eps, vel_output_plus_eps;
        float pos_output_min_eps, vel_output_min_eps;

        forward_dyn_input_vector[6] = sample.motor_torque_prev_norm + epsilon;
        float* output_plus_eps = forward_model_nn->predict(forward_dyn_input_vector);

        forward_dyn_input_vector[6] = sample.motor_torque_prev_norm - epsilon;
        float* output_min_eps = forward_model_nn->predict(forward_dyn_input_vector);

        pos_output_plus_eps = output_plus_eps[0];
        vel_output_plus_eps = output_plus_eps[1];

        pos_output_min_eps = output_min_eps[0];
        vel_output_min_eps = output_min_eps[1];

        float dpos_du = (pos_output_plus_eps - pos_output_min_eps) / (2 * epsilon);
        float dvel_du = (vel_output_plus_eps - vel_output_min_eps) / (2 * epsilon);

        float error_derivative = control_error * ((-sample.joint_pos_prev_norm) * dpos_du + (-sample.joint_vel_prev_norm) * dvel_du);

        xSemaphoreTake(mutex_inverse_dyn_network, portMAX_DELAY);
        float error = nn->train_SGD_ext_loss(input_vector, control_error, &error_derivative);
        xSemaphoreGive(mutex_inverse_dyn_network);

        control_error = error;

        learned = true;
    }
    return learned;

}

void InverseDynamicsLearner::set_scale(float max_motor_torque, float max_vel, float max_acc, float max_joint_torque) {
    this->max_motor_torque = max_motor_torque;
    this->max_vel = max_vel;
    this->max_acc = max_acc;
    this->max_joint_torque = max_joint_torque;

    inv_acc = 1.0 / max_acc;
    inv_max_motor_torque = 1.0 / max_motor_torque;
    inv_max_vel = 1.0 / max_vel;
    inv_max_joint_torque = 1.0 / max_joint_torque;
}

void InverseDynamicsLearner::save_parameters_on_flash() {

    // inverse dynamic controller net

    nn_model_weights inv_nn_weights = nn->get_model_weights();
    nn_save_model_weights_on_flash(inv_nn_weights, "inv_dyn");


    nn_model_weights forward_dyn_weigths = forward_model_nn->get_model_weights();
    nn_save_model_weights_on_flash(forward_dyn_weigths, "forw_nn");


    Serial.println("DRVSYS_INFO: NN Saved weights on Flash");

}



