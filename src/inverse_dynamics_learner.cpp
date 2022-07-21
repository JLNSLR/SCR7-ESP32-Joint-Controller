#include <inverse_dynamics_learner.h>
#include <drive_system_types.h>


InverseDynamicsLearner::InverseDynamicsLearner() {

}

void InverseDynamicsLearner::init() {
    Serial.println("DRVSYS_INFO: Initializing NN for Inverse Dynamics.");

    nn = new NeuralNetwork(depth, width, activation);

    nn->init_weights_randomly(0.2, -0.1);

    // Setup hyperparameters
    nn->learning_rate = 1e-3;
    nn->loss_type = MSE;
    nn->update_rule = adam;

    nn->regularization = lasso;
    nn->reg_penalty_factor = 1e-6;

    nn->decay_factor = 1e6;
    nn->lr_schedule = error_adaptive;
    nn->minimal_learning_rate = 1e-3;
    nn->lr_error_factor = 1e-4;
    nn->maximal_learning_rate = 7e-3;

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

    activate_automatic = DRVSYS_ACTIVATE_LEARNED_CONTROL_AUTOMATIC;
    prediction_error_threshold = DRVSYS_INV_DYN_ACTIVATION_ERROR_THRESHOLD;

    mutex_inverse_dyn_network = xSemaphoreCreateBinary();
    mutex_training_buffer = xSemaphoreCreateBinary();


    xSemaphoreGive(mutex_inverse_dyn_network);
    xSemaphoreGive(mutex_training_buffer);
}
float InverseDynamicsLearner::predict_torque(const drvSys_FullDriveState state_rad) {

    static float last_prediction = 0.0;
    float torque_pred = 0.0;
    float input_vector[7];
    input_vector[0] = state_rad.joint_pos * inv_max_angle;
    input_vector[1] = state_rad.joint_vel * inv_max_vel;
    input_vector[2] = state_rad.joint_acc * inv_acc;
    input_vector[3] = state_rad.motor_pos * inv_max_angle;
    input_vector[4] = state_rad.motor_vel * inv_max_vel;
    input_vector[5] = state_rad.motor_acc * inv_acc;
    input_vector[6] = state_rad.joint_torque * inv_max_joint_torque;
    if (xSemaphoreTake(mutex_inverse_dyn_network, (TickType_t)3) == pdTRUE) {
        torque_pred = *nn->predict(input_vector) * max_motor_torque;
        xSemaphoreGive(mutex_inverse_dyn_network);
    }
    else {
        torque_pred = last_prediction;
    }


    return torque_pred;

}
void InverseDynamicsLearner::add_data(const drvSys_FullDriveState state_rad) {

    inv_dynamics_sample sample;
    sample.joint_pos_norm = state_rad.joint_pos * inv_max_angle;
    sample.joint_vel_norm = state_rad.joint_vel * inv_max_vel;
    sample.joint_acc_norm = state_rad.joint_acc * inv_acc;
    sample.motor_pos_norm = state_rad.motor_pos * inv_max_angle;
    sample.motor_vel_norm = state_rad.motor_vel * inv_max_vel;
    sample.motor_acc_norm = state_rad.motor_acc * inv_acc;
    sample.joint_torque = state_rad.joint_torque * inv_max_joint_torque;

    sample.output_motor_torque_norm = state_rad.motor_torque * inv_max_motor_torque;
    if (xSemaphoreTake(mutex_training_buffer, (TickType_t)1) == pdTRUE) {

        buffer.push(sample);
        xSemaphoreGive(mutex_training_buffer);
    }



}
bool InverseDynamicsLearner::learning_step() {



    bool learned = false;
    if (buffer.isEmpty() == false) {
        xSemaphoreTake(mutex_training_buffer, portMAX_DELAY);
        inv_dynamics_sample sample = buffer.pop();
        xSemaphoreGive(mutex_training_buffer);

        float input_vector[7] = { 0 };
        input_vector[0] = sample.joint_pos_norm;
        input_vector[1] = sample.joint_vel_norm;
        input_vector[2] = sample.joint_acc_norm;
        input_vector[3] = sample.motor_pos_norm;
        input_vector[4] = sample.motor_vel_norm;
        input_vector[5] = sample.motor_acc_norm;
        input_vector[6] = sample.joint_torque;

        float motor_torque = sample.output_motor_torque_norm;

        xSemaphoreTake(mutex_inverse_dyn_network, portMAX_DELAY);
        float error = nn->train_SGD(input_vector, &motor_torque);
        xSemaphoreGive(mutex_inverse_dyn_network);



        learned = true;

        delta_prediction_error = error - prediction_error;
        prediction_error = abs(error);
        /*
            if (nn->filtered_error <= prediction_error_threshold) {
                if (activate_automatic) {
                    control_active = true;
                }
            }
            else {
                if (activate_automatic) {
                    control_active = false;;
                }
            }
        }

        if (delta_prediction_error < abs(prediction_error_threshold)) {
            low_change_iterations++;

            if (low_change_iterations > 1e4) {
                nn->learning_rate = 1e-6;
                nn->update_rule = sgd;
                low_learning_mode = true;
            }
        }
        else {
            low_change_iterations = 0;

            if (low_learning_mode) {
                nn->learning_rate = 1e-3;
                nn->update_rule = adam;
                low_learning_mode = false;;
            }
        }

        if (prediction_error < 0.1) {
            low_error_iterations++;
            if (low_error_iterations > 1e3) {
                xSemaphoreTake(mutex_inverse_dyn_network, portMAX_DELAY);
                nn_save_model_weights_on_flash(nn->get_model_weights(), "inv_dyn");
                xSemaphoreGive(mutex_inverse_dyn_network);
                saved_weights = true;
                low_error_iterations = 0;
            }
        }
        else {
            low_error_iterations = 0;
        }
        */

        return learned;

    }
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



