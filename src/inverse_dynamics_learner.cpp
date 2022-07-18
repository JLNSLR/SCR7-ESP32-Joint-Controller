#include <inverse_dynamics_learner.h>
#include <drive_system_types.h>


InverseDynamicsLearner::InverseDynamicsLearner() {

}

void InverseDynamicsLearner::init() {
    Serial.println("DRVSYS_INFO: Initializing NN for Inverse Dynamics.");
    const int depth = 4;
    int width[depth] = { 7,7,5,1 };
    nn_activation_f activation[depth - 1] = { leakyReLu, leakyReLu, Linear };

    nn = new NeuralNetwork(depth, width, activation);

    nn->init_weights_randomly(0.5, -0.5);

    // Setup hyperparameters
    nn->learning_rate = 1e-3;
    nn->loss_type = MSE;
    nn->update_rule = adam;

    nn->decay_factor = 1e6;
    nn->lr_schedule = error_adaptive;
    nn->minimal_learning_rate = 1e-3;
    nn->lr_error_factor = 1e-2;
    nn->maximal_learning_rate = 1e-1;

    // setup scalers
    inv_acc = 1.0 / max_acc;
    inv_max_motor_torque = 1.0 / max_motor_torque;
    inv_max_vel = 1.0 / max_vel;
    inv_max_joint_torque = 1.0 / max_joint_torque;

    // load weights from the flash if available
    nn_model_weights weights = nn_load_model_weights_from_flash("inv_dyn", nn->n_weights);

    if (weights.n_weights == nn->n_weights) {
        nn->load_model_weights(weights);
    }

    activate_automatic = DRVSYS_ACTIVATE_LEARNED_CONTROL_AUTOMATIC;
    prediction_error_threshold = DRVSYS_INV_DYN_ACTIVATION_ERROR_THRESHOLD;

    xSemaphoreGive(mutex_inverse_dyn_network);
}
float InverseDynamicsLearner::predict_torque(const drvSys_FullDriveState state_rad) {

    static float last_prediction = 0.0;
    float torque_pred = 0.0;

    Serial.println("Predicting with inv NN");
    if (control_active) {
        float input_vector[7];
        Serial.println("trying to predict");
        input_vector[0] = state_rad.joint_pos * inv_max_angle;
        input_vector[1] = state_rad.joint_vel * inv_max_vel;
        input_vector[2] = state_rad.joint_acc * inv_acc;
        input_vector[3] = state_rad.motor_pos * inv_max_angle;
        input_vector[4] = state_rad.motor_vel * inv_max_vel;
        input_vector[5] = state_rad.motor_acc * inv_acc;
        input_vector[6] = state_rad.joint_torque * inv_max_joint_torque;
        Serial.println("trying to predict");
        if (xSemaphoreTake(mutex_inverse_dyn_network, (TickType_t)1) == pdTRUE) {
            torque_pred = *nn->predict(input_vector) * max_motor_torque;
            xSemaphoreGive(mutex_inverse_dyn_network);
            last_prediction = torque_pred;
            Serial.println("predicte");
        }
        else {
            torque_pred = last_prediction;
        }

        return torque_pred;

    }
    return torque_pred;

}
void InverseDynamicsLearner::add_data(const drvSys_FullDriveState state_rad) {

    Serial.println("Adding Data to INV NN");
    inv_dynamics_sample sample;
    sample.input_vector[0] = state_rad.joint_pos * inv_max_angle;
    sample.input_vector[1] = state_rad.joint_vel * inv_max_vel;
    sample.input_vector[2] = state_rad.joint_acc * inv_acc;
    sample.input_vector[3] = state_rad.motor_pos * inv_max_angle;
    sample.input_vector[4] = state_rad.motor_vel * inv_max_vel;
    sample.input_vector[5] = state_rad.motor_acc * inv_acc;
    sample.input_vector[6] = state_rad.joint_torque * inv_max_joint_torque;

    sample.output = state_rad.motor_torque * inv_max_motor_torque;

    addSampleToBuffer(sample);
}
bool InverseDynamicsLearner::learning_step() {



    bool learned = false;
    if (samples_in_buffer > 0) {

        inv_dynamics_sample sample = takeSampleFromBuffer();
        Serial.println("Perform learning step");
        xSemaphoreTake(mutex_inverse_dyn_network, portMAX_DELAY);
        float error = nn->train_SGD(sample.input_vector, &sample.output);
        xSemaphoreGive(mutex_inverse_dyn_network);

        learned = true;

        delta_prediction_error = error - prediction_error;
        prediction_error = abs(error);

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
    inv_max_motor_torque = 1.0 / max_joint_torque;
}

void InverseDynamicsLearner::addSampleToBuffer(inv_dynamics_sample sample) {
    //xSemaphoreTake(mutex_inverse_dyn_network, portMAX_DELAY);
    if (samples_in_buffer < buffer_size) {
        samples_in_buffer++;
        sample_vector[samples_in_buffer] = sample;
    }
    else {
        sample_vector[0] = sample;
    }
    //xSemaphoreGive(mutex_inverse_dyn_network);
    Serial.println(samples_in_buffer);
}
inv_dynamics_sample InverseDynamicsLearner::takeSampleFromBuffer() {
    inv_dynamics_sample sample = sample_vector[samples_in_buffer];
    samples_in_buffer--;

    if (nn->n_iterations % 10 == 0) {
        this->randomize(sample_vector, samples_in_buffer);
    }

    return sample;

}

void InverseDynamicsLearner::swap(inv_dynamics_sample* a, inv_dynamics_sample* b) {
    inv_dynamics_sample temp = *a;
    *a = *b;
    *b = temp;
}

void InverseDynamicsLearner::randomize(inv_dynamics_sample arr[], int n) {
    if (n < 1) {
        return;
    }
    srand(time(NULL));
    int i;
    for (i = n - 1; i > 0; i--) {
        int j = rand() % (i + 1);
        swap(&arr[i], &arr[j]);
    }
}