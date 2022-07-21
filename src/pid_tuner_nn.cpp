#include <pid_tuner_nn.h>


PID_Tuner_NN::PID_Tuner_NN() {

};

void PID_Tuner_NN::init() {

    const int depth = PID_TUNER_NN_DEPTH;
    int width[depth] = { PID_TUNER_INPUTS, PID_TUNER_NN_HIDDEN_LAYER_1_SIZE, PID_TUNER_NN_HIDDEN_LAYER_2_SIZE, PID_TUNER_OUTPUTS };
    nn_activation_f activation[depth - 1] = { leakyReLu,leakyReLu,ReLu };


    nn = new NeuralNetwork(depth, width, activation);

    nn->init_weights_randomly(0.25, -0.1);

    // load weights from the flash if available
    nn_model_weights weights = nn_load_model_weights_from_flash("pid_nn", nn->n_weights);

    if (weights.n_weights == nn->n_weights) {
        Serial.println("DRVSYS_PID_TUNER: Loading model NN weights from flash");
        nn->load_model_weights(weights);
    }

    /* setup NN reference system */

    const int depth_ref = 4;
    int width_ref[depth_ref] = { 5,5,3,2 };
    nn_activation_f activation_ref[depth_ref] = { leakyReLu, leakyReLu, Linear };


    nn_reference = new NeuralNetwork(depth_ref, width_ref, activation_ref);
    nn->init_weights_randomly(0.5, 0.75);

    // load weights from the flash if available
    nn_model_weights weights_ref = nn_load_model_weights_from_flash("nn_ref", nn_reference->n_weights);

    if (weights.n_weights == nn->n_weights) {
        Serial.println("DRVSYS_PID_TUNER_REF: Loading model NN weights from flash");
        nn_reference->load_model_weights(weights_ref);
    }

    xSemaphoreGive(mutex_pid_tuner_net);



};

bool PID_Tuner_NN::learning_step() {

    bool learned = false;

    if (current_buffer_size > 0) {
        PID_Tuner_sample sample = sample_buffer[current_buffer_size - 1];
        current_buffer_size--;

        // train reference network
        float error_ref = train_reference_nn(sample);

        // turn sample into input_vector
        float input_vector[PID_TUNER_INPUTS];
        input_vector[0] = sample.joint_pos * inv_max_angle;
        input_vector[1] = sample.joint_vel * inv_max_vel;
        input_vector[2] = sample.motor_pos * inv_max_angle;
        input_vector[3] = sample.motor_vel * inv_max_vel;
        input_vector[4] = sample.pos_target * inv_max_angle;
        input_vector[5] = sample.vel_target;


        // Calculate Errors
        float pos_error = (sample.joint_pos_t_plus - sample.pos_target);
        float vel_error = (sample.joint_vel_t_plus - sample.vel_target);
        float error = 0.5 * pos_error * pos_error + 0.5 * vel_error * vel_error;

        // obtain error_derivative w.r.t to Controller output u

        float derror_dpos = (sample.joint_pos_t_plus - sample.pos_target);
        float derror_dvel = (sample.joint_vel_t_plus - sample.vel_target);

        nn_ref_output_derivative derivatives_y = estimateOutputDerivative(sample);

        float djoint_pos_du = derivatives_y.dpos_du * max_angle;
        float djoint_vel_du = derivatives_y.dvel_du * max_vel;

        float du_dKp = sample.error;
        float du_dKi = sample.ITermError;
        float du_dKd = sample.dError;

        float du_dKvel_ff = sample.vel_target;
        float du_dKjointP = pos_error;

        float output_torque_error = (djoint_pos_du * derror_dpos + djoint_vel_du * derror_dvel);

        float error_derivative_Kp = du_dKp * output_torque_error;
        float error_derivative_Ki = du_dKi * output_torque_error;
        float error_derivative_Kd = du_dKd * output_torque_error;
        float error_derivative_KjointP = du_dKjointP * output_torque_error;
        float error_derivative_Kvel_ff = du_dKvel_ff * output_torque_error;


        float error_derivatives[5];
        error_derivatives[0] = error_derivative_Kp;
        error_derivatives[1] = error_derivative_Ki;
        error_derivatives[2] = error_derivative_Kd;
        error_derivatives[3] = error_derivative_KjointP;
        error_derivatives[4] = error_derivative_Kvel_ff;



        xSemaphoreTake(mutex_pid_tuner_net, portMAX_DELAY);
        float nn_error = nn->train_SGD_ext_loss(input_vector, error, error_derivatives);
        xSemaphoreGive(mutex_pid_tuner_net);

        learned = true;

        if (nn->filtered_error < 0.1) {
            low_error_iterations++;
            if (low_error_iterations > 1e3) {
                xSemaphoreTake(mutex_pid_tuner_net, portMAX_DELAY);
                nn_save_model_weights_on_flash(nn->get_model_weights(), "pid_nn");
                nn_save_model_weights_on_flash(nn_reference->get_model_weights(), "nn_ref");
                xSemaphoreGive(mutex_pid_tuner_net);
                saved_weights = true;
                low_error_iterations = 0;
            }
        }
        else {
            low_error_iterations = 0;
        }

    }
    return learned;



};


void PID_Tuner_NN::set_scaling(float max_vel, float max_motor_torque) {
    max_vel = max_vel;
    max_motor_torque = max_motor_torque;

    inv_max_vel = 1.0 / max_vel;
    inv_max_motor_torque = 1.0 / max_motor_torque;
}

void PID_Tuner_NN::addSample(const PID_Tuner_sample sample) {

    if (current_buffer_size < max_buffer_size) {
        current_buffer_size++;
        sample_buffer[current_buffer_size - 1] = sample;
    }
    else if (current_buffer_size == max_buffer_size) {
        sample_buffer[max_buffer_size - 1] = sample;
    }

}

drvSys_PID_Gains PID_Tuner_NN::predictOptimalGains(const drvSys_FullDriveState state, const drvSys_driveTargets targets) {


    float input_vector[PID_TUNER_INPUTS];
    input_vector[0] = state.joint_pos * inv_max_angle;
    input_vector[1] = state.joint_vel * inv_max_vel;
    input_vector[2] = state.motor_pos * inv_max_angle;
    input_vector[3] = state.motor_vel * inv_max_vel;
    input_vector[4] = targets.pos_target * inv_max_angle;
    input_vector[5] = targets.vel_target * inv_max_vel;

    xSemaphoreTake(mutex_pid_tuner_net, portMAX_DELAY);
    float* gain_vector_pointer;
    gain_vector_pointer = nn->predict(input_vector);
    xSemaphoreGive(mutex_pid_tuner_net);

    drvSys_PID_Gains gains;
    gains.K_p = gain_vector_pointer[0];
    gains.K_i = gain_vector_pointer[1];
    gains.K_d = gain_vector_pointer[2];
    gains.K_vel_ff = gain_vector_pointer[3];

    return gains;

}

float PID_Tuner_NN::train_reference_nn(const PID_Tuner_sample sample) {

    // train reference network
    float input_vector_ref[5];
    input_vector_ref[0] = sample.motor_torque_u * inv_max_motor_torque;
    input_vector_ref[1] = sample.joint_pos * inv_max_angle;
    input_vector_ref[2] = sample.joint_vel * inv_max_vel;;
    input_vector_ref[3] = sample.motor_pos * inv_max_angle;
    input_vector_ref[4] = sample.motor_vel * inv_max_vel;


    float output_vector_ref[2];
    output_vector_ref[0] = sample.joint_pos_t_plus * inv_max_vel;
    output_vector_ref[1] = sample.joint_vel_t_plus * inv_max_vel;

    float nn_ref_error = nn_reference->train_SGD(input_vector_ref, output_vector_ref);

    return nn_ref_error;
}

const nn_ref_output_derivative PID_Tuner_NN::estimateOutputDerivative(const PID_Tuner_sample sample) {

    float epsilon = 1e-3;

    float input_vector_minus_eps[5];
    input_vector_minus_eps[0] = sample.motor_torque_u * inv_max_motor_torque;
    input_vector_minus_eps[1] = sample.joint_pos * inv_max_angle;
    input_vector_minus_eps[2] = sample.joint_vel * inv_max_vel;
    input_vector_minus_eps[3] = sample.motor_pos * inv_max_angle;
    input_vector_minus_eps[4] = sample.motor_vel * inv_max_vel;


    float input_vector_plus_eps[5];
    input_vector_plus_eps[0] = sample.motor_torque_u * inv_max_motor_torque + epsilon;
    input_vector_plus_eps[1] = sample.joint_pos * inv_max_angle;
    input_vector_plus_eps[2] = sample.joint_vel * inv_max_vel;
    input_vector_plus_eps[3] = sample.motor_pos * inv_max_angle;
    input_vector_plus_eps[4] = sample.motor_vel * inv_max_vel;

    float output_min_eps[2] = { 0,0 };
    float output_plus_eps[2] = { 0,0 };

    output_min_eps[0] = nn_reference->predict(input_vector_minus_eps)[0];
    output_min_eps[1] = nn_reference->predict(input_vector_minus_eps)[1];
    output_plus_eps[0] = nn_reference->predict(input_vector_plus_eps)[0];
    output_plus_eps[1] = nn_reference->predict(input_vector_plus_eps)[1];

    nn_ref_output_derivative out;
    out.dpos_du = (output_plus_eps[0] - output_min_eps[0]) / 2 * epsilon;
    out.dpos_du = (output_plus_eps[1] - output_min_eps[1]) / 2 * epsilon;

    return out;

}


void PID_Tuner_NN::collectSampleTimeStep(drvSys_FullDriveState state) {
    if (sampling_part == 1) {
        input_sample.joint_pos_t_plus = state.joint_pos;
        input_sample.joint_vel_t_plus = state.joint_vel;


        addSample(input_sample);

        sampling_part = 0;
    }
}
void PID_Tuner_NN::collectSamplePIDState(drvSys_driveTargets target, drvSys_FullDriveState state, float error, float dError, float ITermError) {

    if (sampling_part == 0) {
        input_sample.joint_pos = state.joint_pos;
        input_sample.joint_vel = state.joint_vel;
        input_sample.motor_pos = state.motor_pos;
        input_sample.motor_vel = state.motor_vel;
        input_sample.pos_target = target.pos_target;
        input_sample.vel_target = target.vel_target;
        input_sample.dError = dError;
        input_sample.error = error;
        input_sample.ITermError = ITermError;

        sampling_part = 1;
    }
}







