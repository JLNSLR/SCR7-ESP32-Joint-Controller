#include <NN/NeuralNetwork.h>
#include <cmath>
#include <cstdlib>
#include <stdint.h>
#include <Arduino.h>

//#define NN_DEBUG

NeuralNetwork::NeuralNetwork(const int depth, int* width, nn_activation_f* f_act_per_layer) :
    depth(depth), width(width), activation_function_per_layer(f_act_per_layer) {

    // add bias neurons to all except the output layer
    for (int layer = 0; layer < depth - 2; layer++) {
        width[layer]++;
    }

    n_iterations = 0;
    // Allocate Memory for all weights
    weights = new float** [depth];
    weights_grad = new float** [depth];
    adam_m = new float** [depth];
    adam_v = new float** [depth];

    for (int layer = 1; layer < depth; layer++) {
        weights[layer] = new float* [width[layer]];
        weights_grad[layer] = new float* [width[layer]];

        adam_m[layer] = new float* [width[layer]];
        adam_v[layer] = new float* [width[layer]];


        for (int neuron = 0; neuron < width[layer]; neuron++) {
            // create no inputs to bias neurons
            if ((layer != (depth - 1)) && (neuron == (width[layer] - 1))) {
                continue;
            }
            else {

                weights[layer][neuron] = new float[width[layer - 1]];
                weights_grad[layer][neuron] = new float[width[layer - 1]];

                adam_m[layer][neuron] = new float[width[layer - 1]];
                adam_v[layer][neuron] = new float[width[layer - 1]];
            }

        }
    }

    // Initialize weights
    for (int layer = 1; layer < depth; layer++) {
        for (int neuron = 0; neuron < width[layer]; neuron++) {

            if ((layer != depth - 1) && (neuron == width[layer] - 1)) {
                continue;
            }
            for (int input = 0; input < width[layer - 1]; input++) {
                if (activation_function_per_layer[layer] == Linear) {

                    weights[layer][neuron][input] = random_float(-0.5, 0.5);
                }
                else {
                    weights[layer][neuron][input] = random_float(0.4, -0.2);

                }
                adam_m[layer][neuron][input] = 0.0;
                adam_v[layer][neuron][input] = 0.0;
                n_weights++;
            }
        }
    }

    // Allocate memory for neuron outputs

    neuron_outputs = new float* [depth];
    preact = new float* [depth];
    neuron_loss = new float* [depth];
    for (int layer = 0; layer < depth; layer++) {
        neuron_outputs[layer] = new float[width[layer]];
        preact[layer] = new float[width[layer]];
        neuron_loss[layer] = new float[width[layer]];
    }

    // init weight vector;
    weight_vector = new float[n_weights];

    // Initialize neuron_outputs
    for (int layer = 0; layer < depth; layer++) {
        for (int neuron = 0; neuron < width[layer]; neuron++) {
            neuron_outputs[layer][neuron] = 0.0;
            preact[layer][neuron] = 0.0;
            neuron_loss[layer][neuron] = 0.0;


            // set bias neurons to 1.0
            if ((neuron == (width[layer] - 1)) && (layer != (depth - 1))) {
                neuron_outputs[layer][neuron] = 1.0;
            }
        }
    }

    // Allocate memory and initialize network jacobian
    int n_network_outputs = width[depth - 1];
    int n_network_inputs = width[0];

    for (int n = 0; n < n_network_outputs; n++) {
        network_jacobian = new float* [n_network_inputs];
        network_jacobian[n] = { 0 };
    }

    // Initialize Barrier Loss
    for (int output_neuron_n = 0; output_neuron_n < width[depth - 1]; output_neuron_n++) {
        barrier_loss = new diff_fct_out[width[depth - 1]];
        barrier_loss[output_neuron_n].x = 0;
        barrier_loss[output_neuron_n].derivative = 0;
    }


    targets = new float[width[depth - 1]];

    // Set standard values
    learning_rate = 0.001;
    regularization = none;
    reg_penalty_factor = 0.001;
    barrier_softness = 1e-3;
    loss_type = MSE;
    update_rule = sgd;
    output_barriers = false;



};

NeuralNetwork::~NeuralNetwork() {

    for (int layer = 0; layer < depth; layer++) {
        delete[] neuron_outputs[layer];
        delete[] neuron_loss[layer];
        delete[] preact[layer];
        for (int neuron = 0; neuron < width[depth]; neuron++) {


            delete[] weights[layer][neuron];
            delete[] weights_grad[layer][neuron];
            delete[] adam_m[layer][neuron];
            delete[] adam_v[layer][neuron];

        }
        delete[] weights[layer];
        delete[] weights_grad[layer];
        delete[] adam_m[layer];
        delete[] adam_v[layer];
    }
    delete[] neuron_outputs;
    delete[] neuron_loss;
    delete[] preact;

    delete[] weights;
    delete[] weights_grad;
    delete[] adam_m;
    delete[] adam_v;

    for (int i = 0; i < width[depth - 1]; i++) {
        delete[] network_jacobian[i];
    }
    delete[] network_jacobian;


    delete[] barrier_loss;
}

void NeuralNetwork::set_input(float input_vector[]) {

    neuron_outputs[0] = input_vector;
    neuron_outputs[0][width[0] - 1] = 1.0;
}

float* NeuralNetwork::predict(float input_vector[]) {
    set_input(input_vector);
    propagate_forward();
    return neuron_outputs[depth - 1];
}

void NeuralNetwork::predict(float input_vector[], float output_vector[]) {
    set_input(input_vector);
    propagate_forward();

    for (int i = 0; i < width[depth - 1]; i++) {
        output_vector[i] = neuron_outputs[depth - 1][i];
        //Serial.println(output_vector[0]);
    }
}

float* NeuralNetwork::get_output() {
    return neuron_outputs[depth - 1];
}

float NeuralNetwork::train_SGD(float* inputs, float* targets) {


    set_input(inputs);
    propagate_forward();
    this->targets = targets;
    float total_error = backpropagation(inputs, targets);

    apply_gradient_descent(update_rule);

    return total_error;

}

float NeuralNetwork::train_SGD_ext_loss(float* inputs, float ext_loss, float* ext_loss_derivative) {

    set_input(inputs);
    propagate_forward();

    float total_error = backpropagation(inputs, ext_loss, ext_loss_derivative);

    apply_gradient_descent(update_rule);

    return total_error;
}

float NeuralNetwork::train_SGD_S(sample_t samples[], int n_samples) {

    float last_error = 0;
    for (int i = 0; i < n_samples; i++) {

        last_error = train_SGD(samples[i].inputs, samples[i].outputs);

    }
    return last_error;

}

float NeuralNetwork::train_batch(batch_loss_t batch) {

    backprop_batch_mode = true;

    // set weights to zero
    int layer, neuron, input;

    for (layer = 1; layer < depth; layer++) {
        for (neuron = 0; neuron < width[layer]; neuron++) {

            // Handle Bias Neurons in all layers except the output layer
            if (neuron == (width[layer] - 1) && layer != (depth - 1)) {

            }
            else {
                for (input = 0; input < width[layer - 1]; input++) {
                    //calculate average weight
                    weights_grad[layer][neuron][input] = 0;

                }
            }
        }
    }

    float total_error = 0;
    //iterate over samples in batch
    backprop_batch_mode = true;
    for (int s = 0; s < batch.n_samples; s++) {

        //backpropagate for each sample
        float* inputs = batch.samples[s].input;
        float loss = 0;
        float* loss_derivatives = new float(width[depth - 1]);
        for (int i = 0; i < width[depth - 1]; i++) {
            loss += batch.samples[s].loss[i].x;
            loss_derivatives[i] = batch.samples[s].loss[i].derivative;
        }

        total_error += backpropagation(inputs, loss, loss_derivatives);

    }
    for (layer = 1; layer < depth; layer++) {
        for (neuron = 0; neuron < width[layer]; neuron++) {

            // Handle Bias Neurons in all layers except the output layer
            if (neuron == (width[layer] - 1) && layer != (depth - 1)) {

            }
            else {
                for (input = 0; input < width[layer - 1]; input++) {
                    //calculate average weight
                    weights_grad[layer][neuron][input] = weights_grad[layer][neuron][input] / float(batch.n_samples);

                }
            }
        }
    }
    //update with average weight gradient
    apply_gradient_descent(update_rule);

    backprop_batch_mode = false;

    return total_error;
}

float NeuralNetwork::train_batch(batch_t batch) {

    backprop_batch_mode = true;

    // set weights to zero
    int layer, neuron, input;

    for (layer = 1; layer < depth; layer++) {
        for (neuron = 0; neuron < width[layer]; neuron++) {

            // Handle Bias Neurons in all layers except the output layer
            if (neuron == width[layer] - 1 && layer != depth - 1) {

            }
            else {
                for (input = 0; input < width[layer - 1]; input++) {
                    //calculate average weight
                    weights_grad[layer][neuron][input] = 0;

                }
            }
        }
    }

    float total_error = 0;
    //iterate over samples in batch
    backprop_batch_mode = true;
    for (int s = 0; s < batch.n_samples; s++) {

        //backpropagate for each sample
        total_error += backpropagation(batch.samples[s].inputs, batch.samples[s].outputs);

    }
    for (layer = 1; layer < depth; layer++) {
        for (neuron = 0; neuron < width[layer]; neuron++) {

            // Handle Bias Neurons in all layers except the output layer
            if ((neuron == (width[layer] - 1)) && (layer != (depth - 1))) {

            }
            else {
                for (input = 0; input < width[layer - 1]; input++) {
                    //calculate average weight
                    weights_grad[layer][neuron][input] = weights_grad[layer][neuron][input] / float(batch.n_samples);

                }
            }
        }
    }
    //update with average weight gradient
    apply_gradient_descent(update_rule);

    backprop_batch_mode = false;

    return total_error;

}

void NeuralNetwork::propagate_forward() {

    float neuron_sum;

    int layer, neuron, input;

    // Calculate neuron outputs

#ifdef NN_DEBUG
    Serial.println("Start forward propagation");
#endif // NN_DEBUG

    for (layer = 1; layer < depth; layer++) {
#ifdef NN_DEBUG
        Serial.print("NN: Layer: ");
        Serial.println(layer);
#endif // NN_DEBUG
        for (neuron = 0; neuron < width[layer]; neuron++) {

            neuron_sum = 0;

#ifdef NN_DEBUG
            Serial.print("NN: Neuron: ");
            Serial.println(neuron);
#endif // NN_DEBUG

            // Handle Bias Neurons in all layers except the output layer
            if ((neuron == (width[layer] - 1)) && (layer != (depth - 1))) {
                neuron_outputs[layer][neuron] = 1.0;
                preact[layer][neuron] = 1.0;
            }
            else {
                for (input = 0; input < width[layer - 1]; input++) {

#ifdef NN_DEBUG
                    Serial.print("NN: Input: ");
                    Serial.println(input);
                    Serial.println(neuron_outputs[layer - 1][input]);
#endif // NN_DEBUG


                    neuron_sum += neuron_outputs[layer - 1][input] * weights[layer][neuron][input];
                }

                neuron_outputs[layer][neuron] = apply_activation_function(neuron_sum, layer);

                preact[layer][neuron] = neuron_sum;
            }

        }
    }

}

float NeuralNetwork::backpropagation(float* inputs, float* targets) {

    float loss_sum = 0;

    float loss_derivatives[width[depth - 1]] = { 0 };

    float total_output_error = 0;


#ifdef NN_DEBUG
    Serial.println("NN: Start Backpropagation loss calc: ");
    Serial.print("N Outputs ");
    Serial.println(width[depth - 1]);
#endif // NN_DEBUG


    for (int output_neuron_n = 0; output_neuron_n < width[depth - 1]; output_neuron_n++) {
        //Calculate loss between the target and the output of the last layer
        float output_error = neuron_outputs[depth - 1][output_neuron_n] - targets[output_neuron_n];
        diff_fct_out loss = get_loss(output_error, loss_type);

        loss_derivatives[output_neuron_n] = loss.derivative;



#ifdef NN_DEBUG
        Serial.print("Output ");
        Serial.println(neuron_outputs[depth - 1][output_neuron_n]);
        Serial.print("Targets ");
        Serial.println(targets[output_neuron_n]);


        Serial.print("Loss derivatives ");
        Serial.println(loss_derivatives[output_neuron_n]);
        Serial.print("Error ");
        Serial.print(output_neuron_n);
        Serial.print(" ");
        Serial.println(output_error);
#endif // NN_DEBUG


        loss_sum += loss.x;
        total_output_error += abs(output_error);

    }

    current_error = loss_sum;

    filtered_error = loss_sum * filtered_error_alpha + (1 - filtered_error_alpha) * prev_error;
    prev_error = filtered_error;




#ifdef NN_DEBUG
    Serial.println("NN: Enter Backpropagation: ");
#endif // NN_DEBUG
    backpropagation(inputs, loss_sum, loss_derivatives);

    return total_output_error;


}


float NeuralNetwork::backpropagation(float* inputs, float loss, float* loss_derivatives) {

    float total_error = loss;

    float regularization_penalty_error = 0;

    //Handle Losses of output layer

#ifdef NN_DEBUG
    Serial.println("NN: Calculate losses of output layer ");
#endif // NN_DEBUG


    if (output_barriers) {
        barrier_loss = get_barrier_loss();
    }

    for (int output_neuron_n = 0; output_neuron_n < width[depth - 1]; output_neuron_n++) {
        //Calculate loss between the target and the output of the last layer
        float barrier_loss_grad = 0;
        if (output_barriers) {
            barrier_loss->derivative;
        }
        float loss_y = loss_derivatives[output_neuron_n];
        neuron_loss[depth - 1][output_neuron_n] = clip_gradient(loss_y + barrier_loss_grad);


#ifdef NN_DEBUG
        Serial.print("Neuron loss output");
        Serial.println(neuron_loss[depth - 1][output_neuron_n]);
#endif // NN_DEBUG

        if (output_barriers) {
            total_error += barrier_loss[output_neuron_n].x;
        }

    }

#ifdef NN_DEBUG
    Serial.println("NN: Enter start width hidden layers: ");
#endif // NN_DEBUG

    int layer, neuron, connection;

    // Calculate losses through the inner layers, excluding layer 0;

    for (layer = depth - 2; layer > 0; layer--) {

        for (neuron = 0; neuron < width[layer]; neuron++) {
            double neuron_sum = 0;

            for (connection = 0; connection < width[layer + 1]; connection++) {


                if ((connection == width[layer + 1] - 1) && ((layer + 1) != (depth - 1))) {
                    //skip since the neuron of the previous is a bias neuron, that has no connection to current layer
                    continue;
                }
                else {
                    neuron_sum += clip_gradient(neuron_loss[layer + 1][connection] * grad_activation_function(preact[layer + 1][connection], layer) * weights[layer + 1][connection][neuron]);

                }
            }
            neuron_loss[layer][neuron] = neuron_sum;
#ifdef NN_DEBUG
            Serial.print("Neuron loss ");
            Serial.print(layer);
            Serial.print(" ");
            Serial.print(neuron);
            Serial.print(" ");
            Serial.println(neuron_loss[layer][neuron]);
#endif // NN_DEBUG
        }
    }

#ifdef NN_DEBUG
    Serial.println("NN: Calculate weight adjustments");
#endif // NN_DEBUG

    // Calculate Weight Adjustments
    for (layer = depth - 1; layer > 0; layer--) {
#ifdef NN_DEBUG
        Serial.print("NN: Layer ");
        Serial.println(layer);
#endif // NN_DEBUG
        for (neuron = 0; neuron < width[layer]; neuron++) {
#ifdef NN_DEBUG
            Serial.print("NN: Neuron ");
            Serial.println(neuron);
#endif // NN_DEBUG

            if (neuron == (width[layer] - 1) && (layer != (depth - 1))) {
                continue;
            }
            for (connection = 0; connection < width[layer - 1]; connection++) {

                float regularization_grad = 0.0;
                if (regularization == ridge) {
                    regularization_grad = weights[layer][neuron][connection];

                    regularization_penalty_error += weights[layer][neuron][connection] * weights[layer][neuron][connection];
                }
                else if (regularization == lasso) {
                    if (weights[layer][neuron][connection] > 0.0) {
                        regularization_grad = 1.0;
                    }
                    else if (weights[layer][neuron][connection] < 0.0) {
                        regularization_grad = -1.0;
                    }

                    regularization_penalty_error += weights[layer][neuron][connection];
                }
#ifdef NN_DEBUG
                Serial.print("NN: Connection ");
                Serial.println(connection);
#endif // NN_DEBUG
                float new_weight_grad = clip_gradient(neuron_loss[layer][neuron] *
                    grad_activation_function(preact[layer][neuron], layer) * neuron_outputs[layer - 1][connection]
                    + reg_penalty_factor * regularization_grad);
                if (backprop_batch_mode) {
                    weights_grad[layer][neuron][connection] = new_weight_grad;
                }
                else {
                    weights_grad[layer][neuron][connection] = new_weight_grad;
                }




#ifdef NN_DEBUG
                Serial.print("Weight adjustment loss ");
                Serial.print(layer);
                Serial.print(" ");
                Serial.print(neuron);
                Serial.print(" ");
                Serial.print(connection);
                Serial.print(" ");
                Serial.println(weights_grad[layer][neuron][connection]);
#endif // NN_DEBUG
            }
        }
    }

    total_error = total_error + regularization_penalty_error * regularization_penalty_error;

    return total_error;


}

void NeuralNetwork::apply_gradient_descent(grad_descent_update_rule update_method) {



#ifdef NN_DEBUG
    Serial.println("NN: Apply Gradient Descent");
#endif // NN_DEBUG

    apply_learning_rate_scheduler();

    static float beta_1_raised = 1.0;
    static float beta_2_raised = 1.0;

    const float adam_epsilon = 1e-4;

    n_iterations++;

    // Apply weight adjustments --- ADAM Implementation

    for (int layer = 1; layer < depth; layer++) {
        for (int neuron = 0; neuron < width[layer]; neuron++) {
            for (int connection = 0; connection < width[layer - 1]; connection++) {

                if (!((neuron == width[layer] - 1) && (layer != depth - 1))) { // bias neurons are not connected previous layers



                    if (update_method == sgd) {


#ifdef NN_DEBUG
                        Serial.print("Apply Weight adjustment  ");
                        Serial.print(layer);
                        Serial.print(" ");
                        Serial.print(neuron);
                        Serial.print(" ");
                        Serial.print(connection);
                        Serial.print(" Before:");
                        Serial.print(weights[layer][neuron][connection]);

                        if (abs(weights_grad[layer][neuron][connection]) > __FLT_MAX__) {
                            weights_grad[layer][neuron][connection] = 0.01 * weights_grad[layer][neuron][connection];
                        }
#endif // NN_DEBUG

                        weights[layer][neuron][connection] -= learning_rate * weights_grad[layer][neuron][connection];
#ifdef NN_DEBUG
                        Serial.print(" After:");
                        Serial.println(weights[layer][neuron][connection]);
#endif
                    }

                    else if (update_method == adam) {
                        adam_m[layer][neuron][connection] = beta_1 * adam_m[layer][neuron][connection] + (1 - beta_1) * weights_grad[layer][neuron][connection];

                        float grad_second_momentum = weights_grad[layer][neuron][connection] * weights_grad[layer][neuron][connection];

                        adam_v[layer][neuron][connection] = beta_2 * adam_v[layer][neuron][connection] + (1 - beta_2) * grad_second_momentum;

                        beta_1_raised = beta_1_raised * beta_1;
                        beta_2_raised = beta_2_raised * beta_2;

                        float m_hat = adam_m[layer][neuron][connection] / (1 - beta_1_raised);
                        float v_hat = adam_v[layer][neuron][connection] / (1 - beta_2_raised);

                        //float weight_update = m_hat / (sqrtf(v_hat) + adam_epsilon);
                        float weight_update = m_hat * fast_inv_sqrt(v_hat + adam_epsilon);


                        weights[layer][neuron][connection] -= learning_rate * weight_update;
                    }

                }

            }
        }
    }




}

float** NeuralNetwork::get_network_derivative(float* input_vector, bool finite_differerences) {

    // Finite differences:

    if (finite_differerences) {

        float epsilon = 1e-3;
        float input_vector_neg[width[0]];
        float input_vector_pos[width[0]];
        // iterate over network inputs
        for (int i = 0; i < width[0]; i++) {
            // iterate over network outputs
            for (int n = 0; n < width[depth - 1]; n++) {
                input_vector_neg[i] = input_vector[i] - epsilon;
                input_vector_pos[i] = input_vector[i] + epsilon;

                float* y_e_neg = predict(input_vector_neg);
                float* y_e_pos = predict(input_vector_pos);

                network_jacobian[n][i] = (y_e_pos[i] - y_e_neg[i]) / (2.0 * epsilon);
            }
        }

    }
    else {
        // Do modified backprop

        // output derivative
        for (int output_neuron = 0; output_neuron < width[depth - 1]; output_neuron++) {

            neuron_derivatives[depth - 1][output_neuron] = 1.0;
            for (int n_output = 0; n_output < width[depth - 1]; n_output++) {
                if (n_output == output_neuron) {
                    neuron_derivatives[depth - 1][output_neuron] = 1.0;
                }
                else {
                    neuron_derivatives[depth - 1][output_neuron] = 0.0;
                }
            }

#ifdef NN_DEBUG
            Serial.println("NN: Enter start width hidden layers: ");
#endif // NN_DEBUG

            int layer, neuron, connection;

            // Calculate derivatives through the inner layers
            for (layer = depth - 2; layer >= 0; layer--) {

                for (neuron = 0; neuron < width[layer]; neuron++) {
                    double neuron_sum = 0;

                    for (connection = 0; connection < width[layer + 1]; connection++) {


                        if (connection == width[layer + 1] - 1 && layer + 1 != depth - 1) {
                            //skip since the neuron of the previous is a bias neuron, that has no connection to current layer
                            continue;
                        }
                        else {
                            neuron_sum += neuron_derivatives[layer + 1][connection] * grad_activation_function(preact[layer + 1][connection], layer) * weights[layer + 1][connection][neuron];

                        }
                    }
                    neuron_derivatives[layer][neuron] = neuron_sum;

                }
            }

            for (int input_neuron = 0; input_neuron < width[0]; input_neuron++) {
                network_jacobian[output_neuron][input_neuron] = neuron_derivatives[0][input_neuron];

            }
        }
    }



    return network_jacobian;

}

void NeuralNetwork::init_backprop_differentiation(bool allow_backprop_derivative) {

    // allocate memory for intermediate derivatives

    if (allow_backprop_derivative) {

        neuron_derivatives = new float* [depth];
        for (int layer = 0; layer < depth; layer++) {
            neuron_derivatives[layer] = new float[width[layer]];
        }
    }

    backprop_jacobian_initialized = allow_backprop_derivative;


}


diff_fct_out NeuralNetwork::get_loss(float x, nn_loss_function loss_type) {

    diff_fct_out loss;
    if (loss_type == MSE) {
        loss.x = 0.5 * x * x;
        loss.derivative = x;

        return loss;
    }

    if (loss_type == MAE) {
        loss.x = abs(x);

        if (x > 0) {
            loss.derivative = 1.0;
        }
        else {
            loss.derivative = -1.0;
        }
    }

    if (loss_type == huber_loss) {
        if (abs(x) < huber_threshold) {
            loss.x = 0.5 * x * x;
            loss.derivative = x;
        }
        if (abs(x) >= huber_threshold) {
            loss.x = (abs(x) - 0.5 * huber_threshold);
            if (x >= 0) {
                loss.derivative = 1.0;
            }
            else {
                loss.derivative = -1.0;
            }
        }
    }

    return loss;


}

void NeuralNetwork::init_weights_randomly(float max, float min) {

    // Initialize weights

    for (int layer = 1; layer < depth; layer++) {
        for (int neuron = 0; neuron < width[layer]; neuron++) {

            if ((layer != depth - 1) && (neuron == width[layer] - 1)) {
                continue;
            }
            for (int input = 0; input < width[layer - 1]; input++) {
                if (activation_function_per_layer[layer] == Linear) {

                    /* [min, max] */

                    weights[layer][neuron][input] = random_float(min, max);
                }
                else {
                    weights[layer][neuron][input] = random_float(max + 0.25 * max, min + 0.25 * min);
                }
                adam_m[layer][neuron][input] = 0.0;
                adam_v[layer][neuron][input] = 0.0;
            }
        }
    }
}


float NeuralNetwork::apply_activation_function(float x, int layer) {

    nn_activation_f function_type = activation_function_per_layer[layer - 1];

    if (function_type == ReLu) {
        return f_ReLu(x);
    }
    if (function_type == Linear) {
        return x;
    };
    if (function_type == leakyReLu) {
        return f_leakyReLu(x);
    }
    return x;
}

float NeuralNetwork::grad_activation_function(float x, int layer) {

    nn_activation_f function_type = activation_function_per_layer[layer - 1];

    if (function_type == ReLu) {
        return grad_ReLu(x);
    }
    if (function_type == Linear) {
        return 1.0;
    };
    if (function_type == leakyReLu) {
        return grad_leakyReLu(x);
    }

    return 1.0;
}


float NeuralNetwork::random_float(float max, float min) {

    float scale = rand() / (float)RAND_MAX; /* [0, 1.0] */
    return min + scale * (max - min);      /* [min, max] */
}

float NeuralNetwork::f_ReLu(float x) {

    if (x > 0.0) {

        return x;
    }
    else {

        return 0.0;
    }


}

float NeuralNetwork::grad_ReLu(float x) {

    if (x > 0) {
        return 1.0;
    }
    return 0.0;
}

float NeuralNetwork::f_leakyReLu(float x) {
    if (x >= 0) {
        return x;
    }
    else {
        return 0.01 * x;
    }
}
float NeuralNetwork::grad_leakyReLu(float x) {
    if (x >= 0) {
        return 1.0;
    }
    else {
        return 0.01 * x;
    }
}

void NeuralNetwork::set_output_barriers(bool limit_max[], bool limit_min[], float outputs_min[], float outputs_max[]) {

    this->output_limits_max = limit_max;
    this->output_limits_min = limit_min;
    this->outputs_max = outputs_max;
    this->outputs_min = outputs_max;

    output_barriers = false;
    for (int output_neuron = 0; output_neuron < width[depth - 1]; output_neuron++) {
        if (limit_max[output_neuron] || limit_min[output_neuron]) {
            output_barriers = true;
        }
    }
}


diff_fct_out* NeuralNetwork::get_barrier_loss() {

    barrier_loss = new diff_fct_out[width[depth - 1]];
    for (int output_neuron_n = 0; output_neuron_n < width[depth - 1]; output_neuron_n++) {
        barrier_loss[output_neuron_n].x = 0.0;
        barrier_loss[output_neuron_n].derivative = 0.0;

        if (output_limits_max[output_neuron_n]) {
            float neuron_output = neuron_outputs[depth - 1][output_neuron_n];
            float distance2max = neuron_output - outputs_max[output_neuron_n];
            float loss_max = -barrier_softness * log(-distance2max);
            float loss_max_derivative = -barrier_softness / (-distance2max);

            barrier_loss[output_neuron_n].x += loss_max;
            barrier_loss[output_neuron_n].derivative += loss_max_derivative;
        }

        if (output_limits_min[output_neuron_n]) {
            float neuron_output = neuron_outputs[depth - 1][output_neuron_n];
            float distance2min = outputs_min[output_neuron_n] - neuron_output;
            float loss_min = -barrier_softness * log(-distance2min);
            float loss_min_derivative = -barrier_softness / (-distance2min);

            barrier_loss[output_neuron_n].x += loss_min;
            barrier_loss[output_neuron_n].derivative += loss_min_derivative;
        }

    }

    return barrier_loss;


}


float NeuralNetwork::clip_gradient(float input_grad) {

    if (input_grad > max_gradient_abs) {
        return max_gradient_abs;
    }
    if (input_grad < -max_gradient_abs) {
        return -max_gradient_abs;
    }

    return input_grad;
}

float NeuralNetwork::fast_inv_sqrt(float number) {


    /* Magic Fast Inverse Square Root */
    // Uses forbidden 32bit floating point bit shifting 
    // https://en.wikipedia.org/wiki/Fast_inverse_square_root
    // All Credit goes to geniuses from the 80ies

    union {
        float    f;
        uint32_t i;
    } conv = { .f = number };
    conv.i = 0x5f3759df - (conv.i >> 1); //magic number 
    conv.f *= 1.5F - (number * 0.5F * conv.f * conv.f);
    return conv.f;
}

nn_model_weights NeuralNetwork::get_model_weights() {

    int current_weight = 0;
    for (int layer = 1; layer < depth; layer++) {
        for (int neuron = 0; neuron < width[layer]; neuron++) {
            for (int input = 0; input < width[layer - 1]; input++) {


                if (neuron == width[layer] - 1 && layer != depth - 1) {
                    continue;
                }
                else {
                    Serial.println(current_weight);
                    weight_vector[current_weight] = weights[layer][neuron][input];
                    current_weight++;
                }
            }
        }
    }

    nn_model_weights nn_weight_data;
    nn_weight_data.n_weights = n_weights;
    nn_weight_data.weights = weight_vector;

    return nn_weight_data;



}
void NeuralNetwork::load_model_weights(nn_model_weights nn_weights) {

    if (nn_weights.n_weights != n_weights) {
        return;
    }

    int current_weight = 0;
    for (int layer = 1; layer < depth; layer++) {
        for (int neuron = 0; neuron < width[layer]; neuron++) {
            for (int input = 0; input < width[layer - 1]; input++) {

                if (neuron == width[layer] - 1 && layer != depth - 1) {
                    continue;
                }
                else {
                    weights[layer][neuron][input] = nn_weights.weights[current_weight];
                    current_weight++;
                }
            }
        }
    }

}

void NeuralNetwork::apply_learning_rate_scheduler() {

    int n_iterations_since_restart = 0;

    if (lr_schedule == cosine_annealing) {

        n_iterations_since_restart = n_iterations % int((n_warm_restart_period * n_warm_restart_multiplier));

        learning_rate = minimal_learning_rate + 0.5 * (maximal_learning_rate - minimal_learning_rate) * (1 + cosf(PI * (float(n_iterations_since_restart) / float((n_warm_restart_period * n_warm_restart_multiplier)))));

        if (cosine_decay) {
            learning_rate = learning_rate / (1 + decay_factor * n_iterations);
        }
    }
    if (lr_schedule == time_decay) {
        if (cosine_decay) {
            learning_rate = learning_rate / (1 + decay_factor * n_iterations);
        }
    }
    if (lr_schedule == error_adaptive) {

        learning_rate = minimal_learning_rate + lr_error_factor * current_error;
    }
    if (lr_schedule == error_adaptive_filtered) {

        learning_rate = minimal_learning_rate + lr_error_factor * filtered_error;
    }



}


void NeuralNetwork::printWeights() {
    nn_model_weights weights = get_model_weights();

    /*
    for (int i = 0; i < weights.n_weights; i++) {
        Serial.print(i);
        Serial.print(" ");
        Serial.println(weights.weights[i]);
    }
    */
}





