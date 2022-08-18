#ifndef NEURAL_NETWORK_H
#define NEURAL_NETWORK_H

//#define NN_DEBUG


enum nn_activation_f { Linear, ReLu, leakyReLu };

enum nn_loss_function { MSE, MAE, huber_loss };

enum nn_regularization { none, ridge, lasso };

enum nn_learning_rate_schedule { no_schedule, time_decay, cosine_annealing, error_adaptive, error_adaptive_filtered };

struct sample_t {
    float* inputs;
    float* outputs;
};

struct batch_t {
    sample_t* samples;
    int n_samples;
};


struct diff_fct_out {
    float x;
    float derivative;
};

struct sample_loss_t {
    float* input;
    diff_fct_out* loss;
};

struct batch_loss_t {
    sample_loss_t* samples;
    int n_samples;
};


struct nn_model_weights {
    int n_weights;
    float* weights;
};

enum grad_descent_update_rule { sgd, adam };

class NeuralNetwork {
public:

    NeuralNetwork(const int depth, int width[], nn_activation_f f_act_per_layer[]);
    ~NeuralNetwork();
    void propagate_forward();
    float train_SGD(float* inputs, float* targets);
    float train_SGD_S(sample_t samples[], int n_samples);
    float train_SGD_ext_loss(float* inputs, float ext_loss, float* ext_loss_derivative);
    float train_batch(batch_t batch);
    float train_batch(batch_loss_t batch_ext_loss);

    float backpropagation(float* inputs, float* targets);
    float backpropagation(float* inputs, float loss, float* loss_derivatives);

    float* predict(float input_vector[]);

    void predict(float input_vector[], float output_vector[]);
    void set_input(float input_vector[]);
    float* get_output();

    void init_weights_randomly(float max, float min);

    /**
     * @brief Computes the jacobian of the neural network w.r.t to its inputs y'= f'(x)
     *
     * @param input_vector current
     * @param finite_differences
     * @return float** Jacobian matrix n*i n - outputs, i - inputs[output_n][input_i]
     */
    float** get_network_derivative(float* input_vector, bool finite_differences = true);
    void apply_gradient_descent(grad_descent_update_rule update_method = adam);

    nn_model_weights get_model_weights();
    void load_model_weights(nn_model_weights weights_data);

    void set_output_barriers(bool limit_max[], bool limit_min[], float outputs_min[], float outputs_max[]);

    /**
     * @brief Allocates memory to store the intermediate variables required for differntiating via backprop
     * the network w.r.t its inputs -> get the derivative if the network (see get_network_derivative)
     * If function is not called, the network derivative is taken via finite differences
     * -> more computation time, but less memory
     *
     * @param allow_backprop_differentiation
     */
    void init_backprop_differentiation(bool allow_backprop_differentiation);

    void printWeights();

    // Model Parameters
    const int depth;
    const int* width; //array of size depth
    nn_activation_f const* activation_function_per_layer; // array of size depth-1
    float*** weights;
    int n_weights = 0;


    // Hyperparameters
    float learning_rate = 0.01;
    nn_loss_function loss_type = MSE;
    grad_descent_update_rule update_rule = adam;

    //Regularization
    nn_regularization regularization = none;
    float reg_penalty_factor = 1e-8;
    float barrier_softness = 1e-3;
    // Gradient Clipping
    float max_gradient_abs = 100.0;

    //Error estimate
    float filtered_error = 0.0;
    float filtered_error_alpha = 1e-2;
    float filtered_error_td = 0.0;

    //Threshold for huber loss
    float huber_threshold = 1.35;


    //Learning Rate scheduler
    int n_iterations = 0;
    float lr_error_factor = 1e-3;
    nn_learning_rate_schedule lr_schedule = no_schedule;
    float decay_factor = 1e-4;
    float minimal_learning_rate = 1e-3;
    float maximal_learning_rate = 8e-3;
    int n_warm_restart_period = 1e4;
    float n_warm_restart_multiplier = 1.25;
    bool cosine_decay = true;

private:

    // Internal values
    float** neuron_outputs;

    // Internal Training Values
    float** neuron_loss; // error derivative w.r.t to neuron output
    float** preact; //  neuron preactivation
    float*** weights_grad; //delta weights 

    float* weight_vector;
    float backprop_batch_mode = false;

    float current_error = 0.0;
    float prev_error = 0.0;

    float* targets;

    float** neuron_derivatives; //for backpropagation to calculate the networks gradient net y = f(x) -> y' = f'(x')
    float** network_jacobian;

    bool backprop_jacobian_initialized = false;

    // Core Functions for Training
    float apply_activation_function(float x, int layer);
    float grad_activation_function(float x, int layer);

    diff_fct_out get_loss(float x, nn_loss_function loss_type);
    float clip_gradient(float input_grad);

    // Activation Functions
    float f_ReLu(float x);
    float grad_ReLu(float x);
    float f_leakyReLu(float x);
    float grad_leakyReLu(float x);

    // Gradient Descent Method Variables
    const float beta_1 = 0.9;
    const float beta_2 = 0.999;
    float*** adam_m;
    float*** adam_v;

    // Output Barrier Parameters
    bool output_barriers = false;
    bool* output_limits_max;
    bool* output_limits_min;
    float* outputs_max;
    float* outputs_min;
    diff_fct_out* barrier_loss;


    //Helper Functions
    diff_fct_out* get_barrier_loss();
    float fast_inv_sqrt(float number);
    float random_float(float max, float min);

    void error_adaptive_learning_rate(float min_learning_rate, float max_learning_rate, float slope);
    void apply_learning_rate_scheduler();


};




#endif // !NEURAL_NETWORK_H
