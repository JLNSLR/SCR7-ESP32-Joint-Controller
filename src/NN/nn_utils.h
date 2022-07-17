#ifndef NN_UTILS_H
#define NN_UTILS_H

#include<Arduino.h>
#include <NN/NeuralNetwork.h>

void nn_save_model_weights_on_flash(nn_model_weights weight_data, char model_name[]);

/**
 * @brief loads model weights from flash if they are available. If no data is available
 * the n_weights parameter of is set to zero
 *
 * @param model_name - name to identify the nn on the flash
 * @param n_weights
 * @return nn_model_weights
 */
nn_model_weights nn_load_model_weights_from_flash(char model_name[], int n_weights);

void nn_clear_data_on_flash(char model_name[]);



#endif // !NN_UTILS_H