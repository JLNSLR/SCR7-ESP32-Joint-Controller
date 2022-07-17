#include <NN/nn_utils.h>
#include <Preferences.h>

void nn_save_model_weights_on_flash(nn_model_weights weight_data, char model_name[]) {

    Preferences pref;
    pref.begin(model_name);

    int n_bytes = weight_data.n_weights * sizeof(float);

    char* weight_bytes = new char[n_bytes];

    memcpy(weight_bytes, weight_data.weights, n_bytes);

    pref.putBytes("nn_w", weight_bytes, n_bytes);

    pref.putBool("nn_saved", true);

    pref.end();

    delete weight_bytes;
}

nn_model_weights nn_load_model_weights_from_flash(char model_name[], int n_weights) {

    Preferences pref;
    pref.begin(model_name);

    nn_model_weights weight_data;


    if (!pref.getBool("nn_saved")) {
        weight_data.n_weights = 0;
        return weight_data;
    }


    int n_bytes = n_weights * sizeof(float);
    char* weight_bytes = new char[n_bytes];

    pref.getBytes("nn_w", weight_bytes, n_bytes);


    float* weight_vector = new float[n_bytes];

    memcpy(weight_vector, weight_bytes, n_bytes);

    pref.end();

    delete weight_bytes;

    weight_data.n_weights = n_weights;
    weight_data.weights = weight_vector;

    return weight_data;

}

void nn_clear_data_on_flash(char model_name[]) {

    Preferences pref;
    pref.begin(model_name);

    pref.putBool("nn_saved", false);

    pref.clear();

    pref.end();

}
