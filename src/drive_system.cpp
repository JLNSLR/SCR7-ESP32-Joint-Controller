#include <drive_system.h>
#include "soc/timer_group_struct.h"
#include "soc/timer_group_reg.h"
#include <signal_processing/IIRFilter.h>

/* ####################################################################
############### RTOS AND TIMING PARAMETERS ############################
######################################################################*/
/*Shared Variables Semaphores */
SemaphoreHandle_t drvSys_mutex_motor_position = xSemaphoreCreateBinary();
SemaphoreHandle_t drvSys_mutex_motor_vel = xSemaphoreCreateBinary();
SemaphoreHandle_t drvSys_mutex_motor_acc = xSemaphoreCreateBinary();;

SemaphoreHandle_t drvSys_mutex_joint_position = xSemaphoreCreateBinary();
SemaphoreHandle_t drvSys_mutex_joint_vel = xSemaphoreCreateBinary();
SemaphoreHandle_t drvSys_mutex_joint_acc = xSemaphoreCreateBinary();;

SemaphoreHandle_t drvSys_mutex_joint_torque = xSemaphoreCreateBinary();

SemaphoreHandle_t drvSys_mutex_torque_target = xSemaphoreCreateBinary();
SemaphoreHandle_t drvSys_mutex_motor_commanded_torque = xSemaphoreCreateBinary();
SemaphoreHandle_t drvSys_mutex_position_command = xSemaphoreCreateBinary();;

/*Hardware Timer Setup Variables*/
const uint16_t drvSys_timer_prescaler_divider = DRVSYS_TIMER_PRESCALER_DIV; // with 80MHz Clock, makes the timer tick every 1us
const uint64_t drvSys_timer_alarm_rate_us = DRVSYS_TIMER_ALARM_RATE_US; //generate timer alarm every 50us

hw_timer_t* drvSys_foc_timer;
volatile const  int32_t drvSys_timer_foc_ticks = DRVSYS_FOC_PERIOD_US / drvSys_timer_alarm_rate_us;
volatile const int32_t drvSys_timer_encoder_process_ticks = DRVSYS_PROCESS_ENCODERS_PERIOD_US / drvSys_timer_alarm_rate_us;
volatile const int32_t drvSys_timer_torque_control_ticks = DRVSYS_CONTROL_TORQUE_PERIOD_US / drvSys_timer_alarm_rate_us;
volatile const int32_t drvSys_timer_pos_control_ticks = DRVSYS_CONTROL_POS_PERIOD_US / drvSys_timer_alarm_rate_us;


/*###########################################################################
################### Drive System Object Definitions #########################
############################################################################*/

/* Drive System General Variables */

/* Drive Constants */

const drvSys_Constants drvSys_constants = { .nominal_current_mA = DRVSYS_PHASE_CURRENT_NOMINAL_mA,
    .transmission_ratio = DRVSYS_TRANSMISSION_RATIO,
    .joint_id = JOINT_ID,
    .motor_torque_constant = DRVSYS_TORQUE_CONSTANT };




/*######## Changing Parameter Initial Parameters ####### */
/* Drive System Control Mode */
drvSys_controlMode drvSys_mode = dual_control;
drvSys_StateFlag drvSys_state_flag = not_ready;

/* Drive State Parameters */
drvSys_controllerCondition drvSys_controller_state = { .control_mode = drvSys_mode,
.state_flag = drvSys_state_flag,
.calibrated = false,
.hit_neg_limit = false,
.hit_positive_limit = false,
.overtemperature = false,
.temperature_warning = false,
.temperature = 20 };


drvSys_parameters drvSys_parameter_config;


/* #####################################################################################
####################### ----- Drive Components  ------ #################################
###################################################################################### */


AS5048A drvSys_magnetic_motor_encoder(CS_ENCODER);
AS5048A drvSys_magnetic_joint_encoder(CS_JOINT_ENCODER);

TMC2160Stepper drvSys_driver(CS_TMC, R_SENSE);

/* Dynamic Kalman Filter */
//DriveDynamicKalmanFilter kalman_filter(DRVSYS_FOC_PERIOD_US * 1e-6, DRVSYS_TRANSMISSION_RATIO);
KinematicKalmanFilter motor_kinematic_kalman_filter(DRVSYS_FOC_PERIOD_US * 1e-6);
KinematicKalmanFilter joint_kinematic_kalman_filter(DRVSYS_PROCESS_ENCODERS_PERIOD_US * 1e-6);

/* FOC Controller */
FOCController drvSys_foc_controller(&drvSys_magnetic_motor_encoder, &drvSys_driver, DRVSYS_PHASE_CURRENT_MAX_mA, DRVSYS_TORQUE_CONSTANT, glob_SPI_mutex);


// Torque Sensor TODO

// Neural Network to learn inverse dynamics
InverseDynamicsLearner* drvSys_nn_inverse_dynamics;
unsigned int drvSys_max_learning_iterations_per_step = DRVSYS_MAX_LEARNING_ITERATION_PER_STEP;
float drvSys_inv_dyn_pred_torque = 0.0;


//Neural Network to learn PID Parameters
PID_Tuner_NN* drvSys_pid_tune_nn;
bool drvSys_pid_tuner_active = DRVSYS_PID_GAIN_LEARNING_ACTIVE;

// Axis alignment Variables

bool drvSys_flip_global_alignment = DRVSYS_GLOB_DIR_FLIP;

float drvSys_torque_dir_align = DRVSYS_TORQUE_ALIGN_DIR;
float drvSys_motor_encoder_dir_align = DRVSYS_MOTOR_ENC_ALIGN_DIR;
float drvSys_joint_encoder_dir_align = DRVSYS_JOINT_ENC_ALIGN_DIR;
float drvSys_torque_sensor_dir_align = DRVSYS_TORQUE_ALIGN_DIR;

bool drvSys_axis_aligned_flag = false;


bool nn_inv_initialized = false;
bool nn_pid_initialized = false;

float drvSys_angle_offset_motor = 0;
float drvSys_angle_offset_joint = 0;


// Axis Calibration Variables

int32_t drvSys_joint_encoder_offset = DRVSYS_RAW_JOINT_ENC_OFFSET;
int32_t drvSys_joint_zero_set = false;
int32_t drvSys_motor_encoder_offset = DRVSYS_RAW_MOTOR_ENC_OFFSET;
int32_t drvSys_motor_encoder_rollovers = 0;

bool drvSys_joint_calibrated_flag = false;

/* Differentiators */
Differentiator drvSys_differentiator_motor_pos(DRVSYS_PROCESS_ENCODERS_FREQU);

/* ########## Controllers ################ */
PIDController drvSys_position_controller(0, 0, 0);
float drvSys_vel_ff_gain = DRVSYS_VEL_FF_GAIN;


/* --- Drive System Main State Variables --- */
// Motor Kinematic State
float drvSys_motor_position;
float drvSys_motor_velocity;
float drvSys_motor_acc;
// Joint Kinematic State
float drvSys_joint_position;
float drvSys_joint_velocity;
float drvSys_joint_acc;
// Motor Torque 
float drvSys_motor_torque_commanded = 0;
// Joint Torque Sensor Value
float drvSys_joint_torque = 0.0;


// Peripheral Sensor Values
int32_t drvSys_hall_sensor_val = 0;
float drvSys_motor_temperature = 20.0;


/* Notch Filters */

drvSys_notch_filter_params drvSys_notch_filters;

IIRFilter<2> notch_filter_0;
IIRFilter<2> notch_filter_1;

//define Task Handlers
TaskHandle_t drvSys_foc_th;
TaskHandle_t drvSys_process_encoders_th;
TaskHandle_t drvSys_torque_controller_th;
TaskHandle_t drvSys_PID_dual_controller_th;
TaskHandle_t drvSys_process_joint_encoder_th;
TaskHandle_t drvSys_process_torque_sensor_th;
TaskHandle_t drvSys_admittance_controller_th;
TaskHandle_t drvSys_learn_dynamics_th;
TaskHandle_t drvSys_tune_pid_th;
TaskHandle_t drvSys_handle_periphal_sensors_th;




/* #############################################################
######################## Target values #########################
###############################################################*/
float drvSys_pos_target = 0;
float drvSys_vel_target = 0;
float drvSys_acc_target = 0;
float drvSys_torque_ff = 0.0;

float _drvSys_torque_target = 0;

// used to differentiate between torque_ff and reference torque for admittance control
float drvSys_joint_torque_ref = 0.0;


/* ####### Drive System Preferences ########### */

//used to save parameters into flash memory
Preferences drv_sys_preferences;
//Namespaces for Parameters
// PID
const char* drvSys_posPID_saved_gains = "posGains";
const char* drvSys_velPID_saved_gains = "velGains";
const char* drvSys_admittance_saved_gains = "admGains";
const char* drvSys_saved_offsets = "offs";
// Offset Calibration
const char* drvSys_encoder_offset = "posOffset";
// Alignment
const char* drvSys_alignment_setting = "align";

//Estimated Parameters
const char* drvSys_dynamic_params = "dynParams";


/* ###############################################################
##################################################################
################## Function Implementations ######################
##################################################################
################################################################ */



const drvSys_driveState drvSys_get_drive_state() {
    drvSys_driveState state;
    xSemaphoreTake(drvSys_mutex_joint_position, portMAX_DELAY);
    state.joint_pos = drvSys_joint_position;
    xSemaphoreGive(drvSys_mutex_joint_position);

    xSemaphoreTake(drvSys_mutex_joint_vel, portMAX_DELAY);
    state.joint_vel = drvSys_joint_velocity;
    xSemaphoreGive(drvSys_mutex_joint_vel);

    xSemaphoreTake(drvSys_mutex_joint_acc, portMAX_DELAY);
    state.joint_acc = drvSys_joint_acc;
    xSemaphoreGive(drvSys_mutex_joint_acc);

    xSemaphoreTake(drvSys_mutex_joint_torque, portMAX_DELAY);
    state.joint_torque = drvSys_joint_torque;
    xSemaphoreGive(drvSys_mutex_joint_torque);

    state.motor_torque = drvSys_motor_torque_commanded;


    return state;
};

const drvSys_FullDriveState drvSys_get_full_drive_state() {
    drvSys_FullDriveState state;

    xSemaphoreTake(drvSys_mutex_joint_position, portMAX_DELAY);
    state.joint_pos = drvSys_joint_position;
    xSemaphoreGive(drvSys_mutex_joint_position);

    xSemaphoreTake(drvSys_mutex_joint_vel, portMAX_DELAY);
    state.joint_vel = drvSys_joint_velocity;
    xSemaphoreGive(drvSys_mutex_joint_vel);

    xSemaphoreTake(drvSys_mutex_joint_acc, portMAX_DELAY);
    state.joint_acc = drvSys_joint_acc;
    xSemaphoreGive(drvSys_mutex_joint_acc);

    xSemaphoreTake(drvSys_mutex_joint_torque, portMAX_DELAY);
    state.joint_torque = drvSys_joint_torque;
    xSemaphoreGive(drvSys_mutex_joint_torque);

    xSemaphoreTake(drvSys_mutex_motor_position, portMAX_DELAY);
    state.motor_pos = drvSys_motor_position;
    xSemaphoreGive(drvSys_mutex_motor_position);

    xSemaphoreTake(drvSys_mutex_motor_vel, portMAX_DELAY);
    state.motor_vel = drvSys_motor_velocity;
    xSemaphoreGive(drvSys_mutex_motor_vel);

    xSemaphoreTake(drvSys_mutex_motor_acc, portMAX_DELAY);
    state.motor_acc = drvSys_motor_acc;
    xSemaphoreGive(drvSys_mutex_motor_acc);

    state.motor_torque = drvSys_motor_torque_commanded;

    return state;

};

drvSys_parameters& drvSys_get_parameters() {
    return drvSys_parameter_config;
};

const drvSys_controllerCondition drvSys_get_controllerState() {
    return drvSys_controller_state;
};


const drvSys_Constants drvSys_get_constants() {
    return drvSys_constants;
}
void drvSys_set_target(drvSys_driveTargets targets) {

    _drvSys_set_target_pos(targets.pos_target);
    _drvSys_set_target_velocity(targets.vel_target);
    drvSys_acc_target = targets.acc_target;
    drvSys_set_feed_forward_torque(targets.motor_torque_ff);
    drvSys_joint_torque_ref = targets.ref_torque;

}

void drvSys_limit_torque(float torque_limit) {
    drvSys_parameter_config.max_torque_Nm = torque_limit;
}

void _drvSys_calibrate_with_hallsensor() {


    Serial.println("DRVSYS_INFO: Start Calibration via Hall sensor");
    pinMode(HALL_SENSOR_PIN, INPUT);

    int32_t shift = DRVSYS_ANGLE_ORIGIN_2_HALL_DEG * (16383.0 / 360.0);

    drvSys_magnetic_motor_encoder.resetAbsolutZero();
    drvSys_magnetic_joint_encoder.resetAbsolutZero();



    drvSys_hall_sensor_val = analogRead(HALL_SENSOR_PIN);

    int dir = 0;

    pinMode(TMC_STEP, OUTPUT);
    pinMode(TMC_DIR, OUTPUT);

    digitalWrite(TMC_DIR, dir);

    drvSys_foc_controller.driver->direct_mode(false);
    //Reduce phase current to nominal value in stepper mode, to remain a cooler motor
    drvSys_foc_controller.driver->rms_current(DRVSYS_PHASE_CURRENT_NOMINAL_mA);

    float average_hall_value = drvSys_hall_sensor_val;
    float largest_hall_value = 0;
    bool calibration_finished = false;

    int32_t raw_joint_angle = 0;
    int32_t raw_motor_angle = 0;
    int motor_rollovers = 0;

    int n_falling = 0;

    float limit_angle = DRVSYS_CAL_ANGLE_LIMIT;

    const int AS5048A_ANGLE = 0x3FFF;


    word zero_pos_joint = 0;

    const int buffer_size = 50;
    CircularBuffer<float, buffer_size> buffer;
    CircularBuffer<float, buffer_size> buffer_noise;

    bool found_peak = false;
    int stepcount = DRVSYS_CAL_LARGE_STEPCOUNT;
    float prev_hall_val = analogRead(HALL_SENSOR_PIN);

    float start_angle = drvSys_magnetic_joint_encoder.getRotationDeg();


    float noise_multiplier = 10;

    int steps_since_maxima = 0;
    int count_no_maxima = 0;

    int n_rollovers_at_zero = 0;



    while (!calibration_finished) {

        //take 100 Hall sensor readings
        float current_average_val = 0;
        int n_averages = 100;
        for (int i = 0; i < n_averages; i++) {
            current_average_val += analogRead(HALL_SENSOR_PIN);
        }
        current_average_val = current_average_val / float(n_averages);

        // calculate noise level
        buffer_noise.unshift(current_average_val - prev_hall_val);
        prev_hall_val = current_average_val;

        buffer.unshift(current_average_val);

        // Calculate moving average hall value
        float buffer_sum = 0;
        for (int i = 0; i < buffer.size(); i++) {
            buffer_sum += buffer[i];
        }
        average_hall_value = buffer_sum / buffer.size();

        // Calculate moving average noise level
        float buffer_sum_noise = 0;
        for (int i = 0; i < buffer_noise.size(); i++) {
            buffer_sum_noise += abs(buffer_noise[i]);
        }
        float average_noise_val = abs(buffer_sum_noise / buffer_noise.size());

        /* Output Calibration Hall values */
        Serial.print("Current Hall value: ");
        Serial.print(current_average_val);
        Serial.print(", ");
        Serial.print(" Average Hall value: ");
        Serial.print(average_hall_value);
        Serial.print(", ");
        Serial.print(" Average Hall noise value: ");
        Serial.println(average_noise_val);

        if (current_average_val > average_hall_value + average_noise_val * noise_multiplier) {
            if (current_average_val > largest_hall_value) {


                n_rollovers_at_zero = drvSys_magnetic_motor_encoder._n_rollovers;
                raw_motor_angle = drvSys_magnetic_motor_encoder.getRawRotation(true);

                zero_pos_joint = drvSys_magnetic_joint_encoder.getRawRotation(true);

                /*
                zero_pos_joint = drvSys_magnetic_joint_encoder.getRawRotation(true);
                zero_pos_motor = drvSys_magnetic_motor_encoder.getRawRotation(true);
                */

                noise_multiplier = 1;

                largest_hall_value = current_average_val;

                Serial.print(" New Hall maximum at: ");
                Serial.println(zero_pos_joint);

                found_peak = true;
                //reduce step coint 
                stepcount = DRVSYS_CAL_SMALL_STEPCOUNT;
                steps_since_maxima = 0;
            }
        }
        else {
            count_no_maxima++;
            if (count_no_maxima > 1000) {
                stepcount = stepcount + 100;
                count_no_maxima = 0;
            }
        }

        //Change direction when no peaks found after 90°
        if (abs(start_angle - drvSys_magnetic_joint_encoder.getRotationDeg()) >= limit_angle && found_peak == false) {
            dir = -dir;
            digitalWrite(TMC_DIR, dir);
        }


        //detect falling hall values
        if ((current_average_val + average_noise_val - largest_hall_value) < 0 && found_peak) {
            n_falling++;
        };
        // do a step
        for (int i = 0; i < stepcount; i++) {
            digitalWrite(TMC_STEP, HIGH);
            delayMicroseconds(5);
            digitalWrite(TMC_STEP, LOW);
            delayMicroseconds(5);
            steps_since_maxima += stepcount;
        }

        // Finish Calibration 
        if (n_falling > 2000) {
            calibration_finished = true;

            //drvSys_magnetic_joint_encoder.setOffsetAngle(raw_joint_angle);

            int32_t motor_zero_angle = raw_motor_angle;
            drvSys_motor_encoder_offset = motor_zero_angle;
            drvSys_motor_encoder_rollovers = n_rollovers_at_zero;

            drvSys_magnetic_motor_encoder.setOffsetAngle(motor_zero_angle);
            drvSys_magnetic_joint_encoder.ProgramAbsolZeroPosition(zero_pos_joint);
            drvSys_joint_zero_set = true;


            Serial.println("DRVSYS_INFO: Joint Angle Offset Calibration finished");



            drvSys_joint_calibrated_flag = true;





        }



    }

    drvSys_save_encoder_offsets_to_Flash();
    //set back controller in FOC mode
    drvSys_foc_controller.driver->direct_mode(true);
    //increase current settung for FOC mode again
    drvSys_foc_controller.driver->rms_current(DRVSYS_PHASE_CURRENT_MAX_mA);

};

void _drvSys_align_axis() {

    Serial.println("DRVSYS_INFO: Start Alignment Movement.");
    xSemaphoreTake(drvSys_mutex_joint_position, portMAX_DELAY);
    float joint_angle_start = drvSys_joint_position;
    xSemaphoreGive(drvSys_mutex_joint_position);

    xSemaphoreTake(drvSys_mutex_motor_position, portMAX_DELAY);
    float motor_angle_start = drvSys_motor_position;
    xSemaphoreGive(drvSys_mutex_motor_position);



    // move axis with positive torque
    _drvSys_set_target_torque(0.0);

    const TickType_t move_delay = 500 / portTICK_PERIOD_MS;

    vTaskDelay(move_delay);

    _drvSys_set_target_torque(0.0);

    vTaskDelay(move_delay);

    // read resulting angles
    xSemaphoreTake(drvSys_mutex_joint_position, portMAX_DELAY);
    float delta_joint_angle = drvSys_joint_position - joint_angle_start;
    xSemaphoreGive(drvSys_mutex_joint_position);
    xSemaphoreTake(drvSys_mutex_motor_position, portMAX_DELAY);
    float delta_motor_angle = drvSys_motor_position - motor_angle_start;
    xSemaphoreGive(drvSys_mutex_motor_position);

    if (delta_joint_angle < 0.0) {
        drvSys_joint_encoder_dir_align = -1.0 * drvSys_joint_encoder_dir_align;
    };
    if (delta_motor_angle < 0.0) {
        drvSys_motor_encoder_dir_align = -1.0 * drvSys_motor_encoder_dir_align;
    }


    Serial.print("DRVSYS_INFO: Joint Encoder Delta ");
    Serial.print(delta_joint_angle);
    Serial.print(", Motor Encoder Delta ");
    Serial.println(delta_motor_angle);
    Serial.print("DRVSYS_INFO: Joint Encoder Alignment Factor ");
    Serial.print(drvSys_joint_encoder_dir_align);
    Serial.print("DRVSYS_INFO: Motor Encoder Alignment Factor ");
    Serial.println(drvSys_motor_encoder_dir_align);


    drvSys_axis_aligned_flag = true;

    Serial.println("DRVSYS_INFO: Axis aligned.");
}

void _drvSys_load_parameters_from_Flash() {



#ifdef DRV_SYS_DEBUG
    drvSys_reset_alignment_data_on_Flash();
    //drvSys_reset_encoder_offset_data_on_Flash();
#endif 

    Serial.println("DRVSYS_INFO: Load Parameters from Flash... ");

    // load offset data;
    if (drvSys_read_encoder_offsets_from_Flash()) {
        drvSys_joint_calibrated_flag = true;
    }
    else {
        drvSys_joint_calibrated_flag = false;
    }

    //load alignment data;

    if (drvSys_read_alignment_from_Flash()) {
        drvSys_axis_aligned_flag = true;
    }
    else {
        drvSys_axis_aligned_flag = false;
    }

    // load PID Parameters

    _drvSys_read_pos_PID_gains_from_flash();
    _drvSys_read_admittanceGains_from_flash();

    //drvSys_loadOffsets();


}

void _drvSys_setup_FOC_Driver() {

#ifdef DRV_SYS_DEBUG
    Serial.println("DRVSYS_INFO: Setting up FOC Drive System.");
#endif // DRV_SYS_DEBUG

    drvSys_foc_controller.setup_driver(); // intialize FOC Driver
    //enter hardcoded Electric Angle Offset
    drvSys_foc_controller.calibrate_phase_angle(FOC_EMPIRIC_PHASE_ANGLE_OFFSET);

};

void drvSys_initialize() {

    /* Initial Controller Gains */
    drvSys_PID_Gains pid_gains_pos = { .K_p = 0.0075, .K_i = 0.0, .K_d = 0.0 };
    drvSys_admittance_parameters drvSys_admittance_gains;
    drvSys_admittance_gains.virtual_spring = 0.0;
    drvSys_admittance_gains.virtual_damping = 0.0;
    drvSys_admittance_gains.virtual_inertia = 0.0;


    /* Set up Drive System Parameters */
    drvSys_parameter_config.max_current_mA = DRVSYS_PHASE_CURRENT_MAX_mA;
    drvSys_parameter_config.max_vel = DRVSYS_VEL_MAX;
    drvSys_parameter_config.max_torque_Nm = DRVSYS_TORQUE_LIMIT;
    drvSys_parameter_config.pid_gains = pid_gains_pos;
    drvSys_parameter_config.admittance_gains = drvSys_admittance_gains;
    drvSys_parameter_config.limit_high_deg = DRVSYS_POS_LIMIT_HIGH;
    drvSys_parameter_config.limit_low_deg = DRVSYS_POS_LIMIT_LOW;
    drvSys_parameter_config.endStops_enabled = DRVSYS_LIMITS_ENABLED;

    drvSys_parameter_config.pid_gains.K_vel_ff = drvSys_vel_ff_gain;


    /* Setup Hall Pin */
    pinMode(HALL_SENSOR_PIN, INPUT);


    /* Load Parameters from Flash */
    _drvSys_load_parameters_from_Flash();


    // Check hard coded settings
    if (DRVSYS_ENCODERS_CALIBRATED_FLAG) {
        drvSys_joint_calibrated_flag = true;
    }
    if (DRVSYS_AXIS_ALIGNED_FLAG) {
        drvSys_axis_aligned_flag = true;
    }

    /* Initialize Encoders */

    //Motor Encoder
    Serial.println("DRVSYS_INFO: Initialize AS5048A Motor Encoder.");
    Serial.println("--------- AS5048A Motor Encoder State ---------");
    Serial.println("");

    drvSys_magnetic_motor_encoder.init();
    drvSys_magnetic_motor_encoder.printState();

    Serial.println("");
    Serial.println("-----------------------------------------------");
    Serial.println("");

    drvSys_magnetic_motor_encoder.printErrors();
    drvSys_magnetic_motor_encoder.rollover_detection = true;

    Serial.println("");
    Serial.println("###############################################");
    Serial.println("");

    //Joint Encoder
    Serial.println("DRVSYS_INFO: Initialize AS5048A Motor Encoder.");
    Serial.println("--------- AS5048A Joint Encoder State ---------");
    Serial.println("");
    drvSys_magnetic_joint_encoder.init();
    drvSys_magnetic_joint_encoder.printState();

    Serial.println("");
    Serial.println("-----------------------------------------------");
    Serial.println("");

    drvSys_magnetic_joint_encoder.printErrors();

    Serial.println("");
    Serial.println("###############################################");
    Serial.println("");

    drvSys_magnetic_joint_encoder.rollover_detection = true;

    // Make sure mutexes are available
    xSemaphoreGive(drvSys_mutex_joint_position);
    xSemaphoreGive(drvSys_mutex_motor_position);
    xSemaphoreGive(drvSys_mutex_position_command);
    xSemaphoreGive(drvSys_mutex_joint_acc);
    xSemaphoreGive(drvSys_mutex_joint_vel);
    xSemaphoreGive(drvSys_mutex_motor_acc);
    xSemaphoreGive(drvSys_mutex_motor_vel);
    xSemaphoreGive(drvSys_mutex_joint_torque);
    xSemaphoreGive(drvSys_mutex_motor_commanded_torque);
    xSemaphoreGive(drvSys_mutex_torque_target);
    xSemaphoreGive(glob_SPI_mutex);



    // Initialize Torque Sensor

    // TO DO

    // Make Safety Checks on Sensors 


    // TO DO


    // Initialize Kinematic Kalman Filters 

    motor_kinematic_kalman_filter.init(DRVSYS_PROCESS_ENCODERS_PERIOD_US * 1e-6);
    joint_kinematic_kalman_filter.init(DRVSYS_PROCESS_ENCODERS_PERIOD_US * 1e-6);


    joint_kinematic_kalman_filter.setAccelChange(DRVSYS_KIN_KALMAN_JOINT_ACCELERATION_STD);
    motor_kinematic_kalman_filter.setAccelChange(DRVSYS_KIN_KALMAN_MOTOR_ACCELERATION_STD);

    // Initialize Notch Filters
    drvSys_notch_filters.notch_0_active = DRVSYS_NOTCH_0_ACTIVE;
    drvSys_notch_filters.notch_1_active = DRVSYS_NOTCH_1_ACTIVE;
    drvSys_notch_filters.notch_bw_0 = DRVSYS_NOTCH_0_BW; //Hz
    drvSys_notch_filters.notch_frequ_0 = DRVSYS_NOTCH_0_FREQU; //Hz
    drvSys_notch_filters.notch_bw_1 = DRVSYS_NOTCH_1_BW; //Hz
    drvSys_notch_filters.notch_frequ_1 = DRVSYS_NOTCH_1_FREQU; //Hz

    drvSys_set_notch_filters(0, drvSys_notch_filters.notch_frequ_0,
        drvSys_notch_filters.notch_bw_0, drvSys_notch_filters.notch_0_active);
    drvSys_set_notch_filters(1, drvSys_notch_filters.notch_frequ_1,
        drvSys_notch_filters.notch_bw_1, drvSys_notch_filters.notch_1_active);


    // Setup FOC Driver
    _drvSys_setup_FOC_Driver();

    /* Initialize Hall Sensor for Calibration */
    pinMode(HALL_SENSOR_PIN, INPUT);

#ifdef ALLOW_ENCODER_CALIBRATION_ROUTINE
    if (drvSys_joint_calibrated_flag == false) {

        // TODO: Set Calibration Lights

        Serial.println("DRVSYS_INFO: Encoders not calibrated. Starting Calibration Routine");
        _drvSys_calibrate_with_hallsensor();
        drvSys_save_encoder_offsets_to_Flash();
    }
#endif

    if (drvSys_axis_aligned_flag && drvSys_joint_calibrated_flag) {
        drvSys_controller_state.calibrated = true;

        drvSys_parameter_config.endStops_enabled = DRVSYS_LIMITS_ENABLED;

        Serial.println("DRVSYS_INFO: Drive is fully calibrated.");
    }
    else {
        Serial.println("DRVSYS_INFO: Drive is not fully calibrated. ");
    }

    // Setup interrupt timer to generate periodic sample trigger
    _drvSys_setup_interrupts();

    /* --- create Tasks for FOC Control --- */
    xTaskCreatePinnedToCore(
        _drvSys_foc_controller_task,   // function name
        "FOC_Controller_Task", // task name
        DRVSYS_STACKSIZE_FOC_CONTROL,      // Stack size (bytes)
        NULL,      // task parameters
        foc_prio,         // task priority
        &drvSys_foc_th,
        DRVSYS_FOC_CORE // task handle
    );

    xTaskCreatePinnedToCore(
        _drvSys_process_encoders_task,   // function name
        "Process_encoder_task", // task name
        DRVSYS_STACKSIZE_PROCESS_ENCODER_TASK,      // Stack size (bytes)
        NULL,      // task parameters
        process_sensor_prio,         // task priority
        &drvSys_process_encoders_th,
        DRVSYS_ENCODER_CORE // task handle
    );

    xTaskCreatePinnedToCore(_drvSys_torque_controller_task,
        "Torque_Controller_Task",
        DRVSYS_STACKSIZE_TORQUE_CONTROL_TASK,
        NULL,
        torque_control_prio,
        &drvSys_torque_controller_th,
        DRVSYS_TORQUE_CONTROL_CORE
    );


    /* ---- create closed loop controller tasks, are not called until closed loop control is */

/* Create all the Static Controller Tasks */

    xTaskCreatePinnedToCore(_drvSys_PID_dual_controller_task,
        "Position_Controller_Task",
        DRVSYS_STACKSIZE_PID_CONTROLLER_TASK,
        NULL,
        pid_dual_control_prio,
        &drvSys_PID_dual_controller_th,
        DRVSYS_PID_CORE
    );


    /* --- create Task --- */
    xTaskCreatePinnedToCore(
        _drvSys_admittance_controller_task,   // function name
        "Admittance_Controller_Task", // task name
        DRVSYS_STACKSIZE_ADMITTANCE_CONTROLLER_TASK,      // Stack size (bytes)
        NULL,      // task parameters
        admittance_control_prio,         // task priority
        &drvSys_admittance_controller_th,
        DRVSYS_ADMITTANCE_CORE // task handle
    );

    //suspend task until controller is started
    vTaskSuspend(drvSys_admittance_controller_th);




    //if initialization was succesful:
    drvSys_state_flag = ready;




};

void _drvSys_inverse_dynamics_nn_setup() {

    Serial.println("DRVSYS_INFO: Setup NN for Inverse Dynamics");
    drvSys_nn_inverse_dynamics = new InverseDynamicsLearner();
    drvSys_nn_inverse_dynamics->init();
    float max_joint_torque = drvSys_parameter_config.max_torque_Nm * DRVSYS_TRANSMISSION_RATIO;
    float max_vel = drvSys_parameter_config.max_vel;
    float max_motor_torque = drvSys_constants.motor_torque_constant;
    drvSys_nn_inverse_dynamics->set_scale(max_motor_torque, max_vel, DRVSYS_ACC_MAX, max_joint_torque);

    //nn_inv_initialized = true;


    /* --- create Task --- */
    /*
    xTaskCreatePinnedToCore(
        _drvSys_learn_dynamics_task,   // function name
        "Learn_Dynamics_Task", // task name
        DRVSYS_STACKSIZE_LEARN_DYNAMICS_TASK,      // Stack size (bytes)
        NULL,      // task parameters
        learn_dynamics_prio,         // task priority
        &drvSys_learn_dynamics_th,
        DRVSYS_LEARNING_CORE // task handle
    );

    Serial.println("DRVSYS_INFO: Succesful setup NN for PID Tuner");
    */


};

void _drvSys_learn_dynamics_task(void* parameters) {

    const TickType_t learning_delay = DRVSYS_LEARNING_PERIOD_MS / portTICK_PERIOD_MS;

    while (true) {

        for (int i = 0; i < drvSys_max_learning_iterations_per_step; i++) {
            if (drvSys_nn_inverse_dynamics->learning_step()) {
                continue;
            }
        }


        vTaskDelay(learning_delay);
    }
}

void drvSys_inv_dyn_nn_set_max_learning_iterations_per_step(unsigned int n_iterations) {
    drvSys_max_learning_iterations_per_step = n_iterations;
}

void drvSys_inv_dyn_nn_set_learning_rate(float lr, float lr_error_scale) {
    drvSys_nn_inverse_dynamics->nn->minimal_learning_rate = lr;
    drvSys_nn_inverse_dynamics->nn->lr_error_factor = lr_error_scale;
}
float drvSys_inv_dyn_nn_pred_error_filtered() {
    return drvSys_nn_inverse_dynamics->nn->filtered_error;

}

float drvSys_inv_dyn_read_predicted_torque() {
    return drvSys_inv_dyn_pred_torque;
}


float _drvSys_inv_dyn_nn_predict_torque() {

    drvSys_FullDriveState state = drvSys_get_full_drive_state();
    //Replace Joint Values with targets
    state.joint_acc = drvSys_acc_target;
    state.joint_pos = drvSys_pos_target;
    state.joint_vel = drvSys_vel_target;

    float predicted_torque = drvSys_nn_inverse_dynamics->predict_torque(state);

    return predicted_torque;
}

void drvSys_inv_dyn_nn_activate_control(bool active) {

    drvSys_nn_inverse_dynamics->control_active = active;
}

void _drvSys_nn_pid_tuner_setup() {

    Serial.println("DRVSYS_INFO: Setup NN for PID Tuner");
    drvSys_pid_tune_nn = new PID_Tuner_NN();
    drvSys_pid_tune_nn->init();
    drvSys_pid_tune_nn->set_scaling(drvSys_parameter_config.max_vel, drvSys_constants.motor_torque_constant);

    //nn_pid_initialized = true;

    /* --- create Task --- */
    /*
    xTaskCreatePinnedToCore(
        _drvSys_nn_pid_tuner_learn_pid_tuning_task,   // function name
        "Tune PID Task", // task name
        DRVSYS_STACKSIZE_LEARN_PID_GAINS_TASK,      // Stack size (bytes)
        NULL,      // task parameters
        learn_dynamics_prio,         // task priority
        &drvSys_tune_pid_th,
        DRVSYS_LEARNING_CORE // task handle
    );
    Serial.println("DRVSYS_INFO: Succesful setup NN for PID Tuner");
    */

}

void _drvSys_nn_pid_tuner_learn_pid_tuning_task(void* parameters) {

    const TickType_t learning_delay = DRVSYS_LEARNING_PERIOD_MS / portTICK_PERIOD_MS;

    while (true) {

        for (int i = 0; i < drvSys_max_learning_iterations_per_step; i++) {
            if (drvSys_pid_tuner_active) {


                if (drvSys_pid_tune_nn->learning_step()) {
                    continue;
                }
            }
        }


        vTaskDelay(learning_delay);
    }
}


void drvSys_nn_pid_tuner_set_learning_rate(float lr, float lr_error_scale) {
    drvSys_pid_tune_nn->nn->minimal_learning_rate = lr;
    drvSys_pid_tune_nn->nn->lr_error_factor = lr_error_scale;
}
void drvSys_nn_pid_tuner_activate(bool activate) {
    drvSys_pid_tuner_active = activate;
}


int32_t  drvSys_start_foc_processing() {

    if (drvSys_state_flag == not_ready) {
        Serial.println("DRVSYS_INFO: Can not start FOC Processing, Drive system is not ready.");
        return -1;

    }

    Serial.println("DRVSYS_INFO: Start FOC Processing");


    // Start Timer -> Starts Processing!
    Serial.println("DRVSYS_INFO: Start Interrupt Timer");
    timerAlarmEnable(drvSys_foc_timer);

    drvSys_foc_controller.set_target_torque(0.0);


    vTaskDelay(100 / portTICK_PERIOD_MS);
    /* ------ Calibration Routines that require running FOC Control ------- */
#ifdef ALLOW_ELECTRIC_ANGLE_CALIBRATION
    drvSys_calibrate_FOC();
#endif 

    drvSys_motor_torque_commanded = 0.0;
    drvSys_acc_target = 0.0;
    drvSys_vel_target = 0.0;
    drvSys_pos_target = 0.0;

    // Align axis if required

/*
#ifdef ALLOW_AXIS_ALIGN_CALIBRATION
    if (drvSys_axis_aligned_flag == false) {

        Serial.println("DRVSYS_INFO: Axis are not aligned. Starting Alignment Routine");
        _drvSys_align_axis();
        drvSys_save_alignment_to_Flash();

        if (drvSys_axis_aligned_flag && drvSys_joint_calibrated_flag) {
            drvSys_controller_state.calibrated = true;

            drvSys_software_end_stops_active = true;

            Serial.println("DRVSYS_INFO: Drive is fully calibrated.");
        }
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
#endif
*/


/* ---------------------------------------------------------------------------- */


//flag start foc controller
    Serial.println("DRVSYS_INFO: FOC Direct Torque Controller is now active.");
    drvSys_controller_state.state_flag = foc_direct_torque;

    return 1;
};

int32_t drvSys_start_motion_control(drvSys_controlMode control_mode) {

    if (drvSys_controller_state.state_flag == ready
        || drvSys_controller_state.state_flag == not_ready || drvSys_controller_state.state_flag == error) {

        Serial.println("DRVSYS_ERROR: Drive not ready to start Closed Loop Motion Control.");

        return -1;

    }

    if (drvSys_controller_state.state_flag == closed_loop_control_inactive || drvSys_controller_state.state_flag == foc_direct_torque) {

        Serial.println("DRVSYS_INFO: Start Closed Loop Motion Control.");
        drvSys_mode = control_mode;
        drvSys_controller_state.control_mode = drvSys_mode;

        switch (drvSys_mode) {
        case dual_control:
            _drvSys_setup_dual_controller();
            break;

        case direct_torque:
            _drvSys_setup_direct_controller();
            break;

        case admittance_control:
            _drvSys_setup_admittance_controller();
            break;
        default:
            _drvSys_setup_dual_controller();
            break;
        }


        drvSys_controller_state.state_flag = closed_loop_control_active;
    }
    else {
        Serial.println("DRVSYS_ERROR: Can only change motion control mode when control is already stopped");
        return -1;
    }

    return 1;

};

void _drvSys_setup_dual_controller() {


    /* Position Controller */
    drvSys_position_controller.setSampleTime(DRVSYS_CONTROL_POS_PERIOD_US);
    //Velocity Limitation
    float torque_limit = drvSys_parameter_config.max_torque_Nm;
    drvSys_position_controller.setOutputLimits(torque_limit * (-1.0), torque_limit);
    drvSys_position_controller.setpoint = 0.0;

    drvSys_position_controller.Initialize();
    drvSys_position_controller.setMode(PID_MODE_INACTIVE);
    drvSys_position_controller.SetControllerDirection(PID_DIR_DIRECT);
    drvSys_position_controller.setDifferentialFilter(DRVSYS_POS_PID_FILTER_DERIVATIVE, DRVSYS_POS_PID_FILTER_DERIVATIVE_ALPHA);
    drvSys_position_controller.setErrorDeadBand(DRVSYS_POS_PID_DEADBAND);
    drvSys_position_controller.derivative_on_measurement = DRVSYS_POS_PID_DERIVATIVE_ON_MEASUREMENT;
    drvSys_position_controller.setTuning(PID_GAIN_P_STANDARD, PID_GAIN_I_STANDARD, PID_GAIN_D_STANDARD);


    _drvSys_read_pos_PID_gains_from_flash();

    Serial.println("DRVSYS_INFO: Setup PID Position Controller with Velocity Feedforward");

    drvSys_position_controller.setMode(PID_MODE_ACTIVE);
    drvSys_torque_ff = 0.0;

    /* Start Learning Systems */

    _drvSys_inverse_dynamics_nn_setup();
    //_drvSys_nn_pid_tuner_setup();

    Serial.println("DRVSYS_INFO: Initialized Neural Networks to learn System Dynamics");

};

void drvSys_set_advanced_PID_settings(int type, float filter_alpha, float deadzone, bool active) {

    if (type == 0) {
        if (filter_alpha <= 0) {
            drvSys_position_controller.setInputFilter(false);
        }
        if (filter_alpha < 1.0) {
            drvSys_position_controller.setInputFilter(true, filter_alpha);
        }

        if (deadzone <= 0) {
            drvSys_position_controller.setErrorDeadBand(0.0);
        }
        else {
            drvSys_position_controller.setErrorDeadBand(deadzone);
        }

        if (active) {
            drvSys_position_controller.setMode(PID_MODE_ACTIVE);
        }
        else {
            drvSys_position_controller.setMode(PID_MODE_ACTIVE);
        }

    }
}


void drvSys_stop_controllers() {

    drvSys_position_controller.setMode(PID_MODE_INACTIVE);

    //deactivate inverse dynamics controller
    //...

    drvSys_controller_state.state_flag = closed_loop_control_inactive;

    vTaskSuspend(drvSys_admittance_controller_th);

    _drvSys_set_target_torque(0.0);

    Serial.println("DRVSYS_INFO: Stopped Closed Loop Controllers");

};

void drvSys_set_kalman_filter_acc_noise(float acc_noise, bool joint) {


    if (joint) {
        joint_kinematic_kalman_filter.setAccelChange(acc_noise);
    }
    else {// motor
        motor_kinematic_kalman_filter.setAccelChange(acc_noise);
    }
}

void drvSys_set_ff_gains(float gain, bool vel) {

    if (vel) {
        drvSys_parameter_config.pid_gains.K_vel_ff = gain;
        drvSys_vel_ff_gain = gain;
    }
    else {
        drvSys_parameter_config.pid_gains.K_joint_P_gain = gain;
    }
}

void drvSys_set_notch_filters(int filter_id, float notch_frequ, float bandwidth_f, bool activate) {

    // Calculate coefficients

    //bandwidth
    float sample_frequ = DRVSYS_CONTROL_TORQUE_FREQU;
    float omega_bw = 2 * PI * bandwidth_f / sample_frequ;
    float a_bw = exp(omega_bw);

    // notch frequency
    float omega_c = 2 * PI * notch_frequ / sample_frequ;

    // Coefficients
    float a_0 = 1;
    float a_1 = 2 * a_bw * cos(omega_c);
    float a_2 = -a_bw * a_bw;

    float coefs_a[3] = { a_0,a_1,a_2 };

    float b_0 = 1;
    float b_1 = -2 * cos(omega_c);
    float b_2 = 2;

    float coefs_b[3] = { b_0, b_1, b_2 };


    if (filter_id == 0) {
        notch_filter_0.setCoefficients(coefs_a, coefs_b);

        if (activate) {
            drvSys_notch_filters.notch_0_active = true;
        }

    }
    if (filter_id == 1) {

        notch_filter_1.setCoefficients(coefs_a, coefs_b);

        if (activate) {
            drvSys_notch_filters.notch_1_active = true;
        }

    }

    xSemaphoreTake(glob_Serial_mutex, portMAX_DELAY);
    Serial.print("DRVSYS_INFO: Set up Notch Filter ");
    Serial.print(filter_id);
    Serial.print(" with f_c = ");
    Serial.print(notch_frequ);
    Serial.print(", bw = ");
    Serial.println(bandwidth_f);
    xSemaphoreGive(glob_Serial_mutex);


};

float _drvSys_check_joint_limit(float input) {

    if (drvSys_joint_calibrated_flag && drvSys_axis_aligned_flag && drvSys_parameter_config.endStops_enabled) {
        xSemaphoreTake(drvSys_mutex_joint_position, portMAX_DELAY);
        float pos_deg = drvSys_joint_position;
        xSemaphoreGive(drvSys_mutex_joint_position);

        if (pos_deg >= drvSys_parameter_config.limit_high_deg) {
            if (input > 0.0) {
                drvSys_controller_state.hit_positive_limit = true;
                drvSys_controller_state.hit_neg_limit = false;
                return 0.0;
            }
        }
        if (pos_deg <= drvSys_parameter_config.limit_low_deg) {
            if (input < 0.0) {
                drvSys_controller_state.hit_positive_limit = false;
                drvSys_controller_state.hit_neg_limit = true;
                return 0.0;
            }
        }
        drvSys_controller_state.hit_positive_limit = false;
        drvSys_controller_state.hit_neg_limit = false;
    }

    return input;

}



void _drvSys_set_target_torque(float torque) {

    float torque_limit = drvSys_parameter_config.max_torque_Nm;

    xSemaphoreTake(drvSys_mutex_torque_target, portMAX_DELAY);

    if (_drvSys_torque_target > torque_limit) {
        _drvSys_torque_target = torque_limit;
    }
    else if (_drvSys_torque_target < (-1.0) * torque_limit) {
        _drvSys_torque_target = (-1.0) * torque_limit;
    }
    _drvSys_torque_target = torque;
    xSemaphoreGive(drvSys_mutex_torque_target);

};

void drvSys_set_feed_forward_torque(float torque_ff) {
    drvSys_torque_ff = torque_ff;

};

void _drvSys_set_target_velocity(float vel) {


    if (vel > drvSys_parameter_config.max_vel) {
        vel = drvSys_parameter_config.max_vel;
    }
    else if (vel < (-1.0) * drvSys_parameter_config.max_vel) {
        vel = (-1.0) * drvSys_parameter_config.max_vel;
    }
    drvSys_vel_target = vel;

};


void _drvSys_set_target_pos(float pos) {

    if (pos > drvSys_parameter_config.limit_high_deg) {
        pos = drvSys_parameter_config.limit_high_deg;
    }
    else if (pos < drvSys_parameter_config.limit_low_deg) {
        pos = drvSys_parameter_config.limit_low_deg;
    }
    drvSys_pos_target = pos;

};

const drvSys_driveTargets drvSys_get_targets() {
    drvSys_driveTargets targets;

    targets.motor_torque_ff = drvSys_foc_controller.target_torque;
    targets.pos_target = drvSys_pos_target;
    targets.vel_target = drvSys_vel_target;
    targets.ref_torque = drvSys_joint_torque_ref;
    targets.acc_target = drvSys_acc_target;

    return targets;
};

void _drvSys_setup_interrupts() {

    Serial.println("DRVSYS_INFO: Setup Control Interrupts.");
    drvSys_foc_timer = timerBegin(0, drvSys_timer_prescaler_divider, true);
    timerAttachInterrupt(drvSys_foc_timer, &_drvSys_on_foc_timer, true);
    timerAlarmWrite(drvSys_foc_timer, drvSys_timer_alarm_rate_us, true);

}

void _drvSys_set_empiric_phase_shift(float phase_shift_factor) {
    drvSys_foc_controller.set_empiric_phase_shift_factor(phase_shift_factor);
};



void IRAM_ATTR _drvSys_on_foc_timer() {
    volatile static uint64_t tickCount = 0;

    tickCount++;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (tickCount % drvSys_timer_foc_ticks == 0) { // react every 200us ->5kHz
        vTaskNotifyGiveFromISR(drvSys_foc_th, &xHigherPriorityTaskWoken);
    }
    if (tickCount % drvSys_timer_encoder_process_ticks == 0) { //react every 250us
        vTaskNotifyGiveFromISR(drvSys_process_encoders_th, &xHigherPriorityTaskWoken);
    }
    if (tickCount % drvSys_timer_torque_control_ticks == 0) {
        vTaskNotifyGiveFromISR(drvSys_torque_controller_th, &xHigherPriorityTaskWoken);
    }

    if (drvSys_controller_state.state_flag == closed_loop_control_active) {

        /*
        if (tickCount % drvSys_timer_vel_control_ticks == 0) {
            vTaskNotifyGiveFromISR(drvSys_vel_controller_th, &xHigherPriorityTaskWoken);
        }
        */
        if (tickCount % drvSys_timer_pos_control_ticks == 0) {
            vTaskNotifyGiveFromISR(drvSys_PID_dual_controller_th, &xHigherPriorityTaskWoken);
        }
    }

    portYIELD_FROM_ISR();
}

void _drvSys_foc_controller_task(void* parameters) {

    uint32_t foc_thread_notification;
    TIMERG0.wdt_wprotect = TIMG_WDT_WKEY_VALUE;
    TIMERG0.wdt_feed = 1;
    TIMERG0.wdt_wprotect = 0;

    while (true) {

        foc_thread_notification = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (foc_thread_notification) {
            drvSys_foc_controller.foc_control();
        }

    }
}

void _drvSys_process_encoders_task(void* parameters) {

    uint32_t encoder_processing_thread_notification;
    while (true) {
        encoder_processing_thread_notification = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);


        if (encoder_processing_thread_notification) {

            /* use Kalman Filter */

            //static const float encoder2Deg = 180.0 / 8192.0;
            static const float encoder2Rad = PI / 8192.0;

            static const float inverse_transmission = 1.0 / drvSys_constants.transmission_ratio;


            float motor_pos_sensor_val = drvSys_motor_encoder_dir_align * drvSys_magnetic_motor_encoder.last_sample * encoder2Rad * inverse_transmission;

            motor_pos_sensor_val = motor_pos_sensor_val;

            KinematicStateVector motor_state = motor_kinematic_kalman_filter.estimateStates(motor_pos_sensor_val);

            xSemaphoreTake(drvSys_mutex_motor_position, portMAX_DELAY);
            drvSys_motor_position = motor_state.pos;
            xSemaphoreGive(drvSys_mutex_motor_position);

            xSemaphoreTake(drvSys_mutex_motor_vel, portMAX_DELAY);
            drvSys_motor_velocity = motor_state.vel;
            xSemaphoreGive(drvSys_mutex_motor_vel);

            xSemaphoreTake(drvSys_mutex_motor_acc, portMAX_DELAY);
            drvSys_motor_acc = motor_state.acc;
            xSemaphoreGive(drvSys_mutex_motor_acc);


            xSemaphoreTake(glob_SPI_mutex, portMAX_DELAY);
            float raw_joint_angle_rad = drvSys_joint_encoder_dir_align * drvSys_magnetic_joint_encoder.getRotationCentered(true) * encoder2Rad;
            xSemaphoreGive(glob_SPI_mutex);


            if (raw_joint_angle_rad)
                raw_joint_angle_rad = raw_joint_angle_rad - drvSys_angle_offset_joint - drvSys_joint_encoder_offset;

            KinematicStateVector joint_state = joint_kinematic_kalman_filter.estimateStates(raw_joint_angle_rad);

            xSemaphoreTake(drvSys_mutex_joint_acc, portMAX_DELAY);
            drvSys_joint_acc = joint_state.acc;
            xSemaphoreGive(drvSys_mutex_joint_acc);

            xSemaphoreTake(drvSys_mutex_joint_vel, portMAX_DELAY);
            drvSys_joint_velocity = joint_state.vel;
            xSemaphoreGive(drvSys_mutex_joint_vel);

            xSemaphoreTake(drvSys_mutex_joint_position, portMAX_DELAY);
            drvSys_joint_position = joint_state.pos;
            xSemaphoreGive(drvSys_mutex_joint_position);


            // collect Samples for Inverse Dynamics Learner

            if (nn_inv_initialized) {

                drvSys_nn_inverse_dynamics->add_data(drvSys_get_full_drive_state());
            }

            // collect second part of PID Tuning Learner
            if (drvSys_pid_tuner_active && nn_pid_initialized) {
                //drvSys_pid_tune_nn->collectSampleTimeStep(drvSys_get_full_drive_state());
            }


        }
    }

};

void _drvSys_process_torque_sensor_task(void* parameters) {

    const TickType_t torque_sensor_delay = DRVSYS_PROCESS_TORQUE_SENSOR_PERIOD_MS / portTICK_PERIOD_MS;
    while (true) {
        //...
        // TO DO!!!!!

        vTaskDelay(torque_sensor_delay);
    }
};

void _drvSys_PID_dual_controller_task(void* parameters) {
    uint32_t position_controller_processing_thread_notification;

    static float inv_transmission = 1.0 / double(drvSys_constants.transmission_ratio);

    while (true) {

        position_controller_processing_thread_notification = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if (position_controller_processing_thread_notification) {

            if (drvSys_mode == dual_control || drvSys_mode == admittance_control) {

                // Obtain Optimal Gains from NN
                if (drvSys_pid_tuner_active) {
                    //drvSys_PID_Gains gains = drvSys_pid_tune_nn->predictOptimalGains(drvSys_get_full_drive_state(), drvSys_get_targets());
                    //drvSys_update_PID_gains(gains);
                }
                //

                xSemaphoreTake(drvSys_mutex_motor_position, portMAX_DELAY);
                float actual_pos_motor = drvSys_motor_position;
                xSemaphoreGive(drvSys_mutex_motor_position);

                xSemaphoreTake(drvSys_mutex_joint_position, portMAX_DELAY);
                float actual_pos_joint = drvSys_joint_position;
                xSemaphoreGive(drvSys_mutex_joint_position);

                xSemaphoreTake(drvSys_mutex_position_command, portMAX_DELAY);
                float pos_target_joint = drvSys_pos_target;
                xSemaphoreGive(drvSys_mutex_position_command);

                // Collect first half of PID tuning sample
                if (drvSys_pid_tuner_active && nn_pid_initialized) {
                    drvSys_pid_tune_nn->collectSamplePIDState(drvSys_get_targets(), drvSys_get_full_drive_state(),
                        drvSys_position_controller.error, drvSys_position_controller.dError, drvSys_position_controller.iTerm);
                }

                float sign = 1.0;
                if (pos_target_joint - actual_pos_motor < 0) {
                    sign = -1.0;
                }

                drvSys_position_controller.setSetPoint(pos_target_joint, false);

                drvSys_position_controller.input = actual_pos_joint;
                drvSys_position_controller.compute();


                /* Handle controller output */

                float motor_torque_target = drvSys_position_controller.output + drvSys_vel_target * drvSys_vel_ff_gain;

                _drvSys_set_target_torque(motor_torque_target);


            }
        }
    }
}


void _drvSys_torque_controller_task(void* parameters) {
    TIMERG0.wdt_wprotect = TIMG_WDT_WKEY_VALUE;
    TIMERG0.wdt_feed = 1;
    TIMERG0.wdt_wprotect = 0;
    uint32_t torque_controller_processing_thread_notification;

    while (true) {
        torque_controller_processing_thread_notification = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if (torque_controller_processing_thread_notification) {

            if (nn_inv_initialized) {
                drvSys_inv_dyn_pred_torque = _drvSys_inv_dyn_nn_predict_torque();
            }
            xSemaphoreTake(drvSys_mutex_motor_commanded_torque, portMAX_DELAY);
            drvSys_motor_torque_commanded = _drvSys_torque_target + drvSys_torque_ff + drvSys_inv_dyn_pred_torque;

            // Apply Notch Filters to avoid resonance frequencies

            /*
            if (drvSys_notch_filters.notch_0_active) {
                notch_filter_0.setInput(drvSys_m_torque_commanded);
                notch_filter_0.compute();
                drvSys_m_torque_commanded = notch_filter_0.output;
            }
            if (drvSys_notch_filters.notch_1_active) {
                notch_filter_1.setInput(drvSys_m_torque_commanded);
                notch_filter_1.compute();
                drvSys_m_torque_commanded = notch_filter_1.output;
            }
            */

            drvSys_foc_controller.set_target_torque(_drvSys_check_joint_limit(drvSys_motor_torque_commanded));
            xSemaphoreGive(drvSys_mutex_motor_commanded_torque);

        }
    };
};


void _drvSys_setup_direct_controller() {


    Serial.println("DRVSYS_INFO: Setup Direct Torque Controller");
    drvSys_torque_ff = 0.0;


}

void _drvSys_setup_admittance_controller() {

    // Setup internal position and velocity loop
    _drvSys_setup_dual_controller();

    _drvSys_read_admittanceGains_from_flash();
    Serial.println("DRVSYS_INFO: Setup Admittance Controller");


    vTaskResume(drvSys_admittance_controller_th);

};

void _drvSys_admittance_controller_task(void* parameters) {

    static float prev_admittance_target_pos = 0;
    static const float delta_t_admittance_control = 0.01;

    const TickType_t admittance_delay = DRVSYS_CONTROL_ADMITTANCE_PERIOD_MS / portTICK_PERIOD_MS;

    while (true) {

        float virtual_spring = drvSys_parameter_config.admittance_gains.virtual_spring;
        float virtual_damping = drvSys_parameter_config.admittance_gains.virtual_damping;

        xSemaphoreTake(drvSys_mutex_joint_torque, portMAX_DELAY);
        float current_joint_torque = drvSys_joint_torque;
        xSemaphoreGive(drvSys_mutex_joint_torque);

        float desired_joint_torque = drvSys_joint_torque_ref;

        float desired_velocity = drvSys_vel_target;
        float desired_position = drvSys_pos_target;

        xSemaphoreTake(drvSys_mutex_joint_position, portMAX_DELAY);
        float current_position = drvSys_joint_position;
        xSemaphoreGive(drvSys_mutex_joint_position);


        // use relationship torque = K*error + d*error_d + torque_ref to obtain velocity target in time domain
        float vel_admittance_target = (desired_joint_torque - current_joint_torque
            + virtual_spring * (desired_position - current_position)) / virtual_damping
            + desired_velocity;

        //use first order euler integration to calculate position target
        float pos_admittance_target = prev_admittance_target_pos + delta_t_admittance_control * vel_admittance_target;


        /* Write position and velocity target to inner control loop */

        _drvSys_set_target_velocity(vel_admittance_target);
        _drvSys_set_target_pos(pos_admittance_target);

        vTaskDelay(admittance_delay);
    }

};


void _drvSys_monitor_system_task(void* parameters) {



    // handle Temperature Sensor if Available

    int error_iterations = 0;


    const TickType_t monitoring_delay = DRVSYS_CONTROL_ADMITTANCE_PERIOD_MS / portTICK_PERIOD_MS;

    while (true) {
        drvSys_hall_sensor_val = analogRead(HALL_SENSOR_PIN);


#ifdef DRVSYS_MOTOR_TEMP_SENSOR_AVAILABLE
        //TODO
#endif

        float sensor_difference = abs(drvSys_motor_position / drvSys_constants.transmission_ratio - drvSys_joint_position);

        if (sensor_difference * RAD2DEG > DRVSYS_SENSOR_MAX_DIFFERENCE_DEG) {
            // hit error flag
            error_iterations++;
            //Handle error
            drvSys_controller_state.state_flag = error;

            if (error_iterations > 100) {
                ESP.restart();
            }



        }

        if (sensor_difference * RAD2DEG < DRVSYS_SENSOR_MAX_DIFFERENCE_DEG && drvSys_controller_state.state_flag == error) {

            drvSys_controller_state.state_flag = foc_direct_torque;
            //Recover from error

            error_iterations = 0;
        }



        vTaskDelay(monitoring_delay);
    }
}

void drvSys_update_PID_gains(drvSys_PID_Gains gains) {

    // Write them to overall Drive System parameters
    drvSys_parameter_config.pid_gains.K_p = gains.K_p;
    drvSys_parameter_config.pid_gains.K_i = gains.K_i;
    drvSys_parameter_config.pid_gains.K_d = gains.K_d;
    drvSys_parameter_config.pid_gains.K_vel_ff = gains.K_vel_ff;
    drvSys_parameter_config.pid_gains.K_joint_P_gain = gains.K_joint_P_gain;

    drvSys_vel_ff_gain = gains.K_vel_ff;

    // Write them to controller instance
    drvSys_position_controller.setTuning(gains.K_p, gains.K_i, gains.K_d);

}

bool drvSys_read_encoder_offsets_from_Flash() {
    drv_sys_preferences.begin(drvSys_encoder_offset, false);

    bool data_available = drv_sys_preferences.getBool("off_avail", false);

    if (data_available) {
        drvSys_motor_encoder_offset = drv_sys_preferences.getInt("motor", 0);
        drvSys_motor_encoder_rollovers = drv_sys_preferences.getInt("m_roll", 0);
        drvSys_magnetic_motor_encoder._n_rollovers = drvSys_motor_encoder_rollovers;
        drvSys_magnetic_motor_encoder.setOffsetAngle(drvSys_motor_encoder_offset);
        drvSys_joint_zero_set = drv_sys_preferences.getBool("j_set", false);

        Serial.println(drvSys_motor_encoder_offset);

        Serial.println("DRVSYS_INFO: Read Encoder Offsets from Flash.");
        drvSys_joint_calibrated_flag = true;
    }
    else {
        Serial.println("DRVSYS_INFO: No Encoder Offset Data available.");
    }
    drv_sys_preferences.end();

    return data_available;

};

void drvSys_save_encoder_offsets_to_Flash() {

    drv_sys_preferences.begin(drvSys_encoder_offset, false);
    drv_sys_preferences.putBool("off_avail", true);
    drv_sys_preferences.putInt("motor", drvSys_motor_encoder_offset);
    drv_sys_preferences.putInt("m_roll", drvSys_motor_encoder_rollovers);
    drv_sys_preferences.putBool("j_set", drvSys_joint_zero_set);
    drv_sys_preferences.end();

    Serial.println("DRVSYS_INFO: Saved Encoder Offsets to Flash.");

};

void drvSys_reset_encoder_offset_data_on_Flash() {

    drv_sys_preferences.begin(drvSys_encoder_offset, false);
    drv_sys_preferences.putBool("off_avail", false);
    drv_sys_preferences.putInt("joint", 0);
    drv_sys_preferences.putInt("motor", 0);
    drv_sys_preferences.putInt("m_roll", 0);
    drv_sys_preferences.end();

    Serial.println("DRVSYS_INFO: Removed Encoder Offsets to Flash.");


}

bool drvSys_read_alignment_from_Flash() {

    drv_sys_preferences.begin(drvSys_alignment_setting, false);

    bool saved_data_available = drv_sys_preferences.getBool("ax_avail", false);

    if (saved_data_available) {
        drvSys_flip_global_alignment = drv_sys_preferences.getBool("glob_flip", false);
        drvSys_flip_global_alignment = drv_sys_preferences.getFloat("tMotDir", DRVSYS_TORQUE_ALIGN_DIR);
        drvSys_joint_encoder_dir_align = drv_sys_preferences.getFloat("jEncDir", DRVSYS_JOINT_ENC_ALIGN_DIR);
        drvSys_motor_encoder_dir_align = drv_sys_preferences.getFloat("mEncDir", DRVSYS_MOTOR_ENC_ALIGN_DIR);
        drvSys_torque_dir_align = drv_sys_preferences.getFloat("SensDir", DRVSYS_TORQUE_ALIGN_DIR);

        drvSys_axis_aligned_flag = true;

        Serial.println("DRVSYS_INFO: Read Axis Alignment from Flash");
    }
    else {
        Serial.println("DRVSYS_INFO: No Axis Alignment Data available.");
    }

    drv_sys_preferences.end();

    return saved_data_available;


}

void drvSys_save_alignment_to_Flash() {

    drv_sys_preferences.begin(drvSys_alignment_setting, false);

    drv_sys_preferences.putBool("ax_avail", true);
    drv_sys_preferences.putBool("glob_flip", drvSys_flip_global_alignment);
    drv_sys_preferences.putFloat("tMotDir", drvSys_torque_dir_align);
    drv_sys_preferences.putFloat("jEncDir", drvSys_joint_encoder_dir_align);
    drv_sys_preferences.putFloat("mEncDir", drvSys_motor_encoder_dir_align);
    drv_sys_preferences.putFloat("tSensDir", drvSys_torque_dir_align);

    drv_sys_preferences.end();

    Serial.println("DRVSYS_INFO: Saved axis alignment data in Flash");
}

void drvSys_reset_alignment_data_on_Flash() {
    drv_sys_preferences.begin(drvSys_alignment_setting, false);

    drv_sys_preferences.putBool("ax_avail", false);
    drv_sys_preferences.putBool("glob_flip", false);
    drv_sys_preferences.putFloat("tMotDir", 1.0);
    drv_sys_preferences.putFloat("jEncDir", 1.0);
    drv_sys_preferences.putFloat("mEncDir", 1.0);
    drv_sys_preferences.putFloat("tSensDir", 1.0);

    drv_sys_preferences.end();

    Serial.println("DRVSYS_INFO: Removed axis alignment data in Flash");

}




void drvSys_set_pos_PID_gains(const float Kp, const float Ki, const  float Kd, bool save) {
    // Set Position PID settings
    // Write them to overall Drive System parameters
    drvSys_parameter_config.pid_gains.K_p = Kp;
    drvSys_parameter_config.pid_gains.K_i = Ki;
    drvSys_parameter_config.pid_gains.K_d = Kd;

    // Write them to controller instance
    drvSys_position_controller.setTuning(Kp, Ki, Kd);

    if (save) { //save them to Flash;
        drvSys_save_pos_PID_gains();
    }
};

void drvSys_save_pos_PID_gains() {

    float* gains = drvSys_position_controller.getGains();

    drv_sys_preferences.begin(drvSys_posPID_saved_gains, false);

    drv_sys_preferences.putBool("PIDAvail", true);
    drv_sys_preferences.putFloat("P", gains[0]);
    drv_sys_preferences.putFloat("I", gains[1]);
    drv_sys_preferences.putFloat("D", gains[2]);
    drv_sys_preferences.putFloat("vel_ff", drvSys_vel_ff_gain);

    drv_sys_preferences.end();
}


void drvSys_set_admittance_params(float virtual_spring, float virtual_damping, float virtual_inertia, bool save) {

    drvSys_parameter_config.admittance_gains.virtual_spring = virtual_spring;
    drvSys_parameter_config.admittance_gains.virtual_damping = virtual_damping;
    drvSys_parameter_config.admittance_gains.virtual_inertia = virtual_inertia;

    if (save) {
        drvSys_save_admittance_params();
    }
};
void drvSys_save_admittance_params() {


    float virtual_spring = drvSys_parameter_config.admittance_gains.virtual_spring;
    float virtual_damping = drvSys_parameter_config.admittance_gains.virtual_damping;
    float virtual_inertia = drvSys_parameter_config.admittance_gains.virtual_inertia;
    drv_sys_preferences.begin(drvSys_admittance_saved_gains, false);

    drv_sys_preferences.putBool("admGainAvail", true);
    drv_sys_preferences.putFloat("spring", virtual_spring);
    drv_sys_preferences.putFloat("damper", virtual_damping);
    drv_sys_preferences.putFloat("inertia", virtual_inertia);

    drv_sys_preferences.end();
};

void drvSys_setOffsets(float motor_offset_deg, float joint_offset_deg, bool save, bool reset) {


    if (motor_offset_deg == 0) {
        drvSys_angle_offset_joint = joint_offset_deg * DEG2RAD;
    }
    else if (joint_offset_deg == 0) {
        drvSys_angle_offset_motor = motor_offset_deg * DEG2RAD;
    }
    else {
        drvSys_angle_offset_joint = joint_offset_deg * DEG2RAD;
        drvSys_angle_offset_motor = motor_offset_deg * DEG2RAD;
    }


    if (save) {
        drv_sys_preferences.begin(drvSys_saved_offsets, false);
        drv_sys_preferences.putFloat("motor", drvSys_angle_offset_motor);
        drv_sys_preferences.putFloat("joint", drvSys_angle_offset_joint);
        drv_sys_preferences.end();

        Serial.println("DRVSYS_INFO: Saved Offset Angles");
    }
    if (reset) {
        drv_sys_preferences.begin(drvSys_saved_offsets, false);
        drv_sys_preferences.putFloat("motor", 0);
        drv_sys_preferences.putFloat("joint", 0);
        drv_sys_preferences.end();

        Serial.println("DRVSYS_INFO: Reset Offset Angles");
    }
}

void drvSys_loadOffsets() {
    drv_sys_preferences.begin(drvSys_saved_offsets, false);
    drvSys_angle_offset_motor = drv_sys_preferences.getFloat("motor", drvSys_angle_offset_joint);
    drvSys_angle_offset_joint = drv_sys_preferences.getFloat("joint", drvSys_angle_offset_motor);
    drv_sys_preferences.end();

    Serial.print("DRVSYS_INFO: Loading Motor Offset Angle: ");
    Serial.println(drvSys_angle_offset_motor);
    Serial.print("DRVSYS_INFO: Loading Joint Offset Angle: ");
    Serial.println(drvSys_angle_offset_joint);

}

bool _drvSys_read_pos_PID_gains_from_flash() {
    /* Rea dand set position PID Settings */
    drv_sys_preferences.begin(drvSys_posPID_saved_gains, false);

    bool pos_pid_gains_available = drv_sys_preferences.getBool("PIDAvail", false);

    if (pos_pid_gains_available) {

        float K_pos_P = drv_sys_preferences.getFloat("P", drvSys_parameter_config.pid_gains.K_p);
        float K_pos_I = drv_sys_preferences.getFloat("I", drvSys_parameter_config.pid_gains.K_i);
        float K_pos_D = drv_sys_preferences.getFloat("D", drvSys_parameter_config.pid_gains.K_d);
        float vel_ff_gains = drv_sys_preferences.getFloat("vel_ff", drvSys_vel_ff_gain);


        // Write them to overall Drive System parameters
        drvSys_parameter_config.pid_gains.K_p = K_pos_P;
        drvSys_parameter_config.pid_gains.K_i = K_pos_I;
        drvSys_parameter_config.pid_gains.K_d = K_pos_D;

        drvSys_vel_ff_gain = vel_ff_gains;

        // Write them to controller instance
        drvSys_position_controller.setTuning(K_pos_P, K_pos_I, K_pos_D);

        Serial.println("DRVSYS_INFO: Read Position PID Gains from Flash.");
        Serial.println("DRVSYS_INFO: P = " + String(K_pos_P) + ", I = " + String(K_pos_I)
            + ", D = " + String(K_pos_D) + "Vel ff Gain: " + String(vel_ff_gains));
    }
    else {
        Serial.println("DRVSYS_INFO: No Position PID Gains available on Flash.");
    }
    drv_sys_preferences.end();

    return pos_pid_gains_available;


}
bool _drvSys_read_admittanceGains_from_flash() {

    drv_sys_preferences.begin(drvSys_admittance_saved_gains, false);

    bool adm_pid_gains_available = drv_sys_preferences.getBool("admGainAvail", false);

    if (adm_pid_gains_available) {

        float virtual_spring = drv_sys_preferences.getFloat("P", drvSys_parameter_config.admittance_gains.virtual_spring);
        float virtual_damping = drv_sys_preferences.getFloat("I", drvSys_parameter_config.admittance_gains.virtual_damping);
        float virtual_inertia = drv_sys_preferences.getFloat("D", drvSys_parameter_config.admittance_gains.virtual_inertia);
        drv_sys_preferences.end();

        drvSys_parameter_config.admittance_gains.virtual_spring = virtual_spring;
        drvSys_parameter_config.admittance_gains.virtual_damping = virtual_damping;
        drvSys_parameter_config.admittance_gains.virtual_inertia = virtual_inertia;

        Serial.println("DRVSYS_INFO: Admittance Parameters read from Flash");

    }
    else {
        Serial.println("DRVSYS_INFO: No Admittance Parameters available on Flash.");
    }
    drv_sys_preferences.end();


    return adm_pid_gains_available;
};


void drvSys_calibrate_FOC() {

    xSemaphoreTake(glob_Serial_mutex, portMAX_DELAY);
    Serial.println("DRVSYS_INFO: Start FOC Calibration");

    Serial.println("----------------------------------");
    Serial.println("----------------------------------");

    //drvSys_foc_controller.calibrate_phase_angle(0);

    Serial.print("DRVSYS_FOC_CAL: Start with Phase Angle Value: ");
    xSemaphoreGive(glob_Serial_mutex);
    Serial.println(drvSys_foc_controller.phase_null_angle);
    xSemaphoreTake(glob_Serial_mutex, portMAX_DELAY);

    Serial.println("DRVSYS_FOC_CAL: Sweep Angle Area");
    xSemaphoreGive(glob_Serial_mutex);

    bool calibration_finished = false;

    int iteration = 0;

    float score = 0;

    float highest_score = 0;
    float best_angle = 0;

    int sweep_range = 150;
    int start_val = drvSys_foc_controller.phase_null_angle - sweep_range;

    drvSys_foc_controller.phase_null_angle = start_val;


    while (!calibration_finished) {
        drvSys_foc_controller.set_target_torque(DRVSYS_TORQUE_CONSTANT * 0.8);
        vTaskDelay(150 / portTICK_PERIOD_MS);

        int N_samples = 300;
        float forward_vel = 0.0;
        for (int i = 0; i < N_samples; i++) {
            xSemaphoreTake(drvSys_mutex_motor_vel, portMAX_DELAY);
            forward_vel += drvSys_motor_velocity;
            xSemaphoreGive(drvSys_mutex_motor_vel);

            vTaskDelay(1 / portTICK_PERIOD_MS);
        }
        drvSys_foc_controller.set_target_torque(0.0);
        vTaskDelay(500 / portTICK_PERIOD_MS);
        forward_vel = abs(forward_vel / float(N_samples));


        xSemaphoreTake(glob_Serial_mutex, portMAX_DELAY);
        Serial.print("DRVSYS_FOC_CAL: Forward velocity: ");
        Serial.println(forward_vel);
        xSemaphoreGive(glob_Serial_mutex);

        drvSys_foc_controller.set_target_torque(-DRVSYS_TORQUE_CONSTANT * 0.8);

        vTaskDelay(150 / portTICK_PERIOD_MS);

        float backward_vel = 0.0;
        for (int i = 0; i < N_samples; i++) {
            xSemaphoreTake(drvSys_mutex_motor_vel, portMAX_DELAY);
            backward_vel += drvSys_motor_velocity;
            xSemaphoreGive(drvSys_mutex_motor_vel);

            vTaskDelay(1 / portTICK_PERIOD_MS);
        }
        drvSys_foc_controller.set_target_torque(0.0);
        vTaskDelay(500 / portTICK_PERIOD_MS);

        backward_vel = abs(backward_vel / float(N_samples));
        xSemaphoreTake(glob_Serial_mutex, portMAX_DELAY);
        Serial.print("DRVSYS_FOC_CAL: Backward velocity: ");
        Serial.println(backward_vel);
        xSemaphoreGive(glob_Serial_mutex);

        float vel_difference = abs(forward_vel - backward_vel);

        float larger_vel;
        float smaller_vel;
        if (backward_vel < forward_vel) {
            larger_vel = forward_vel;
            smaller_vel = backward_vel;
        }
        else {
            larger_vel = backward_vel;
            smaller_vel = forward_vel;
        }

        if ((forward_vel < 0.5) || (backward_vel < 0.5)) {
            score = -100;
        }
        else {

            score = (backward_vel + forward_vel) * 0.2 / vel_difference;

            score = (backward_vel + forward_vel) / larger_vel + smaller_vel / larger_vel + smaller_vel / 30;

        }
        xSemaphoreTake(glob_Serial_mutex, portMAX_DELAY);
        Serial.print("DRVSYS_FOC_CAL: Score: ");
        Serial.println(score);
        Serial.print("DRVSYS_FOC_CAL: Phase Angle value: ");
        Serial.println(drvSys_foc_controller.phase_null_angle);
        xSemaphoreGive(glob_Serial_mutex);

        if (score > highest_score) {
            highest_score = score;
            best_angle = drvSys_foc_controller.phase_null_angle;
        }


        //just sweep

        drvSys_foc_controller.phase_null_angle = drvSys_foc_controller.phase_null_angle + 1;

        if (drvSys_foc_controller.phase_null_angle > start_val + 2 * sweep_range) {
            calibration_finished = true;
        }

        xSemaphoreTake(glob_Serial_mutex, portMAX_DELAY);
        Serial.print("DRVSYS_FOC_CAL: New Phase Angle value: ");
        Serial.println(drvSys_foc_controller.phase_null_angle);
        xSemaphoreGive(glob_Serial_mutex);

        iteration++;

        if (iteration > 2 * sweep_range) {
            calibration_finished = true;
        }
        xSemaphoreTake(glob_Serial_mutex, portMAX_DELAY);
        Serial.print("DRVSYS_FOC_CAL: Iteration: ");
        Serial.println(iteration);
        Serial.print("DRVSYS_FOC_CAL: Highest score ");
        Serial.print(highest_score);
        Serial.print(" at ");
        Serial.println(best_angle);

        Serial.println("------------------------");
        xSemaphoreGive(glob_Serial_mutex);



    }

    drvSys_foc_controller.phase_null_angle = best_angle;
    xSemaphoreTake(glob_Serial_mutex, portMAX_DELAY);

    Serial.println("###------------------------###");
    Serial.println("###------------------------###");
    Serial.print("DRVSYS_FOC_CAL: Final phase angle: ");
    Serial.println(drvSys_foc_controller.phase_null_angle);
    Serial.println("###------------------------###");
    Serial.println("DRVSYS: Calibration Finished");
    Serial.println("###------------------------###");
    xSemaphoreGive(glob_Serial_mutex);


    vTaskDelay(2000 / portTICK_PERIOD_MS);

}







