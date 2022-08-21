#include <drive_system.h>
#include "soc/timer_group_struct.h"
#include "soc/timer_group_reg.h"
#include <signal_processing/IIRFilter.h>

#include <DallasTemperature.h>

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
SemaphoreHandle_t drvSys_mutex_position_command = xSemaphoreCreateBinary();
SemaphoreHandle_t drvSys_mutex_accel_command = xSemaphoreCreateBinary();
SemaphoreHandle_t drvSys_mutex_velocity_command = xSemaphoreCreateBinary();

/*Hardware Timer Setup Variables*/
const uint16_t drvSys_timer_prescaler_divider = DRVSYS_TIMER_PRESCALER_DIV; // with 80MHz Clock, makes the timer tick every 1us
const uint64_t drvSys_timer_alarm_rate_us = DRVSYS_TIMER_ALARM_RATE_US; //generate timer alarm every 50us

hw_timer_t* drvSys_foc_timer;
volatile const  int32_t drvSys_timer_foc_ticks = DRVSYS_FOC_PERIOD_US / drvSys_timer_alarm_rate_us;
volatile const int32_t drvSys_timer_encoder_process_ticks = DRVSYS_PROCESS_ENCODERS_PERIOD_US / drvSys_timer_alarm_rate_us;
volatile const int32_t drvSys_timer_torque_control_ticks = DRVSYS_CONTROL_TORQUE_PERIOD_US / drvSys_timer_alarm_rate_us;
volatile const int32_t drvSys_timer_pos_control_ticks = DRVSYS_CONTROL_POS_PERIOD_US / drvSys_timer_alarm_rate_us;
volatile const int32_t drvSys_timer_stepper_control_ticks = DRVSYS_CONTROL_STEPPER_PERIOD_US / drvSys_timer_alarm_rate_us;


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

/* Drive State Parameters */
drvSys_controllerCondition drvSys_controller_state = { .control_mode = closed_loop_foc,
.state_flag = not_ready,
.calibrated = false,
.hit_neg_limit = false,
.hit_positive_limit = false,
.overtemperature = false,
.temperature_warning = false,
.temperature = 20,
     .fan_level = 0,
    .neural_control_active = false };


drvSys_parameters drvSys_parameter_config;


/* #####################################################################################
####################### ----- Drive Components  ------ #################################
###################################################################################### */


AS5048A drvSys_magnetic_motor_encoder(CS_ENCODER);
AS5048A drvSys_magnetic_joint_encoder(CS_JOINT_ENCODER);

TMC2160Stepper drvSys_driver(CS_TMC, R_SENSE);

/* Dynamic Kalman Filter */
//DriveDynamicKalmanFilter kalman_filter(DRVSYS_FOC_PERIOD_US * 1e-6, DRVSYS_TRANSMISSION_RATIO);
KinematicKalmanFilter motor_kinematic_kalman_filter(float(DRVSYS_PROCESS_ENCODERS_PERIOD_US) * 1e-6);
KinematicKalmanFilter joint_kinematic_kalman_filter(float(DRVSYS_PROCESS_ENCODERS_PERIOD_US) * 1e-6);

/* FOC Controller */
FOCController drvSys_foc_controller(&drvSys_magnetic_motor_encoder, &drvSys_driver, DRVSYS_PHASE_CURRENT_MAX_mA, DRVSYS_TORQUE_CONSTANT, glob_SPI_mutex);


/* Closed Loop Stepper Controller */
ClosedLoopStepperController drvSys_stepper_controller(&drvSys_driver);

// Torque Sensor 
TorqueSensor drvSys_torque_sensor(DRVSYS_TORQUE_SENSOR_TYPE);

// Neural Network to learn inverse dynamics
NeuralController* neural_controller;
unsigned int drvSys_max_learning_iterations_per_step = DRVSYS_MAX_LEARNING_ITERATION_PER_STEP;
float drvSys_neural_control_pred_torque = 0.0;
bool drvSys_neural_control_auto_activation = DRVSYS_NN_CONTROL_AUTO_ACTIVE;


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
bool drvSys_joint_zero_set = false;
float drvSys_motor_encoder_offset = 0;


bool drvSys_joint_calibrated_flag = false;
bool drvSys_torque_sensor_available_flag = false;

/* Differentiators */
Differentiator drvSys_differentiator_motor_pos(DRVSYS_PROCESS_ENCODERS_FREQU);

long last_time_0 = 0;
long delta = 0;

/* ########## Controllers ################ */
PIDController drvSys_position_controller(0, 0, 0);
PIDController drvSys_velocity_controller(0, 0, 0);
float drvSys_vel_ff_gain = DRVSYS_VEL_FF_GAIN;
float drvSys_acc_ff_gain = DRVSYS_ACC_FF_GAIN;


/* --- Drive System Main State Variables --- */
// Motor Kinematic State
float drvSys_motor_position = 0;
float drvSys_motor_velocity = 0;
float drvSys_motor_acc = 0;
float drvSys_motor_position_prev = 0;
float drvSys_motor_velocity_prev = 0;
float drvSys_motor_acc_prev = 0;
// Joint Kinematic State
float drvSys_joint_position = 0;
float drvSys_joint_velocity = 0;
float drvSys_joint_acc = 0;
float drvSys_joint_position_prev = 0;
float drvSys_joint_velocity_prev = 0;
float drvSys_joint_acc_prev = 0;
// Motor Torque 
float drvSys_motor_torque_commanded = 0;
float drvSys_motor_torque_command_prev = 0;
// Joint Torque Sensor Value
float drvSys_joint_torque = 0.0;
float drvSys_joint_torque_prev = 0;
float drvSys_delta_angle = 0;

float torque_uncalibrated = 0.0;


// Peripheral Sensor Values
int32_t drvSys_hall_sensor_val = 0;
float drvSys_motor_temperature = 20.0;

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(TEMP_SENSE_PIN);
// Pass our oneWire reference to Dallas Temperature.
DallasTemperature dallas_temp(&oneWire);

/* Notch Filter */

drvSys_notch_filter_params drvSys_notch_filters;
float notch_b_coefs[3] = { 0 };

/* Temperature Sensor on Motor */

Generic_LM75_11Bit motor_temp_sensor;

//define Task Handlers
TaskHandle_t drvSys_foc_th;
TaskHandle_t drvSys_process_encoders_th;
TaskHandle_t drvSys_torque_controller_th;
TaskHandle_t drvSys_PID_dual_controller_th;
TaskHandle_t drvSys_process_joint_encoder_th;
TaskHandle_t drvSys_process_torque_sensor_th;
TaskHandle_t drvSys_admittance_controller_th;
TaskHandle_t drvSys_neural_controller_th;
TaskHandle_t drvSys_handle_periphal_sensors_th;
TaskHandle_t drvSys_stepper_controller_th;
TaskHandle_t drvSys_monitor_th;


const float encoder2Rad = PI / 8192.0;


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


float drvSys_pid_torque = 0;


/* ####### Drive System Preferences ########### */

//used to save parameters into flash memory
Preferences drv_sys_preferences;
//Namespaces for Parameters
// PID
const char* drvSys_PID_saved_gains = "PIDGains";
const char* drvSys_admittance_saved_gains = "admGains";
// Offset Calibration
const char* drvSys_encoder_offset = "posOffset";
// Alignment
const char* drvSys_alignment_setting = "align";
const char* drvSys_saved_offsets = "offs";


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

    xSemaphoreTake(drvSys_mutex_motor_commanded_torque, portMAX_DELAY);
    state.motor_torque = drvSys_motor_torque_commanded;
    xSemaphoreGive(drvSys_mutex_motor_commanded_torque);

    return state;

};

drvSys_FullDriveStateTimeSample drvSys_get_full_drive_state_time_samples() {
    drvSys_FullDriveStateTimeSample state_data;
    state_data.state = drvSys_get_full_drive_state();
    state_data.state_prev.joint_acc = drvSys_joint_acc_prev;
    state_data.state_prev.joint_pos = drvSys_joint_position_prev;
    state_data.state_prev.joint_vel = drvSys_joint_velocity_prev;
    state_data.state_prev.motor_acc = drvSys_motor_acc_prev;
    state_data.state_prev.motor_pos = drvSys_motor_position_prev;
    state_data.state_prev.motor_vel = drvSys_motor_velocity_prev;
    state_data.state_prev.motor_torque = drvSys_motor_torque_command_prev;
    state_data.state_prev.joint_torque = drvSys_joint_torque_prev;

    state_data.drvSys_feedback_torque = drvSys_pid_torque;

    return state_data;

}

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
    pinMode(HALL_SENSOR_PIN, INPUT); // make sure Hall sensor pin is set

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

    int n_falling = 0;

    float limit_angle = DRVSYS_CAL_ANGLE_LIMIT;

    word zero_pos_joint = 0;

    const int buffer_size = 50;
    CircularBuffer<float, buffer_size> buffer;
    CircularBuffer<float, buffer_size> buffer_noise;

    bool found_peak = false;
    int stepcount = DRVSYS_CAL_LARGE_STEPCOUNT;
    float prev_hall_val = analogRead(HALL_SENSOR_PIN);

    float start_angle = drvSys_magnetic_joint_encoder.getRotationDeg();
    float noise_multiplier = 10;

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

                zero_pos_joint = drvSys_magnetic_joint_encoder.getRawRotation(true);

                noise_multiplier = 1;
                largest_hall_value = current_average_val;

                Serial.print(" New Hall maximum at: ");
                Serial.println(zero_pos_joint);

                found_peak = true;
                //reduce step coint 
                stepcount = DRVSYS_CAL_SMALL_STEPCOUNT;
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
            delayMicroseconds(7);
            digitalWrite(TMC_STEP, LOW);
            delayMicroseconds(7);
        }

        // Finish Calibration 
        if (n_falling > 4000) {
            calibration_finished = true;

            drvSys_magnetic_joint_encoder.ProgramAbsolZeroPosition(zero_pos_joint);
            drvSys_joint_zero_set = true;
            drvSys_joint_encoder_offset = zero_pos_joint;


            Serial.println("DRVSYS_INFO: Joint Angle Offset Calibration finished");

            drvSys_joint_calibrated_flag = true;

        }

    }

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
    //drvSys_reset_alignment_data_on_Flash();
    //drvSys_reset_encoder_offset_data_on_Flash();
#endif 

    Serial.println("DRVSYS_INFO: Load Parameters from Flash... ");

    // load offset data;
    if (drv_Sys_check_if_joint_encoder_is_calibrated()) {
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

    _drvSys_read_PID_gains_from_flash();
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
    drvSys_PID_Gains pos_pid_gains = { .K_p = PID_POS_GAIN_P, .K_i = PID_POS_GAIN_I, .K_d = PID_POS_GAIN_D };
    drvSys_PID_Gains vel_pid_gains = { .K_p = PID_VEL_GAIN_P, .K_i = PID_VEL_GAIN_I, .K_d = PID_VEL_GAIN_D };
    drvSys_admittance_parameters drvSys_admittance_gains;
    drvSys_admittance_gains.virtual_spring = 0.0;
    drvSys_admittance_gains.virtual_damping = 0.0;
    drvSys_admittance_gains.virtual_inertia = 0.0;


    /* Set up Drive System Parameters */
    drvSys_parameter_config.max_current_mA = DRVSYS_PHASE_CURRENT_MAX_mA;
    drvSys_parameter_config.max_vel = DRVSYS_VEL_MAX;
    drvSys_parameter_config.max_torque_Nm = DRVSYS_TORQUE_LIMIT;
    drvSys_parameter_config.pos_pid_gains = pos_pid_gains;
    drvSys_parameter_config.vel_pid_gains = vel_pid_gains;
    drvSys_parameter_config.admittance_gains = drvSys_admittance_gains;
    drvSys_parameter_config.limit_high_deg = DRVSYS_POS_LIMIT_HIGH;
    drvSys_parameter_config.limit_low_deg = DRVSYS_POS_LIMIT_LOW;
    drvSys_parameter_config.endStops_enabled = DRVSYS_LIMITS_ENABLED;


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
    xSemaphoreGive(drvSys_mutex_velocity_command);
    xSemaphoreGive(drvSys_mutex_accel_command);
    xSemaphoreGive(drvSys_mutex_torque_target);
    xSemaphoreGive(glob_SPI_mutex);
    xSemaphoreGive(glob_SPI_mutex);



    // Initialize Torque Sensor

    drvSys_torque_sensor_available_flag = drvSys_torque_sensor.init(DRVSYS_TORQUE_SENSOR_TYPE);

    // Init Motor Temp Sensor
    if (TEMP_SENSOR_AVAILABLE) {
        drvSys_controller_state.temperature = motor_temp_sensor.readTemperatureHighC();
    }

    if (FAN_AVAILABLE) {
        ledcSetup(0, 312500, 4);
        ledcAttachPin(FAN_PWM_PIN, 0);

        ledcWrite(0, 0);
    }



    /* Initialize Hall Sensor for Calibration */
    pinMode(HALL_SENSOR_PIN, INPUT);

    // Setup FOC Driver
    _drvSys_setup_FOC_Driver();

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

    if (drvSys_joint_calibrated_flag) {
        float current_joint_angle = drvSys_magnetic_joint_encoder.getRotationCentered(true) * encoder2Rad;
        float current_motor_angle = drvSys_motor_encoder_dir_align * drvSys_magnetic_motor_encoder.getRotation(true) * encoder2Rad * (1.0 / drvSys_constants.transmission_ratio);
        drvSys_motor_encoder_offset = current_motor_angle - current_joint_angle;

    }

    // Initialize Kinematic Kalman Filters 
    motor_kinematic_kalman_filter.init(DRVSYS_PROCESS_ENCODERS_PERIOD_US * 1e-6);
    joint_kinematic_kalman_filter.init(DRVSYS_PROCESS_ENCODERS_PERIOD_US * 1e-6);

    joint_kinematic_kalman_filter.setAccelChange(DRVSYS_KIN_KALMAN_JOINT_ACCELERATION_STD);
    motor_kinematic_kalman_filter.setAccelChange(DRVSYS_KIN_KALMAN_MOTOR_ACCELERATION_STD);

    motor_kinematic_kalman_filter.sensor_noise = 0.05 * DEG2RAD / float(DRVSYS_TRANSMISSION_RATIO);

    // Setup Encoder task
    xTaskCreatePinnedToCore(
        _drvSys_process_encoders_task,   // function name
        "Process_encoder_task", // task name
        DRVSYS_STACKSIZE_PROCESS_ENCODER_TASK,      // Stack size (bytes)
        NULL,      // task parameters
        process_sensor_prio,         // task priority
        &drvSys_process_encoders_th,
        DRVSYS_ENCODER_CORE // task handle
    );

    // Setup Torque Sensor task
    if (drvSys_torque_sensor_available_flag) {
        xTaskCreatePinnedToCore(
            _drvSys_process_torque_sensor_task,   // function name
            "Process_torque_sensor_task", // task name
            DRVSYS_STACKSIZE_TORQUE_SENSE_TASK,      // Stack size (bytes)
            NULL,      // task parameters
            process_sensor_prio,         // task priority
            &drvSys_process_torque_sensor_th,
            DRVSYS_TORQUE_SENSE_CORE // task handle
        );
    }

    xTaskCreatePinnedToCore(
        _drvSys_monitor_system_task,   // function name
        "monitoring_task", // task name
        DRVSYS_STACKSIZE_MONITOR_TASK,      // Stack size (bytes)
        NULL,      // task parameters
        monitor_prio,         // task priority
        &drvSys_monitor_th,
        DRVSYS_MONITOR_CORE // task handle
    );



    //if initialization was succesful:
    drvSys_controller_state.state_flag = ready;

    drvSys_initialize_foc_based_control();

    drvSys_initialize_stepper_based_control();


};

void drvSys_initialize_foc_based_control() {
    // Initialize Notch Filters
    drvSys_notch_filters.notch_active = DRVSYS_NOTCH_ACTIVE;
    drvSys_notch_filters.notch_frequ = DRVSYS_NOTCH_FREQU; //Hz

    drvSys_set_notch_filter(drvSys_notch_filters.notch_frequ, drvSys_notch_filters.notch_active);


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

    _drvSys_neural_controller_setup();

    //suspend task until controller is started
    vTaskSuspend(drvSys_admittance_controller_th);
}

void _drvSys_neural_controller_setup() {

    Serial.println("DRVSYS_INFO: Setup NN for Inverse Dynamics");

    neural_controller = new NeuralController();
    neural_controller->init();

    drvSys_controller_state.neural_control_active = false;

    neural_controller->init_pid_learner(drvSys_position_controller, drvSys_velocity_controller);

    /* --- create Task --- */
    xTaskCreatePinnedToCore(
        _drvSys_learn_neural_control_task,   // function name
        "Learn_Dynamics_Task", // task name
        DRVSYS_STACKSIZE_LEARN_DYNAMICS_TASK,      // Stack size (bytes)
        NULL,      // task parameters
        learn_dynamics_prio,         // task priority
        &drvSys_neural_controller_th,
        DRVSYS_LEARNING_CORE // task handle
    );

    Serial.println("DRVSYS_INFO: Finished NN Inverse Dynamics Setup");


}

void drvSys_initialize_stepper_based_control() {

    Serial.println("DRVSYS_INFO: Initialize Closed Loop Stepper Controller");


    xTaskCreatePinnedToCore(
        _drvSys_closed_loop_stepper_task,   // function name
        "Stepper_Controller_task", // task name
        DRVSYS_STACKSIZE_PROCESS_ENCODER_TASK,      // Stack size (bytes)
        NULL,      // task parameters
        stepper_control_prio,         // task priority
        &drvSys_stepper_controller_th,
        1 // task handle
    );

    vTaskSuspend(drvSys_stepper_controller_th);

}

void _drvSys_start_stepper_controller() {
    Serial.println("DRVSYS_INFO: Setup Closed Loop Stepper Controller");

    drvSys_stepper_controller.setup(&drvSys_driver);

    drvSys_stepper_controller.set_target_vel(0);



    // Stop Torque Based Controllers

    vTaskSuspend(drvSys_torque_controller_th);
    vTaskSuspend(drvSys_admittance_controller_th);
    vTaskSuspend(drvSys_foc_th);
    vTaskSuspend(drvSys_PID_dual_controller_th);


    Serial.println("DRVSYS_INFO: Start Stepper Controller");
    drvSys_stepper_controller.start();
    vTaskResume(drvSys_stepper_controller_th);

    drvSys_controller_state.control_mode = stepper_mode;
}


void _drvSys_stop_stepper_controller() {

    Serial.println("DRVSYS_INFO: Stop Stepper Controller");

    drvSys_stepper_controller.stop();
    vTaskSuspend(drvSys_stepper_controller_th);


    vTaskResume(drvSys_torque_controller_th);
    vTaskResume(drvSys_admittance_controller_th);
    vTaskResume(drvSys_foc_th);
    vTaskResume(drvSys_PID_dual_controller_th);


}

cascade_gains _drvSys_predict_pid_gains() {

    cascade_gains gains = neural_controller->predict_gains(drvSys_get_full_drive_state(), drvSys_get_targets());

    return gains;
};

float drvSys_pid_tuner_error(bool average) {

    if (average) {
        return neural_controller->average_pid_control_error;
    }
    else {
        return neural_controller->pid_control_error;
    }

}


drvSys_PID_Gains drvSys_get_PID_gains(bool pos_controller) {


    drvSys_PID_Gains gains;
    if (pos_controller) {
        float* gains_ptr = drvSys_position_controller.getGains();

        gains.K_p = gains_ptr[0];
        gains.K_i = gains_ptr[1];
        gains.K_d = gains_ptr[2];
    }
    else {
        float* gains_ptr = drvSys_velocity_controller.getGains();

        gains.K_p = gains_ptr[0];
        gains.K_i = gains_ptr[1];
        gains.K_d = gains_ptr[2];
    }

    return gains;
}


drvSys_driveState drvSys_get_emulator_pred() {

    drvSys_driveState pred;
    pred = neural_controller->emulator_predict_next_state(drvSys_get_full_drive_state());
    return pred;
}

void _drvSys_learn_neural_control_task(void* parameters) {

    static long counter = 0;
    const TickType_t learning_delay = DRVSYS_LEARNING_PERIOD_MS / portTICK_PERIOD_MS;
    const int learn_iterations = 1;
    static long learning_counter = 0;
    const int minimum_learning_iterations = 1e3;

    const int learn_iterations_pid_tuner = 1;



    while (true) {

        if (drvSys_controller_state.control_mode == closed_loop_foc && drvSys_controller_state.state_flag == closed_loop_control_active) {

            if (learning_counter > minimum_learning_iterations || neural_controller->control_net_pretrained) {
                if (drvSys_neural_control_auto_activation) {

                    //drvSys_controller_state.neural_control_active = true;
                }

            }

            if (learning_counter > minimum_learning_iterations && neural_controller->average_control_error < 0.1) {




            }

            //drvSys_controller_state.neural_control_active = true;
            // Collecting Samples for Training
            drvSys_FullDriveStateTimeSample sample_data = drvSys_get_full_drive_state_time_samples();
            drvSys_driveTargets targets = drvSys_get_targets();
            neural_controller->add_sample(sample_data, targets);
            neural_controller->learning_step_controller();

            float pos_pid_prev_error = drvSys_position_controller.prev_error;
            float pos_pid_errsum = drvSys_position_controller.iTerm;
            float vel_pid_errsum = drvSys_velocity_controller.iTerm;
            neural_controller->add_pid_sample(sample_data.state, targets, pos_pid_errsum, pos_pid_prev_error, vel_pid_errsum);

            if (counter % learn_iterations_pid_tuner == 0) {
                neural_controller->learning_step_pid_tuner();
                taskYIELD();
            }

            if (counter % learn_iterations == 0) {
                for (int i = 0; i < 20; i++) {
                    neural_controller->learning_step_emulator();
                    //neural_controller->learning_step_inverse_dyn();


                    /*if (drvSys_controller_state.neural_control_active) {
                        neural_controller->learning_step_controller();
                    }*/
                    taskYIELD();
                }
                learning_counter++;

            }
        }

        counter++;
        vTaskDelay(learning_delay);
    }


}

void drvSys_inv_dyn_nn_set_max_learning_iterations_per_step(unsigned int n_iterations) {
    drvSys_max_learning_iterations_per_step = n_iterations;
}


float drvSys_neural_control_read_predicted_torque() {
    return drvSys_neural_control_pred_torque;
}

float drvSys_get_pid_torque() {
    return drvSys_pid_torque;
}

float drvSys_neural_control_error() {
    return neural_controller->control_error;
}

float _drvSys_neural_control_predict_torque() {

    drvSys_FullDriveState state = drvSys_get_full_drive_state();
    drvSys_driveTargets targets = drvSys_get_targets();

    float predicted_torque = neural_controller->predict_control_torque(state, targets);

    return predicted_torque;
}

void drvSys_neural_control_activate(bool active) {

    drvSys_controller_state.neural_control_active = active;
}

void drvSys_adapt_nn_parameter(int nn_type, int parameter_type, float value) {
    if (nn_type == 0) {
        if (parameter_type == 0) { // learning rate
            neural_controller->emulator_nn->learning_rate = value;
            return;
        }
        else if (parameter_type == 1) { // max gradient
            neural_controller->emulator_nn->max_gradient_abs = value;
        }
        else if (parameter_type == 2) { // min learning rate
            neural_controller->emulator_nn->minimal_learning_rate = value;
        }
        else if (parameter_type == 3) { // error learning rate factor
            neural_controller->emulator_nn->lr_error_factor = value;
        }
        else if (parameter_type == 4) { // maximum learning rate
            neural_controller->emulator_nn->maximal_learning_rate = value;
        }
        else if (parameter_type == 5) {
            if (value == 0.0) {
                neural_controller->emulator_nn->lr_schedule = no_schedule;
            }
            else if (value == 1.0) {
                neural_controller->emulator_nn->lr_schedule = error_adaptive;
            }
        }
    }
    if (nn_type == 1) {
        if (parameter_type == 0) { // learning rate
            neural_controller->controller_nn->learning_rate = value;
            return;
        }
        else if (parameter_type == 1) { // max gradient
            neural_controller->controller_nn->max_gradient_abs = value;
        }
        else if (parameter_type == 2) { // min learning rate
            neural_controller->controller_nn->minimal_learning_rate = value;
        }
        else if (parameter_type == 3) { // error learning rate factor
            neural_controller->controller_nn->lr_error_factor = value;
        }
        else if (parameter_type == 4) { // maximum learning rate
            neural_controller->controller_nn->maximal_learning_rate = value;
        }
        else if (parameter_type == 5) {
            if (value == 0.0) {
                neural_controller->controller_nn->lr_schedule = no_schedule;
            }
            else if (value == 1.0) {
                neural_controller->controller_nn->lr_schedule = error_adaptive;
            }
        }
    }
}
/**
 * @brief
 *
 * @param nn_type 0 - Emulator, 1-Controller
 * @param error_type 0 error, 1 filtered error
 * @return float
 */
float drvSys_get_neural_control_error(int nn_type, int error_type) {
    if (nn_type == 0) {
        if (error_type == 0) {
            return neural_controller->emulator_error;
        }
        if (error_type == 1) {
            return neural_controller->average_emulator_error;
        }
    }
    if (nn_type == 1) {
        if (error_type == 0) {
            return neural_controller->control_error;
        }
        if (error_type == 1) {
            return neural_controller->average_control_error;
        }
    }
    return 0;
}

void drvSys_neural_control_save_nets(bool reset) {
    if (reset) {
        neural_controller->reset_controller_network();
        neural_controller->reset_emulator_network();
    }
    else {
        neural_controller->save_controller_network();
        neural_controller->save_emulator_network();
    }

}



int32_t  drvSys_start_foc_processing() {

    if (drvSys_controller_state.state_flag == not_ready) {
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
    drvSys_pos_target = drvSys_joint_position;

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
    drvSys_controller_state.state_flag = closed_loop_control_inactive;

    return 1;
};

int32_t drvSys_start_motion_control(drvSys_controlMode control_mode) {

    if (drvSys_controller_state.state_flag == not_ready || drvSys_controller_state.state_flag == error) {

        Serial.println("DRVSYS_ERROR: Drive not ready to start Closed Loop Motion Control.");

        return -1;

    }

    if (drvSys_controller_state.state_flag == closed_loop_control_inactive) {

        Serial.println("DRVSYS_INFO: Start Closed Loop Motion Control.");
        drvSys_controlMode drvSys_mode = control_mode;


        switch (drvSys_mode) {
        case closed_loop_foc:

            _drvSys_setup_dual_controller();
            break;

        case direct_torque:
            _drvSys_setup_direct_controller();
            break;

        case admittance_control:
            _drvSys_setup_admittance_controller();
            break;
        case stepper_mode:
            _drvSys_start_stepper_controller();
            break;
        default:
            _drvSys_setup_dual_controller();
            break;
        }
        drvSys_controller_state.control_mode = drvSys_mode;


        drvSys_controller_state.state_flag = closed_loop_control_active;
        _drvSys_set_target_torque(0.0);
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

    float vel_limit = 180.0 * DEG2RAD;
    drvSys_position_controller.setOutputLimits(vel_limit * (-1.0), vel_limit);
    drvSys_position_controller.setpoint = 0;


    drvSys_position_controller.Initialize();
    drvSys_position_controller.setMode(PID_MODE_INACTIVE);
    drvSys_position_controller.SetControllerDirection(PID_DIR_DIRECT);
    drvSys_position_controller.setDifferentialFilter(DRVSYS_POS_PID_FILTER_DERIVATIVE, DRVSYS_POS_PID_FILTER_DERIVATIVE_ALPHA);
    drvSys_position_controller.setErrorDeadBand(DRVSYS_POS_PID_DEADBAND);
    drvSys_position_controller.derivative_on_measurement = DRVSYS_POS_PID_DERIVATIVE_ON_MEASUREMENT;
    drvSys_position_controller.setTuning(drvSys_parameter_config.pos_pid_gains.K_p, drvSys_parameter_config.pos_pid_gains.K_i, drvSys_parameter_config.pos_pid_gains.K_d);

    drvSys_position_controller.d_term_vel = false;
    _drvSys_read_PID_gains_from_flash();


    /* Position Controller */
    drvSys_velocity_controller.setSampleTime(DRVSYS_CONTROL_VEL_PERIOD_US);
    //Velocity Limitation
    drvSys_velocity_controller.setOutputLimits(torque_limit * (-1.0), torque_limit);
    drvSys_velocity_controller.setpoint = 0.0;


    drvSys_velocity_controller.Initialize();
    drvSys_velocity_controller.setMode(PID_MODE_INACTIVE);
    drvSys_velocity_controller.SetControllerDirection(PID_DIR_DIRECT);
    drvSys_velocity_controller.setDifferentialFilter(1, 0.03);
    drvSys_velocity_controller.setErrorDeadBand(0);

    drvSys_velocity_controller.setInputFilter(true, DRVSYS_VEL_PID_INPUT_FILTER_ALPHA);

    drvSys_velocity_controller.derivative_on_measurement = DRVSYS_POS_PID_DERIVATIVE_ON_MEASUREMENT;
    drvSys_velocity_controller.setTuning(drvSys_parameter_config.vel_pid_gains.K_p, drvSys_parameter_config.vel_pid_gains.K_i, drvSys_parameter_config.vel_pid_gains.K_d);



    Serial.println("DRVSYS_INFO: Setup PID Position Controller with Velocity Feedforward");

    drvSys_pos_target = drvSys_joint_position;
    drvSys_vel_target = 0;

    drvSys_position_controller.setMode(PID_MODE_ACTIVE);
    drvSys_velocity_controller.setMode(PID_MODE_ACTIVE);
    drvSys_torque_ff = 0.0;



};



void drvSys_stop_controllers() {

    drvSys_position_controller.setMode(PID_MODE_INACTIVE);

    //deactivate inverse dynamics controller
    //...
    drvSys_controller_state.state_flag = closed_loop_control_inactive;

    if (drvSys_controller_state.control_mode == stepper_mode) {
        _drvSys_stop_stepper_controller();
    }

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

void drvSys_set_ff_gains(float gain) {

    drvSys_vel_ff_gain = gain;

}

void drvSys_set_notch_filter(float notch_frequ, bool activate) {

    // Calculate coefficients

    float sample_frequ = DRVSYS_CONTROL_TORQUE_FREQU;

    // notch frequency
    float omega_c = 2.0 * PI * notch_frequ / sample_frequ;

    // Coefficients

    float divisor = 2 - 2 * cos(omega_c);

    float b_0 = 1.0 / divisor;
    float b_1 = -2.0 * cos(omega_c) / divisor;
    float b_2 = 1 / divisor;

    notch_b_coefs[0] = b_0;
    notch_b_coefs[1] = b_1;
    notch_b_coefs[2] = b_2;

    xSemaphoreTake(glob_Serial_mutex, portMAX_DELAY);
    Serial.print("DRVSYS_INFO: Set up Notch Filter");
    Serial.print(" with f_c = ");
    Serial.println(notch_frequ);
    xSemaphoreGive(glob_Serial_mutex);


};

float _drvSys_compute_notch_FIR_filter(float input, float* b_coef) {

    static float prev_input[2] = { 0 };

    float output = input * b_coef[0] + b_coef[1] * prev_input[0] + prev_input[1] * b_coef[2];

    prev_input[1] = prev_input[0];
    prev_input[0] = input;


    return output;

}

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

    if (torque > torque_limit) {
        torque = torque_limit;
    }
    else if (torque < (-1.0) * torque_limit) {
        torque = (-1.0) * torque_limit;
    }
    xSemaphoreTake(drvSys_mutex_torque_target, portMAX_DELAY);
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
    xSemaphoreTake(drvSys_mutex_velocity_command, portMAX_DELAY);
    drvSys_vel_target = vel;
    xSemaphoreGive(drvSys_mutex_velocity_command);

};


void _drvSys_set_target_pos(float pos) {

    if (pos > drvSys_parameter_config.limit_high_deg) {
        pos = drvSys_parameter_config.limit_high_deg;
    }
    else if (pos < drvSys_parameter_config.limit_low_deg) {
        pos = drvSys_parameter_config.limit_low_deg;
    }
    xSemaphoreTake(drvSys_mutex_position_command, portMAX_DELAY);
    drvSys_pos_target = pos;
    xSemaphoreGive(drvSys_mutex_position_command);

};

const drvSys_driveTargets drvSys_get_targets() {
    drvSys_driveTargets targets;

    targets.motor_torque_ff = drvSys_torque_ff;
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

    // Process Encoders & Filter Data
    if (tickCount % drvSys_timer_encoder_process_ticks == 0) { //react every 250us
        vTaskNotifyGiveFromISR(drvSys_process_encoders_th, &xHigherPriorityTaskWoken);
    }

    // Torque Based Control Active
    if (drvSys_controller_state.control_mode != stepper_mode) {
        if (tickCount % drvSys_timer_foc_ticks == 0) { // react every 200us ->5kHz
            vTaskNotifyGiveFromISR(drvSys_foc_th, &xHigherPriorityTaskWoken);
        }
        if (tickCount % drvSys_timer_torque_control_ticks == 0) {
            vTaskNotifyGiveFromISR(drvSys_torque_controller_th, &xHigherPriorityTaskWoken);
        }

        if (drvSys_controller_state.state_flag == closed_loop_control_active) {

            if (tickCount % drvSys_timer_pos_control_ticks == 0) {
                vTaskNotifyGiveFromISR(drvSys_PID_dual_controller_th, &xHigherPriorityTaskWoken);
            }
        }
    }

    if (drvSys_controller_state.control_mode == stepper_mode) {

        if (tickCount % drvSys_timer_stepper_control_ticks == 0) {
            vTaskNotifyGiveFromISR(drvSys_stepper_controller_th, &xHigherPriorityTaskWoken);
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

    static const float encoder2Rad = PI / 8192.0;

    static const float inverse_transmission = 1.0 / drvSys_constants.transmission_ratio;

    static long counter = 0;

    const int divider = 4;


    uint32_t encoder_processing_thread_notification;
    while (true) {
        encoder_processing_thread_notification = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);


        if (encoder_processing_thread_notification) {

            /* use Kalman Filter */
            float motor_pos_sensor_val;
            if (drvSys_controller_state.control_mode == stepper_mode) {
                motor_pos_sensor_val = drvSys_motor_encoder_dir_align * drvSys_magnetic_motor_encoder.getRotation(true) * encoder2Rad * inverse_transmission;
            }
            else {
                motor_pos_sensor_val = drvSys_motor_encoder_dir_align * drvSys_magnetic_motor_encoder.last_sample * encoder2Rad * inverse_transmission;
            }

            motor_pos_sensor_val = motor_pos_sensor_val - drvSys_motor_encoder_offset;


            KinematicStateVector motor_state = motor_kinematic_kalman_filter.estimateStates(motor_pos_sensor_val);

            xSemaphoreTake(drvSys_mutex_motor_position, portMAX_DELAY);
            if (counter % divider == 0) {
                drvSys_motor_position_prev = drvSys_motor_position;
            }
            drvSys_motor_position = motor_state.pos;
            xSemaphoreGive(drvSys_mutex_motor_position);

            xSemaphoreTake(drvSys_mutex_motor_vel, portMAX_DELAY);
            if (counter % divider == 0) {
                drvSys_motor_velocity_prev = drvSys_motor_velocity;
            }
            drvSys_motor_velocity = motor_state.vel;
            xSemaphoreGive(drvSys_mutex_motor_vel);

            xSemaphoreTake(drvSys_mutex_motor_acc, portMAX_DELAY);
            if (counter % divider == 0) {
                drvSys_motor_acc_prev = drvSys_motor_acc;
            }
            drvSys_motor_acc = motor_state.acc;
            xSemaphoreGive(drvSys_mutex_motor_acc);

            xSemaphoreTake(glob_SPI_mutex, portMAX_DELAY);
            float raw_joint_angle_rad = drvSys_joint_encoder_dir_align * drvSys_magnetic_joint_encoder.getRotationCentered(false) * encoder2Rad;
            xSemaphoreGive(glob_SPI_mutex);

            raw_joint_angle_rad = raw_joint_angle_rad - drvSys_angle_offset_joint;

            KinematicStateVector joint_state = joint_kinematic_kalman_filter.estimateStates(raw_joint_angle_rad);

            xSemaphoreTake(drvSys_mutex_joint_acc, portMAX_DELAY);
            if (counter % divider == 0) {
                drvSys_joint_acc_prev = drvSys_joint_acc;
            }
            drvSys_joint_acc = joint_state.acc;
            xSemaphoreGive(drvSys_mutex_joint_acc);

            xSemaphoreTake(drvSys_mutex_joint_vel, portMAX_DELAY);
            if (counter % divider == 0) {
                drvSys_joint_velocity_prev = drvSys_joint_velocity;
            }
            drvSys_joint_velocity = joint_state.vel;
            xSemaphoreGive(drvSys_mutex_joint_vel);

            xSemaphoreTake(drvSys_mutex_joint_position, portMAX_DELAY);
            if (counter % divider == 0) {
                drvSys_joint_position_prev = drvSys_joint_position;
            }
            drvSys_joint_position = joint_state.pos;
            drvSys_delta_angle = drvSys_joint_position - drvSys_motor_position;
            xSemaphoreGive(drvSys_mutex_joint_position);


            //handle joint torque 
            if (counter % divider == 0) {
                drvSys_joint_torque_prev = drvSys_joint_torque;
            }

            counter++;



        }
    }

};

void _drvSys_process_torque_sensor_task(void* parameters) {

    const TickType_t torque_sensor_delay = DRVSYS_PROCESS_TORQUE_SENSOR_PERIOD_MS / portTICK_PERIOD_MS;

    while (true) {
        //...

        float torque_data = drvSys_torque_sensor.get_torque_measurement();

        torque_uncalibrated = torque_data;
        if (drvSys_torque_sensor.calibrated) {
            xSemaphoreTake(drvSys_mutex_joint_torque, portMAX_DELAY);
            drvSys_joint_torque_prev = drvSys_joint_torque;
            drvSys_joint_torque = torque_data;
            xSemaphoreGive(drvSys_mutex_joint_torque);
        }
        vTaskDelay(torque_sensor_delay);
    }
};

void _drvSys_PID_dual_controller_task(void* parameters) {
    uint32_t position_controller_processing_thread_notification;

    static int counter = 0;

    static float position_control_output = 0;

    static const int pos_divider = DRVSYS_CONTROL_POS_PERIOD_US / DRVSYS_CONTROL_VEL_PERIOD_US;

    static const int gain_update_divider = 10000 / DRVSYS_CONTROL_VEL_PERIOD_US; // every 200 Hz


    const float frequ_limit = DRVSYS_NN_CONTROL_BANDWIDTH;
    const float torque_ff_alpha = 1 - exp(-DRVSYS_CONTROL_VEL_PERIOD_US * 1e-6 / (1.0 / frequ_limit));
    static float prev_torque_ff = 0;
    static float torque_ff = 0;

    while (true) {

        position_controller_processing_thread_notification = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if (position_controller_processing_thread_notification) {


            if (drvSys_controller_state.control_mode == closed_loop_foc || drvSys_controller_state.control_mode == admittance_control) {

                if (counter % pos_divider == 0) { // Handle position controller with 800 Hz

                    if (counter % gain_update_divider == 0) {
                        cascade_gains updated_gains = neural_controller->predict_gains(drvSys_get_full_drive_state(), drvSys_get_targets());

                        drvSys_velocity_controller.setTuning(updated_gains.vel_Kp, updated_gains.vel_Ki, 0);
                        drvSys_position_controller.setTuning(updated_gains.pos_Kp, updated_gains.pos_Ki, updated_gains.pos_Kd);

                        drvSys_vel_ff_gain = updated_gains.vel_ff_gain;
                        drvSys_acc_ff_gain = updated_gains.acc_ff_gain;

                    };

                    xSemaphoreTake(drvSys_mutex_joint_position, portMAX_DELAY);
                    float actual_pos_joint = drvSys_joint_position;
                    xSemaphoreGive(drvSys_mutex_joint_position);

                    xSemaphoreTake(drvSys_mutex_position_command, portMAX_DELAY);
                    float pos_target_joint = drvSys_pos_target;
                    xSemaphoreGive(drvSys_mutex_position_command);

                    drvSys_position_controller.setSetPoint(pos_target_joint, false);

                    drvSys_position_controller.input = actual_pos_joint;
                    drvSys_position_controller.compute();

                    position_control_output = drvSys_position_controller.output;
                }

                // Handle Velocity Controller with full frequency (2500Hz)
                xSemaphoreTake(drvSys_mutex_motor_vel, portMAX_DELAY);
                float actual_vel = drvSys_motor_velocity;
                xSemaphoreGive(drvSys_mutex_motor_vel);

                xSemaphoreTake(drvSys_mutex_velocity_command, portMAX_DELAY);
                float target_vel = drvSys_vel_target;
                xSemaphoreGive(drvSys_mutex_velocity_command);


                float velocity_target_val = target_vel + position_control_output;


                drvSys_velocity_controller.setSetPoint(velocity_target_val, false);
                drvSys_velocity_controller.input = actual_vel;
                drvSys_velocity_controller.compute();


                /* Handle controller output */

                drvSys_pid_torque = drvSys_velocity_controller.output;

                float torque_ff_static = drvSys_vel_target * drvSys_vel_ff_gain + drvSys_acc_target * drvSys_acc_ff_gain;

                if (counter % pos_divider == 0) {
                    if (drvSys_controller_state.neural_control_active) {
                        drvSys_neural_control_pred_torque = _drvSys_neural_control_predict_torque();
                        torque_ff = torque_ff_alpha * drvSys_neural_control_pred_torque + (1.0 - torque_ff_alpha) * prev_torque_ff;
                        drvSys_neural_control_pred_torque = torque_ff;
                        prev_torque_ff = torque_ff;

                    }
                }

                //Serial.println(drvSys_pid_torque);

                //float motor_torque_target = drvSys_pid_torque + drvSys_vel_target * drvSys_vel_ff_gain + drvSys_acc_target * drvSys_acc_ff_gain;


                float motor_torque_target = drvSys_pid_torque + drvSys_neural_control_pred_torque + torque_ff_static;


                _drvSys_set_target_torque(motor_torque_target);
                counter++;


            }
        }
    }
}


void _drvSys_torque_controller_task(void* parameters) {
    TIMERG0.wdt_wprotect = TIMG_WDT_WKEY_VALUE;
    TIMERG0.wdt_feed = 1;
    TIMERG0.wdt_wprotect = 0;
    uint32_t torque_controller_processing_thread_notification;

    //const float frequ_limit = 1.0 / (2.0 * DRVSYS_CONTROL_TORQUE_PERIOD_US * 1e-6);

    static int counter = 0;

    float motor_torque_command = 0;


    while (true) {
        torque_controller_processing_thread_notification = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if (torque_controller_processing_thread_notification) {

            /*
            xSemaphoreTake(drvSys_mutex_motor_commanded_torque, portMAX_DELAY);
            drvSys_motor_torque_command_prev = drvSys_motor_torque_commanded;
            xSemaphoreGive(drvSys_mutex_motor_commanded_torque);
            */

            /*
            if (drvSys_controller_state.neural_control_active) {
                if (counter % 1 == 0) {
                    drvSys_neural_control_pred_torque = _drvSys_neural_control_predict_torque();
                    torque_ff = torque_ff_alpha * drvSys_neural_control_pred_torque + (1.0 - torque_ff_alpha) * prev_torque_ff;
                    drvSys_neural_control_pred_torque = torque_ff;
                    prev_torque_ff = torque_ff;
                }
            }
            */

            motor_torque_command = _drvSys_torque_target + drvSys_torque_ff;// + torque_ff;

            // Apply Notch Filter to avoid resonance frequency

            if (drvSys_notch_filters.notch_active) {
                motor_torque_command = _drvSys_compute_notch_FIR_filter(motor_torque_command, notch_b_coefs);
            }


            xSemaphoreTake(drvSys_mutex_motor_commanded_torque, portMAX_DELAY);
            drvSys_motor_torque_commanded = _drvSys_check_joint_limit(motor_torque_command);
            drvSys_foc_controller.set_target_torque(drvSys_motor_torque_commanded);
            if (counter % 4 == 0) {
                drvSys_motor_torque_command_prev = drvSys_motor_torque_commanded;
            }
            xSemaphoreGive(drvSys_mutex_motor_commanded_torque);
            counter++;

        }
    };
};


float drvSys_get_torque(bool raw) {

    if (raw) {
        return drvSys_torque_sensor.raw_sensor_val;
    }
    if (drvSys_torque_sensor.calibrated) {
        return drvSys_joint_torque;
    }
    else {
        return torque_uncalibrated;
    }

}

void _drvSys_closed_loop_stepper_task(void* parameters) {

    TIMERG0.wdt_wprotect = TIMG_WDT_WKEY_VALUE;
    TIMERG0.wdt_feed = 1;
    TIMERG0.wdt_wprotect = 0;
    uint32_t stepper_controller_thread_notification;

    static float dir_vel = 1.0;
    static float dir_pos = 1.0;

    if (FLIP_DIR_VEL) {
        dir_vel = -1.0;
    }

    if (FLIP_DIR_POS) {
        dir_pos = -1.0;
    }

    while (true) {
        stepper_controller_thread_notification = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if (stepper_controller_thread_notification) {

            xSemaphoreTake(drvSys_mutex_position_command, portMAX_DELAY);
            float target_pos = drvSys_pos_target;
            xSemaphoreGive(drvSys_mutex_position_command);

            xSemaphoreTake(drvSys_mutex_velocity_command, portMAX_DELAY);
            float target_vel = drvSys_vel_target;
            xSemaphoreGive(drvSys_mutex_velocity_command);


            xSemaphoreTake(drvSys_mutex_joint_position, portMAX_DELAY);
            float actual_pos = drvSys_joint_position;
            xSemaphoreGive(drvSys_mutex_joint_position);

            xSemaphoreTake(drvSys_mutex_joint_vel, portMAX_DELAY);
            float actual_vel = drvSys_joint_velocity;
            xSemaphoreGive(drvSys_mutex_joint_vel);

            float pos_error = actual_pos - target_pos;

            float pos_error_gain = 0.5;


            float vel_target = dir_vel * target_vel + dir_pos * pos_error * pos_error_gain;

            drvSys_stepper_controller.set_target_vel(vel_target);

        }

    }
}


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

    static bool overtemp_warning = false;
    static bool was_in_overtemp = false;
    static bool undervoltage = false;
    static bool error_sig = false;
    static bool prev_error = false;
    static bool encoder_diff_error = false;

    static bool increased_temp_state = false;

    int error_iterations = 0;

    static int fan_val = 0;

    float temperature_c = 20;

    static int count_recover_iterations = 0;


    const TickType_t monitoring_delay = DRVSYS_MONITOR_PERIOD_MS / portTICK_PERIOD_MS;


    dallas_temp.begin();


    while (true) {
        drvSys_hall_sensor_val = analogRead(HALL_SENSOR_PIN);


        if (TEMP_SENSOR_AVAILABLE) {
            dallas_temp.requestTemperatures(); // Send the command to get temperatures
            float read_temp = dallas_temp.getTempCByIndex(0);
            if ((read_temp > -50)) {
                temperature_c = read_temp;
            }

            drvSys_controller_state.temperature = temperature_c;
        }

        if (FAN_AVAILABLE) {

            fan_val = 15.0 * (temperature_c - 25.0) / (65 - 25);

            if (fan_val > 15) {
                fan_val = 15;
            }

            if (FAN_12V) {
                fan_val = int(float(fan_val) * 0.5);

            }

            ledcWrite(0, fan_val);

        }

        xSemaphoreTake(glob_SPI_mutex, portMAX_DELAY);

        undervoltage = drvSys_driver.uv_cp();
        error_sig = drvSys_driver.drv_err();
        drvSys_driver.GSTAT(0b111);
        bool overtemp = drvSys_driver.ot();
        bool overtemp_pre_warn = drvSys_driver.otpw();

        xSemaphoreGive(glob_SPI_mutex);


        if (!undervoltage) { // Driver Temperature Sensor only reliable when there is no undervoltage

            drvSys_controller_state.overtemperature = overtemp;
            drvSys_controller_state.temperature_warning = overtemp_pre_warn;

            if (drvSys_controller_state.temperature_warning && !increased_temp_state) {

                Serial.println("DRVSYS_INFO: Driver Temperature is high. Reducing Output Current.");
                drvSys_parameter_config.max_torque_Nm = DRVSYS_TORQUE_CONSTANT;

                increased_temp_state = true;

                //if fan available turn to max

                if (FAN_AVAILABLE) {

                    if (FAN_12V) {
                        fan_val = 8;
                    }
                    else {
                        fan_val = 15;
                    }

                    ledcWrite(0, fan_val);
                    drvSys_controller_state.fan_level = 15; //4 bit maxima
                }
            }

            if (!drvSys_controller_state.temperature_warning && increased_temp_state) {
                //leave increased temp state
                increased_temp_state = false;

                if (FAN_AVAILABLE) {
                    if (FAN_12V) {
                        fan_val = 4;
                    }
                    else {
                        fan_val = 8;
                    }

                    drvSys_controller_state.fan_level = 8; //4 bit maxima
                }
            }

            if (drvSys_controller_state.overtemperature) {

                Serial.println("DRVSYS_INFO: Driver overtemperature. Stopping Controller.");
                drvSys_controller_state.state_flag == error;
                set_leds(255, 0, 0, true, 100);

                was_in_overtemp = true;

            }

            if (drvSys_controller_state.overtemperature == false && was_in_overtemp) {
                Serial.println("DRVSYS_INFO: Driver cooled down. Driver is ready again.");
                drvSys_controller_state.state_flag = ready;
                set_leds(0, 0, 255, true, 500);

                was_in_overtemp = false;
            }

        }


        float sensor_difference = abs(drvSys_motor_position - drvSys_joint_position);

        if (sensor_difference * RAD2DEG > DRVSYS_SENSOR_MAX_DIFFERENCE_DEG) {
            // hit error flag
            error_iterations++;
            //Handle error
            drvSys_controller_state.state_flag = error;

            Serial.println("DRVSYS_ERROR: Encoder Difference too high. One Encoder might be faulty.");
            encoder_diff_error = true;
        }
        else {
            error_iterations = 0;
            encoder_diff_error = false;
        }


        if (error_sig) {
            drvSys_controller_state.state_flag = error;
            Serial.println("DRVSYS_INFO: Drive Error detected");
        }
        else {
            error_sig = false;
        }

        if (undervoltage) {
            Serial.println("DRVSYS_INFO: Undervoltage detected");
            drvSys_controller_state.state_flag = error;
        }
        else {
            undervoltage = false;
        }


        if (drvSys_controller_state.state_flag == error) {

            drvSys_stop_controllers();

            Serial.println("DRVSYS_ERROR: Stopped Controllers because of Drive System Error.");
            set_leds(255, 0, 0, true, 100);

            prev_error = true;
        }

        if (prev_error && !error_sig && !undervoltage && !encoder_diff_error) {
            count_recover_iterations++;
            if (count_recover_iterations > 5) {
                drvSys_controller_state.state_flag = ready;

                Serial.println("DRVSYS_INFO: System recovered from an Error. ");
                set_leds(0, 0, 255, true, 500);
                prev_error = false;

                drvSys_foc_controller.setup_driver();
                drvSys_start_foc_processing();
                count_recover_iterations = 0;

                drvSys_start_motion_control();
            }

        }


        vTaskDelay(monitoring_delay);
    }
}

void drvSys_set_torque_boost_active(bool active) {

    static bool state_active = true;

    //activate after deactivation
    if (active && !state_active) {
        drvSys_foc_controller.set_max_current(DRVSYS_PHASE_CURRENT_MAX_mA, DRVSYS_TORQUE_LIMIT);
        drvSys_parameter_config.max_current_mA = DRVSYS_PHASE_CURRENT_MAX_mA;
        drvSys_parameter_config.max_torque_Nm = DRVSYS_TORQUE_LIMIT;

        state_active = true;
    }

    //deactivate when already active
    else if (!active && state_active) {

        drvSys_foc_controller.set_max_current(DRVSYS_PHASE_CURRENT_NOMINAL_mA, DRVSYS_TORQUE_CONSTANT);
        drvSys_parameter_config.max_current_mA = DRVSYS_PHASE_CURRENT_NOMINAL_mA;
        drvSys_parameter_config.max_torque_Nm = DRVSYS_TORQUE_CONSTANT;
        state_active = false;
    }
}

float drvSys_get_delta_angle() {
    return drvSys_delta_angle;
}

bool drv_Sys_check_if_joint_encoder_is_calibrated() {
    drv_sys_preferences.begin(drvSys_encoder_offset, false);

    drvSys_joint_zero_set = drv_sys_preferences.getBool("j_set", false);


    if (drvSys_joint_zero_set) {
        Serial.println("DRVSYS_INFO: Joint Encoder calibration data is available.");

        drvSys_joint_encoder_offset = drv_sys_preferences.getInt("val", drvSys_joint_encoder_offset);

        drvSys_magnetic_joint_encoder.resetAbsolutZero();
        vTaskDelay(100);
        drvSys_magnetic_joint_encoder.setZeroPosition(drvSys_joint_encoder_offset);
    }
    else {
        Serial.println("DRVSYS_INFO: No Encoder Offset Data available.");
    }
    drv_sys_preferences.end();

    return drvSys_joint_zero_set;

};

void drvSys_save_encoder_offsets_to_Flash() {

    drv_sys_preferences.begin(drvSys_encoder_offset, false);
    drv_sys_preferences.putBool("j_set", true);
    drv_sys_preferences.putInt("val", drvSys_joint_encoder_offset);
    drv_sys_preferences.end();

    Serial.println("DRVSYS_INFO: Saved Encoder Offsets.");

};

void drvSys_reset_encoder_offset_data_on_Flash() {

    drv_sys_preferences.begin(drvSys_encoder_offset, false);
    drv_sys_preferences.putBool("j_set", false);
    drv_sys_preferences.putInt("val", 0);
    drv_sys_preferences.end();

    Serial.println("DRVSYS_INFO: Removed Encoder Offsets.");


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



void drvSys_set_PID_gains(bool pos, const float Kp, const float Ki, const  float Kd, bool save) {

    if (pos == true) {
        // Set Position PID settings
        // Write them to overall Drive System parameters
        drvSys_parameter_config.pos_pid_gains.K_p = Kp;
        drvSys_parameter_config.pos_pid_gains.K_i = Ki;
        drvSys_parameter_config.pos_pid_gains.K_d = Kd;

        // Write them to controller instance
        drvSys_position_controller.setTuning(Kp, Ki, Kd);


    }
    // set velocity gains
    if (pos == false) {
        drvSys_parameter_config.vel_pid_gains.K_p = Kp;
        drvSys_parameter_config.vel_pid_gains.K_i = Ki;
        drvSys_parameter_config.vel_pid_gains.K_d = Kd;

        // Write them to controller instance
        drvSys_velocity_controller.setTuning(Kp, Ki, Kd);
    }





    if (save) { //save them to Flash;
        drvSys_save_PID_gains();
    }


};

void drvSys_adv_PID_settings(bool pos, int type, float value) {

    if (pos == true) {

        if (type == 0) {

            if (value != 0) {
                drvSys_position_controller.setInputFilter(true, value);
            }
            else {
                drvSys_position_controller.setInputFilter(false, 1.0);
            }

        }
        if (type == 1) {

            drvSys_position_controller.setErrorDeadBand(value);
        }
    }
    if (pos == false) {

        if (type == 0) {

            if (value != 0) {
                drvSys_velocity_controller.setInputFilter(true, value);
            }
            else {
                drvSys_velocity_controller.setInputFilter(false, 1.0);
            }

        }
        if (type == 1) {

            drvSys_velocity_controller.setErrorDeadBand(value);
        }
    }

}

void drvSys_save_PID_gains() {

    drv_sys_preferences.begin(drvSys_PID_saved_gains, false);

    drv_sys_preferences.putBool("PID_Data", true);
    drv_sys_preferences.putFloat("pP", drvSys_parameter_config.pos_pid_gains.K_p);
    drv_sys_preferences.putFloat("pI", drvSys_parameter_config.pos_pid_gains.K_i);
    drv_sys_preferences.putFloat("pD", drvSys_parameter_config.pos_pid_gains.K_d);
    drv_sys_preferences.putFloat("vP", drvSys_parameter_config.vel_pid_gains.K_p);
    drv_sys_preferences.putFloat("vI", drvSys_parameter_config.vel_pid_gains.K_i);
    drv_sys_preferences.putFloat("vD", drvSys_parameter_config.vel_pid_gains.K_d);

    drv_sys_preferences.end();
}

void drvSys_remove_PID_gains_from_flash() {

    drv_sys_preferences.begin(drvSys_PID_saved_gains, false);

    drv_sys_preferences.putBool("PID_Data", false);

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

bool _drvSys_read_PID_gains_from_flash() {
    /* Read and set position PID Settings */
    drv_sys_preferences.begin(drvSys_PID_saved_gains, false);

    bool pid_gains_available = drv_sys_preferences.getBool("PID_Data", false);

    if (pid_gains_available) {

        float K_pos_P = drv_sys_preferences.getFloat("pP", drvSys_parameter_config.pos_pid_gains.K_p);
        float K_pos_I = drv_sys_preferences.getFloat("pI", drvSys_parameter_config.pos_pid_gains.K_i);
        float K_pos_D = drv_sys_preferences.getFloat("pD", drvSys_parameter_config.pos_pid_gains.K_d);

        drvSys_set_PID_gains(true, K_pos_P, K_pos_I, K_pos_D, false);

        Serial.println("DRVSYS_INFO: Read Position PID Gains from Flash.");
        Serial.println("DRVSYS_INFO: P = " + String(K_pos_P) + ", I = " + String(K_pos_I)
            + ", D = " + String(K_pos_D));

        float K_vel_P = drv_sys_preferences.getFloat("vP", drvSys_parameter_config.pos_pid_gains.K_p);
        float K_vel_I = drv_sys_preferences.getFloat("vI", drvSys_parameter_config.pos_pid_gains.K_i);
        float K_vel_D = drv_sys_preferences.getFloat("vD", drvSys_parameter_config.pos_pid_gains.K_d);



        drvSys_set_PID_gains(false, K_vel_P, K_vel_I, K_vel_D, false);

        Serial.println("DRVSYS_INFO: Read Velocity PID Gains from Flash.");
        Serial.println("DRVSYS_INFO: P = " + String(K_vel_P) + ", I = " + String(K_vel_I)
            + ", D = " + String(K_vel_D));
    }

    else {
        Serial.println("DRVSYS_INFO: No PID Gains available on Flash.");
    }
    drv_sys_preferences.end();

    return pid_gains_available;


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


    vTaskSuspend(drvSys_torque_controller_th);
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
        vTaskDelay(300 / portTICK_PERIOD_MS);

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

        vTaskDelay(300 / portTICK_PERIOD_MS);

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

    vTaskResume(drvSys_torque_controller_th);


    vTaskDelay(2000 / portTICK_PERIOD_MS);

}







