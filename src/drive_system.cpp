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
volatile const  int32_t drvSys_timer_vel_control_ticks = DRVSYS_CONTROL_VEL_PERIOD_US / drvSys_timer_alarm_rate_us;
volatile const int32_t drvSys_timer_pos_control_ticks = DRVSYS_CONTROL_POS_PERIOD_US / drvSys_timer_alarm_rate_us;


/*###########################################################################
################### Drive System Object Definitions #########################
############################################################################*/

/* Drive System General Variables */

/* Drive Constants */

drvSys_Constants drvSys_constants = { .nominal_current_mA = DRVSYS_PHASE_CURRENT_NOMINAL_mA,
    .transmission_ratio = DRVSYS_TRANSMISSION_RATIO,
    .joint_id = JOINT_ID,
    .motor_torque_constant = DRVSYS_TORQUE_CONSTANT };



/* Drive System Control Mode */

drvSys_controlMode drvSys_mode = dual_control;

drvSys_StateFlag drvSys_state_flag = not_ready;


/* Drive State Parameters */
drvSys_controllerState drvSys_controller_state = { .control_mode = drvSys_mode,
.state_flag = drvSys_state_flag,
.calibrated = false,
.overtemperature = false,
.temperature_warning = false,
.temperature = 20 };


struct {
    int target_type; //0 - pos, 1 - vel, 2 - torque
    float max;
    float min;
    int shape; // 0 - rectangle, 1 - sine, 2 - ramp
    float period;
    bool active;
}drvSys_test_signal;


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


// Axis alignment Variables

bool drvSys_flip_global_alignment = DRVSYS_GLOB_DIR_FLIP;

float drvSys_torque_dir_align = DRVSYS_TORQUE_ALIGN_DIR;
float drvSys_motor_encoder_dir_align = DRVSYS_MOTOR_ENC_ALIGN_DIR;
float drvSys_joint_encoder_dir_align = DRVSYS_JOINT_ENC_ALIGN_DIR;
float drvSys_torque_sensor_dir_align = DRVSYS_TORQUE_ALIGN_DIR;

bool drvSys_axis_aligned_flag = false;


// Axis Calibration Variables

int32_t drvSys_raw_joint_encoder_offset = DRVSYS_RAW_JOINT_ENC_OFFSET;
int32_t drvSys_raw_motor_encoder_offset = DRVSYS_RAW_MOTOR_ENC_OFFSET;
bool drvSys_joint_calibrated_flag = false;

bool drvSys_software_end_stops_active = false;
bool hit_positive_limit = false;
bool hit_negative_limit = false;


/* Differentiators */
Differentiator drvSys_differentiator_motor_pos(DRVSYS_PROCESS_ENCODERS_FREQU);

/* ########## Controllers ################ */
PIDController drvSys_velocity_controller(0.00, 0.0000, 0);
PIDController drvSys_position_controller(0, 0, 0);


float drvSys_motor_position;
float drvSys_motor_velocity;
float drvSys_motor_acc;

float drvSys_joint_position;
float drvSys_joint_velocity;
float drvSys_joint_acc;

float drvSys_joint_torque;

int32_t drvSys_hall_sensor_val;
float drvSys_motor_temperature = 20.0;


/* Notch Filters */

drvSys_notch_filter_params drvSys_notch_filters;

IIRFilter<2> notch_filter_0;
IIRFilter<2> notch_filter_1;

//define Task Handlers
TaskHandle_t drvSys_foc_th;
TaskHandle_t drvSys_process_encoders_th;
TaskHandle_t drvSys_torque_controller_th;
TaskHandle_t drvSys_vel_controller_th;
TaskHandle_t drvSys_pos_controller_th;
TaskHandle_t drvSys_process_joint_encoder_th;
TaskHandle_t drvSys_process_torque_sensor_th;
TaskHandle_t drvSys_admittance_controller_th;
TaskHandle_t drvSys_test_signal_th;


/* #############################################################
######################## Target values #########################
###############################################################*/
float drvSys_pos_target = 0;
float drvSys_vel_target = 0;
float drvSys_torque_target = 0;
float drvSys_m_torque_commanded = 0;

// used to differentiate between torque_ff and reference torque for admittance control
float drvSys_joint_torque_ref = 0.0;


/*######## Feed Forward targets #############*/
float drvSys_vel_ff = 0.0;
float drvSys_torque_ff = 0.0;


/* ####### Drive System Preferences ########### */

//used to save parameters into flash memory
Preferences drv_sys_preferences;
//Namespaces for Parameters
// PID
const char* drvSys_posPID_saved_gains = "posGains";
const char* drvSys_velPID_saved_gains = "velGains";
const char* drvSys_admittance_saved_gains = "admGains";
// Offset Calibration
const char* drvSys_encoder_offset = "posOffset";
// Alignment
const char* drvSys_alignment_setting = "align";

//Estimated Parameters
const char* drvSys_dynamic_params = "dynParams";


/* Drive System Priority constants */
enum drvSys_priorities {
    foc_prio = 10, process_sensor_prio = 9, torque_control_prio = 9,
    vel_control_prio = 8, pos_control_prio = 7, admittance_control_prio = 6
};

/* ###############################################################
##################################################################
################## Function Implementations ######################
##################################################################
################################################################ */



drvSys_driveState drvSys_get_drive_state() {
    drvSys_driveState state;
    xSemaphoreTake(drvSys_mutex_motor_position, portMAX_DELAY);
    state.joint_pos = drvSys_motor_position;
    xSemaphoreGive(drvSys_mutex_motor_position);

    xSemaphoreTake(drvSys_mutex_motor_vel, portMAX_DELAY);
    state.joint_vel = drvSys_motor_velocity;
    xSemaphoreGive(drvSys_mutex_motor_vel);

    xSemaphoreTake(drvSys_mutex_motor_acc, portMAX_DELAY);
    state.joint_acc = drvSys_motor_acc;
    xSemaphoreGive(drvSys_mutex_motor_acc);

    xSemaphoreTake(drvSys_mutex_joint_torque, portMAX_DELAY);
    state.joint_torque = drvSys_joint_torque;
    xSemaphoreGive(drvSys_mutex_joint_torque);

    state.motor_torque = drvSys_m_torque_commanded;


    return state;
};

drvSys_extendedDriveState drvSys_get_extended_drive_state() {
    drvSys_extendedDriveState state;

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

    state.motor_torque = drvSys_m_torque_commanded;


    return state;

};

drvSys_parameters drvSys_get_parameters() {
    return drvSys_parameter_config;
};

drvSys_controllerState drvSys_get_controllerState() {
    return drvSys_controller_state;
};


drvSys_Constants drvSys_get_constants() {
    return drvSys_constants;
}

void _drvSys_calibrate_with_hallsensor() {


    Serial.println("DRVSYS_INFO: Start Calibration via Hall sensor");
    pinMode(HALL_SENSOR_PIN, INPUT);

    int32_t shift = DRVSYS_ANGLE_ORIGIN_2_HALL_DEG * (16383.0 / 360.0);

    drvSys_hall_sensor_val = analogRead(HALL_SENSOR_PIN);

    pinMode(TMC_STEP, OUTPUT);
    pinMode(TMC_DIR, OUTPUT);

    digitalWrite(TMC_DIR, LOW);

    drvSys_foc_controller.driver->direct_mode(false);

    float average_hall_value = drvSys_hall_sensor_val;
    float largest_hall_value = 0;
    bool calibration_finished = false;

    int32_t raw_joint_angle = 0;
    int32_t raw_motor_angle = 0;

    int n_falling = 0;

    float limit_angle = 90.0;


    const int buffer_size = 50;
    CircularBuffer<float, buffer_size> buffer;
    CircularBuffer<float, buffer_size> buffer_noise;

    bool found_peak = false;
    int stepcount = 10000;
    float prev_hall_val = analogRead(HALL_SENSOR_PIN);

    float start_angle = drvSys_magnetic_joint_encoder.getRotationDeg();


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
        Serial.print(', ');
        Serial.print(" Average Hall value: ");
        Serial.print(average_hall_value);
        Serial.print(', ');
        Serial.print(" Average Hall noise value: ");
        Serial.println(average_noise_val);

        if (current_average_val > average_hall_value + average_noise_val * 10) {
            if (current_average_val > largest_hall_value) {
                raw_joint_angle = drvSys_magnetic_joint_encoder.getRotation(true) + shift;
                raw_motor_angle = drvSys_magnetic_motor_encoder.getRotation(true) + shift;

                largest_hall_value = current_average_val;

                Serial.print(" New Hall maximum at: ");
                Serial.println(raw_joint_angle);

                found_peak = true;
                //reduce step coint 
                stepcount = 1000;
            }
        }

        //Change direction when no peaks found after 90°
        if (abs(start_angle - drvSys_magnetic_joint_encoder.getRotationDeg()) >= limit_angle && found_peak == false) {
            digitalWrite(TMC_DIR, HIGH);
        }


        //detect falling hall values
        if ((current_average_val + average_noise_val - largest_hall_value) < 0 && found_peak) {
            n_falling++;
        };


        // Finish Calibration 
        if (n_falling > 100) {
            calibration_finished = true;

            drvSys_magnetic_joint_encoder.setOffsetAngle(raw_joint_angle);
            drvSys_magnetic_motor_encoder.setOffsetAngle(raw_motor_angle);

            drvSys_raw_joint_encoder_offset = raw_joint_angle;
            drvSys_raw_motor_encoder_offset = raw_motor_angle;

            Serial.println("DRVSYS_INFO: Joint Angle Offset Calibration finished");

            drvSys_joint_calibrated_flag = true;

        }

        // do a step
        for (int i = 0; i < stepcount; i++) {
            digitalWrite(TMC_STEP, HIGH);
            delayMicroseconds(10);
            digitalWrite(TMC_STEP, LOW);
            delayMicroseconds(10);
        }


    }


    //set back controller in FOC mode
    drvSys_foc_controller.driver->direct_mode(true);

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
    drvSys_set_target_torque(0.0);

    const TickType_t move_delay = 500 / portTICK_PERIOD_MS;

    vTaskDelay(move_delay);

    drvSys_set_target_torque(0.0);

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
    _drvSys_read_vel_PID_gains_from_flash();
    _drvSys_read_admittanceGains_from_flash();


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
    drvSys_PID_Gains pid_gains_pos = { .K_p = 0.0, .K_i = 0.0, .K_d = 0.0 };
    drvSys_PID_Gains pid_gains_vel = { .K_p = 0.018, .K_i = 0.000, .K_d = 0.0 };

    drvSys_admittance_parameters drvSys_admittance_gains;
    drvSys_admittance_gains.virtual_spring = 0.0;
    drvSys_admittance_gains.virtual_damping = 0.0;
    drvSys_admittance_gains.virtual_inertia = 0.0;


    /* Set up Drive System Parameters */
    drvSys_parameter_config.max_current_mA = DRVSYS_PHASE_CURRENT_MAX_mA;
    drvSys_parameter_config.max_vel = DRVSYS_VEL_MAX;
    drvSys_parameter_config.max_torque_Nm = DRVSYS_TORQUE_LIMIT;
    drvSys_parameter_config.vel_pid_gains = pid_gains_vel;
    drvSys_parameter_config.pos_pid_gains = pid_gains_pos;
    drvSys_parameter_config.admittance_gains = drvSys_admittance_gains;
    drvSys_parameter_config.limit_high_deg = DRVSYS_POS_LIMIT_HIGH;
    drvSys_parameter_config.limit_low_deg = DRVSYS_POS_LIMIT_LOW;


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


    joint_kinematic_kalman_filter.setAccelChange(0.01);
    motor_kinematic_kalman_filter.setAccelChange(4.0);

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

        drvSys_software_end_stops_active = true;

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
        2000,      // Stack size (bytes)
        NULL,      // task parameters
        foc_prio,         // task priority
        &drvSys_foc_th,
        0 // task handle
    );

    xTaskCreatePinnedToCore(
        _drvSys_process_encoders_task,   // function name
        "Process_encoder_task", // task name
        5000,      // Stack size (bytes)
        NULL,      // task parameters
        process_sensor_prio,         // task priority
        &drvSys_process_encoders_th,
        0 // task handle
    );

    xTaskCreatePinnedToCore(_drvSys_torque_controller_task,
        "Torque_Controller_Task",
        2000,
        NULL,
        torque_control_prio,
        &drvSys_torque_controller_th,
        0
    );


    /* ---- create closed loop controller tasks, are not called until closed loop control is

/* Create all the Static Controller Tasks */
    xTaskCreatePinnedToCore(
        _drvSys_velocity_controller_task,   // function name
        "Velocity_Controller_Task", // task name
        1000,      // Stack size (bytes)
        NULL,      // task parameters
        vel_control_prio,         // task priority
        &drvSys_vel_controller_th,
        1 // task handle
    );

    xTaskCreatePinnedToCore(_drvSys_position_controller_task,
        "Position_Controller_Task",
        1000,
        NULL,
        pos_control_prio,
        &drvSys_pos_controller_th,
        1
    );


    /* --- create Task --- */
    xTaskCreatePinnedToCore(
        _drvSys_admittance_controller_task,   // function name
        "Admittance_Controller_Task", // task name
        3000,      // Stack size (bytes)
        NULL,      // task parameters
        admittance_control_prio,         // task priority
        &drvSys_admittance_controller_th,
        0 // task handle
    );

    //suspend task until controller is started
    vTaskSuspend(drvSys_admittance_controller_th);

    //if initialization was succesful:
    drvSys_state_flag = ready;


};

int32_t  drvSys_start_foc_processing() {

    if (drvSys_state_flag == not_ready) {
        Serial.println("DRVSYS_INFO: Can not start FOC Processing, Drive system is not ready.");
        return -1;

    }

    Serial.println("DRVSYS_INFO: Start FOC Processing");


    // Start Timer -> Starts Processing!
    Serial.println("DRVSYS_INFO: Start Interrupt Timer");
    timerAlarmEnable(drvSys_foc_timer);

    drvSys_foc_controller.set_target_torque(0.2);


    vTaskDelay(100 / portTICK_PERIOD_MS);
    /* ------ Calibration Routines that require running FOC Control ------- */
#ifdef ALLOW_ELECTRIC_ANGLE_CALIBRATION
    drvSys_calibrate_FOC();
#endif 


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

        case impedance_control:
            //_drvSys_setup_impedance_controller();
            break;

        case hybrid_control:
            //_drvSys_setup_hybrid_controoller();
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
    /*Velocity Controller*/
    drvSys_velocity_controller.setSampleTime(DRVSYS_CONTROL_VEL_PERIOD_US);
    // Torque Limitation

    float torque_limit = drvSys_parameter_config.max_torque_Nm;
    drvSys_velocity_controller.setOutputLimits(torque_limit * (-1.0), torque_limit);
    drvSys_velocity_controller.setpoint = 0.0;


    drvSys_velocity_controller.Initialize();
    drvSys_velocity_controller.setMode(PID_MODE_INACTIVE);
    drvSys_velocity_controller.SetControllerDirection(PID_DIR_DIRECT);
    drvSys_velocity_controller.setErrorDeadBand(2.0);
    drvSys_velocity_controller.setOutputFilter(true, 0.1);

    _drvSys_read_vel_PID_gains_from_flash();

    drvSys_vel_ff = 0.0;


    /* Position Controller */
    drvSys_position_controller.setSampleTime(DRVSYS_CONTROL_POS_PERIOD_US);
    //Velocity Limitation
    float max_velocity = drvSys_parameter_config.max_vel;
    drvSys_position_controller.setOutputLimits(max_velocity * (-1.0), max_velocity);
    drvSys_position_controller.setpoint = 0.0;

    drvSys_position_controller.Initialize();
    drvSys_position_controller.setMode(PID_MODE_INACTIVE);
    drvSys_position_controller.SetControllerDirection(PID_DIR_DIRECT);

    _drvSys_read_pos_PID_gains_from_flash();

    Serial.println("DRVSYS_INFO: Setup Cascade Position and Velocity Controller");

    drvSys_position_controller.setMode(PID_MODE_ACTIVE);
    drvSys_velocity_controller.setMode(PID_MODE_ACTIVE);

    drvSys_torque_ff = 0.0;
    drvSys_vel_ff = 0.0;

};

void drvSys_set_advanced_PID_settings(int type, float filter_alpha, float deadzone, bool active) {

    if (type == 0) {
        if (filter_alpha <= 0) {
            drvSys_position_controller.setOutputFilter(false);
        }
        if (filter_alpha < 1.0) {
            drvSys_position_controller.setOutputFilter(true, filter_alpha);
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
    else if (type == 0) {
        if (filter_alpha <= 0) {
            drvSys_velocity_controller.setOutputFilter(false);
        }
        if (filter_alpha < 1.0) {
            drvSys_velocity_controller.setOutputFilter(true, filter_alpha);
        }

        if (deadzone <= 0) {
            drvSys_velocity_controller.setErrorDeadBand(0.0);
        }
        else {
            drvSys_velocity_controller.setErrorDeadBand(deadzone);
        }

        if (active) {
            drvSys_velocity_controller.setMode(PID_MODE_ACTIVE);
        }
        else {
            drvSys_velocity_controller.setMode(PID_MODE_ACTIVE);
        }
    }
}


void drvSys_stop_controllers() {

    drvSys_position_controller.setMode(PID_MODE_INACTIVE);
    drvSys_velocity_controller.setMode(PID_MODE_INACTIVE);

    drvSys_controller_state.state_flag = closed_loop_control_inactive;

    vTaskSuspend(drvSys_admittance_controller_th);

    drvSys_set_target_motor_torque(0.0);

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

    if (drvSys_joint_calibrated_flag && drvSys_axis_aligned_flag && drvSys_software_end_stops_active) {
        xSemaphoreTake(drvSys_mutex_joint_position, portMAX_DELAY);
        float pos_deg = drvSys_joint_position;
        xSemaphoreGive(drvSys_mutex_joint_position);

        if (pos_deg >= drvSys_parameter_config.limit_high_deg) {
            if (input > 0.0) {
                hit_positive_limit = true;
                hit_negative_limit = false;
                return 0.0;
            }
        }
        if (pos_deg <= drvSys_parameter_config.limit_low_deg) {
            if (input < 0.0) {
                hit_negative_limit = true;
                hit_positive_limit = false;
                return 0.0;
            }
        }
        hit_negative_limit = false;
        hit_positive_limit = false;
    }

    return input;

}

void drvSys_set_target_motor_torque(float target_torque) {

    drvSys_foc_controller.set_target_torque(_drvSys_check_joint_limit(target_torque));
};



void drvSys_set_target_torque(float torque) {

    float torque_limit = drvSys_parameter_config.max_torque_Nm;

    xSemaphoreTake(drvSys_mutex_torque_target, portMAX_DELAY);

    if (drvSys_torque_target > torque_limit) {
        drvSys_torque_target = torque_limit;
    }
    else if (drvSys_torque_target < (-1.0) * torque_limit) {
        drvSys_torque_target = (-1.0) * torque_limit;
    }
    drvSys_torque_target = torque;
    xSemaphoreGive(drvSys_mutex_torque_target);

};

void drvSys_set_feed_forward_torque(float torque_ff) {
    drvSys_torque_ff = torque_ff;

};

void drvSys_set_feed_forward_velocity(float vel_ff) {
    drvSys_vel_ff = vel_ff;
}

void drvSys_set_target_velocity(float vel) {


    if (vel > drvSys_parameter_config.max_vel) {
        vel = drvSys_parameter_config.max_vel;
    }
    else if (vel < (-1.0) * drvSys_parameter_config.max_vel) {
        vel = (-1.0) * drvSys_parameter_config.max_vel;
    }
    drvSys_vel_target = vel;

};


void drvSys_set_target_pos(float pos) {

    if (pos > drvSys_parameter_config.limit_high_deg) {
        pos = drvSys_parameter_config.limit_high_deg;
    }
    else if (pos < drvSys_parameter_config.limit_low_deg) {
        pos = drvSys_parameter_config.limit_low_deg;
    }
    drvSys_position_controller.setSetPoint(pos);

};

drvSys_driveTargets drvSys_get_targets() {
    drvSys_driveTargets targets;

    targets.motor_torque_target = drvSys_foc_controller.target_torque;
    targets.pos_target = drvSys_position_controller.setpoint;
    targets.vel_target = drvSys_velocity_controller.setpoint;
    targets.ref_torque = drvSys_joint_torque_ref;

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


        if (tickCount % drvSys_timer_vel_control_ticks == 0) {
            vTaskNotifyGiveFromISR(drvSys_vel_controller_th, &xHigherPriorityTaskWoken);
        }
        if (tickCount % drvSys_timer_pos_control_ticks == 0) {
            vTaskNotifyGiveFromISR(drvSys_pos_controller_th, &xHigherPriorityTaskWoken);
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

            static const float encoder2Deg = 180.0 / 8192.0;


            float motor_pos_sensor_val = drvSys_motor_encoder_dir_align * drvSys_magnetic_motor_encoder.last_sample * encoder2Deg;
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
            float raw_joint_angle_deg = drvSys_joint_encoder_dir_align * drvSys_magnetic_joint_encoder.getRotationDeg(true);
            xSemaphoreGive(glob_SPI_mutex);

            KinematicStateVector joint_state = joint_kinematic_kalman_filter.estimateStates(raw_joint_angle_deg);

            xSemaphoreTake(drvSys_mutex_joint_acc, portMAX_DELAY);
            drvSys_joint_acc = joint_state.acc;
            xSemaphoreGive(drvSys_mutex_joint_acc);

            xSemaphoreTake(drvSys_mutex_joint_vel, portMAX_DELAY);
            drvSys_joint_velocity = joint_state.vel;
            xSemaphoreGive(drvSys_mutex_joint_vel);

            xSemaphoreTake(drvSys_mutex_joint_position, portMAX_DELAY);
            drvSys_joint_position = joint_state.pos;
            xSemaphoreGive(drvSys_mutex_joint_position);


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

void _drvSys_velocity_controller_task(void* parameters) {

    uint32_t velocity_controller_processing_thread_notification;

    static float inv_transmission = 1.0 / double(drvSys_constants.transmission_ratio);

    while (true) {

        velocity_controller_processing_thread_notification = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (velocity_controller_processing_thread_notification) {
            if (drvSys_mode == dual_control || drvSys_mode == admittance_control) {
                if (drvSys_velocity_controller.getMode() == PID_MODE_ACTIVE) {
                    xSemaphoreTake(drvSys_mutex_motor_vel, portMAX_DELAY);
                    float actual_vel = drvSys_motor_velocity;
                    xSemaphoreGive(drvSys_mutex_motor_vel);

                    drvSys_velocity_controller.setSetPoint(drvSys_vel_target, true);

                    drvSys_velocity_controller.input = actual_vel * inv_transmission;
                    drvSys_velocity_controller.compute();

                    /* Handle Velocity Controller Output */

                    drvSys_set_target_torque(drvSys_velocity_controller.output);


                }
            }
        }
    }
}

void _drvSys_position_controller_task(void* parameters) {
    uint32_t position_controller_processing_thread_notification;

    while (true) {

        position_controller_processing_thread_notification = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if (position_controller_processing_thread_notification) {

            if (drvSys_mode == dual_control || drvSys_mode == admittance_control) {
                xSemaphoreTake(drvSys_mutex_motor_position, portMAX_DELAY);
                float actual_pos = drvSys_motor_position;
                xSemaphoreGive(drvSys_mutex_motor_position);

                drvSys_position_controller.input = actual_pos;
                drvSys_position_controller.compute();

                /* Handle controller output */

                float velocity_target = drvSys_position_controller.output + drvSys_vel_ff;

                drvSys_set_target_velocity(velocity_target);


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

            xSemaphoreTake(drvSys_mutex_motor_commanded_torque, portMAX_DELAY);
            drvSys_m_torque_commanded = drvSys_torque_target + drvSys_torque_ff;

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

            drvSys_foc_controller.set_target_torque(drvSys_m_torque_commanded);
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

        float desired_velocity = drvSys_vel_ff;
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

        drvSys_set_feed_forward_velocity(vel_admittance_target);
        drvSys_set_target_pos(pos_admittance_target);

        vTaskDelay(admittance_delay);
    }

};


void _drvSys_process_peripheral_sensors_task() {

    drvSys_hall_sensor_val = analogRead(HALL_SENSOR_PIN);

    // handle Temperature Sensor if Available

#ifdef DRVSYS_MOTOR_TEMP_SENSOR_AVAILABLE
//TODO
#endif
}

bool drvSys_read_encoder_offsets_from_Flash() {
    drv_sys_preferences.begin(drvSys_encoder_offset, false);

    bool data_available = drv_sys_preferences.getBool("off_avail", false);

    if (data_available) {
        drvSys_raw_joint_encoder_offset = drv_sys_preferences.getInt("joint", 0);
        drvSys_raw_motor_encoder_offset = drv_sys_preferences.getInt("motor", 0);

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
    drv_sys_preferences.putInt("joint", drvSys_raw_joint_encoder_offset);
    drv_sys_preferences.putInt("motor", drvSys_raw_motor_encoder_offset);
    drv_sys_preferences.end();

    Serial.println("DRVSYS_INFO: Saved Encoder Offsets to Flash.");

};

void drvSys_reset_encoder_offset_data_on_Flash() {

    drv_sys_preferences.begin(drvSys_encoder_offset, false);
    drv_sys_preferences.putBool("off_avail", false);
    drv_sys_preferences.putInt("joint", 0);
    drv_sys_preferences.putInt("motor", 0);
    drv_sys_preferences.end();

    Serial.println("DRVSYS_INFO: Removed Encoder Offsets to Flash.");


}

bool drvSys_read_alignment_from_Flash() {

    drv_sys_preferences.begin(drvSys_alignment_setting, false);

    bool saved_data_available = drv_sys_preferences.getBool("ax_avail", false);

    if (saved_data_available) {
        drvSys_flip_global_alignment = drv_sys_preferences.getBool("glob_flip", false);
        drvSys_flip_global_alignment = drv_sys_preferences.getFloat("tMotDir", 1.0);
        drvSys_joint_encoder_dir_align = drv_sys_preferences.getFloat("jEncDir", 1.0);
        drvSys_motor_encoder_dir_align = drv_sys_preferences.getFloat("mEncDir", 1.0);
        drvSys_torque_dir_align = drv_sys_preferences.getFloat("SensDir", 1.0);

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
    drvSys_parameter_config.pos_pid_gains.K_p = Kp;
    drvSys_parameter_config.pos_pid_gains.K_i = Ki;
    drvSys_parameter_config.pos_pid_gains.K_d = Kd;

    // Write them to controller instance
    drvSys_position_controller.setTuning(Kp, Ki, Kd);

    if (save) { //save them to Flash;
        drvSys_save_pos_PID_gains();
    }
};

void drvSys_save_pos_PID_gains() {

    float* gains = drvSys_position_controller.getGains();

    drv_sys_preferences.begin(drvSys_posPID_saved_gains, false);

    drv_sys_preferences.putBool("posGainAvail", true);
    drv_sys_preferences.putFloat("P", gains[0]);
    drv_sys_preferences.putFloat("I", gains[1]);
    drv_sys_preferences.putFloat("D", gains[2]);

    drv_sys_preferences.end();
}
void drvSys_set_vel_PID_gains(float Kp, float Ki, float Kd, bool save) {
    // Set Position PID settings

    // Write them to overall Drive System parameters
    drvSys_parameter_config.vel_pid_gains.K_p = Kp;
    drvSys_parameter_config.vel_pid_gains.K_i = Ki;
    drvSys_parameter_config.vel_pid_gains.K_d = Kd;

    // Write them to controller instance
    drvSys_velocity_controller.setTuning(Kp, Ki, Kd);

    if (save) { // save them to the Flash
        drvSys_save_vel_PID_gains();
    }
};

void drvSys_save_vel_PID_gains() {

    float* gains = drvSys_velocity_controller.getGains();

    drv_sys_preferences.begin(drvSys_velPID_saved_gains, false);

    drv_sys_preferences.putBool("velGainAvail", true);
    drv_sys_preferences.putFloat("P", gains[0]);
    drv_sys_preferences.putFloat("I", gains[1]);
    drv_sys_preferences.putFloat("D", gains[2]);

    drv_sys_preferences.end();
};

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

bool _drvSys_read_pos_PID_gains_from_flash() {
    /* Rea dand set position PID Settings */
    drv_sys_preferences.begin(drvSys_posPID_saved_gains, false);

    bool pos_pid_gains_available = drv_sys_preferences.getBool("posGainAvail", false);

    if (pos_pid_gains_available) {

        float K_pos_P = drv_sys_preferences.getFloat("P", drvSys_parameter_config.pos_pid_gains.K_p);
        float K_pos_I = drv_sys_preferences.getFloat("I", drvSys_parameter_config.pos_pid_gains.K_i);
        float K_pos_D = drv_sys_preferences.getFloat("D", drvSys_parameter_config.pos_pid_gains.K_d);


        // Write them to overall Drive System parameters
        drvSys_parameter_config.pos_pid_gains.K_p = K_pos_P;
        drvSys_parameter_config.pos_pid_gains.K_i = K_pos_I;
        drvSys_parameter_config.pos_pid_gains.K_d = K_pos_D;

        // Write them to controller instance
        drvSys_position_controller.setTuning(K_pos_P, K_pos_I, K_pos_D);

        Serial.println("DRVSYS_INFO: Read Position PID Gains from Flash.");
        Serial.println("DRVSYS_INFO: P = " + String(K_pos_P) + ", I = " + String(K_pos_I) + ", D = " + String(K_pos_D));
    }
    else {
        Serial.println("DRVSYS_INFO: No Position PID Gains available on Flash.");
    }
    drv_sys_preferences.end();

    return pos_pid_gains_available;


}
bool _drvSys_read_vel_PID_gains_from_flash() {

    /* Read and set velocity PID Settings */
    drv_sys_preferences.begin(drvSys_velPID_saved_gains, false);

    bool vel_pid_gains_available = drv_sys_preferences.getBool("velGainAvail", false);

    if (vel_pid_gains_available) {
        float K_P = drv_sys_preferences.getFloat("P", drvSys_parameter_config.vel_pid_gains.K_p);
        float K_I = drv_sys_preferences.getFloat("I", drvSys_parameter_config.vel_pid_gains.K_i);
        float K_D = drv_sys_preferences.getFloat("D", drvSys_parameter_config.vel_pid_gains.K_d);

        // Write them to overall Drive System parameters
        drvSys_parameter_config.vel_pid_gains.K_p = K_P;
        drvSys_parameter_config.vel_pid_gains.K_i = K_I;
        drvSys_parameter_config.vel_pid_gains.K_d = K_D;

        // Write them to controller instance
        drvSys_velocity_controller.setTuning(K_P, K_I, K_D);

        Serial.println("DRVSYS_INFO: Read Velo PID Gains from Flash.");
        Serial.println("DRVSYS_INFO: P = " + String(K_P) + ", I = " + String(K_I) + ", D = " + String(K_D));

    }
    else {
        Serial.println("DRVSYS_INFO: No Velocity PID Gains available on Flash.");
    }

    drv_sys_preferences.end();

    return vel_pid_gains_available;

};
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



void _drvSys_test_signal_task(void* parameters) {

    const TickType_t test_signal_delay = 10 / portTICK_PERIOD_MS; // 100Hz

    static int tick_counter = 0;
    static bool rectangle_toggle = false;
    static float output_sig = 0.0;
    static float sig_time = 0.0;
    const float delta_t = 0.01;


    while (drvSys_test_signal.active) {
        int ticks_half_period = (drvSys_test_signal.period / 2) / 10;

        //rectangle
        if (drvSys_test_signal.shape == 0) {


            if (tick_counter % ticks_half_period == 0) {
                rectangle_toggle = !rectangle_toggle;

                if (rectangle_toggle) {
                    output_sig = drvSys_test_signal.max;
                }
                else {
                    output_sig = drvSys_test_signal.min;
                }
            }
        }

        // triangle
        if (drvSys_test_signal.shape == 2) {
            float triangle_step_slope = (drvSys_test_signal.max - drvSys_test_signal.min) / float(ticks_half_period);

            if (tick_counter % ticks_half_period == 0) {
                rectangle_toggle = !rectangle_toggle;

                if (rectangle_toggle) {
                    triangle_step_slope = triangle_step_slope;
                }
                else {
                    triangle_step_slope = -triangle_step_slope;
                }
            }
            output_sig = output_sig + triangle_step_slope;

        }

        // sine wave
        if (drvSys_test_signal.shape == 1) {

            float frequ = 1 / drvSys_test_signal.period;

            float offset = drvSys_test_signal.max + drvSys_test_signal.min;
            float amplitude = drvSys_test_signal.max - offset;

            output_sig = offset + amplitude * sin(2 * PI * frequ * sig_time);

            sig_time = tick_counter * delta_t;

            tick_counter++;

            //output signal

            if (drvSys_test_signal.target_type == 0) {
                drvSys_set_target_pos(output_sig);
            };

            if (drvSys_test_signal.target_type == 1) {
                drvSys_set_target_velocity(output_sig);
            };
            if (drvSys_test_signal.target_type == 2) {
                drvSys_set_target_torque(output_sig);
            };


            vTaskDelay(test_signal_delay);

        }
    }

    vTaskDelete(drvSys_test_signal_th);

}


void drvSys_start_test_signal(int signal_target, int shape, float max, float min, float period) {

    drvSys_test_signal.target_type = signal_target;
    drvSys_test_signal.shape = shape;
    drvSys_test_signal.max = max;
    drvSys_test_signal.min = min;
    drvSys_test_signal.period = period;

    xTaskCreatePinnedToCore(
        _drvSys_test_signal_task,   // function name
        "output_test_signal_task", // task name
        1000,      // Stack size (bytes)
        NULL,      // task parameters
        2,         // task priority
        &drvSys_test_signal_th,
        1 // task handle
    );

    drvSys_test_signal.active = true;

    xSemaphoreTake(glob_Serial_mutex, portMAX_DELAY);
    Serial.println("DRYSYS_INFO: Started Test Signal.");
    xSemaphoreGive(glob_Serial_mutex);
};

void drvSys_stop_test_signal() {

    drvSys_test_signal.active = false;
    xSemaphoreTake(glob_Serial_mutex, portMAX_DELAY);
    Serial.println("DRYSYS_INFO: Stopped Test Signal.");
    xSemaphoreGive(glob_Serial_mutex);
}



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


void drvSys_start_debug_output() {

    xTaskCreatePinnedToCore(
        _drvSys_debug_print_position_task,   // function name
        "print_position_debug_task", // task name
        2000,      // Stack size (bytes)
        NULL,      // task parameters
        2,         // task priority
        NULL,
        1 // task handle
    );
}

void _drvSys_debug_print_position_task() {


}






