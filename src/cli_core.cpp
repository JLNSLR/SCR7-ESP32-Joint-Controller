#include <cli_core.h>
//#include <JCRTL_CLI_interface_functions.h>

char cli_line[LINE_BUF_SIZE];
char cli_args[MAX_NUM_ARGS][ARG_BUF_SIZE];

bool cli_active = true;

TaskHandle_t cli_task;

void cli_init() {

    Serial.println("Startet Command Line Interface (CLI).");

    memset(cli_line, 0, LINE_BUF_SIZE);
    memset(cli_args, 0, sizeof(cli_args[0][0]) * MAX_NUM_ARGS * ARG_BUF_SIZE);



    // Start Task

    xTaskCreate(
        _cli_task,   // function name
        "cli_task", // task name
        5000,      // Stack size (bytes)
        NULL,      // task parameters
        8,         // task priority
        &cli_task // task handle
    );

    cli_active = true;


}
void cli_read_line_cmd() {

    String line_string;

    if (Serial.available()) {
        line_string = Serial.readStringUntil(';');
        if (line_string.length() < LINE_BUF_SIZE) {
            line_string.toCharArray(cli_line, LINE_BUF_SIZE);
#ifdef CLI_DEBUG
            Serial.println(line_string);
#endif
        }
        else {
            Serial.println("CLI Input too long. Command ignored.");
        }
    }
}

void cli_parse_line_cmd() {
    char* argument;
    int counter = 0;

    argument = strtok(cli_line, " ");

    while ((argument != NULL)) {
        if (counter < MAX_NUM_ARGS) {
            if (strlen(argument) < ARG_BUF_SIZE) {
                strcpy(cli_args[counter], argument);
                argument = strtok(NULL, " ");
                counter++;
            }
            else {
                Serial.println("Input string too long.");
                break;
            }
        }
        else {
            break;
        }
    }


#ifdef CLI_DEBUG
    for (int i = 0; i < counter; i++) {
        Serial.println(cli_args[i]);
    }
#endif
}

void cli_execute_line_cmd() {

    /* ########################################################## */
        // Add functions to handle commands
    bool processed_cmd = false; // flag to indicate wether a command has been processed

    processed_cmd = jctrl_cli_process_torque_command(cli_args);

    if (!processed_cmd) {
        processed_cmd = jctrl_cli_process_drive_sys_command(cli_args);
    }
    if (!processed_cmd) {
        processed_cmd = jctrl_cli_process_position_command(cli_args);
    }
    if (!processed_cmd) {
        processed_cmd = jctrl_cli_process_nn_commands(cli_args);
    }
    if (!processed_cmd) {
        processed_cmd = jctrl_cli_process_pid_command(cli_args);
    }
    if (!processed_cmd) {
        processed_cmd = jctrl_cli_process_output_command(cli_args);
    }
    if (!processed_cmd) {
        processed_cmd = jctrl_cli_process_adapt_kalman(cli_args);
    }
    if (!processed_cmd) {
        processed_cmd = jctrl_cli_process_motion_planner_commands(cli_args);
    }
    if (!processed_cmd) {
        processed_cmd = jctrl_cli_manage_calibration_command(cli_args);
    }
    if (!processed_cmd) {
        processed_cmd = jctrl_cli_process_trajectory_command(cli_args);
    }
     if (!processed_cmd) {
        processed_cmd = jctrl_cli_process_controller_state_command(cli_args);
    }





    /* ################################################################## */
        // Clear command arguments after processing
    memset(cli_line, 0, LINE_BUF_SIZE);
    memset(cli_args, 0, sizeof(cli_args[0][0]) * MAX_NUM_ARGS * ARG_BUF_SIZE);


}

void _cli_task(void* parameters) {


    const TickType_t cli_delay = CLI_PERIOD_MS / portTICK_PERIOD_MS;

    while (cli_active) {

        xSemaphoreTake(glob_Serial_mutex, portMAX_DELAY);


        cli_read_line_cmd();
        cli_parse_line_cmd();
        cli_execute_line_cmd();
        xSemaphoreGive(glob_Serial_mutex);

        if (cli_output_active) {
            xSemaphoreTake(glob_Serial_mutex, portMAX_DELAY);
            _jctrl_cli_output_periodic();
            xSemaphoreGive(glob_Serial_mutex);
        }

        vTaskDelay(cli_delay);

    }


}


