#ifndef CLI_CORE_H
#define CLI_CORE_H

#define LINE_BUF_SIZE 128   //Maximum input string length
#define ARG_BUF_SIZE 64     //Maximum argument string length
#define MAX_NUM_ARGS 8      //Maximum number of arguments

#define CLI_DEBUG

#define CLI_PERIOD_MS 10 // 100 Hz

#include <Arduino.h>
#include <joint_control_global_def.h>
#include <JCRTL_CLI_interface_functions.h>

extern bool cli_active;

void cli_init();

/**
 * @brief reads a string from the serial port
 *
 */
void cli_read_line_cmd();
/**
 * @brief parses an input string and extracts commands and arguments
 *
 * @param input_string
 */
void cli_parse_line_cmd();

/**
 * @brief executes a command according to its arguments
 *
 */
void cli_execute_line_cmd();

void _cli_task(void* parameters);


#endif //CLI_CORE_H