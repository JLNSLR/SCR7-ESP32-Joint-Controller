#include <Arduino.h>
#include <FreeRTOS.h>
#include <drive_system.h>

#include<led_control.h>
#include <FastLED.h>

#include "soc/timer_group_struct.h"
#include "soc/timer_group_reg.h"

//CAN Testing

#include <cli_core.h>
#include <motion_interface.h>
#include <Wire.h>
#include <torqueSensor.h>

#include <joint_can_handler.h>



// --- Global Variables --- //

int counter = 0;



void setup()
{


  init_leds();
  set_leds(255, 255, 0, true, 0);
  TIMERG0.wdt_wprotect = TIMG_WDT_WKEY_VALUE;
  TIMERG0.wdt_feed = 1;
  TIMERG0.wdt_wprotect = 0;
  Serial.begin(115200);
  Serial.setTimeout(1);
  SPI.begin();
  Wire.begin(SDA, SCL, 400e3);
  xSemaphoreGive(glob_SPI_mutex);
  xSemaphoreGive(glob_Serial_mutex);
  xSemaphoreGive(glob_I2C_mutex);


  // Initialize Drive System 
  drvSys_initialize();
  Serial.println("JCTRL_INFO: Setting up Drive System succesful. Starting FOC-Controller.");
  drvSys_start_foc_processing();

  set_leds(0, 255, 0, true, 0);

  // Calibration of FOC
  //drvSys_calibrate_FOC();


  vTaskDelay(1000);

  /** Setup ASCI Interface */
  pinMode(HALL_SENSOR_PIN, INPUT);

  xSemaphoreGive(glob_Serial_mutex);

  // Start Motion Control Interface
  Serial.println("JCTRL_INFO: Starting Motion Control Interface.");
  drvSys_start_motion_control(closed_loop_foc);

  // Initialize Command-Line-Interface (CLI)
  cli_init();
  start_motion_interface();

  set_standard_lights();

  // Initialize CAN Interface;
  
  //can_init();
  //can_start_interface();
  


  //set LED blue
  set_leds(0, 0, 150, true, 0);

}

void loop()
{


  cli_read_line_cmd();
  cli_parse_line_cmd();
  cli_execute_line_cmd();


  vTaskDelay(10);
  TIMERG0.wdt_wprotect = TIMG_WDT_WKEY_VALUE;
  TIMERG0.wdt_feed = 1;
  TIMERG0.wdt_wprotect = 0;



  counter++;

  /*
  if (counter % 500 == 0) {

    if (!motion_planner.executing_traj_flag) {
      target = (float(rand()) / float(RAND_MAX)) * 175;
      if (counter % 5 == 0) {
        dir = -dir;
      }
      vel = (float(rand()) / float(RAND_MAX)) * 60.0 + 5.0;
      acc = (float(rand()) / float(RAND_MAX)) * 100.0 + 10.0;

      handle_motion_command(target * DEG2RAD * dir, vel * DEG2RAD, acc * DEG2RAD);
    }


  }
  */




}