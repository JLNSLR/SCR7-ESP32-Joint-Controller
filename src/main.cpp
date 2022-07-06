#include <Arduino.h>
#include <FreeRTOS.h>
#include <drive_system.h>

#include <FastLED.h>

#include "soc/timer_group_struct.h"
#include "soc/timer_group_reg.h"

//CAN Testing

#include <CAN.h>
#include <cli_core.h>



// --- Global Variables --- //

CRGB leds[1];




void setup()
{

  TIMERG0.wdt_wprotect = TIMG_WDT_WKEY_VALUE;
  TIMERG0.wdt_feed = 1;
  TIMERG0.wdt_wprotect = 0;
  Serial.begin(115200);
  Serial.setTimeout(1);
  SPI.begin();
  xSemaphoreGive(glob_SPI_mutex);
  xSemaphoreGive(glob_Serial_mutex);



  // Setup LEDs 
  FastLED.addLeds<NEOPIXEL, RGB_LED_PIN>(leds, 1);
  leds[0] = CRGB::GreenYellow;
  FastLED.setBrightness(255);
  FastLED.show();


  // Initialize Command-Line-Interface (CLI)
  cli_init();


  // Initialize Drive System 
  drvSys_initialize();

  Serial.println("JCTRL_INFO: Setting up Drive System succesful. Starting FOC-Controller.");
  drvSys_start_foc_processing();


  //drvSys_start_motion_control(cascade_position_control);

  //jCtrl_CLI_debug_output_start(true, false, false, false, false, false);


  //drvSys_start_debug_output();
  // Calibration of FOC
  //drvSys_calibrate_FOC();

  //drvSys_calibrate_FOC_phase();


  /** Setup ASCI Interface */

  pinMode(HALL_SENSOR_PIN, INPUT);


  Serial.println("Init CAN");
  CAN.setPins(CAN_RX, CAN_TX);

  if (!CAN.begin(1000E3)) {
    Serial.println("Starting CAN failed!");
  }
  else {
    Serial.println("Starting CAN successful!");
  }

  xSemaphoreGive(glob_Serial_mutex);

  leds[0] = CRGB::GreenYellow;
  FastLED.setBrightness(255);
  FastLED.show();


}

void loop()
{
  /*
  leds[0] = CRGB::GreenYellow;
  FastLED.setBrightness(30);
  FastLED.show();
  vTaskDelay(500);
  // Now turn the LED off, then pause
  leds[0] = CRGB::Black;
  FastLED.show();
  vTaskDelay(500);
  //vTaskDelay(10);

  /*
   Serial.print(motor_encoder.getRotationDeg() / 90.0);
   Serial.print("\t");
   Serial.println(joint_encoder.getRotationDeg());
   */

   //Serial.println(motor_encoder.getRotationDeg() / joint_encoder.getRotationDeg());


  cli_read_line_cmd();
  cli_parse_line_cmd();
  cli_execute_line_cmd();

  /*
  drvSys_extendedDriveState drvSys_state_data = drvSys_get_extended_drive_state();

  xSemaphoreTake(glob_Serial_mutex, portMAX_DELAY);
  Serial.print(drvSys_state_data.joint_pos);
  Serial.print("\t");
  Serial.print(drvSys_state_data.joint_vel);
  Serial.print("\t");
  Serial.print(drvSys_state_data.joint_acc);
  Serial.print("\t");
  Serial.print(drvSys_state_data.motor_pos);
  Serial.print("\t");
  Serial.print(drvSys_state_data.motor_vel);
  Serial.print("\t");
  Serial.println(drvSys_state_data.motor_acc);


  xSemaphoreGive(glob_Serial_mutex);
  */
  vTaskDelay(10);
  TIMERG0.wdt_wprotect = TIMG_WDT_WKEY_VALUE;
  TIMERG0.wdt_feed = 1;
  TIMERG0.wdt_wprotect = 0;





  /*
   Serial.println("Sending packet");

   CAN.beginPacket(0x12);
   CAN.write('h');
   CAN.write('e');
   CAN.write('l');
   CAN.write('l');
   CAN.write('o');
   CAN.endPacket();

   Serial.println("done");

   vTaskDelay(2000);




   // try to parse packet
   int packetSize = CAN.parsePacket();

   if (packetSize) {
     // received a packet
     Serial.print("Received ");

     if (CAN.packetExtended()) {
       Serial.print("extended ");
     }

     if (CAN.packetRtr()) {
       // Remote transmission request, packet contains no data
       Serial.print("RTR ");
     }

     Serial.print("packet with id 0x");
     Serial.print(CAN.packetId(), HEX);

     if (CAN.packetRtr()) {
       Serial.print(" and requested length ");
       Serial.println(CAN.packetDlc());
     } else {
       Serial.print(" and length ");
       Serial.println(packetSize);

       // only print packet data for non-RTR packets
       while (CAN.available()) {
         Serial.print((char)CAN.read());
       }
       Serial.println();
     }

     Serial.println();

   }*/

}