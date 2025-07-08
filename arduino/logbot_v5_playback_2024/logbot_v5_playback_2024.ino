/*
---------------------------------------------------------------
logbot-v5 (main board)
nRF52840   |  Main program; Program Memory = 1MB; RAM = 256KB
ESP32      |  Camera control 
---------------------------------------------------------------
VCAM (extention board)
ESP32      | Speaker Control
---------------------------------------------------------------
Sensors 
RTC                     RX8564LC
GPS                     u-blox ZOE-M8Q
IMU                     BMX160 
Barometer               LPS22HB
Water Pressure Sensor   MS5837
Illuminance sensor      BH1721FVC
---------------------------------------------------------------
*/

// Pin & I2C Address
#include "pin_v5.h" // Pin
#include "I2C_address_v5.h" // I2C Address

// Library
#include <SdFat.h> // SdFat Adafruit fork 2.2.3 // https://www.arduino.cc/reference/en/libraries/sdfat-adafruit-fork/
#include <Adafruit_TinyUSB.h> // import SdFat.h before Adafruit_TinyUSB.h
#include <Wire.h>
#include <TimeLib.h>
#include <SoftwareSerial.h>
#include <SparkFun_BMI270_Arduino_Library.h> // // BMI270 https://github.com/sparkfun/SparkFun_BMI270_Arduino_Library
#include <DFRobot_BMM150.h> // BMM150 https://github.com/DFRobot/DFRobot_BMM150
#include "u-blox/SparkFun_u-blox_GNSS_Arduino_Library.h" // // u-blox GNSS version 2.2.22 https://github.com/sparkfun/SparkFun_u-blox_GNSS_Arduino_Library

// SD
SdFat sd;
File binFile;

// Class for GPS, BMI270, BMM150
SFE_UBLOX_GNSS myGNSS; // u-blox
BMI270 imu;
DFRobot_BMM150_I2C bmm150(&Wire, BMM150_SLAVE_ADDRESS);

// Config files
// #include "configs/v5_umineko_2024_debug.h"
// #include "configs/v5_umineko_2024_no_sleep.h" // no startup delay, no sleep
#include "configs/v5_umineko_2024_sleep_ctrl_by_time_2.h" // 03:30 to 19:00, sleep until the next 03:30 after the initial startup

// Data Structure
#include "UserDataType_v05.h"
rtc1_t rtc_time;
rtc1_t rtc_time_copy;
rtc1_t gps_time;         // GPS time data for RTC time synchronization
rtc1_t gps_time_copy;    // GPS time data for RTC time synchronization
gps_t gps_data;          // GPS time and location information
gps_t gps_data_copy;     // copy of gps_data
gps_t last_gps_loc;      // copy of gps_data
data1_t imu_data;        // acc, gyro, mag sensor data
data1_t imu_data_copy;   // acc, gyro, mag sensor data
data2_t other_data;      // sensor data + other info
data2_t other_data_copy; // sensor data + other info

// data buffer (An array containing structures as elements)
rtc1_t rtc_time_buf[DATA_BUFFER_SIZE_SEC];
gps_t gps_data_buf[DATA_BUFFER_SIZE_SEC];
data1_t imu_data_buf[DATA_BUFFER_SIZE_SEC];
data2_t other_data_buf[DATA_BUFFER_SIZE_SEC];

// Camera Control
SoftwareSerial SerialESP32(ESP32_CTRL_RXD_PIN, ESP32_CTRL_TXD_PIN); // (RXD, TXD)

// Speaker Control
SoftwareSerial SerialVCAM(VCAM_CTRL_RXD_PIN, VCAM_CTRL_TXD_PIN); // (RXD, TXD)


void setup() 
{
  // Base setup
  delay(200);
  set_up_pinMode_GPIO_port();
  green_led_blink(100, 100, 10);
  start_serial_connection();
  check_battery_status_at_start(other_data.battery_level, 10);
  wait_until_enter_key_if_connected_to_pc();
  start_I2C_connection(); // Wire | I2C connection
  // SD setup
  set_up_SD();
  initialize_log_file_in_sd();
  write_settings_txt(); // Save settings
  // Sensor setup
  set_up_GPS_module(); // trying GPS ON in the first loop
  set_up_sensors(); // BMI270, BMM150, Barometer, Water Pressure, Illuminance sensor
  countdown_green_led_blink_before_start();
  if (LOGGING_START_JST_HOUR < 100 && startup_delay_remaining_min == 0) startup_delay_remaining_min = 10; // Temporarily assign 10 minutes
  // RTC time initialization using GPS data
  wait_until_gps_position_fix(10); // time_updated_count = 10 (Exit the while loop after successfully obtaining 10 position fixes)
  sleep_control(); // check if the logger should sleep or not
  start_time = millis(); //  update just before the start of logging
  last_time_1hz = start_time;
  last_time_tmp = start_time;
} // End of setup()


void loop() 
{ 
  // --------------------------------------------
  // 25 Hz Logging Routine (common process)
  // --------------------------------------------
  start_time_tmp = millis();
  // unsigned long diff_time = (start_time_tmp - (last_time_tmp - 40));
  unsigned long diff_time_tmp = start_time_tmp + 40 - last_time_tmp;
  bool skip_this_loop = false;
  if (diff_time_tmp > 40 + 1) skip_this_loop = true; // Allow + 1 ms
  else skip_this_loop = false;
  // If a delay occurs, this diff value will exceed 40 ms.
  // In that case, skip reading the data and mark it as missing, 
  // then immediately proceed to the next loop to adjust time.
  // In this way, even with significant delays (100 - 400 ms), the system will recover after a few loops.
  if (!skip_this_loop) // 41 ms (+ 1 ms deviation is allowed) 
  {
    // start_time_acc = millis();
    if (imu_power_on) read_BMI270_Data(imu_data.acc, imu_data.gyro, magnitude_buffer, block_counter); // start_index = block_counter
    if (imu_power_on && block_counter == 0) BMM150_FIFO_read_data(imu_data.mag);
    // end_time_acc = millis();
  #ifdef DEBUG_SERIAL_PRINT
    if (connected_to_pc)
    {
      Serial.print(" "); Serial.print(process_time_tmp); Serial.print(" | ");
      Serial.print("b"); Serial.print(block_counter); Serial.print(" ("); Serial.print(diff_time_tmp); Serial.print(") ");
    }
  #endif // DEBUG_SERIAL_PRINT
  }
  else
  { // Since IMU data is not being read in this loop, the data will be missing 
    delay_occurred_counter += 1;
  }

  // start_time_gps = millis();
  if (read_gps_data_now) myGNSS.checkUblox();
  // end_time_gps = millis();

#ifdef DEBUG_SERIAL_PRINT
  if (block_counter == 0) 
  {
    start_time_1hz = start_time_tmp;
    unsigned long diff_time_1hz = start_time_1hz + 1000 - last_time_1hz;
    if (connected_to_pc)
    {
      Serial.println(""); Serial.print("1 Hz loop time: "); Serial.println(diff_time_1hz);
    }
  }
#endif // DEBUG_SERIAL_PRINT

  // -----------------------------------------
  // Process per block 
  // -----------------------------------------
  if (block_counter == 0) // 3 + 6 = 9 ms
  {
    debug_1hz_green_led_blink(true); // ON
    read_RTC_and_update_time(rtc_time);
    check_battery_status_v2(other_data.battery_level); // update battery_level_v per min (at 00 s) 
    read_barometer(other_data.atmopress);
    read_illuminance(other_data.illuminance);
    prepare_water_D1(); // D1 should be read after 20 ms after this function
  }
  else if (block_counter == 1) // 4 ms
  {
    debug_1hz_green_led_blink(false); // OFF
    read_water_D1(other_data.D1);
    prepare_water_D2(); // D2 should be read after 20 ms after this function
  }
  else if (block_counter == 2) // 
  {
    read_water_D2(other_data.D2);
  #ifdef _PLUS_WATER_DEPTH
    calculate_wps(other_data.D1, other_data.D2);
  #endif // _PLUS_WATER_DEPTH
  // Send the command to turn on the camera 
  // when the conditions for turning on the camera are met
  // The command is sent here to allow for some wait time after the ESP32 startup
  // The ESP32 startup occurs when block_counter == 5 ()
    send_camera_control_command(); // recording start command
  }
  else if (block_counter == 3) // 3 + 4 = 7 ms (serial print only for debug mode)
  {
    // Send speaker command
    send_speaker_command();
  }
  else if (block_counter == 4) // 3 + SD write time
  {
    camera_countdown(); // call this before camera_control

    // Real-time behavior recognition -> Control camera and speaker related variables
    // run_real_time_behavior_recognition();
    if (trigger_type == 0) 
    { // 0: time (camera sampling at specified intervals)
      control_camera_interval_sampling();
    } else if (trigger_type == 1) 
    { // 1:   acc data trigger real-time behaviour recognition
      real_time_abr_using_acc_data_and_intervention();
    }
    debug_detection_yellow_led_blink(true); // ON
  }
  else if (block_counter == 5)
  {
    debug_detection_yellow_led_blink(false); // OFF

    // Control ESP32 (Main board) ON/OFF
    camera_control(); // recording stop command and turning off ESP32 for controlling the camera module
    
    // Control VCAM and audio playback intervention
  #ifdef USE_SPEAKER
    // Control ESP32 (VCAM) ON/OFF
    if (cam_spk_gps_control_version == 1)
    {
      speaker_control_and_countdown_for_final_check();
    }
    else if (cam_spk_gps_control_version == 2)
    {
      play_audio_control_v2();
    }
    else if (cam_spk_gps_control_version == 3)
    {
      play_audio_control_v3();
    }
  #endif // USE_SPEAKER

  }
  else if (block_counter == 6)
  {
    // Write SD
    run_sd_write_routine();
    // 100 ms delay -> recovery at block 9 - 10
    // 400 ms delay -> recovery at block 17 - 18
    // 500 ms delay -> recovery at block 20 - 21
  }
  else if (block_counter == 7)
  {
  #ifdef DEBUG_SERIAL_PRINT
    if (connected_to_pc) 
    { // To monitor the data that will be written to the SD card, 
      // insert the below function before the SD write operation.
      // print_debug_message_if_connected_to_PC(rtc_time, gps_data, imu_data_copy, other_data);
      print_debug_message_if_connected_to_PC(rtc_time_copy, gps_data_copy, imu_data_copy, other_data_copy);
    }
  #endif // DEBUG_SERIAL_PRINT
  }
  // else if (block_counter == 8)
  // {
  // }
  // else if (block_counter == 9)
  // {
  // }
  // else if (block_counter == 10)
  // {
  // }
  // else if (block_counter == 11)
  // {
  // }
  // else if (block_counter == 12)
  // {
  // }
  // else if (block_counter == 13)
  // {
  // }
  // else if (block_counter == 14)
  // {
  // }
  // else if (block_counter == 15)
  // {
  // }
  // else if (block_counter == 16)
  // { 
  // }
  // else if (block_counter == 17)
  // { 
  // }
  // else if (block_counter == 18)
  // {
  // }
  // else if (block_counter == 19)
  // {
  // }
  // else if (block_counter == 20)
  // {
  // }
  // else if (block_counter == 21)
  // { 
  // }
  else if (block_counter == 22) // 
  { 
    // GPS data check
    run_update_GPS_data_routine();
    run_GPS_power_save_control_routine();
    debug_GPS_green_led_blink(true); // ON
  }
  else if (block_counter == 23) // 3 ms
  {
    debug_GPS_green_led_blink(false); // OFF
    if (gps_location_updated)
    {
      outside_bounding_box = logbot_is_outside_bounding_box_v2(); // this use last_gps_loc
      outside_bounding_box_buffer[outside_bounding_box_buffer_index] = outside_bounding_box;
      outside_bounding_box_buffer_index = (outside_bounding_box_buffer_index + 1) % 10;
    }
    if (force_gps_1hz_mode)
    {
      gps_fix_type_buf_check_ok = check_gps_fix_type_buffer(gps_fix_type_buffer, GPS_FIX_TYPE_BUFFER_SIZE);
      outside_bounding_box_buf_check_ok = check_outside_bounding_box_buffer(outside_bounding_box_buffer, OUTSIDE_BOUNDING_BOX_BUFFER_SIZE);
    }
    check_if_gps_1hz_mode_stable_and_outside_bb();
  }
  
  if (block_counter == 24)
  { 
    control_force_gps_1hz_mode();
    update_variables(other_data); //  Update trigger-related variables

    // Copy the IMU data once the data for 25 loops has been accumulated.
    copy_acc_and_gyro_data(imu_data, imu_data_copy);
    init_acc_and_gyro_data(imu_data); // init acc & gyro data
    copy_magnitude_data(magnitude_buffer, magnitude_buffer_copy);
    copy_mag_data(imu_data, imu_data_copy);
    
    // Copy the other 1 Hz data
    copy_rtc_time(rtc_time, rtc_time_copy);
    copy_other_data(other_data, other_data_copy);
    copy_GPS_data(gps_data, gps_data_copy);

    // Update the buffer to be written to the SD card
    // 01 second buf: 00 second -> 0, 01 second -> 0, ..., 58 seconds ->  0, 59 seconds ->  0
    // 02 second buf: 00 second -> 0, 01 second -> 1, ..., 58 seconds ->  0, 59 seconds ->  1
    // 10 second buf: 00 second -> 0, 01 second -> 1, ..., 58 seconds ->  8, 59 seconds ->  9
    // 60 second buf: 00 second -> 0, 01 second -> 1, ..., 58 seconds -> 58, 59 seconds -> 59
    data_buffer_index = current_sec % DATA_BUFFER_SIZE_SEC;
    copy_rtc_time_to_buf(rtc_time_copy);
    copy_imu_data_to_buf(imu_data_copy);
    copy_other_data_to_buf(other_data_copy);
    copy_GPS_data_to_buf(gps_data_copy);
    current_min_for_sd_write = current_min; 
    current_sec_for_sd_write = current_sec;

  #ifdef DEBUG_SERIAL_PRINT
    if (connected_to_pc)
    {
      Serial.print("delay_occurred_counter: "); Serial.println(delay_occurred_counter);
    }
  #endif // DEBUG_SERIAL_PRINT

    process_time_total_copy = process_time_total;
    delay_occurred_counter = 0;
    process_time_total = 0;
    delay_time_total = 0;
    sleep_control();
  }

  // If a delay occurs, immediately exit this loop.
  end_time_tmp = millis();
  process_time_tmp = end_time_tmp - last_time_tmp;
  process_time_total = process_time_total + process_time_tmp;
  // If there is a write delay with the SD card, set the delay to 0 ms.
  delay_time_tmp = calc_delay_time_milliseconds(last_time_tmp, end_time_tmp, (40 - 3), false); 
  delay(delay_time_tmp);

  // For adjusting time
  if (RTC_1HZ_CLOCK_PIN_USE_MODE == 2 && block_counter == 24)
  {
    pinMode(RTC_CLOCK_1HZ, INPUT);
    while(digitalRead(RTC_CLOCK_1HZ) == 1); // Wait for the falling edge (1 -> 0) of the RTC 1 Hz clock.
  }
  else if (RTC_1HZ_CLOCK_PIN_USE_MODE < 2 && block_counter == 24)
  {
    while ( (millis() - last_time_1hz) < 1000 ); //
  }
  else
  {
    while ( (millis() - last_time_tmp) < 40 ); //
  }

  if (block_counter == 24) last_time_1hz += 1000; // 1000 ms
  block_counter = (block_counter + 1) % 25; // 25 Hz (0, 1, 2, ...., 24)
  last_time_tmp += 40; // 40 ms  
} // End of loop()


// --------------------------------------------------------------------------
// Functions
// --------------------------------------------------------------------------

unsigned long convert_micros_to_millis(unsigned long microseconds)
{
  // Convert microseconds to milliseconds and return the integer part.
  unsigned long milliseconds = microseconds / 1000;
  return milliseconds;
}


unsigned long calc_delay_time_microseconds(
  const unsigned long start_time, 
  unsigned long end_time, 
  unsigned long max_usec, 
  bool print
) 
{
  unsigned long delay_time = 0;
  unsigned long process_time = 0;
  unsigned long process_time_actual = 0;
  float _process_time = 0.0;
  float _delay_time = 0.0;

  process_time = end_time - start_time;
  process_time_actual = process_time;
  // Serial.println(process_time);
  if (process_time >= max_usec) {
    process_time = max_usec;
    delay_time = 0;
  } 
  else {
    delay_time = (max_usec - process_time);
  }

  // _process_time = process_time / 1000;
  // _delay_time = delay_time / 1000;
  // Serial.print(_process_time); Serial.print(":"); Serial.print(_delay_time); Serial.print(" | ");
  
  if (process_time_actual >= max_usec) 
  {
    delay_occurred_counter++;

#ifdef DEBUG_SERIAL_PRINT
    if (connected_to_pc) 
    {
      Serial.print("!!! WARNING: process_time_actual = "); 
      float process_time_actual_f = process_time_actual / 1000.0;
      Serial.print(process_time_actual_f, 3); Serial.print(" !!! "); Serial.print("block_counter = "); Serial.println(block_counter);
      Serial.println(F("----------------------------------------------------------------------------------------------------------------------"));
    }
#endif // DEBUG_SERIAL_PRINT
  }

#ifdef DEBUG_SERIAL_PRINT
  if (connected_to_pc && print) 
  {
    Serial.print("Time info:");
    Serial.print(" process_time_actual: "); Serial.print(process_time_actual); Serial.print(" (actual time in us)");
    // Serial.print(" start_time: "); Serial.print(start_time); Serial.print(" end_time: "); Serial.print(end_time);
    Serial.print(" process_time: "); Serial.print(process_time); Serial.print(" delay_time: "); Serial.print(delay_time); Serial.println(" (us)");
    Serial.println(F("----------------------------------------------------------------------------------------------------------------------"));
  }
#endif // DEBUG_SERIAL_PRINT
  return delay_time;
}

unsigned long calc_delay_time_milliseconds(
  const unsigned long start_time, 
  unsigned long end_time, 
  uint16_t max_msec, 
  bool print
) 
{
  // Serial.print("start_time = "); Serial.println(start_time);
  // Serial.print("end_time = "); Serial.println(end_time);

  unsigned long delay_time = 0;
  unsigned long process_time = 0;
  unsigned long process_time_actual = 0;

  process_time = end_time - start_time;
  process_time_actual = process_time;

  if (process_time >= max_msec) 
  {
    process_time = max_msec;
    delay_time = 0;
  } 
  else 
  {
    delay_time = (max_msec - process_time);
  }

  if (process_time_actual >= max_msec) 
  {
    // delay_occurred_counter++;

#ifdef DEBUG_SERIAL_PRINT
    if (connected_to_pc) 
    {
      Serial.print("!!! WARNING: process_time_actual = "); Serial.print(process_time_actual); Serial.print(" (ms) !!! "); 
      Serial.print("delay_time = "); Serial.print(delay_time); Serial.print(" (ms) | "); 
      Serial.print("block = "); Serial.println(block_counter);
      Serial.println(F("----------------------------------------------------------------------------------------------------------------------"));
    }
#endif // DEBUG_SERIAL_PRINT

  }

#ifdef DEBUG_SERIAL_PRINT
  if (connected_to_pc && print) {
    Serial.print("Time info:");
    // Serial.print(" start_time: "); Serial.print(start_time); Serial.print(" end_time: "); Serial.print(end_time);
    Serial.print(" process_time: "); Serial.print(process_time); Serial.print(" delay_time: "); Serial.println(delay_time); Serial.println(" (ms)");
    Serial.println(F("--------------------------------------------------------------------"));
  }
#endif // DEBUG_SERIAL_PRINT
  return delay_time;
}


void wait_until_gps_position_fix(uint8_t time_updated_count) 
{
  // --------------------------------------------
  // GPS Position Fix before logging
  // --------------------------------------------

#ifdef DEBUG_SERIAL_PRINT
  if (connected_to_pc) 
  {
    Serial.println("--------------------------------------------------");
    Serial.println("Get GPS position fix (2D or 3D) before logging");
    Serial.println("--------------------------------------------------");
  }
#endif // DEBUG_SERIAL_PRINT

  // Read the current time just before entering the while loop of wait_until_gps_position_fix().
  start_time = millis();
  last_time = start_time;

  while (!time_initialized) 
  {  
    myGNSS.checkUblox(); // Check for the arrival of new data and process it.
    gps_location_updated = check_myGNSS_and_read_data(gps_time, gps_data);
    myGNSS.checkCallbacks();

    gps_fix_40ms_counter = (gps_fix_40ms_counter + 1) % 25; // 25 Hz
    if (gps_fix_40ms_counter == 0)
    {
      gps_fix_1s_counter++; // 40 ms * 25 = 1000 ms
      File logFile = sd.open("gps_rtc_init_log.txt", FILE_WRITE);
      if (logFile) logFile.println(gps_fix_1s_counter); 
      logFile.close();
      #ifdef DEBUG_SERIAL_PRINT
        if (connected_to_pc) 
        {
          Serial.print("gps_fix_1s_counter = "); Serial.println(gps_fix_1s_counter);
        }
      #endif // DEBUG_SERIAL_PRINT
    }

    if (gps_fix_40ms_counter == 0 || gps_fix_40ms_counter % 2 == 1)
    {
      green_led_blink(10, 0, 1);
    }
    else
    {
      yellow_led_blink(10, 0, 1);
    }

    if (gps_new_data)
    {
      if (gps_location_updated) 
      {
        save_log_data_to_sd_v2(rtc_time, gps_time, gps_data);
        if (connected_to_pc) Serial.println("gps_location_updated = true!");
      } 
      else 
      {
        File logFile = sd.open("gps_rtc_init_log.txt", FILE_WRITE);
        if (logFile) logFile.print(". "); 
        logFile.close();
      #ifdef DEBUG_SERIAL_PRINT
        if (connected_to_pc) Serial.print(". ");
      #endif // DEBUG_SERIAL_PRINT
      }
    }
    
    // If the position cannot be fixed within 5 minutes, 
    // sleep for the specified number of seconds and then force a restart.
    if ( ( (millis() - start_time) > 60 * 5 * 1000) 
        && !time_initialized
        && !connected_to_pc) 
    {
      // sleep_sec = 10; // 10 s
      // sleep_sec =  1 * 60; //  1 m
      // sleep_sec = 10 * 60; // 10 m
      // sleep_sec = 30 * 60; // 30 m
      // sleep_sec = 60 * 60; // 60 m
      // sleep_sec = 30 * 60 - ((5 + 1) * 60); // Reboot after 30 m
      sleep_sec = 60 * 60 - ((5 + 1) * 60); // Reboot after 60 m
      sleep_nRF52840_by_sending_command_to_ESP32(sleep_sec); // sleep for specified seconds and restart
    }

    // Time synchronization
    if (gps_location_updated) 
    { 
      if (gps_time.sec != gps_time_copy.sec)
      { // If the gps_time seconds have been updated, update gps_time_copy
        gps_time_updated_count++;
        copy_GPS_time(gps_time, gps_time_copy);
      }

    #ifdef DEBUG_SERIAL_PRINT
      if (connected_to_pc) 
      {
        Serial.print("gps_time_updated_count: "); 
        Serial.println(gps_time_updated_count); 
      }
    #endif // DEBUG_SERIAL_PRINT

      if (gps_time_updated_count >= time_updated_count) 
      { // Initialize the RTC time with the GPS time
        // The RTC time may be one second ahead of the GPS time
        adjust_RTC_time_using_GPS_time_data_v1(gps_time); 
        read_RTC_and_update_time(rtc_time);
      #ifdef MAKE_SAVE_DIRS
        if ((startup_delay_remaining_min == 0) 
            || ( (startup_delay_remaining_min > 0 || LOGGING_START_JST_HOUR < 100) && sd.exists("start_up_delay_condition.txt")))
        {
          make_save_dirs(rtc_time, 40); // To reduce the SD write time
        }
      #endif // MAKE_SAVE_DIRS
      }
      else 
      {
        time_initialized = false;
      }
    }

    // For debugging, force time synchronization after 5 seconds and exit the while loop
    #ifdef DEBUG_FAKE_GPS_IN_SETUP
      if (!time_initialized && ( (millis() - start_time) > 5 * 1000) ) 
      {
        adjust_RTC_with_fake_GPS_data(); // time_initialized = true;
        read_RTC_and_update_time(rtc_time);
      #ifdef MAKE_SAVE_DIRS
      if ((startup_delay_remaining_min == 0) 
          || ((startup_delay_remaining_min > 0 || LOGGING_START_JST_HOUR < 100) && sd.exists("start_up_delay_condition.txt")))
      {
        // make_save_dirs(rtc_time, 10); // To reduce the SD write time
        make_save_dirs(rtc_time, 5); // To reduce the SD write time
        // Create only one directory (use this to test if new directory will be successfully created. 
        // It will take more than 40 ms to create a new directory)
        // make_save_dirs(rtc_time, 1);
      }
      #endif // MAKE_SAVE_DIRS
      }
    #endif // DEBUG_FAKE_GPS_IN_SETUP

    delay_time_tmp = calc_delay_time_milliseconds(last_time, millis(), (40 - 5), false);
    delay(delay_time_tmp);
    while (millis() - last_time < 40)
    {
      //
    }
    last_time += 40;

  }
}


void sleep_control() 
{
  if (time_initialized && !camera_command && !camera_recording)
  {
    if (startup_delay_remaining_min > 0 || LOGGING_START_JST_HOUR < 100) {
      // After exiting from startup_delay, ensure it doesn't happen again.
      // Check the file with the condition before sleeping.
      // If the condition file exists in the SD card, 
      // it means startup_delay has already completed, so set startup_delay_remaining_min as zero.
      // If the start_up_delay file doesn't exist, write a new file to the SD card.
      if (sd.exists("start_up_delay_condition.txt")) // Once the device goes to sleep (startup_delay), this will be written.
      {
        // File conditionFile = sd.open("start_up_delay_condition.txt", FILE_WRITE);
        // if (conditionFile) {
        //   conditionFile.println("restarted"); 
        // }
        // conditionFile.close();
        // sd.remove("start_up_delay_condition.txt");
        startup_delay_remaining_min = 0; // no more start up delay
      } 
      else 
      {
        if (LOGGING_START_JST_HOUR < 100)
        {
          // Note that rtc_time data is in UTC!
          uint16_t current_min_counter = convert_gmt_to_local_hour(rtc_time.hour) * 60 + rtc_time.min;
          // uint16_t target_min_counter = convert_gmt_to_local_hour(LOGGING_START_JST_HOUR) * 60 + 0;
          uint16_t target_min_counter = LOGGING_START_JST_HOUR * 60 + LOGGING_START_JST_MIN;
          if (target_min_counter - current_min_counter >= 0)
          { // If the difference is greater than or equal to 0, use the difference as is.
            startup_delay_remaining_min = target_min_counter - current_min_counter;
          }
          else
          { // If the difference is negative, add 24 hours (1440 minutes) to it.
            startup_delay_remaining_min = target_min_counter - current_min_counter + 24 * 60;
          }

          // Since it takes some time for position fix and starting SD write after reboot, subtract 2 minutes.
          if (startup_delay_remaining_min > 2) startup_delay_remaining_min - 2; 

          #ifdef DEBUG_SERIAL_PRINT
            if (connected_to_pc)
            {
              Serial.print("current_min_counter "); Serial.println(current_min_counter);
              Serial.print("target_min_counter "); Serial.println(target_min_counter);
            }
          #endif // DEBUG_SERIAL_PRINT
        }

        sleep_sec = startup_delay_remaining_min * 60;

      #ifdef DEBUG_SERIAL_PRINT
        if (connected_to_pc)
        {
          Serial.print("startup_delay_remaining_min: "); Serial.print(startup_delay_remaining_min); Serial.println(" (m)"); 
          Serial.print("sleep_sec: "); Serial.print(sleep_sec); Serial.println(" (s)"); 
          Serial.println("Sleep check -> Sleep by start_up_delay_remaining_min"); // turn off the power!
        }
      #endif // DEBUG_SERIAL_PRINT

        File conditionFile = sd.open("start_up_delay_condition.txt", FILE_WRITE);
        if (conditionFile) 
        {
          conditionFile.println("start_up_delay");
          conditionFile.print("STARTUP_DELAY_MIN: "); conditionFile.println(STARTUP_DELAY_MIN);
          conditionFile.print("LOGGING_START_JST_HOUR: "); conditionFile.println(LOGGING_START_JST_HOUR);
          conditionFile.print("LOGGING_START_JST_MIN: "); conditionFile.println(LOGGING_START_JST_MIN);
          conditionFile.print("startup_delay_remaining_min: "); conditionFile.print(startup_delay_remaining_min); conditionFile.println(" (min)");
          conditionFile.print("sleep_sec: "); conditionFile.print(sleep_sec); conditionFile.println(" (sec)");
          conditionFile.println("no more start_up_delay");  
        }
        conditionFile.close();
        // delay(1000);
        sleep_nRF52840_by_sending_command_to_ESP32(sleep_sec);
      }
    }

  // Sleep if the current time is outside the specified range (e.g., from 18:00 to 05:00).
  #ifdef SLEEP_CTRL_BY_TIME // Time
    if (sleep_control_by_time(min_counter)) {
      if (min_counter < DAWN) {
        sleep_sec = 60 * (DAWN - min_counter);
      }
      else if (min_counter > DUSK) { 
        sleep_sec = 60 * (1440 - min_counter + DAWN);
      }
    #ifdef DEBUG_SERIAL_PRINT
      Serial.println("Sleep check -> Sleep by sleep_control_by_time"); // turn off the power!
    #endif // DEBUG_SERIAL_PRINT
      sleep_nRF52840_by_sending_command_to_ESP32(sleep_sec);
    }
  #endif // SLEEP_CTRL_BY_TIME // Time
  }
}


void wait_until_enter_key_if_connected_to_pc()
{
  if (connected_to_pc)
  {
    // Serial.println(F(" "));
    Serial.println(F("------------------------------------------------------------"));
    Serial.print(F("Press Enter Key to start !"));
    while (Serial.available() == false) {
      // wait until pressing Enter key or any
    }
    Serial.println(F(" -> Start !"));
    Serial.println(F("------------------------------------------------------------"));
    Serial.println(F(" "));
    Serial.flush();
  }
}


void start_serial_connection() 
{
  // pinMode(EXT_BOARD_CONNECT_PIN, INPUT);
  VBUS_state = digitalRead(EXT_BOARD_CONNECT_PIN);
  if (VBUS_state == 1) { // HIGH
    connected_to_pc = true;
  } else { // LOW
    connected_to_pc = false;
  }

  // delay(100);
  if (connected_to_pc) {
    Serial.begin(115200);
    // Serial.begin(9600);
    while (Serial == false) {
      // wait until Serial is ready
    }
    delay(1000);
    Serial.print(F("Connected to PC -> Serial Ready at ")); 
    Serial.println(F("115200"));
    // Serial.println(F("9600"));

    // for debug
    Serial.print("VBUS_state: "); Serial.println(VBUS_state);

    Serial.print(F("Compiled on: ")); 
    Serial.print(F(__DATE__)); 
    Serial.print(F(" ")); 
    Serial.println(F(__TIME__));
    Serial.println(F(" "));
    Serial.println(F("-------------------- logbot-v5 settings --------------------"));

    Serial.print(F("Base Mode: "));
  #ifdef UMINEKO_2024_MODE
    Serial.println(F("UMINEKO_2024_MODE"));
  #elif defined(UMINEKO_2024_DAY_MODE)
    Serial.println(F("UMINEKO_2024_DAY_MODE"));
  #elif defined(UMINEKO_2024_NIGHT_MODE)
    Serial.println(F("UMINEKO_2024_NIGHT_MODE"));
  #elif defined(DEBUG_UMINEKO_2024_MODE)
    Serial.println(F("DEBUG_UMINEKO_2024_MODE"));
  #elif defined(INTERVAL_SAMPLING_MODE)
    Serial.println(F("DEBUG_UMINEKO_2024_MODE"));
  #endif
    Serial.println(F("------------------------------------------------------------"));
    

    // For debugging
    // Serial.println(F("------------------------------------------------------------"));
    // Serial.println(F("| Debug | --------------------------------------------------"));
    Serial.println(F("| Debug |"));
    Serial.print(F("DEBUG_SERIAL_PRINT: "));
  #ifdef DEBUG_SERIAL_PRINT
    Serial.println(F("True"));
  #else // DEBUG_SERIAL_PRINT
    Serial.println(F("False"));
  #endif // DEBUG_SERIAL_PRINT
    Serial.print(F("DEBUG_FAKE_GPS_IN_SETUP: "));
  #ifdef DEBUG_FAKE_GPS_IN_SETUP
    Serial.println(F("True"));
  #else // DEBUG_FAKE_GPS_IN_SETUP
    Serial.println(F("False"));
  #endif // DEBUG_FAKE_GPS_IN_SETUP
    Serial.print(F("DEBUG_FAKE_GPS_IN_LOOP: "));
  #ifdef DEBUG_FAKE_GPS_IN_LOOP
    Serial.println(F("True"));
  #else // DEBUG_FAKE_GPS_IN_LOOP
    Serial.println(F("False"));
  #endif // DEBUG_FAKE_GPS_IN_LOOP

    Serial.print(F("RTC_1HZ_CLOCK_PIN_USE_MODE: ")); Serial.println(RTC_1HZ_CLOCK_PIN_USE_MODE);

    // Serial.println(F("------------------------------------------------------------"));
    // Serial.println(F("| Sleep Control | ------------------------------------------"));
    Serial.println(F("| Sleep Control |"));
    Serial.print(F("STARTUP_DELAY_MIN: ")); Serial.println(STARTUP_DELAY_MIN);
    Serial.print(F("LOGGING_START_JST_HOUR: ")); Serial.println(LOGGING_START_JST_HOUR);
    Serial.print(F("LOGGING_START_JST_MIN: ")); Serial.println(LOGGING_START_JST_MIN);
    Serial.print(F("DAWN: ")); Serial.print(DAWN); Serial.print(F(" (")); Serial.print(DAWN/60); Serial.println(F(")"));
    Serial.print(F("DUSK: ")); Serial.print(DUSK); Serial.print(F(" (")); Serial.print(DUSK/60); Serial.println(F(")"));

    // Serial.println(F("------------------------------------------------------------"));
    // Serial.println(F("| SD | -----------------------------------------------------"));
    Serial.println(F("| SD |"));
    Serial.print(F("MAKE_SAVE_DIRS: "));
  #ifdef MAKE_SAVE_DIRS
    Serial.println(F("True"));
  #else // MAKE_SAVE_DIRS
    Serial.println(F("False"));
  #endif // MAKE_SAVE_DIRS
    Serial.print(F("DATA_BUFFER_SIZE_SEC: ")); Serial.println(DATA_BUFFER_SIZE_SEC);
    

    // GPS Mode
    // Serial.println(F("------------------------------------------------------------"));
    // Serial.println(F("| GPS | --------------------------------------------"));
    Serial.println(F("| GPS |"));
    Serial.print(F("GPS Mode: "));
  #ifdef GPS_CONTINUOUS_1HZ_MODE
    Serial.println(F("GPS_CONTINUOUS_1HZ_MODE"));
  #elif defined(GPS_POWER_ON_OFF_MODE_01ON_02OFF)
    Serial.println(F("GPS_POWER_ON_OFF_MODE_01ON_02OFF"));
  #elif defined(GPS_POWER_ON_OFF_MODE_02ON_13OFF)
    Serial.println(F("GPS_POWER_ON_OFF_MODE_02ON_13OFF"));
  #elif defined(GPS_POWER_ON_OFF_MODE_03ON_27OFF)
    Serial.println(F("GPS_POWER_ON_OFF_MODE_03ON_27OFF"));
  #endif
    Serial.print(F("GPS_1HZ_RECORD_AFTER_START_COUNTDOWN_SEC: ")); Serial.println(GPS_1HZ_RECORD_AFTER_START_COUNTDOWN_SEC);
    Serial.print(F("FORCE_GPS_1HZ_MODE_SEC: ")); Serial.println(FORCE_GPS_1HZ_MODE_SEC);
    Serial.print(F("FORCE_GPS_1HZ_MODE_INIT_SEC: ")); Serial.println(FORCE_GPS_1HZ_MODE_INIT_SEC);
    Serial.print(F("STOP_GPS_1HZ_MODE_IF_BIRD_IS_NOT_FLYING: "));
  #ifdef STOP_GPS_1HZ_MODE_IF_BIRD_IS_NOT_FLYING
    Serial.println(F("True"));
  #else // 
    Serial.println(F("False"));
  #endif // 
    Serial.print(F("STOP_GPS_1HZ_MODE_IF_BIRD_IS_NOT_FLYING_START_SEC: ")); Serial.println(STOP_GPS_1HZ_MODE_IF_BIRD_IS_NOT_FLYING_START_SEC);

    // Serial.println(F("------------------------------------------------------------"));
    // Serial.println(F("| Behaviour Recognition | ----------------------------------"));
    Serial.println(F("| Behaviour Recognition |"));
    USE_LOCATION_BOUNDING_BOX_TRIGGER
    Serial.print(F("CAM_SPK_GPS_CONTROL_VERSION: ")); Serial.println(CAM_SPK_GPS_CONTROL_VERSION);
    Serial.print(F("TRIGGER_TYPE: ")); Serial.println(TRIGGER_TYPE);
    Serial.print(F("TARGET_BEHAVIOR_CLASS_NUMBER: ")); Serial.println(TARGET_BEHAVIOR_CLASS_NUMBER);
    Serial.print(F("TARGET_BEHAVIOR_CONSECUTIVE_SEC: ")); Serial.println(TARGET_BEHAVIOR_CONSECUTIVE_SEC);
    Serial.print(F("BEHAVIOR_CLASS_BUFFER_SIZE: ")); Serial.println(BEHAVIOR_CLASS_BUFFER_SIZE);
    Serial.print(F("MINIMUM_TARGET_BEHAVIOR_COUNT: ")); Serial.println(MINIMUM_TARGET_BEHAVIOR_COUNT);
    Serial.print(F("GPS_FIX_TYPE_BUFFER_SIZE: ")); Serial.println(GPS_FIX_TYPE_BUFFER_SIZE);
    Serial.print(F("OUTSIDE_BOUNDING_BOX_BUFFER_SIZE: ")); Serial.println(OUTSIDE_BOUNDING_BOX_BUFFER_SIZE);
    Serial.print(F("USE_LOCATION_BOUNDING_BOX_TRIGGER: "));
  #ifdef USE_LOCATION_BOUNDING_BOX_TRIGGER
    Serial.println(F("True"));
  #else // USE_LOCATION_BOUNDING_BOX_TRIGGER
    Serial.println(F("False"));
  #endif // USE_LOCATION_BOUNDING_BOX_TRIGGER
    Serial.print(F("Bounding Box:"));
    Serial.print(F(" NE ")); Serial.print(AREA_NE_LAT, 5); Serial.print(F(", ")); Serial.print(AREA_NE_LON, 5);
    Serial.print(F(" SW ")); Serial.print(AREA_SW_LAT, 5); Serial.print(F(", ")); Serial.print(AREA_SW_LON, 5);
    Serial.println(F(""));

    // Serial.println(F("------------------------------------------------------------"));
    // Serial.println(F("| Camera & Speaker | ---------------------------------------"));
    Serial.println(F("| Camera & Speaker |"));
    Serial.print(F("USE_SPEAKER: "));
  #ifdef USE_SPEAKER
    Serial.println(F("True"));
  #else // USE_SPEAKER
    Serial.println(F("False"));
  #endif // USE_SPEAKER
    Serial.print(F("AUDIO_FILE_LENGTH: "));
  #ifdef AUDIO_FILE_LENGTH_4_2
    Serial.println(F("AUDIO_FILE_LENGTH_4_2"));
  #elif defined(GPS_POWER_ON_OAUDIO_FILE_LENGTH_9_4FF_MODE)
    Serial.println(F("AUDIO_FILE_LENGTH_9_4"));
  #elif defined(AUDIO_FILE_LENGTH_14_6)
    Serial.println(F("AUDIO_FILE_LENGTH_14_6"));
  #endif
    Serial.print(F("SPEAKER_REST_TIME_SEC: ")); Serial.println(SPEAKER_REST_TIME_SEC);
    Serial.print(F("CAMERA_REST_TIME_SEC: ")); Serial.println(CAMERA_REST_TIME_SEC);
    Serial.print(F("CAMERA_REST_TIME_SEC_INIT: ")); Serial.println(CAMERA_REST_TIME_SEC_INIT);
    Serial.print(F("CAMERA_RECORD_TIME_SEC: ")); Serial.println(CAMERA_RECORD_TIME_SEC);
    Serial.print(F("FINAL_CHECK_STAND_BY_PHASE_SEC: ")); Serial.println(FINAL_CHECK_STAND_BY_PHASE_SEC);
    Serial.print(F("FINAL_CHECK_PHASE_SEC: ")); Serial.println(FINAL_CHECK_PHASE_SEC);
    Serial.print(F("MINIMUM_SEC_AFTER_PLAY_AUDIO: ")); Serial.println(MINIMUM_SEC_AFTER_PLAY_AUDIO);
    
    Serial.print(F("DEBUG_SAMPLING_TEST: "));
  #ifdef DEBUG_SAMPLING_TEST
    Serial.println(F("True"));
    Serial.print(F("DEBUG_SAMPLING_TEST_INTERVAL_MIN: ")); Serial.println(DEBUG_SAMPLING_TEST_INTERVAL_MIN);
    Serial.print(F("NUM_VIDEOS_FOR_DEBUG_SAMPLING_TEST: ")); Serial.println(NUM_VIDEOS_FOR_DEBUG_SAMPLING_TEST);
  #else // DEBUG_SAMPLING_TEST
    Serial.println(F("False"));
  #endif // DEBUG_SAMPLING_TEST
    Serial.println(F("------------------------------------------------------------"));
    // Serial.println(F(" "));
    // Serial.print(F("Press Enter Key to start !"));
    // while (Serial.available() == false) {
    //   // wait until pressing Enter key or any
    // }
    // Serial.println(F(" -> Start !"));
    // Serial.println(F(" "));
    // Serial.flush();
  }
}


void write_settings_txt() 
{
  // Delete settings.txt if exists
  if (sd.exists("settings.txt")) sd.remove("settings.txt");

  // Write settings to a new settings.txt file
  File settingsFile = sd.open("settings.txt", FILE_WRITE);
  if (settingsFile) 
  {
  
    // Write the settings
    settingsFile.print(F("Connected to PC -> settingsFile Ready at ")); 
    settingsFile.println(F("115200"));
    // settingsFile.println(F("9600"));

    // for debug
    settingsFile.print("VBUS_state: "); settingsFile.println(VBUS_state);

    settingsFile.print(F("Compiled on: ")); 
    settingsFile.print(F(__DATE__)); 
    settingsFile.print(F(" ")); 
    settingsFile.println(F(__TIME__));
    settingsFile.println(F(" "));
    settingsFile.println(F("-------------------- logbot-v5 settings --------------------"));

    settingsFile.print(F("Base Mode: "));
  #ifdef UMINEKO_2024_MODE
    settingsFile.println(F("UMINEKO_2024_MODE"));
  #elif defined(UMINEKO_2024_DAY_MODE)
    settingsFile.println(F("UMINEKO_2024_DAY_MODE"));
  #elif defined(UMINEKO_2024_NIGHT_MODE)
    settingsFile.println(F("UMINEKO_2024_NIGHT_MODE"));
  #elif defined(DEBUG_UMINEKO_2024_MODE)
    settingsFile.println(F("DEBUG_UMINEKO_2024_MODE"));
  #elif defined(INTERVAL_SAMPLING_MODE)
    settingsFile.println(F("DEBUG_UMINEKO_2024_MODE"));
  #endif
    settingsFile.println(F("------------------------------------------------------------"));
    
    // settingsFile.println(F("------------------------------------------------------------"));
    // settingsFile.println(F("| Debug | --------------------------------------------------"));
    settingsFile.println(F("| Debug |"));
    settingsFile.print(F("DEBUG_SERIAL_PRINT: "));
  #ifdef DEBUG_SERIAL_PRINT
    settingsFile.println(F("True"));
  #else // DEBUG_SERIAL_PRINT
    settingsFile.println(F("False"));
  #endif // DEBUG_SERIAL_PRINT

    settingsFile.print(F("DEBUG_FAKE_GPS_IN_SETUP: "));
  #ifdef DEBUG_FAKE_GPS_IN_SETUP
    settingsFile.println(F("True"));
  #else // DEBUG_FAKE_GPS_IN_SETUP
    settingsFile.println(F("False"));
  #endif // DEBUG_FAKE_GPS_IN_SETUP
    settingsFile.print(F("DEBUG_FAKE_GPS_IN_LOOP: "));
  #ifdef DEBUG_FAKE_GPS_IN_LOOP
    settingsFile.println(F("True"));
  #else // DEBUG_FAKE_GPS_IN_LOOP
    settingsFile.println(F("False"));
  #endif // DEBUG_FAKE_GPS_IN_LOOP

    settingsFile.print(F("RTC_1HZ_CLOCK_PIN_USE_MODE: ")); settingsFile.println(RTC_1HZ_CLOCK_PIN_USE_MODE);

    // settingsFile.println(F("------------------------------------------------------------"));
    // settingsFile.println(F("| Sleep Control | ------------------------------------------"));
    settingsFile.println(F("| Sleep Control |"));
    settingsFile.print(F("STARTUP_DELAY_MIN: ")); settingsFile.println(STARTUP_DELAY_MIN);
    settingsFile.print(F("LOGGING_START_JST_HOUR: ")); settingsFile.println(LOGGING_START_JST_HOUR);
    settingsFile.print(F("LOGGING_START_JST_MIN: ")); settingsFile.println(LOGGING_START_JST_MIN);
    settingsFile.print(F("DAWN: ")); settingsFile.print(DAWN); settingsFile.print(F(" (")); settingsFile.print(DAWN/60); settingsFile.println(F(")"));
    settingsFile.print(F("DUSK: ")); settingsFile.print(DUSK); settingsFile.print(F(" (")); settingsFile.print(DUSK/60); settingsFile.println(F(")"));

    // settingsFile.println(F("------------------------------------------------------------"));
    // settingsFile.println(F("| SD | -----------------------------------------------------"));
    settingsFile.println(F("| SD |"));
    settingsFile.print(F("MAKE_SAVE_DIRS: "));
  #ifdef MAKE_SAVE_DIRS
    settingsFile.println(F("True"));
  #else // MAKE_SAVE_DIRS
    settingsFile.println(F("False"));
  #endif // MAKE_SAVE_DIRS
    settingsFile.print(F("DATA_BUFFER_SIZE_SEC: ")); settingsFile.println(DATA_BUFFER_SIZE_SEC);
    

    // GPS Mode
    // settingsFile.println(F("------------------------------------------------------------"));
    // settingsFile.println(F("| GPS | --------------------------------------------"));
    settingsFile.println(F("| GPS |"));
    settingsFile.print(F("GPS Mode: "));
  #ifdef GPS_CONTINUOUS_1HZ_MODE
    settingsFile.println(F("GPS_CONTINUOUS_1HZ_MODE"));
  #elif defined(GPS_POWER_ON_OFF_MODE_01ON_02OFF)
    settingsFile.println(F("GPS_POWER_ON_OFF_MODE_01ON_02OFF"));
  #elif defined(GPS_POWER_ON_OFF_MODE_02ON_13OFF)
    settingsFile.println(F("GPS_POWER_ON_OFF_MODE_02ON_13OFF"));
  #elif defined(GPS_POWER_ON_OFF_MODE_03ON_27OFF)
    settingsFile.println(F("GPS_POWER_ON_OFF_MODE_03ON_27OFF"));
  #endif
    settingsFile.print(F("GPS_1HZ_RECORD_AFTER_START_COUNTDOWN_SEC: ")); settingsFile.println(GPS_1HZ_RECORD_AFTER_START_COUNTDOWN_SEC);
    settingsFile.print(F("FORCE_GPS_1HZ_MODE_SEC: ")); settingsFile.println(FORCE_GPS_1HZ_MODE_SEC);
    settingsFile.print(F("FORCE_GPS_1HZ_MODE_INIT_SEC: ")); settingsFile.println(FORCE_GPS_1HZ_MODE_INIT_SEC);
    settingsFile.print(F("STOP_GPS_1HZ_MODE_IF_BIRD_IS_NOT_FLYING: "));
  #ifdef STOP_GPS_1HZ_MODE_IF_BIRD_IS_NOT_FLYING
    settingsFile.println(F("True"));
  #else // 
    settingsFile.println(F("False"));
  #endif // 
    settingsFile.print(F("STOP_GPS_1HZ_MODE_IF_BIRD_IS_NOT_FLYING_START_SEC: ")); settingsFile.println(STOP_GPS_1HZ_MODE_IF_BIRD_IS_NOT_FLYING_START_SEC);

    // settingsFile.println(F("------------------------------------------------------------"));
    // settingsFile.println(F("| Behaviour Recognition | ----------------------------------"));
    settingsFile.println(F("| Behaviour Recognition |"));
    USE_LOCATION_BOUNDING_BOX_TRIGGER
    settingsFile.print(F("CAM_SPK_GPS_CONTROL_VERSION: ")); settingsFile.println(CAM_SPK_GPS_CONTROL_VERSION);
    settingsFile.print(F("TRIGGER_TYPE: ")); settingsFile.println(TRIGGER_TYPE);
    settingsFile.print(F("TARGET_BEHAVIOR_CLASS_NUMBER: ")); settingsFile.println(TARGET_BEHAVIOR_CLASS_NUMBER);
    settingsFile.print(F("TARGET_BEHAVIOR_CONSECUTIVE_SEC: ")); settingsFile.println(TARGET_BEHAVIOR_CONSECUTIVE_SEC);
    settingsFile.print(F("BEHAVIOR_CLASS_BUFFER_SIZE: ")); settingsFile.println(BEHAVIOR_CLASS_BUFFER_SIZE);
    settingsFile.print(F("MINIMUM_TARGET_BEHAVIOR_COUNT: ")); settingsFile.println(MINIMUM_TARGET_BEHAVIOR_COUNT);
    settingsFile.print(F("GPS_FIX_TYPE_BUFFER_SIZE: ")); settingsFile.println(GPS_FIX_TYPE_BUFFER_SIZE);
    settingsFile.print(F("OUTSIDE_BOUNDING_BOX_BUFFER_SIZE: ")); settingsFile.println(OUTSIDE_BOUNDING_BOX_BUFFER_SIZE);
    settingsFile.print(F("USE_LOCATION_BOUNDING_BOX_TRIGGER: "));
  #ifdef USE_LOCATION_BOUNDING_BOX_TRIGGER
    settingsFile.println(F("True"));
  #else // USE_LOCATION_BOUNDING_BOX_TRIGGER
    settingsFile.println(F("False"));
  #endif // USE_LOCATION_BOUNDING_BOX_TRIGGER
    settingsFile.print(F("Bounding Box:"));
    settingsFile.print(F(" NE ")); settingsFile.print(AREA_NE_LAT, 5); settingsFile.print(F(", ")); settingsFile.print(AREA_NE_LON, 5);
    settingsFile.print(F(" SW ")); settingsFile.print(AREA_SW_LAT, 5); settingsFile.print(F(", ")); settingsFile.print(AREA_SW_LON, 5);
    settingsFile.println(F(""));

    // settingsFile.println(F("------------------------------------------------------------"));
    // settingsFile.println(F("| Camera & Speaker | ---------------------------------------"));
    settingsFile.println(F("| Camera & Speaker |"));
    settingsFile.print(F("USE_SPEAKER: "));
  #ifdef USE_SPEAKER
    settingsFile.println(F("True"));
  #else // USE_SPEAKER
    settingsFile.println(F("False"));
  #endif // USE_SPEAKER
    settingsFile.print(F("AUDIO_FILE_LENGTH: "));
  #ifdef AUDIO_FILE_LENGTH_4_2
    settingsFile.println(F("AUDIO_FILE_LENGTH_4_2"));
  #elif defined(GPS_POWER_ON_OAUDIO_FILE_LENGTH_9_4FF_MODE)
    settingsFile.println(F("AUDIO_FILE_LENGTH_9_4"));
  #elif defined(AUDIO_FILE_LENGTH_14_6)
    settingsFile.println(F("AUDIO_FILE_LENGTH_14_6"));
  #endif
    settingsFile.print(F("SPEAKER_REST_TIME_SEC: ")); settingsFile.println(SPEAKER_REST_TIME_SEC);
    settingsFile.print(F("CAMERA_REST_TIME_SEC: ")); settingsFile.println(CAMERA_REST_TIME_SEC);
    settingsFile.print(F("CAMERA_REST_TIME_SEC_INIT: ")); settingsFile.println(CAMERA_REST_TIME_SEC_INIT);
    settingsFile.print(F("CAMERA_RECORD_TIME_SEC: ")); settingsFile.println(CAMERA_RECORD_TIME_SEC);
    settingsFile.print(F("FINAL_CHECK_STAND_BY_PHASE_SEC: ")); settingsFile.println(FINAL_CHECK_STAND_BY_PHASE_SEC);
    settingsFile.print(F("FINAL_CHECK_PHASE_SEC: ")); settingsFile.println(FINAL_CHECK_PHASE_SEC);
    settingsFile.print(F("MINIMUM_SEC_AFTER_PLAY_AUDIO: ")); settingsFile.println(MINIMUM_SEC_AFTER_PLAY_AUDIO);
    
    settingsFile.print(F("DEBUG_SAMPLING_TEST: "));
  #ifdef DEBUG_SAMPLING_TEST
    settingsFile.println(F("True"));
    settingsFile.print(F("DEBUG_SAMPLING_TEST_INTERVAL_MIN: ")); settingsFile.println(DEBUG_SAMPLING_TEST_INTERVAL_MIN);
    settingsFile.print(F("NUM_VIDEOS_FOR_DEBUG_SAMPLING_TEST: ")); settingsFile.println(NUM_VIDEOS_FOR_DEBUG_SAMPLING_TEST);
  #else // DEBUG_SAMPLING_TEST
    settingsFile.println(F("False"));
  #endif // DEBUG_SAMPLING_TEST
    
  }
  
  settingsFile.close();

}


void start_I2C_connection() 
{
  Wire.begin();
  Wire.setClock(100000L); // 100kHz as default
  // Wire.setClock(400000L); // 400kHz
  delay(200);
}




bool protectedWrite(int pin, int val) 
{
  if (val == HIGH) {
    switch (pin) {
      case 0:
      case 1:
      case 2:
      case 5:
      case 6:
      case 7:
      case 8:
      case 9:
      case 10:
      case 11:
      case 12:
      case 13:
      case 14:
      case A1:
      case A2:
      case A3:
      case A4:
      case A5:
      case A6:
      case A7:
      case 22:
      case 23:
      case 24:
      case 25:
      case 26:
      case 27:
      case 28:
      case 29:
      case 30:
      case 31:
      case 32:
      case 33:
        return false;
        break;
      default:
        digitalWrite(pin, val);
        return true;
        break;
    }
  }
  if (val == LOW) {
    switch (pin) {
      case A3:
      case A4:
        return false;
        break;
      default:
        digitalWrite(pin, val);
        return true;
        break;
    }
  }
  return false;
}


// LED functions
void strobe(const uint16_t &on, const uint16_t &off, const bool &green) 
{
  if (green) {
    protectedWrite(GREEN_LED_PIN, HIGH);
    delay(on);
    protectedWrite(GREEN_LED_PIN, LOW);
  }
  else {
    protectedWrite(YELLOW_LED_PIN, HIGH);
    delay(on);
    protectedWrite(YELLOW_LED_PIN, LOW);
  }
  if (off > 0) {
    delay(off);  
  }
}

void count_down_strobe(const uint8_t &n) 
{
  for (uint8_t i = n; i > 0; i--) {
    delay(100);
    for (uint8_t j = i; j > 1; j--) {
      strobe(25, 25, true);
    }
    strobe(25, 0, true);
  }
}

void green_led_blink(uint16_t msec1, uint16_t msec2, uint8_t count) 
{
  for (uint8_t i = 0; i < count; i++) {
    strobe(msec1, msec2, true);
  }
}

void yellow_led_blink(uint16_t msec1, uint16_t msec2, uint8_t count) 
{
  for (uint8_t i = 0; i < count; i++) {
    strobe(msec1, msec2, false);
  }
}

void countdown_green_led_blink_before_start()
{
  #ifdef DEBUG_SERIAL_PRINT
    if (connected_to_pc) Serial.print("Countdown: "); 
  #endif // DEBUG_SERIAL_PRINT

  uint8_t num = 5;
  for (int i = 0; i < num; i++)
  {
    green_led_blink(300, 700, 1); // wait for 5 sec
  #ifdef DEBUG_SERIAL_PRINT
    if (connected_to_pc) 
    {
      Serial.print(num - i); 
      Serial.print(" ");
      if (i == num - 1)
      {
        Serial.println("");
      } 
    }
  #endif // DEBUG_SERIAL_PRINT
  }
}


void debug_1hz_green_led_blink(bool led_on)
{
#ifdef DEBUG_1HZ_LED
  if (led_on) protectedWrite(GREEN_LED_PIN, HIGH);
  else protectedWrite(GREEN_LED_PIN, LOW);
#endif // DEBUG_1HZ_LED  
}


void debug_GPS_green_led_blink(bool led_on)
{
#ifdef DEBUG_GPS_LED
  if (gps_location_updated)
  {
    if (led_on) protectedWrite(GREEN_LED_PIN, HIGH);
    else protectedWrite(GREEN_LED_PIN, LOW);
  }
#endif // DEBUG_GPS_LED  
}


void debug_detection_yellow_led_blink(bool led_on)
{
#ifdef DEBUG_DETECTION_LED
  if (behavior_class_int == TARGET_BEHAVIOR_CLASS_NUMBER)
  {
    if (led_on) protectedWrite(YELLOW_LED_PIN, HIGH);
    else protectedWrite(YELLOW_LED_PIN, LOW);
  }
#endif // DEBUG_DETECTION_LED
}



void set_up_pinMode_GPIO_port() {
  // LED
  pinMode(GREEN_LED_PIN, OUTPUT);
  pinMode(YELLOW_LED_PIN, OUTPUT);
  protectedWrite(GREEN_LED_PIN, LOW);
  protectedWrite(YELLOW_LED_PIN, LOW);
  // Battery
  pinMode(BATTERY_MES_PIN, OUTPUT);
  delay(20);
  digitalWrite(BATTERY_MES_PIN, LOW);
  delay(20);
  // RTC
  if (RTC_1HZ_CLOCK_PIN_USE_MODE > 0) pinMode(RTC_CLOCK_1HZ, INPUT); // this may cause some RTC bug 
  // GPS
  pinMode(GPS_PWR_PIN, OUTPUT);
  delay(20);
  digitalWrite(GPS_PWR_PIN, LOW);
  delay(20);
  // ESP32 on main board
  pinMode(ESP32_PWR_PIN, OUTPUT); 
  delay(20);
  digitalWrite(ESP32_PWR_PIN, LOW);
  delay(20);
  // VCAM ESP32
  pinMode(VCAM_ESP32_PWR_PIN, OUTPUT); 
  delay(20);
  digitalWrite(VCAM_ESP32_PWR_PIN, LOW);
  delay(20);
  // Port for checking external board connection
  pinMode(EXT_BOARD_CONNECT_PIN, INPUT); 
  delay(100);
  // SD
  pinMode(SD_PWR_PIN, OUTPUT); 
  delay(20);
  pinMode(SD_CMD_PIN, OUTPUT);
  delay(20); 
  pinMode(SD_CLK_PIN, OUTPUT); 
  delay(20);
  pinMode(SD_DAT0_PIN, OUTPUT);
  delay(20);
  pinMode(SD_CD_PIN, INPUT);
  delay(20);
  digitalWrite(SD_PWR_PIN, LOW);
  delay(20);
  digitalWrite(SD_CMD_PIN, LOW);
  delay(20);
  digitalWrite(SD_CLK_PIN, LOW);
  delay(20);
  digitalWrite(SD_DAT0_PIN, LOW);
  delay(20);
}


void check_battery_status_at_start(int16_t &d_battery_level, uint8_t sample_size) 
{
  // Check the battery level several times here and calculate the average value.
  // If the battery voltage is low, stop the process here.
  float battery_level_v_array[sample_size];
  float sum = 0;
  float average = 0;
#ifdef DEBUG_SERIAL_PRINT
  if (connected_to_pc) Serial.print(F("Checking Battery Level")); 
#endif // DEBUG_SERIAL_PRINT
  digitalWrite(BATTERY_MES_PIN, HIGH);
  delay(1000);
  for (uint8_t i = 0; i < sample_size; i++)
  { 
  #ifdef DEBUG_SERIAL_PRINT
    if (connected_to_pc) Serial.print(F(" .")); 
  #endif // DEBUG_SERIAL_PRINT
    battery_level = analogRead(BATTERY_VOLT_PIN);
    battery_level_v = 0.00703125 * battery_level;
    battery_level_v_array[i] = battery_level_v;
  // #ifdef DEBUG_SERIAL_PRINT
  //   if (connected_to_pc) 
  //   {
  //     Serial.print("battery_level_v: "); Serial.println(battery_level_v);
  //   }
  // #endif // DEBUG_SERIAL_PRINT
    // delay(200);
    yellow_led_blink(100, 100, 1);
  }
  
  // Sum the elements in the array
  for (uint8_t i = 0; i < sample_size; i++) {
    sum += battery_level_v_array[i];
  }

  average = sum / sample_size;

  digitalWrite(BATTERY_MES_PIN, LOW); // turn off BATTERY_MES_PIN

  d_battery_level = average; // other_data.battery_level

#ifdef DEBUG_SERIAL_PRINT
  if (connected_to_pc) 
  {
    Serial.println(""); 
    Serial.print("Average battery_level_v = "); 
    Serial.print(average);
    Serial.print(" (N="); Serial.print(sample_size); Serial.println(") ");
    if (average < 3.5) 
    {
      Serial.println(" -> Low battery! Charge the device!");
      while (average < 3.5)
      {
        yellow_led_blink(200, 800, 1);
        green_led_blink(200, 800, 1);
      }
    }
    else 
    {
      Serial.println("-> OK!");
    }
  }
#endif // DEBUG_SERIAL_PRINT



}

void check_battery_status_v2(int16_t &d_battery_level) 
{
  // Set BATTERY_MES_PIN to HIGH at 59 seconds, 
  // and measure battery voltage at 00 seconds. Do nothing at other times.
  if (current_sec == 59) 
  {
    digitalWrite(BATTERY_MES_PIN, HIGH);\
  }
  else if (current_sec == 0) 
  {
    battery_level = analogRead(BATTERY_VOLT_PIN);
    battery_level_v = 0.00703125 * battery_level;
    // #ifdef DEBUG_SERIAL_PRINT
    // if (connected_to_pc) 
    // {
    //   Serial.print("battery_level_v: "); Serial.println(battery_level_v);
    // }
    // #endif // DEBUG_SERIAL_PRINT
    digitalWrite(BATTERY_MES_PIN, LOW); // turn off BATTERY_MES_PIN
  }

  d_battery_level = battery_level; // other_data.battery_level

#ifdef DEBUG_SERIAL_PRINT
  battery_level_percent = (int) ((battery_level_v / BATTERY_LEVEL_V_MAX) * 100); 
  if (battery_level_percent >= 100) 
  {
    battery_level_percent = 100;
  }
#endif // DEBUG_SERIAL_PRINT
}

void print_debug_message_if_connected_to_PC(
  const rtc1_t &rtc_time,
  const gps_t &gps_data, 
  const data1_t &imu_data,
  const data2_t &other_data
) 
{
  if (connected_to_pc) {
    // Serial.println(F("--------------------------------------------------------------------"));
    // yy/mm/dd hh:mm:ss
    char rtc_datetime_array[42];
    sprintf(
      rtc_datetime_array, 
      "20%02d/%02d/%02d %02d:%02d:%02d", 
      rtc_time.year, 
      rtc_time.month, 
      rtc_time.day, 
      rtc_time.hour, 
      rtc_time.min, 
      rtc_time.sec);
    Serial.print(F("RTC | Date Time: ")); Serial.print(rtc_datetime_array);
    Serial.println(F(""));

    // GPS GNSS
    char gps_data_time_array[20];
    sprintf(
      gps_data_time_array, 
      "%02d:%02d:%02d", 
      gps_data.gps_hour, 
      gps_data.gps_min, 
      gps_data.gps_sec);
    Serial.print(F("GPS |")); 
    Serial.print(F(" Time: ")); Serial.print(gps_data_time_array);
    Serial.print(F(" |"));

    Serial.print(F(" Lat: ")); Serial.print(gps_data.latitude, 6); 
    Serial.print(F(" Lon: ")); Serial.print(gps_data.longitude, 6); 
    Serial.print(F(" Alt: ")); Serial.print(gps_data.altitude, 3); 
    Serial.print(F(" Status: ")); Serial.print(gps_data.fix_type); 
    Serial.print(F(" SIV: ")); Serial.print(gps_data.siv); // SIV: the number of satellites used in fix
    Serial.println(F(""));

    Serial.print(F("GPS |")); 
    Serial.print(F(" updated: ")); Serial.print(gps_location_updated);
    Serial.print(F(" force_gps_1hz_mode: ")); Serial.print(force_gps_1hz_mode);
    Serial.print(F(" init countdown: ")); Serial.print(force_gps_1hz_mode_init_countdown_sec);
    Serial.print(F(" 1 Hz countdown: ")); Serial.print(force_gps_1hz_mode_countdown_sec);
    Serial.println(F(""));
    
    // Acc, Gyro, Mag Debug
    Serial.print(F("IMU | ")); 
    Serial.print("Acc (g)");  
    Serial.print(" x: "); Serial.print(imu_data.acc[0]); 
    Serial.print(" y: "); Serial.print(imu_data.acc[1]); 
    Serial.print(" z: "); Serial.print(imu_data.acc[2]); Serial.print(F(" | "));
    Serial.print("Gyro (deg/sec)");  
    Serial.print(" x: "); Serial.print(imu_data.gyro[0]); 
    Serial.print(" y: "); Serial.print(imu_data.gyro[1]); 
    Serial.print(" z: "); Serial.print(imu_data.gyro[2]); Serial.print(F(" | "));
    Serial.print("Mag (uT)");  
    Serial.print(" x: "); Serial.print(imu_data.mag[0]); 
    Serial.print(" y: "); Serial.print(imu_data.mag[1]); 
    Serial.print(" z: "); Serial.print(imu_data.mag[2]);
    Serial.println(F(""));

    // Water Pressure 
    // Serial.print(F("Water Pressure | ")); 
    // Serial.print("D1: ");  Serial.print(other_data.D1); Serial.print(" ");
    // Serial.print("D2: ");  Serial.print(other_data.D2); Serial.println(" ");
    // Serial.print(F(" 0: ")); 
    // Serial.print(water_pressure_buffer[0], 0);
    // Serial.print(F(" 1: ")); 
    // Serial.print(water_pressure_buffer[1], 0);
    // Serial.print(F(" 2: ")); 
    // Serial.println(water_pressure_buffer[2], 0);

    // Serial.print(F("water_pressure_index: ")); 
    // Serial.println(water_pressure_index);
    // Serial.print(F("Behavior class: ")); 
    // Serial.println(other_data.behavior_class); 
    
    
    // Camera monitoring
    Serial.print(F("CAM |")); 
    Serial.print(F(" command: ")); Serial.print(other_data.camera_command); 
    Serial.print(F(" recording:   ")); Serial.print(other_data.camera_recording);
    Serial.print(F(" count:   ")); Serial.print(other_data.camera_count);
    Serial.print(F(" recording time: ")); Serial.print(camera_record_time_countdown_sec);
    Serial.print(F(" rest time: ")); Serial.print(camera_rest_time_countdown_sec); 
    Serial.println(F(""));

    // Speaker monitoring
    Serial.print(F("SPK |")); 
    Serial.print(F(" play:    ")); Serial.print(other_data.play_audio); 
    Serial.print(F(" speaker on:  ")); Serial.print(other_data.speaker_on);
    Serial.print(F(" file:  ")); Serial.print(other_data.audio_file);
    Serial.print(F(" played?:  ")); Serial.print(audio_played);
    Serial.print(F(" countdown: ")); Serial.print(speaker_turn_on_countdown_sec);
    Serial.print(F(" rest time: ")); Serial.print(speaker_rest_time_countdown_sec); 
    Serial.println(F(""));

    Serial.print(F("OTR | ")); // Others
    Serial.print(F("behaviour class: ")); Serial.print(other_data.behavior_class); Serial.print(F(" | "));
    // Serial.print(F("final check: ")); Serial.print(final_check_target_behavior_count); 
    // Serial.print(F("/")); Serial.print(SPEAKER_TURN_ON_BEHAVIOR_COUNT); Serial.print(F(" | "));
    // Serial.print(F("Battery: ")); 
    Serial.print("battery: "); Serial.print(battery_level_v); Serial.print(" (v) | ");
    // Serial.print(battery_level_percent); Serial.print(F("% | "));
    Serial.print(F("illuminance: ")); Serial.print(other_data.illuminance); Serial.print(F(" | "));
    Serial.println(F(""));

    // Process Time
    unsigned long adjustment_time = (3 * 25); // 3 (40 ms) * 25
    delay_time_total = 1000 - (process_time_total_copy + adjustment_time);
    Serial.print(F("TIM | "));
    Serial.print(F("total process: ")); Serial.print(process_time_total_copy); Serial.print(F(" | "));
    Serial.print(F("adjust time: ")); Serial.print(adjustment_time); Serial.print(F(" | "));
    Serial.print(F("delay: ")); Serial.print(delay_time_total); Serial.print(F(" | ")); 
    Serial.print(F("sd write: ")); Serial.print(process_time_sd); Serial.print(F(" | "));
    // Serial.print(F("loop2: ")); Serial.print(process_time_loop2); Serial.print(F(" (ms) | "));
    // total delay time from main loop block 0, 1, 2, ..., 24 + loop_sd
    Serial.print(F(" (ms) | ")); Serial.println(F(""));

  Serial.println(F("---------------------------------------------------------------------------------------------------------------------------"));
  }
}


void initialize_log_file_in_sd() {
  File logFile = sd.open("gps_rtc_init_log.txt", FILE_WRITE);
  if (logFile) {
    logFile.println(F("--------------------------------------------------------------------"));
    logFile.println(F("Start")); 
    logFile.println(F("--------------------------------------------------------------------"));
  }
  logFile.close();
  
}

void save_log_data_to_sd_v2(  
  const rtc1_t &rtc_time, 
  const rtc1_t &gps_time,
  const gps_t &gps_data
  )
  {
    File logFile = sd.open("gps_rtc_init_log.txt", FILE_WRITE);
    if (logFile)
    {
      logFile.println(F("-----------------------------------------------------------------------------------------"));
      // RTC
      // yy/mm/dd hh:mm:ss
      char rtc_datetime_array[40];
      sprintf(
        rtc_datetime_array, 
        "20%02d/%02d/%02d %02d:%02d:%02d", 
        rtc_time.year, 
        rtc_time.month, 
        rtc_time.day, 
        rtc_time.hour, 
        rtc_time.min, 
        rtc_time.sec
      );
      logFile.print(F("RTC Time: ")); 
      logFile.println(rtc_datetime_array);

      // GPS Data
      char gps_data_time_array[40];
      sprintf(
        gps_data_time_array, 
        "%04d/%02d/%02d %02d:%02d:%02d", 
        gps_time.year, 
        gps_time.month, 
        gps_time.day, 
        gps_time.hour, 
        gps_time.min, 
        gps_time.sec
      );
      logFile.print(F("GPS Time: ")); 
      logFile.print(gps_data_time_array);
      logFile.print(" ");
      logFile.print(F(" Lat: ")); logFile.print(gps_data.latitude, 4); 
      logFile.print(F(" Lon: ")); logFile.print(gps_data.longitude, 4); 
      logFile.print(F(" Alt: ")); logFile.print(gps_data.altitude, 3); 
      logFile.print(F(" Status: ")); logFile.print(gps_data.fix_type); 
      logFile.print(F(" SIV: ")); logFile.print(gps_data.siv); 
      logFile.println();
    }
  logFile.close();
  }

// ----------------------------
// I2C functions
// ----------------------------
uint8_t I2C_read_byte(const uint8_t &address, 
                      const uint8_t &subaddress)
{
  uint8_t data[1] = {0};
  I2C_read(address, subaddress, 1, data);
  return data[0];  
}

uint8_t I2C_read(
  const uint8_t &address, 
  const uint8_t &n_bytes, 
  uint8_t data[])
{
  uint8_t bytes_read = 0;
  
  // Read Nbytes
  Wire.requestFrom(address, n_bytes); 
  for (uint8_t i = 0; i < n_bytes; i++) {
    if (!Wire.available()) {
      delay(5);  
    }
    if (Wire.available()) {
      data[i]=Wire.read();
      bytes_read++;
    }
    else {
      break;
    }
  }

  return bytes_read;
}

uint8_t I2C_read(
  const uint8_t &address, 
  const uint8_t &subaddress, 
  const uint8_t &n_bytes, 
  uint8_t data[])
{
  uint8_t bytes_read = 0;
  I2C_write_byte_2(address, subaddress);
  return I2C_read(address, n_bytes, data);
}

// Write a byte (Data) in device (Address) at register (Register)
void I2C_write_byte(
  const uint8_t &address, 
  const uint8_t &subaddress, 
  const uint8_t &data) {
  // Serial.println("I2C_write_byte 1");
  // Set register address
  Wire.beginTransmission(address);
  Wire.write(subaddress);
  Wire.write(data);
  Wire.endTransmission();
}

// renamed the function
void I2C_write_byte_2(const uint8_t &address, const uint8_t &data) 
{
  Wire.beginTransmission(address); // Set register address
  Wire.write(data);
  Wire.endTransmission();
}

// ----------------------
// Speaker functions
// ----------------------
void select_audio_file2_v2() { // Used for LBP08 and LBP09
  if (random_audio_file == true) 
  { // randomly select audio file
    audio_file_int = random(1, 2+1); // 1 or 2
  }
  else 
  { // alternatively select audio file
    if (last_played_audio_file_int == 2) {
      audio_file_int = 1; // Falcon, Hayabusa
    }
    else {
      audio_file_int = 2; // Noise
    }
  }
#ifdef DEBUG_SERIAL_PRINT
  Serial.print("| select_audio_file2_v2() -> "); Serial.print(audio_file_int); Serial.println(" |");
#endif // DEBUG_SERIAL_PRINT
}


void select_audio_file2() { // Used for LBP01 and LBP07
  if (random_audio_file == true) 
  { // randomly select audio file
  if (current_sec % 2 == 1) 
  { // Odd number, return 1 (Falcon, Hayabusa)
    audio_file_int = 1;
  }
  else
  { // Even number, return 2 (White noise)
    audio_file_int = 2;
  }
  #ifdef DEBUG_SAMPLING_TEST
    // Use random function
    audio_file_int = random(1, 2+1); // 1 or 2
  #endif // DEBUG_SAMPLING_TEST
  }
  else 
  { // alternatively select audio file
    if (last_played_audio_file_int == 2) {
      audio_file_int = 1; // Falcon, Hayabusa
    }
    else {
      audio_file_int = 2; // Noise
    }
  }
#ifdef DEBUG_SERIAL_PRINT
  Serial.print("| select_audio_file2() -> "); Serial.print(audio_file_int); Serial.println(" |");
#endif // DEBUG_SERIAL_PRINT
}


void select_audio_file3() {
  if (random_audio_file == true) 
  { // randomly select audio file
    if (current_sec % 3 == 1) 
    {
      audio_file_int = 1;
    }
    else if (current_sec % 3 == 2) 
    {
      audio_file_int = 2;
    }
    else 
    {
      audio_file_int = 3;
      final_check_target_behavior_count = 0; // reset
    }
  #ifdef DEBUG_SAMPLING_TEST
    // Use random function
    audio_file_int = random(1, 3+1);
  #endif // DEBUG_SAMPLING_TEST
  }
  else 
  { // alternatively select audio file
    if (last_played_audio_file_int == 2) {
      audio_file_int = 1; // Hayabusa
    }
    else {
      audio_file_int = 2; // Noise
    }
  }
}


/*
 * SD_ReadWrite
 */

void set_up_SD() {
  // this pin must be set high 
  // before using the SD library or setting pins 50-53 high, 
  // a short may occur otherwise
  digitalWrite(SD_PWR_PIN, HIGH); 
  delay(1000);
  while (!sd.begin(SD_CD_PIN)) {
    strobe(1000, 500, false);
    strobe(1000, 500, false);
  }
  SD_power_on = true;
#ifdef DEBUG_SERIAL_PRINT
  if (connected_to_pc) {
    Serial.println("SD Cards: OK");
  }
#endif // DEBUG_SERIAL_PRINT
  yellow_led_blink(100, 100, 3);
}

void start_SD() {
  if (!SD_power_on) {
    digitalWrite(SD_PWR_PIN, HIGH); 
    delay(100);
    while (!sd.begin()) {
      strobe(1000, 500, false);
      strobe(1000, 500, false);
    }
    SD_power_on = true;
  }
}

void stop_SD() {
  if (SD_power_on) {
    sd.end();
    digitalWrite(SD_PWR_PIN, LOW);
    delay(100);
    digitalWrite(SD_CMD_PIN, LOW);
    digitalWrite(SD_CLK_PIN, LOW);
    digitalWrite(SD_CD_PIN, LOW);
    SD_power_on = false;
    #ifdef DEBUG_SERIAL_PRINT
      if (connected_to_pc) {
        Serial.println("turning off SD");
      }
    #endif // DEBUG_SERIAL_PRINT

  }
}


// NOTE: arrays decay to pointers when passed to a function, 
// so sizeof(array) must be called before the function call, not within the function
// Now, this function is called in set_up_water_pressure_sensor() function in WPS_MS5837.ino
void write_C(const uint16_t C[], const uint8_t &array_size)
{
  if (SD_power_on)
  {
    File calibFile = sd.open(F("calib.bin"), FILE_WRITE);
    if(calibFile){
      // uint8_t size = 
      calibFile.write((uint8_t *)C, array_size/sizeof(uint8_t));
      // Serial.println(size);
      calibFile.close();
    }
  }
}


void make_save_dirs(rtc1_t &rtc_time, uint8_t num_dirs) {
  setStartTime(rtc_time);
  // Serial.println(rtc_time);
  time_t startTime = now();

  if (SD_power_on)
  { 
    for (int i = 0; i < num_dirs; i++) {
      // Calculate the next time
      time_t future = startTime + i * 3600;
      
      // Decompose time
      tmElements_t futureTime;
      breakTime(future, futureTime);

      // Create a directory with the specified name
      char dirname[9];
      snprintf(
        dirname, sizeof(dirname), "%02d%02d%02d%02d", 
        futureTime.Year % 100, futureTime.Month, futureTime.Day, futureTime.Hour
      );
      
      if (sd.exists(dirname)) 
      {
        if (connected_to_pc) {
          Serial.print("Dir:  ");
          Serial.print(dirname);
          Serial.println(" already exists.");
        }
      }
      else 
      {
        if (sd.mkdir(dirname)) {
          if (connected_to_pc) {
            Serial.print("Created directory:  ");
            Serial.print(dirname);
            Serial.println("");
          }
        } else {
          if (connected_to_pc) {
            Serial.print("Failed to create directory: ");
            Serial.print(dirname);
            Serial.println("");
          }
        }
      }
    }
  }
}


// Convert from RTC1 structure to TimeLib's tmElements_t structure
tmElements_t rtc1_to_tmElements(const rtc1_t &rtc_time) {
  tmElements_t tm;
  tm.Year = rtc_time.year;
  tm.Month = rtc_time.month;
  tm.Day = rtc_time.day;
  tm.Hour = rtc_time.hour;
  tm.Minute = rtc_time.min;
  tm.Second = rtc_time.sec;
  return tm;
}

// Set the start time
void setStartTime(const rtc1_t &rtc_time) {
  tmElements_t tm = rtc1_to_tmElements(rtc_time);
  // Serial.print(tm.Year); Serial.print(tm.Month); Serial.print(tm.Day); Serial.print(tm.Hour); 
  setTime(makeTime(tm)); // Set the start time using TimeLib's setTime function
}

// used in save_to_sd function
void write_binary_data(
  const char directory[], 
  const char filename[], 
  const rtc1_t &rtc_time, 
  const data1_t &imu_data, 
  const data2_t &other_data, 
  const gps_t &gps_data
)
{          
  if (SD_power_on)
  {
    
    if (current_sec_for_sd_write == 0 || sd_first_write_after_start) 
    { // Check and create the directory only when it's the 0 second or the very first write
      // Open the file
      if (sd.exists(directory))
      {
      #ifdef DEBUG_SERIAL_PRINT
        if (connected_to_pc) Serial.println("The directory already exists.");
      #endif // DEBUG_SERIAL_PRINT
      } 
      else
      {
        sd.mkdir(directory);
      #ifdef DEBUG_SERIAL_PRINT
        if (connected_to_pc) Serial.print("No directory -> Create new directory: "); Serial.println(directory);
      #endif // DEBUG_SERIAL_PRINT
      }

      // binFile = sd.open(filename, FILE_WRITE);
      binFile = sd.open(filename, O_WRITE | O_CREAT | O_AT_END | O_APPEND);
      sd_first_write_after_start = false;
    }

    if (!binFile) binFile = sd.open(filename, O_WRITE | O_CREAT | O_AT_END | O_APPEND);

    if (binFile)
    {
      // rtc_time
      binFile.write((uint8_t *)&rtc_time, sizeof(rtc_time));

      // imu_data
      binFile.write((uint8_t *)&imu_data, sizeof(imu_data));

      // other data
      binFile.write((uint8_t *)&other_data.atmopress, sizeof(other_data.atmopress));
      binFile.write((uint8_t *)&other_data.illuminance, sizeof(other_data.illuminance));
      binFile.write((uint8_t *)&other_data.D1, sizeof(other_data.D1));
      binFile.write((uint8_t *)&other_data.D2, sizeof(other_data.D2));
      binFile.write((uint8_t *)&other_data.battery_level, sizeof(other_data.battery_level)); 
      binFile.write((uint8_t *)&other_data.behavior_class, sizeof(other_data.behavior_class));
      binFile.write((uint8_t *)&other_data.camera_command, sizeof(other_data.camera_command));
      binFile.write((uint8_t *)&other_data.camera_recording, sizeof(other_data.camera_recording));
      binFile.write((uint8_t *)&other_data.camera_count, sizeof(other_data.camera_count));
      binFile.write((uint8_t *)&other_data.play_audio, sizeof(other_data.play_audio));
      binFile.write((uint8_t *)&other_data.speaker_on, sizeof(other_data.speaker_on));
      binFile.write((uint8_t *)&other_data.audio_file, sizeof(other_data.audio_file));
      binFile.write((uint8_t *)&other_data.prev_sd_write_time_ms, sizeof(other_data.prev_sd_write_time_ms));
      binFile.write((uint8_t *)&other_data.delay_occurred_counter, sizeof(other_data.delay_occurred_counter));

      // gps_data
      binFile.write((uint8_t *)&gps_data.gps_hour, sizeof(gps_data.gps_hour));
      binFile.write((uint8_t *)&gps_data.gps_min, sizeof(gps_data.gps_min));
      binFile.write((uint8_t *)&gps_data.gps_sec, sizeof(gps_data.gps_sec));
      binFile.write((uint8_t *)&gps_data.latitude, sizeof(gps_data.latitude));
      binFile.write((uint8_t *)&gps_data.longitude, sizeof(gps_data.longitude));
      binFile.write((uint8_t *)&gps_data.altitude, sizeof(gps_data.altitude));
      binFile.write((uint8_t *)&gps_data.fix_type, sizeof(gps_data.fix_type));
      binFile.write((uint8_t *)&gps_data.siv, sizeof(gps_data.siv));

      // Close the file in SD card when current_sec_for_sd_write == 59, 
      // Otherwise, just keep the file open. (512 bytes buffer)
      if (current_sec_for_sd_write == 59) binFile.close(); // to save the data
    }
    
  #ifdef DEBUG_SERIAL_PRINT
    else 
    {
      // if the file didn't open, print an error:
      if (connected_to_pc) Serial.println("error opening binFile");
    }
  #endif // DEBUG_SERIAL_PRINT
  }
}


void write_binary_data_v2(
  const rtc1_t &rtc_time, 
  const data1_t &imu_data, 
  const data2_t &other_data, 
  const gps_t &gps_data
)
{          

  if (binFile)
  {
    // rtc_time
    binFile.write((uint8_t *)&rtc_time, sizeof(rtc_time));

    // imu_data
    binFile.write((uint8_t *)&imu_data, sizeof(imu_data));

    // other data
    binFile.write((uint8_t *)&other_data.atmopress, sizeof(other_data.atmopress));
    binFile.write((uint8_t *)&other_data.illuminance, sizeof(other_data.illuminance));
    binFile.write((uint8_t *)&other_data.D1, sizeof(other_data.D1));
    binFile.write((uint8_t *)&other_data.D2, sizeof(other_data.D2));
    binFile.write((uint8_t *)&other_data.battery_level, sizeof(other_data.battery_level));
    binFile.write((uint8_t *)&other_data.behavior_class, sizeof(other_data.behavior_class));
    binFile.write((uint8_t *)&other_data.camera_command, sizeof(other_data.camera_command));
    binFile.write((uint8_t *)&other_data.camera_recording, sizeof(other_data.camera_recording));
    binFile.write((uint8_t *)&other_data.camera_count, sizeof(other_data.camera_count));
    binFile.write((uint8_t *)&other_data.play_audio, sizeof(other_data.play_audio));
    binFile.write((uint8_t *)&other_data.speaker_on, sizeof(other_data.speaker_on));
    binFile.write((uint8_t *)&other_data.audio_file, sizeof(other_data.audio_file));
    binFile.write((uint8_t *)&other_data.prev_sd_write_time_ms, sizeof(other_data.prev_sd_write_time_ms));
    binFile.write((uint8_t *)&other_data.delay_occurred_counter, sizeof(other_data.delay_occurred_counter));

    // gps_data
    binFile.write((uint8_t *)&gps_data.gps_hour, sizeof(gps_data.gps_hour)); 
    binFile.write((uint8_t *)&gps_data.gps_min, sizeof(gps_data.gps_min));
    binFile.write((uint8_t *)&gps_data.gps_sec, sizeof(gps_data.gps_sec));
    binFile.write((uint8_t *)&gps_data.latitude, sizeof(gps_data.latitude));
    binFile.write((uint8_t *)&gps_data.longitude, sizeof(gps_data.longitude));
    binFile.write((uint8_t *)&gps_data.altitude, sizeof(gps_data.altitude));
    binFile.write((uint8_t *)&gps_data.fix_type, sizeof(gps_data.fix_type));
    binFile.write((uint8_t *)&gps_data.siv, sizeof(gps_data.siv));
  }
}



void write_binary_data_buf_v2(
  const rtc1_t &rtc_time_buf, 
  const data1_t &imu_data_buf,
  const data2_t &other_data_buf,
  const gps_t &gps_data_buf
)
{
  if (binFile)
  {
    // rtc_time
    binFile.write((uint8_t *)&rtc_time_buf, sizeof(rtc_time_buf));

    // imu_data
    binFile.write((uint8_t *)&imu_data_buf, sizeof(imu_data_buf));

    // other data
    binFile.write((uint8_t *)&other_data_buf.atmopress, sizeof(other_data_buf.atmopress));
    binFile.write((uint8_t *)&other_data_buf.illuminance, sizeof(other_data_buf.illuminance));
    binFile.write((uint8_t *)&other_data_buf.D1, sizeof(other_data_buf.D1));
    binFile.write((uint8_t *)&other_data_buf.D2, sizeof(other_data_buf.D2));
    binFile.write((uint8_t *)&other_data_buf.battery_level, sizeof(other_data_buf.battery_level));
    binFile.write((uint8_t *)&other_data_buf.behavior_class, sizeof(other_data_buf.behavior_class));
    binFile.write((uint8_t *)&other_data_buf.camera_command, sizeof(other_data_buf.camera_command));
    binFile.write((uint8_t *)&other_data_buf.camera_recording, sizeof(other_data_buf.camera_recording));
    binFile.write((uint8_t *)&other_data_buf.camera_count, sizeof(other_data_buf.camera_count));
    binFile.write((uint8_t *)&other_data_buf.play_audio, sizeof(other_data_buf.play_audio));
    binFile.write((uint8_t *)&other_data_buf.speaker_on, sizeof(other_data_buf.speaker_on));
    binFile.write((uint8_t *)&other_data_buf.audio_file, sizeof(other_data_buf.audio_file));
    binFile.write((uint8_t *)&other_data_buf.prev_sd_write_time_ms, sizeof(other_data_buf.prev_sd_write_time_ms));
    binFile.write((uint8_t *)&other_data_buf.delay_occurred_counter, sizeof(other_data_buf.delay_occurred_counter));

    // gps_data
    binFile.write((uint8_t *)&gps_data_buf.gps_hour, sizeof(gps_data_buf.gps_hour));
    binFile.write((uint8_t *)&gps_data_buf.gps_min, sizeof(gps_data_buf.gps_min));
    binFile.write((uint8_t *)&gps_data_buf.gps_sec, sizeof(gps_data_buf.gps_sec));
    binFile.write((uint8_t *)&gps_data_buf.latitude, sizeof(gps_data_buf.latitude));
    binFile.write((uint8_t *)&gps_data_buf.longitude, sizeof(gps_data_buf.longitude));
    binFile.write((uint8_t *)&gps_data_buf.altitude, sizeof(gps_data_buf.altitude));
    binFile.write((uint8_t *)&gps_data_buf.fix_type, sizeof(gps_data_buf.fix_type));
    binFile.write((uint8_t *)&gps_data_buf.siv, sizeof(gps_data_buf.siv));
  }    
#ifdef DEBUG_SERIAL_PRINT
  else 
  {
    // if the file didn't open, print an error:
    if (connected_to_pc) Serial.println("error opening binFile");
  }
#endif // DEBUG_SERIAL_PRINT
}


void run_sd_write_routine()
{
  start_time_sd = millis();

//   if (last_time_sd == 0) last_time_sd = start_time_sd;
// #ifdef DEBUG_SERIAL_PRINT
//   if (connected_to_pc) 
//   {
//     Serial.println("");
//     Serial.print("loop2: run_sd_write_routine() loop time = ");
//     Serial.print( (start_time_sd - (last_time_sd))/1000.0, 3); Serial.print(" (ms) | ");
//   }
// #endif // DEBUG_SERIAL_PRINT
//   last_time_sd += 1000; // 1000 ms
  
  if (sensor_data_ready_to_write) 
  {
  #ifdef DEBUG_SERIAL_PRINT
    if (connected_to_pc) Serial.println("SD Write");
  #endif // DEBUG_SERIAL_PRINT
    save_all_data_buf_to_sd(rtc_time_copy); // The file name is determined by the data in rtc_time_copy
  }
  else 
  {
  #ifdef DEBUG_SERIAL_PRINT
    if (connected_to_pc) Serial.println("SD Wait");
  #endif // DEBUG_SERIAL_PRINT
    // Do not save data for the first 20 seconds 
    // because the sensor data can be noisy. 
    wait_until_sd_write_ready();
  }

  process_time_sd = millis() - start_time_sd;
  other_data.prev_sd_write_time_ms = process_time_sd; // update
}


void wait_until_sd_write_ready() 
{
  if (!sensor_data_ready_to_write && sd_write_start_countdown > 0)
  {
    sd_write_start_countdown--;
    sensor_data_ready_to_write = false;
  } 
  else if (!sensor_data_ready_to_write && sd_write_start_countdown == 0)
  {
    if (current_sec_for_sd_write == 59) // Start writing to SD at the next 00 second
    {
      sensor_data_ready_to_write = true;
    }
  }
  else 
  {
    sd_write_start_countdown = 0;
  }
  delay(1);
}

void save_all_data_to_sd(
  const rtc1_t &rtc_time, 
  const data1_t &imu_data,
  const data2_t &other_data,
  const gps_t &gps_data
) 
{ // Note that the data structure here is the copied data
  if (SD_power_on) {
    char directory[9] = {'\0'};
    char filename[16] = {'\0'};
    // char directory[15] = {'\0'};
    // char filename[22] = {'\0'};

    set_filename(directory, filename, rtc_time);
    // Serial.print("directory: "); Serial.println(directory);
    // Serial.print("filename: "); Serial.println(filename);
    write_binary_data(directory, filename, rtc_time, imu_data, other_data, gps_data);
  }
}

void save_all_data_buf_to_sd(const rtc1_t &rtc_time)

{ // Note that the data structure here is the copied data
  if (SD_power_on) 
  { 
    if (current_sec_for_sd_write == 0)
    {
      char directory[9] = {'\0'};
      char filename[16] = {'\0'};
      // char directory[15] = {'\0'};
      // char filename[22] = {'\0'};

      set_filename(directory, filename, rtc_time); // data_buffer_index = 0
      // Serial.print("directory: "); Serial.println(directory);
      // Serial.print("filename: "); Serial.println(filename);

      // Open the file
      if (sd.exists(directory))
      {
      #ifdef DEBUG_SERIAL_PRINT
        if (connected_to_pc) Serial.println("The directory already exists.");
      #endif // DEBUG_SERIAL_PRINT
      } 
      else
      {
        sd.mkdir(directory);
      #ifdef DEBUG_SERIAL_PRINT
        if (connected_to_pc) Serial.print("No directory -> Create new directory: "); Serial.println(directory);
      #endif // DEBUG_SERIAL_PRINT
      }
      binFile = sd.open(filename, O_WRITE | O_CREAT | O_AT_END | O_APPEND);
      if (!binFile) binFile = sd.open(filename, O_WRITE | O_CREAT | O_AT_END | O_APPEND);
    }
    
    if ((current_sec_for_sd_write + 1) % DATA_BUFFER_SIZE_SEC == 0)
    {
      // For a 60-second buffer, write data once per minute (at the 59th second)
      // For a 10-second buffer, write data once every 10 seconds (at the 9th, 19th, 29th, 39th, 49th, and 59th seconds)
      // For a 1-second buffer, write data every second
    // #ifdef DEBUG_SERIAL_PRINT
    //   if (connected_to_pc)
    //   {
    //     Serial.println(" "); Serial.println("Writing to SD card"); Serial.println();
    //   }
    // #endif // DEBUG_SERIAL_PRINT
      for (int i = 0; i < DATA_BUFFER_SIZE_SEC; i++)
      {
        write_binary_data_v2(rtc_time_buf[i], imu_data_buf[i], other_data_buf[i], gps_data_buf[i]);
      }
    }
    
    if (binFile && current_sec_for_sd_write == 59) binFile.close(); // to save the data

  }
}

void set_filename(char directory[], char filename[], const rtc1_t &rtc_time) 
{
  uint8_t start_idx = 0;
  // uint8_t start_idx = 6;

  char yy[5] = {'\0'};
  char MM[3] = {'\0'};
  char dd[3] = {'\0'};
  char hh[3] = {'\0'};
  char mm[3] = {'\0'};
  sprintf(yy, "%04d", rtc_time.year);
  sprintf(MM, "%02d", rtc_time.month);
  sprintf(dd, "%02d", rtc_time.day);
  sprintf(hh, "%02d", rtc_time.hour);
  sprintf(mm, "%02d", rtc_time.min);
  directory[start_idx+0] = yy[2]; 
  directory[start_idx+1] = yy[3];
  directory[start_idx+2] = MM[0]; 
  directory[start_idx+3] = MM[1];
  directory[start_idx+4] = dd[0]; 
  directory[start_idx+5] = dd[1];
  directory[start_idx+6] = hh[0]; 
  directory[start_idx+7] = hh[1];
  filename[start_idx+0] = yy[2]; 
  filename[start_idx+1] = yy[3];
  filename[start_idx+2] = MM[0]; 
  filename[start_idx+3] = MM[1];
  filename[start_idx+4] = dd[0]; 
  filename[start_idx+5] = dd[1];
  filename[start_idx+6] = hh[0]; 
  filename[start_idx+7] = hh[1];
  //filename[start_idx+4]='0'; filename[start_idx+5]='0';
  filename[start_idx+8]='/';
  filename[start_idx+9]=mm[0]; 
  filename[start_idx+10]=mm[1];
  filename[start_idx+11] = '.'; 
  filename[start_idx+12] = 'b'; 
  filename[start_idx+13] = 'i'; 
  filename[start_idx+14] = 'n';
}


// Waiting until the last second gives 
// the illumination sensor as much time as possible to get its reading
void update_variables(data2_t &other_data)
{
  other_data.camera_command = camera_command;
  other_data.camera_recording = camera_recording;
  if (camera_recording)
  {
    other_data.camera_count = camera_count;
  }
  else
  {
    other_data.camera_count = -1; // Set to -1 when not recording video
  }

  // Note! 0: target behaviour 1: other behaviours
  other_data.behavior_class = behavior_class_int; // TARGET_BEHAVIOR_CLASS_NUMBER 0
  // other_data.behavior_class = -1 // for debug

  other_data.play_audio = play_audio; // Check if the countdown for intervention has started
  other_data.speaker_on = speaker_on; // Check if the intervention has actually occurred

  // record the audio file id
  if (play_audio || speaker_on) {
    other_data.audio_file = audio_file_int;
  }
  else {
    other_data.audio_file = 0;
  }

  other_data.delay_occurred_counter = delay_occurred_counter;

}


// --------------------
// Sensor functions
// --------------------

// IMU data is read earlier to minimize any jitter in the access times to the FIFO buffers
void set_up_sensors() {

  // IMU BMI270
  set_up_BMI270();
  delay(200);

#ifdef DEBUG_SERIAL_PRINT
  if (connected_to_pc) {
    Serial.println("BMI270: OK");
  }
#endif // DEBUG_SERIAL_PRINT
  yellow_led_blink(100, 0, 1);

  // Magnetic sensor BMM150
  set_up_BMM150();

#ifdef DEBUG_SERIAL_PRINT
  if (connected_to_pc) {
    Serial.println("BMM150: OK");
  }
#endif // DEBUG_SERIAL_PRINT
  yellow_led_blink(100, 0, 1);


  // Barometer
  // set_up_barometer();
  // if (connected_to_pc) {
  //   Serial.println("LPS22HB:    OK");
  // }
  // delay(200);
  // yellow_led_blink(100, 0, 1);

// #ifdef USE_WATER_PRESSURE_SENSOR
  // Water Pressure Sensor
  set_up_water_pressure_sensor();
#ifdef DEBUG_SERIAL_PRINT
  if (connected_to_pc) {
    Serial.println("WPS MS5837: OK");
  }
#endif // DEBUG_SERIAL_PRINT
  yellow_led_blink(100, 0, 1);
// #endif // USE_WATER_PRESSURE_SENSOR

  turn_on_illuminance_sensor();
  delay(200);

#ifdef DEBUG_SERIAL_PRINT
  if (connected_to_pc) {
    Serial.println("BH1721FVC: OK");
  }
  if (connected_to_pc) {
    Serial.println("Sensor setup -> Completed.");
  }
#endif // DEBUG_SERIAL_PRINT
}


void set_up_RTC() {
  delay(200);
#ifdef DEBUG_SERIAL_PRINT
  if (connected_to_pc) {
    Serial.println("trying to setup RTC");
  }
#endif // DEBUG_SERIAL_PRINT

#ifdef DEBUG_SERIAL_PRINT
  if (connected_to_pc) {
    Serial.println("RTC RX8564LC: OK");
  }
#endif // DEBUG_SERIAL_PRINT
  delay(200);
  yellow_led_blink(100, 0, 1);
}


static uint8_t convert_gmt_to_local_hour(const uint8_t &gmt_hour) {
  // Ex1) gmt_hour = 0 (JST 09:00 AM)-> (24 + 0 + 9) % 24 = 9
  // Ex2) gmt_hour = 9 (JST 18:00 PM)-> (24 + 9 + 9) % 24 = 18
  // Ex3) gmt_hour = 18 (JST 03:00 AM)-> (24 + 18 + 9) % 24 = 2
  return (24 + gmt_hour + UTC_OFFSET) % 24;
}


void init_RTC_time(rtc1_t &rtc_time) {
  rtc_time.year = 0;
  rtc_time.month = 0;
  rtc_time.day = 0;
  rtc_time.hour = 0;
  rtc_time.min = 0;
  rtc_time.sec = 0;
}

void read_RTC_and_update_time(rtc1_t &rtc_time) {
  init_RTC_time(rtc_time); // initialization
  read_rtc_time(rtc_time);
  // Serial.print("RTC | year: "); Serial.print(rtc_time.year); 
  // Serial.print(" month: "); Serial.print(rtc_time.month);
  // Serial.print(" hour: "); Serial.print(rtc_time.hour);
  // Serial.print(" min: "); Serial.print(rtc_time.min);
  // Serial.print(" sec: "); Serial.print(rtc_time.sec); Serial.println("");
  min_counter = convert_gmt_to_local_hour(rtc_time.hour) * 60 + rtc_time.min; // DAWN, DUSK
  current_hour = rtc_time.hour;
  current_min = rtc_time.min;
  current_sec = rtc_time.sec; // use this variable to control GPS
}


void wait_until_rtc_time_sec_changed_in_setup()
{
  read_rtc_time(rtc_time);
  uint8_t last_rtc_time_sec = rtc_time.sec;
  uint8_t current_rtc_time_sec = rtc_time.sec;

  while (current_rtc_time_sec == last_rtc_time_sec) 
  { // Exit the loop when the current value differs from the last value
    read_time_sec_only(rtc_time);
    current_rtc_time_sec = rtc_time.sec;
    // delayMicroseconds(1);
  #ifdef DEBUG_SERIAL_PRINT
    if (connected_to_pc) Serial.print(".");
  #endif // DEBUG_SERIAL_PRINT
  }

#ifdef DEBUG_SERIAL_PRINT
  if (connected_to_pc) 
  {
    Serial.print("last: "); Serial.print(last_rtc_time_sec); Serial.print(" ");
    Serial.print("current: "); Serial.print(current_rtc_time_sec); Serial.println(" ");
    Serial.println(""); Serial.println("RTC sec changed!");
  }
#endif // DEBUG_SERIAL_PRINT
}

void wait_until_rtc_time_sec_changed_in_loop()
{
  // Note that read_time() is already called and rtc_time is already updated
  uint8_t last_rtc_time_sec = rtc_time.sec;

  // unsigned long read_time_start = micros();
  read_time_sec_only(rtc_time);
  // Serial.print("read_time(rtc): "); Serial.println(micros() - read_time_start); // 2930 us = 2.93 ms

  uint8_t current_rtc_time_sec = rtc_time.sec;
  while (current_rtc_time_sec == last_rtc_time_sec) 
  {
    read_rtc_time(rtc_time);
    current_rtc_time_sec = rtc_time.sec;
    // delayMicroseconds(1);
  #ifdef DEBUG_SERIAL_PRINT
    if (connected_to_pc) Serial.print(".");
  #endif // DEBUG_SERIAL_PRINT
  }

#ifdef DEBUG_SERIAL_PRINT
  if (connected_to_pc) 
  {
    Serial.println(""); 
    Serial.print("last: "); Serial.print(last_rtc_time_sec); Serial.print(" ");
    Serial.print("current: "); Serial.print(current_rtc_time_sec); Serial.println(" ");
    Serial.println("RTC sec changed!");
  }
#endif // DEBUG_SERIAL_PRINT

}


void sleep_nRF52840_by_sending_command_to_ESP32(uint32_t &sleep_sec) {
  enable_ESP32();
  delay(1000);
  yellow_led_blink(500, 500, 3);
  start_SerialESP32();
  delay(100);
  SerialESP32.print("sleep "); SerialESP32.println(sleep_sec); // 1 ~ 2375999 in seconds
}


// Sleep control by Time: returns True if the logger should sleep
// If SLEEP_CTRL_BY_TIME is not defined, always return false
static bool sleep_control_by_time(const uint16_t &minute_time) 
{
// minute_time = min_counter
// Check if the minute_time is outside the range specified by DAWN and DUSK, return true for sleep
// All units (minute_time, DAWN, DUSK) are in minutes (JST)
// Example: JST 09:30 -> 9 * 60 + 30 = 570
#ifdef SLEEP_CTRL_BY_TIME
    if (minute_time < DAWN || minute_time > DUSK) {
      return true;
    }
    else {
      return false;
    }
#else // SLEEP_CTRL_BY_TIME 
  return false;
#endif // SLEEP_CTRL_BY_TIME
}

// static bool sleep_control_by_location(const gps_t &gps_data) 
// {
// #ifdef SLEEP_CTRL_BY_LOCATION
//   if (logbot_is_within_bounding_box(gps_data)) {
//     return true;
//   }
//   else {
//     return false;
//   }
// #else // SLEEP_CTRL_BY_LOCATION
//   return false;
// #endif // SLEEP_CTRL_BY_LOCATION
// }


// Function to check if a value is within a specified area
// Returns true if the value is within the area, false if outside
static bool logbot_is_within_bounding_box(const gps_t &gps_data) {
  if (gps_data.latitude <  AREA_NE_LAT 
      && gps_data.latitude > AREA_SW_LAT 
      && gps_data.longitude <  AREA_NE_LON 
      && gps_data.longitude > AREA_SW_LON) {
    return true;
  }
  else {
    return false;
    // When the location data is all zeros, it is of course judged as outside.
  }
}



// -------------------
// GPS functions
// -------------------
void turn_on_GPS_power() {
  digitalWrite(GPS_PWR_PIN, HIGH);
  // delay(1000);
  gps_power_on = true;
#ifdef DEBUG_SERIAL_PRINT
  if (connected_to_pc) Serial.println("GPS POWER ON");
#endif // DEBUG_SERIAL_PRINT
}


void factory_reset_GPS_module()
{
  /*
  When the GPS module behaves unexpectedly after writing Power Mode-related configurations, 
  initializing it with factoryDefaults may restore its normal operation.
  */

#ifdef DEBUG_SERIAL_PRINT
  Serial.print(F("u-blox myGNSS.factoryReset(); "));
  myGNSS.factoryReset(); // it worked with GPS module that showed wired behaviours and messagess 
  Serial.println(F("Completed.")); 
  Serial.print(F("Turn off the device and upload new program.")); 
  Serial.println(F(" ")); 
  Serial.print(F("Or it will start recording in 5 s.")); 
  Serial.println(F(" "));
#endif // DEBUG_SERIAL_PRINT 
  for (int i = 0; i < 1 * 5; i++) 
  {
    // 1 sec
    green_led_blink(450, 50, 1);
    yellow_led_blink(450, 50, 1);
  }
}


void set_up_GPS_module() 
{
  // Turn ON the u-blox GPS module
  turn_on_GPS_power();
  delay(1000);
  // Start serial (UART) communication with the GPS module
  setup_gps_uart_serial1_9600();
  delay(1000);

#ifdef GPS_FACTORY_RESET
  // Factory Reset the GPS module
  factory_reset_GPS_module();
#endif // GPS_FACTORY_RESET

#ifdef GPS_FACTORY_DEFAULT
  // Reset the config of GPS module to factory defaults
  myGNSS.factoryDefault();
#endif // GPS_FACTORY_DEFAULT

  // NOTE: Power Management or Power Mode
  // Do NOT do anything. Otherwise, it will cause problems.

  // Basic settings
  myGNSS.setUART1Output(COM_TYPE_UBX); //Set the UART port to output UBX only
  myGNSS.setUART2Output(COM_TYPE_UBX); //Set the UART port to output UBX only
  myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT); //Save (only) the communications port settings to flash and BBR  
  myGNSS.setNavigationFrequency(NAVIGATION_FREQUENCY_4Hz);

  // set callbacks
  myGNSS.setAutoPVTcallbackPtr(&callbackPVT); // Enable automatic NAV PVT messages with callback to callbackPVT

  // Save configuration
  myGNSS.saveConfiguration(); //Save the current settings to flash and BBR

  // initialize GPS data structure
  init_GPS_data(gps_data);
  init_GPS_data(last_gps_loc);
  yellow_led_blink(100, 100, 3);

#ifdef DEBUG_SERIAL_PRINT
  if (connected_to_pc) Serial.print("GPS u-blox ZOE-M8Q: OK"); Serial.println("");
#endif // DEBUG_SERIAL_PRINT 

}


void callbackPVT(UBX_NAV_PVT_data_t *ubxDataStruct)
{
  Serial.println(F("Hey! The NAV PVT callback has been called!"));
}

void setup_gps_uart_serial1_9600() {

  Serial1.begin(9600);
  while (!Serial1) delay(10);
  Serial1.flush();

#ifdef DEBUG_SERIAL_PRINT
  if (connected_to_pc) Serial.println("Trying to start myGNSS using Serial1");
#endif // DEBUG_SERIAL_PRINT

  myGNSS.begin(Serial1);
  while (myGNSS.begin(Serial1, 2000) == false) delay(10);

#ifdef DEBUG_SERIAL_PRINT
  if (connected_to_pc) Serial.println("GNSS: connected at 9600 baud (Serial1)");
#endif // DEBUG_SERIAL_PRINT

}


void turn_off_GPS_power() { 
  if (gps_power_on) {
#ifndef GPS_CONNECTION_I2C
    Serial1.end();
    // delay(50);
#endif // GPS_CONNECTION_I2C
    digitalWrite(GPS_PWR_PIN, LOW);
    // delay(200);
    gps_power_on = false;
  #ifdef DEBUG_SERIAL_PRINT
    if (connected_to_pc) Serial.println("turning off GPS module");
  #endif // DEBUG_SERIAL_PRINT
  }
}

void restart_GPS() {
  digitalWrite(GPS_PWR_PIN, HIGH);
}

// void restart_serial_connection_GPS() {
//   Serial.begin(9600);
//   Serial1.begin(9600);
// }

// void hardware_backup_GPS() {
//   digitalWrite(GPS_PWR_PIN, LOW);
// }

// void stop_serial_connection_GPS() {
//   Serial.end();
//   Serial1.end();
// }

void init_GPS_time(rtc1_t &gps_time) {
  gps_time.year = 0;
  gps_time.month = 0;
  gps_time.day = 0;
  gps_time.hour = 0;
  gps_time.min = 0;
  gps_time.sec = 0;
}


// Check if new NAV PVT data has been received:
// If myGNSS.packetUBXNAVPVT->automaticFlags.flags.bits.callbackCopyValid is true, it indicates new PVT data has been received and has been copied.
// automaticFlags.flags.bits.callbackCopyValid will be cleared automatically when the callback is called.
void check_myGNSS_and_print() 
{
  if (myGNSS.packetUBXNAVPVT->automaticFlags.flags.bits.callbackCopyValid == true)
  {
    // But, we can manually clear the callback flag too. This will prevent the callback from being called!
    myGNSS.packetUBXNAVPVT->automaticFlags.flags.bits.callbackCopyValid = false; // Comment this line if you still want the callback to be called

    // Serial.println();

    Serial.print(F("Time: ")); // Print the time
    uint8_t hms = myGNSS.packetUBXNAVPVT->callbackData->hour; // Print the hours
    if (hms < 10) Serial.print(F("0")); // Print a leading zero if required
    Serial.print(hms);
    Serial.print(F(":"));
    hms = myGNSS.packetUBXNAVPVT->callbackData->min; // Print the minutes
    if (hms < 10) Serial.print(F("0")); // Print a leading zero if required
    Serial.print(hms);
    Serial.print(F(":"));
    hms = myGNSS.packetUBXNAVPVT->callbackData->sec; // Print the seconds
    if (hms < 10) Serial.print(F("0")); // Print a leading zero if required
    Serial.print(hms);
    Serial.print(F("."));
    unsigned long millisecs = myGNSS.packetUBXNAVPVT->callbackData->iTOW % 1000; // Print the milliseconds
    if (millisecs < 100) Serial.print(F("0")); // Print the trailing zeros correctly
    if (millisecs < 10) Serial.print(F("0"));
    Serial.print(millisecs);

    long latitude = myGNSS.packetUBXNAVPVT->callbackData->lat; // Print the latitude
    Serial.print(F(" Lat: "));
    Serial.print(latitude);

    long longitude = myGNSS.packetUBXNAVPVT->callbackData->lon; // Print the longitude
    Serial.print(F(" Long: "));
    Serial.print(longitude);
    Serial.print(F(" (degrees * 10^-7)"));

    long altitude = myGNSS.packetUBXNAVPVT->callbackData->hMSL; // Print the height above mean sea level
    Serial.print(F(" Height above MSL: "));
    Serial.print(altitude);
    Serial.println(F(" (mm)"));
  }
}

bool check_myGNSS_and_read_data(rtc1_t &gps_time, gps_t &gps_data) 
{

  gps_new_data = false;

  if (myGNSS.packetUBXNAVPVT->automaticFlags.flags.bits.callbackCopyValid == true)
  {
    gps_new_data = true;

    // But, we can manually clear the callback flag too. This will prevent the callback from being called!
    myGNSS.packetUBXNAVPVT->automaticFlags.flags.bits.callbackCopyValid = false; // Comment this line if you still want the callback to be called

    uint8_t fix_type = myGNSS.packetUBXNAVPVT->callbackData->fixType;
    uint8_t siv = myGNSS.packetUBXNAVPVT->callbackData->numSV;
    gps_data.fix_type = fix_type;
    gps_data.siv = siv;
    
  #ifdef DEBUG_FAKE_GPS_IN_LOOP
    if (force_gps_1hz_mode) 
    {
      gps_fix_type_buffer[gps_fix_type_buffer_index] = 3;
    }
    else
    {
      gps_fix_type_buffer[gps_fix_type_buffer_index] = 0;
    }
  #else
    gps_fix_type_buffer[(current_sec % GPS_FIX_TYPE_BUFFER_SIZE)] = fix_type;
  #endif
  // Update the ring buffer index (when index + 1 == BUFFER_SIZE, reset to 0).
  gps_fix_type_buffer_index = (gps_fix_type_buffer_index + 1) % GPS_FIX_TYPE_BUFFER_SIZE;

  // #ifdef DEBUG_SERIAL_PRINT
  //   if (connected_to_pc) 
  //   {
  //     Serial.print("gps_data.fix_type: "); Serial.print(gps_data.fix_type); Serial.print(" ");
  //   }
  // #endif // DEBUG_SERIAL_PRINT

    // GPS Fix Type: 
    // 0x00 = no fix; 0x01 = dead reckoning only; 0x02 = 2D-fix; 0x03 = 3D-fix
    // 0x04 = GPS + dead reckoning combined; 0x05 = Time only fix; 0x06..0xff = reserved
    if (gps_data.fix_type == 2 || gps_data.fix_type == 3) // 2D- and 3D-fix
    // if (gps_data.fix_type == 3) // 3D Fix only
    {
      
      uint16_t raw_year = myGNSS.packetUBXNAVPVT->callbackData->year; // 2023, 2024, 2025, ...
      // Serial.print("year: "); Serial.println(raw_year);
      uint8_t _year = raw_year - 2000; // 2024 - 2000 = 24
      uint8_t _month = myGNSS.packetUBXNAVPVT->callbackData->month;
      uint8_t _day = myGNSS.packetUBXNAVPVT->callbackData->day;
      
      gps_time.year = _year;
      gps_time.month = _month;
      gps_time.day = _day;

      // Serial.print(F("Time: ")); // Print the time
      uint8_t _hour = myGNSS.packetUBXNAVPVT->callbackData->hour; // Print the hours
      // if (_hour < 10) Serial.print(F("0")); // Print a leading zero if required
      // Serial.print(_hour);
      // Serial.print(F(":"));
      uint8_t _min = myGNSS.packetUBXNAVPVT->callbackData->min; // Print the minutes
      // if (_min < 10) Serial.print(F("0")); // Print a leading zero if required
      // Serial.print(_min);
      // Serial.print(F(":"));
      uint8_t _sec = myGNSS.packetUBXNAVPVT->callbackData->sec; // Print the seconds
      // if (_sec < 10) Serial.print(F("0")); // Print a leading zero if required
      // Serial.print(_sec);
      // Serial.print(F("."));
      unsigned long _millisecs = myGNSS.packetUBXNAVPVT->callbackData->iTOW % 1000; // Print the milliseconds
      // if (_millisecs < 100) Serial.print(F("0")); // Print the trailing zeros correctly
      // if (_millisecs < 10) Serial.print(F("0"));
      // Serial.print(_millisecs);

      long latitude = myGNSS.packetUBXNAVPVT->callbackData->lat; // Print the latitude
      // Serial.print(F(" Lat: "));
      // Serial.print(latitude);

      long longitude = myGNSS.packetUBXNAVPVT->callbackData->lon; // Print the longitude
      // Serial.print(F(" Long: "));
      // Serial.print(longitude);
      // Serial.print(F(" (degrees * 10^-7)"));

      long altitude = myGNSS.packetUBXNAVPVT->callbackData->hMSL; // Print the height above mean sea level
      // Serial.print(F(" Height above MSL: "));
      // Serial.print(altitude);
      // Serial.println(F(" (mm)"));

      gps_time.hour = _hour;
      gps_time.min = _min;
      gps_time.sec = _sec;

      /*
      It might be better to modify it so that the data is written and analyzed using long instead of float. 
      However, since the Bounding Box information is specified as float, 
      it is easier to handle float for comparison. 
      Additionally, it is better to keep it as float when using the decision tree for GPS.
      */
      float _latitude = latitude * 0.0000001; // * 10^(-7)
      float _longitude = longitude * 0.0000001; // * 10^(-7)
      float _altitude = altitude / 1000.0; // * 10^3

      gps_data.gps_hour = _hour;
      gps_data.gps_min = _min;
      gps_data.gps_sec = _sec;
      gps_data.latitude = _latitude;
      gps_data.longitude = _longitude;
      gps_data.altitude = _altitude;
      // gps_data.latitude = latitude;
      // gps_data.longitude = longitude;
      // gps_data.altitude = altitude;

      // time_initialized = true;
      gps_location_updated = true;

      return true; // -> gps_location_updated

    } 
    else 
    {
    // #ifdef DEBUG_SERIAL_PRINT
    //   Serial.print(F("No update for GPS data. "));
    // #endif // DEBUG_SERIAL_PRINT
      gps_data.fix_type = 0;
      return false;
    }
  } 
  else
  {
    gps_data.fix_type = 0;
    return false;
  }
}

void update_GPS_data_callback() 
{

// #ifdef DEBUG_SERIAL_PRINT
//   if (connected_to_pc) Serial.print("update_GPS_data_callback() -> ");
// #endif // DEBUG_SERIAL_PRINT

  gps_location_updated = check_myGNSS_and_read_data(gps_time, gps_data);
  copy_GPS_data(gps_data, last_gps_loc);

// #ifdef DEBUG_SERIAL_PRINT
//   if (connected_to_pc) 
//   {
//     Serial.print("gps_location_updated: "); Serial.println(gps_location_updated);
//   }
// #endif // DEBUG_SERIAL_PRINT
}


void run_update_GPS_data_routine()
{
  gps_location_updated = false;
  init_GPS_data(gps_data); // initialize GPS data, run outside update_GPS_data_callback()
  if (read_gps_data_now) update_GPS_data_callback(); // update GPS data
  if (read_gps_data_now) myGNSS.checkCallbacks();
#ifdef DEBUG_FAKE_GPS_IN_LOOP
  if (read_gps_data_now && !gps_location_updated) 
  {
    gps_location_updated = true;
    fake_GPS_data(gps_data);
    copy_GPS_data(gps_data, last_gps_loc);
  }
#endif // DEBUG_FAKE_GPS_IN_LOOP

// #ifdef DEBUG_GPS_LED
//   if (gps_location_updated) 
//   {
//     // yellow_led_blink(10, 0, 1);
//     green_led_blink(10, 0, 1);
//   }
// #endif // DEBUG_GPS_LED
}


void run_GPS_power_save_control_routine()
{
#ifdef GPS_POWER_ON_OFF_MODE_01ON_02OFF
  control_GPS_power_to_save_power_v5();
#endif // GPS_POWER_ON_OFF_MODE_01ON_02OFF

#ifdef GPS_POWER_ON_OFF_MODE_02ON_13OFF
  control_GPS_power_to_save_power_v3();
#endif // GPS_POWER_ON_OFF_MODE_02ON_13OFF

#ifdef GPS_POWER_ON_OFF_MODE_03ON_27OFF
  control_GPS_power_to_save_power_v4();
#endif // GPS_POWER_ON_OFF_MODE_03ON_27OFF
}


#ifdef GPS_POWER_ON_OFF_MODE_02ON_13OFF
void control_GPS_power_to_save_power_v3() {

// #define GPS_INTERVAL_MIN 3 // for debug
#define GPS_INTERVAL_MIN 15

  if ( force_gps_1hz_mode || camera_command || camera_recording 
      // || ( ((current_min + 2) % GPS_INTERVAL_MIN == 0) && current_sec >= 0 ) 
      || ( ((current_min + 1) % GPS_INTERVAL_MIN == 0) && current_sec >= 0 ) 
      || ( ((current_min + 0) % GPS_INTERVAL_MIN == 0) && current_sec >= 0 )
  )
  {
    if (!gps_power_on) 
    {
    #ifdef GPS_HW_BACKUP_ON_OFF_MODE
      pinMode(GPS_PWR_PIN, OUTPUT);
      digitalWrite(GPS_PWR_PIN, HIGH);
      gps_wait_time_sec_count_after_start = 4 + 1; // 0s: 4->3, 1s: 3->2, 2s:2->1 (read & init), 3s: 1->0 (read) 
    #endif // GPS_HW_BACKUP_ON_OFF_MODE
    #ifdef GPS_SW_BACKUP_ON_OFF_MODE
      myGNSS.getLatitude(10); // this query will wake up GPS module from SW backup mode
      gps_wait_time_sec_count_after_start = 2 + 1;
    #endif // GPS_SW_BACKUP_ON_OFF_MODE
      gps_power_on = true;
    #ifdef DEBUG_SERIAL_PRINT
      if (connected_to_pc) Serial.println(F("GPS -> ON !"));
    #endif // DEBUG_SERIAL_PRINT
    }
    
    read_gps_data_now = true;

    if (gps_wait_time_sec_count_after_start >= 3) 
    {
      read_gps_data_now = false;
      init_GPS_data(gps_data);
    }
    else if (gps_wait_time_sec_count_after_start == 2 || gps_wait_time_sec_count_after_start == 1) 
    {
      read_gps_data_now = true;
      init_GPS_data(gps_data);
    }
    else  // gps_wait_time_sec_count_after_start == 0
    {
      read_gps_data_now = true;
    }

    if (gps_wait_time_sec_count_after_start > 0) gps_wait_time_sec_count_after_start--;

  }
  else
  {
    // pinMode(GPS_PWR_PIN, OUTPUT);
    // digitalWrite(GPS_PWR_PIN, LOW);

    if (gps_power_on && gps_1hz_record_after_start_countdown_sec == 0)
    {
    #ifdef GPS_HW_BACKUP_ON_OFF_MODE
      pinMode(GPS_PWR_PIN, OUTPUT);
      digitalWrite(GPS_PWR_PIN, LOW);
      gps_power_on = false;
      #ifdef DEBUG_SERIAL_PRINT
        if (connected_to_pc) Serial.println(F("GPS -> OFF"));
      #endif // DEBUG_SERIAL_PRINT
    #endif // GPS_HW_BACKUP_ON_OFF_MODE
    // #ifdef GPS_SW_BACKUP_ON_OFF_MODE
    //   power_off_command_acknowledged = myGNSS.powerOff(12 * 60 * 1000, 10);
    //   if (power_off_command_acknowledged) 
    //   {
    //     gps_power_on = false;
    // #ifdef DEBUG_SERIAL_PRINT
    //   if (connected_to_pc) 
    //   {
    //     Serial.println(F("myGNSS.powerOff command acknowledged"));
    //     Serial.println(F("GPS -> OFF"));
        
    //   }
    // #endif // DEBUG_SERIAL_PRINT
    //   }
    // #endif // GPS_SW_BACKUP_ON_OFF_MODE
    }
    read_gps_data_now = false;
    // read_gps_data_now = true;
  } 

  if (gps_1hz_record_after_start_countdown_sec > 0)
  {
    gps_1hz_record_after_start_countdown_sec--;
    read_gps_data_now = true;
  } 

}
#endif // GPS_POWER_ON_OFF_MODE_02ON_13OFF


#ifdef GPS_POWER_ON_OFF_MODE_03ON_27OFF
void control_GPS_power_to_save_power_v4() {

  if ( force_gps_1hz_mode || camera_command || camera_recording 
    // || ( ((current_min + 4) % 30 == 0) && current_sec >= 0 ) 
    // || ( ((current_min + 3) % 30 == 0) && current_sec >= 0 ) 
    || ( ((current_min + 2) % 30 == 0) && current_sec >= 0 ) 
    || ( ((current_min + 1) % 30 == 0) && current_sec >= 0 ) 
    || ( ((current_min + 0) % 30 == 0) && current_sec >= 0 )
  )
  {
    if (!gps_power_on) 
    {
    #ifdef GPS_HW_BACKUP_ON_OFF_MODE
      pinMode(GPS_PWR_PIN, OUTPUT);
      digitalWrite(GPS_PWR_PIN, HIGH);
      gps_wait_time_sec_count_after_start = 4 + 1; // 0s: 4->3, 1s: 3->2, 2s:2->1 (read & init), 3s: 1->0 (read) 
    #endif // GPS_HW_BACKUP_ON_OFF_MODE
    // #ifdef GPS_SW_BACKUP_ON_OFF_MODE
    //   myGNSS.getLatitude(10); // this query will wake up GPS module from SW backup mode
    //   gps_wait_time_sec_count_after_start = 2 + 1;
    // #endif // GPS_SW_BACKUP_ON_OFF_MODE
      gps_power_on = true;
    #ifdef DEBUG_SERIAL_PRINT
      if (connected_to_pc) Serial.println(F("GPS -> ON !"));
    #endif // DEBUG_SERIAL_PRINT
    }
    
    read_gps_data_now = true;

    if (gps_wait_time_sec_count_after_start >= 3) 
    {
      read_gps_data_now = false;
      // init_GPS_data(gps_data);
    }
    else if (gps_wait_time_sec_count_after_start == 2 || gps_wait_time_sec_count_after_start == 1) 
    {
      read_gps_data_now = true;
      init_GPS_data(gps_data);
    }
    else  // gps_wait_time_sec_count_after_start == 0
    {
      read_gps_data_now = true;
    }

    if (gps_wait_time_sec_count_after_start > 0) gps_wait_time_sec_count_after_start--;

  }
  else
  {
    // pinMode(GPS_PWR_PIN, OUTPUT);
    // digitalWrite(GPS_PWR_PIN, LOW);

    read_gps_data_now = false;
    // read_gps_data_now = true;
    
    if (gps_power_on && gps_1hz_record_after_start_countdown_sec == 0)
    {
    #ifdef GPS_HW_BACKUP_ON_OFF_MODE
      pinMode(GPS_PWR_PIN, OUTPUT);
      digitalWrite(GPS_PWR_PIN, LOW);
      gps_power_on = false;
      #ifdef DEBUG_SERIAL_PRINT
        if (connected_to_pc) Serial.println(F("GPS -> OFF"));
      #endif // DEBUG_SERIAL_PRINT
    #endif // GPS_HW_BACKUP_ON_OFF_MODE
    // #ifdef GPS_SW_BACKUP_ON_OFF_MODE
    //   power_off_command_acknowledged = myGNSS.powerOff(25 * 60 * 1000, 10);
    //   if (power_off_command_acknowledged) 
    //   {
    //     gps_power_on = false;
    // #ifdef DEBUG_SERIAL_PRINT
    //   if (connected_to_pc) 
    //   {
    //     Serial.println(F("myGNSS.powerOff command acknowledged"));
    //     Serial.println(F("GPS -> OFF"));
        
    //   }
    // #endif // DEBUG_SERIAL_PRINT
    //   }
    // #endif // GPS_SW_BACKUP_ON_OFF_MODE
    }

  } 

  if (gps_1hz_record_after_start_countdown_sec > 0)
  {
    gps_1hz_record_after_start_countdown_sec--;
    read_gps_data_now = true;
  } 

}
#endif // GPS_POWER_ON_OFF_MODE_03ON_27OFF


#ifdef GPS_POWER_ON_OFF_MODE_01ON_02OFF
void control_GPS_power_to_save_power_v5() {

#define GPS_INTERVAL_MIN 3 // for debug
// #define GPS_INTERVAL_MIN 15

  if ( force_gps_1hz_mode || camera_command || camera_recording 
      // || ( ((current_min + 2) % GPS_INTERVAL_MIN == 0) && current_sec >= 0 ) 
      // || ( ((current_min + 1) % GPS_INTERVAL_MIN == 0) && current_sec >= 0 ) 
      || ( ((current_min + 0) % GPS_INTERVAL_MIN == 0) && current_sec >= 0 )
  )
  {
    if (!gps_power_on) 
    {
    #ifdef GPS_HW_BACKUP_ON_OFF_MODE
      pinMode(GPS_PWR_PIN, OUTPUT);
      digitalWrite(GPS_PWR_PIN, HIGH);
      gps_wait_time_sec_count_after_start = 4 + 1; // 0s: 4->3, 1s: 3->2, 2s:2->1 (read & init), 3s: 1->0 (read) 
    #endif // GPS_HW_BACKUP_ON_OFF_MODE
    // #ifdef GPS_SW_BACKUP_ON_OFF_MODE
    //   myGNSS.getLatitude(10); // this query will wake up GPS module from SW backup mode
    //   gps_wait_time_sec_count_after_start = 2 + 1;
    // #endif // GPS_SW_BACKUP_ON_OFF_MODE
      gps_power_on = true;
    #ifdef DEBUG_SERIAL_PRINT
      if (connected_to_pc) Serial.println(F("GPS -> ON !"));
    #endif // DEBUG_SERIAL_PRINT
    }
    
    read_gps_data_now = true;

    if (gps_wait_time_sec_count_after_start >= 3) 
    {
      read_gps_data_now = false;
      init_GPS_data(gps_data);
    }
    else if (gps_wait_time_sec_count_after_start == 2 || gps_wait_time_sec_count_after_start == 1) 
    {
      read_gps_data_now = true;
      init_GPS_data(gps_data);
    }
    else  // gps_wait_time_sec_count_after_start == 0
    {
      read_gps_data_now = true;
    }

    if (gps_wait_time_sec_count_after_start > 0) gps_wait_time_sec_count_after_start--;

  }
  else
  {
    // pinMode(GPS_PWR_PIN, OUTPUT);
    // digitalWrite(GPS_PWR_PIN, LOW);

    if (gps_power_on && gps_1hz_record_after_start_countdown_sec == 0)
    {
    #ifdef GPS_HW_BACKUP_ON_OFF_MODE
      pinMode(GPS_PWR_PIN, OUTPUT);
      digitalWrite(GPS_PWR_PIN, LOW);
      gps_power_on = false;
      #ifdef DEBUG_SERIAL_PRINT
        if (connected_to_pc) Serial.println(F("GPS -> OFF"));
      #endif // DEBUG_SERIAL_PRINT
    #endif // GPS_HW_BACKUP_ON_OFF_MODE
    // #ifdef GPS_SW_BACKUP_ON_OFF_MODE
    //   power_off_command_acknowledged = myGNSS.powerOff(12 * 60 * 1000, 10);
    //   if (power_off_command_acknowledged) 
    //   {
    //     gps_power_on = false;
    // #ifdef DEBUG_SERIAL_PRINT
    //   if (connected_to_pc) 
    //   {
    //     Serial.println(F("myGNSS.powerOff command acknowledged"));
    //     Serial.println(F("GPS -> OFF"));
        
    //   }
    // #endif // DEBUG_SERIAL_PRINT
    //   }
    // #endif // GPS_SW_BACKUP_ON_OFF_MODE
    }
    read_gps_data_now = false;
    // read_gps_data_now = true;
  } 

  if (gps_1hz_record_after_start_countdown_sec > 0)
  {
    gps_1hz_record_after_start_countdown_sec--;
    read_gps_data_now = true;
  } 

}
#endif // GPS_POWER_ON_OFF_MODE_01ON_02OFF


void control_force_gps_1hz_mode() 
{
#ifdef GPS_CONTINUOUS_1HZ_MODE
  force_gps_1hz_mode = true;
  force_gps_1hz_mode_countdown_sec = -1;
  force_gps_1hz_mode_init_countdown_sec = FORCE_GPS_1HZ_MODE_SEC;
#else // GPS_CONTINUOUS_1HZ_MODE
  if (camera_command || camera_recording)
  {
    force_gps_1hz_mode = true;
  }

  bool bird_may_be_still_flying = true;
  if (force_gps_1hz_mode && !camera_command && !camera_recording) 
  {
    if (force_gps_1hz_mode_init_countdown_sec >= 0)
    { // Once entering the 1Hz GPS recording mode, 
      // continue recording for 3 minutes without any interruption.
      force_gps_1hz_mode_init_countdown_sec--;
    }
    else if (force_gps_1hz_mode_init_countdown_sec < 0)
    {
      // During this period, determine if it is gps_1hz_mode_stable_and_outside_bb 
      // to trigger video recording and intervention.
      force_gps_1hz_mode_init_countdown_sec = -1;
      if (force_gps_1hz_mode_countdown_sec >= 0)
      {
        force_gps_1hz_mode_countdown_sec--;
        force_gps_1hz_mode = true;
      }
    #ifdef STOP_GPS_1HZ_MODE_IF_BIRD_IS_NOT_FLYING
      if (force_gps_1hz_mode_countdown_sec <= STOP_GPS_1HZ_MODE_IF_BIRD_IS_NOT_FLYING_START_SEC)
      { 
        // When the remaining time of GPS 1Hz Mode is less than 20 minutes, start the judgement.
        // If there are fewer target behaviors than 10/60 for 5 consecutive minutes, 
        // forcibly set force_gps_1hz_mode = false;
        // bird_may_be_still_flying = check_target_behavior_buffer_1min(3, 10); // 3 min, 10/60
        bird_may_be_still_flying = check_target_behavior_buffer_1min(5, 10); // 5 min, 10/60 
      }
    #endif // STOP_GPS_1HZ_MODE_IF_BIRD_IS_NOT_FLYING
    }
  }

  if ( force_gps_1hz_mode && (!bird_may_be_still_flying || force_gps_1hz_mode_countdown_sec < 0) )
  {
    force_gps_1hz_mode = false;
    force_gps_1hz_mode_init_countdown_sec = FORCE_GPS_1HZ_MODE_INIT_SEC; // reset
  } 
  
  if (!force_gps_1hz_mode) 
  {
    force_gps_1hz_mode_countdown_sec = -1;
    force_gps_1hz_mode_init_countdown_sec = FORCE_GPS_1HZ_MODE_INIT_SEC; // reset
  }

#endif // GPS_CONTINUOUS_1HZ_MODE
  
// #ifdef DEBUG_SERIAL_PRINT
//   if (connected_to_pc) 
//   {
//     Serial.print("force_gps_1hz_mode: "); Serial.print(force_gps_1hz_mode); Serial.print(" | ");
//     Serial.print("init_countdown_sec: "); Serial.print(force_gps_1hz_mode_init_countdown_sec); Serial.print(" | ");
//     Serial.print("countdown_sec: "); Serial.print(force_gps_1hz_mode_countdown_sec); 
//     Serial.println("");
//   }
// #endif // DEBUG_SERIAL_PRINT
}


bool check_target_behavior_buffer_1min(uint8_t stop_count_min, uint8_t behaviour_count_threshold)
{
  // Check whether to continue GPS 1Hz recording every minute.
  // Avoid current_sec == 00, as it involves SD card processing and other tasks.
  if (current_sec == 30) 
  { 
    
    uint8_t num_zeros = 0;
    for (int i = 0; i < 60; ++i) {
      // 0: target behavior, 1: non-target behavior
      if (target_behavior_buffer_1min[i] == 0) num_zeros++; 
    }
    
    if ( (num_zeros < behaviour_count_threshold) ) 
    { 
      force_gps_1hz_mode_stop_count++;
    }
    else
    {
      force_gps_1hz_mode_stop_count = 0;
    }
  }

  if (force_gps_1hz_mode_stop_count >= stop_count_min)
  { 
    // If there is little to no movement for a continuous period of stop_count_min minutes, 
    // stop using force_gps_1hz_mode.
    force_gps_1hz_mode_stop_count = 0; // reset
    return false; // force_gps_1hz_mode = false;
  }
  else
  {
    return true;
  }
}


void adjust_RTC_time_using_GPS_time_data_v1(rtc1_t &gps_time) {
  time_initialized = adjust_RTC1(gps_time);
}

void adjust_RTC_time_using_GPS_time_data_v2(rtc1_t &gps_time) {

  
  // To address the issue of the time discrepancy between RTC and GPS, 
  // we decided to use the GPS time minus 1 second when adjusting the RTC.

  // To correct the time discrepancy between RTC and GPS, we deliberately subtract 1 second.
  if (gps_time.sec == 0) { // If it's 0 seconds, set it to 59 seconds of the previous minute.
    gps_time.min = gps_time.min - 1;
    gps_time.sec = 59;
  } 
  else {
    gps_time.sec = gps_time.sec - 1; 
  }
  
  time_initialized = adjust_RTC1(gps_time);
}

void init_GPS_data(gps_t &gps_data) {
  gps_data.gps_hour = 0;
  gps_data.gps_min = 0;
  gps_data.gps_sec = 0;
  gps_data.latitude = 0;
  gps_data.longitude = 0;
  gps_data.altitude = 0;
  gps_data.fix_type = 0;
  gps_data.siv = 0;
}


void copy_rtc_time(const rtc1_t &rtc_time, rtc1_t &rtc_time_copy)
{
  rtc_time_copy.year = rtc_time.year;
  rtc_time_copy.month = rtc_time.month;
  rtc_time_copy.day = rtc_time.day;
  rtc_time_copy.hour = rtc_time.hour;
  rtc_time_copy.min = rtc_time.min;
  rtc_time_copy.sec = rtc_time.sec;
}

void copy_rtc_time_to_buf(const rtc1_t &rtc_time)
{
  uint8_t i = data_buffer_index;
  
  rtc_time_buf[i].year = rtc_time.year;
  rtc_time_buf[i].month = rtc_time.month;
  rtc_time_buf[i].day = rtc_time.day;
  rtc_time_buf[i].hour = rtc_time.hour;
  rtc_time_buf[i].min = rtc_time.min;
  rtc_time_buf[i].sec = rtc_time.sec;
}


void copy_acc_and_gyro_data(const data1_t &imu_data, data1_t &imu_data_copy)
{

  for (int i = 0; i < ACC_BUFFER_SIZE * 3; i++)
  {
    imu_data_copy.acc[i] = imu_data.acc[i];
    imu_data_copy.gyro[i] = imu_data.gyro[i];
  }
}

void init_acc_and_gyro_data(data1_t &imu_data)
{
  for (int i = 0; i < ACC_BUFFER_SIZE * 3; i++)
  {
    imu_data.acc[i] = 0;
    imu_data.gyro[i] = 0;
  }
}


void copy_mag_data(const data1_t &imu_data, data1_t &imu_data_copy)
{
  for (int i = 0; i < 1 * 3; i++) 
  {
    imu_data_copy.mag[i] = imu_data.mag[i];
  }
}

void copy_imu_data(const data1_t &imu_data, data1_t &imu_data_copy)
{

  for (int i = 0; i < ACC_BUFFER_SIZE * 3; i++)
  {
    imu_data_copy.acc[i] = imu_data.acc[i];
    imu_data_copy.gyro[i] = imu_data.gyro[i];
  }
  
  for (int i = 0; i < 1 * 3; i++) 
  {
    imu_data_copy.mag[i] = imu_data.mag[i];
  }
}

void copy_magnitude_data(float magnitude_buffer[], float magnitude_buffer_copy[])
{
  for (int i = 0; i < ACC_BUFFER_SIZE; i++)
  {
    magnitude_buffer_copy[i] = magnitude_buffer[i];
  }
}


void copy_imu_data_to_buf(const data1_t &imu_data)
{
  uint8_t i = data_buffer_index;
  // uint8_t start_index_acc_gyro = data_buffer_index * ACC_BUFFER_SIZE * 3;
  // uint8_t start_index_mag = data_buffer_index * 3;

  for (int j = 0; j < ACC_BUFFER_SIZE * 3; j++)
  {
    imu_data_buf[i].acc[j] = imu_data.acc[j];
    imu_data_buf[i].gyro[j] = imu_data.gyro[j];
  }
  
  for (int j = 0; j < 1 * 3; j++) 
  {
    imu_data_buf[i].mag[j] = imu_data.mag[j];
  }
}


void copy_other_data(const data2_t &other_data, data2_t &other_data_copy)
{
  other_data_copy.atmopress = other_data.atmopress;
  other_data_copy.illuminance = other_data.illuminance;
  other_data_copy.D1 = other_data.D1;
  other_data_copy.D2 = other_data.D2;
  other_data_copy.battery_level = other_data.battery_level;
  other_data_copy.behavior_class = other_data.behavior_class;
  other_data_copy.camera_command = other_data.camera_command;
  other_data_copy.camera_recording = other_data.camera_recording;
  other_data_copy.camera_count = other_data.camera_count;
  other_data_copy.play_audio = other_data.play_audio;
  other_data_copy.speaker_on = other_data.speaker_on;
  other_data_copy.audio_file = other_data.audio_file;
  other_data_copy.prev_sd_write_time_ms = other_data.prev_sd_write_time_ms;
  other_data_copy.delay_occurred_counter = other_data.delay_occurred_counter;
}


void copy_other_data_to_buf(const data2_t &other_data)
{
  uint8_t i = data_buffer_index;

  other_data_buf[i].atmopress = other_data.atmopress;
  other_data_buf[i].illuminance = other_data.illuminance;
  other_data_buf[i].D1 = other_data.D1;
  other_data_buf[i].D2 = other_data.D2;
  other_data_buf[i].battery_level = other_data.battery_level;
  other_data_buf[i].behavior_class = other_data.behavior_class;
  other_data_buf[i].camera_command = other_data.camera_command;
  other_data_buf[i].camera_recording = other_data.camera_recording;
  other_data_buf[i].camera_count = other_data.camera_count;
  other_data_buf[i].play_audio = other_data.play_audio;
  other_data_buf[i].speaker_on = other_data.speaker_on;
  other_data_buf[i].audio_file = other_data.audio_file;
  other_data_buf[i].prev_sd_write_time_ms = other_data.prev_sd_write_time_ms;
  other_data_buf[i].delay_occurred_counter= other_data.delay_occurred_counter;
}


void copy_other_data_1(const data2_t &other_data, data2_t &other_data_copy)
{
  // other_data_copy.atmopress = other_data.atmopress;
  // other_data_copy.illuminance = other_data.illuminance;
  // other_data_copy.D1 = other_data.D1;
  // other_data_copy.D2 = other_data.D2;
  // other_data_copy.battery_level = other_data.battery_level;
  other_data_copy.behavior_class = other_data.behavior_class;
  other_data_copy.camera_command = other_data.camera_command;
  other_data_copy.camera_recording = other_data.camera_recording;
  other_data_copy.camera_count = other_data.camera_count;
  other_data_copy.play_audio = other_data.play_audio;
  other_data_copy.speaker_on = other_data.speaker_on;
  other_data_copy.audio_file = other_data.audio_file;
  other_data_copy.prev_sd_write_time_ms = other_data.prev_sd_write_time_ms;
  other_data_copy.delay_occurred_counter = other_data.delay_occurred_counter;
}

void copy_other_data_2(const data2_t &other_data, data2_t &other_data_copy)
{
  other_data_copy.atmopress = other_data.atmopress;
  other_data_copy.illuminance = other_data.illuminance;
  other_data_copy.D1 = other_data.D1;
  other_data_copy.D2 = other_data.D2;
  other_data_copy.battery_level = other_data.battery_level;
  // other_data_copy.behavior_class = other_data.behavior_class;
  // other_data_copy.camera_command = other_data.camera_command;
  // other_data_copy.camera_recording = other_data.camera_recording;
  // other_data_copy.camera_count = other_data.camera_count;
  // other_data_copy.play_audio = other_data.play_audio;
  // other_data_copy.speaker_on = other_data.speaker_on;
  // other_data_copy.audio_file = other_data.audio_file;
  // other_data_copy.prev_sd_write_time_ms = other_data.prev_sd_write_time_ms;
  // other_data_copy.delay_occurred_counter = other_data.delay_occurred_counter;
}


void copy_GPS_time(const rtc1_t &gps_time, rtc1_t &gps_time_copy) 
{
  gps_time_copy.year = gps_time.year;
  gps_time_copy.month = gps_time.month;
  gps_time_copy.day = gps_time.day;
  gps_time_copy.hour = gps_time.hour;
  gps_time_copy.min = gps_time.min;
  gps_time_copy.sec = gps_time.sec;
}

void copy_GPS_data(const gps_t &gps_data, gps_t &gps_data_copy) 
{
  gps_data_copy.gps_hour = gps_data.gps_hour;
  gps_data_copy.gps_min = gps_data.gps_min;
  gps_data_copy.gps_sec = gps_data.gps_sec;
  gps_data_copy.latitude = gps_data.latitude;
  gps_data_copy.longitude = gps_data.longitude;
  gps_data_copy.altitude = gps_data.altitude;
  gps_data_copy.fix_type = gps_data.fix_type;
  gps_data_copy.siv = gps_data.siv;
}

void copy_GPS_data_to_buf(const gps_t &gps_data) 
{
  uint8_t i = data_buffer_index;

  gps_data_buf[i].gps_hour = gps_data.gps_hour;
  gps_data_buf[i].gps_min = gps_data.gps_min;
  gps_data_buf[i].gps_sec = gps_data.gps_sec;
  gps_data_buf[i].latitude = gps_data.latitude;
  gps_data_buf[i].longitude = gps_data.longitude;
  gps_data_buf[i].altitude = gps_data.altitude;
  gps_data_buf[i].fix_type = gps_data.fix_type;
  gps_data_buf[i].siv = gps_data.siv;
}


void adjust_RTC_with_fake_GPS_data() 
{
  if (!time_initialized) {
    rtc1_t gps_time;

    gps_time.year = 25;
    gps_time.month = 2;
    gps_time.day = 28;
    
    // 09:00:00 JST
    // gps_time.min = 0;
    // gps_time.min = 0;
    // gps_time.sec = 0;

    // 07:58:30 JST 
    // The date changes in UTC and the time changes soon after 
    // → The folder will be changed (can be used for debugging).
    gps_time.hour = 22; 
    gps_time.min = 58;
    gps_time.sec = 30;

    // 04:55:00
    // gps_time.hour = 19;
    // gps_time.min = 55;
    // gps_time.sec = 00;

    adjust_RTC1(gps_time);
    fake_GPS_data(gps_data);
    time_initialized = true;
  #ifdef DEBUG_SERIAL_PRINT
    if (connected_to_pc) 
    {
      Serial.println(F("RTC adjusted using fake gps data"));
      Serial.println(F("----------------------------------------------------------------------------------------------------------------------"));
    }
  #endif // DEBUG_SERIAL_PRINT
    delay(1000);
  }
}

void fake_GPS_data(gps_t &gps_data) {
  gps_data.gps_hour = 255;
  gps_data.gps_min = 255;
  gps_data.gps_sec = 255;
  // 34.81777, 135.52165 // Handai IST
  gps_data.latitude = 34.81777;
  gps_data.longitude = 135.52165;
  gps_data.altitude = 67;
  gps_data.fix_type = 3;
  gps_data.siv = 255;
}



boolean logbot_is_outside_bounding_box() {
#ifdef USE_LOCATION_BOUNDING_BOX_TRIGGER
  // done separately because we want this run each second 
  // inside BB -> False, outside BB -> True
  return trigger_control_using_location_bounding_box(gps_data, min_counter); // true or false
#endif // USE_LOCATION_BOUNDING_BOX_TRIGGER

#ifndef USE_LOCATION_BOUNDING_BOX_TRIGGER
  return true; // always return true
#endif // USE_LOCATION_BOUNDING_BOX_TRIGGER
}

boolean logbot_is_outside_bounding_box_v2() {
  
  bool is_outside_bb = false;

#ifdef USE_LOCATION_BOUNDING_BOX_TRIGGER
  // done separately because we want this run each second 
  // 
  if (logbot_is_within_bounding_box(last_gps_loc))
  {
    is_outside_bb = false; // inside BB -> False
  }
  else
  {
    is_outside_bb = true; // outside BB -> True
  }
#else
  is_outside_bb = true; // always return true
#endif // USE_LOCATION_BOUNDING_BOX_TRIGGER

// #ifdef DEBUG_SERIAL_PRINT
//   if (connected_to_pc)
//   {
//     Serial.print("outside_bounding_box: "); Serial.println(is_outside_bb);
//   }
// #endif // DEBUG_SERIAL_PRINT

  return is_outside_bb;
}


// Camera Control Using Bounding Box
static bool trigger_control_using_location_bounding_box(
  const gps_t &gps_data, 
  const uint16_t &min_counter) 
{
  // If GPS data is available, reset gps_timout by assigning GPS_TIMEOUT_MIN to it.
  // Assign min_counter to last_min.
  if (gps_data.latitude != 0 && gps_data.longitude != 0) 
  {
    copy_GPS_data(gps_data, last_gps_loc);
    gps_timeout = GPS_TIMEOUT_MIN;
    last_min = min_counter;
  }
  else 
  { 
    // If GPS data is not received and last_min is different from min_counter, decrement gps_timeout.
    // If GPS data has not been received for GPS_TIMEOUT_MIN minutes or more, the if statement below will return false.
    if (last_min != min_counter) { // min_counter is updated only once per minute.
      if (gps_timeout > 0) {
        gps_timeout -= 1;
      }
      last_min = min_counter;
    }
  }
  
  // If gps_timeout == 0 (i.e., GPS data has not been received for GPS_TIMEOUT_MIN minutes),
  // return false. (Return the same result as when the logger is within the bounding box (BB).)
  // Otherwise, return the opposite of the result from logbot_is_within_bounding_box().
  // i.e., if logbot_is_within_bounding_box() returns true, return false.
  if (gps_timeout == 0) {
    return false;
  }
  else {
    return !logbot_is_within_bounding_box(last_gps_loc);
    // Since there is a "!" (not operator), 
    // if the logger is within the bounding box (BB), it will return False;
    // if the logger is outside the bounding box, it will return True.
  }
}


// -------------------------
// BMI270
// -------------------------
// https://github.com/sparkfun/SparkFun_BMI270_Arduino_Library/blob/main/examples/Example05_FIFOBuffer/Example05_FIFOBuffer.ino

void set_up_BMI270() 
{
  // Check if sensor is connected and initialize
  // Address is optional (defaults to 0x68)
  // while (imu.beginI2C(i2cAddress) != BMI2_OK) {
  while (imu.beginI2C(BMI270_SLAVE_ADDRESS) != BMI2_OK) {
    // Not connected, inform user
  #ifdef DEBUG_SERIAL_PRINT
  if (connected_to_pc)
  {
    Serial.println("Error: BMI270 not connected, check wiring and I2C address!");
  }
  #endif // DEBUG_SERIAL_PRINT
    // Wait a bit to see if connection is established
    delay(1000);
  }

// #ifdef DEBUG_SERIAL_PRINT
//   Serial.println("BMI270 connected!");
// #endif // DEBUG_SERIAL_PRINT
  imu_power_on = true;

  // The default ODR (output data rate) is 100Hz for the accelerometer, and
  // 200Hz and gyroscope. Those are too fast for this example, and we want
  // them to be the same for the FIFO, so we'll reduce them both to the
  // minimum ODR that both sensors support (25Hz)
  // imu.setAccelODR(BMI2_ACC_ODR_25HZ);
  // imu.setGyroODR(BMI2_GYR_ODR_25HZ);
  imu.setAccelODR(BMI2_ACC_ODR_100HZ);
  imu.setGyroODR(BMI2_GYR_ODR_100HZ);

}


void read_BMI270_Data(
  float acc_data[], float gyro_data[], float magnitude_buffer[], uint8_t start_index
)
{

  imu.getSensorData();
  
  float acc_x, acc_y, acc_z;
  float gyro_x, gyro_y, gyro_z;
  uint8_t acc_data_index = start_index * 3;
  uint8_t gyro_data_index = start_index * 3;
  uint8_t magnitude_data_index = start_index * 1;

  acc_x = imu.data.accelX;
  acc_y = imu.data.accelY;
  acc_z = imu.data.accelZ;
  gyro_x = imu.data.gyroX;
  gyro_y = imu.data.gyroY;
  gyro_z = imu.data.gyroZ;

  acc_data[acc_data_index++] = acc_x;
  acc_data[acc_data_index++] = acc_y;
  acc_data[acc_data_index++] = acc_z;

  gyro_data[gyro_data_index++] = gyro_x;
  gyro_data[gyro_data_index++] = gyro_y;
  gyro_data[gyro_data_index++] = gyro_z;

  magnitude_buffer[magnitude_data_index++] = sqrt(acc_x*acc_x + acc_y*acc_y + acc_z*acc_z);

  // // Print acceleration data
  // Serial.print("Acceleration in g's");
  // Serial.print("\t");
  // Serial.print("X: ");
  // Serial.print(imu.data.accelX, 3);
  // Serial.print("\t");
  // Serial.print("Y: ");
  // Serial.print(imu.data.accelY, 3);
  // Serial.print("\t");
  // Serial.print("Z: ");
  // Serial.print(imu.data.accelZ, 3);

  // Serial.print("\t");

  // // Print rotation data
  // Serial.print("Rotation in deg/sec");
  // Serial.print("\t");
  // Serial.print("X: ");
  // Serial.print(imu.data.gyroX, 3);
  // Serial.print("\t");
  // Serial.print("Y: ");
  // Serial.print(imu.data.gyroY, 3);
  // Serial.print("\t");
  // Serial.print("Z: ");
  // Serial.println(imu.data.gyroZ, 3);
  
}


// -------------------------
// BMM150
// -------------------------
// https://github.com/DFRobot/DFRobot_BMM150/blob/master/examples/getGeomagneticData/getGeomagneticData.ino
void set_up_BMM150() {

  //When using I2C communication, use the following program to construct an object by DFRobot_BMM150_I2C
  /*!
    * @brief Constructor 
    * @param pWire I2C controller
    * @param I2C address
    *        i2c Address select, that CS and SDO pin select 1 or 0 indicates the high or low respectively. There are 4 combinations: 
    *          I2C_ADDRESS_1 0x10  (CS:0 SDO:0)
    *          I2C_ADDRESS_2 0x11  (CS:0 SDO:1)
    *          I2C_ADDRESS_3 0x12  (CS:1 SDO:0)
    *          I2C_ADDRESS_4 0x13  (CS:1 SDO:1) default i2c address
    */
  // DFRobot_BMM150_I2C bmm150(&Wire, I2C_ADDRESS_4);
  // DFRobot_BMM150_I2C bmm150(&Wire, BMM150_SLAVE_ADDRESS);

  while (bmm150.begin()) 
  {
    Serial.println("bmm150 init failed, Please try again!");
    delay(1000);
  } 
// #ifdef DEBUG_SERIAL_PRINT
//   Serial.println("bmm150 init success!");
// #endif // DEBUG_SERIAL_PRINT
  /**!
   * Set sensor operation mode
   * opMode:
   *   BMM150_POWERMODE_NORMAL  // normal mode  Get geomagnetic data normally
   *   BMM150_POWERMODE_FORCED  // forced mode  Single measurement, the sensor restores to sleep mode when the measurement is done.
   *   BMM150_POWERMODE_SLEEP   // sleep mode   Users can visit all the registers, but can't measure geomagnetic data
   *   BMM150_POWERMODE_SUSPEND // suspend mode At the time the sensor cpu doesn't work and can't implement any operation.
   *                                            Users can only visit the content of the control register BMM150_REG_POWER_CONTROL
   */
  bmm150.setOperationMode(BMM150_POWERMODE_NORMAL);

  /**!
   * Set preset mode, make it easier for users to configure sensor to get geomagnetic data
   * presetMode:
   *   BMM150_PRESETMODE_LOWPOWER      // Low power mode, get a small number of data and take the mean value.
   *   BMM150_PRESETMODE_REGULAR       // Regular mode, get a number of data and take the mean value.
   *   BMM150_PRESETMODE_ENHANCED      // Enhanced mode, get a large number of data and take the mean value.
   *   BMM150_PRESETMODE_HIGHACCURACY  // High accuracy mode, get a huge number of data and take the mean value.
   */
  bmm150.setPresetMode(BMM150_PRESETMODE_REGULAR);

  /**!
   * Set the rate of obtaining geomagnetic data, the higher, the faster (without delay function)
   * rate:
   *   BMM150_DATA_RATE_02HZ
   *   BMM150_DATA_RATE_06HZ
   *   BMM150_DATA_RATE_08HZ
   *   BMM150_DATA_RATE_10HZ   (default rate)
   *   BMM150_DATA_RATE_15HZ
   *   BMM150_DATA_RATE_20HZ
   *   BMM150_DATA_RATE_25HZ
   *   BMM150_DATA_RATE_30HZ
   */
  bmm150.setRate(BMM150_DATA_RATE_10HZ);

  /**!
   * Enable the measurement at x-axis, y-axis and z-axis, default to be enabled, no config required, the geomagnetic data at x, y and z will be incorrect when disabled.
   * Refer to setMeasurementXYZ() function in the .h file if you want to configure more parameters.
   */
  bmm150.setMeasurementXYZ();

}

// call the function below: 
// BMM150_FIFO_read_data(imu_data.mag);
void BMM150_FIFO_read_data(float mag_data[]) 
{
  sBmm150MagData_t magData = bmm150.getGeomagneticData();
  // Serial.print("mag x = "); Serial.print(magData.x); Serial.println(" uT");
  // Serial.print("mag y = "); Serial.print(magData.y); Serial.println(" uT");
  // Serial.print("mag z = "); Serial.print(magData.z); Serial.println(" uT");

  mag_data[0] = magData.x;
  mag_data[1] = magData.y;
  mag_data[2] = magData.z;

  // float type data
  // Serial.print("mag x = "); Serial.print(magData.xx); Serial.println(" uT");
  // Serial.print("mag y = "); Serial.print(magData.yy); Serial.println(" uT");
  // Serial.print("mag z = "); Serial.print(magData.zz); Serial.println(" uT");

  // float compassDegree = bmm150.getCompassDegree();
  // Serial.print("the angle between the pointing direction and north (counterclockwise) is:");
  // Serial.println(compassDegree);
  // Serial.println("--------------------------------");
  // delay(100);
}


/* Blue Robotics MS5837 Library Example
-----------------------------------------------------
Title: Blue Robotics MS5837 Library Example
Description: This example demonstrates the MS5837 Library with a connected
sensor. The example reads the sensor and prints the resulting values
to the serial terminal.
The code is designed for the Arduino Uno board and can be compiled and 
uploaded via the Arduino 1.0+ software.
-------------------------------
The MIT License (MIT)
Copyright (c) 2015 Blue Robotics Inc.
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
-------------------------------*/



uint16_t C[8];

void set_up_water_pressure_sensor() {
  // Reset the MS5837, per datasheet
  I2C_write_byte_2(WPT_MS5837_ADDRESS, MS5837_RESET); // Otsuka renamed function
  // Wait for reset to complete
  delay(10);

  // Read calibration values and CRC
  for ( uint8_t i = 0 ; i < 7 ; i++ ) {
    I2C_write_byte_2(WPT_MS5837_ADDRESS, MS5837_PROM_READ+i*2); // Otsuka
    Wire.requestFrom(WPT_MS5837_ADDRESS, 2);
    C[i] = (Wire.read() << 8) | Wire.read();
  }

  if (SD_power_on) {
    write_C(C, sizeof(C));
  }
}


void read_water_pressure_data()
{
  prepare_water_D1();
  delay(20);
  read_water_D1(other_data.D1);
  prepare_water_D2();
  delay(20);
  read_water_D2(other_data.D2);
}

void prepare_water_D1() {
  I2C_write_byte_2(WPT_MS5837_ADDRESS, MS5837_CONVERT_D1_8192);
}

void read_water_D1(uint32_t &D1) {
  I2C_write_byte_2(WPT_MS5837_ADDRESS, MS5837_ADC_READ);
  Wire.requestFrom(WPT_MS5837_ADDRESS, 3);
  D1 = 0;
  D1 = Wire.read();
  D1 = (D1 << 8) | Wire.read();
  D1 = (D1 << 8) | Wire.read();
}

void prepare_water_D2() {
  I2C_write_byte_2(WPT_MS5837_ADDRESS, MS5837_CONVERT_D2_8192);
}

void read_water_D2(uint32_t &D2) {
  I2C_write_byte_2(WPT_MS5837_ADDRESS, MS5837_ADC_READ);
  Wire.requestFrom(WPT_MS5837_ADDRESS, 3);
  D2 = 0;
  D2 = Wire.read();
  D2 = (D2 << 8) | Wire.read();
  D2 = (D2 << 8) | Wire.read();
}




void read_water_D1D2(uint32_t &D1, uint32_t &D2) {
  I2C_write_byte_2(WPT_MS5837_ADDRESS, MS5837_CONVERT_D1_8192);
  delay(20); // Max conversion time per datasheet
  I2C_write_byte_2(WPT_MS5837_ADDRESS, MS5837_ADC_READ);

  Wire.requestFrom(WPT_MS5837_ADDRESS, 3);
  D1 = 0;
  D1 = Wire.read();
  D1 = (D1 << 8) | Wire.read();
  D1 = (D1 << 8) | Wire.read();

  I2C_write_byte_2(WPT_MS5837_ADDRESS, MS5837_CONVERT_D2_8192);
  delay(20); // Max conversion time per datasheet
  
  I2C_write_byte_2(WPT_MS5837_ADDRESS, MS5837_ADC_READ);

  Wire.requestFrom(WPT_MS5837_ADDRESS, 3);
  D2 = 0;
  D2 = Wire.read();
  D2 = (D2 << 8) | Wire.read();
  D2 = (D2 << 8) | Wire.read();

#ifdef _PLUS_WATER_DEPTH
  calculate_wps(D1, D2);
#endif // _PLUS_WATER_DEPTH
}

#ifdef _PLUS_WATER_DEPTH
void calculate_wps(const uint32_t &D1, const uint32_t &D2)  {
  // Given C1-C6 and D1, D2, calculated TEMP and P
  // Do conversion first and then second order temp compensation

  int32_t TEMP;
  int32_t P;
  int32_t dT = 0;
  int64_t SENS = 0;
  int64_t OFF = 0;
  int32_t SENSi = 0;
  int32_t OFFi = 0;  
  int32_t Ti = 0;    
  int64_t OFF2 = 0;
  int64_t SENS2 = 0;
  
  // Terms called
  dT = D2-uint32_t(C[5])*256l;
  SENS = int64_t(C[1])*32768l+(int64_t(C[3])*dT)/256l;
  OFF = int64_t(C[2])*65536l+(int64_t(C[4])*dT)/128l;
  P = (D1*SENS/(2097152l)-OFF)/(8192l);
  
  // Temp conversion
  TEMP = 2000l+int64_t(dT)*C[6]/8388608LL;
  
  // Second order compensation
  if((TEMP/100)<20){         // Low temp
    Ti = (3*int64_t(dT)*int64_t(dT))/(8589934592LL);
    OFFi = (3*(TEMP-2000)*(TEMP-2000))/2;
    SENSi = (5*(TEMP-2000)*(TEMP-2000))/8;
    if((TEMP/100)<-15){    // Very low temp
      OFFi = OFFi+7*(TEMP+1500l)*(TEMP+1500l);
      SENSi = SENSi+4*(TEMP+1500l)*(TEMP+1500l);
    }
  }
  else if((TEMP/100)>=20){    // High temp
    Ti = 2*(dT*dT)/(137438953472LL);
    OFFi = (1*(TEMP-2000)*(TEMP-2000))/16;
    SENSi = 0;
  }
  
  OFF2 = OFF-OFFi;           // Calculate pressure and temp second order
  SENS2 = SENS-SENSi;
  
  TEMP = (TEMP-Ti);
  
  P = (((D1*SENS2)/2097152l-OFF2)/8192l);
  
  // Implementing a buffer queue
  // https://daeudaeu.com/stack_queue/#i-3
  if (water_pressure_fill_level >= WATER_PRESSURE_BUFFER_SIZE) {
    uint8_t i = 0;
    for (i = 0; i < WATER_PRESSURE_BUFFER_SIZE - 1; i++) {
      water_pressure_buffer[i] = water_pressure_buffer[i+1]; // shift the values in the buffer queue one position towards the front
    }
    water_pressure_buffer[WATER_PRESSURE_BUFFER_SIZE-1] = (uint16_t)(P / 10); // Assign a new value to the last element of the buffer
  } else {
    water_pressure_buffer[water_pressure_index] = (uint16_t)(P / 10); // Initially fill with 0, 1, 2 in order
    water_pressure_index = (water_pressure_index + 1) % WATER_PRESSURE_BUFFER_SIZE; // 1%3=1, 2%3=2, 3%3=3
  }
  // water_pressure_buffer[water_pressure_index] = (uint16_t)(P / 10);
  // if (water_pressure_index == 3) { // 0, 1, 2, 0, 1, 2, ...
  //   water_pressure_index = 0;
  // }
  // else {
  //   water_pressure_index = water_pressure_index % WATER_PRESSURE_BUFFER_SIZE
  // }
  // water_pressure_index = (water_pressure_index + 1) % WATER_PRESSURE_BUFFER_SIZE; // 1%3=1, 2%3=2, 3%3=3

  water_pressure_fill_level = water_pressure_fill_level < WATER_PRESSURE_BUFFER_SIZE ? water_pressure_fill_level + 1 : water_pressure_fill_level;
  // A ? B : C -> If A is true, execute B, otherwise execute C

// #ifdef USE_PRESSURE
//   pressure_ctrl = (uint16_t)(P/10.0f);  
// #endif // USE_PRESSURE

}
#endif // _PLUS_WATER_DEPTH

uint8_t crc4(uint16_t n_prom[]) {
  uint16_t n_rem = 0;

  n_prom[0] = ((n_prom[0]) & 0x0FFF);
  n_prom[7] = 0;

  for ( uint8_t i = 0 ; i < 16; i++ ) {
    if ( i%2 == 1 ) {
      n_rem ^= (uint16_t)((n_prom[i>>1]) & 0x00FF);
    } else {
      n_rem ^= (uint16_t)(n_prom[i>>1] >> 8);
    }
    for ( uint8_t n_bit = 8 ; n_bit > 0 ; n_bit-- ) {
      if ( n_rem & 0x8000 ) {
        n_rem = (n_rem << 1) ^ 0x3000;
      } else {
        n_rem = (n_rem << 1);
      }
    }
  }
  
  n_rem = ((n_rem >> 12) & 0x000F);

  return n_rem ^ 0x00;
}


// Barometric pressure sensor

void set_up_barometer() {
  // Works when ODR (output data rate) is written to 0x10
  I2C_write_byte(ATM_LPS22HB_ADDRESS, 0x10, LPS22HB_1HZ);
}

void read_barometer(uint32_t &pressure) {
  if (use_barometer) {
    uint8_t buf[3]={0};
    I2C_read(ATM_LPS22HB_ADDRESS, 0x28, 3, buf);
    uint32_t b = ((uint32_t)buf[2] << 8) | buf[1];
    pressure = (b << 8) | buf[0];
  } 
  else {
    pressure = 0;
  }
}

// Illuminance sensor

// Initialization of the BH1721 light sensor
void BH1721_Init() 
{
  BH1721_Send_command(0x01);  // Power ON
  BH1721_Send_command(0x10);  // auto resolution mode start
  delay(200);
}

// Reading light intensity data from the BH1721 sensor
int32_t BH1721_Illumination_Read() 
{
  Wire.requestFrom(ILL_BH1721FVC_ADDRESS, 2);
  int32_t n = (Wire.read() << 8) | Wire.read();
  return n;
}

// Sending command to the BH1721 light sensor
void BH1721_Send_command(const int8_t n)
{
  Wire.beginTransmission(ILL_BH1721FVC_ADDRESS);
  Wire.write(n);
  Wire.endTransmission();
}

// Turn on the illuminance sensor beforehand (150 msec required) ? 
// On logbot-v1, v2, and v4, the light sensor was turned ON/OFF every second.
// How much does the current consumption differ 
// between keeping the sensor always ON and turning it ON/OFF every second?
void turn_on_illuminance_sensor() 
{
  if (!illum_on) {
    // Illuminance sensor power ON
    I2C_write_byte_2(ILL_BH1721FVC_ADDRESS, 0x01);
    // Auto-resolution mode
    I2C_write_byte_2(ILL_BH1721FVC_ADDRESS, CONT_AUTO_RES);
    illum_on = true;
  }
}

void turn_on_illuminance_sensor_v2() 
{
  if (!illum_on) {
    BH1721_Send_command(0x01);  // Power ON
    BH1721_Send_command(0x10);  // auto resolution mode start
    illum_on = true;
  }
}

void turn_off_illuminance_sensor() 
{
  if (illum_on) {
    I2C_write_byte_2(ILL_BH1721FVC_ADDRESS, 0x00);
    illum_on = false;
  }
}

void turn_off_illuminance_sensor_v2() 
{
  if (illum_on) {
    BH1721_Send_command(0x00);
    illum_on = false;
  }
}

void read_illuminance(uint16_t &illuminance) 
{ // other_data.illuminance
  uint8_t buf[2];
  I2C_read(ILL_BH1721FVC_ADDRESS, 2, buf);
  illuminance = ((uint16_t)buf[0] << 8) | buf[1];  // reconstruct 16 bit data
  illuminance = illuminance / MEASUREMENT_ADJUSTMENT_TERM; // see data sheet for more information
  // must be called prior to using illuminance for camera control
  illuminance_ctrl = illuminance;
  // turn_off_illuminance_sensor();
}

void read_illuminance_v2(uint16_t &illuminance) 
{ // other_data.illuminance
  int32_t val = BH1721_Illumination_Read();
  Serial.print("val: "); Serial.println(val);
  illuminance = val / MEASUREMENT_ADJUSTMENT_TERM; // see data sheet for more information
  // must be called prior to using illuminance for camera control
  illuminance_ctrl = illuminance;
  // turn_off_illuminance_sensor_v2();
}


// ---------------------------------------------------
// Functions to control camera and speaker
// ---------------------------------------------------

// void run_real_time_behavior_recognition() 
// {
//   if (trigger_type == 0) 
//   { // 0: time (camera sampling at specified intervals)
//     control_camera_interval_sampling();
//   } else if (trigger_type == 1) 
//   { // 1:   acc data trigger real-time behaviour recognition
//     real_time_abr_using_acc_data_and_intervention();
//   }
// #ifdef _PLUS_WATER_DEPTH
//   else if (trigger_type == 2) { // 2: water depth data trigger (+ real-time behaviour recognition using acc data)
//     real_time_abr_using_water_depth_data_and_intervention();
//   }
// #endif // _PLUS_WATER_DEPTH 
//   else {
//     real_time_abr_using_acc_data_and_intervention();
//   }
// }

void real_time_abr_using_acc_data_and_intervention() 
{
  // outside_bounding_box = logbot_is_outside_bounding_box(); 

  if (cam_spk_gps_control_version == 1) 
  {
    trigger_control_by_acc_behavior_recognition_v1();
  }
  else if (cam_spk_gps_control_version == 2)
  {
    trigger_control_by_acc_behavior_recognition_v2();
  }
  else if (cam_spk_gps_control_version == 3)
  {
    trigger_control_by_acc_behavior_recognition_v3();
  }

// #ifdef DEBUG_SERIAL_PRINT
//   Serial.println("DEBUG: running real time animal behaviour recognition for test.");
// #endif // DEBUG_SERIAL_PRINT
}

#ifdef _PLUS_WATER_DEPTH
void real_time_abr_using_water_depth_data_and_intervention() 
{
  // outside_bounding_box = logbot_is_outside_bounding_box(); 
  trigger_control_by_water_depth_outlier_detection();
}
#endif // _PLUS_WATER_DEPTH

void control_camera_interval_sampling() 
{
  // outside_bounding_box = logbot_is_outside_bounding_box(); 

  if (cam_spk_gps_control_version == 1)
  {
    trigger_control_by_time();
  }
  else if (cam_spk_gps_control_version == 2)
  {
    trigger_control_by_time_v2();
  }
  
}

// A system to capture video every 5 to 30 minutes
// For example, if the remainder when dividing current_min by a specific interval (e.g., 5, 10, 15 minutes) is zero,
// and if current_sec is 0, then trigger video capture.
bool trigger_time_detection() 
{
  if ((current_min % CAMERA_SAMPLING_INTERVAL_MIN) == 0) 
  {
    return true;
  } 
  else 
  {
    return false;
  }
}

void trigger_control_by_time() 
{
  if (camera_count < MAX_CAMERA_COUNT) {
    if (trigger_time_detection() 
        // && outside_bounding_box 
        && !camera_command 
        && !camera_recording
        && camera_record_time_countdown_sec < 0 
        && camera_rest_time_countdown_sec < 0 
        // && gps_per_sec_stop_countdown_sec < 0
      #ifdef USE_SPEAKER
        && speaker_rest_time_countdown_sec < 0
      #endif // USE_SPEAKER
        && illuminance_ctrl >= ILLUM_THRESHOLD) 
    {
      // Camera
      camera_command = true;
      camera_record_time_countdown_sec = CAMERA_RECORD_TIME_SEC;

    #ifdef USE_SPEAKER
      play_audio = true; // new for speaker version
      speaker_turn_on_countdown_sec = SPEAKER_TURN_ON_BEHAVIOR_COUNT; 
      // countdown until playing audio from speaker
    #endif // USE_SPEAKER
    }
  }
}

void trigger_control_by_time_v2() 
{
  if (camera_count < MAX_CAMERA_COUNT) {
    if (trigger_time_detection() 
        && !camera_command 
        && !camera_recording
        && camera_record_time_countdown_sec < 0 
        && camera_rest_time_countdown_sec < 0 
      #ifdef USE_SPEAKER
        && speaker_rest_time_countdown_sec < 0
      #endif // USE_SPEAKER
        && illuminance_ctrl >= ILLUM_THRESHOLD) 
    {
      // Camera
      camera_command = true;
      camera_record_time_countdown_sec = CAMERA_RECORD_TIME_SEC;

      // GPS
      force_gps_1hz_mode = true;
      force_gps_1hz_mode_countdown_sec = CAMERA_RECORD_TIME_SEC;
      // force_gps_1hz_mode_countdown_sec = 5 * 60;

      init_gps_fix_type_buffer(gps_fix_type_buffer, GPS_FIX_TYPE_BUFFER_SIZE);
      init_behavior_class_buffer(behavior_class_buffer, BEHAVIOR_CLASS_BUFFER_SIZE);
    }
  }
}

void trigger_control_by_acc_behavior_recognition_v1() 
{
  if (camera_count < MAX_CAMERA_COUNT) 
  {

    bool real_time_abr_detected = real_time_behavior_recognition_using_acc_data();

  #ifdef GPS_CONTINUOUS_1HZ_MODE
    force_gps_1hz_mode = true;
  #elif defined(GPS_POWER_ON_OFF_MODE_2) || defined(GPS_POWER_ON_OFF_MODE_02ON_13OFF) || defined(GPS_POWER_ON_OFF_MODE_03ON_27OFF)

    if (real_time_abr_detected 
        && gps_1hz_record_after_start_countdown_sec == 0
        && camera_rest_time_after_start_countdown_sec <= 0) 
    { 
      // During the first few minutes of GPS data recording
      // and the first period when the camera is not capturing,
      // do not trigger any functions.
      force_gps_1hz_mode = true;
      // Each time 5 seconds of continuous flight is detected,
      // increase the duration by 2 minutes.
      // If stable flapping flight is detected, the duration will be updated continuously.
      // The maximum duration is 5 minutes.
      force_gps_1hz_mode_countdown_sec += 100;
      if (force_gps_1hz_mode_countdown_sec > 300) force_gps_1hz_mode_countdown_sec = 300; 
    }
  #endif // GPS_POWER_ON_OFF_MODE_02ON_13OFF
  
    if (real_time_abr_detected
        && gps_location_updated
        && outside_bounding_box 
        && !camera_command 
        && !camera_recording
        && camera_record_time_countdown_sec < 0 
        && camera_rest_time_countdown_sec < 0 
        && camera_rest_time_after_start_countdown_sec <= 0
      #ifdef USE_SPEAKER
        && speaker_rest_time_countdown_sec < 0
      #endif // USE_SPEAKER
        && illuminance_ctrl >= ILLUM_THRESHOLD)
    {
      // Camera
      camera_command = true;
      camera_record_time_countdown_sec = CAMERA_RECORD_TIME_SEC;

    #ifdef USE_SPEAKER
      // Speaker
      play_audio = true; // new for speaker version
      speaker_turn_on_countdown_sec = SPEAKER_TURN_ON_BEHAVIOR_COUNT; 
      // countdown until playing audio from speaker
    #endif // USE_SPEAKER

    }
  }
}


void trigger_control_by_acc_behavior_recognition_v2() 
{
  /*
  Turn on the camera and simultaneously set force_gps_1hz_mode to true
  Three phases during video recording: 
    0: Standby phase
    1: Final check phase
    2: Post-final check phase
  */
  
  if (camera_count < MAX_CAMERA_COUNT) 
  {

    bool real_time_abr_detected = real_time_behavior_recognition_using_acc_data();
    if (real_time_abr_detected
        && outside_bounding_box
        && !camera_command 
        && !camera_recording
        && camera_record_time_countdown_sec < 0 
        && camera_rest_time_countdown_sec < 0 
        && camera_rest_time_after_start_countdown_sec <= 0
      #ifdef USE_SPEAKER
        && speaker_rest_time_countdown_sec < 0
      #endif // USE_SPEAKER
        && illuminance_ctrl >= ILLUM_THRESHOLD)
    {
      // Camera
      camera_command = true;
      camera_record_time_countdown_sec = CAMERA_RECORD_TIME_SEC;

      // GPS
      force_gps_1hz_mode = true;
      // force_gps_1hz_mode_countdown_sec = CAMERA_RECORD_TIME_SEC;
      force_gps_1hz_mode_countdown_sec = FORCE_GPS_1HZ_MODE_SEC;

      init_gps_fix_type_buffer(gps_fix_type_buffer, GPS_FIX_TYPE_BUFFER_SIZE);
      init_behavior_class_buffer(behavior_class_buffer, BEHAVIOR_CLASS_BUFFER_SIZE);

    }

  }
}


void trigger_control_by_acc_behavior_recognition_v3() 
{
  /*
  Triggers and conditions for video recording:
    0: 5 consecutive detection of flights → First, set force_gps_1hz_mode to true and continue recording for 3 minutes.
    1: Perform GPS stability check and determine whether inside or outside Kabushima BB every second.
        If the GPS has had a 3D-Fix for the past 10 seconds and all location data are outside Kabushima, 
        determine that the flight is stable.
          -> Set gps_1hz_mode_stable_and_outside_bb = true to enable video recording.
          -> Force the force_gps_1hz_mode to remain active for at least 10 minutes.
        If 3D-Fix is not achieved, continue for 30 minutes.
        However, if there is little movement, exit 1 Hz mode.
        If staying inside Kabushima, it is likely stationary, so similarly exit 1 Hz mode.
    2. After detecting 5 consecutive flights again,
        with stable GPS data and confirmation of being outside the Kabushima BB.
        -> Start video recording
        (When video recording starts, reset the remaining time of 1 Hz Mode to 30 minutes.)
  
  Three phases during video recording: 
    0: Stand by phase: 0 - 60 seconds
        Wait for approximately 1 minute as a standby period after starting video recording,
        because 1) the footage can be used as pre-intervention data, and 2) the GPS data may be affected by camera noise. 
    1: final check phase: 61 - 120 seconds
        After 1 minute and up to 2 minutes from the start of recording, 
        perform final check for the audio playback.
    2: post final check phase: 121 - 185 seconds
        If audio playback was completed, continue the video recording (post-intervention data).
        If no audio playback was completed, stop the video recording.
  */
  
  bool real_time_abr_detected = real_time_behavior_recognition_using_acc_data();
#ifdef GPS_CONTINUOUS_1HZ_MODE
  force_gps_1hz_mode = true;
#elif defined(GPS_POWER_ON_OFF_MODE_02ON_13OFF) || defined(GPS_POWER_ON_OFF_MODE_03ON_27OFF)
  if (real_time_abr_detected 
      && !force_gps_1hz_mode // ignore if the logger is already in 1Hz mode
      && gps_1hz_record_after_start_countdown_sec == 0
      && camera_rest_time_after_start_countdown_sec <= 0) 
  { 
    // Do not respond during the initial GPS data recording period within the first few minutes 
    // after startup and the initial period when the camera is not recording.
    force_gps_1hz_mode = true;
    force_gps_1hz_mode_countdown_sec = FORCE_GPS_1HZ_MODE_SEC; // 30 min
    init_gps_fix_type_buffer(gps_fix_type_buffer, GPS_FIX_TYPE_BUFFER_SIZE);
    init_outside_bounding_box_buffer(outside_bounding_box_buffer, OUTSIDE_BOUNDING_BOX_BUFFER_SIZE);
  }
#endif // GPS_POWER_ON_OFF_MODE_02ON_13OFF
// Can be used in situations where the logger cannot be manually shaken 
// (e.g., testing GPS data and camera noise while driving in a car).
// After startup, setting force_gps_1hz_mode to true requires 5 consecutive flights (by shaking the logger), 
// but after that, the logger will automatically record videos at fixed intervals.
#ifdef DEBUG_SAMPLING_TEST
  if (force_gps_1hz_mode && current_min % DEBUG_SAMPLING_TEST_INTERVAL_MIN == 0 && current_sec == 0)
  {
    real_time_abr_detected = true;
  }
  // if (force_gps_1hz_mode && camera_count > 0 && (camera_count + 1) % 10 == 0) 
  // {
  //   force_gps_1hz_mode = false;
  //   force_gps_1hz_mode_countdown_sec = 0;
  // }
#endif // DEBUG_SAMPLING_TEST

  // --------------------------------------------------
  // Detection of target behaviour -> Turn on Camera 
  // --------------------------------------------------
  if (real_time_abr_detected // 5 consecutive flights
      && force_gps_1hz_mode // GPS 1 Hz
      && gps_1hz_mode_stable_and_outside_bb // generally, stable GPS data & outside Kabushima BB
      && gps_location_updated // the last GPS data is OK
      && outside_bounding_box // the last location is outside Kabushima BB
      && !camera_command 
      && !camera_recording
      && camera_record_time_countdown_sec < 0 
      && camera_rest_time_countdown_sec < 0 
      && camera_rest_time_after_start_countdown_sec <= 0
      && camera_count < MAX_CAMERA_COUNT
      && battery_level_v > 3.6
    #ifdef USE_SPEAKER
      && speaker_rest_time_countdown_sec < 0
    #endif // USE_SPEAKER
      && illuminance_ctrl >= ILLUM_THRESHOLD
  )
  {
    // Camera
    camera_command = true;
    camera_record_time_countdown_sec = CAMERA_RECORD_TIME_SEC;
    
    // GPS
    force_gps_1hz_mode = true;
    force_gps_1hz_mode_countdown_sec = FORCE_GPS_1HZ_MODE_SEC; // 30 min
    // init_gps_fix_type_buffer(gps_fix_type_buffer, GPS_FIX_TYPE_BUFFER_SIZE);
    // init_behavior_class_buffer(behavior_class_buffer, BEHAVIOR_CLASS_BUFFER_SIZE);
  }
  // Audio playback from the build-in speaker is controlled by play_audio_control().
}


#ifdef _PLUS_WATER_DEPTH
void trigger_control_by_water_depth_outlier_detection() 
{
  if (camera_count < MAX_CAMERA_COUNT) {

    if (real_time_behavior_recognition_using_water_depth_data() 
        && water_pressure_fill_level >= WATER_PRESSURE_BUFFER_SIZE
        && outside_bounding_box 
        && !camera_command 
        && !camera_recording
        && camera_record_time_countdown_sec < 0 
        && camera_rest_time_countdown_sec < 0 
        && illuminance_ctrl >= ILLUM_THRESHOLD) {

      // Camera
      camera_command = true;
      camera_record_time_countdown_sec = CAMERA_RECORD_TIME_SEC;

    }
  }
}
#endif // _PLUS_WATER_DEPTH


static bool real_time_behavior_recognition_using_acc_data() 
{
  // Select which decision tree to use and run the decision tree.
  uint8_t acc_tree_id = ACC_TREE_ID;
  if (acc_tree_id == 0) 
  {
    behavior_class_int = -1;
#ifdef DEBUG_HAND_SHAKE
    // Hand shake for Debug
    behavior_class_int = shake_wakeup();
#endif // DEBUG_HAND_SHAKE 
  }
  else if (acc_tree_id == 2) 
  {
    // behavior_class_int = Run_Umineko_Flying_Binary_2_2023();
    behavior_class_int = Run_Umineko_Flying_Binary_2_2024();
  // #ifdef DEBUG_SERIAL_PRINT
  //   Serial.print("DEBUG: Run_Umineko_Flying_Binary_2_2024");
  //   Serial.print(" behavior_class_int: "); Serial.println(behavior_class_int);
  // #endif // DEBUG_SERIAL_PRINT
  }
  // Update the behavior recognition buffer for final check.
  behavior_class_buffer[behavior_class_buffer_index] = behavior_class_int;
  // Update the index of the ring buffer (when index + 1 == BUFFER_SIZE, reset to 0).
  behavior_class_buffer_index = (behavior_class_buffer_index + 1) % BEHAVIOR_CLASS_BUFFER_SIZE;

  // Update the behavior recognition buffer for 1 minute.
  target_behavior_buffer_1min[target_behavior_buffer_1min_index] = behavior_class_int;
  target_behavior_buffer_1min_index = (target_behavior_buffer_1min_index + 1) % 60;

  // Algorithms to turn ON the trigger 
  // when the target behavior is recognized continuously for the specified number of seconds.
  // (e.g., detection of flight for consective 5 seconds)
  if (behavior_class_int == TARGET_BEHAVIOR_CLASS_NUMBER) {
    if ( (target_behavior_count + 1) < TARGET_BEHAVIOR_CONSECUTIVE_SEC) {
      target_behavior_count++;
      return false;
    }
    else if ( (target_behavior_count + 1) >= TARGET_BEHAVIOR_CONSECUTIVE_SEC) {
      target_behavior_count = 0;
      return true; // Turn on camera and start video recording
    }
    return false;
  }
  else { // behavior_class_int != 0
    target_behavior_count = 0; // initialize
    return false;
  }

}

#ifdef _PLUS_WATER_DEPTH
static bool real_time_behavior_recognition_using_water_depth_data() {
  // ACC Tree to update behaviour recognition result
  uint8_t acc_tree_id = ACC_TREE_ID;
  if (acc_tree_id == 0) {
    behavior_class_int = -1;
#ifdef DEBUG_HAND_SHAKE
    // Hand shake for Debug
    behavior_class_int = shake_wakeup();
#endif // DEBUG_HAND_SHAKE 
  }
  else if (acc_tree_id == 2) {
    behavior_class_int = Run_Umineko_Flying_Binary_2_2024();
  }

  // Water Depth Tree
  uint8_t water_depth_tree_id = WATER_DEPTH_TREE_ID;
  if (water_depth_tree_id == 0) {
    water_depth_outlier = CheckDepthOutlier(); // return 0 if the outlier detected, else 1
  }
  if (water_depth_outlier == 0) { // 0 = target_behaviour = outlier!
#ifdef DEBUG_SERIAL_PRINT
    if (connected_to_pc) {
      Serial.println("Water Depth: Outlier");
    }
#endif // DEBUG_SERIAL_PRINT
    return true; // -> turn on camera
  }
  else {
#ifdef DEBUG_SERIAL_PRINT
    if (connected_to_pc) {
      Serial.println("Water Depth: Inlier");
    }
#endif // DEBUG_SERIAL_PRINT
    return false;
  }
}
#endif // _PLUS_WATER_DEPTH

#ifdef DEBUG_HAND_SHAKE
static float RMS_shake(const float buffer1[]) {
  float sumX2 = 0;
  for (uint8_t i = 0; i < ACC_BUFFER_SIZE; i++) {
      sumX2 += buffer1[i] * buffer1[i];
  }
  return sqrt(sumX2 / ACC_BUFFER_SIZE);
}

static bool shake_wakeup() {
  float magnitude_rms = RMS_shake(magnitude_buffer);
#ifdef DEBUG_SERIAL_PRINT
  if (connected_to_pc) {
    Serial.print("magnitude_rms: ");
    Serial.println(magnitude_rms);
  }
#endif // DEBUG_SERIAL_PRINT
  if (magnitude_rms >= 1.2) {
  #ifdef DEBUG_SERIAL_PRINT
    if (connected_to_pc) Serial.println("Shake Detected !");
    green_led_blink(10, 0, 1);
  #endif // DEBUG_SERIAL_PRINT
    return 0; // target behavior
  }
  else {
    return 1;
  }
}
#endif // DEBUG_HAND_SHAKE

static float int16_to_float(int16_t raw_val) {
  // #ifdef SIMULATE_8_BIT_RAW
  //   raw_val = (raw_val >> 8) << 8;
  // #endif // SIMULATE_8_BIT_RAW
 return 8.0 * raw_val / 32767.0;
}

void start_SerialESP32() {
  SerialESP32.begin(9600);
  while (!SerialESP32) {
    // wait until Serial is ready
  }
  clear_buffer_SerialESP32();
}

void clear_buffer_SerialESP32() {
  while (SerialESP32.available() > 0) {
    SerialESP32.read();
  }
  // while (SerialESP32.read() != -1);
}

void enable_ESP32() {
  digitalWrite(ESP32_PWR_PIN, HIGH);
  // delay(100);
  esp32_enabled = true;
#ifdef DEBUG_SERIAL_PRINT
  if (connected_to_pc) Serial.println("| DEBUG: ESP32 ON |");
#endif // DEBUG_SERIAL_PRINT
}

void disable_ESP32() {
  SerialESP32.end();
  digitalWrite(ESP32_PWR_PIN, LOW);
  esp32_enabled = false;
#ifdef DEBUG_SERIAL_PRINT
  if (connected_to_pc) Serial.println("DEBUG: ESP32 OFF");
#endif // DEBUG_SERIAL_PRINT
}

void start_SerialVCAM() {
  SerialVCAM.begin(9600);
  // while (!SerialVCAM) {
  //   // wait until Serial is ready
  // }
}

void start_SerialVCAM_and_wait() {
  SerialVCAM.begin(9600);
  while (!SerialVCAM) {
    // wait until Serial is ready
  }
}

void start_SerialVCAM_and_clear_buffer() {
  SerialVCAM.begin(9600);
  while (!SerialVCAM) {
    // wait until Serial is ready
  }
  clear_buffer_SerialVCAM();
}

void clear_buffer_SerialVCAM() {
  while (SerialVCAM.available() > 0) {
    SerialVCAM.read();
  }
  // while (SerialVCAM.read() != -1);
}

void enable_VCAM_ESP32() {
  digitalWrite(VCAM_ESP32_PWR_PIN, HIGH);
  // delay(100);
  vcam_esp32_enabled = true;
#ifdef DEBUG_SERIAL_PRINT
  if (connected_to_pc) 
  {
    Serial.println("| DEBUG: VCAM ON for Speaker Control |");
  }
#endif // DEBUG_SERIAL_PRINT
}

void disable_VCAM_ESP32() {
  SerialVCAM.end();
  digitalWrite(VCAM_ESP32_PWR_PIN, LOW);
  vcam_esp32_enabled = false;
#ifdef DEBUG_SERIAL_PRINT
  if (connected_to_pc) 
  {
    Serial.println("| DEBUG: VCAM OFF for Speaker Control |");
  }
#endif // DEBUG_SERIAL_PRINT
}


void camera_control() {
  if (
    camera_command
    && !camera_recording
    && camera_record_time_countdown_sec > 0
    && camera_rest_time_countdown_sec < 0 
  ) {

    enable_ESP32();
    // delay(30);
    // start_SerialESP32(); 
  }
  else if (
    !camera_command 
    && camera_recording 
    && camera_record_time_countdown_sec < 0) 
  {
    // Stop recording
    SerialESP32.println("ct"); // stop camera recording
    // SerialESP32.println("st"); // stop audio recording
    // initialize camera variables
    camera_recording = false;
    camera_command_sent = false; 
    
    // Initialize the buffer for final check
    init_gps_fix_type_buffer(gps_fix_type_buffer, GPS_FIX_TYPE_BUFFER_SIZE);
    init_behavior_class_buffer(behavior_class_buffer, BEHAVIOR_CLASS_BUFFER_SIZE);

    if (intervention_cancelled)
    {
      camera_rest_time_countdown_sec = 60; // wait for 60 seconds
      speaker_rest_time_countdown_sec = 60; // wait for 60 seconds
      intervention_cancelled = false;
    }
    else 
    {
      // Typically set for longer than 60 seconds, such as 10 minutes or 15 minutes.
      camera_rest_time_countdown_sec = CAMERA_REST_TIME_SEC;

    #ifdef DEBUG_SAMPLING_TEST
      // if ( (camera_count + 1) % 10 == 0 && camera_count > 0) // 9, 19, 29, 39, ...
      if ( (camera_count + 1) % NUM_VIDEOS_FOR_DEBUG_SAMPLING_TEST == 0 && camera_count > 0)
      { 
        // During in-vehicle testing, after the the specified number of video recording, 
        // extend the rest_time to prevent further video recording.
        // If left idle, the GPS 1Hz mode will be turned off after about 15 minutes.
        camera_rest_time_countdown_sec = 60 * 17; // 17 min
      }
    #endif // DEBUG_SAMPLING_TEST
    }
    
    // Be careful! Commenting this out will increase battery consumption significantly.
    disable_ESP32(); // turn off the ESP32 on main board
  }
}


void send_camera_control_command() {
  if (
    camera_command
    && !camera_recording
    && camera_record_time_countdown_sec > 0
    && camera_rest_time_countdown_sec < 0
    && !camera_command_sent
    // && esp32_enabled
  ) {
  
  start_SerialESP32();
  
  // SerialESP32.println("cv agc off"); // Turn off Active Gain Control
  // SerialESP32.println("cs 0"); // 320*240 60fps
  // SerialESP32.println("cs 1"); // 400*296 60fps
  SerialESP32.println("cs 2"); // 640*480 30fps
  // SerialESP32.println("cs 3"); // 800*600 15fps
  camera_command_sent = true;
  camera_recording = true;
  camera_count++; 
  // Audio Recording (WAV file)
  // SerialESP32.println("ss"); // default 32000 sampling freq
  #ifdef DEBUG_SERIAL_PRINT
    if (connected_to_pc) Serial.println("| DEBUG: Camera ON command sent |");
  #endif // DEBUG_SERIAL_PRINT
  }
}

void camera_countdown() {
  if (camera_command) { // camera_record_time_countdown_sec
    if (camera_record_time_countdown_sec > 0) {
      camera_record_time_countdown_sec--;
    }
    else if (camera_record_time_countdown_sec <= 0) { 
      // Stop the recording once the camera recording time exceeds the specified duration.
      camera_record_time_countdown_sec = -1;
      camera_command = false;
    }
  }
  else { // camera_rest_time_countdown_sec
    if (camera_rest_time_countdown_sec > 0) 
    {
      camera_rest_time_countdown_sec--;
    }
    else if (camera_rest_time_countdown_sec <= 0) 
    {
      camera_rest_time_countdown_sec = -1;
    }

    if (camera_rest_time_after_start_countdown_sec > 0)
    {
      camera_rest_time_after_start_countdown_sec--;
    }
    else if (camera_rest_time_after_start_countdown_sec <= 0) 
    {
      camera_rest_time_after_start_countdown_sec = -1;
    }

    
  }
}


void speaker_control_and_countdown_for_final_check() {
  // play_audio: Becomes true when the trigger is turned ON
  // speaker_turn_on_countdown_sec: Assigned a value simultaneously with play_audio
  // speaker_rest_time_countdown_sec: 
  // final_check_target_behavior_count: The number of times target behavior was recognized after play_audio became true
  // final_check_ok: If target behavior continues, the intervention switch is turned ON
  if (play_audio && speaker_turn_on_countdown_sec > 0) {
  #ifdef DEBUG_SERIAL_PRINT
    if (connected_to_pc) Serial.println(F("| SPEAKER FINAL CHECK |"));
  #endif // DEBUG_SERIAL_PRINT

    speaker_turn_on_countdown_sec--;
    if (behavior_class_int == TARGET_BEHAVIOR_CLASS_NUMBER) {
      final_check_target_behavior_count++;
    }
    if (!vcam_esp32_enabled && speaker_turn_on_countdown_sec <= 2) 
    {
      enable_VCAM_ESP32();
      if (trigger_type == 1) {
        final_check_target_behavior_count = SPEAKER_FINAL_CHECK_THRESHOLD;
      }
    }
  } 
  else if (play_audio && speaker_turn_on_countdown_sec <= 0) 
  {
    if (number_of_audio_file_options == 2) 
    {
      select_audio_file2(); // Hayabusa or Noise
    }
    else if (number_of_audio_file_options == 3) 
    {
      select_audio_file3(); // Hyabusa or Noise or NUll
    }
    
    if (final_check_target_behavior_count >= SPEAKER_FINAL_CHECK_THRESHOLD) 
    {
      final_check_ok = true; // Explicitly indicate that the Final Check has been passed -> Audio playback intervention
      final_check_target_behavior_count = 0; // Initialize the counter
    }
    else 
    { 
      // Intervention cancel
      // Only initializes the counters without triggering the intervention
      final_check_target_behavior_count = 0; 
      final_check_ok = false;
      speaker_on = false;
      play_audio = false;
      if (vcam_esp32_enabled) {
        disable_VCAM_ESP32();
      }
      intervention_cancelled = true;
    }
  }
  // Audio playback complete -> Stop VCAM (Wait until the intervention audio finishes)
  // speaker_rest_time_countdown_sec becomes greater than 0 
  // only when an audio playback completed -> see send_speaker_command()
  else if (!play_audio && speaker_rest_time_countdown_sec > 0) {
    if (!camera_command)
    {
      speaker_rest_time_countdown_sec--;
    }

    // Turn off VCAM after VCAM_OFF_WAIT_SEC seconds following the intervention start
    if (speaker_on && speaker_on_sec_count < VCAM_OFF_WAIT_SEC)
    {
      speaker_on_sec_count++;
    }

    if (vcam_esp32_enabled && speaker_on && speaker_on_sec_count >= VCAM_OFF_WAIT_SEC) 
    {
      disable_VCAM_ESP32();
      speaker_on = false;
      speaker_on_sec_count = 0;
    }
  }
  else if (!play_audio && speaker_rest_time_countdown_sec <= 0) {
    speaker_rest_time_countdown_sec = -1;
  }
}



bool check_gps_fix_type_buffer(uint8_t gps_fix_type_buffer[], uint8_t size)
{
  // Return true if all GPS fix types from the past few tens of seconds are 3D-Fix.
  uint8_t counter = 0;
  for (int i = 0; i < size; i++)
  {
    if (gps_fix_type_buffer[i] == 3)
    {
      counter++;
    }
  }

#ifdef DEBUG_SERIAL_PRINT
  if (connected_to_pc)
  {
    Serial.print(F("fix_3d_counter: ")); 
    Serial.print(counter); Serial.print(F("/")); Serial.println(size);
  }
#endif // DEBUG_SERIAL_PRINT

  if (counter == size)
  {
    return true;
  }
  else
  {
    return false;
  }
}

bool check_outside_bounding_box_buffer(bool outside_bounding_box_buffer[], uint8_t size)
{
  // Return true if all GPS data from the past few tens of seconds are outside the BB.
  // Count the number of instances where Kabushima BB is outside == 1.
  uint8_t counter = 0;
  for (int i = 0; i < size; i++)
  {
    if (outside_bounding_box_buffer[i] == 1)
    {
      counter++;
    }
  }

#ifdef DEBUG_SERIAL_PRINT
  if (connected_to_pc)
  {
    Serial.print(F("outside_bounding_box_buffer: ")); 
    Serial.print(counter); Serial.print(F("/")); Serial.println(size);
  }
#endif // DEBUG_SERIAL_PRINT

  if (counter == size)
  {
    return true;
  }
  else
  {
    return false;
  }
}


void init_outside_bounding_box_buffer(bool outside_bounding_box_buffer[], uint8_t size)
{
  // Initialize all elements of the array to 0 (0 = inside Kabushima).
  for (int i = 0; i < size; i++) 
  {
    outside_bounding_box_buffer[i] = 0;
  }
}

void check_if_gps_1hz_mode_stable_and_outside_bb()
{
  // Check whether gps_1hz_mode_stable_and_outside_bb is true.
  // -> Enables setting camera_command to true.
  if (force_gps_1hz_mode 
      && force_gps_1hz_mode_init_countdown_sec < 0 // Continue 1 Hz recording for the first few minutes after force_gps_1hz_mode is activated.
      && gps_fix_type_buf_check_ok
      && outside_bounding_box_buf_check_ok
  )
  {
    gps_1hz_mode_stable_and_outside_bb = true;
  }
  else
  {
    gps_1hz_mode_stable_and_outside_bb = false;
  }
}



bool check_behavior_class_buffer(uint8_t behavior_class_buffer[], uint8_t size, float minimum_count)
{
  // Return true if a certain number or more of the behavior classes 
  // from the past few tens of seconds are the target behavior.
  uint8_t counter = 0;
  for (int i = 0; i < size; i++)
  {
    if (behavior_class_buffer[i] == TARGET_BEHAVIOR_CLASS_NUMBER)
    {
      counter++;
    }
  }

#ifdef DEBUG_SERIAL_PRINT
  if (connected_to_pc)
  {
    Serial.print(F("target_behavior_counter: ")); 
    Serial.print(counter); Serial.print(F("/")); Serial.println(size);
  }
#endif // DEBUG_SERIAL_PRINT

  if ( counter >= minimum_count )
  {
    return true;
  }
  else
  {
    return false;
  }
}


void init_gps_fix_type_buffer(uint8_t gps_fix_type_buffer[], uint8_t size)
{
  // Initialize all elements of the array to 0
  for (int i = 0; i < size; i++) 
  {
    gps_fix_type_buffer[i] = 0;
  }
}

void init_behavior_class_buffer(uint8_t behavior_class_buffer[], uint8_t size)
{
  // Initialize all elements of the array to 1
  for (int i = 0; i < size; i++) 
  {
    behavior_class_buffer[i] = 1; // Note that 0 is the target behavior!
  }
}

void play_audio_control_v2()
{
  if (camera_command && camera_recording)
  {
    
    uint16_t camera_recorded_time_sec = CAMERA_RECORD_TIME_SEC - camera_record_time_countdown_sec;
    uint8_t phase = 0;
    
    // bool gps_final_check_ok = check_gps_fix_type_buffer(gps_fix_type_buffer, GPS_FIX_TYPE_BUFFER_SIZE);
    behavior_class_buf_check_ok = check_behavior_class_buffer(behavior_class_buffer, BEHAVIOR_CLASS_BUFFER_SIZE, MINIMUM_TARGET_BEHAVIOR_COUNT);

    if (camera_recorded_time_sec <= FINAL_CHECK_STAND_BY_PHASE_SEC)
    {
      phase = 0; // stand by phase 
    #ifdef DEBUG_SERIAL_PRINT
      if (connected_to_pc)
      {
        Serial.println("| Phase 0: Stand By Phase |");
      }
    #endif // DEBUG_SERIAL_PRINT
    }
    else if ( camera_recorded_time_sec > FINAL_CHECK_STAND_BY_PHASE_SEC 
              && camera_recorded_time_sec <= FINAL_CHECK_STAND_BY_PHASE_SEC + FINAL_CHECK_PHASE_SEC)
    {
      phase = 1; // final check phase
    #ifdef DEBUG_SERIAL_PRINT
      if (connected_to_pc)
      {
        Serial.println("| Phase 1: Final Check Phase |");
      }
    #endif // DEBUG_SERIAL_PRINT
    }
    else
    {
      phase = 2; // post final check phase
    #ifdef DEBUG_SERIAL_PRINT
      if (connected_to_pc)
      {
        Serial.println("| Phase 2: Post Final Check Phase |");
      }
    #endif // DEBUG_SERIAL_PRINT
    }

    // Phase 0: 
    int16_t threshold_sec = 0;
    int16_t threshold_sec2 = 0;
    if (FINAL_CHECK_STAND_BY_PHASE_SEC >= 2) 
    {
      threshold_sec = FINAL_CHECK_STAND_BY_PHASE_SEC - 2; // 2 sec before
      threshold_sec2 = FINAL_CHECK_STAND_BY_PHASE_SEC - 1; // 1 sec before
    }
    if (phase == 0 && !vcam_esp32_enabled)
    {
      if (camera_recorded_time_sec > threshold_sec)
      {
        // Prepare for audio playback intervention
        enable_VCAM_ESP32();
        start_SerialVCAM();
        if (number_of_audio_file_options == 2) 
        {
          select_audio_file2(); // Hayabusa or Noise
        }
        else if (number_of_audio_file_options == 3) 
        {
          select_audio_file3(); // Hyabusa or Noise or NUll
        }
      }
      else if (camera_recorded_time_sec > threshold_sec2)
      {
        SerialVCAM.println("pv 10");
      } 
    }

    // Phase 1: 
    if (phase == 1 
        && !audio_played // audio playback not yet completed
        && gps_fix_type_buf_check_ok
        && behavior_class_buf_check_ok
        && behavior_class_int == TARGET_BEHAVIOR_CLASS_NUMBER
    ) 
    { // make sure that GPS is OK, the bird in flying state, and the logger in final check phase
      play_audio = true;
      final_check_ok = true;
    }
    // If audio plyaback was completed, turn off the VCAM power after VCAM_OFF_WAIT_SEC has elapsed.
    if (phase >= 1 && vcam_esp32_enabled && speaker_on)
    { 
      speaker_on_sec_count++;
      if (vcam_esp32_enabled && speaker_on_sec_count >= VCAM_OFF_WAIT_SEC)
      {
        // Turn off VCAM
        disable_VCAM_ESP32();
        speaker_on = false;
        speaker_on_sec_count = 0;
        // Keep audio_played as true.
      }
    }

    // Cancell audio playback intervention
    if (phase == 2 && !play_audio && !speaker_on && !audio_played) 
    { // If the audio playback intervention was complete, audio_played remains True.
    #ifdef DEBUG_SERIAL_PRINT
      if (connected_to_pc)
      {
        Serial.println("| Intervention Cancelled -> Turn off VCAM |");
      }
    #endif // DEBUG_SERIAL_PRINT
      // Turn off VCAM
      disable_VCAM_ESP32();
      camera_record_time_countdown_sec = 0; // to stop video recording
      intervention_cancelled = true;
    }

  }
  else // !camera_command || !camera_recording
  {
    if (audio_played) audio_played = false; // reset
    if (speaker_rest_time_countdown_sec >= 0) speaker_rest_time_countdown_sec--;
  }
}


void play_audio_control_v3()
{
  if (camera_command && camera_recording)
  {
    
    uint16_t camera_recorded_time_sec = CAMERA_RECORD_TIME_SEC - camera_record_time_countdown_sec;
    uint8_t phase = 0;
    
    // gps_fix_type_buf_check_ok
    // bool gps_final_check_ok = check_gps_fix_type_buffer(gps_fix_type_buffer, GPS_FIX_TYPE_BUFFER_SIZE);
    bool behavior_class_buf_check_ok = check_behavior_class_buffer(behavior_class_buffer, BEHAVIOR_CLASS_BUFFER_SIZE, MINIMUM_TARGET_BEHAVIOR_COUNT);

    if (camera_recorded_time_sec <= FINAL_CHECK_STAND_BY_PHASE_SEC)
    {
      phase = 0; // stand by phase 
    #ifdef DEBUG_SERIAL_PRINT
      if (connected_to_pc)
      {
        Serial.println("| Phase 0: Stand By Phase |");
      }
    #endif // DEBUG_SERIAL_PRINT
    }
    else if ( camera_recorded_time_sec > FINAL_CHECK_STAND_BY_PHASE_SEC 
              && camera_recorded_time_sec <= FINAL_CHECK_STAND_BY_PHASE_SEC + FINAL_CHECK_PHASE_SEC)
    {
      phase = 1; // final check phase
    #ifdef DEBUG_SERIAL_PRINT
      if (connected_to_pc)
      {
        Serial.println("| Phase 1: Final Check Phase |");
      }
    #endif // DEBUG_SERIAL_PRINT
    }
    else
    {
      phase = 2; // post final check phase
    #ifdef DEBUG_SERIAL_PRINT
      if (connected_to_pc)
      {
        Serial.println("| Phase 2: Post Final Check Phase |");
      }
    #endif // DEBUG_SERIAL_PRINT
    }

    // Phase 0: 
    int16_t threshold_sec = 0;
    if (FINAL_CHECK_STAND_BY_PHASE_SEC >= 2) threshold_sec = FINAL_CHECK_STAND_BY_PHASE_SEC - 2;
    if (phase == 0 && !vcam_esp32_enabled)
    {
      if (camera_recorded_time_sec > threshold_sec)
      {
        // Prepare for audio playback intervention
        enable_VCAM_ESP32();
        start_SerialVCAM();
        if (number_of_audio_file_options == 2) 
        {
          // select_audio_file2(); // Hayabusa or Noise
          select_audio_file2_v2(); // // Hayabusa or Noise added on 2024-05-22
        }
        else if (number_of_audio_file_options == 3) 
        {
          select_audio_file3(); // Hyabusa or Noise or NUll
        }
      }
    }

    // Phase 1: 
    if (phase == 1 
        && !audio_played
        && gps_fix_type_buf_check_ok
        && outside_bounding_box_buf_check_ok
        && behavior_class_buf_check_ok
        && behavior_class_int == TARGET_BEHAVIOR_CLASS_NUMBER
    ) 
    { // make sure that GPS is OK, the bird in flying state, and the logger in final check phase
      play_audio = true;
      final_check_ok = true;
    }
  #ifdef DEBUG_SAMPLING_TEST
    if (phase == 1 
        && !audio_played 
        && gps_fix_type_buf_check_ok
        && outside_bounding_box_buf_check_ok
    )
    { 
      // In this mode, regardless of the results of behavior recognition, 
      // perform video recording and audio intervention at regular intervals.
      play_audio = true;
      final_check_ok = true;
    }
  #endif // DEBUG_SAMPLING_TEST

    // If audio plyaback was completed, turn off the VCAM power after VCAM_OFF_WAIT_SEC has elapsed.
    if (phase >= 1 && vcam_esp32_enabled && speaker_on)
    { 
      speaker_on_sec_count++;
      if (vcam_esp32_enabled && speaker_on_sec_count >= VCAM_OFF_WAIT_SEC)
      {
        // Turn off VCAM
        disable_VCAM_ESP32();
        speaker_on = false;
        speaker_on_sec_count = 0; // reset
        // Keep audio_played as true.
      }
    }

    // Cancell audio playback intervention
    if (phase == 2 && !play_audio && !speaker_on && !audio_played)
    { // If the audio playback intervention was complete, audio_played remains True.
    #ifdef DEBUG_SERIAL_PRINT
      if (connected_to_pc)
      {
        Serial.println("| Intervention Cancelled -> Turn off VCAM |");
      }
    #endif // DEBUG_SERIAL_PRINT
      // Turn off VCAM
      disable_VCAM_ESP32();
      camera_record_time_countdown_sec = 0; // to stop video recording
      intervention_cancelled = true;
    }

  }
  else // !camera_command || !camera_recording
  {
    if (audio_played) audio_played = false; // reset
    if (speaker_rest_time_countdown_sec >= 0) speaker_rest_time_countdown_sec--;
  }
}


void send_speaker_command() 
{
  if (play_audio 
      && final_check_ok 
      && !audio_played
      && camera_recording
  ) 
  {
    // --------------------------------------------------------------------
    // Play audio from the built-in speaker when the conditions are met! 
    // --------------------------------------------------------------------
    // start_SerialVCAM();
    // clear_buffer_SerialVCAM();
    SerialVCAM.println("pv 10");
    // delay(1);
    if (audio_file_int == 1) 
    { 
      // NOTE: use simple and shorter file name to avoid delay
    #ifdef AUDIO_FILENAME_SIMPLE
      SerialVCAM.println("px _hayabusa"); // default audio file for debug test
    #else // AUDIO_FILENAME_SIMPLE
      SerialVCAM.println("px hayabusa_nr_amp4_4.2s.wav"); // 4.2 s version
    #endif // AUDIO_FILENAME_SIMPLE
    #ifdef DEBUG_SERIAL_PRINT
      if (connected_to_pc) Serial.println("| DEBUG: Speaker Play Audio 1 |");
    #endif // DEBUG_SERIAL_PRINT
    }
    else if (audio_file_int == 2) 
    {
    #ifdef AUDIO_FILENAME_SIMPLE
      SerialVCAM.println("px _noise"); // default audio file for debug test
    #else // AUDIO_FILENAME_SIMPLE
      SerialVCAM.println("px noise_amp4_4.2s.wav"); // 4.2 s version
    #endif // AUDIO_FILENAME_SIMPLE
    #ifdef DEBUG_SERIAL_PRINT  
      if (connected_to_pc) Serial.println("| DEBUG: Speaker Play Audio 2 |");
    #endif // DEBUG_SERIAL_PRINT
    }

    // ESP32 on main board 
    speaker_on = true;
    audio_played = true;
    play_audio = false;
    final_check_ok = false;
    // Set SPEAKER_REST_TIME_SEC only when audio playback was performed.
    speaker_rest_time_countdown_sec = SPEAKER_REST_TIME_SEC + 1; // Add 1 to align with the camera rest time
    audio_file_int = 0;
  }
}


// A function to update Acceleration data array that will be passed to ACC TREE Algorithms
// static void update_acc_buffer(const int16_t acc[])
// {
//   float x;
//   float y;
//   float z;
//   for (uint8_t i = 0; i < ACC_BUFFER_SIZE; i++) 
//   {
//     #ifdef USE_ACC_CALIB
//       x = (int16_to_float(acc[(i*3)]) - ACC_CALIB[0]) / ACC_CALIB[1];
//       y = (int16_to_float(acc[(i*3) + 1]) - ACC_CALIB[2]) / ACC_CALIB[3];
//       z = (int16_to_float(acc[(i*3) + 2]) - ACC_CALIB[4]) / ACC_CALIB[5];
//     #else //USE_ACC_CALIB
//       x = int16_to_float(acc[(i*3)]);
//       y = int16_to_float(acc[(i*3) + 1]);
//       z = int16_to_float(acc[(i*3) + 2]);
//     #endif //USE_ACC_CALIB
//     // magnitude buffer data array will be passed to a function controling camera ON/OFF
//     magnitude_buffer[i] = sqrt(x*x + y*y + z*z);
//   }
// }

// ---------------------------------------
// Feature Calculation using ACC Data
// ---------------------------------------
// - MeanCross()
// - OneCross()
// - Mean()
// - Variance()
// - RollingIndex()
// - MeanDiff()
// - ACCRollingMeanCross()
// - RollingOneCross()
// - RMS()
// - Kurtosis()
// - Crest()
// - Energy()
// ---------------------------------------


static bool MeanCross(const float &x, const float &y, const float &mean)
{
  return (x > mean) != (y > mean);
}

static bool OneCross(const float &x, const float &y)
{
  return (x > 1) != (y > 1);
}

static float Mean(const float data[], const uint8_t &buffer_size)
{
  float mean = 0;

  for (uint8_t i = 0; i < buffer_size; i++)
  {
    mean += data[i];
  }

  return mean / buffer_size;
}

static float Variance(const float data[], const uint8_t &buffer_size)
{
  float M = 0;
  float oldM = 0;
  float S = 0;

  for (uint8_t i = 0; i < buffer_size; i++)
  {
    oldM = M;
    M = (M + (data[i] - M) / (i + 1));
    S = (S + (data[i] - M) * (data[i] - oldM));
  }

  return S / (buffer_size - 1);
}

static uint8_t RollingIndex(const uint8_t &index, const uint8_t &buffer_size)
{
  return index % buffer_size;
}

static float MeanDiff(const float data[], const uint8_t &buffer_size, const uint8_t &start_index)
{
  float mean = 0;

  for (byte i = 1; i < buffer_size; i++)
  {
    mean += data[RollingIndex(start_index+i, buffer_size)] - data[RollingIndex(start_index+i-1, buffer_size)];
  }

  return mean / (buffer_size-1);
}


  static uint8_t ACCRollingMeanCross(const float data[], const float &mean)
  {
    uint8_t mc = 0;
    for (uint8_t i = ACC_BUFFER_SIZE - 1; i > 0; i--)
    {
      mc += MeanCross(data[i - 1], data[i], mean) ? 1 : 0;
    }
    return mc;
  }

  static uint8_t RollingOneCross(const float data[])
  {
    uint8_t oc = 0;
    for (uint8_t i = ACC_BUFFER_SIZE - 1; i > 0; i--)
    {
      oc += OneCross(data[i - 1], data[i]) ? 1 : 0;
    }
    return oc;
  }

  static float RMS(const float buffer1[])
  {
    float sumX2 = 0;
    for (uint8_t i = 0; i < ACC_BUFFER_SIZE; i++)
    {
      sumX2 += buffer1[i] * buffer1[i];
    }
    return sqrt(sumX2 / ACC_BUFFER_SIZE);
  }

  static float Kurtosis(const float buffer1[])
  {
    uint8_t n = 0;
    uint8_t n1 = 0;
    float mean = 0;
    float m2 = 0;
    float m3 = 0;
    float m4 = 0;
    float delta = 0;
    float delta_n = 0;
    float delta_n2 = 0;
    float term1 = 0;
    for (uint8_t i = 0; i < ACC_BUFFER_SIZE; i++)
    {
      n1 = i;
      n = n + 1;
      delta = buffer1[i] - mean;
      delta_n = delta / n;
      delta_n2 = delta_n * delta_n;
      term1 = delta * delta_n * n1;
      mean = mean + delta_n;
      m4 = m4 + term1 * delta_n2 * (n * n - 3 * n + 3) + 6 * delta_n2 * m2 - 4 * delta_n * m3;
      m3 = m3 + term1 * delta_n * (n - 2) - 3 * delta_n * m2;
      m2 = m2 + term1;
    }
    if ((m2 * m3) == 0)
    {
      return 0;
    }
    else
    {
      return (n * m4) / (m2 * m2);
    }
  }

  static float Crest(const float buffer1[], const float &rms)
  {
    if (rms == 0)
    {
      return 0;
    }
    else
    {
      float minVal = buffer1[0];
      float maxVal = buffer1[0];
      for (uint8_t i = 1; i < ACC_BUFFER_SIZE; i++)
      {
        if (buffer1[i] > maxVal)
        {
          maxVal = buffer1[i];
        }
        if (buffer1[i] < minVal)
        {
          minVal = buffer1[i];
        }
      }
        return (0.5f * (maxVal - minVal)) / rms;
    }
  }

  static float Energy(const float data[])
  {
    float magEnergy = 0;
    for (uint8_t i = 0; i < ACC_BUFFER_SIZE; i++)
    {
      magEnergy += (data[i] * data[i]);
    }
    return magEnergy / ACC_BUFFER_SIZE;
  }

// for Water Depth Outlier Detector
static uint8_t DepthRollingMeanCross(const float data[], const float &mean) {
  uint8_t mc = 0;
  for (uint8_t i = WATER_PRESSURE_BUFFER_SIZE - 1; i > 0; i--)
  {
    mc += MeanCross(data[i - 1], data[i], mean) ? 1 : 0;
  }
  return mc;
}

// ---------------------------------------------------------------------------------------------------------------------------------------
// Feature Calculation using GPS Data
// ---------------------------------------------------------------------------------------------------------------------------------------
// - ApproximateSpeed()
// - RotateLatLonMatrix()
// - ThreePointAngle()
// - ManhattanDistance()
// - FirstPassTime_Manhattan()
// - ZeroCrossLatLon()
// - MeanApproximateSpeed()
// - VarianceApproximateSpeed()
// - Displacement_Manhattan()
// - MaxDisplacement_Manhattan()
// - MeanDisplacement_Manhattan()
// - VarianceDisplacement_Manhattan()
// - MeanGPSAngle()
// - VarianceGPSAngle()
// - MeanDisplacementAngle()
// - Rotate()
// - Bearing()
// - GPSRollingMeanCross()
// ---------------------------------------------------------------------------------------------------------------------------------------
#ifdef GPS_TREE
// TODO: switch this so that it uses the other rollingindex 
// (the function should not reference GPS_BUFFER_SIZE or gps_index, these should be passed as parameters
// static uint8_t RollingIndex(uint8_t i)
// {
//    int16_t new_i = gps_index - i; //int16 since this can become negative values
//    return new_i > -1 ? new_i : GPS_BUFFER_SIZE + new_i;
// }

static float ApproximateSpeed(const uint8_t &index1, const uint8_t &index2)
{
  float d = minute_of_day_subtraction(minute_buffer[index2], minute_buffer[index1]);
  return d > 0 ? ManhattanDistance(index1, index2) * 16.66667f / d : 0;
}

//used to rotate the matrix 22.5, 45, 67.5, and 90 degrees to find a rotation that maximizes the variance
static void RotateLatLonMatrix(float lat_lon_mean[], float lat_lon_var[])
{
  lat_lon_mean[0] = Mean(latitude_buffer, GPS_BUFFER_SIZE);
  lat_lon_mean[1] = Mean(longitude_buffer, GPS_BUFFER_SIZE);

  float maxVariance = Variance(longitude_buffer, GPS_BUFFER_SIZE);
  uint8_t maxIndex = 0;

  for (uint8_t j = 1; j < 4; j++)
  {
    Rotate(j, lat_lon_mean);

    lat_lon_var[1] = Variance(rotated_longitude_buffer, GPS_BUFFER_SIZE);

    if (lat_lon_var[1] > maxVariance)
    {
      maxVariance = lat_lon_var[1];
      maxIndex = j;
    }
  }

  if (maxIndex < 4)
  {
    Rotate(maxIndex, lat_lon_mean);
    lat_lon_var[1] = maxVariance;
  }

  lat_lon_var[0] = Variance(rotated_latitude_buffer, GPS_BUFFER_SIZE);
  lat_lon_mean[0] = Mean(rotated_latitude_buffer, GPS_BUFFER_SIZE);
  lat_lon_mean[1] = Mean(rotated_longitude_buffer, GPS_BUFFER_SIZE);
}

static int16_t ThreePointAngle(const uint8_t &index1, const uint8_t &index2, const uint8_t &index3)
{
  int16_t a1 = Bearing(index2, index1);
  int16_t a2 = Bearing(index2, index3);

  return (int16_t) min((a1 - a2) < 0 ? a1 - a2 + 360 : a1 - a2, (a2 - a1) < 0 ? a2 - a1 + 360 : a2 - a1);
}

static float ManhattanDistance(const uint8_t &index1, const uint8_t &index2)
{
  float dy = latitude_buffer[index2] - latitude_buffer[index1];
  float dx = longitude_buffer[index2] - longitude_buffer[index1];

  return DEG_LEN * (abs(dx) + abs(dy));
}

static uint16_t FirstPassTime_Manhattan()
{
  uint16_t startTime = minute_buffer[gps_index];
  uint16_t endTime = minute_buffer[gps_index];

  for (uint8_t i = 1; i < GPS_BUFFER_SIZE; i++)
  {
    // TODO: double check that this works well, just moved endtime assignment to before checking distance. 
    // This way the time assigned will be the first time after passing the distance.
    endTime = minute_buffer[RollingIndex(gps_index+i, GPS_BUFFER_SIZE)];
    if (ManhattanDistance(RollingIndex(gps_index+i, GPS_BUFFER_SIZE), gps_index) > FPT_KM)
    {
        break;
    }
  }

  return minute_of_day_subtraction(endTime, startTime);
}

static void ZeroCrossLatLon(uint8_t lat_lon_zc[], const bool &run_rotate, float lat_lon_mean[], float lat_lon_var[])
{
  if (run_rotate)
  {
    RotateLatLonMatrix(lat_lon_mean, lat_lon_var);      
  }
  lat_lon_zc[0] = GPSRollingMeanCross(rotated_latitude_buffer, lat_lon_mean[0]);
  lat_lon_zc[1] = GPSRollingMeanCross(rotated_longitude_buffer, lat_lon_mean[1]);
}

static float MeanApproximateSpeed()
{
  float speed_acum = 0;

  for (uint8_t i = 1; i < GPS_BUFFER_SIZE; i++)
  {
      speed_acum += ApproximateSpeed(
        RollingIndex(gps_index+i-1, GPS_BUFFER_SIZE), 
        RollingIndex(gps_index+i, GPS_BUFFER_SIZE));
  }

  return speed_acum / (GPS_BUFFER_SIZE - 1);
}

static float VarianceApproximateSpeed()
{
  float M = 0;
  float oldM = 0;
  float S = 0;
  float sp = 0;

  for (uint8_t i = 1; i < GPS_BUFFER_SIZE; i++)
  {
    sp = ApproximateSpeed(
      RollingIndex(gps_index+i-1, GPS_BUFFER_SIZE), 
      RollingIndex(gps_index+i, GPS_BUFFER_SIZE));
    oldM = M;
    M = M + (sp - M) / i;
    S = S + (sp - M) * (sp - oldM);
  }

  return S / (GPS_BUFFER_SIZE - 2);
}

static float Displacement_Manhattan()
{
  return ManhattanDistance(
    RollingIndex(gps_index+GPS_BUFFER_SIZE-1, GPS_BUFFER_SIZE), 
    gps_index);
}

static float Distance_Manhattan()
{
  float distance = 0;

  for (uint8_t i = 1; i < GPS_BUFFER_SIZE; i++)
  {
    distance += ManhattanDistance(
      RollingIndex(gps_index+i, GPS_BUFFER_SIZE), 
      RollingIndex(gps_index+i-1, GPS_BUFFER_SIZE));
  }

  return distance;
}

static float MaxDisplacement_Manhattan()
{
  float max_displacement = 0;

  for (uint8_t i = 1; i < GPS_BUFFER_SIZE; i++)
  {
    float displacement = ManhattanDistance(
      RollingIndex(gps_index+i, GPS_BUFFER_SIZE), 
      gps_index);

    if (displacement > max_displacement)
    {
      max_displacement = displacement;
    }
  }

  return max_displacement;
}

static float MeanDisplacement_Manhattan()
{
  float mean_displacement = 0;

  for (uint8_t i = 1; i < GPS_BUFFER_SIZE; i++)
  {
    mean_displacement += ManhattanDistance(
      RollingIndex(gps_index+i, GPS_BUFFER_SIZE), 
      gps_index);
  }

  return mean_displacement / (GPS_BUFFER_SIZE - 1);
}

static float VarianceDisplacement_Manhattan()
{
  float M = 0;
  float oldM = 0;
  float S = 0;
  float d = 0;

  for (uint8_t i = 1; i < GPS_BUFFER_SIZE; i++)
  {
    d = ManhattanDistance(
      RollingIndex(gps_index+i, GPS_BUFFER_SIZE), 
      gps_index);
    oldM = M;
    M = M + (d - M) / i;
    S = S + (d - M) * (d - oldM);
  }

  return S / (GPS_BUFFER_SIZE - 2);
}

static float MeanGPSAngle()
{
  float mean_angle = 0;

  for (uint8_t i = 2; i < GPS_BUFFER_SIZE; i++)
  {
    mean_angle += ThreePointAngle(
      RollingIndex(gps_index+i, GPS_BUFFER_SIZE), 
      RollingIndex(gps_index+i-1, GPS_BUFFER_SIZE), 
      RollingIndex(gps_index+i-2, GPS_BUFFER_SIZE));
  }

  return mean_angle / (GPS_BUFFER_SIZE - 2);
}

static float VarianceGPSAngle()
{
  float M = 0;
  float oldM = 0;
  float S = 0;
  float a = 0;

  for (uint8_t i = 2; i < GPS_BUFFER_SIZE; i++)
  {
    a = ThreePointAngle(
      RollingIndex(gps_index+i, GPS_BUFFER_SIZE), 
      RollingIndex(gps_index+i-1, GPS_BUFFER_SIZE), 
      RollingIndex(gps_index+i-2, GPS_BUFFER_SIZE));
    oldM = M;
    M = M + (a - M) / (i - 1);
    S = S + (a - M) * (a - oldM);
  }

  return S / (GPS_BUFFER_SIZE - 2);
}

static float MeanDisplacementAngle()
{
  float mean_displacement_angle = 0;

  for (uint8_t i = 1; i < GPS_BUFFER_SIZE - 1; i++)
  {
    mean_displacement_angle += ThreePointAngle(
      RollingIndex(gps_index+GPS_BUFFER_SIZE-1, GPS_BUFFER_SIZE), 
      RollingIndex(gps_index+i, GPS_BUFFER_SIZE), 
      gps_index);
  }

  return mean_displacement_angle / (GPS_BUFFER_SIZE - 2);
}

static void Rotate(const int &j, const float lat_lon_mean[])
{
  float x;
  float y;

  float c = cos(DEG_TO_RAD * lat_lon_mean[0]);

  for (uint8_t i = 0; i < GPS_BUFFER_SIZE; i++)
  {
    //Source: http://www.movable-type.co.uk/scripts/latlong.html
    //Equirectangular approximation
    //Formula x = Δλ ⋅ cos φm
    x = (longitude_buffer[i] - lat_lon_mean[1]) * c;
    //y = Δφ
    y = latitude_buffer[i] - lat_lon_mean[0];

    //y′= y cosθ + x sinθ
    rotated_latitude_buffer[i] = y * COS_VALUES[j] + x * SIN_VALUES[j];

    //x′= x cosθ − y sinθ
    rotated_longitude_buffer[i] = x * COS_VALUES[j] - y * SIN_VALUES[j];
  }
}

static int16_t Bearing(const uint8_t &index1, const uint8_t &index2)
{
  float dLon = longitude_buffer[index2] - longitude_buffer[index1];
  float y = sin(DEG_TO_RAD * dLon) * cos(DEG_TO_RAD * latitude_buffer[index2]);
  float x = cos(DEG_TO_RAD * latitude_buffer[index1]) * sin(DEG_TO_RAD * latitude_buffer[index2]) - sin(DEG_TO_RAD * latitude_buffer[index1]) * cos(DEG_TO_RAD * latitude_buffer[index2]) * cos(DEG_TO_RAD * dLon);

  return ((int16_t)(atan2(y, x) * RAD_TO_DEG + 360) % 360);
}

static uint8_t GPSRollingMeanCross(const float data[], const float &mean)
{
  uint8_t mc = 0;

  for (uint8_t i = 1; i < GPS_BUFFER_SIZE; i++)
  {
    mc += MeanCross(
      data[RollingIndex(gps_index+i, GPS_BUFFER_SIZE)], 
      data[RollingIndex(gps_index+i-1, GPS_BUFFER_SIZE)], mean) ? 1 : 0;
  }

  return mc;
}
#endif //GPS_TREE


float generate_random_float(unsigned long random_seed = 0) {
  random_seed = (random_seed * 1103515245UL + 12345UL) % 4294967296UL;
  float u1 = (float)random_seed / 4294967296.0;
  float u2 = (float)random_seed / 4294967296.0;
  float z = sqrt(-2.0 * log(u1)) * cos(2.0 * M_PI * u2);
  return z;
}


void read_random_acc_data_test(
  float acc_data[], 
  // float magnitude_buffer[],
  unsigned long random_seed
  ) 
{
  for (int i = 0; i < (ACC_BUFFER_SIZE*3); i++) {
    
    float acc_x, acc_y, acc_z;
    // uint8_t magnitude_buffer_index = 0;

    // Sample from the standard normal distribution with mean 0 and standard deviation 1.
    float val = generate_random_float(random_seed); 

    // samples[i] = 1.0 + 0.1 * z; // Adjust to mean 1 and standard deviation 0.1.

    if( i % 3 == 0 ) { // 0, 3, ...
      acc_x = 0.0 + 0.2 * val; // Adjust to mean 0 and standard deviation 0.2.
      acc_data[i] = acc_x;
    } 
    else if ( i % 3 == 1 ) {  // 1, 4, ...
      acc_y = 0.0 + 0.2 * val; // Adjust to mean 0 and standard deviation 0.2.
      acc_data[i] = acc_y;
    }
    else { // 2, 5, ...
      acc_z = 1.0 + 0.2 * val; // Adjust to mean 1 and standard deviation 0.2.
      acc_data[i] = acc_z;
      // magnitude_buffer[magnitude_buffer_index++] = sqrt(acc_x*acc_x + acc_y*acc_y + acc_z*acc_z);
    }

    if (i == 0) {
      Serial.print("acc_x: "); Serial.print(acc_x); Serial.print(" ");
    }
    else if (i == 1) {
      Serial.print("acc_y: "); Serial.print(acc_y); Serial.print(" ");
    }
    else if (i == 2) {
      Serial.print("acc_z: "); Serial.print(acc_z); Serial.print(" ");
      Serial.print("magnitude_buffer: "); Serial.print(magnitude_buffer[0]); Serial.println(" ");
    }
  }
}
