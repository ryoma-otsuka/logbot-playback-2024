// -------------------------------------------------------------------------
// v5 umineko 2024 mode
// Sleep control by time 03:30 to 19:00 (v5_umineko_2024_sleep_ctrl_by_time_2.h)
// -------------------------------------------------------------------------

// -------------------------------------------------------------------------
// Select the base mode (select one mode)
// -------------------------------------------------------------------------
#define UMINEKO_2024_MODE
// #define DEBUG_UMINEKO_2024_MODE
// #define INTERVAL_SAMPLING_MODE

// -------------------------------------------------------------------------
// Debug: Uncomment and define the followings for debug
// -------------------------------------------------------------------------
// #define DEBUG_SERIAL_PRINT

// Use them to test camera and speaker without fixing GPS location
// #define DEBUG_FAKE_GPS_IN_SETUP  /// define if you want to debug inside of building or somewhere the logger cannot get gps data
// #define DEBUG_FAKE_GPS_IN_LOOP

// LED for debug
#define DEBUG_1HZ_LED
#define DEBUG_GPS_LED
#define DEBUG_DETECTION_LED

// #define DEBUG_HAND_SHAKE     // define if you want to use shakeWakeUp function to control camera. GPS, and speakers

// Reduce the time required for creating new directories 
// by creating multiple directories immediately after time synchronization
#define MAKE_SAVE_DIRS

// Buffer size for writing to the SD card 
// (how many seconds of data are written in a single write operation)
#define DATA_BUFFER_SIZE_SEC 1 // Write data to SD card every second
// #define DATA_BUFFER_SIZE_SEC 2  //  6 - 14 ms
// #define DATA_BUFFER_SIZE_SEC 5  // 10 - 20 ms
// #define DATA_BUFFER_SIZE_SEC 10 // 20 - 30 ms
// #define DATA_BUFFER_SIZE_SEC 15 // 20 - 30 ms
// #define DATA_BUFFER_SIZE_SEC 20 // 30 - 50 ms
// #define DATA_BUFFER_SIZE_SEC 30 // 50 - 70 ms
// #define DATA_BUFFER_SIZE_SEC 60 // 110 - 120 ms
uint8_t data_buffer_index = 0;

#define RTC_1HZ_CLOCK_PIN_USE_MODE 0 // Do not use RTC_CLOCK_1HZ as input mode 
// #define RTC_1HZ_CLOCK_PIN_USE_MODE 1 // Use only at the start
// #define RTC_1HZ_CLOCK_PIN_USE_MODE 2 // Use every seconds -> this will cause some issues (e.g. freezing)

// Interrupt handling setup
// #define ENABLE_NRF52_TIMER_INTERRUPT

// -------------------------------------------------------------------------
// GPS Settings
// -------------------------------------------------------------------------
// #define GPS_FACTORY_RESET
#define GPS_FACTORY_DEFAULT

// Select only one GPS power mode
// #define GPS_CONTINUOUS_1HZ_MODE
// #define GPS_POWER_ON_OFF_MODE_01ON_02OFF
#define GPS_POWER_ON_OFF_MODE_02ON_13OFF
// #define GPS_POWER_ON_OFF_MODE_03ON_27OFF

#ifndef GPS_CONTINUOUS_1HZ_MODE
  #define GPS_HW_BACKUP_ON_OFF_MODE
  // #define GPS_SW_BACKUP_ON_OFF_MODE
#endif // GPS_CONTINUOUS_1HZ_MODE

#define NAVIGATION_FREQUENCY_1Hz 1
#define NAVIGATION_FREQUENCY_2Hz 2 // 1000/2 = 500
#define NAVIGATION_FREQUENCY_4Hz 4 // 1000/4 = 250
#define NAVIGATION_FREQUENCY_5Hz 5 // 1000/5 = 200
#define MAX_WAIT_250 250
#define MAX_WAIT_300 300

#define GPS_TIMEOUT_MIN 5       // GPS timeout

// -------------------------------------------------------------------------
// Camera & Speaker settings
// -------------------------------------------------------------------------

// Common settings
// 7.2 GB SD: 1 min vids: 400x; 5 min vids: 80x 
// 28.8 GB SD: 1 min vids: 1800x; 5 min vids: 400x
#define MAX_CAMERA_COUNT 400

#define TARGET_BEHAVIOR_CLASS_NUMBER 0 // WARNING: make sure all trees use 0 as the target class number

#define USE_SPEAKER // Uncomment if you want to use speaker: this will uncomment all speaker related codes
#define AUDIO_FILENAME_SIMPLE // Use this if you want to you simple filename version such as _hayabusa or _noise 

#define AUDIO_FILE_LENGTH_4_2 // 4.2 s
// #define AUDIO_FILE_LENGTH_9_4 // 9.4 s
// #define AUDIO_FILE_LENGTH_14_6 // 14.6 s
#ifdef AUDIO_FILE_LENGTH_4_2
  #define VCAM_OFF_WAIT_SEC 5+1
#elif defined(AUDIO_FILE_LENGTH_9_4)
  #define VCAM_OFF_WAIT_SEC 10+1
#elif defined(AUDIO_FILE_LENGTH_14_6)
  #define VCAM_OFF_WAIT_SEC 15+1
#else 
  #define VCAM_OFF_WAIT_SEC 15+1
#endif


// ------------------------------------------------------------------------- 
// Umineko 2024 version
// ------------------------------------------------------------------------- 
#if defined(UMINEKO_2024_MODE)
  #define TRIGGER_TYPE 1 // 1 -> Real Time ABR using ACC data
  #define TARGET_BEHAVIOR_CONSECUTIVE_SEC 5
  #define CAM_SPK_GPS_CONTROL_VERSION 3
  #define ACC_TREE
  #define ACC_TREE_ID 2 // Run_Umineko_Flying_Binary_2_2024() // flying (active flight/passive flight) OR others (stationary/bathing/foraging)
  #define BEHAVIOR_CLASS_BUFFER_SIZE 10
  #define MINIMUM_TARGET_BEHAVIOR_COUNT 6 // 6/10 = 60%
  #define GPS_FIX_TYPE_BUFFER_SIZE 10
  #define OUTSIDE_BOUNDING_BOX_BUFFER_SIZE GPS_FIX_TYPE_BUFFER_SIZE
  // #define CAMERA_REST_TIME_SEC_INIT (60 * 30) // Initial rest time of 30 minutes
  #define CAMERA_REST_TIME_SEC_INIT (60 * 20) // for v5_umineko_2024_sleep_ctrl_by_time_2.h
  #define CAMERA_REST_TIME_SEC   (60 * 10) // 10 min. After audio intervention, the next recording will not begin until 10 min have elapsed.
  #define SPEAKER_REST_TIME_SEC CAMERA_REST_TIME_SEC
  #define CAMERA_RECORD_TIME_SEC ((60 * 3) + 5) // 3 min + 5 sec. NOTE: Added 5 sec for the time lag before the camera is activated.
  #define FINAL_CHECK_STAND_BY_PHASE_SEC (60 + 5)
  #define FINAL_CHECK_PHASE_SEC 60
  #define MINIMUM_SEC_AFTER_PLAY_AUDIO 60
  #define GPS_1HZ_RECORD_AFTER_START_COUNTDOWN_SEC (60 * 5) // Record GPS data at 1Hz for the first 5 minutes after logging starts
  #define FORCE_GPS_1HZ_MODE_SEC (60 * 30) // 30 min
  #define FORCE_GPS_1HZ_MODE_INIT_SEC (60 * 3) // 3 min
  #define STOP_GPS_1HZ_MODE_IF_BIRD_IS_NOT_FLYING
  #define STOP_GPS_1HZ_MODE_IF_BIRD_IS_NOT_FLYING_START_SEC (60 * 20)

// ------------------------------------------------------------------------- 
// Debug mode for Umineko 2024
// ------------------------------------------------------------------------- 
#elif defined(DEBUG_UMINEKO_2024_MODE)
  #define TRIGGER_TYPE 1 // 1 -> Real Time ABR using ACC data
  #define TARGET_BEHAVIOR_CONSECUTIVE_SEC 5
  #define CAM_SPK_GPS_CONTROL_VERSION 3
  #define ACC_TREE
  #define ACC_TREE_ID 2 // Run_Umineko_Flying_Binary_2_2024() // flying (active flight/passive flight) OR others (stationary/bathing/foraging)
  #define BEHAVIOR_CLASS_BUFFER_SIZE 10
  #define MINIMUM_TARGET_BEHAVIOR_COUNT 6 // 6/10 = 60%
  #define GPS_FIX_TYPE_BUFFER_SIZE 10
  #define OUTSIDE_BOUNDING_BOX_BUFFER_SIZE GPS_FIX_TYPE_BUFFER_SIZE
  #define CAMERA_REST_TIME_SEC_INIT 30 // Initial value 30 second
  #define CAMERA_REST_TIME_SEC   (20 * 1)  // 20 sec for debug
  #define SPEAKER_REST_TIME_SEC CAMERA_REST_TIME_SEC
  
  // Record a 3-minute video every 10 minutes
  #define CAMERA_RECORD_TIME_SEC ((60 * 3) + 5) // 3 min sec for debug
  #define FINAL_CHECK_STAND_BY_PHASE_SEC (60 + 5)
  #define FINAL_CHECK_PHASE_SEC 60
  #define MINIMUM_SEC_AFTER_PLAY_AUDIO 60
  #define DEBUG_SAMPLING_TEST // Commenting out will enable only the behavior recognition trigger
  #define DEBUG_SAMPLING_TEST_INTERVAL_MIN 10
  #define NUM_VIDEOS_FOR_DEBUG_SAMPLING_TEST 10

  // Record a 1-minute video every 2 minutes
  // #define CAMERA_RECORD_TIME_SEC ((60 * 1) + 5) // 1 min sec for debug
  // #define FINAL_CHECK_STAND_BY_PHASE_SEC (10 + 5)
  // #define FINAL_CHECK_PHASE_SEC 30
  // #define MINIMUM_SEC_AFTER_PLAY_AUDIO 20
  // #define DEBUG_SAMPLING_TEST
  // #define DEBUG_SAMPLING_TEST_INTERVAL_MIN 2
  // #define NUM_VIDEOS_FOR_DEBUG_SAMPLING_TEST 15

  // #define GPS_1HZ_RECORD_AFTER_START_COUNTDOWN_SEC (60 * 2)
  #define GPS_1HZ_RECORD_AFTER_START_COUNTDOWN_SEC (60 * 5) 
  #define FORCE_GPS_1HZ_MODE_SEC (60 * 30) // 30 min
  #define FORCE_GPS_1HZ_MODE_INIT_SEC (60 * 3) // 3 min
  // #define FORCE_GPS_1HZ_MODE_INIT_SEC (60 * 1) // 1 min
  #define STOP_GPS_1HZ_MODE_IF_BIRD_IS_NOT_FLYING
  #define STOP_GPS_1HZ_MODE_IF_BIRD_IS_NOT_FLYING_START_SEC (60 * 20)
  // #define STOP_GPS_1HZ_MODE_IF_BIRD_IS_NOT_FLYING_START_SEC (60 * 29) // to test this feat 

// ------------------------------------------------------------------------- 
// Interval Sampling Mode
// ------------------------------------------------------------------------- 
#elif defined(INTERVAL_SAMPLING_MODE)
  #define TRIGGER_TYPE 0 // 0 -> sampling at specified intervals
  // Sampling: Record videos at fixed intervals
  #define CAMERA_REST_TIME_SEC_INIT 60 // Initial value 60 second

  // Schedule A. Record a 1-minute video every 2 minutes
  #define CAMERA_SAMPLING_INTERVAL_MIN 2
  #define CAMERA_RECORD_TIME_SEC ((60 * 1) + 2) // 60 sec for debug
  
  // Schedule B. Record a 1 minute and 30 seconds video every 3 minutes
  // #define CAMERA_SAMPLING_INTERVAL_MIN 3
  // #define CAMERA_RECORD_TIME_SEC ((60 * 1 + 30) + 2) // 90 sec for debug

  // Schedule C. Record a 2-minute video every 5 minutes
  // #define CAMERA_SAMPLING_INTERVAL_MIN 5
  // #define CAMERA_RECORD_TIME_SEC ((60 * 2) + 2) // 120 sec for debug

  // Schedule D. Record a 3-minute video every 10 minutes
  // #define CAMERA_SAMPLING_INTERVAL_MIN 10
  // #define CAMERA_RECORD_TIME_SEC ((60 * 3) + 2) // 180 sec for debug

  // Schedule E. Record a 3-minute video every 30 minutes
  // #define CAMERA_SAMPLING_INTERVAL_MIN 30
  // #define CAMERA_RECORD_TIME_SEC ((60 * 3) + 2) // 180 sec for debug

  #define CAMERA_REST_TIME_SEC   (10 * 1)  // 10 sec for debug
  // Not used in this mode (define them just to avoid compile error)
  #define TARGET_BEHAVIOR_CONSECUTIVE_SEC 5
  #define CAM_SPK_GPS_CONTROL_VERSION 2
  #define ACC_TREE
  #define ACC_TREE_ID 2 // Run_Umineko_Flying_Binary_2_2024() // flying (active flight/passive flight) OR others (stationary/bathing/foraging)
  #define GPS_FIX_TYPE_BUFFER_SIZE 5
  #define BEHAVIOR_CLASS_BUFFER_SIZE 10
  #define MINIMUM_TARGET_BEHAVIOR_COUNT 6 // 6/10 = 60%
  #define SPEAKER_REST_TIME_SEC CAMERA_REST_TIME_SEC
  #define FINAL_CHECK_STAND_BY_PHASE_SEC 10
  #define FINAL_CHECK_PHASE_SEC (30 + 2)
  #define MINIMUM_SEC_AFTER_PLAY_AUDIO 20
  #define GPS_1HZ_RECORD_AFTER_START_COUNTDOWN_SEC (60 * 1)
  #define FORCE_GPS_1HZ_MODE_SEC (60 * 30) // 30 min
  #define FORCE_GPS_1HZ_MODE_INIT_SEC (60 * 3) // 3 min
  #define STOP_GPS_1HZ_MODE_IF_BIRD_IS_NOT_FLYING_START_SEC (60 * 20)
#endif


#ifndef CAMERA_SAMPLING_INTERVAL_MIN
  #define CAMERA_SAMPLING_INTERVAL_MIN 2
#endif // CAMERA_SAMPLING_INTERVAL_MIN

#define SPEAKER_TURN_ON_BEHAVIOR_COUNT 15
#define SPEAKER_FINAL_CHECK_THRESHOLD 9

/*
---------------------------------------------------
NOTE:
---------------------------------------------------
TRIGGER_TYPE
  0: sampling at specified intervals
  1: Real Time ABR using ACC data
  2: Real Time ABR using Water Depth data

CAM_SPK_GPS_CONTROL_VERSION
  1: GPS ON -> Camera ON & Final Check
  2: GPS ON & Camera ON -> Stand by Phase ->
      Final check phase -> Post-final check phase 

ACC_TREE_ID
  0: Hand Shake for Debug
  2: Run_Umineko_Flying_Binary_2_2024()
  ---------------------------------------------------
*/

// -------------------------------------------------------------------------
// Data Buffer for Real-time Behavior Recognition
// -------------------------------------------------------------------------
// largest gap in GPS data before the buffer should be restarted
#define MAX_GAP 2 // If consecutive GPS data points are not received, reset the GPS buffer

// Constants used in UserDataType
#define GPS_BUFFER_SIZE   10
#define ACC_READ_SIZE      5
#define ACC_BUFFER_SIZE   25
#define GYRO_BUFFER_SIZE  25

#define DEG_LEN 110.25
#define FPT_KM  2

// -------------------------------------------------------------------------
// Sleep Control
// -------------------------------------------------------------------------
#define UTC_OFFSET 9 // JST: UTC + 9 

// Startup delay
// Decide how many minutes to sleep before starting logging
#define STARTUP_DELAY_MIN  0         // default 0
// #define STARTUP_DELAY_MIN  1         // 1 minutes for debug
// #define STARTUP_DELAY_MIN  2         // 2 minutes for debug
// #define STARTUP_DELAY_MIN 10         // 10 minutes for debug
// #define STARTUP_DELAY_MIN 30         // 30 minutes
// #define STARTUP_DELAY_MIN 60         // 60 minutes
// #define STARTUP_DELAY_MIN (60 * 3)   // 3 hours
// #define STARTUP_DELAY_MIN (60 * 10)  // 10 hours
// #define STARTUP_DELAY_MIN (60 * 24)  // 24 hours

// Specify when to start the first logging.
// Calculate the sleep time by taking the difference from the current time.
// #define LOGGING_START_JST_HOUR 100 // No sleep
// 0, 1, 2, ...., 23, 24
#define LOGGING_START_JST_HOUR  3
// #define LOGGING_START_JST_HOUR  7
// #define LOGGING_START_JST_HOUR  8
// #define LOGGING_START_JST_HOUR  9
// #define LOGGING_START_JST_HOUR 10
// #define LOGGING_START_JST_HOUR 11
// #define LOGGING_START_JST_HOUR 12
// #define LOGGING_START_JST_HOUR 15
// #define LOGGING_START_JST_HOUR 16
// #define LOGGING_START_JST_HOUR 17
// #define LOGGING_START_JST_HOUR 20
// #define LOGGING_START_JST_HOUR 21
// #define LOGGING_START_JST_HOUR 22

// #define LOGGING_START_JST_MIN 0
// #define LOGGING_START_JST_MIN 10
// #define LOGGING_START_JST_MIN 20
#define LOGGING_START_JST_MIN 30
// #define LOGGING_START_JST_MIN 40
// #define LOGGING_START_JST_MIN 50

/*
NOTE:
Suppose the time_initialized was 08:12 in the morning
Next, suppose the time to wake up from sleep is 15:00
Convert 08:12 to minutes → 492 (hour * 60 + minute)
Convert 15:00 to minutes → 900 (hour * 60 + minute)
The difference is 900 - 492 = 408
If the difference is negative, add 24 * 60
sleep_sec = startup_delay_remaining_min * 60;
sleep_nRF52840_by_sending_command_to_ESP32(sleep_sec);
*/

// If it is outside the specified time range, sleep
// Sleep Control by Time
#define SLEEP_CTRL_BY_TIME // Time

// Dusk and Dawn time (0 ~ 1440 = 00:00 ~ 24:00) in minute
// Note that this min_counter is calculated based on JST time
// #define DAWN 0  // 00:00 JST for debugging
// #define DUSK 1440 // 24:00 JST for debugging

// Dusk and Dawn Test
// #define DAWN 700  // 10:00 for sleep mode test
// #define DUSK 710  // 10:30 for sleep mode test

// Umineko 2024
// #define DAWN  240 // 04:00 for Umineko 2024
// #define DUSK 1140 // 19:00 for Umineko 2024

// Umineko 2024 for v5_umineko_2024_sleep_ctrl_by_time_2.h
#define DAWN  210 // 03:30 for Umineko 2024
#define DUSK 1140 // 19:00 for Umineko 2024

// -------------------------------------------------------------------------
// Location Bounding box
// -------------------------------------------------------------------------
#define USE_LOCATION_BOUNDING_BOX_TRIGGER

// Umineko 2024 Bounding Box (Kabushima + 1km)
#define AREA_NE_LAT 40.5477   // North latitude
#define AREA_NE_LON 141.5694  // East longitude
#define AREA_SW_LAT 40.5294   // South latitude
#define AREA_SW_LON 141.5456  // West longitude
// NE: 40.5477, 141.5694
// SW: 40.5294, 141.5456

// Handai IST Bounding Box
// #define AREA_NE_LAT 34.8188   // North latitude
// #define AREA_NE_LON 135.5225  // East longitude
// #define AREA_SW_LAT 34.8174   // South latitude
// #define AREA_SW_LON 135.5210  // West longitude

// -------------------------------------------------------------------------
// Water Pressure Sensor Blue Robotics MS5837
// -------------------------------------------------------------------------
#define USE_WATER_PRESSURE_SENSOR // activate depth sensor if defined
// #define _PLUS_WATER_DEPTH // activate behaviour recognition using depth sensor if defined 

// Used with Cormorants/Streaked Shearwater (Omizunagidori)
#define WATER_PRESSURE_BUFFER_SIZE 3
// #define WATER_PRESSURE_THRESHOLD   20 // in millibars 
// #define WATER_PRESSURE_THRESHOLD   1500 // in millibars 
// NOTE: pressure reading have at least +-5 millibar noise, 
// if possible don't use threshold less than 20 to avoid false positives

#define MS5837_RESET              0x1E
#define MS5837_ADC_READ           0x00
#define MS5837_PROM_READ          0xA0
#define MS5837_CONVERT_D1_8192    0x4A  // (according to datasheet, this requires 20 msec for conversion) 
#define MS5837_CONVERT_D2_8192    0x5A

// -------------------------------------------------------------------------
// Other sensors
// -------------------------------------------------------------------------
// Barometer
#define LPS22HB_1HZ     0x10
#define LPS22HB_10HZ    0x20
#define LPS22HB_25Hz    0x30
#define LPS22HB_50Hz    0x40
#define LPS22HB_75Hz    0x50

// Illuminance sensor
#define MEASUREMENT_ADJUSTMENT_TERM 1.2
#define CONT_AUTO_RES 0x10 
#define ILLUM_THRESHOLD 0 // Threshold value of illuminance sensor

// -------------------------------------------------------------------------
// Compile time & Battery
// -------------------------------------------------------------------------
// Convert __DATE__ to int // NOTE: __DATE__ is a string value of the date at compile time (or more precisely, pre-process time)
#define YEAR ((__DATE__[9] - '0') * 10 + (__DATE__[10] - '0'))

// battery_level_v = 0.0177*battery_level + 0.08
#define BATTERY_LEVEL_V_MAX 4.13

// ------------------------------------------------------------------------
// Declare Global Variables
// ------------------------------------------------------------------------
// battery level
int16_t battery_level = 0;
float battery_level_v = 0;
uint8_t battery_level_percent = 0;

// VBUS state
bool connected_to_pc = false;
bool VBUS_state = false;

// 40 ms routine
uint8_t block_counter = 0;
uint8_t delay_occurred_counter = 0;
bool the_very_first_40ms_loop = true;

unsigned long start_time = 0;
unsigned long current_time = 0;
unsigned long last_time = 0;
unsigned long last_time2 = 0;

unsigned long end_time = 0;
unsigned long process_time = 0;
unsigned long delay_time = 0;

unsigned long start_time_ms = 0;
unsigned long current_time_ms = 0;
unsigned long last_time_ms = 0;

unsigned long process_time_total = 0;
unsigned long process_time_total_copy = 0;
unsigned long delay_time_total = 0;

unsigned long start_time_tmp_block_0 = 0;

// 25 Hz routine
unsigned long start_time_tmp = 0;
unsigned long last_time_tmp = 0;
unsigned long current_time_tmp = 0;
unsigned long process_time_tmp = 0;
unsigned long delay_time_tmp = 0;
unsigned long end_time_tmp = 0;
unsigned long prev_start_time_tmp = 0;

unsigned long start_time_gps = 0;
unsigned long end_time_gps = 0;
unsigned long process_time_gps = 0;

unsigned long start_time_abr = 0;
unsigned long end_time_abr = 0;
unsigned long process_time_abr = 0;

unsigned long start_time_acc = 0;
unsigned long end_time_acc = 0;
unsigned long process_time_acc = 0;
unsigned long prev_start_time_acc = 0;

// 1 Hz routine
unsigned long start_time_1hz = 0;
unsigned long end_time_1hz = 0;
unsigned long process_time_1hz = 0;
unsigned long delay_time_1hz = 0;
unsigned long last_time_1hz = 0;
bool the_very_first_1000ms_loop = true;

unsigned long start_time_sd = 0;
unsigned long last_time_sd = 0;
unsigned long end_time_sd = 0;
unsigned long process_time_sd = 0;
unsigned long delay_time_sd = 0;

#ifdef DEBUG_MODE
unsigned long current_interrupt_time = 0;
unsigned long last_interrupt_time = 0;
unsigned long debug_timer = 0;
#endif //DEBUG_MODE

uint32_t sleep_sec = 0;
bool just_woke_up_from_sleep = false;
bool time_initialized = false;
bool RTC_timer_1Hz = true;
uint16_t min_counter = 0;
uint32_t sec_counter = 0;
uint8_t current_hour = 100;
uint8_t current_min = 100;
uint8_t current_sec = 100;
uint8_t prev_current_min = 100;
uint8_t prev_current_sec = 100;
uint16_t startup_delay_remaining_min = STARTUP_DELAY_MIN;

// SD
bool SD_power_on = false;
uint8_t prev_dir_day = 0;
uint8_t prev_dir_hour = 0;
bool sensor_data_ready_to_write = false;
uint8_t sd_write_start_countdown = 10;
bool sd_first_write_after_start = true;
uint8_t prev_current_min_for_sd_write = 100;
uint8_t current_min_for_sd_write = 100;
uint8_t current_sec_for_sd_write = 100;

// GPS
#ifdef GPS_CONTINUOUS_1HZ_MODE
bool force_gps_1hz_mode = true;
#else //
bool force_gps_1hz_mode = false;
#endif // GPS_CONTINUOUS_1HZ_MODE

int16_t force_gps_1hz_mode_countdown_sec = -1;
// Record GPS data at 1Hz for the first 3 minutes after startup
int16_t force_gps_1hz_mode_init_countdown_sec = FORCE_GPS_1HZ_MODE_INIT_SEC;

uint8_t force_gps_1hz_mode_stop_count = 0;

#ifdef DEBUG_FAKE_GPS_IN_LOOP
  uint16_t gps_1hz_record_after_start_countdown_sec = CAMERA_REST_TIME_SEC_INIT;
#else
  // Record GPS data at 1Hz for the first 5 minutes after startup
  uint16_t gps_1hz_record_after_start_countdown_sec = GPS_1HZ_RECORD_AFTER_START_COUNTDOWN_SEC;
#endif // DEBUG_FAKE_GPS_IN_LOOP

bool gps_power_save_mode = false;
bool gps_power_on = false;
uint8_t gps_wait_time_sec_count_after_start = 0;

#ifdef GPS_SW_BACKUP_ON_OFF_MODE
bool power_off_command_acknowledged = false;
#endif // GPS_SW_BACKUP_ON_OFF_MODE

// Old version for GPS_POWER_ON_OFF_MODE_2
bool gps_on_state = true;
uint8_t gps_target_sec = 0;
uint8_t gps_on_state_counter_min = 0;
uint8_t gps_sleep_counter_min = 0;
// Variable to wait for a continuous position fix for a few seconds before initializing RTC time
uint8_t gps_time_updated_count = 0;
bool gps_location_updated = false;
bool gps_new_data = false;
bool read_gps_data_now = true;
uint16_t gps_fix_1s_counter = 0;
uint32_t gps_fix_40ms_counter = 0;
uint16_t last_min = 0;
uint16_t gps_timeout = 0;

#ifdef GPS_TREE
uint8_t gps_index = 0;
uint16_t gap = 0;       // incremented each time a GPS is not collected, reset when GPS is collected, reset when device first wakes up
uint16_t collected = 0; // incremented each time a new GPS location is counted, reset when gap > MAX_GAP, reset when device first wakes up
uint8_t consec = 0;
float latitude_buffer[GPS_BUFFER_SIZE];           // global variable, so will be initially filled with zeros
float longitude_buffer[GPS_BUFFER_SIZE];          // global variable, so will be initially filled with zeros
float rotated_latitude_buffer[GPS_BUFFER_SIZE];   // global variable, so will be initially filled with zeros
float rotated_longitude_buffer[GPS_BUFFER_SIZE];  // global variable, so will be initially filled with zeros
uint16_t minute_buffer[GPS_BUFFER_SIZE];          // global variable, so will be initially filled with zeros
#endif // GPS_TREE

// Barometer
bool use_barometer = false;

// Illuminance sensor
bool illum_on = false;
uint16_t illuminance_ctrl = 0;

// IMU BMI270
bool imu_power_on = false;
bool acc_suspend_mode = false;
bool gyro_suspend_mode = true;
bool missed_25th_acc_sample = false;

float magnitude_buffer[ACC_BUFFER_SIZE];
float magnitude_buffer_copy[ACC_BUFFER_SIZE];

// Water Pressure Sensor variables
uint16_t pressure_ctrl = 0;
float water_pressure_buffer[WATER_PRESSURE_BUFFER_SIZE];
#ifdef _PLUS_WATER_DEPTH
#define WATER_DEPTH_TREE_ID 0
uint8_t water_depth_outlier = 0;
uint8_t water_pressure_index = 0;
uint8_t water_pressure_fill_level = 0;
#endif //_PLUS_WATER_DEPTH

// Behavior Recognition
int8_t behavior_class_int = -1;
uint8_t target_behavior_count = 0; // Consecutive detection count of the target behavior
uint8_t target_behavior_buffer_1min_index = 0;
uint8_t target_behavior_buffer_1min[60]; // for force_gps_1hz_mode control

uint8_t gps_fix_type_buffer_index = 0;
uint8_t gps_fix_type_buffer[GPS_FIX_TYPE_BUFFER_SIZE]; // for final check
bool gps_fix_type_buf_check_ok = false;

bool outside_bounding_box = true; // Location Bounding Box
bool outside_bounding_box_buffer[OUTSIDE_BOUNDING_BOX_BUFFER_SIZE]; // Buffer to verify that GPS data from the past 10 seconds is in 3D-Fix and outside Kabushima
uint8_t outside_bounding_box_buffer_index = 0;
bool outside_bounding_box_buf_check_ok = false;

#ifdef GPS_CONTINUOUS_1HZ_MODE
  gps_fix_type_buf_check_ok = true;
  outside_bounding_box_buf_check_ok = true;
#endif // GPS_CONTINUOUS_1HZ_MODE

uint8_t behavior_class_buffer_index = 0;
uint8_t behavior_class_buffer[BEHAVIOR_CLASS_BUFFER_SIZE]; // for final check
bool behavior_class_buf_check_ok = false;


// ESP32
bool esp32_enabled = false;

// VCAM ESP32
bool vcam_esp32_enabled = false;

// Trigger Type
uint8_t trigger_type = TRIGGER_TYPE;

uint8_t cam_spk_gps_control_version = CAM_SPK_GPS_CONTROL_VERSION;

// Camera
int16_t camera_count = -1;
bool camera_power_on = false;          // toggled when the camera is actually powered on
bool camera_recording = false;         // toggled when the camera is actually recording
bool camera_command = false;           // toggled when we want the camera to be turned on
bool ack_received = false;             // indicates whether ack recvd after initially supplying power to camera
bool camera_command_sent = false;      // indicates whether we have already sent the timestamp and camera start command
int16_t camera_record_time_countdown_sec = -1; // default -1 
int16_t camera_rest_time_countdown_sec = CAMERA_REST_TIME_SEC_INIT;
int16_t camera_rest_time_after_start_countdown_sec = CAMERA_REST_TIME_SEC_INIT;

#ifdef GPS_CONTINUOUS_1HZ_MODE
bool gps_1hz_mode_stable_and_outside_bb = true;
#else // GPS_CONTINUOUS_1HZ_MODE
bool gps_1hz_mode_stable_and_outside_bb = false;
#endif // GPS_CONTINUOUS_1HZ_MODE

// Speaker
bool final_check_ok = false;
uint8_t final_check_target_behavior_count = 0;
bool play_audio = false;
bool speaker_on = false;
bool audio_played = false;
bool intervention_cancelled = false;
uint8_t speaker_on_sec_count = 0;
bool random_audio_file = true;
uint8_t audio_file_int = 0;
uint8_t last_played_audio_file_int = 2;
int8_t speaker_turn_on_countdown_sec = -1;
int16_t speaker_rest_time_countdown_sec = CAMERA_REST_TIME_SEC_INIT;
uint8_t number_of_audio_file_options = 2; // 2: hayabusa or noise
// uint8_t number_of_audio_file_options = 3; // 3: hayabusa or noise or none
