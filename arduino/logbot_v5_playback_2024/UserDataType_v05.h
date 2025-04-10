#define UserDataType_VERSION 5

// Data structure to store RTC data
struct rtc1_t 
{ // 6 bytes
  uint8_t year; 
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t min;
  uint8_t sec;
};


// see RTC_RX8564LC.ino
struct RTC8564_TIME 
{ 
  uint8_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t min;
  uint8_t sec;
  uint8_t weekday;
};

// Data structure to store GPS data
struct gps_t 
{ // 5 + 4 * 3 = 17 bytes
  uint8_t gps_hour;
  uint8_t gps_min;
  uint8_t gps_sec;
  float latitude;
  float longitude;
  float altitude;
  uint8_t fix_type; // GPS Fix type: (e.g., 2D or 3D fix)
  uint8_t siv;      // SIV: the number of satellites used in fix
};

// Data structure to store IMU data (Acc, Gyro, and Magnet)
struct data1_t 
{ // 300 + 300 + 12 = 612 bytes
  float acc[ACC_BUFFER_SIZE * 3];   // acc_x, acc_y, acc_z // 4 * 25 * 3 = 300
  float gyro[GYRO_BUFFER_SIZE * 3]; // gyro_x, gyro_y, gyro_z // 4 * 25 * 3 = 300
  float mag[3];                     // mag_x, mag_y, mag_z // 4 * 3 = 12
};

// Data structure to store other data
struct data2_t
{ // 24 + 4 + 1 = 29 bytes
  uint32_t atmopress = 0;             // atmospheric pressure data | 4 bytes
  uint16_t illuminance = 0;           // illuminance data | 2 bytes
  uint32_t D1 = 0;                    // water pressure data | 4 bytes
  uint32_t D2 = 0;                    // water pressure data | 4 bytes
  int16_t battery_level = 0;          // battery voltage | 2 bytes
  int8_t behavior_class = -1;         // behaviour class label | 1 byte
  bool camera_command = false;        // decision whether to turn on the camera | 1 byte
  bool camera_recording = false;      // actual camera recording status | 1 byte
  int16_t camera_count = -1;          // the total count of camera recordings | 2 bytes
  bool play_audio = false;            // true if speaker is ON, otherwise false | 1 byte
  bool speaker_on = false;            // true during audio playback, otherwise false | 1 byte
  uint8_t audio_file = 0;             // audio_file int id - record which audio file was played | 1 byte
  float prev_sd_write_time_ms = 0.0;  // SD write time from 1 second ago | 
  uint8_t delay_occurred_counter = 0; // count of delays that occurred (in the main 40 ms loop * 25 = 1000 ms)
};
