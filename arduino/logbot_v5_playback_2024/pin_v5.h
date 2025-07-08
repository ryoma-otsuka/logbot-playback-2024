// Pin numbers used in logbot-v5
#define GPS_RXD_PIN            0  // Serial1 TXD1
#define GPS_TXD_PIN            1  // Serial1 RXD1
#define GREEN_LED_PIN          3  // Green LED
#define YELLOW_LED_PIN         4  // Yellow LED
#define SD_CD_PIN              5  // LOGSD DAT1 (CD) (SS?) 
#define VCAM_ESP32_PWR_PIN     6  // 
#define EXT_P4                 7  //
#define EXT_P5                 8  //
#define ESP32_CTRL_TXD_PIN     9  // ESP32 control: TXD (use SoftwareSerial)
#define ESP32_CTRL_RXD_PIN     10 // ESP32 control: RTD (use SoftwareSerial)
#define BATTERY_MES_PIN        11 // WARNING: Set this high before measuring battery level
#define RTC_CLOCK_1HZ          12 // 
#define VCAM_CTRL_RXD_PIN      13 //
#define VCAM_CTRL_TXD_PIN      14 // 14/A0
#define EXT_LED                A1 // 15/A1
#define GPS_PWR_PIN            A2 // 16/A2
#define SD_PWR_PIN             A3 // 17/A3 WARNING: never write to SD card or set pins 5, 24-26 high when this is low
#define ESP32_PWR_PIN          A4 // 18/A4 ESP32 ON/OFF
#define EXT_BOARD_CONNECT_PIN  A5 // 19/A5
#define BATTERY_VOLT_PIN       A6 // 20/A6 This pin is used to measure battery level
#define EXT_P3                 A7 // 21/A7
#define I2C_SDA_PIN            22 //
#define I2C_SCL_PIN            23 // 
#define SD_DAT0_PIN            24 // LOGSD DAT0 (MISO)
#define SD_CMD_PIN             25 // LOGSD CMD (MOSI)
#define SD_CLK_PIN             26 // LOGSD CLK (SCK)
#define EXT_FROM_SCK           27 //
#define EXT_FROM_CS            28 //
#define EXT_FROM_SI            29 //
#define EXT_FROM_SO            30 //
#define EXT_FROM_WP            31 //
#define EXT_FROM_HOLD          32 //
#define CAM_CTRL_TBD           33 //