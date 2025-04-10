//---------------------------------------------------------------------------
// EPSON TOYOCOM RTC-8564 Test Program
// http://iizukakuromaguro.sakura.ne.jp/365_rtc8564/365_rtc8564.html
//---------------------------------------------------------------------------

#define REG_ADDR_CONTROL1      0x00
#define REG_ADDR_CONTROL2      0x01
#define REG_ADDR_SECONDS       0x02
#define REG_ADDR_MINUTES       0x03
#define REG_ADDR_HOURS         0x04
#define REG_ADDR_DAYS          0x05
#define REG_ADDR_WEEKDAYS      0x06
#define REG_ADDR_MONTHS        0x07
#define REG_ADDR_YEARS         0x08
#define REG_ADDR_MINUTE_ALARM  0x09
#define REG_ADDR_HOUR_ALARM    0x0A
#define REG_ADDR_DAY_ALARM     0x0B
#define REG_ADDR_WEEKDAY_ALARM 0x0C
#define REG_ADDR_CLOCKOUT_FREQ 0x0D
#define REG_ADDR_TIMER_CONTROL 0x0E
#define REG_ADDR_TIMER         0x0F

#define STOP_BIT                5     // CONTROL1
#define INTERRUPT_PERIODIC      4     // CONTROL2
#define ALARM_FLAG              3     // CONTROL2
#define TIMER_FLAG              2     // CONTROL2
#define ALARM_INTERRUPT_ENABLE  1     // CONTROL2
#define TIMER_INTERRUPT_ENABLE  0     // CONTROL2
#define VOLTAGE_LOW             7     // SECONDS
#define ALARM_ENABLE            7     // MIN ALARAM - WEEKDAY ALARAM
#define CLOCK_OUT_ENABLE        7     // CLKOUT
#define CLOCK_OUT_FREQ_32768Hz  0x00  // CLKOUT
#define CLOCK_OUT_FREQ_1024Hz   0x01  // CLKOUT
#define CLOCK_OUT_FREQ_32Hz     0x02  // CLKOUT
#define CLOCK_OUT_FREQ_1Hz      0x03  // CLKOUT
#define TIMER_ENABLE            7     // TIMER CONTROL
#define TIMER_CLOCK_4096Hz      0     // TIMER CONTROL
#define TIMER_CLOCK_64Hz        1     // TIMER CONTROL
#define TIMER_CLOCK_1Hz         2     // TIMER CONTROL
#define TIMER_CLOCK_1_60Hz      3     // TIMER CONTROL

#define MINUTES_MASK   0b01111111
#define HOURS_MASK     0b00111111
#define DAYS_MASK      0b00111111
#define WEEKDAYS_MASK  0b00000111
#define MONTHS_MASK    0b00011111

void rtc8564_get_time( RTC8564_TIME &tm );
void rtc8564_sprintf( char buf[], RTC8564_TIME &tm );

//---------------------------------------------------------------------------
// DECIMAL -> BCD conversion
//---------------------------------------------------------------------------
uint8_t rtc8564_dec2bcd(const uint8_t &data )
{
  return ((( data / 10) << 4) + (data % 10));
}

//---------------------------------------------------------------------------
// BCD -> DECIMAL conversion
//---------------------------------------------------------------------------
uint8_t rtc8564_bcd2dec(const uint8_t &data )
{
  return ((( data >> 4) * 10) + (data % 16));
}

//---------------------------------------------------------------------------
// CLKOUT pin frequency setting
//
// Arguments:
//   CLOCK_OUT_FREQ_32768Hz : 32.768 kHz
//   CLOCK_OUT_FREQ_1024Hz  : 1.024 kHz
//   CLOCK_OUT_FREQ_32Hz    : 32 Hz
//   CLOCK_OUT_FREQ_1Hz     : 1 Hz
//---------------------------------------------------------------------------
void rtc8564_clock_out_freq(const uint8_t &freq )
{
  rtc8564_write_byte( REG_ADDR_CLOCKOUT_FREQ, freq );
}  

//----- Do not output to CLKOUT pin (reduces power consumption) -------------
void rtc8564_clock_out_enable( void )
{
  rtc8564_set_bit( REG_ADDR_CLOCKOUT_FREQ, CLOCK_OUT_ENABLE );
}

//----- Output to CLKOUT pin ------------------------------------------------
void rtc8564_clock_out_disable( void )
{
  rtc8564_clear_bit( REG_ADDR_CLOCKOUT_FREQ, CLOCK_OUT_ENABLE );
} 

//---------------------------------------------------------------------------
// Write one byte to RTC8564
//
// Parameter 1: RTC8564 register address
// Parameter 2: Value to write to the register
//---------------------------------------------------------------------------
void rtc8564_write_byte(const uint8_t &addr, const uint8_t &data)
{
  I2C_write_byte(RTC_RX8564_ADDRESS, addr, data);
}



//---------------------------------------------------------------------------
// Read one byte from RTC8564
//
// Return value: The value read
//---------------------------------------------------------------------------
uint8_t rtc8564_read_byte(const uint8_t &addr)
{
  return I2C_read_byte(RTC_RX8564_ADDRESS, addr);
}

//---------------------------------------------------------------------------
// Check the value of a specific bit in the RTC8564 register
//
// Return value: true if the bit is 1, false if the bit is 0
//---------------------------------------------------------------------------
boolean rtc8564_test_bit(const uint8_t &addr, const uint8_t &bit_position)
{
  uint8_t data;
  
  data = rtc8564_read_byte( addr );
  data &= (0x01 << bit_position);
  if( data == 0x00 ) {
    return false;
  } else {
    return true;
  }
} 

//---------------------------------------------------------------------------
// Set a specific bit in the RTC8564 register
//---------------------------------------------------------------------------
void rtc8564_set_bit(const uint8_t &addr, const uint8_t &bit_position)
{
  uint8_t data;
  
  data = rtc8564_read_byte( addr );
  data |= (0x01 << bit_position);
  rtc8564_write_byte( addr, data ); 
} 

//---------------------------------------------------------------------------
// Clear a specific bit in the RTC8564 register
//---------------------------------------------------------------------------
void rtc8564_clear_bit(const uint8_t &addr, const uint8_t &bit_position)
{
  uint8_t data;
  
  data = rtc8564_read_byte( addr );
  data &= ~(0x01 << bit_position);
  rtc8564_write_byte( addr, data ); 
} 

//---------------------------------------------------------------------------
// RTC8564 initialization
// 
// Set date and time, disable alarm, no CLKOUT output, disable timer
//---------------------------------------------------------------------------
void rtc8564_init(
  const uint8_t &year,
  const uint8_t &month,
  const uint8_t &day,
  const uint8_t &hour,
  const uint8_t &minute,
  const uint8_t &second)
{
  rtc8564_write_byte(REG_ADDR_CONTROL1,0x20); 
  rtc8564_write_byte(REG_ADDR_CONTROL2,0x00);
  rtc8564_set_time( year, month, day, hour, minute, second);
  rtc8564_alarm_disable();
  rtc8564_clock_out_disable();
  rtc8564_write_byte(REG_ADDR_TIMER_CONTROL,0x00);
  rtc8564_write_byte(REG_ADDR_TIMER,0x00);
  rtc8564_clear_bit( REG_ADDR_CONTROL1, STOP_BIT ); 
}

//---------------------------------------------------------------------------
// Alarm function
//    Since date, time, and week can be set individually,
//    it allows use cases like triggering an alarm every Monday at 12:00
//---------------------------------------------------------------------------

//------ Setting of alarm trigger date --------------------------------------
void rtc8564_set_alarm_day(const uint8_t &day)
{
  rtc8564_write_byte(REG_ADDR_DAY_ALARM, rtc8564_dec2bcd(day));
}

//------ Setting of alarm trigger hour --------------------------------------
void rtc8564_set_alarm_hour(const uint8_t &hour)
{
  rtc8564_write_byte(REG_ADDR_HOUR_ALARM, rtc8564_dec2bcd(hour));
}

//------ Setting of alarm trigger minute ------------------------------------
void rtc8564_set_alarm_minute(const uint8_t &minute)
{
  rtc8564_write_byte(REG_ADDR_MINUTE_ALARM, rtc8564_dec2bcd(minute));
}

//------ Setting of alarm trigger week --------------------------------------
void rtc8564_set_alarm_weekday(const uint8_t &weekday)
{
  rtc8564_write_byte(REG_ADDR_WEEKDAY_ALARM, weekday);
}

//------ Set the INT pin to LOW when the alarm is triggered -----------------
void rtc8564_alarm_interrupt_enable( void )
{
  rtc8564_set_bit( REG_ADDR_CONTROL2, ALARM_INTERRUPT_ENABLE );
}

//------ Do not output to the INT pin ---------------------------------------
void rtc8564_alarm_interrupt_disable( void )
{
  rtc8564_clear_bit( REG_ADDR_CONTROL2, ALARM_INTERRUPT_ENABLE );
  rtc8564_alarm_disable();
}

//------ Disable the alarm --------------------------------------------------
void rtc8564_alarm_disable( void )
{
  rtc8564_set_bit( REG_ADDR_DAY_ALARM,     ALARM_ENABLE );
  rtc8564_set_bit( REG_ADDR_HOUR_ALARM,    ALARM_ENABLE );
  rtc8564_set_bit( REG_ADDR_MINUTE_ALARM,  ALARM_ENABLE );
  rtc8564_set_bit( REG_ADDR_WEEKDAY_ALARM, ALARM_ENABLE );
  rtc8564_alarm_clear();
}

//------ Clear the alarm event (alarm trigger flag) -------------------------
void rtc8564_alarm_clear( void )
{
  rtc8564_clear_bit( REG_ADDR_CONTROL2, ALARM_FLAG );
}

//------ Check if the alarm has been triggered ------------------------------
// To detect an alarm event, 
// this needs to be called continuously within the loop
// Return value:
//   true: Event occurred
//   false: No event occurred
//---------------------------------------------------------------------------
boolean rtc8564_alarm_test( void )
{
  if( rtc8564_test_bit(REG_ADDR_CONTROL2, ALARM_FLAG ) ) {
    return true;
  } else {
    return false;
  }
} 

//---------------------------------------------------------------------------
// Timer settings
//  ・Used to trigger events at regular intervals
//  ・The timer is a down-counter that triggers an event when it reaches zero
//--------------------------------------------------------------------------- 

//----- Timer settings --------------------------------------------
// Trigger an event using the combination of two arguments
// Argument 1: Down-counter preset value
// Argument 2: Clock speed for countdown
//-----------------------------------------------------------------
void rtc8564_timer_set(const uint8_t &count, const uint8_t &clock )
{
  rtc8564_timer_disable();
  rtc8564_write_byte(REG_ADDR_TIMER_CONTROL, clock);  
  rtc8564_write_byte(REG_ADDR_TIMER, count);
  rtc8564_timer_enable();
}

//----- Enable the timer ------------------------------------------
void rtc8564_timer_enable( void )
{
  rtc8564_set_bit( REG_ADDR_TIMER_CONTROL, TIMER_ENABLE );
}

//----- Disable the timer -----------------------------------------
void rtc8564_timer_disable( void )
{
  rtc8564_clear_bit( REG_ADDR_TIMER_CONTROL, TIMER_ENABLE );
}

//----- Set the INT pin output to LOW when an event occurs --------
void rtc8564_timer_interrupt_enable( void )
{
  rtc8564_set_bit( REG_ADDR_CONTROL2, TIMER_INTERRUPT_ENABLE );
}

//----- Do not output to the INT pin when an event occurs ---------
void rtc8564_timer_interrupt_disable( void )
{
  rtc8564_clear_bit( REG_ADDR_CONTROL2, TIMER_INTERRUPT_ENABLE );
}

//----- Clear the event trigger flag ------------------------------
void rtc8564_timer_clear( void )
{
  rtc8564_clear_bit( REG_ADDR_CONTROL2, TIMER_FLAG );
}

//----- Test if an event has occurred -----------------------------
// To detect an event, 
// this needs to be called continuously within the loop
// Return value:
//   true: Event occurred
//   false: No event occurred
//-----------------------------------------------------------------
boolean rtc8564_timer_test( void )
{
  if( rtc8564_test_bit(REG_ADDR_CONTROL2, TIMER_FLAG ) ) {
    return true;
  } else {
    return false;
  }
} 

//---------------------------------------------------------------------------
// Set the date and time in RTC8564
//---------------------------------------------------------------------------
void rtc8564_set_time(
  const uint8_t &year,
  const uint8_t &month,
  const uint8_t &day,
  const uint8_t &hour,
  const uint8_t &minute,
  const uint8_t &second)
{
  rtc8564_write_byte(REG_ADDR_SECONDS,  rtc8564_dec2bcd(second));
  rtc8564_write_byte(REG_ADDR_MINUTES,  rtc8564_dec2bcd(minute));  
  rtc8564_write_byte(REG_ADDR_HOURS,    rtc8564_dec2bcd(hour));
  rtc8564_write_byte(REG_ADDR_DAYS,     rtc8564_dec2bcd(day));
  rtc8564_write_byte(REG_ADDR_WEEKDAYS, rtc8564_calc_weekday(2000+year,month,day));   
  rtc8564_write_byte(REG_ADDR_MONTHS,   rtc8564_dec2bcd(month));
  rtc8564_write_byte(REG_ADDR_YEARS,    rtc8564_dec2bcd(year));
} 

//---------------------------------------------------------------------------
// Retrieve the date and time from RTC8564
//
// Argument: Pointer to the date and time structure
//---------------------------------------------------------------------------
void rtc8564_get_time( RTC8564_TIME &tm )
{
  tm.year    = rtc8564_bcd2dec( rtc8564_read_byte( REG_ADDR_YEARS    ));
  tm.month   = rtc8564_bcd2dec( rtc8564_read_byte( REG_ADDR_MONTHS   ) & MONTHS_MASK );
  tm.day     = rtc8564_bcd2dec( rtc8564_read_byte( REG_ADDR_DAYS     ) & DAYS_MASK   );
  tm.hour    = rtc8564_bcd2dec( rtc8564_read_byte( REG_ADDR_HOURS    ) & HOURS_MASK  );
  tm.min     = rtc8564_bcd2dec( rtc8564_read_byte( REG_ADDR_MINUTES  ) & MINUTES_MASK);
  tm.sec     = rtc8564_bcd2dec( rtc8564_read_byte( REG_ADDR_SECONDS  ));
  tm.weekday = rtc8564_bcd2dec( rtc8564_read_byte( REG_ADDR_WEEKDAYS ));
} 

//---------------------------------------------------------------------------
// Format and output the date and time. 
// Call this after obtaining the date and time using rtc8564_get_time()
// 
// Argument 1: Address of the string
// Argument 2: Structure containing the date and time
//---------------------------------------------------------------------------
void rtc8564_sprintf( char buf[], const RTC8564_TIME &tm )
{
  sprintf( buf,"20%02u-%02u-%02u %02u:%02u:%02u\n", 
      tm.year, tm.month, tm.day, tm.hour, tm.min, tm.sec );
}

//---------------------------------------------------------------------------
// Calculate the day of the week using Zeller's formula. 
// RTC8564 cannot calculate the day of the week automatically.
//
// return: 0: Sunday, 6: Saturday
//---------------------------------------------------------------------------
uint8_t rtc8564_calc_weekday( int year, int month, int day )
{
  if( month <= 2 ) {
    month += 12;
    year--;
  }

  return (uint8_t)((year + year/4 - year/100 + year/400 + ((13 * month + 8)/5) + day) % 7);
}

bool adjust_RTC1(const rtc1_t &timedata) {
  if (timedata.year >= YEAR) { 
    // Execute the following only when the GPS data year 
    // is greater than or equal to the compile-time year YEAR
    rtc8564_init(
      timedata.year, 
      timedata.month, 
      timedata.day, 
      timedata.hour, 
      timedata.min, 
      timedata.sec); // Set the time only during the initial power-up
    setCLKOUT_RTC1();
    setTimer_RTC1_1Hz();
    return true;   
  }
  else {
    return false;
  }
}

void setCLKOUT_RTC1() {
  rtc8564_clock_out_freq( CLOCK_OUT_FREQ_1Hz ); // Set the CLKOUT frequency to 1Hz
  rtc8564_clock_out_enable();                   // Output to the CLKOUT pin
}

void setTimer_RTC1_1Hz() {
  rtc8564_set_bit( REG_ADDR_CONTROL2, INTERRUPT_PERIODIC ); // interrupt repeatedly vs 0 for just once
  rtc8564_timer_set( 1, TIMER_CLOCK_1Hz );                  // Trigger a timer event every 1 second
  rtc8564_timer_interrupt_enable();                         // Set the INT pin to LOW on a timer event
}

// Add by Otsuka
void setTimer_RTC1_1_10Hz() {
  rtc8564_set_bit( REG_ADDR_CONTROL2, INTERRUPT_PERIODIC ); // interrupt repeatedly
  rtc8564_timer_set( 
    /* Countdown initial value = */ 10, // 1/60Hz
    /* Countdown frequency = */ TIMER_CLOCK_1Hz );          // Trigger a timer event every 10 seconds
  rtc8564_timer_interrupt_enable();                         // Set the INT pin to LOW on a timer event
}

void setTimer_RTC1_1_60Hz() {
  rtc8564_set_bit( REG_ADDR_CONTROL2, INTERRUPT_PERIODIC ); // interrupt repeatedly
  rtc8564_timer_set( 
    /* Countdown initial value = */ 60, // 1/60Hz
    /* Countdown frequency = */ TIMER_CLOCK_1Hz );          // Trigger a timer event every 60 seconds
  rtc8564_timer_interrupt_enable();                         // Set the INT pin to LOW on a timer event
}


void read_rtc_time(rtc1_t &time) {
  time.year    = rtc8564_bcd2dec( rtc8564_read_byte( REG_ADDR_YEARS    ));
  time.month   = rtc8564_bcd2dec( rtc8564_read_byte( REG_ADDR_MONTHS   ) & MONTHS_MASK );
  time.day     = rtc8564_bcd2dec( rtc8564_read_byte( REG_ADDR_DAYS     ) & DAYS_MASK   );
  time.hour    = rtc8564_bcd2dec( rtc8564_read_byte( REG_ADDR_HOURS    ) & HOURS_MASK  );
  time.min     = rtc8564_bcd2dec( rtc8564_read_byte( REG_ADDR_MINUTES  ) & MINUTES_MASK);
  time.sec     = rtc8564_bcd2dec( rtc8564_read_byte( REG_ADDR_SECONDS  ));

  // The seconds may exceed 80. In that case, subtract 80.
  if(time.sec >= 80)
  {
    time.sec -= 80;
  }
}

void read_time_sec_only(rtc1_t &time) {
  // time.year    = rtc8564_bcd2dec( rtc8564_read_byte( REG_ADDR_YEARS    ));
  // time.month   = rtc8564_bcd2dec( rtc8564_read_byte( REG_ADDR_MONTHS   ) & MONTHS_MASK );
  // time.day     = rtc8564_bcd2dec( rtc8564_read_byte( REG_ADDR_DAYS     ) & DAYS_MASK   );
  // time.hour    = rtc8564_bcd2dec( rtc8564_read_byte( REG_ADDR_HOURS    ) & HOURS_MASK  );
  // time.min     = rtc8564_bcd2dec( rtc8564_read_byte( REG_ADDR_MINUTES  ) & MINUTES_MASK);
  time.sec     = rtc8564_bcd2dec( rtc8564_read_byte( REG_ADDR_SECONDS  ));

  // The seconds may exceed 80. In that case, subtract 80.
  if(time.sec >= 80)
  {
    time.sec -= 80;
  }
}

void show_time() {
  RTC8564_TIME rtc_time;
  char buf[32];

  rtc8564_get_time( rtc_time );
  rtc8564_sprintf( buf, rtc_time );
  // Serial.write( buf );
  // write_SD(buf);
}

String return_time_RTC1() {
  RTC8564_TIME rtc_time;
  char buf[32];

  rtc8564_get_time( rtc_time );
  rtc8564_sprintf( buf, rtc_time );
  return buf;
}

//--------- E N D -----------------------------------------------------------