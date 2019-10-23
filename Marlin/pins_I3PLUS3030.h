#if !defined(__AVR_ATmega1280__) && !defined(__AVR_ATmega2560__)
  #error "Oops!  Make sure you have 'Arduino Mega' selected from the 'Tools -> Boards' menu."
#endif

#ifndef BOARD_NAME
  #define BOARD_NAME "RAMPS 1.4"
#endif

#define LARGE_FLASH true

#define X_STEP_PIN         61
#define X_DIR_PIN          62
#define X_ENABLE_PIN       60
#define X_MIN_PIN          54

#define Y_STEP_PIN         64
#define Y_DIR_PIN          65
#define Y_ENABLE_PIN       2
#define Y_MIN_PIN          24

#define Z_STEP_PIN         67
#define Z_DIR_PIN          69
#define Z_ENABLE_PIN       66
#define Z_MIN_PIN          6

#define E0_STEP_PIN        58 //26
#define E0_DIR_PIN         59 //28
#define E0_ENABLE_PIN      57 //24

#define FAN_PIN            5//56//9 // (Sprinter config)
#define KILL_PIN           -1

#define HEATER_0_PIN       4//10   // EXTRUDER 1
#define HEATER_1_PIN       -1
#define HEATER_2_PIN       -1
#define HEATER_BED_PIN     3
#define TEMP_0_PIN         1    // ANALOG NUMBERING
#define TEMP_1_PIN         -1   // ANALOG NUMBERING
#define TEMP_2_PIN         -1   // ANALOG NUMBERING
#define TEMP_BED_PIN       14   // ANALOG NUMBERING

#define SD_DETECT_PIN      49
#define SERVO0_PIN         7
#define SDSS               53
#define LED_PIN            13

#ifndef FILWIDTH_PIN
	#define FILWIDTH_PIN     12
#endif

#if HAS_DRIVER(TMC2208)
  /**
   * TMC2208 stepper drivers
   *
   * Hardware serial communication ports.
   * If undefined software serial is used according to the pins below
   */
  //#define X_HARDWARE_SERIAL  Serial1
  //#define X2_HARDWARE_SERIAL Serial1
  //#define Y_HARDWARE_SERIAL  Serial1
  //#define Y2_HARDWARE_SERIAL Serial1
  //#define Z_HARDWARE_SERIAL  Serial1
  //#define Z2_HARDWARE_SERIAL Serial1
  //#define E0_HARDWARE_SERIAL Serial1
  //#define E1_HARDWARE_SERIAL Serial1
  //#define E2_HARDWARE_SERIAL Serial1
  //#define E3_HARDWARE_SERIAL Serial1
  //#define E3_HARDWARE_SERIAL Serial1

  /**
   * Software serial
   */

  #define X_SERIAL_TX_PIN    26
  #define X_SERIAL_RX_PIN    10
  #define X2_SERIAL_TX_PIN   -1
  #define X2_SERIAL_RX_PIN   -1

  #define Y_SERIAL_TX_PIN    27
  #define Y_SERIAL_RX_PIN    11
  #define Y2_SERIAL_TX_PIN   -1
  #define Y2_SERIAL_RX_PIN   -1

  #define Z_SERIAL_TX_PIN    28
  #define Z_SERIAL_RX_PIN    12
  #define Z2_SERIAL_TX_PIN   -1
  #define Z2_SERIAL_RX_PIN   -1

  #define E0_SERIAL_TX_PIN   29
  #define E0_SERIAL_RX_PIN   13
  #define E1_SERIAL_TX_PIN   -1
  #define E1_SERIAL_RX_PIN   -1
  #define E2_SERIAL_TX_PIN   -1
  #define E2_SERIAL_RX_PIN   -1
  #define E3_SERIAL_TX_PIN   -1
  #define E3_SERIAL_RX_PIN   -1
  #define E4_SERIAL_TX_PIN   -1
  #define E4_SERIAL_RX_PIN   -1
#endif

#if ENABLED(ULTRA_LCD)
  #define LCD_PINS_RS        37
  #define LCD_PINS_ENABLE    36
  #define LCD_PINS_D4        34
  #define LCD_PINS_D5        35
  #define LCD_PINS_D6        32
  #define LCD_PINS_D7        33

  #define BEEPER_PIN         27
  #define BTN_EN2            28
  #define BTN_EN1            29
  #define BTN_ENC            30
#endif

#define FIL_RUNOUT_PIN      8
