#include "temperature.h"
#include "ultralcd.h"
#ifdef TOUCH_LCD
#include "touch_lcd_address.h"
#include "Marlin.h"
#include "language.h"
#include "cardreader.h"
#include "temperature.h"
#include "stepper.h"
#include "configuration_store.h"
#include "parser.h"
#include <avr/wdt.h>

#if ENABLED(POWER_LOSS_RECOVERY)
  #include "power_loss_recovery.h"
#endif

#include "printcounter.h"

#if ENABLED(PRINTCOUNTER)
  #include "duration_t.h"
#endif

#if ENABLED(FIRST_LAYER_CAL)
  #include "first_lay_cal.h"
#endif

char utf_char_buf;
char utf_string_buf[64];

#include "utf_mapper.h"

#define MAX_MESSAGE_LENGTH VARADDR_STATUS_MSG_LEN

char lcd_status_message[MAX_MESSAGE_LENGTH];
uint8_t lcd_status_update_delay = 1, // First update one loop delayed
        lcd_status_message_level = 0;   // Higher level blocks lower level
       
typedef void(*generalVoidFun)();

static uint16_t ftState = 0x00;
#define FTSTATE_NEED_LOAD_PARAM           0x0001    // need to load paras from eeprom
#define FTSTATE_NEED_SAVE_PARAM           0x0002    // need to save paras to eeprom
#define FTSTATE_EXECUTE_PERIOD_TASK_NOW   0x0004    // excute the lcd period task immediatelly
#define FTSTATE_SERVO_STATUS              0x0008    // servo status
#define FTSTATE_AUTOPID_ING               0x0010    // auto pid
#define FTSTATE_ACTIVE_WARNING            0x0020    // 
#define FTSTATE_STATUS_MESSAGE_NOW        0x0040    // show the lcd status message immediatelly
#define FTSTATE_FIRST_LAYER_CAL           0x0080    // first layer cal
#define FTSTATE_CENTER_ADJUST_ZOFFSET     0x0100    // center adjust zoffset

#define TEMPERTURE_PREHEAT_CHOISE_E1_PLA    0
#define TEMPERTURE_PREHEAT_CHOISE_E1_ABS    1
#define TEMPERTURE_PREHEAT_CHOISE_E1_PET    2
#define TEMPERTURE_PREHEAT_CHOISE_E1_FLEX   3
#define TEMPERTURE_PREHEAT_CHOISE_E1_PVA    4
#define TEMPERTURE_PREHEAT_CHOISE_E1_HIPS   5
#define TEMPERTURE_PREHEAT_CHOISE_E1_PP     6
#define TEMPERTURE_PREHEAT_CHOISE_E1_CUSTOM 7
#define TEMPERTURE_PREHEAT_CHOISE_E2_PLA    8
#define TEMPERTURE_PREHEAT_CHOISE_E2_ABS    9
#define TEMPERTURE_PREHEAT_CHOISE_E2_PET    10
#define TEMPERTURE_PREHEAT_CHOISE_E2_FLEX   11
#define TEMPERTURE_PREHEAT_CHOISE_E2_PVA    12
#define TEMPERTURE_PREHEAT_CHOISE_E2_HIPS   13
#define TEMPERTURE_PREHEAT_CHOISE_E2_PP     14
#define TEMPERTURE_PREHEAT_CHOISE_E2_CUSTOM 15
#define TEMPERTURE_PREHEAT_CHOISE_BED_PLA    16
#define TEMPERTURE_PREHEAT_CHOISE_BED_ABS    17
#define TEMPERTURE_PREHEAT_CHOISE_BED_PET    18
#define TEMPERTURE_PREHEAT_CHOISE_BED_FLEX   19
#define TEMPERTURE_PREHEAT_CHOISE_BED_PVA    20
#define TEMPERTURE_PREHEAT_CHOISE_BED_HIPS   21
#define TEMPERTURE_PREHEAT_CHOISE_BED_PP     22
#define TEMPERTURE_PREHEAT_CHOISE_BED_CUSTOM 23

char filament_choice = 0;
uint8_t unload_purge_times;
char optionId = 0;
uint16_t currentPageId = 0xFFFF, retPageId = 0xFFFF;
uint16_t dwinFileWindowTopIndex = 0;

int16_t lcd_preheat_hotend_temp[FILAMENTS] = { PREHEAT_1_TEMP_HOTEND, \
                                               PREHEAT_2_TEMP_HOTEND, \
                                               PREHEAT_3_TEMP_HOTEND, \
                                               PREHEAT_4_TEMP_HOTEND, \
                                               PREHEAT_5_TEMP_HOTEND, \
                                               PREHEAT_6_TEMP_HOTEND, \
                                               PREHEAT_7_TEMP_HOTEND, \
                                               PREHEAT_8_TEMP_HOTEND };
int16_t lcd_preheat_bed_temp[FILAMENTS] = {  PREHEAT_1_TEMP_BED, \
                                             PREHEAT_2_TEMP_BED, \
                                             PREHEAT_3_TEMP_BED, \
                                             PREHEAT_4_TEMP_BED, \ 
                                             PREHEAT_5_TEMP_BED, \ 
                                             PREHEAT_6_TEMP_BED, \ 
                                             PREHEAT_7_TEMP_BED, \ 
                                             PREHEAT_8_TEMP_BED };
int16_t lcd_preheat_fan_speed[FILAMENTS] = { PREHEAT_1_FAN_SPEED, \
                                             PREHEAT_2_FAN_SPEED, \
                                             PREHEAT_3_FAN_SPEED, \
                                             PREHEAT_4_FAN_SPEED, \
                                             PREHEAT_5_FAN_SPEED, \
                                             PREHEAT_6_FAN_SPEED, \
                                             PREHEAT_7_FAN_SPEED, \
                                             PREHEAT_8_FAN_SPEED };

static uint8_t lcd_icon_state = 0;

float move_dis, move_feedrate;
generalVoidFun periodFun = nullptr;
static touch_lcd myFysTLcd;

#define MOVE_E_LENGTH_EACH_TIME 5.2
#define MOVE_E_FEEDRATE 3.0
#define MOVE_XYZ_FEEDRATE 50.0

#if ENABLED(SDSUPPORT) && PIN_EXISTS(SD_DETECT)
  uint8_t lcd_sd_status;
#endif

static int16_t oldFanSpeed = 0;
static uint16_t oldFeedratePercentage = 0;

bool touch_lcd_aborting_print = false;

#if ENABLED(FIRST_LAYER_CAL)
  static uint8_t first_layer_cal_step = 0;
#endif

#if ENABLED(CENTER_ADJUST_ZOFFSET)
  static uint8_t center_adjust_step = 0;
#endif

#if ENABLED(TMC_Z_CALIBRATION)
  static bool lcd_calibrating_z = 0;
#endif

static void lcd_period_report(int16_t s);
static void lcd_period_prompt_report();
static void lcd_period_task(millis_t& tNow);
static void lcd_event();
static void lcd_check();
static void dwin_on_cmd(millis_t& tNow);
static void lcd_init_datas();
static void dwin_set_status(const char * msg);

#if ENABLED(BABYSTEPPING)
  long babysteps_done = 0;
  int16_t lcd_babysteps = 20;
  #if ENABLED(BABYSTEP_ZPROBE_OFFSET)
    static void lcd_babystep_zoffset();
  #else
    static void lcd_babystep_z();
  #endif
#endif

static void dwin_read_extruders_para() {
  myFysTLcd.ftCmdStart(VARADDR_ACTIVE_EXTRUDER_PARAM);
  if (myFysTLcd.ftCmdReceive(22)) {
    myFysTLcd.ftCmdJump(2);// ignore hotend number
    int16_t t;
    myFysTLcd.ftCmdGetI16(t);
    myFysTLcd.ftCmdGetF32(PID_PARAM(Kp, 0));
    myFysTLcd.ftCmdGetF32(PID_PARAM(Ki, 0));
    PID_PARAM(Ki, 0) = scalePID_i(PID_PARAM(Ki, 0));
    myFysTLcd.ftCmdGetF32(PID_PARAM(Kd, 0));
    PID_PARAM(Kd, 0) = scalePID_d(PID_PARAM(Kd, 0));
    #if ENABLED(PID_EXTRUSION_SCALING)
      myFysTLcd.ftCmdGetF32(PID_PARAM(Kc, 0));
    #else
      myFysTLcd.ftCmdJump(4);
    #endif
    thermalManager.setTargetHotend(t, 0);

    #if HAS_TEMP_BED
      myFysTLcd.ftCmdGetI16(t);
      thermalManager.setTargetBed(t);
    #endif
  }

  #if EXTRUDERS == 2

    myFysTLcd.ftCmdStart(VARADDR_ACTIVE_EXTRUDERS_PARAM);
    if (myFysTLcd.ftCmdReceive(20)) {
      myFysTLcd.ftCmdJump(2);// ignore hotend number
      int16_t t;
      myFysTLcd.ftCmdGetI16(t);
      myFysTLcd.ftCmdGetF32(PID_PARAM(Kp, 1));
      myFysTLcd.ftCmdGetF32(PID_PARAM(Ki, 1));
      PID_PARAM(Ki, 1) = scalePID_i(PID_PARAM(Ki, 1));
      myFysTLcd.ftCmdGetF32(PID_PARAM(Kd, 1));
      PID_PARAM(Kd, 1) = scalePID_d(PID_PARAM(Kd, 1));
      #if ENABLED(PID_EXTRUSION_SCALING)
        myFysTLcd.ftCmdGetF32(PID_PARAM(Kc, 1));
      #else
        myFysTLcd.ftCmdJump(4);
      #endif
      thermalManager.setTargetHotend(t, 1);
    }

  #endif
}

static void dwin_write_extruders_para() {
  myFysTLcd.ftCmdStart(VARADDR_ACTIVE_EXTRUDER_PARAM);
  //int n = active_extruder;
  myFysTLcd.ftCmdPut16(0);
  myFysTLcd.ftCmdPut16(thermalManager.target_temperature[0]);
  myFysTLcd.ftCmdPutF32(PID_PARAM(Kp, 0));
  myFysTLcd.ftCmdPutF32(unscalePID_i(PID_PARAM(Ki, 0)));
  myFysTLcd.ftCmdPutF32(unscalePID_d(PID_PARAM(Kd, 0)));
  #if ENABLED(PID_EXTRUSION_SCALING)
    myFysTLcd.ftCmdPutF32(PID_PARAM(Kc, 0));
  #else
    myFysTLcd.ftCmdJump(4);
  #endif
  
  #if HAS_TEMP_BED
    myFysTLcd.ftCmdPut16(thermalManager.target_temperature_bed);
  #else
    myFysTLcd.ftCmdJump(2);
  #endif
  myFysTLcd.ftCmdSend();

  #if EXTRUDERS == 2
    myFysTLcd.ftCmdStart(VARADDR_ACTIVE_EXTRUDERS_PARAM);
    myFysTLcd.ftCmdPut16(1);
    myFysTLcd.ftCmdPut16(thermalManager.target_temperature[1]);
    myFysTLcd.ftCmdPutF32(PID_PARAM(Kp, 1));
    myFysTLcd.ftCmdPutF32(unscalePID_i(PID_PARAM(Ki, 1)));
    myFysTLcd.ftCmdPutF32(unscalePID_d(PID_PARAM(Kd, 1)));
    #if ENABLED(PID_EXTRUSION_SCALING)
      myFysTLcd.ftCmdPutF32(PID_PARAM(Kc, 1));
    #else
      myFysTLcd.ftCmdJump(4);
    #endif

    myFysTLcd.ftCmdSend();
  #endif
}

static void dwin_write_print_tune_para(){
  uint8_t e;
  myFysTLcd.ftCmdStart(VARADDR_PARAM_TUNE);
  #if HAS_TEMP_ADC_0
    myFysTLcd.ftCmdPut16(thermalManager.target_temperature[0]);
  #else
    myFysTLcd.ftCmdJump(2);
  #endif
  #if HAS_TEMP_ADC_1
    myFysTLcd.ftCmdPut16(thermalManager.target_temperature[1]);
  #else
    myFysTLcd.ftCmdJump(2);
  #endif
  #if HAS_TEMP_ADC_2
    myFysTLcd.ftCmdPut16(thermalManager.target_temperature[2]);
  #else
    myFysTLcd.ftCmdJump(2);
  #endif
  #if HAS_TEMP_ADC_3
    myFysTLcd.ftCmdPut16(thermalManager.target_temperature[3]);
  #else
    myFysTLcd.ftCmdJump(2);
  #endif
  #if HAS_TEMP_ADC_4
    myFysTLcd.ftCmdPut16(thermalManager.target_temperature[4]);
  #else
    myFysTLcd.ftCmdJump(2);
  #endif
  #if HAS_TEMP_BED
    myFysTLcd.ftCmdPut16(thermalManager.target_temperature_bed);
  #else
    myFysTLcd.ftCmdJump(2);
  #endif

  // VARADDR_PARAM_TUNE + 0x06
  #if FAN_COUNT > 0
    for(e=0; e<FAN_COUNT; e++) myFysTLcd.ftCmdPut16(fanSpeeds[e]);
    for (; e<5; e++) myFysTLcd.ftCmdJump(2);
  #else
    myFysTLcd.ftCmdJump(10);
  #endif

  // VARADDR_PARAM_TUNE + 0x0b
  myFysTLcd.ftCmdPut16(feedrate_percentage);
  for (e = 0; e < EXTRUDERS; e++) {
    myFysTLcd.ftCmdPut16(planner.flow_percentage[e]); 
  }
  for (; e < 5; e++) myFysTLcd.ftCmdJump(2);
  
  myFysTLcd.ftCmdSend();
}

static void dwin_read_print_tune_para()
{
    int16_t t;
    myFysTLcd.ftCmdStart(VARADDR_PARAM_TUNE);
    if (myFysTLcd.ftCmdReceive(32)) {
      #if HAS_TEMP_ADC_0
        myFysTLcd.ftCmdGetI16(t);
        thermalManager.setTargetHotend(t, 0);
      #else
        myFysTLcd.ftCmdJump(2);
      #endif
      #if HAS_TEMP_ADC_1
        myFysTLcd.ftCmdGetI16(t);
        thermalManager.setTargetHotend(t, 1);
      #else
        myFysTLcd.ftCmdJump(2);
      #endif
      #if HAS_TEMP_ADC_2
        myFysTLcd.ftCmdGetI16(t);
        thermalManager.setTargetHotend(t,2);
      #else
        myFysTLcd.ftCmdJump(2);
      #endif
      #if HAS_TEMP_ADC_3
        myFysTLcd.ftCmdGetI16(t);
        thermalManager.setTargetHotend(t,3);
      #else
        myFysTLcd.ftCmdJump(2);
      #endif
      #if HAS_TEMP_ADC_4
        myFysTLcd.ftCmdGetI16(t);
        thermalManager.setTargetHotend(t, 4);
      #else
        myFysTLcd.ftCmdJump(2);
      #endif
      #if HAS_TEMP_BED
        myFysTLcd.ftCmdGetI16(t);
        thermalManager.setTargetBed(t);
      #else
        myFysTLcd.ftCmdJump(2);
      #endif

      uint8_t e;
      #if FAN_COUNT > 0
        for (e = 0; e<FAN_COUNT; e++) myFysTLcd.ftCmdGetI16(fanSpeeds[e]);
        for (; e<5; e++) myFysTLcd.ftCmdJump(2);
      #else
        myFysTLcd.ftCmdJump(10);
      #endif
      
      myFysTLcd.ftCmdGetI16(feedrate_percentage);
      for (e = 0; e < EXTRUDERS; e++) {         
		    myFysTLcd.ftCmdGetI16(planner.flow_percentage[e]);
		    planner.refresh_e_factor(e);
      }
      for (; e<5; e++)myFysTLcd.ftCmdJump(2);		
    }
}

static void dwin_write_print_tune_fan_para(){
  uint8_t e;
  myFysTLcd.ftCmdStart(VARADDR_PARAM_TUNE+6);  
  // VARADDR_PARAM_TUNE + 6
  #if FAN_COUNT > 0
    for(e=0;e<FAN_COUNT;e++) {
      myFysTLcd.ftCmdPut16(fanSpeeds[e]);
    }
    for (; e<5; e++)myFysTLcd.ftCmdJump(2);
  #else
    myFysTLcd.ftCmdJump(10);
  #endif

  myFysTLcd.ftCmdSend();
}

static void dwin_read_print_tune_fan_para()
{
    int16_t t;
    myFysTLcd.ftCmdStart(VARADDR_PARAM_TUNE+6);
    if (myFysTLcd.ftCmdReceive(10)) {      
      uint8_t e;
      #if FAN_COUNT > 0
        for (e = 0; e<FAN_COUNT; e++) myFysTLcd.ftCmdGetI16(fanSpeeds[e]);
        for (; e<5; e++) myFysTLcd.ftCmdJump(2);
      #else
        myFysTLcd.ftCmdJump(10);
      #endif	
    }
}

static void dwin_write_motor_para() {    
  myFysTLcd.ftCmdStart(VARADDR_PARAM_MOTOR);
  for (uint8_t i = 0; i < 4; i++)
  {
      myFysTLcd.ftCmdPutF32(planner.axis_steps_per_mm[i]);
  }
  for (uint8_t i = 0; i < 4; i++)
  {
      myFysTLcd.ftCmdPutF32(planner.max_feedrate_mm_s[i]);
  }
  myFysTLcd.ftCmdSend();

  for (uint8_t i = 0; i < 4; i++)
  {
      myFysTLcd.ftCmdPutI32((int32_t)planner.max_acceleration_mm_per_s2[i]);
  }
  myFysTLcd.ftCmdPutF32(planner.acceleration);
  myFysTLcd.ftCmdPutF32(planner.retract_acceleration);
  myFysTLcd.ftCmdPutF32(planner.travel_acceleration);
  myFysTLcd.ftCmdPutF32(planner.min_feedrate_mm_s);
  myFysTLcd.ftCmdSend();

  //myFysTLcd.ftCmdPutI32((int32_t)planner.min_segment_time);
  myFysTLcd.ftCmdPutI32((int32_t)planner.min_segment_time_us);
  myFysTLcd.ftCmdPutF32(planner.min_travel_feedrate_mm_s);
  for(uint8_t i=0;i<4;i++)
  {
      myFysTLcd.ftCmdPutF32(planner.max_jerk[i]);
  }
  myFysTLcd.ftCmdSend();

#if HAS_HOME_OFFSET
  #if ENABLED(DELTA)
  myFysTLcd.ftCmdJump(8);
  float f=DELTA_HEIGHT + home_offset[Z_AXIS];
  myFysTLcd.ftCmdPutF32(f);
  #else
  for(uint8_t i=0;i<3;i++)
  {
      myFysTLcd.ftCmdPutF32(home_offset[i]);
  }
  #endif
#else
  myFysTLcd.ftCmdJump(12);
#endif
#if HAS_MOTOR_CURRENT_PWM
  for (uint8_t q = 0; q<3;q++)
  {
      myFysTLcd.ftCmdPutI32((int32_t)stepper.motor_current_setting[q]);
  }
  myFysTLcd.ftCmdPutI32(MOTOR_CURRENT_PWM_RANGE);
#else
  myFysTLcd.ftCmdJump(16);
#endif
  myFysTLcd.ftCmdSend();
}
static void dwin_read_motor_para() {   
  myFysTLcd.ftCmdStart(VARADDR_PARAM_MOTOR);
  if (myFysTLcd.ftCmdReceive(32))
  {
      for (uint8_t i = 0; i < 4; i++)
      {
          myFysTLcd.ftCmdGetF32(planner.axis_steps_per_mm[i]);
      }
			planner.refresh_positioning(); // geo-f:20180726
      for (uint8_t i = 0; i < 4; i++)
      {
          myFysTLcd.ftCmdGetF32(planner.max_feedrate_mm_s[i]);
      }
  }
  myFysTLcd.ftCmdClear();
  if (myFysTLcd.ftCmdReceive(32))
  {
      int32_t t;
      for (uint8_t i = 0; i < 4; i++)
      {
          myFysTLcd.ftCmdGetI32(t);
          planner.max_acceleration_mm_per_s2[i] = t;
      }
      myFysTLcd.ftCmdGetF32(planner.acceleration);
      myFysTLcd.ftCmdGetF32(planner.retract_acceleration);
      myFysTLcd.ftCmdGetF32(planner.travel_acceleration);
      myFysTLcd.ftCmdGetF32(planner.min_feedrate_mm_s);
  }
  myFysTLcd.ftCmdClear();
  if (myFysTLcd.ftCmdReceive(24))
  {
      int32_t t;
      myFysTLcd.ftCmdGetI32(t);
      //planner.min_segment_time = t;
      planner.min_segment_time_us = t; 
      myFysTLcd.ftCmdGetF32(planner.min_travel_feedrate_mm_s);
      myFysTLcd.ftCmdGetF32(planner.max_jerk[X_AXIS]);
      myFysTLcd.ftCmdGetF32(planner.max_jerk[Y_AXIS]);
      myFysTLcd.ftCmdGetF32(planner.max_jerk[Z_AXIS]);
      myFysTLcd.ftCmdGetF32(planner.max_jerk[E_AXIS]);
  }
  myFysTLcd.ftCmdClear();
  if (myFysTLcd.ftCmdReceive(24))
  {
#if HAS_HOME_OFFSET
  #if ENABLED(DELTA)
      myFysTLcd.ftCmdJump(8);
      myFysTLcd.ftCmdGetF32(home_offset[Z_AXIS]);
      home_offset[Z_AXIS] -= DELTA_HEIGHT;
  #else
      for (uint8_t i = 0; i < 3; i++)
      {
          myFysTLcd.ftCmdGetF32(home_offset[i]);
      }
  #endif
#else
      myFysTLcd.ftCmdJump(12);
#endif
#if HAS_MOTOR_CURRENT_PWM
      int32_t t;
      for (uint8_t q = 0; q<3; q++)
      {
          myFysTLcd.ftCmdGetI32(t);
          stepper.motor_current_setting[q]=t;
      }
#else
      myFysTLcd.ftCmdJump(12);
#endif
  }
}

#if HOTENDS > 1
static void dwin_write_extruders_offset() {
  uint8_t e;
  myFysTLcd.ftCmdStart(VARADDR_PARAM_EXTRUDERS_OFFSET);
  #if EXTRUDERS==5
    for (e = 1; e < 4; e++)
    {
        for (uint8_t axis = 0; axis < 3; axis++)
        {
            myFysTLcd.ftCmdPutF32(hotend_offset[axis][e]);
        }
    }
    myFysTLcd.ftCmdSend(); 
    for (uint8_t axis = 0; axis < 3; axis++)
    {
        myFysTLcd.ftCmdPutF32(hotend_offset[axis][4]);
    }
    myFysTLcd.ftCmdSend(); 
  #else
    for (e = 1; e < EXTRUDERS; e++)
    {
        for (uint8_t axis = 0; axis < 3; axis++)
        {
            myFysTLcd.ftCmdPutF32(hotend_offset[axis][e]);
        }
    }
    myFysTLcd.ftCmdSend();
  #endif
}
static void dwin_read_extruders_offset()
{
  myFysTLcd.ftCmdStart(VARADDR_PARAM_EXTRUDERS_OFFSET);
  #if EXTRUDERS==5
    if (myFysTLcd.ftCmdReceive(36))
    {
        for (uint8_t e = 1; e < 4; e++)
        for (uint8_t axis = 0; axis < 3; axis++)
        {
            myFysTLcd.ftCmdGetF32(hotend_offset[axis][e]);
        }
    }
    myFysTLcd.ftCmdClear();
    if (myFysTLcd.ftCmdReceive(12))
    {
        for (uint8_t axis = 0; axis < 3; axis++)
        {
            myFysTLcd.ftCmdGetF32(hotend_offset[axis][4]);
        }
    }
  #else
    if (myFysTLcd.ftCmdReceive((EXTRUDERS - 1) * 12))
    {
        for (uint8_t e = 1; e < EXTRUDERS; e++)
        for (uint8_t axis = 0; axis < 3; axis++)
        {
            myFysTLcd.ftCmdGetF32(hotend_offset[axis][e]);
        }
    }
  #endif
}
static void dwin_write_extruders_motor()
{
    myFysTLcd.ftCmdStart(VARADDR_PARAM_EXTRUDERS_MOTOR);
#if EXTRUDERS==5
    for (uint8_t e = 1; e < 4; e++)
    {
        myFysTLcd.ftCmdPutF32(planner.axis_steps_per_mm[3 + e]);
        myFysTLcd.ftCmdPutF32(planner.max_feedrate_mm_s[3 + e]);
        myFysTLcd.ftCmdPutF32(planner.max_acceleration_mm_per_s2[3 + e]);
    }
    myFysTLcd.ftCmdSend();
    myFysTLcd.ftCmdPutF32(planner.axis_steps_per_mm[7]);
    myFysTLcd.ftCmdPutF32(planner.max_feedrate_mm_s[7]);
    myFysTLcd.ftCmdPutF32(planner.max_acceleration_mm_per_s2[7]);
    myFysTLcd.ftCmdSend();
#else
    for (uint8_t e = 1; e < EXTRUDERS; e++)
    {
        myFysTLcd.ftCmdPutF32(planner.axis_steps_per_mm[3 + e]);
        myFysTLcd.ftCmdPutF32(planner.max_feedrate_mm_s[3 + e]);
        myFysTLcd.ftCmdPutF32(planner.max_acceleration_mm_per_s2[3 + e]);
    }
    myFysTLcd.ftCmdSend();
#endif
}
static void dwin_read_extruders_motor_para()
{
    int32_t t;
    myFysTLcd.ftCmdStart(VARADDR_PARAM_EXTRUDERS_MOTOR);
#if EXTRUDERS==5
    if (myFysTLcd.ftCmdReceive(36))
    {
        for (uint8_t e = 1; e < 4; e++)
        {
            myFysTLcd.ftCmdGetF32(planner.axis_steps_per_mm[3 + e]);
            myFysTLcd.ftCmdGetF32(planner.max_feedrate_mm_s[3 + e]);
            myFysTLcd.ftCmdGetI32(t);
            planner.max_acceleration_mm_per_s2[3 + e] = t;
        }
    }
    myFysTLcd.ftCmdClear();
    if (myFysTLcd.ftCmdReceive(12))
    {
        myFysTLcd.ftCmdGetF32(planner.axis_steps_per_mm[7]);
        myFysTLcd.ftCmdGetF32(planner.max_feedrate_mm_s[7]);
        myFysTLcd.ftCmdGetI32(t);
        planner.max_acceleration_mm_per_s2[7] = t;
    }
#else
    if (myFysTLcd.ftCmdReceive((EXTRUDERS - 1) * 12))
    {
        for (uint8_t e = 1; e < EXTRUDERS; e++)
        {
            myFysTLcd.ftCmdGetF32(planner.axis_steps_per_mm[3 + e]);
            myFysTLcd.ftCmdGetF32(planner.max_feedrate_mm_s[3 + e]);
            myFysTLcd.ftCmdGetI32(t);
            planner.max_acceleration_mm_per_s2[3 + e] = t;
        }
    }
#endif
}
static void dwin_write_extruders_temp() {
  uint8_t e;
  myFysTLcd.ftCmdStart(VARADDR_PARAM_EXTRUDERS_TEMP);
  #if EXTRUDERS>3
    for (e = 1; e < 3; e++) {
      myFysTLcd.ftCmdPutF32(PID_PARAM(Kp, e));
      myFysTLcd.ftCmdPutF32(unscalePID_i(PID_PARAM(Ki, e)));
      myFysTLcd.ftCmdPutF32(unscalePID_d(PID_PARAM(Kd, e)));
      #if ENABLED(PID_EXTRUSION_SCALING)
        myFysTLcd.ftCmdPutF32(PID_PARAM(Kc, e));
      #else
        myFysTLcd.ftCmdJump(4);
      #endif
    }
    myFysTLcd.ftCmdSend();
    for (; e < EXTRUDERS; e++) {
      myFysTLcd.ftCmdPutF32(PID_PARAM(Kp, e));
      myFysTLcd.ftCmdPutF32(unscalePID_i(PID_PARAM(Ki, e)));
      myFysTLcd.ftCmdPutF32(unscalePID_d(PID_PARAM(Kd, e)));
      #if ENABLED(PID_EXTRUSION_SCALING)
        myFysTLcd.ftCmdPutF32(PID_PARAM(Kc, e));
      #else
        myFysTLcd.ftCmdJump(4);
      #endif
    }
    myFysTLcd.ftCmdSend();
  #else
    for (e = 1; e < EXTRUDERS; e++)
    {
        myFysTLcd.ftCmdPutF32(PID_PARAM(Kp, e));
        myFysTLcd.ftCmdPutF32(unscalePID_i(PID_PARAM(Ki, e)));
        myFysTLcd.ftCmdPutF32(unscalePID_d(PID_PARAM(Kd, e)));
        #if ENABLED(PID_EXTRUSION_SCALING)
        myFysTLcd.ftCmdPutF32(PID_PARAM(Kc, e));
        #else
        myFysTLcd.ftCmdJump(4);
        #endif
    }
    myFysTLcd.ftCmdSend();
  #endif
}
static void dwin_read_extruders_temp()
{
  uint8_t e;
  myFysTLcd.ftCmdStart(VARADDR_PARAM_EXTRUDERS_TEMP);
  #if EXTRUDERS>3
    if (myFysTLcd.ftCmdReceive(32)) {
        for (e = 1; e < 3; e++)
        {
            myFysTLcd.ftCmdGetF32(PID_PARAM(Kp, e));
            myFysTLcd.ftCmdGetF32(PID_PARAM(Ki, e));
            PID_PARAM(Ki, e) = scalePID_i(PID_PARAM(Ki, e));
            myFysTLcd.ftCmdGetF32(PID_PARAM(Kd, e));
            PID_PARAM(Kd, e) = scalePID_d(PID_PARAM(Kd, e));
        #if ENABLED(PID_EXTRUSION_SCALING)
            myFysTLcd.ftCmdGetF32(PID_PARAM(Kc, e));
        #else
            myFysTLcd.ftCmdJump(4);
        #endif
        }
    }
    myFysTLcd.ftCmdClear();
    if (myFysTLcd.ftCmdReceive(32))
    {
        for (; e < EXTRUDERS; e++)
        {
            myFysTLcd.ftCmdGetF32(PID_PARAM(Kp, e));
            myFysTLcd.ftCmdGetF32(PID_PARAM(Ki, e));
            PID_PARAM(Ki, e) = scalePID_i(PID_PARAM(Ki, e));
            myFysTLcd.ftCmdGetF32(PID_PARAM(Kd, e));
            PID_PARAM(Kd, e) = scalePID_d(PID_PARAM(Kd, e));
        #if ENABLED(PID_EXTRUSION_SCALING)
            myFysTLcd.ftCmdGetF32(PID_PARAM(Kc, e));
        #else
            myFysTLcd.ftCmdJump(4);
        #endif
        }
    }
  #else
    if (myFysTLcd.ftCmdReceive(16*(EXTRUDERS-1)))
    {
        for (e = 1; e < EXTRUDERS; e++)
        {
            myFysTLcd.ftCmdGetF32(PID_PARAM(Kp, e));
            myFysTLcd.ftCmdGetF32(PID_PARAM(Ki, e));
            PID_PARAM(Ki, e) = scalePID_i(PID_PARAM(Ki, e));
            myFysTLcd.ftCmdGetF32(PID_PARAM(Kd, e));
            PID_PARAM(Kd, e) = scalePID_d(PID_PARAM(Kd, e));
        #if ENABLED(PID_EXTRUSION_SCALING)
            myFysTLcd.ftCmdGetF32(PID_PARAM(Kc, e));
        #else
            myFysTLcd.ftCmdJump(4);
        #endif
        }
    }
  #endif
}
#endif

static void dwin_write_leveling_para() {
  myFysTLcd.ftCmdStart(VARADDR_PARAM_LEVELING);
  // Global Leveling
  #if ENABLED(ENABLE_LEVELING_FADE_HEIGHT)
    myFysTLcd.ftCmdPutF32(planner.z_fade_height);
  #else
    myFysTLcd.ftCmdJump(4);
  #endif

  // Planar Bed Leveling matrix
  #if ABL_PLANAR
    for (char i = 0; i < 9; i++) {
      myFysTLcd.ftCmdPutF32(planner.bed_level_matrix.matrix[i]);
    }
  #else
    myFysTLcd.ftCmdJump(36);
  #endif

  myFysTLcd.ftCmdSend();
}

static void dwin_read_leveling_para() {
  myFysTLcd.ftCmdStart(VARADDR_PARAM_LEVELING);
  if (myFysTLcd.ftCmdReceive(4)) {
    // Global Leveling
    #if ENABLED(ENABLE_LEVELING_FADE_HEIGHT)
      myFysTLcd.ftCmdGetF32(planner.z_fade_height);
    #endif
  }
  if (myFysTLcd.ftCmdReceive(36)) {
    // Planar Bed Leveling matrix
    #if ABL_PLANAR
      for (char i = 0; i < 9; i++) {
        myFysTLcd.ftCmdGetF32(planner.bed_level_matrix.matrix[i]);
      }
    #endif
  }
}

static void dwin_write_zoffset_para() {
  myFysTLcd.ftCmdStart(VARADDR_PARAM_ZOFFSET);
  #if HAS_BED_PROBE
    myFysTLcd.ftCmdPutF16_2(zprobe_zoffset);
  #endif
  myFysTLcd.ftCmdSend();
}

static void dwin_read_zoffset_para() {
  myFysTLcd.ftCmdStart(VARADDR_PARAM_ZOFFSET);
  if (myFysTLcd.ftCmdReceive(2)) {
    #if HAS_BED_PROBE
      myFysTLcd.ftCmdGetF16_2(zprobe_zoffset);
    #endif
  }
}

static void dwin_write_temperture_para() {
  myFysTLcd.ftCmdStart(VARADDR_PARAM_TEMP);
  #if ENABLED(PIDTEMP)
    myFysTLcd.ftCmdPutF32(PID_PARAM(Kp, 0));
    myFysTLcd.ftCmdPutF32(unscalePID_i(PID_PARAM(Ki, 0)));
    myFysTLcd.ftCmdPutF32(unscalePID_d(PID_PARAM(Kd, 0)));
    #if ENABLED(PID_EXTRUSION_SCALING)
    myFysTLcd.ftCmdPutF32(PID_PARAM(Kc, 0));
    #else
    myFysTLcd.ftCmdJump(4);
    #endif
  #else
    myFysTLcd.ftCmdJump(16);
  #endif //!PIDTEMP
    myFysTLcd.ftCmdSend();

  #if DISABLED(PIDTEMPBED)
    myFysTLcd.ftCmdJump(16);
  #else
    myFysTLcd.ftCmdPutF32(thermalManager.bedKp);
    myFysTLcd.ftCmdPutF32(unscalePID_i(thermalManager.bedKi));
    myFysTLcd.ftCmdPutF32(unscalePID_d(thermalManager.bedKd));
    myFysTLcd.ftCmdJump(4);
  #endif
  myFysTLcd.ftCmdSend();
}
static void dwin_read_temperture_para()
{
    myFysTLcd.ftCmdStart(VARADDR_PARAM_TEMP);
    if (myFysTLcd.ftCmdReceive(32)) {
      #if ENABLED(PIDTEMP)
        myFysTLcd.ftCmdGetF32(PID_PARAM(Kp, 0));
        myFysTLcd.ftCmdGetF32(PID_PARAM(Ki, 0));
        PID_PARAM(Ki, 0) = scalePID_i(PID_PARAM(Ki, 0));
        myFysTLcd.ftCmdGetF32(PID_PARAM(Kd, 0));
        PID_PARAM(Kd, 0) = scalePID_d(PID_PARAM(Kd, 0));
        #if ENABLED(PID_EXTRUSION_SCALING)
          myFysTLcd.ftCmdGetF32(PID_PARAM(Kc, 0));
        #else
          myFysTLcd.ftCmdJump(4);
        #endif
        thermalManager.update_pid();
      #else
        myFysTLcd.ftCmdJump(16);
      #endif //!PIDTEMP

      #if DISABLED(PIDTEMPBED)
        myFysTLcd.ftCmdJump(16);
      #else
        myFysTLcd.ftCmdGetF32(thermalManager.bedKp);
        myFysTLcd.ftCmdGetF32(thermalManager.bedKi);
        thermalManager.bedKi = scalePID_i(thermalManager.bedKi);
        myFysTLcd.ftCmdGetF32(thermalManager.bedKd);
        thermalManager.bedKd = scalePID_d(thermalManager.bedKd);
        myFysTLcd.ftCmdJump(4);
      #endif
    }
    myFysTLcd.ftCmdClear();
}

static void dwin_write_filament_para()
{
    myFysTLcd.ftCmdStart(VARADDR_PARAM_MATERIAL);
#if ENABLED(FWRETRACT)
    myFysTLcd.ftCmdPut16(autoretract_enabled);
    myFysTLcd.ftCmdPutF32(retract_length);
    #if EXTRUDERS > 1
    myFysTLcd.ftCmdPutF32(retract_length_swap);
    #else
    myFysTLcd.ftCmdJump(4);
    #endif
    myFysTLcd.ftCmdPutF32(retract_feedrate_mm_s);
    myFysTLcd.ftCmdPutF32(retract_zlift);
    myFysTLcd.ftCmdPutF32(retract_recover_length);
    #if EXTRUDERS > 1
    myFysTLcd.ftCmdPutF32(retract_recover_length_swap);
    #else
    myFysTLcd.ftCmdJump(4);
    #endif
    myFysTLcd.ftCmdPutF32(retract_recover_feedrate_mm_s);
#else
    myFysTLcd.ftCmdJump(30);
#endif // FWRETRACT
    //myFysTLcd.ftCmdPut16(volumetric_enabled); 
	myFysTLcd.ftCmdPut16(parser.volumetric_enabled);
    myFysTLcd.ftCmdSend();

    uint8_t e;
    //for (e = 0; e<EXTRUDERS; e++)myFysTLcd.ftCmdPutF32(filament_size[e]); 
    for (e = 0; e<EXTRUDERS; e++)myFysTLcd.ftCmdPutF32(planner.filament_size[e]); 
    for (; e < 5; e++)myFysTLcd.ftCmdJump(4);

// Linear Advance
#if ENABLED(LIN_ADVANCE)
    myFysTLcd.ftCmdPutF32(planner.extruder_advance_k);
    myFysTLcd.ftCmdPutF32(planner.advance_ed_ratio);
#else
    myFysTLcd.ftCmdJump(8);
#endif
    myFysTLcd.ftCmdSend();
}
static void dwin_read_filament_para()
{
    myFysTLcd.ftCmdStart(VARADDR_PARAM_MATERIAL);
    if(myFysTLcd.ftCmdReceive(32))
    {
        int16_t t;
    #if ENABLED(FWRETRACT)
        myFysTLcd.ftCmdGetI16(t);
        autoretract_enabled=t;
        myFysTLcd.ftCmdGetF32(retract_length);
        #if EXTRUDERS > 1
        myFysTLcd.ftCmdGetF32(retract_length_swap);
        #else
        myFysTLcd.ftCmdJump(4);
        #endif
        myFysTLcd.ftCmdGetF32(retract_feedrate_mm_s);
        myFysTLcd.ftCmdGetF32(retract_zlift);
        myFysTLcd.ftCmdGetF32(retract_recover_length);
        #if EXTRUDERS > 1
        myFysTLcd.ftCmdGetF32(retract_recover_length_swap);
        #else
        myFysTLcd.ftCmdJump(4);
        #endif
        myFysTLcd.ftCmdGetF32(retract_recover_feedrate_mm_s);
    #else
        myFysTLcd.ftCmdJump(30);
    #endif // FWRETRACT
        myFysTLcd.ftCmdGetI16(t);
        //volumetric_enabled = t; 
        parser.volumetric_enabled = t;
    }
    myFysTLcd.ftCmdClear();
    if(myFysTLcd.ftCmdReceive(28))
    {
        uint8_t e;
        //for (e = 0; e<EXTRUDERS; e++)myFysTLcd.ftCmdGetF32(filament_size[e]);
        for (e = 0; e<EXTRUDERS; e++)myFysTLcd.ftCmdGetF32(planner.filament_size[e]);
        for (; e < 5; e++)myFysTLcd.ftCmdJump(4);
        // Linear Advance
    #if ENABLED(LIN_ADVANCE)
        myFysTLcd.ftCmdGetF32(planner.extruder_advance_k);
        myFysTLcd.ftCmdGetF32(planner.advance_ed_ratio);
    #endif
    }
}

static void dwin_write_tmc2130_para()
{
    int16_t t;
    myFysTLcd.ftCmdStart(VARADDR_PARAM_TMC2130);
#if ENABLED(HAVE_TMC2130)
    #if ENABLED(X_IS_TMC2130)
    t = stepperX.getCurrent();
    myFysTLcd.ftCmdPut16(t);
    #else
    myFysTLcd.ftCmdJump(2);
    #endif
    #if ENABLED(Y_IS_TMC2130)
    t=stepperY.getCurrent();
    myFysTLcd.ftCmdPut16(t);
    #else
    myFysTLcd.ftCmdJump(2);
    #endif
    #if ENABLED(Z_IS_TMC2130)
    t=stepperZ.getCurrent();
    myFysTLcd.ftCmdPut16(t);
    #else
    myFysTLcd.ftCmdJump(2);
    #endif
    #if ENABLED(E0_IS_TMC2130)
    t=stepperE0.getCurrent();
    myFysTLcd.ftCmdPut16(t);
    #else
    myFysTLcd.ftCmdJump(2);
    #endif
    #if ENABLED(X2_IS_TMC2130)
    t=stepperX2.getCurrent();
    myFysTLcd.ftCmdPut16(t);
    #else
    myFysTLcd.ftCmdJump(2);
    #endif
    #if ENABLED(Y2_IS_TMC2130)
    t=stepperY2.getCurrent();
    myFysTLcd.ftCmdPut16(t);
    #else
    myFysTLcd.ftCmdJump(2);
    #endif
    #if ENABLED(Z2_IS_TMC2130)
    t=stepperZ2.getCurrent();
    myFysTLcd.ftCmdPut16(t);
    #else
    myFysTLcd.ftCmdJump(2);
    #endif
    #if ENABLED(E1_IS_TMC2130)
    t=stepperE1.getCurrent();
    myFysTLcd.ftCmdPut16(t);
    #else
    myFysTLcd.ftCmdJump(2);
    #endif
    #if ENABLED(E2_IS_TMC2130)
    t=stepperE2.getCurrent();
    myFysTLcd.ftCmdPut16(t);
    #else
    myFysTLcd.ftCmdJump(2);
    #endif
    #if ENABLED(E3_IS_TMC2130)
    t=stepperE3.getCurrent();
    myFysTLcd.ftCmdPut16(t);
    #else
    myFysTLcd.ftCmdJump(2);
    #endif
    #if ENABLED(E4_IS_TMC2130)
    t=stepperE4.getCurrent();
    myFysTLcd.ftCmdPut16(t);
    #else
    myFysTLcd.ftCmdJump(2);
    #endif
#else
    myFysTLcd.ftCmdJump(22);
#endif
    myFysTLcd.ftCmdSend();
}
static void dwin_read_tmc2130_para()
{
    int16_t t;
    myFysTLcd.ftCmdStart(VARADDR_PARAM_TMC2130);
    if (myFysTLcd.ftCmdReceive(22))
    {
	#if ENABLED(HAVE_TMC2130)
    #if ENABLED(X_IS_TMC2130)
        myFysTLcd.ftCmdGetI16(t);
        stepperX.setCurrent(t, R_SENSE, HOLD_MULTIPLIER);
    #else
        myFysTLcd.ftCmdJump(2);
    #endif
    #if ENABLED(Y_IS_TMC2130)
        myFysTLcd.ftCmdGetI16(t);
        stepperY.setCurrent(t, R_SENSE, HOLD_MULTIPLIER);
    #else
        myFysTLcd.ftCmdJump(2);
    #endif
    #if ENABLED(Z_IS_TMC2130)
        myFysTLcd.ftCmdGetI16(t);
        stepperZ.setCurrent(t, R_SENSE, HOLD_MULTIPLIER);
    #else
        myFysTLcd.ftCmdJump(2);
    #endif
    #if ENABLED(X2_IS_TMC2130)
        myFysTLcd.ftCmdGetI16(t);
        stepperX2.setCurrent(t, R_SENSE, HOLD_MULTIPLIER);
    #else
        myFysTLcd.ftCmdJump(2);
    #endif
    #if ENABLED(Y2_IS_TMC2130)
        myFysTLcd.ftCmdGetI16(t)
        stepperY2.setCurrent(t, R_SENSE, HOLD_MULTIPLIER);
    #else
        myFysTLcd.ftCmdJump(2);
    #endif
    #if ENABLED(Z2_IS_TMC2130)
        myFysTLcd.ftCmdGetI16(t);
        stepperZ2.setCurrent(t, R_SENSE, HOLD_MULTIPLIER);
    #else
        myFysTLcd.ftCmdJump(2);
    #endif
    #if ENABLED(E0_IS_TMC2130)
        myFysTLcd.ftCmdGetI16(t);
        stepperE0.setCurrent(t, R_SENSE, HOLD_MULTIPLIER);
    #else
        myFysTLcd.ftCmdJump(2);
    #endif
    #if ENABLED(E1_IS_TMC2130)
        myFysTLcd.ftCmdGetI16(t);
        stepperE1.setCurrent(t, R_SENSE, HOLD_MULTIPLIER);
    #else
        myFysTLcd.ftCmdJump(2);
    #endif
    #if ENABLED(E2_IS_TMC2130)
        myFysTLcd.ftCmdGetI16(t);
        stepperE2.setCurrent(t, R_SENSE, HOLD_MULTIPLIER);
    #else
        myFysTLcd.ftCmdJump(2);
    #endif
    #if ENABLED(E3_IS_TMC2130)
        myFysTLcd.ftCmdGetI16(t);
        stepperE3.setCurrent(t, R_SENSE, HOLD_MULTIPLIER);
    #else
        myFysTLcd.ftCmdJump(2);
    #endif
    #if ENABLED(E4_IS_TMC2130)
        myFysTLcd.ftCmdGetI16(t);
        stepperE4.setCurrent(t, R_SENSE, HOLD_MULTIPLIER);
    #else
        myFysTLcd.ftCmdJump(2);
    #endif
#else
        myFysTLcd.ftCmdJump(22);
#endif
    }
}

static void dwin_write_system_para()
{
    myFysTLcd.ftCmdStart(VARADDR_PARAM_SYSTEM);
#ifdef FYS_ENERGY_CONSERVE_HEIGHT
    myFysTLcd.ftCmdPutF16(zEnergyHeight);
#else
    myFysTLcd.ftCmdJump(2);
#endif

#if PIN_EXISTS(PS_ON)&&defined(FYS_ACTIVE_TIME_OVER)
    int16_t t=max_inactive_time/1000UL;
    myFysTLcd.ftCmdPut16(t);
#else
    myFysTLcd.ftCmdJump(2);
#endif
    myFysTLcd.ftCmdSend();
}
static void dwin_read_system_para()
{
    myFysTLcd.ftCmdStart(VARADDR_PARAM_SYSTEM);
    if (myFysTLcd.ftCmdReceive(4))
    {
      #ifdef FYS_ENERGY_CONSERVE_HEIGHT
        myFysTLcd.ftCmdGetF16(zEnergyHeight);
#else
        myFysTLcd.ftCmdJump(2);
#endif
#if PIN_EXISTS(PS_ON)&&defined(FYS_ACTIVE_TIME_OVER)
        int16_t t;
        myFysTLcd.ftCmdGetI16(t);
        max_inactive_time = (millis_t)t* 1000UL;
#else
        myFysTLcd.ftCmdJump(2);
#endif
    }
}

void dwin_popup(const char* msg, EM_POPUP_PAGE_TYPE pageChoose, char funid)
{
    char str[INFO_POPUP_LEN + 1], i, j, ch;
    if (msg)
    for (j = 0; j < INFOS_NUM; j++) {
      memset(str, 0, INFO_POPUP_LEN);
      i = 0;
      while (ch = pgm_read_byte(msg)) {
        if (ch == '\n') {
            msg++;
            break;
        }
        str[i++] = ch;
        msg++;
        if (i >= INFO_POPUP_LEN)break;
      }
      touch_lcd::ftPuts(VARADDR_POP_INFOS[j], str, INFO_POPUP_LEN);
    }
    
    switch (pageChoose) {
      case EPPT_INFO_POPUP:
        #if FYSTLCD_PAGE_EXIST(INFO_POPUP)
          lcd_pop_page(FTPAGE(INFO_POPUP));
        #endif
        break;
      case EPPT_INFO_WAITING:
        #if FYSTLCD_PAGE_EXIST(INFO_WAITING)
          lcd_pop_page(FTPAGE(INFO_WAITING));
        #endif
        break;
      case EPPT_INFO_OPTION:
        switch (funid) {
          #if ENABLED(ADVANCED_PAUSE_FEATURE)
            case 1:
                optionId = 1;
                advanced_pause_menu_response = ADVANCED_PAUSE_RESPONSE_WAIT_FOR;
                touch_lcd::ftPutsPGM(VARADDR_QUESTION_LEFT, PSTR("Resume"),8);
                touch_lcd::ftPutsPGM(VARADDR_QUESTION_RIGHT, PSTR("Extrude"),8);
                break;
          #endif
          default:
              break;
        }
        #if FYSTLCD_PAGE_EXIST(INFO_OPTION)
          lcd_pop_page(FTPAGE(INFO_OPTION));
        #endif
        break;
      default:
          break;
    }
}

void dwin_popup_shutdown() {
  if (
      #if FYSTLCD_PAGE_EXIST(INFO_POPUP)
        currentPageId == FTPAGE(INFO_POPUP) || 
      #endif
      #if FYSTLCD_PAGE_EXIST(INFO_WAITING)
        currentPageId == FTPAGE(INFO_WAITING) ||
      #endif
      #if FYSTLCD_PAGE_EXIST(INFO_OPTION)
        currentPageId == FTPAGE(INFO_OPTION) ||
      #endif
      0) {
    lcd_pop_page(retPageId);
  }
}

static void dwin_update_fan_status_ico(uint8_t ifan) {
  #if FAN_COUNT > 0

    if(ifan == 0) {    
      myFysTLcd.ftCmdStart(VARADDR_STATUS_FAN);
      if (fanSpeeds[0]>0) {
        myFysTLcd.ftCmdPut16(1);
      }
      else {
        myFysTLcd.ftCmdPut16(0);
      }
      myFysTLcd.ftCmdSend();
    }
    #if FAN_COUNT > 1

    else if(ifan == 1) {
      myFysTLcd.ftCmdStart(VARADDR_STATUS_FAN1);
      if (fanSpeeds[1]>0) {
        myFysTLcd.ftCmdPut16(1);
      }
      else {
        myFysTLcd.ftCmdPut16(0);
      }
      myFysTLcd.ftCmdSend();
    }

    #endif

  #endif
}

static void dwin_update_e0_status() {
  myFysTLcd.ftCmdStart(VARADDR_STATUS_E0);
  if(thermalManager.degTargetHotend(0) > 0) {
    myFysTLcd.ftCmdPut16(1);
  }
  else {
    myFysTLcd.ftCmdPut16(0);
  }
  myFysTLcd.ftCmdSend();
}

#if EXTRUDERS > 1
  static void dwin_update_e1_status() {
    myFysTLcd.ftCmdStart(VARADDR_STATUS_E1);
    if(thermalManager.degTargetHotend(1) > 0) {
      myFysTLcd.ftCmdPut16(1);
    }
    else {
      myFysTLcd.ftCmdPut16(0);
    }
    myFysTLcd.ftCmdSend();
  }
#endif

static void dwin_update_bed_status() {
  myFysTLcd.ftCmdStart(VARADDR_STATUS_BED);
  if(thermalManager.degTargetBed() > 0) {
    myFysTLcd.ftCmdPut16(1);
  }
  else {
    myFysTLcd.ftCmdPut16(0);
  }
  myFysTLcd.ftCmdSend();
}

void lcd_init() {
  #if defined (SDSUPPORT) && PIN_EXISTS(SD_DETECT)
    pinMode(SD_DETECT_PIN, INPUT);
    WRITE(SD_DETECT_PIN, HIGH);
  #endif
  
  touch_lcd::ftBegin();
  
  lcd_boot_music();

  #ifdef BOOT_ANIMATION
    lcd_set_page_force(BOOT_SCREEN_BEGIN_PAGE);
  #endif
    
  #if FYSTLCD_PAGE_EXIST(MAIN)
    retPageId = FTPAGE(MAIN);
    currentPageId = FTPAGE(MAIN);
  #endif

  #if ENABLED(SDSUPPORT) && PIN_EXISTS(SD_DETECT)
    SET_INPUT_PULLUP(SD_DETECT_PIN);
    lcd_sd_status = 2; // Unknown
    dwinFileWindowTopIndex = 0;
  #endif

  lcd_init_datas();// so NO zero data in screen
}

static void lcd_init_datas() {
  move_dis = 0.1;
  move_feedrate = 30.0;

  int16_t tval = move_dis*FYSTLCD_DOT_TEN_MUL;
  myFysTLcd.ftCmdStart(VARADDR_MOVE_DIS_SIGN);
  myFysTLcd.ftCmdPut16(tval);
  myFysTLcd.ftCmdSend();

  tval = move_feedrate*FYSTLCD_DOT_TEN_MUL;
  myFysTLcd.ftCmdStart(VARADDR_MOVE_SPEED_SIGN);
  myFysTLcd.ftCmdPut16(tval);
  myFysTLcd.ftCmdSend();

  myLcdEvt |= ((uint16_t)0x0001 << LCDEVT_DETAIL_EXTRUDER);
  
  touch_lcd::ftPuts(VARADDR_VERSION_DATE, SHORT_BUILD_VERSION, ATTACH_STR_LEN);
  //touch_lcd::ftPuts(VARADDR_SERIAL_NUMBER, SERIAL_NUMBER, ATTACH_STR_LEN);

  #if ENABLED(PRINTCOUNTER)
    char buffer[21];
    printStatistics state = print_job_timer.getStats();
    sprintf_P(buffer, PSTR("%u"), state.totalPrints);
    touch_lcd::ftPuts(VARADDR_INFO_PRINT, buffer, ATTACH_STR_LEN);

    duration_t elapsed = state.printTime;
    elapsed.toString(buffer);
    touch_lcd::ftPuts(VARADDR_INFO_PRINT_ACC_TIME, buffer, ATTACH_STR_LEN);
  #endif
  
  millis_t pt=0;
  myFysTLcd.ftCmdStart(VARADDR_PRINT_TIME);
  myFysTLcd.ftPutTime(pt);
  myFysTLcd.ftCmdSend();

  dwin_update_fan_status_ico(0);
  dwin_update_fan_status_ico(1);
  dwin_update_e0_status();
  #if EXTRUDERS > 1
    dwin_update_e1_status();
  #endif
  dwin_update_bed_status();
}

#ifdef BOOT_ANIMATION

  static void lcd_boot_screen(millis_t& tNow) {
    static bool done = false;

    if(done) return;

    if(myLcdEvt & ((uint16_t)0x0001 << LCDEVT_IF_CONTINE_PRINT)) {
      done = true;
      return;
    }

    if(lcd_get_page() == BOOT_SCREEN_END_PAGE) {
      done = true;
      #if FYSTLCD_PAGE_EXIST(MAIN)
        lcd_set_page_force(FTPAGE(MAIN));
      #endif
    }
  }

#else  

  static void lcd_boot_screen(millis_t& tNow)  {
    static millis_t period = 30; // 8ms
    static uint16_t pic_num = 0;

    if(myLcdEvt & ((uint16_t)0x0001 << LCDEVT_IF_CONTINE_PRINT)) {
      pic_num = BOOT_SCREEN_ICONS+2;
      return;
    }

    if(pic_num > BOOT_SCREEN_ICONS+1)  {
      return;
    }
    
    if (tNow > period) {
      if(pic_num > BOOT_SCREEN_ICONS)  {
        #if FYSTLCD_PAGE_EXIST(MAIN)
          lcd_set_page_force(FTPAGE(MAIN));
        #endif  
        pic_num++;
        return;
      }
      
      myFysTLcd.ftCmdStart(VARADDR_BOOT_SCREEN);
      myFysTLcd.ftCmdPut16(pic_num);
      myFysTLcd.ftCmdSend();
      period = tNow + 30;
      pic_num++;
    }
  }

#endif

static void lcd_event() {
  if (myLcdEvt == 0x0000) return;
  uint8_t n;
  char sdFileName[FYSTLCD_FILENAME_LEN],*t;
  for (n = 0; n < 16; n++){
    if (myLcdEvt&(1 << n)){
      myLcdEvt &= ~(uint16_t)(1 << n);
      break;
    }
  }
  switch (n) {
    case LCDEVT_IF_CONTINE_PRINT:
      #if FYSTLCD_PAGE_EXIST(POWER_LOSS_RECOVERY)
        lcd_set_page(FTPAGE(POWER_LOSS_RECOVERY));
      #endif    
      break;
    
    case LCDEVT_READY_CONTINE_PRINT:
      #if ENABLED(SDSUPPORT)
        if (card.longFilename[0]) strncpy(sdFileName, card.longFilename, FYSTLCD_FILENAME_LEN);
        else strncpy(sdFileName, card.filename, FYSTLCD_FILENAME_LEN);
        //t = strchr(sdFileName, '.');
        //while (*t)*t++ = '\0';
        touch_lcd::ftPuts(VARADDR_PRINTFILE_NAME, sdFileName, FYSTLCD_FILENAME_LEN);
      #endif
      break;

    case LCDEVT_AUTOPID_SUCCESS:
      ftState &= ~FTSTATE_AUTOPID_ING;
      ftState |= FTSTATE_EXECUTE_PERIOD_TASK_NOW;
      dwin_write_extruders_para();
      lcd_setstatusPGM(PSTR("PID autotune finished ."), 1);
      #if FYSTLCD_PAGE_EXIST(PID_QUESTION)
        lcd_set_page(FTPAGE(PID_QUESTION));
      #endif
      break;
    
    case LCDEVT_AUTOPID_FAIL:
      ftState &= ~FTSTATE_AUTOPID_ING;
      ftState |= FTSTATE_EXECUTE_PERIOD_TASK_NOW;
      dwin_write_extruders_para();
      lcd_setstatusPGM(PSTR("PID autotune fail ."), 1);
      #if FYSTLCD_PAGE_EXIST(PID_QUESTION)
        lcd_set_page(FTPAGE(PID_QUESTION));
      #endif
      break;
      
    case LCDEVT_DETAIL_EXTRUDER:
      ftState |= FTSTATE_EXECUTE_PERIOD_TASK_NOW;
      dwin_write_extruders_para();
      break;
    
    case LCDEVT_LEVELING_COMPLETE:          
      #if FYSTLCD_PAGE_EXIST(AUTO_LEVELING_COMPLETE)&&!FYSTLCD_PAGE_EXIST(AUTO_LEVELING)
      	dwin_write_leveling_para();
        if( !IS_SD_PRINTING && !print_job_timer.isRunning() ) 
        { 
           lcd_set_page(FTPAGE(MAIN));
        }
      #endif
			
      disable_X(); 
      disable_Y();  
      break;
    
    case LCDEVT_PRINTING_COMPLETE:
      #if FYSTLCD_PAGE_EXIST(ACCOMPLISH_PRINT)
        lcd_set_page(FTPAGE(ACCOMPLISH_PRINT));
      #elif FYSTLCD_PAGE_EXIST(MAIN)
        lcd_set_page(FTPAGE(MAIN));
      #endif   
      break;
  }
}

static void lcd_check() {   

  #if defined(SDSUPPORT) && PIN_EXISTS(SD_DETECT)    

    const uint8_t sd_status = IS_SD_INSERTED();
    if (sd_status != lcd_sd_status) {

      uint8_t old_sd_status = lcd_sd_status; // prevent re-entry to this block!
      lcd_sd_status = sd_status;

      if (sd_status) {
        safe_delay(500); // Some boards need a delay to get settled
        card.initsd();
        if (old_sd_status == 2) {
          card.beginautostart();  // Initial boot
        }
        else {
          LCD_MESSAGEPGM(MSG_SD_INSERTED);
          if(card.cardOK) {
            myFysTLcd.ftCmdStart(VARADDR_STATUS_SD);
            myFysTLcd.ftCmdPut16(1);
            myFysTLcd.ftCmdSend();
          }
        }
      }
      else {
        card.release();
        if (old_sd_status != 2) {
          LCD_MESSAGEPGM(MSG_SD_REMOVED);
          myFysTLcd.ftCmdStart(VARADDR_STATUS_SD);
          myFysTLcd.ftCmdPut16(0);
          myFysTLcd.ftCmdSend();
          //if (!on_status_screen()) return_to_status();
          bool changePage = false
            #if FYSTLCD_PAGE_EXIST(FILELIST)
              ||currentPageId == PAGENUM_FILELIST
            #endif
            #if FYSTLCD_PAGE_EXIST(ACCOMPLISH_PRINT)
              ||currentPageId==PAGENUM_ACCOMPLISH_PRINT
            #endif
              ;
          if (changePage) {
            #if FYSTLCD_PAGE_EXIST(MAIN)
              lcd_set_page(FTPAGE(MAIN));
            #endif
          }
        }
      }

      //refresh();
      dwinFileWindowTopIndex = 0;
    }
  #endif

  #if ENABLED(POWER_LOSS_RECOVERY)
    if (job_recovery_commands_count && job_recovery_phase == JOB_RECOVERY_IDLE) {
      //lcd_goto_screen(lcd_job_recovery_menu);
      SERIAL_ECHOLN("lcd_job_recovery");
      job_recovery_phase = JOB_RECOVERY_MAYBE; // Waiting for a response
      lcd_set_event(LCDEVT_IF_CONTINE_PRINT);
    }
  #endif

  #if FAN_COUNT > 0
    if (oldFanSpeed != fanSpeeds[active_extruder]) {
      oldFanSpeed=fanSpeeds[active_extruder];
      myFysTLcd.ftCmdStart(VARADDR_TUNE_FAN_SPEED);
      myFysTLcd.ftCmdPut16(oldFanSpeed);
      myFysTLcd.ftCmdSend();
    }
  #endif
  
  if (oldFeedratePercentage != feedrate_percentage) {
    oldFeedratePercentage = feedrate_percentage;
    myFysTLcd.ftCmdStart(VARADDR_TUNE_PRINT_PERCENTAGE);
    myFysTLcd.ftCmdPut16(oldFeedratePercentage);
    myFysTLcd.ftCmdSend();
  }
}

#if ENABLED(FIRST_LAYER_CAL)

static void lcd_task_first_layer_cal() {
  if(ftState&FTSTATE_FIRST_LAYER_CAL) {

		char cmd1[30];
		static uint8_t filament = 0;

    if (!planner.has_blocks_queued() && commands_in_queue==0) { //  && !saved_printing
      switch(first_layer_cal_step) {
      case 0:
        first_layer_cal_step = 11;
        break;
      case 20:
        filament = 0;
        first_layer_cal_step = 11;
        break;
      case 21:
        filament = 1;
        first_layer_cal_step = 11;
        break;
      case 22:
        filament = 2;
        first_layer_cal_step = 11;
        break;
      case 23:
        filament = 3;
        first_layer_cal_step = 11;
        break;
      case 24:
        filament = 4;
        first_layer_cal_step = 11;
        break;
      case 11:
        if(lay1cal_preheat_f(filament))
          first_layer_cal_step = 10;
        break;
      case 10:
        lay1cal_load_filament(cmd1, filament);
        first_layer_cal_step = 9;
        break;
      case 9:
        // popup zoffset menu
        #if FYSTLCD_PAGE_EXIST(FIRST_LAYER_PRINT)
          lcd_set_page(FTPAGE(FIRST_LAYER_PRINT));
        #endif
        lay1cal_intro_line();
        first_layer_cal_step = 8;
        break;
      case 8:
        if(lay1cal_before_meander())
          first_layer_cal_step = 7;
        break;
      case 7:
        if(lay1cal_meander(cmd1))
          first_layer_cal_step = 6;
        break;
      case 6:
        if(lay1cal_square_loop(cmd1,0,16))
          first_layer_cal_step = 2;
        break;
      case 2:
        if(lay1cal_end()) {
          first_layer_cal_step = 1;
        }
        break;
      case 1:
        lcd_setstatusPGM(PSTR(WELCOME_MSG));
        first_layer_cal_step = 0;
        ftState &= ~FTSTATE_FIRST_LAYER_CAL;
        #if FYSTLCD_PAGE_EXIST(UTILITY)
          lcd_set_page(FTPAGE(UTILITY));
        #endif
        break;
      }
	  }
	}
}

#endif

#if ENABLED(CENTER_ADJUST_ZOFFSET)

static int8_t enqueue_center_adjust_commands_P(const char * const pgcode) {
  if(commands_in_queue < BUFSIZE) {
    enqueue_and_echo_commands_P(pgcode);
    return 0;
  }
  else {
    return -1;
  }
}

static bool center_adjust_leveling() {

  static uint8_t i = 0;

  static const char cmd_0[] PROGMEM = "G28";
  static const char cmd_1[] PROGMEM = "G29";
  static const char cmd_2[] PROGMEM = "G1 X150 Y150";
  static const char cmd_3[] PROGMEM = "G1 Z0";
  static const char cmd_4[] PROGMEM = "M84 XY";

  static const char * const cmd[] PROGMEM = {
    cmd_0, cmd_1, cmd_2, cmd_3, cmd_4,
  };

  for (; i < COUNT(cmd); ++i) {
    if(0 > enqueue_center_adjust_commands_P(static_cast<char*>(pgm_read_ptr(&cmd[i])))) {
      return false;
    }
  }

  i = 0;
  return true;
}

static void lcd_task_center_adjust_zoffset() {

  if(ftState&FTSTATE_CENTER_ADJUST_ZOFFSET) {

    if (!planner.has_blocks_queued() && commands_in_queue==0) {
      switch(center_adjust_step) {
      case 0:
        if(center_adjust_leveling())
          center_adjust_step = 1;
        break;
      case 1:
        #if FYSTLCD_PAGE_EXIST(MANUAL_LEVELING)
          lcd_set_page_force(FTPAGE(MANUAL_LEVELING));
        #endif
          center_adjust_step = 2;
        break;
      case 2:
        break;
      case 3:
        enqueue_and_echo_commands_P(PSTR("G1 Z10"));
        enqueue_and_echo_commands_P(PSTR("G28 XY"));
        enqueue_and_echo_commands_P(PSTR("M500"));
        center_adjust_step = 5;
        break;
      case 4:
        enqueue_and_echo_commands_P(PSTR("G1 Z10"));
        enqueue_and_echo_commands_P(PSTR("G28 XY"));
        center_adjust_step = 5;
        break;
      case 5:
        #if FYSTLCD_PAGE_EXIST(MAIN)
          lcd_set_page_force(FTPAGE(MAIN));
        #endif
        center_adjust_step = 0;
        ftState &= ~FTSTATE_CENTER_ADJUST_ZOFFSET;
        break;
      }
	  }
	}
}

#endif


#if ENABLED(TMC_Z_CALIBRATION)
void lcd_flag_calibrate_z_done() {
  if(lcd_calibrating_z) {
    lcd_calibrating_z = false;
    ftState|=FTSTATE_NEED_SAVE_PARAM;
    #if FYSTLCD_PAGE_EXIST(UTILITY)
      lcd_set_page_force(FTPAGE(UTILITY));
    #endif
  }
}
#endif

static void lcd_task() {
  if ((ftState&FTSTATE_NEED_SAVE_PARAM) \
      && (commands_in_queue < BUFSIZE)) {
    enqueue_and_echo_commands_P(PSTR("M500"));
    ftState &= ~FTSTATE_NEED_SAVE_PARAM;
  }

  if ((ftState&FTSTATE_NEED_LOAD_PARAM) \
      && (commands_in_queue < BUFSIZE)) {
    enqueue_and_echo_commands_P(PSTR("M501"));
    ftState &= ~FTSTATE_NEED_LOAD_PARAM;
  }

  #if ENABLED(FIRST_LAYER_CAL)
    lcd_task_first_layer_cal();
  #endif

  #if ENABLED(CENTER_ADJUST_ZOFFSET)
    lcd_task_center_adjust_zoffset();
  #endif
}

static void lcd_period_task(millis_t& tNow)  {
  static millis_t period = 1000;
   
  if ( tNow > period || \
       ftState&FTSTATE_EXECUTE_PERIOD_TASK_NOW) {

    if (periodFun) periodFun();
    
    millis_t distance = 0;
    
	  #if defined(FYS_ACTIVE_TIME_OVER)
      if (tNow<previous_move_ms) {
        previous_move_ms = tNow;
        distance = max_inactive_time;
      }
      else {
        distance = tNow - previous_move_ms;
        if (distance > max_inactive_time)distance = 0;
        else distance = max_inactive_time - distance;
      }
      
      if (distance<21000) {
        if (!(ftState&FTSTATE_ACTIVE_WARNING)) {
          ftState |= FTSTATE_ACTIVE_WARNING;
          dwin_popup_shutdown();
        }
      }
      else if (ftState&FTSTATE_ACTIVE_WARNING) {
        ftState &= ~FTSTATE_ACTIVE_WARNING;
        dwin_popup_shutdown();
      }
	  #endif
	
    lcd_period_report(distance / 1000);
    
    period = tNow + 1000;
    ftState &= ~FTSTATE_EXECUTE_PERIOD_TASK_NOW;
  }

  // status message
  static millis_t status_period = 1300;
  if ( tNow > status_period || \
       ftState&FTSTATE_STATUS_MESSAGE_NOW ) {
      dwin_set_status(lcd_status_message);
  }       

  #if defined(VARADDR_PROMPT_DATA)&&VARADDR_PROMPT_DATA>0
    static millis_t prompt = 200;
    if (tNow > prompt) {
      lcd_period_prompt_report();
      prompt = tNow + 200;
    }
  #endif
}

static void lcd_load_settings() {
  if (commands_in_queue < BUFSIZE) {
    enqueue_and_echo_commands_P(PSTR("M501"));
    ftState &= ~FTSTATE_NEED_LOAD_PARAM;
  }
  else
    ftState |= FTSTATE_NEED_LOAD_PARAM;
}

static void lcd_save_settings() {
  if (commands_in_queue < BUFSIZE) {
    enqueue_and_echo_commands_P(PSTR("M500"));
    ftState &= ~FTSTATE_NEED_SAVE_PARAM;
  }
  else
    ftState |= FTSTATE_NEED_SAVE_PARAM;
}

void lcd_update() {
  millis_t t = millis();
  lcd_event();
  lcd_check();
  dwin_on_cmd(t);
  lcd_task();
  lcd_period_task(t);
  lcd_boot_screen(t);
}

static inline void lcd_pid_autotune() {
  char str[30];
  dwin_read_extruders_para();
  sprintf(str, "M303 E%d C5 S210 U1", active_extruder);
  enqueue_and_echo_command(str);
  ftState |= FTSTATE_AUTOPID_ING;
}

static inline void lcd_pid_autotune_bed() {
  char str[30];
  dwin_read_extruders_para();
  sprintf(str, "M303 E-1 C5 S70 U1");
  enqueue_and_echo_command(str);
  ftState |= FTSTATE_AUTOPID_ING;
}

#if ENABLED(BABYSTEPPING)

  void _lcd_babystep(const AxisEnum axis, const char* msg) {
    UNUSED(msg);
    const int16_t babystep_increment = (int32_t)10 * (BABYSTEP_MULTIPLICATOR);
    thermalManager.babystep_axis(axis, babystep_increment);
    babysteps_done += babystep_increment;
  }

  #if ENABLED(BABYSTEP_XY)
    void _lcd_babystep_x() { _lcd_babystep(X_AXIS, PSTR(MSG_BABYSTEP_X)); }
    void _lcd_babystep_y() { _lcd_babystep(Y_AXIS, PSTR(MSG_BABYSTEP_Y)); }
    void lcd_babystep_x() { _lcd_babystep_x();} // need to clear babysteps_done before calling
    void lcd_babystep_y() { _lcd_babystep_y();} // need to clear babysteps_done before calling
  #endif

  #if ENABLED(BABYSTEP_ZPROBE_OFFSET)

    void lcd_babystep_zoffset() {
      const int16_t babystep_increment = lcd_babysteps * (BABYSTEP_MULTIPLICATOR);
      const float new_zoffset = zprobe_zoffset + planner.steps_to_mm[Z_AXIS] * babystep_increment;
      SERIAL_ECHOLNPAIR("babystep_increment:",babystep_increment);
      SERIAL_ECHOLNPAIR("new_zoffset:",new_zoffset);
      if (WITHIN(new_zoffset, Z_PROBE_OFFSET_RANGE_MIN, Z_PROBE_OFFSET_RANGE_MAX)) {
          thermalManager.babystep_axis(Z_AXIS, babystep_increment);
          SERIAL_ECHOLNPAIR("btd:",thermalManager.babystepsTodo[Z_AXIS]);
          zprobe_zoffset = new_zoffset;
      }
    }

  #else // !BABYSTEP_ZPROBE_OFFSET

    void _lcd_babystep_z() { _lcd_babystep(Z_AXIS, PSTR(MSG_BABYSTEP_Z)); }
    void lcd_babystep_z() { _lcd_babystep_z();} // need to clear babysteps_done before calling

  #endif // !BABYSTEP_ZPROBE_OFFSET

#endif // BABYSTEPPING

static void move_axis(AxisEnum axis, float val) {
		if(planner.movesplanned()!=0) return;

    if(axis==E_AXIS) {           
      if(thermalManager.degHotend(active_extruder) > 180) {
        // Get the new position
        current_position[axis] += val;

        move_feedrate = MOVE_E_FEEDRATE;
      }
      else {
        return;
      }
    }
    else {
      float min = current_position[axis] - 1000,
            max = current_position[axis] + 1000;

      #if HAS_SOFTWARE_ENDSTOPS
        // Limit to software endstops, if enabled
        if (soft_endstops_enabled) {
          #if ENABLED(MIN_SOFTWARE_ENDSTOPS)
            min = soft_endstop_min[axis];
          #endif
          #if ENABLED(MAX_SOFTWARE_ENDSTOPS)
            max = soft_endstop_max[axis];
          #endif
        }
      #endif
  

      // Get the new position
      current_position[axis] += val;

      // Limit only when trying to move towards the limit
      NOLESS(current_position[axis], min);
      NOMORE(current_position[axis], max);
      
      move_feedrate = MOVE_XYZ_FEEDRATE;
    }

    planner.buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], move_feedrate>0.0 ? move_feedrate : pgm_read_float(&homing_feedrate_mm_s[axis]), active_extruder);
}

static void manual_leveling_move(float x, float y) {
  current_position[Z_AXIS] = Z_CLEARANCE_DEPLOY_PROBE;
  planner.buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], MMM_TO_MMS(HOMING_FEEDRATE_Z), active_extruder);
  current_position[X_AXIS] = x;
  current_position[Y_AXIS] = y;
  planner.buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], MMM_TO_MMS(HOMING_FEEDRATE_XY), active_extruder);
  current_position[Z_AXIS] = 0;
  planner.buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], MMM_TO_MMS(HOMING_FEEDRATE_Z), active_extruder);
}

static int16_t filament_temp_preheat(bool ifHeatBed = false) {
    int16_t tempe, tempb;
    uint8_t extruder_index = 0;
    if (filament_choice < FILAMENTS){
      extruder_index = 0;
      tempe = lcd_preheat_hotend_temp[filament_choice];
      //SERIAL_ECHOLNPAIR("tempb1:", tempb);
    }
    #if EXTRUDERS>1
      else if(filament_choice < FILAMENTS*2){
        extruder_index = 1;
        tempe = lcd_preheat_hotend_temp[filament_choice-FILAMENTS];
        tempb = lcd_preheat_bed_temp[filament_choice-FILAMENTS];
      }
    #endif
    #if HAS_TEMP_BED
      else if(filament_choice < FILAMENTS*3){
        extruder_index = 0;
        tempb = lcd_preheat_bed_temp[filament_choice-FILAMENTS*2];
      }
    #endif
    else{
      extruder_index = 0;
      tempe = 260;
      tempb = 100;
      //SERIAL_ECHOLNPAIR("tempb2:", tempb);
    }
    
    if (ifHeatBed) { 
      #if HAS_TEMP_BED
        thermalManager.setTargetBed(tempb);
      #endif
    }
    else { 
      thermalManager.setTargetHotend(tempe, extruder_index);
    }
    
    #if FAN_COUNT > 0
    if(extruder_index<FAN_COUNT) {
      #if EXTRUDERS>1
        fanSpeeds[extruder_index] = lcd_preheat_fan_speed[filament_choice-FILAMENTS];
      #else
        fanSpeeds[extruder_index] = lcd_preheat_fan_speed[filament_choice];
      #endif
    }
    #endif

    myFysTLcd.ftCmdStart(VARADDR_ACTIVE_EXTRUDER_HOTEND);
    myFysTLcd.ftCmdPut16(tempe);    
    myFysTLcd.ftCmdSend();

    #if HAS_TEMP_BED
      if (ifHeatBed) {
        myFysTLcd.ftCmdStart(VARADDR_ACTIVE_EXTRUDER_BED);
        myFysTLcd.ftCmdPut16(tempb);    
        myFysTLcd.ftCmdSend();
      }
    #endif

    #if EXTRUDERS>1
      myFysTLcd.ftCmdStart(VARADDR_ACTIVE_EXTRUDERS_HOTEND);
      myFysTLcd.ftCmdPut16(tempe);      
      myFysTLcd.ftCmdSend();
    #endif

    return tempe;
}

static void filament_load() {
  // wait for homing cmd finished
  if(commands_in_queue > 0) return;

  int16_t tempe = 0;
  uint8_t extru_index = 0;
  
  if (filament_choice < FILAMENTS){
    extru_index = 0;
    tempe = lcd_preheat_hotend_temp[filament_choice];
  }
  #if EXTRUDERS>1
    else if(filament_choice < FILAMENTS*2){
      extru_index = 1;
      tempe = lcd_preheat_hotend_temp[filament_choice-FILAMENTS];
    }
  #endif

  if (planner.movesplanned()<2 && \
      ABS(thermalManager.degHotend(extru_index)-tempe)<10){

    #if FYSTLCD_PAGE_EXIST(FILAMENT_LOADING)
      lcd_set_page_force(FTPAGE(FILAMENT_LOADING));
    #endif
    current_position[E_AXIS] += MOVE_E_LENGTH_EACH_TIME;
    planner.buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], 
                        current_position[E_AXIS], MOVE_E_FEEDRATE, extru_index);
  }
}

static void filament_unload() {
  // wait for homing cmd finished
  if(commands_in_queue > 0) return;
  
  int16_t tempe = 0;
  uint8_t extru_index = 0;

  if (filament_choice < FILAMENTS){
    extru_index = 0;
    tempe = lcd_preheat_hotend_temp[filament_choice];
  }
  #if EXTRUDERS>1
    else if(filament_choice < FILAMENTS*2){
      extru_index = 1;
      tempe = lcd_preheat_hotend_temp[filament_choice-FILAMENTS];
    }
  #endif

  if (planner.movesplanned()<2 && \
      ABS(thermalManager.degHotend(extru_index)-tempe)<10) {

    #if FYSTLCD_PAGE_EXIST(FILAMENT_UNLOADING)
      lcd_set_page_force(FTPAGE(FILAMENT_UNLOADING));
    #endif

    if(unload_purge_times < 2) {
      current_position[E_AXIS] += MOVE_E_LENGTH_EACH_TIME;
      unload_purge_times++;
    }
    else {
      current_position[E_AXIS] -= MOVE_E_LENGTH_EACH_TIME;
      unload_purge_times = 0;
    }
    planner.buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], \
                        current_position[E_AXIS], MOVE_E_FEEDRATE, extru_index);
  }
}

static void dwin_set_status(const char* msg) {
  touch_lcd::ftPuts(VARADDR_STATUS, msg, VARADDR_STATUS_MSG_LEN);
}

static void dwin_clear_pop_infos() {
  for(int i = 0;i < INFOS_NUM;i++) {
    touch_lcd::ftPuts(VARADDR_POP_INFOS[i], " ", INFO_POPUP_LEN);
  }  
}

// valid ：flag to update list
static void dwin_update_file_list(bool valid) {  
  #if ENABLED(SDSUPPORT)

    uint8_t fileWindowStartIndex=0;
    if(card.isIndir()) {
      touch_lcd::ftPuts(VARADDR_FILES_NAME[0], "..", FYSTLCD_FILENAME_LEN);
      fileWindowStartIndex = 1;
    }
  
    char sdFileName[FYSTLCD_FILENAME_LEN + 1] = { 0 };
    for (uint8_t i = fileWindowStartIndex; i<FILE_WINDOW_SIZE; i++) {
        if (sdFileName[0] || sdFileName[1]) {
          memset(sdFileName, 0, FYSTLCD_FILENAME_LEN+1);            
        }
        
        if (valid) {
          if ((i-fileWindowStartIndex) < dwinFileWindowTopIndex) {
            card.getfilename(dwinFileWindowTopIndex - 1 - (i-fileWindowStartIndex));
            if(card.longFilename[0]!=0) {
              strncpy(sdFileName, card.longFilename, FYSTLCD_FILENAME_LEN);
            }
            else{
              strncpy(sdFileName, card.filename, FYSTLCD_FILENAME_LEN);
            }

            //SERIAL_PROTOCOLLN(card.longFilename);
            //SERIAL_PROTOCOLLN(sdFileName);
          }
        }
        
        touch_lcd::ftPuts(VARADDR_FILES_NAME[i], sdFileName, FYSTLCD_FILENAME_LEN);
    }
  #endif
}

static void dwin_file_select(const char *name,const char *longname) {    
  #if ENABLED(SDSUPPORT)
    if (card.sdprinting) return; 
  
    char sdFileName[FYSTLCD_FILENAME_LEN + 1] = { 0 };    
    //card.getfilename(sd_num - 1);                  
    if(longname[0]!=0) {
      strncpy(sdFileName, longname, FYSTLCD_FILENAME_LEN);
    }
    else {
      strncpy(sdFileName, name, FYSTLCD_FILENAME_LEN);
    }    
    
    if (card.isFileOpen()) card.closefile();
    card.openFile(name, true);
    #if !defined(FILE_PRINT_NEED_CONRIRM)
      card.startFileprint();
      print_job_timer.start();
      lcd_icon_state = 0;
    #endif
    
    //char*t = strchr(sdFileName, '.');
    //while (*t)*t++ = '\0';
    touch_lcd::ftPuts(VARADDR_PRINTFILE_NAME, sdFileName, FYSTLCD_FILENAME_LEN);
  #endif
}

void lcd_showFilename() {
  #if ENABLED(SDSUPPORT)
    char sdFileName[FYSTLCD_FILENAME_LEN + 1] = { 0 };    
                  
    if(card.longFilename[0]!=0) {
      strncpy(sdFileName, card.longFilename, FYSTLCD_FILENAME_LEN);
    }
    else {
      strncpy(sdFileName, card.filename, FYSTLCD_FILENAME_LEN);
    }
    
    char*t = strchr(sdFileName, '.');
    while (*t)*t++ = '\0';
    touch_lcd::ftPuts(VARADDR_PRINTFILE_NAME, sdFileName, FYSTLCD_FILENAME_LEN);
  #endif
}

static void report_commands(uint16_t& tar) {
  for (uint8_t i = 0; i<4; i++) {
    char c = (tar >> ((3 - i) << 2)) & 0x0F;
    if (c > 9)MYSERIAL0.write(c - 10 + 'A');
    else MYSERIAL0.write(c + '0');
  }
}

static void dwin_on_cmd_tool(uint16_t tval) {
  switch (tval) {
    case VARADDR_TOOL_TUNEPID_ENTER:
      dwin_write_extruders_para();
      ftState |= FTSTATE_EXECUTE_PERIOD_TASK_NOW;
      #if FYSTLCD_PAGE_EXIST(TUNE_PID)
        lcd_set_page(FTPAGE(TUNE_PID));
      #endif
      break;
    case VARADDR_TOOL_TUNEPID_APPLY:
      dwin_read_extruders_para();
      break;
    case VARADDR_TOOL_TUNEPID_SAVE:
      dwin_read_extruders_para();
      lcd_save_settings();
      break;
    case VARVAL_TOOL_HOME_ALL:
      enqueue_and_echo_commands_P(PSTR("G28"));
      break;
    case VARVAL_TOOL_HOME_X:
      enqueue_and_echo_commands_P(PSTR("G28 X0"));
      break;
    case VARVAL_TOOL_HOME_Y:
      enqueue_and_echo_commands_P(PSTR("G28 Y0"));
      break;
    case VARVAL_TOOL_HOME_Z:
      enqueue_and_echo_commands_P(PSTR("G28 Z0"));
      break;
    case VARVAL_TOOL_HOME_XY:
      enqueue_and_echo_commands_P(PSTR("G28 X0 Y0"));
      break;
    case VARVAL_TOOL_SHUTDOWN:
  		#if ENABLED(FYS_POWER_STATUS)&&PIN_EXISTS(PS_ON)
        powerStatus = POWER_DOWN_DOING;
  		#endif
      break;
      
    case VARVAL_TOOL_COOLDOWN:
      for (uint8_t e = 0; e<EXTRUDERS; e++) {
        thermalManager.setTargetHotend(0, e);
      }
      #if HAS_TEMP_BED
        thermalManager.setTargetBed(0);
      #endif
      dwin_update_e0_status();
      #if EXTRUDERS > 1
        dwin_update_e1_status();
      #endif
      dwin_update_bed_status();
      break;

    case VARVAL_TOOL_PREHEAT_SWITHCH_E1:
      if(thermalManager.degTargetHotend(0) > 0) {
        thermalManager.setTargetHotend(0, 0);
      }
      else {
        thermalManager.setTargetHotend(PREHEAT_1_TEMP_HOTEND, 0);
      }
      dwin_update_e0_status();
      break;

    #if EXTRUDERS > 1
    case VARVAL_TOOL_PREHEAT_SWITHCH_E2:
      if(thermalManager.degTargetHotend(1) > 0) {
        thermalManager.setTargetHotend(0, 1);
      }
      else {
        thermalManager.setTargetHotend(PREHEAT_1_TEMP_HOTEND, 1);
      }
      dwin_update_e1_status();
      break;
    #endif

    case VARVAL_TOOL_PREHEAT_SWITHCH_BED:
      if(thermalManager.degTargetBed() > 0) {
        thermalManager.setTargetBed(0);
      }
      else {
        thermalManager.setTargetBed(PREHEAT_1_TEMP_BED);
      }
      dwin_update_bed_status();
      break;
      
    case VARVAL_TOOL_PREHEAT_E1: 
      #if FYSTLCD_PAGE_EXIST(TEMP_PREHEAT_E1)
        lcd_set_page(FTPAGE(TEMP_PREHEAT_E1));
      #endif      
      break;
    case VARVAL_TOOL_PREHEAT_E2:
      #if EXTRUDERS>1
        #if FYSTLCD_PAGE_EXIST(TEMP_PREHEAT_E2)
          lcd_set_page(FTPAGE(TEMP_PREHEAT_E2));
        #endif 
      #endif
      break;
    
    case VARVAL_TOOL_PREHEAT_BED:
      #if HAS_TEMP_BED
        #if FYSTLCD_PAGE_EXIST(TEMP_PREHEAT_BED)
          lcd_set_page(FTPAGE(TEMP_PREHEAT_BED));
        #endif 
      #endif
      break;

    case VARVAL_TOOL_PREHEAT_PLA:
      filament_choice = TEMPERTURE_PREHEAT_CHOISE_E1_PLA;
      filament_temp_preheat();
      filament_choice = TEMPERTURE_PREHEAT_CHOISE_BED_PLA;
      filament_temp_preheat(true);
      dwin_write_extruders_para();
      break;
    case VARVAL_TOOL_PREHEAT_ABS:
      filament_choice = TEMPERTURE_PREHEAT_CHOISE_E1_ABS;
      filament_temp_preheat();
      filament_choice = TEMPERTURE_PREHEAT_CHOISE_BED_ABS;
      filament_temp_preheat(true);
      dwin_write_extruders_para();
      break;
    case VARVAL_TOOL_PREHEAT_PVA:
      filament_choice = TEMPERTURE_PREHEAT_CHOISE_E1_PVA;
      filament_temp_preheat();
      filament_choice = TEMPERTURE_PREHEAT_CHOISE_BED_PVA;
      filament_temp_preheat(true);
      dwin_write_extruders_para();
      break;
    case VARVAL_TOOL_PREHEAT_FLEX:
      filament_choice = TEMPERTURE_PREHEAT_CHOISE_E1_FLEX;
      filament_temp_preheat();
      filament_choice = TEMPERTURE_PREHEAT_CHOISE_BED_FLEX;
      filament_temp_preheat(true);
      dwin_write_extruders_para();
      break;
    case VARVAL_TOOL_PREHEAT_PET:
      filament_choice = TEMPERTURE_PREHEAT_CHOISE_E1_PET;
      filament_temp_preheat();
      filament_choice = TEMPERTURE_PREHEAT_CHOISE_BED_PET;
      filament_temp_preheat(true);
      dwin_write_extruders_para();
      break;
    case VARVAL_TOOL_PREHEAT_HIPS:
      filament_choice = TEMPERTURE_PREHEAT_CHOISE_E1_HIPS;
      filament_temp_preheat();
      filament_choice = TEMPERTURE_PREHEAT_CHOISE_BED_HIPS;
      filament_temp_preheat(true);
      dwin_write_extruders_para();
      break;
    case VARVAL_TOOL_PREHEAT_PP:
      filament_choice = TEMPERTURE_PREHEAT_CHOISE_E1_PP;
      filament_temp_preheat();
      filament_choice = TEMPERTURE_PREHEAT_CHOISE_BED_PP;
      filament_temp_preheat(true);
      dwin_write_extruders_para();
      break;
    case VARVAL_TOOL_PREHEAT_CUSTOM:
      filament_choice = TEMPERTURE_PREHEAT_CHOISE_E1_CUSTOM;
      #if FYSTLCD_PAGE_EXIST(TEMP_PREHEAT_CUSTOM)
        lcd_set_page_force(FTPAGE(TEMP_PREHEAT_CUSTOM));
      #endif      
      break;
      
    case VARVAL_TOOL_PREHEAT_EXTRUDER2_PLA:
      filament_choice = TEMPERTURE_PREHEAT_CHOISE_E2_PLA;
      filament_temp_preheat();
      dwin_write_extruders_para();
      break;
    case VARVAL_TOOL_PREHEAT_EXTRUDER2_ABS:
      filament_choice = TEMPERTURE_PREHEAT_CHOISE_E2_ABS;
      filament_temp_preheat();
      dwin_write_extruders_para();
      break;
    case VARVAL_TOOL_PREHEAT_EXTRUDER2_PVA:
      filament_choice = TEMPERTURE_PREHEAT_CHOISE_E2_PVA;
      filament_temp_preheat();
      dwin_write_extruders_para();
      break;
    case VARVAL_TOOL_PREHEAT_EXTRUDER2_FLEX:
      filament_choice = TEMPERTURE_PREHEAT_CHOISE_E2_FLEX;
      filament_temp_preheat();
      dwin_write_extruders_para();
      break;
    case VARVAL_TOOL_PREHEAT_EXTRUDER2_PET:
      filament_choice = TEMPERTURE_PREHEAT_CHOISE_E2_PET;
      filament_temp_preheat();
      dwin_write_extruders_para();
      break;
    case VARVAL_TOOL_PREHEAT_EXTRUDER2_HIPS:
      filament_choice = TEMPERTURE_PREHEAT_CHOISE_E2_HIPS;
      filament_temp_preheat();
      dwin_write_extruders_para();
      break;
    case VARVAL_TOOL_PREHEAT_EXTRUDER2_PP:
      filament_choice = TEMPERTURE_PREHEAT_CHOISE_E2_PP;
      filament_temp_preheat();
      dwin_write_extruders_para();
      break;
    case VARVAL_TOOL_PREHEAT_EXTRUDER2_CUSTOM:
      filament_choice = TEMPERTURE_PREHEAT_CHOISE_E2_CUSTOM;
      #if FYSTLCD_PAGE_EXIST(TEMP_PREHEAT_CUSTOM)
        lcd_set_page(FTPAGE(TEMP_PREHEAT_CUSTOM));
      #endif      
      break;

    case VARVAL_TOOL_PREHEAT_BED_PLA:
      filament_choice = TEMPERTURE_PREHEAT_CHOISE_BED_PLA;
      filament_temp_preheat(true);
      dwin_write_extruders_para();
      break;
    case VARVAL_TOOL_PREHEAT_BED_ABS:
      filament_choice = TEMPERTURE_PREHEAT_CHOISE_BED_ABS;
      filament_temp_preheat(true);
      dwin_write_extruders_para();
      break;
    case VARVAL_TOOL_PREHEAT_BED_PVA:
      filament_choice = TEMPERTURE_PREHEAT_CHOISE_BED_PVA;
      filament_temp_preheat(true);
      dwin_write_extruders_para();
      break;
    case VARVAL_TOOL_PREHEAT_BED_FLEX:
      filament_choice = TEMPERTURE_PREHEAT_CHOISE_BED_FLEX;
      filament_temp_preheat(true);
      dwin_write_extruders_para();
      break;
    case VARVAL_TOOL_PREHEAT_BED_PET:
      filament_choice = TEMPERTURE_PREHEAT_CHOISE_BED_PET;
      filament_temp_preheat(true);
      dwin_write_extruders_para();
      break;
    case VARVAL_TOOL_PREHEAT_BED_HIPS:
      filament_choice = TEMPERTURE_PREHEAT_CHOISE_BED_HIPS;
      filament_temp_preheat(true);
      dwin_write_extruders_para();
      break;
    case VARVAL_TOOL_PREHEAT_BED_PP:
      filament_choice = TEMPERTURE_PREHEAT_CHOISE_BED_PP;
      filament_temp_preheat(true);
      dwin_write_extruders_para();
      break;
    case VARVAL_TOOL_PREHEAT_BED_CUSTOM:
      filament_choice = TEMPERTURE_PREHEAT_CHOISE_BED_CUSTOM;
      #if FYSTLCD_PAGE_EXIST(TEMP_PREHEAT_CUSTOM)
        lcd_set_page(FTPAGE(TEMP_PREHEAT_CUSTOM));
      #endif      
      break;

    case VARVAL_TOOL_PREHEAT_CUSTOM_APPLY:
      
      break;

    case VARVAL_TOOL_PREHEAT_CUSTOM_CANCEL:
      thermalManager.setTargetHotend(0, 0);
      #if HAS_HEATED_BED
        thermalManager.setTargetBed(0);
      #endif
      #if ENABLED(FIRST_LAYER_CAL)
        if(ftState&FTSTATE_FIRST_LAYER_CAL) {
          ftState &= ~FTSTATE_FIRST_LAYER_CAL;
          first_layer_cal_step = 0;
          lay1cal_init();
        }
      #endif
      break;

    #if ENABLED(BABYSTEPPING)
      #if ENABLED(BABYSTEP_ZPROBE_OFFSET)
        case VARVAL_TOOL_BABYSTEP_UP_Z:
          lcd_babysteps = 20;
          lcd_babystep_zoffset();
          break;
        case VARVAL_TOOL_BABYSTEP_DOWN_Z:
          lcd_babysteps = -20;
          lcd_babystep_zoffset();
          break;
        case VARVAL_TOOL_BABYSTEP_Z_SAVE:
          #if ENABLED(FIRST_LAYER_CAL)
            first_layer_cal_step = 2;
            lay1cal_init();
          #endif
          #if FYSTLCD_PAGE_EXIST(INFO_WAITING)
            lcd_set_page(FTPAGE(INFO_WAITING));
          #endif
          break;
      #endif
    #endif

    case VARVAL_TOOL_CALIBRATE_Z:
      //
      // TMC Z Calibration
      //
      #if ENABLED(TMC_Z_CALIBRATION)
        lcd_calibrating_z = true;
        enqueue_and_echo_commands_P(PSTR("G28\nM915"));
      #endif
      break;

  #if ENABLED(CENTER_ADJUST_ZOFFSET)

    case VARVAL_TOOL_ADJUST_ZOFFSET: {
      #if FYSTLCD_PAGE_EXIST(INFO_WAITING)
        dwin_popup(PSTR("   Leveling is in progress."),EPPT_INFO_WAITING);
      #endif

      zprobe_zoffset_last = zprobe_zoffset;
      SERIAL_ECHOLNPAIR("zprobe_zoffset:",zprobe_zoffset);
      move_dis = 0.1;
      center_adjust_step = 0;
      ftState |= FTSTATE_CENTER_ADJUST_ZOFFSET;
    }
    break;

    case VARVAL_TOOL_ADJUST_ZOFFSET_CONFIRM: {
      #if FYSTLCD_PAGE_EXIST(INFO_WAITING)
        dwin_popup(PSTR("   Leveling is in progress."),EPPT_INFO_WAITING);
      #endif
			center_adjust_step = 3;
    }
    break;

    case VARVAL_TOOL_ADJUST_ZOFFSET_BACK: {
      #if FYSTLCD_PAGE_EXIST(INFO_WAITING)
        dwin_popup(PSTR("   Leveling is in progress."),EPPT_INFO_WAITING);
      #endif
      zprobe_zoffset = zprobe_zoffset_last;
      center_adjust_step = 4;
    }
    break;

  #endif

    case VARVAL_TOOL_M999:
      Running = true;
      MYSERIAL0.flush();
      SERIAL_ECHOLNPGM("M999 ok.");
      break;
    case VARVAL_TOOL_COOLDOWN_ACTIVE:
      thermalManager.setTargetHotend(0, active_extruder);
      break;
    case VARVAL_TOOL_AUTOPID:
      lcd_pid_autotune();
      break;
    case VARVAL_TOOL_AUTOPID_BED:
      lcd_pid_autotune_bed();
      break;
    case VARVAL_TOOL_LOCK_AXIS:
      myFysTLcd.ftCmdStart(VARADDR_STATUS_AXIS_LOCK);
      if (X_ENABLE_READ == X_ENABLE_ON) {
        myFysTLcd.ftCmdJump(2);
        enqueue_and_echo_commands_P(PSTR("M18"));
      }
      else {
        myFysTLcd.ftCmdPut16(1);
        enqueue_and_echo_commands_P(PSTR("M17"));
      }
      myFysTLcd.ftCmdSend();

      break;
      
    #if FAN_COUNT > 0
    
      case VARVAL_TOOL_FAN_SWITCH:
        if (fanSpeeds[0]>0) {
          fanSpeeds[0]=0;
        }
        else {
          fanSpeeds[0]=255;
        }
        dwin_update_fan_status_ico(0);
        break;

      #if FAN_COUNT > 1
        case VARVAL_TOOL_FAN1_SWITCH:
          if (fanSpeeds[1]>0) {
            fanSpeeds[1]=0;
          }
          else {
            fanSpeeds[1]=255;
          }
          dwin_update_fan_status_ico(1);
          break;
      #endif
        
    #endif
    
    #if HAS_SERVOS
      case VARVAL_TOOL_SERVO_SWITCH:
        myFysTLcd.ftCmdStart(VARADDR_STATUS_SERVO);
        if(ftState&FTSTATE_SERVO_STATUS) {
          ftState&=~FTSTATE_SERVO_STATUS;
          servo[0].move(90);
          myFysTLcd.ftCmdJump(2);
        }
        else {
          ftState|=FTSTATE_SERVO_STATUS;
          servo[0].move(10);
          myFysTLcd.ftCmdPut16(1);
        }
        myFysTLcd.ftCmdSend();
        break;
      case VARVAL_TOOL_SERVO_RESET:
        servo[0].move(160);
        break;
    #endif
    
    case VARVAL_TOOL_EXCHANGE_T:
      #if EXTRUDERS>1
        if (active_extruder + 1 < EXTRUDERS) {
          char str[3] = { 'T', active_extruder + '1', 0 };
          enqueue_and_echo_command(str);
        }
        else {
          char str[3] = { 'T', '0', 0 };
          enqueue_and_echo_command(str);
        }
      #endif
      break;
    case VARVAL_TOOL_POPINFO_CONFIRM:
      wait_for_user = false;
      dwin_popup_shutdown();
      break;
      
    case VARVAL_TOOL_AUTO_STOP_FILAMENT:
      if (periodFun == filament_load || periodFun == filament_unload) {
        periodFun = nullptr;
        filament_choice = 0;
      }
      stepper.quick_stop();
      thermalManager.setTargetHotend(0, 0);
      #if EXTRUDERS>1
        thermalManager.setTargetHotend(0, 1);
      #endif
      dwin_write_extruders_para();
      #if FYSTLCD_PAGE_EXIST(UTILITY)
        lcd_set_page(FTPAGE(UTILITY));
      #endif
      break;
      
    case VARVAL_TOOL_AUTO_LEVELING:
      //#if HAS_BED_PROBE
      //  zprobe_zoffset -=0.3;
      //#endif
      dwin_popup(PSTR("\n    Leveling is in progress."),EPPT_INFO_WAITING);
      enqueue_and_echo_commands_P(PSTR("G28"));
      enqueue_and_echo_commands_P(PSTR("G29"));
      break;
        
    case VARVAL_TOOL_AUTO_LEVELING_BREAK:
      break;                        

    case VARVAL_TOOL_LEFTMOVE_X_0_1:
     	move_axis(X_AXIS, -0.1);
      break;
    case VARVAL_TOOL_RIGHTMOVE_X_0_1:
     	move_axis(X_AXIS, 0.1);
      break;
    case VARVAL_TOOL_LEFTMOVE_Y_0_1:
      move_axis(Y_AXIS, -0.1);
      break;
    case VARVAL_TOOL_RIGHTMOVE_Y_0_1:
      move_axis(Y_AXIS, 0.1);
      break;
    case VARVAL_TOOL_LEFTMOVE_Z_0_1:
      move_axis(Z_AXIS, -0.1);
      break;
    case VARVAL_TOOL_RIGHTMOVE_Z_0_1:
      move_axis(Z_AXIS, 0.1);
      break;
    case VARVAL_TOOL_LEFTMOVE_E_0_1:
     	move_axis(E_AXIS, -0.1);
      break;
    case VARVAL_TOOL_RIGHTMOVE_E_0_1:
      move_axis(E_AXIS, 0.1);
      break;
    case VARVAL_TOOL_LEFTMOVE_X_1:
     	move_axis(X_AXIS, -1);
      break;
    case VARVAL_TOOL_RIGHTMOVE_X_1:
     	move_axis(X_AXIS, 1);
      break;
    case VARVAL_TOOL_LEFTMOVE_Y_1:
      move_axis(Y_AXIS, -1);
      break;
    case VARVAL_TOOL_RIGHTMOVE_Y_1:
      move_axis(Y_AXIS, 1);
      break;
    case VARVAL_TOOL_LEFTMOVE_Z_1:
      move_axis(Z_AXIS, -1);
      break;
    case VARVAL_TOOL_RIGHTMOVE_Z_1:
      move_axis(Z_AXIS, 1);
      break;
    case VARVAL_TOOL_LEFTMOVE_E_1:
     	move_axis(E_AXIS, -1);
      break;
    case VARVAL_TOOL_RIGHTMOVE_E_1:
      move_axis(E_AXIS, 1);
      break;
    case VARVAL_TOOL_LEFTMOVE_X_10:
     	move_axis(X_AXIS, -10);
      break;
    case VARVAL_TOOL_RIGHTMOVE_X_10:
     	move_axis(X_AXIS, 10);
      break;
    case VARVAL_TOOL_LEFTMOVE_Y_10:
      move_axis(Y_AXIS, -10);
      break;
    case VARVAL_TOOL_RIGHTMOVE_Y_10:
      move_axis(Y_AXIS, 10);
      break;
    case VARVAL_TOOL_LEFTMOVE_Z_10:
      move_axis(Z_AXIS, -10);
      break;
    case VARVAL_TOOL_RIGHTMOVE_Z_10:
      move_axis(Z_AXIS, 10);
      break;
    case VARVAL_TOOL_LEFTMOVE_E_10:
     	move_axis(E_AXIS, -10);
      break;
    case VARVAL_TOOL_RIGHTMOVE_E_10:
      move_axis(E_AXIS, 10);
      break;
    case VARVAL_TOOL_LEFTMOVE_X:
      move_axis(X_AXIS, -move_dis);
      break;
    case VARVAL_TOOL_RIGHTMOVE_X:
      move_axis(X_AXIS, move_dis);
      break;
    case VARVAL_TOOL_FORWARDMOVE_Y:
      move_axis(Y_AXIS, -move_dis);
      break;
    case VARVAL_TOOL_BACKMOVE_Y:
      move_axis(Y_AXIS, move_dis);
      break;
    case VARVAL_TOOL_UPMOVE_Z:
      move_axis(Z_AXIS, move_dis);
      break;
    case VARVAL_TOOL_DOWNMOVE_Z:
      move_axis(Z_AXIS, -move_dis);
      break;
    case VARVAL_TOOL_ENTER_PAPER_HEIGHT:
      #if FYSTLCD_PAGE_EXIST(TOOL_PAPERHEIGHT)
        lcd_set_page(FTPAGE(TOOL_PAPERHEIGHT));
      #endif
      break;
    case VARVAL_TOOL_RESET:
      wdt_enable(WDTO_30MS);
      while (1) {};
      break;
    case VARVAL_TOOL_OPTION_LEFT:
      dwin_popup_shutdown();
      switch (optionId) {
        #if ENABLED(ADVANCED_PAUSE_FEATURE)
        case 1:
            advanced_pause_menu_response = ADVANCED_PAUSE_RESPONSE_RESUME_PRINT;
            break;
        #endif
        default:
            break;
      }
      break;
    case VARVAL_TOOL_OPTION_RIGHT:
      dwin_popup_shutdown();
      switch (optionId) {
        #if ENABLED(ADVANCED_PAUSE_FEATURE)
        case 1:
          advanced_pause_menu_response = ADVANCED_PAUSE_RESPONSE_EXTRUDE_MORE;
          break;
        #endif
        default:
          break;
      }
      break;
    case VARVAL_TOOL_COOLDOWN_HOTEND:
      thermalManager.setTargetHotend(0, active_extruder);
      myFysTLcd.ftCmdStart(VARADDR_TUNE_PREHEAT_HOTEND_TEMP);
      myFysTLcd.ftCmdJump(2);
      myFysTLcd.ftCmdSend();
      break;
    case VARVAL_TOOL_COOLDOWN_BED:
      #if HAS_HEATED_BED
        thermalManager.setTargetBed(0);
        myFysTLcd.ftCmdStart(VARADDR_TUNE_PREHEAT_BED_TEMP);
        myFysTLcd.ftCmdJump(2);
        myFysTLcd.ftCmdSend();
      #endif
      break;
    case VARVAL_TOOL_EMERGENCY_STOP_MOTOR:       
      disable_all_steppers();
      break;
    }
}

#if ENABLED(POWER_LOSS_RECOVERY)

  static void lcd_power_loss_recovery_resume() {
    char cmd[20];

    // Return to status now
    //lcd_return_to_status();
    #if FYSTLCD_PAGE_EXIST(PRINT)
      lcd_set_page(FTPAGE(PRINT));
    #endif
    
    delay(20);

    #if HAS_HEATED_BED
      const int16_t bt = job_recovery_info.target_temperature_bed;
      if (bt) {
        // Restore the bed temperature
        sprintf_P(cmd, PSTR("M190 S%i"), bt);
        enqueue_and_echo_command(cmd);
      }
    #endif

    // Restore all hotend temperatures
    HOTEND_LOOP() {
      const int16_t et = job_recovery_info.target_temperature[e];
      if (et) {
        #if HOTENDS > 1
          sprintf_P(cmd, PSTR("T%i"), e);
          enqueue_and_echo_command(cmd);
        #endif
        sprintf_P(cmd, PSTR("M109 S%i"), et);
        enqueue_and_echo_command(cmd);
      }
    }

    #if HOTENDS > 1
      sprintf_P(cmd, PSTR("T%i"), job_recovery_info.active_hotend);
      enqueue_and_echo_command(cmd);
    #endif

    // Restore print cooling fan speeds
    for (uint8_t i = 0; i < FAN_COUNT; i++) {
      int16_t f = job_recovery_info.fanSpeeds[i];
      if (f) {
        sprintf_P(cmd, PSTR("M106 P%i S%i"), i, f);
        enqueue_and_echo_command(cmd);
      }
    }

    // Start draining the job recovery command queue
    job_recovery_phase = JOB_RECOVERY_YES;
  }

  static void lcd_power_loss_recovery_cancel() {
    card.removeJobRecoveryFile();
    card.autostart_index = 0;
    //lcd_return_to_status();
    #if FYSTLCD_PAGE_EXIST(MAIN)
      lcd_set_page(FTPAGE(MAIN));
    #endif
    delay(20);
  }

#endif // POWER_LOSS_RECOVERY

#if ENABLED(SDSUPPORT)

  void lcd_sdcard_pause() {
    card.pauseSDPrint();
    print_job_timer.pause();
    #if ENABLED(PARK_HEAD_ON_PAUSE)
      enqueue_and_echo_commands_P(PSTR("M125"));
    #endif
    lcd_reset_status();
  }

  void lcd_sdcard_resume() {
    #if ENABLED(PARK_HEAD_ON_PAUSE)
      enqueue_and_echo_commands_P(PSTR("M24"));
    #else
      card.startFileprint();
      print_job_timer.start();
    #endif
    lcd_reset_status();
  }

  void lcd_sdcard_stop() {
    wait_for_heatup = wait_for_user = false;
    card.abort_sd_printing = true;
    touch_lcd_aborting_print = true;
    lcd_setstatusPGM(PSTR(MSG_PRINT_ABORTED), -1);
    //lcd_return_to_status();
    #if FYSTLCD_PAGE_EXIST(MAIN)
      lcd_set_page(FTPAGE(MAIN));
    #endif
  }

#endif // SDSUPPORT 


static void dwin_on_cmd_print(uint16_t tval)
{
  #if ENABLED(SDSUPPORT)
    if (card.cardOK) {      
      switch (tval) {
        case VARVAL_PRINT_FILELIST:
          if(touch_lcd_aborting_print) return ;

          if (print_job_timer.isRunning() || print_job_timer.isPaused()) {
            #if FYSTLCD_PAGE_EXIST(PRINT)
              lcd_set_page(FTPAGE(PRINT));             
            #endif                
          }
          else {          
            #if FYSTLCD_PAGE_EXIST(FILELIST)
              lcd_set_page(FTPAGE(FILELIST));
            #endif
            card.setroot();
            dwinFileWindowTopIndex = card.getnrfilenames();
            //card.getWorkDirName();
            dwin_update_file_list(true);

            //SERIAL_ECHOLNPAIR("root index:", dwinFileWindowTopIndex);
          }
          break;

        case VARVAL_PRINT_FILELIST_DOWNPAGE:
        case VARVAL_PRINT_FILELIST_UPPAGE: {
          uint8_t realSize=FILE_WINDOW_SIZE;
          uint16_t fileCnt = card.getnrfilenames();
          if(card.isIndir()) realSize-=1; 
          if (tval == VARVAL_PRINT_FILELIST_DOWNPAGE)
          {                  
            if (dwinFileWindowTopIndex > realSize)
            {
              dwinFileWindowTopIndex -= realSize;
            }
            else
            {
              dwinFileWindowTopIndex = fileCnt;
            }
          }
          else
          {
            if (dwinFileWindowTopIndex+ realSize <= fileCnt)
            {
              dwinFileWindowTopIndex += realSize;
            }
            else
            {
              dwinFileWindowTopIndex = fileCnt%realSize;
              if(dwinFileWindowTopIndex==0) 
              {
                dwinFileWindowTopIndex = realSize;
              }
            }
          }
          dwin_update_file_list(true);      
        }
        break;
        
        case VARVAL_PRINT_RESUME_PRINT:
          if (card.isFileOpen()) {
            if (!card.sdprinting) {                                  
              lcd_sdcard_resume();
            }
          }
          break;

        case VARVAL_PRINT_PAUSE:
          if (card.isFileOpen()) {
            if (card.sdprinting) {                                  
              lcd_sdcard_pause();
            }
          }
          break;
            
        case VARVAL_PRINT_STOP:
          if (card.isFileOpen()) {                           
            lcd_sdcard_stop();              
            lcd_icon_state = 0;
          }
          break;
            
        case VARVAL_PRINT_TUNE_ENTER:
          dwin_write_print_tune_para();
          #if FYSTLCD_PAGE_EXIST(PRINT_TUNE)
            lcd_set_page(FTPAGE(PRINT_TUNE));
          #endif
          break;
            
        case VARVAL_PRINT_TUNE_APPLY:
          retPageId = PAGENUM_PRINT;
          currentPageId = PAGENUM_PRINT;
          dwin_read_print_tune_para();
          #if ENABLED(BABYSTEPPING)
            settings.save();
          #endif
          break;

        case VARVAL_PRINT_TUNE_FAN_ENTER:
          dwin_write_print_tune_fan_para();
          #if FYSTLCD_PAGE_EXIST(PRINT_TUNE_FAN)
            lcd_set_page(FTPAGE(PRINT_TUNE_FAN));
          #endif
          break;
            
        case VARVAL_PRINT_TUNE_FAN_APPLY:
          retPageId = PAGENUM_PRINT;
          currentPageId = PAGENUM_PRINT;
          dwin_read_print_tune_fan_para();
          break;

        #if ENABLED(POWER_LOSS_RECOVERY)
          case VARVAL_PRINT_RECOVERY_YES:
            SERIAL_PROTOCOLPGM("Power-loss continue ");
            touch_lcd::ftPuts(VARADDR_PRINTFILE_NAME, job_recovery_info.sd_filename, FYSTLCD_FILENAME_LEN);
            SERIAL_PROTOCOLLNPAIR("file:",job_recovery_info.sd_filename);
            lcd_power_loss_recovery_resume();
            break;              
          case VARVAL_PRINT_RECOVERY_NO:
            SERIAL_PROTOCOLPGM("Power-loss cancel");
            touch_lcd::ftPuts(VARADDR_PRINTFILE_NAME, " ", FYSTLCD_FILENAME_LEN);
            lcd_power_loss_recovery_cancel();
            break;
        #endif
        
        #if defined(FILE_PRINT_NEED_CONRIRM)
          case VARVAL_PRINT_CONFIRM:
            card.startFileprint();
            print_job_timer.start();
            lcd_icon_state = 0;
            #if FYSTLCD_PAGE_EXIST(PRINT)
            lcd_set_page(FTPAGE(PRINT));
            #endif
            break;
        #endif
        
        case VARVAL_PRINT_SD_REFRESH:
          card.initsd();
          if (card.cardOK) {
            dwinFileWindowTopIndex = card.getnrfilenames();
            card.getWorkDirName();
            dwin_update_file_list(true);
            SERIAL_ECHOLNPGM("Refresh ok.");
          }
          else {
            dwinFileWindowTopIndex = 0;
            dwin_update_file_list(false);
          }
          break;
            
        case VARVAL_PRINT_REPRINT:
          #if ENABLED(FYS_RECORD_CURRENT_PRINT_FILE)
            if (card.isFileOpen())card.closefile();
            card.openFile(currentPrintFile, true);
            if (!card.isFileOpen())
            {
                lcd_popup("Target file open fail.");
                return;
            }
            card.startFileprint();
            print_job_timer.start();
          #endif

          #if FYSTLCD_PAGE_EXIST(PRINT)
            lcd_set_page(FTPAGE(PRINT));
          #endif
          break;

        case VARVAL_PRINT_FILE_HOME:
          while(card.isIndir()) card.updir();
          #if FYSTLCD_PAGE_EXIST(MAIN)
            lcd_set_page(FTPAGE(MAIN));
          #endif
          break;
            
        default:
            for (uint8_t i = 0; i < FILE_WINDOW_SIZE; i++) {
              if (tval == VARVAL_PRINT_FILECHOOSE[i]) {                        
                if(card.isIndir()) {
                  if(dwinFileWindowTopIndex+1<=i) break;
                }
                else {
                  if(dwinFileWindowTopIndex<=i) break;
                }
                
              	if (card.isIndir() && 0 == i) { // meaning ".."
              		card.updir();
              		dwinFileWindowTopIndex = card.get_num_Files();
              		dwin_update_file_list(true);       
              		SERIAL_ECHOLNPAIR("root index:", dwinFileWindowTopIndex);
              	}
              	else {
                  uint8_t realIndex;
                  if(card.isIndir()) {
                    realIndex = dwinFileWindowTopIndex - i;
                  }
                  else {
                    realIndex = dwinFileWindowTopIndex - i - 1;
                  }
                                    
              		//if (card.getfilename(realIndex)) // handles empty directory (no action)
              		//{
              		  card.getfilename(realIndex);
              			if (card.filenameIsDir) {
              				card.chdir(card.filename);                				
              				dwinFileWindowTopIndex = card.get_num_Files();
                      //SERIAL_ECHOLNPAIR("isDir index:", dwinFileWindowTopIndex);
              				dwin_update_file_list(true);
              			}
              			else {              			  
                      //SERIAL_ECHOLNPAIR("file select:", card.filename);
                      #if defined(FILE_PRINT_NEED_CONRIRM)

                        #if FYSTLCD_PAGE_EXIST(PRINTFILE_CONFIRM)
                         lcd_set_page(FTPAGE(PRINTFILE_CONFIRM));
                        #endif

                      #elif FYSTLCD_PAGE_EXIST(PRINT)
                        lcd_set_page(FTPAGE(PRINT));
                      #endif

                      dwin_file_select(card.filename,card.longFilename);                                                
                      break;
              			}
              		}
              	//}                                                       
              }
            }
            break;
      }
    }
    else {
      if(!IS_SD_INSERTED()) {
        lcd_setalertstatusPGM(PSTR("SD card not found."));
        SERIAL_ECHOLN("SD card not found.");
        return;
      }

      card.initsd();
      if(!card.cardOK) {
        lcd_setalertstatusPGM(PSTR(MSG_SD_INIT_FAIL));
        SERIAL_ECHOLNPGM(MSG_SD_INIT_FAIL);
        return;
      }
      
      switch (tval) {
        case VARVAL_PRINT_FILELIST:
          if (print_job_timer.isRunning() || print_job_timer.isPaused()) {
            #if FYSTLCD_PAGE_EXIST(PRINT)
              lcd_set_page(FTPAGE(PRINT));
            #endif                
          }
          else {
            #if FYSTLCD_PAGE_EXIST(FILELIST)
              lcd_set_page(FTPAGE(FILELIST));
            #endif
            card.setroot();
            dwinFileWindowTopIndex = card.getnrfilenames();
            //card.getWorkDirName();
            dwin_update_file_list(true);
            //SERIAL_ECHOLNPAIR("root index:", dwinFileWindowTopIndex);
          }
          break;
        case VARVAL_PRINT_SD_REFRESH:
          if (card.cardOK) {
            dwinFileWindowTopIndex = card.getnrfilenames();
            card.getWorkDirName();
            dwin_update_file_list(true);
          }
          else {
            dwinFileWindowTopIndex = 0;
            dwin_update_file_list(false);
          }
          break;

        #if ENABLED(POWER_LOSS_RECOVERY)
          case VARVAL_PRINT_RECOVERY_NO:
            touch_lcd::ftPuts(VARADDR_PRINTFILE_NAME, " ", FYSTLCD_FILENAME_LEN);
            lcd_power_loss_recovery_cancel();
            break;
        #endif
      }
    }
  #endif
}

static void dwin_on_cmd_setting(uint16_t tval) {
  switch (tval) {
    case VARVAL_SETTING_MOTOR_ENTER:
        dwin_write_motor_para();
        #if HOTENDS > 1
          dwin_write_extruders_motor();
        #endif
        #if FYSTLCD_PAGE_EXIST(SETTING_MOTOR)
          lcd_set_page_force(FTPAGE(SETTING_MOTOR));
        #endif
        break;
    case VARVAL_SETTING_MOTOR_APPLY:
        dwin_read_motor_para();
        #if HOTENDS > 1
          dwin_read_extruders_motor_para();
        #endif  
        break;
    case VARVAL_SETTING_MOTOR_SAVE:
        dwin_read_motor_para();
        #if HOTENDS > 1
          dwin_read_extruders_motor_para();
        #endif
        lcd_save_settings();
        #if FYSTLCD_PAGE_EXIST(SETTING)
          lcd_set_page(FTPAGE(SETTING));
        #endif
        break;
    case VARVAL_SETTING_LEVELING_ENTER:
        dwin_write_leveling_para();
        #if FYSTLCD_PAGE_EXIST(SETTING_LEVELING)
          lcd_set_page(FTPAGE(SETTING_LEVELING));
        #endif
        break;
    case VARVAL_SETTING_LEVELING_APPLY:
        dwin_read_leveling_para();
        #if FYSTLCD_PAGE_EXIST(SETTING)
          lcd_set_page(FTPAGE(SETTING));
        #endif
        break;
    case VARVAL_SETTING_LEVELING_SAVE:
        dwin_read_leveling_para();
        lcd_save_settings();
        break;
    case VARVAL_SETTING_TEMP_PARA_ENTER:
        dwin_write_temperture_para();
        #if HOTENDS > 1
          dwin_write_extruders_temp();
        #endif
        #if FYSTLCD_PAGE_EXIST(SETTING_TEMP_PARA)
          lcd_set_page(FTPAGE(SETTING_TEMP_PARA));
        #endif
        break;
    case VARVAL_SETTING_TEMP_PARA_APPLY:
        dwin_read_temperture_para();
        #if HOTENDS > 1
          dwin_read_extruders_temp();
        #endif
        #if FYSTLCD_PAGE_EXIST(SETTING)
          lcd_set_page(FTPAGE(SETTING));
        #endif
        break;
    case VARVAL_SETTING_TEMP_PARA_SAVE:
        dwin_read_temperture_para();
        #if HOTENDS > 1
          dwin_read_extruders_temp();
        #endif
        lcd_save_settings();
        #if FYSTLCD_PAGE_EXIST(SETTING)
          lcd_set_page(FTPAGE(SETTING));
        #endif
        break;
    case VARVAL_SETTING_MATERIAL_ENTER:
        dwin_write_filament_para();
        #if FYSTLCD_PAGE_EXIST(SETTING_MATERIAL)
        lcd_set_page(FTPAGE(SETTING_MATERIAL));
        #endif
        break;
    case VARVAL_SETTING_MATERIAL_APPLY:
        dwin_read_filament_para();
        break;
    case VARVAL_SETTING_MATERIAL_SAVE:
        dwin_read_filament_para();
        lcd_save_settings();
        break;
    case VARVAL_SETTING_TMC2130_ENTER:
        dwin_write_tmc2130_para();
        #if FYSTLCD_PAGE_EXIST(SETTING_TMC2130)
        lcd_set_page(FTPAGE(SETTING_TMC2130));
        #endif
        break;
    case VARVAL_SETTING_TMC2130_APPLY:
        dwin_read_tmc2130_para();
        break;
    case VARVAL_SETTING_TMC2130_SAVE:
        dwin_read_tmc2130_para();
        lcd_save_settings();
        break;
    case VARVAL_SETTINGS_SAVE:
        lcd_save_settings();
        break;
    case VARVAL_SETTINGS_RESET:
        print_job_timer.initStats();
        settings.reset();
        break;
    case VARVAL_SETTINGS_RESETSAVE:
        print_job_timer.initStats();
        settings.reset();
        lcd_save_settings();
        break;
    case VARVAL_SETTINGS_LOAD:
        lcd_load_settings();
        break;
    case VARVAL_SETTINGS_SYSTEM:
        dwin_write_system_para();
        #if FYSTLCD_PAGE_EXIST(SETTING_SYSTEM)
          lcd_set_page(FTPAGE(SETTING_SYSTEM));
        #endif
        break;
    case VARVAL_SETTINGS_SYSTEM_APPLY:
        dwin_read_system_para();
        break;
    case VARVAL_SETTINGS_SYSTEM_SAVE:
        dwin_read_system_para();
        lcd_save_settings();
        break;
    case VARVAL_SETTING_ZOFFSET_ENTER:
        dwin_write_zoffset_para();
        #if FYSTLCD_PAGE_EXIST(SETTING_ZOFFSET)
          lcd_set_page(FTPAGE(SETTING_ZOFFSET));
        #endif
        break;
    case VARVAL_SETTING_ZOFFSET_APPLY:
        dwin_read_zoffset_para();
        break;
    case VARVAL_SETTING_ZOFFSET_SAVE:
        dwin_read_zoffset_para();
        lcd_save_settings();
        break;
  }
}

static void dwin_on_cmd(millis_t& tNow) {
  if (!touch_lcd::ftAvailableCmd()) return;
  
	#if defined(FYS_ACTIVE_TIME_OVER)
    previous_move_ms = tNow;
	#endif

  uint16_t tval = touch_lcd::ftCmdVal16();

  #if 0
  SERIAL_ECHOPGM(" Addr:");
  report_commands(touch_lcd::ftAddr);
  SERIAL_ECHOPGM(" Val:");
  report_commands(tval);
  #endif
  
  uint8_t cmd[2];
  switch (touch_lcd::ftAddr) {
  case VARADDR_TOOL_ZOFFSET_BABYSTEPS:
    lcd_babysteps = tval;
    break;
    
  case VARADDR_TOOL:
      dwin_on_cmd_tool(tval);
      break;
      
  case VARADDR_PRINT: 
      dwin_on_cmd_print(tval);
      break;
    
  case VARADDR_SETTING:
      dwin_on_cmd_setting(tval);
      break;
      
  #if HOTENDS > 1
    case VARADDR_EXTRUDERS:
      switch (tval) {
        case VARVAL_EXTRUDERS_OFFSET_ENTER:
          dwin_write_extruders_offset();
          #if FYSTLCD_PAGE_EXIST(SETTING_EXTRUDERS_OFFSET)
          lcd_set_page(FTPAGE(SETTING_EXTRUDERS_OFFSET));
          #endif
          break;
        case VARVAL_EXTRUDERS_OFFSET_APPLY:
          dwin_read_extruders_offset();
          break;
        case VARVAL_EXTRUDERS_OFFSET_SAVE:
          dwin_read_extruders_offset();
          lcd_save_settings();
          break;
        case VARVAL_EXTRUDERS_MOTOR_ENTER:
          dwin_write_extruders_motor();
          #if FYSTLCD_PAGE_EXIST(SETTING_EXTRUDERS_MOTOR)
          lcd_set_page(FTPAGE(SETTING_EXTRUDERS_MOTOR));
          #endif
          break;
        case VARVAL_EXTRUDERS_MOTOR_APPLY:
          dwin_read_extruders_motor_para();
          break;
        case VARVAL_EXTRUDERS_MOTOR_SAVE:
          dwin_read_extruders_motor_para();
          lcd_save_settings();
          break;
        case VARVAL_EXTRUDERS_TEMP_ENTER:
          dwin_write_extruders_temp();
          #if FYSTLCD_PAGE_EXIST(SETTING_EXTRUDERS_TEMP)
          lcd_set_page(FTPAGE(SETTING_EXTRUDERS_TEMP));
          #endif
          break;
        case VARVAL_EXTRUDERS_TEMP_APPLY:
          dwin_read_extruders_temp();
          #if FYSTLCD_PAGE_EXIST(SETTING)
            lcd_set_page(FTPAGE(SETTING));
          #endif
          break;
        case VARVAL_EXTRUDERS_TEMP_SAVE:
          dwin_read_extruders_temp();
          lcd_save_settings();
          #if FYSTLCD_PAGE_EXIST(SETTING)
            lcd_set_page(FTPAGE(SETTING));
          #endif
          break;
      }
      break;
  #endif
  
  case VARADDR_FILAMENT_AUTO_ADD:
    if(tval > VARVAL_FILAMENT_OPE_EXTRU2) {
      enqueue_and_echo_commands_P(PSTR("G28 XY"));
    }
    switch (tval) {
      case VARVAL_FILAMENT_OPE_EXTRU1:
        #if FYSTLCD_PAGE_EXIST(FILAMENT_LOAD_1)
          lcd_set_page(FTPAGE(FILAMENT_LOAD_1));
        #endif
        break;
      case VARVAL_FILAMENT_OPE_EXTRU2:          
        #if EXTRUDERS>1
          #if FYSTLCD_PAGE_EXIST(FILAMENT_LOAD_2)
            lcd_set_page(FTPAGE(FILAMENT_LOAD_2));
          #endif
        #endif
        break;
        
      case VARVAL_FILAMENT_OPE_EXTRU1_PLA:
        filament_choice = TEMPERTURE_PREHEAT_CHOISE_E1_PLA;
        filament_temp_preheat(false);
        periodFun = filament_load;
        break;
      case VARVAL_FILAMENT_OPE_EXTRU1_ABS:
        filament_choice = TEMPERTURE_PREHEAT_CHOISE_E1_ABS;
        filament_temp_preheat(false);
        periodFun = filament_load;
        break;
      case VARVAL_FILAMENT_OPE_EXTRU1_PET:
        filament_choice = TEMPERTURE_PREHEAT_CHOISE_E1_PET;
        filament_temp_preheat(false);
        periodFun = filament_load;
        break;
      case VARVAL_FILAMENT_OPE_EXTRU1_FLEX:
        filament_choice = TEMPERTURE_PREHEAT_CHOISE_E1_FLEX;
        filament_temp_preheat(false);
        periodFun = filament_load;
        break;

      #if EXTRUDERS>1
        case VARVAL_FILAMENT_OPE_EXTRU2_PLA:
          filament_choice = TEMPERTURE_PREHEAT_CHOISE_E2_PLA;
          filament_temp_preheat(false);
          periodFun = filament_load;
          break;
        case VARVAL_FILAMENT_OPE_EXTRU2_ABS:
          filament_choice = TEMPERTURE_PREHEAT_CHOISE_E2_ABS;
          filament_temp_preheat(false);
          periodFun = filament_load;
          break;
        case VARVAL_FILAMENT_OPE_EXTRU2_PET:
          filament_choice = TEMPERTURE_PREHEAT_CHOISE_E2_PLA;
          filament_temp_preheat(false);
          periodFun = filament_load;
          break;
        case VARVAL_FILAMENT_OPE_EXTRU2_FLEX:
          filament_choice = TEMPERTURE_PREHEAT_CHOISE_E2_PLA;
          filament_temp_preheat(false);
          periodFun = filament_load;          
          break;      
      #endif
    }      
    break;
    
  case VARADDR_FILAMENT_AUTO_REMOVE:
    if(tval > VARVAL_FILAMENT_OPE_EXTRU2) {
      unload_purge_times = 0;
      enqueue_and_echo_commands_P(PSTR("G28 XY"));
    }
    switch (tval) {
      case VARVAL_FILAMENT_OPE_EXTRU1:
        #if FYSTLCD_PAGE_EXIST(FILAMENT_UNLOAD_1)
          lcd_set_page(FTPAGE(FILAMENT_UNLOAD_1));
        #endif
        break;
      case VARVAL_FILAMENT_OPE_EXTRU2:          
        #if EXTRUDERS>1
          #if FYSTLCD_PAGE_EXIST(FILAMENT_UNLOAD_2)
            lcd_set_page(FTPAGE(FILAMENT_UNLOAD_2));
          #endif
        #endif
        break;

      case VARVAL_FILAMENT_OPE_EXTRU1_PLA:
        filament_choice = TEMPERTURE_PREHEAT_CHOISE_E1_PLA;
        filament_temp_preheat(false);
        periodFun = filament_unload;
        break;
      case VARVAL_FILAMENT_OPE_EXTRU1_ABS:
        filament_choice = TEMPERTURE_PREHEAT_CHOISE_E1_ABS;
        filament_temp_preheat(false);
        periodFun = filament_unload;
        break;
      case VARVAL_FILAMENT_OPE_EXTRU1_PET:
        filament_choice = TEMPERTURE_PREHEAT_CHOISE_E1_PET;
        filament_temp_preheat(false);
        periodFun = filament_unload;
        break;
      case VARVAL_FILAMENT_OPE_EXTRU1_FLEX:
        filament_choice = TEMPERTURE_PREHEAT_CHOISE_E1_FLEX;
        filament_temp_preheat(false);
        periodFun = filament_unload;
        break;

      #if EXTRUDERS>1
        case VARVAL_FILAMENT_OPE_EXTRU2_PLA:
          filament_choice = TEMPERTURE_PREHEAT_CHOISE_E2_PLA;
          filament_temp_preheat(false);
          periodFun = filament_unload;
          break;
        case VARVAL_FILAMENT_OPE_EXTRU2_ABS:
          filament_choice = TEMPERTURE_PREHEAT_CHOISE_E2_ABS;
          filament_temp_preheat(false);
          periodFun = filament_unload;
          break;
        case VARVAL_FILAMENT_OPE_EXTRU2_PET:
          filament_choice = TEMPERTURE_PREHEAT_CHOISE_E2_PLA;
          filament_temp_preheat(false);
          periodFun = filament_unload;
          break;
        case VARVAL_FILAMENT_OPE_EXTRU2_FLEX:
          filament_choice = TEMPERTURE_PREHEAT_CHOISE_E2_PLA;
          filament_temp_preheat(false);
          periodFun = filament_unload;
          break;      
      #endif   
    }      
    break;

#if ENABLED(FIRST_LAYER_CAL)

  case VARADDR_LAYER1_PREHEAT:
    if(tval > VARVAL_LAYER1_PREHEAT_EXTRU2) {
      enqueue_and_echo_commands_now_P(PSTR("G28 XY"));
    }
    switch (tval) {
      case VARVAL_LAYER1_PREHEAT_EXTRU1_PLA:
        filament_choice = TEMPERTURE_PREHEAT_CHOISE_E1_PLA;
        first_layer_cal_step = 20;
        ftState |= FTSTATE_FIRST_LAYER_CAL;
        break;
      case VARVAL_LAYER1_PREHEAT_EXTRU1_ABS:
        filament_choice = TEMPERTURE_PREHEAT_CHOISE_E1_ABS;
        first_layer_cal_step = 21;
        ftState |= FTSTATE_FIRST_LAYER_CAL;
        break;
      case VARVAL_LAYER1_PREHEAT_EXTRU1_PET:
        filament_choice = TEMPERTURE_PREHEAT_CHOISE_E1_PET;
        first_layer_cal_step = 22;
        ftState |= FTSTATE_FIRST_LAYER_CAL;
        break;
      case VARVAL_LAYER1_PREHEAT_EXTRU1_FLEX:
        filament_choice = TEMPERTURE_PREHEAT_CHOISE_E1_FLEX;
        first_layer_cal_step = 23;
        ftState |= FTSTATE_FIRST_LAYER_CAL;
        break;
    }
    break;

#endif

  case VARADDR_MOVE_DIS:
      move_dis = tval;
      move_dis /= FYSTLCD_DOT_TEN_MUL;
      myFysTLcd.ftCmdStart(VARADDR_MOVE_DIS_SIGN);
      myFysTLcd.ftCmdPut16(tval);
      myFysTLcd.ftCmdSend();
      break;

  case VARADDR_MOVE_SPEED:
      move_feedrate = tval;
      move_feedrate /= FYSTLCD_DOT_TEN_MUL;
      myFysTLcd.ftCmdStart(VARADDR_MOVE_SPEED_SIGN);
      myFysTLcd.ftCmdPut16(tval);
      myFysTLcd.ftCmdSend();
      break;

  case VARADDR_JUMP_PAGE:
      retPageId = tval; 
      currentPageId = tval;
      //SERIAL_ECHOPGM("Page to");
      //MYSERIAL0.println((int)tval);
      break;

  case VARADDR_TUNE_PRINT_PERCENTAGE:
      oldFeedratePercentage = tval;
      feedrate_percentage = tval;
      break;
  case VARADDR_TUNE_FAN_SPEED:
    #if FAN_COUNT > 0
      oldFanSpeed=tval;
      fanSpeeds[active_extruder] = tval;
    #endif
    break;

  case VARADDR_TUNE_PREHEAT_CUSTOM:
    filament_choice = TEMPERTURE_PREHEAT_CHOISE_E1_CUSTOM;
    myFysTLcd.ftCmdStart(VARADDR_TUNE_PREHEAT_CUSTOM);
    if (myFysTLcd.ftCmdReceive(2)) {
      myFysTLcd.ftCmdGetI16(lcd_preheat_hotend_temp[FILAMENTS-1]);
      filament_temp_preheat();
    }

    dwin_write_extruders_para();
    break;

  case VARADDR_TUNE_PREHEAT_CUSTOM_BED:
    filament_choice = TEMPERTURE_PREHEAT_CHOISE_BED_CUSTOM;
    myFysTLcd.ftCmdStart(VARADDR_TUNE_PREHEAT_CUSTOM_BED);
    if (myFysTLcd.ftCmdReceive(2)) {
      myFysTLcd.ftCmdGetI16(lcd_preheat_bed_temp[FILAMENTS-1]);
      filament_temp_preheat(true);
    }

    dwin_write_extruders_para();
    break;

  case VARADDR_TUNE_PREHEAT_HOTEND_TEMP:
    thermalManager.setTargetHotend(tval, 0);
    break;
  case VARADDR_TUNE_PREHEAT_BED_TEMP:
    #if HAS_HEATED_BED
      thermalManager.setTargetBed(tval);
    #endif
    break;
  case VARADDR_TUNE_PREHEAT_HOTEND2_SELECT:      
    #if EXTRUDERS>1
      #if FYSTLCD_PAGE_EXIST(FILAMENT_PURGE2)
        lcd_set_page(FTPAGE(FILAMENT_PURGE2));
      #endif
    #endif
    break;
  case VARADDR_TUNE_PREHEAT_HOTEND2_TEMP:
    #if EXTRUDERS>1
      thermalManager.setTargetHotend(tval, 1);
    #endif
    break;

  case VARADDR_TUNE_MOVE_E:
    switch(tval) {
      case VARVAL_TUNE_FORWARD_E:      
        move_axis(E_AXIS, 5);
        break;
      case VARVAL_TUNE_BACKWARD_E:
        move_axis(E_AXIS, -5);
        break;
    }
    break;

  default:
    break;;
  }
}

#if defined(VARADDR_PROMPT_DATA)&&VARADDR_PROMPT_DATA>0
static void lcd_period_prompt_report() {
  /*******************
  *   10A0 X_MIN_ENDSTOP 
  *   10A1 X_MAX_ENDSTOP 
  *   10A2 Y_MIN_ENDSTOP 
  *   10A3 Y_MAX_ENDSTOP 
  *   10A4 Z_MIN_ENDSTOP 
  *   10A5 Z_MAX_ENDSTOP 
  *   10A6 reserved
  *   10A7 FAN ico 
  *   10A8 PRINT ico 
  *   10A9 reserved
  *   10AA autoPid ico 
  ********************/
  myFysTLcd.ftCmdStart(VARADDR_PROMPT_DATA);
  #if HAS_X_MIN //10A0
    if(READ(X_MIN_PIN) ^ X_MIN_ENDSTOP_INVERTING) myFysTLcd.ftCmdPut16(1);
    else myFysTLcd.ftCmdJump(2);
  #else
    myFysTLcd.ftCmdJump(2);
  #endif

  #if HAS_X_MAX //10A1
    if (READ(X_MAX_PIN) ^ X_MAX_ENDSTOP_INVERTING)myFysTLcd.ftCmdPut16(1);
    else myFysTLcd.ftCmdJump(2);
  #else
    myFysTLcd.ftCmdJump(2);
  #endif

  #if HAS_Y_MIN //10A2
    if(READ(Y_MIN_PIN) ^ Y_MIN_ENDSTOP_INVERTING)myFysTLcd.ftCmdPut16(1);
    else myFysTLcd.ftCmdJump(2);
  #else
    myFysTLcd.ftCmdJump(2);
  #endif

  #if HAS_Y_MAX //10A3
    if(READ(Y_MAX_PIN) ^ Y_MAX_ENDSTOP_INVERTING)myFysTLcd.ftCmdPut16(1);
    else myFysTLcd.ftCmdJump(2);
  #else
    myFysTLcd.ftCmdJump(2);
  #endif

  #if HAS_Z_MIN //10A4
    if(READ(Z_MIN_PIN) ^ Z_MIN_ENDSTOP_INVERTING)myFysTLcd.ftCmdPut16(1);
    else myFysTLcd.ftCmdJump(2);
  #else
    myFysTLcd.ftCmdJump(2);
  #endif

  #if HAS_Z_MAX //10A5
    if(READ(Z_MAX_PIN) ^ Z_MAX_ENDSTOP_INVERTING) myFysTLcd.ftCmdPut16(1);
    else myFysTLcd.ftCmdJump(2);
  #else
    myFysTLcd.ftCmdJump(2);
  #endif

  myFysTLcd.ftCmdJump(2);//10A6 reserved
  
  lcd_icon_state++;
  if (lcd_icon_state > 9) lcd_icon_state = 0;

  //10A7 fan ico
  #if FAN_COUNT > 0
    if (fanSpeeds[active_extruder] > 0) {
      myFysTLcd.ftCmdPut16(lcd_icon_state);      
    } 
    else {
      myFysTLcd.ftCmdJump(2);
    }
  #else
    myFysTLcd.ftCmdJump(2);
  #endif

  //10A8 print ico
  #if ENABLED(SDSUPPORT)
    if (card.sdprinting)
      myFysTLcd.ftCmdPut16(lcd_icon_state);
    else
  #endif
  myFysTLcd.ftCmdJump(2);

  //10A9 waiting ico
  myFysTLcd.ftCmdPut16(lcd_icon_state);

  //10AA auto PID ico
  if (ftState&FTSTATE_AUTOPID_ING) {
    myFysTLcd.ftCmdPut16(lcd_icon_state);
  }
  else {
    myFysTLcd.ftCmdJump(2);
  }
  myFysTLcd.ftCmdSend();
}
#endif

static void lcd_period_report(int16_t s) {
  uint8_t i;
  char buffer[21];

  myFysTLcd.ftCmdStart(VARADDR_PERIOD_DATA);
  
  myFysTLcd.ftCmdPut16((int16_t)thermalManager.degHotend(0));
  
  #if HAS_FAN0
    myFysTLcd.ftCmdPut16(fanSpeeds[0]);
  #else
    myFysTLcd.ftCmdJump(2);
  #endif
  
  #if HAS_TEMP_BED
   myFysTLcd.ftCmdPut16((int16_t)thermalManager.degBed());
  #else
   myFysTLcd.ftCmdJump(2);
  #endif
  myFysTLcd.ftCmdPut16(feedrate_percentage);

  // 
  for (i = 0; i < 4;i++){
    myFysTLcd.ftCmdPutF16(current_position[i]);
  }

  #if ENABLED(SDSUPPORT)
    static int progress = 0;
    if (IS_SD_PRINTING())progress = card.percentDone();
    if (progress> 100)progress = 0;
    myFysTLcd.ftCmdPut16(progress);
  #else
    myFysTLcd.ftCmdJump(2);
  #endif
  
  #if defined(FYS_ACTIVE_TIME_OVER)
    myFysTLcd.ftCmdPut16(s);
  #else
    myFysTLcd.ftCmdJump(2);
  #endif
  myFysTLcd.ftCmdSend();
  
  // Zprope zoffset
  #if HAS_BED_PROBE
    myFysTLcd.ftCmdStart(VARADDR_ZOFFSET_DATA);
    myFysTLcd.ftCmdPutF16_2(zprobe_zoffset);
    myFysTLcd.ftCmdSend();
  #endif
  
  #if EXTRUDERS>1
    myFysTLcd.ftCmdStart(VARADDR_EXTRUDERS_TEMP);
    for (uint8_t e = 1; e < EXTRUDERS; e++) {
      myFysTLcd.ftCmdPut16((int16_t)thermalManager.degHotend(e));
    }
    myFysTLcd.ftCmdSend();
 #endif

  #if HAS_FAN1    
    myFysTLcd.ftCmdStart(VARADDR_FANS);
    myFysTLcd.ftCmdPut16(fanSpeeds[1]);
    myFysTLcd.ftCmdSend();      
  #endif

  myFysTLcd.ftCmdStart(VARADDR_FLOW);
  for (uint8_t e = 0; e < EXTRUDERS; e++) {
    myFysTLcd.ftCmdPut16(planner.flow_percentage[e]);
  }
  myFysTLcd.ftCmdSend();

  #if ENABLED(SDSUPPORT)
  
    if (IS_SD_PRINTING()) {
      myFysTLcd.ftCmdStart(VARADDR_PRINT_TIME);
      millis_t pt = print_job_timer.duration();
      card.percentDone();
      float total = float(pt) / (float)card.percentDone();
      static float smoothTotal = 0;
      myFysTLcd.ftPutTime(pt);
      myFysTLcd.ftCmdSend(8);
      smoothTotal = (smoothTotal * 999L + total) / 1000L;
      if (isinf(smoothTotal))smoothTotal = total;
      if (pt < 120) {
        myFysTLcd.ftPutPGM(PSTR("Unknown."));
        smoothTotal = total;
      }
      else {
        pt = smoothTotal;
        myFysTLcd.ftPutTime(pt);
      }
      myFysTLcd.ftCmdSend();
    }
    
    #if ENABLED(PRINTCOUNTER)

      printStatistics state = print_job_timer.getStats();
      sprintf_P(buffer, PSTR("%u"), state.totalPrints);
      touch_lcd::ftPuts(VARADDR_INFO_PRINT, buffer, ATTACH_STR_LEN);

      duration_t elapsed = state.printTime;
      elapsed.toString(buffer);
      touch_lcd::ftPuts(VARADDR_INFO_PRINT_ACC_TIME, buffer, ATTACH_STR_LEN);

    #endif

  #endif
}

// Update the status page data 
void lcd_finishstatus(const bool persist=false) {
  UNUSED(persist);
}

bool lcd_hasstatus() { return (lcd_status_message[0] != '\0'); }

void lcd_setstatus(const char * const message, const bool persist) {
  if (lcd_status_message_level > 0) return;

  // Here we have a problem. The message is encoded in UTF8, so
  // arbitrarily cutting it will be a problem. We MUST be sure
  // that there is no cutting in the middle of a multibyte character!

  // Get a pointer to the null terminator
  const char* pend = message + strlen(message);

  //  If length of supplied UTF8 string is greater than
  // our buffer size, start cutting whole UTF8 chars
  while ((pend - message) > MAX_MESSAGE_LENGTH) {
    --pend;
    while (!START_OF_UTF8_CHAR(*pend)) --pend;
  };

  // At this point, we have the proper cut point. Use it
  uint8_t maxLen = pend - message;
  strncpy(lcd_status_message, message, maxLen);
  lcd_status_message[maxLen] = '\0';

  lcd_finishstatus(persist);
}

void lcd_setstatusPGM(const char * const message, int8_t level) {
  if (level < 0) level = lcd_status_message_level = 0;
  if (level < lcd_status_message_level) return;
  lcd_status_message_level = level;

  // Here we have a problem. The message is encoded in UTF8, so
  // arbitrarily cutting it will be a problem. We MUST be sure
  // that there is no cutting in the middle of a multibyte character!

  // Get a pointer to the null terminator
  const char* pend = message + strlen_P(message);

  //  If length of supplied UTF8 string is greater than
  // our buffer size, start cutting whole UTF8 chars
  while ((pend - message) > MAX_MESSAGE_LENGTH) {
    --pend;
    while (!START_OF_UTF8_CHAR(pgm_read_byte(pend))) --pend;
  };

  // At this point, we have the proper cut point. Use it
  uint8_t maxLen = pend - message;
  strncpy_P(lcd_status_message, message, maxLen);
  lcd_status_message[maxLen] = '\0';

  lcd_finishstatus(level > 0);
}

void lcd_status_printf_P(const uint8_t level, const char * const fmt, ...) {
  if (level < lcd_status_message_level) return;
  lcd_status_message_level = level;
  va_list args;
  va_start(args, fmt);
  vsnprintf_P(lcd_status_message, MAX_MESSAGE_LENGTH, fmt, args);
  va_end(args);
  lcd_finishstatus(level > 0);
}

// 
void lcd_setalertstatusPGM(const char * const message) {
  lcd_setstatusPGM(message, 1);
  //lcd_return_to_status();
}

void lcd_reset_alert_level() { lcd_status_message_level = 0; }

void lcd_print_utf(const char *str, uint8_t n) {
  char c;
  uint8_t i=0;
  while (n && (c = *str)) {
    n -= charset_mapper(c);
    ++str;
    utf_string_buf[i++] = utf_char_buf;
  }
  utf_string_buf[i] = '\0';
}

// we just save the translated char to utf_char_buf
// n : the length of the text box
void lcd_printPGM_utf(const char *str, uint8_t n) {
  char c;
  uint8_t i=0;
  while (n && (c = pgm_read_byte(str))) {
    n -= charset_mapper(c);
    ++str;
    utf_string_buf[i++] = utf_char_buf;
  }
  utf_string_buf[i] = '\0';
}

// just fill in datas
void lcd_kill_screen() {
  dwin_clear_pop_infos();

  //SERIAL_ECHOLN("lcd_kill_screen");

  //SERIAL_ECHOLNPAIR("lcd_status_message ",lcd_status_message);
  touch_lcd::ftPuts(VARADDR_POP_INFOS[0], lcd_status_message, INFO_POPUP_LEN);

  lcd_printPGM_utf(PSTR(MSG_HALTED),INFO_POPUP_LEN);
  //SERIAL_ECHOLNPAIR("utf_string_buf ",utf_string_buf);
  touch_lcd::ftPuts(VARADDR_POP_INFOS[1], utf_string_buf, INFO_POPUP_LEN);

  lcd_printPGM_utf(PSTR(MSG_PLEASE_RESET),INFO_POPUP_LEN);
  //SERIAL_ECHOLNPAIR("utf_string_buf ",utf_string_buf);
  touch_lcd::ftPuts(VARADDR_POP_INFOS[2], utf_string_buf, INFO_POPUP_LEN);

  #if FYSTLCD_PAGE_EXIST(INFO_ERROR)    
    lcd_set_page(FTPAGE(INFO_ERROR));
    //SERIAL_ECHOLN("lcd_pop_page");
  #endif
}

/**
 * Reset the status message
 */
void lcd_reset_status() {
  static const char paused[] PROGMEM = MSG_PRINT_PAUSED;
  static const char printing[] PROGMEM = MSG_PRINTING;
  static const char welcome[] PROGMEM = WELCOME_MSG;
  const char *msg;
  if (print_job_timer.isPaused())
    msg = paused;
  #if ENABLED(SDSUPPORT)
    else if (card.sdprinting)
      return lcd_setstatus(card.longest_filename(), true);
  #endif
  else if (print_job_timer.isRunning())
    msg = printing;
  else
    msg = welcome;

  lcd_setstatusPGM(msg, -1);
}

/**
 * set status message
 * draw the kill screen
 *
 */
void kill_screen(const char* lcd_msg) {
  lcd_setalertstatusPGM(lcd_msg); 
  lcd_kill_screen();       
}

void lcd_boot_music() {
  touch_lcd::ftPlayMusic(0x00, 0x0a, 0xFF);
  delay(100);
}

void lcd_shutDown() {
  touch_lcd::ftPlayMusic(0x05, 0x02, 0xFF);
  lcd_set_page(0x00);
}

void lcd_set_return_page_print() { 
	#if FYSTLCD_PAGE_EXIST(PRINT)
		lcd_set_return_page(FTPAGE(PRINT));
	#endif
}

void lcd_set_page_print() { 
	#if FYSTLCD_PAGE_EXIST(PRINT)
		lcd_set_page(FTPAGE(PRINT));
	#endif
}

void lcd_set_page_main() { 
	#if FYSTLCD_PAGE_EXIST(MAIN)
		lcd_set_page(FTPAGE(MAIN));
	#endif
}

  
#endif //TOUCH_LCD

