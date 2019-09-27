//! @file
//! @date Jun 10, 2019
//! @author Marek Bel , GiliFuuu
//! @brief First layer (Z offset) calibration

#include "MarlinConfig.h"
#include "first_lay_cal.h"
#include <avr/pgmspace.h>
#include "Marlin.h"

#ifdef FIRST_LAYER_CAL

extern uint8_t commands_in_queue;

int8_t enqueue_layre1cal_commands_P(const char * const pgcode) {
  if(commands_in_queue < BUFSIZE) {
    enqueue_and_echo_commands_P(pgcode);
    return 0;
  }
  else {
    return -1;
  }
}


//! @brief Preheat
bool lay1cal_preheat() {
  static uint8_t i = 0;

  static const char cmd_preheat_0[] PROGMEM = "M107";
  static const char cmd_preheat_1[] PROGMEM = "M104 S" STRINGIFY(PREHEAT_1_TEMP_HOTEND);
  static const char cmd_preheat_2[] PROGMEM = "M140 S" STRINGIFY(PREHEAT_1_TEMP_BED);
  static const char cmd_preheat_3[] PROGMEM = "M190 S" STRINGIFY(PREHEAT_1_TEMP_BED);
  static const char cmd_preheat_4[] PROGMEM = "M109 S" STRINGIFY(PREHEAT_1_TEMP_HOTEND);
  static const char cmd_preheat_5[] PROGMEM = "G28";
  static const char cmd_preheat_6[] PROGMEM = "G92 E0.0";

  static const char * const preheat_cmd[] PROGMEM = {
    cmd_preheat_0,
    cmd_preheat_1,
    cmd_preheat_2,
    cmd_preheat_3,
    cmd_preheat_4,
    cmd_preheat_5, //call MSG_M117_V2_CALIBRATION before
    cmd_preheat_6,
  };

  for (; i < COUNT(preheat_cmd); ++i) {
    if (5 == i) { 
      if(0 > enqueue_layre1cal_commands_P(PSTR("M117 First layer cal."))) {
        return false;
      }
    }
    if(0 > enqueue_layre1cal_commands_P(static_cast<char*>(pgm_read_ptr(&preheat_cmd[i])))) {
      return false;
    }
  }

  i = 0;
  return true;
}

//! @brief Load filament
//! @param cmd_buffer character buffer needed to format gcodes
//! @param filament filament to use (applies for MMU only)
void lay1cal_load_filament(char *cmd_buffer, uint8_t filament)
{
  static uint8_t i = 0;
/*
  if (mmu_enabled)
  {
      enqueue_and_echo_commands_P(PSTR("M83"));
      enqueue_and_echo_commands_P(PSTR("G1 Y-3.0 F1000.0"));
      enqueue_and_echo_commands_P(PSTR("G1 Z0.4 F1000.0"));
      sprintf_P(cmd_buffer, PSTR("T%d"), filament);
      enqueue_and_echo_commands_P(cmd_buffer);
  }
  */
  i = 0;
}

//! @brief Print intro line
void lay1cal_intro_line()
{
  static uint8_t i = 0;
  /*
  static const char cmd_intro_mmu_3[] PROGMEM = "G1 X55.0 E32.0 F1073.0";
  static const char cmd_intro_mmu_4[] PROGMEM = "G1 X5.0 E32.0 F1800.0";
  static const char cmd_intro_mmu_5[] PROGMEM = "G1 X55.0 E8.0 F2000.0";
  static const char cmd_intro_mmu_6[] PROGMEM = "G1 Z0.3 F1000.0";
  static const char cmd_intro_mmu_7[] PROGMEM = "G92 E0.0";
  static const char cmd_intro_mmu_8[] PROGMEM = "G1 X240.0 E25.0  F2200.0";
  static const char cmd_intro_mmu_9[] PROGMEM = "G1 Y-2.0 F1000.0";
  static const char cmd_intro_mmu_10[] PROGMEM = "G1 X55.0 E25 F1400.0";
  static const char cmd_intro_mmu_11[] PROGMEM = "G1 Z0.20 F1000.0";
  static const char cmd_intro_mmu_12[] PROGMEM = "G1 X5.0 E4.0 F1000.0";

  static const char * const intro_mmu_cmd[] PROGMEM =
  {
    cmd_intro_mmu_3,
    cmd_intro_mmu_4,
    cmd_intro_mmu_5,
    cmd_intro_mmu_6,
    cmd_intro_mmu_7,
    cmd_intro_mmu_8,
    cmd_intro_mmu_9,
    cmd_intro_mmu_10,
    cmd_intro_mmu_11,
    cmd_intro_mmu_12,
  };

  if (mmu_enabled)
  {
      for (uint8_t i = 0; i < COUNT(intro_mmu_cmd); ++i) {
          enqueue_and_echo_commands_P(static_cast<char*>(pgm_read_ptr(&intro_mmu_cmd[i])));
      }
  }
  else
  {*/
    enqueue_and_echo_commands_P(PSTR("G1 X60.0 E9.0 F1000.0"));
    enqueue_and_echo_commands_P(PSTR("G1 X100.0 E12.5 F1000.0"));
//}

  i = 0;

}

//! @brief Setup for printing meander
bool lay1cal_before_meander() {
  static uint8_t i = 0;
  
  static const char cmd_pre_meander_0[] PROGMEM = "G92 E0.0";
  static const char cmd_pre_meander_1[] PROGMEM = "G21"; //set units to millimeters TODO unsupported command
  static const char cmd_pre_meander_2[] PROGMEM = "G90"; //use absolute coordinates
  static const char cmd_pre_meander_3[] PROGMEM = "M83"; //use relative distances for extrusion TODO: duplicate
  static const char cmd_pre_meander_4[] PROGMEM = "G1 E-1.50000 F2100.00000";
  static const char cmd_pre_meander_5[] PROGMEM = "G1 Z5 F7200.000";
  static const char cmd_pre_meander_6[] PROGMEM = "M204 S1000"; //set acceleration
  static const char cmd_pre_meander_7[] PROGMEM = "G1 F4000";

  static const char * const cmd_pre_meander[] PROGMEM = {
    cmd_pre_meander_0,
    cmd_pre_meander_1,
    cmd_pre_meander_2,
    cmd_pre_meander_3,
    cmd_pre_meander_4,
    cmd_pre_meander_5,
    cmd_pre_meander_6,
    cmd_pre_meander_7,
  };

  for (; i < COUNT(cmd_pre_meander); ++i) {
    if(0 > enqueue_layre1cal_commands_P(static_cast<char*>(pgm_read_ptr(&cmd_pre_meander[i])))) {
      return false;
    }    
  }

  i = 0;
  return true;
}


//! @brief Count extrude length
//!
//! @param layer_height layer height in mm
//! @param extrusion_width extrusion width in mm
//! @param extrusion_length extrusion length in mm
//! @return filament length in mm which needs to be extruded to form line
static constexpr float count_e(float layer_height, float extrusion_width, float extrusion_length) {
  return (extrusion_length * layer_height * extrusion_width / (M_PI * pow(1.75, 2) / 4));
}

static const float width = 0.4; //!< line width
static const float length = 20 - width; //!< line length
static const float height = 0.2; //!< layer height TODO This is wrong, as current Z height is 0.15 mm
static const float extr = count_e(height, width, length); //!< E axis movement needed to print line

//! @brief Print meander
//! @param cmd_buffer character buffer needed to format gcodes
bool lay1cal_meander(char *cmd_buffer) {
  static uint8_t i = 0;
  char str_1[16];

  static const char cmd_meander_0[] PROGMEM = "G1 X50 Y155";
  static const char cmd_meander_1[] PROGMEM = "G1 Z0.150 F7200.000";
  static const char cmd_meander_2[] PROGMEM = "G1 F1080";
  static const char cmd_meander_3[] PROGMEM = "G1 X75 Y155 E2.5";
  static const char cmd_meander_4[] PROGMEM = "G1 X100 Y155 E2";
  static const char cmd_meander_5[] PROGMEM = "G1 X200 Y155 E2.62773";
  static const char cmd_meander_6[] PROGMEM = "G1 X200 Y135 E0.66174";
  static const char cmd_meander_7[] PROGMEM = "G1 X50 Y135 E3.62773";
  static const char cmd_meander_8[] PROGMEM = "G1 X50 Y115 E0.49386";
  static const char cmd_meander_9[] PROGMEM = "G1 X200 Y115 E3.62773";
  static const char cmd_meander_10[] PROGMEM = "G1 X200 Y95 E0.49386";
  static const char cmd_meander_11[] PROGMEM = "G1 X50 Y95 E3.62773";
  static const char cmd_meander_12[] PROGMEM = "G1 X50 Y75 E0.49386";
  static const char cmd_meander_13[] PROGMEM = "G1 X200 Y75 E3.62773";
  static const char cmd_meander_14[] PROGMEM = "G1 X200 Y55 E0.49386";
  static const char cmd_meander_15[] PROGMEM = "G1 X50 Y55 E3.62773";

  static const char * const cmd_meander[] PROGMEM = {
    cmd_meander_0,
    cmd_meander_1,
    cmd_meander_2,
    cmd_meander_3,
    cmd_meander_4,
    cmd_meander_5,
    cmd_meander_6,
    cmd_meander_7,
    cmd_meander_8,
    cmd_meander_9,
    cmd_meander_10,
    cmd_meander_11,
    cmd_meander_12,
    cmd_meander_13,
    cmd_meander_14,
    cmd_meander_15,
  };

  for (; i < COUNT(cmd_meander); ++i) {
    if(0 > enqueue_layre1cal_commands_P(static_cast<char*>(pgm_read_ptr(&cmd_meander[i])))) {
      return false;
    }
  }

  dtostrf(extr, 2, 3, str_1);
  sprintf_P(cmd_buffer, PSTR("G1 X50 Y35 E%s"), str_1);
  if(!enqueue_and_echo_command(cmd_buffer)) {
    return false;
  }

  i = 0;
  return true;
}

//! @brief Print square
//!
//! This function needs to be called 16 times for i from 0 to 15.
//!
//! @param cmd_buffer character buffer needed to format gcodes
//! @param i iteration
bool lay1cal_square(char *cmd_buffer, uint8_t i) {
  static uint8_t index = 0;
  char str_1[16], str_e[16],str_es[16];;

  const float extr_short_segment = count_e(height, width, width);

  static const char fmt1[] PROGMEM = "G1 X%d Y%s E%s";
  static const char fmt2[] PROGMEM = "G1 Y%s E%s";

  dtostrf(extr, 2, 3, str_e);
  dtostrf(extr_short_segment, 2, 3, str_es);
  
  if(index == 0) {
    dtostrf((35 - i*width * 2), 2, 2, str_1);
    sprintf_P(cmd_buffer, fmt1, 70, str_1, str_e);
    if(!enqueue_and_echo_command(cmd_buffer)) {
      return false;
    }
    index++;
  }

  if(index == 1) {
    dtostrf((35 - (2 * i + 1)*width), 2, 2, str_1);
    sprintf_P(cmd_buffer, fmt2, str_1, str_es);
    if(!enqueue_and_echo_command(cmd_buffer)) {
      return false;
    }
    index++;
  }

  if(index == 2) {
    dtostrf((35 - (2 * i + 1)*width), 2, 2, str_1);
    sprintf_P(cmd_buffer, fmt1, 50, str_1, str_e);
    if(!enqueue_and_echo_command(cmd_buffer)) {
      return false;
    }
    index++;
  }

  if(index == 3) {
    dtostrf((35 - (i + 1)*width * 2), 2, 2, str_1);
    sprintf_P(cmd_buffer, fmt2, str_1, str_es);
    if(!enqueue_and_echo_command(cmd_buffer)) {
      return false;
    }
    index++;
  }

  index = 0;
  return true;
}

bool lay1cal_square_loop(char *cmd_buffer, uint8_t i1, uint8_t i2) {
  static uint8_t i = i1;
  for (; i < i2; i++) {
    if(!lay1cal_square(cmd_buffer, i)) {
      return false;
    }
  }

  i = 0;
  return true;
}

bool lay1cal_end() {
  static uint8_t i = 0;

  static const char cmd_end_0[] PROGMEM = "M107";
  static const char cmd_end_1[] PROGMEM = "G1 E-0.07500 F2100.00000";
  static const char cmd_end_2[] PROGMEM = "M104 S0";
  static const char cmd_end_3[] PROGMEM = "M140 S0";
  static const char cmd_end_4[] PROGMEM = "G1 Z10 F1300.000";
  static const char cmd_end_5[] PROGMEM = "G1 X10 Y180 F4000";
  static const char cmd_end_6[] PROGMEM = "M84";
  
  static const char * const cmd_end[] PROGMEM = {
    cmd_end_0,
    cmd_end_1,
    cmd_end_2,
    cmd_end_3,
    cmd_end_4,
    cmd_end_5,
    cmd_end_6,
  };

  for (; i < COUNT(cmd_end); ++i) {
    /*
    if (5 == i && mmu_enabled) { 
      if(0 > enqueue_layre1cal_commands_P(PSTR("M702 C"))) {
        return false;
      }
    }
    */
    if(0 > enqueue_layre1cal_commands_P(static_cast<char*>(pgm_read_ptr(&cmd_end[i])))) {
      return false;
    }
  }

  i = 0;
  return true;
}


#endif
