//! @file
//! @date Jun 10, 2019
//! @author Marek Bel

#ifndef FIRMWARE_FIRST_LAY_CAL_H_
#define FIRMWARE_FIRST_LAY_CAL_H_

#include <stdint.h>

void lay1cal_init();
bool lay1cal_preheat();
bool lay1cal_preheat_f(uint8_t filament);
void lay1cal_load_filament(char *cmd_buffer, uint8_t filament);
void lay1cal_intro_line();
bool lay1cal_before_meander();
bool lay1cal_meander(char *cmd_buffer);
bool lay1cal_square(char *cmd_buffer, uint8_t i);
bool lay1cal_square_loop(char *cmd_buffer, uint8_t i1, uint8_t i2);
bool lay1cal_end();


#endif /* FIRMWARE_FIRST_LAY_CAL_H_ */
