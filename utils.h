
#ifndef _UTILS_H_
#define _UTILS_H_
#include "lcd.h"

void cheapitoa (int16_t val, char* buff) ;
void convert_print_value(uint8_t posx, uint8_t posy, float value, char* displ_buff);
void convert_print_value_mult_1000(uint8_t posx, uint8_t posy, float value, char* displ_buff);
#define circular_inc(index, BUFF_SIZE) (index<BUFF_SIZE-1)?(index++):(index=0)
#define circular_dec(index, BUFF_SIZE) (index>0)?(index--):(index=0)

#endif