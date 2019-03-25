#include "utils.h"
#include <string.h>
#include <stdlib.h>

void cheapitoa (int16_t val, char* buff) 
//quick conversion of a max. 10 bit long uint to a 10-base string of fixed length
//optimized for 8-bit processor
//max value = 2^10 - 1  =1023
{
	if (val < 0){buff[0] = '-';
		val = -val;
	}else{buff[0] = ' ';};
	
	if (val < 1000){
		buff[1] = '0';
	}else{
		buff[1] = '1';
		val = val - 1000;
	}
	if (val >= 900){buff[2]='9';}
	else if (val >= 800){buff[2]='8';}
	else if (val >= 700){buff[2]='7';}
	else if (val >= 600){buff[2]='6';}
	else if (val >= 500){buff[2]='5';}
	else if (val >= 400){buff[2]='4';}
	else if (val >= 300){buff[2]='3';}
	else if (val >= 200){buff[2]='2';}
	else if (val >= 100){buff[2]='1';}
	else 				{buff[2]='0';}
	//val = val - (((uint8_t) buff[1])-0x30)*100;//substract second digit
	val = val%100;
	buff[3] = 0x30 + val/10;
	buff[4] = 0x30 + val%10;
	buff[5] = 0;
	
}


void convert_print_value_mult_1000(uint8_t posx, uint8_t posy, float value, char* displ_buff){
		itoa( value, displ_buff, 10);
		//trick the div by 1000 by placing a comma 
		uint8_t len = strlen(displ_buff);
		int8_t commapos = len - 3; //3 decimal places, we divide the value by 1000
		if (commapos < 0) commapos = 0;
		//if len < 4, comma will be at first place
		//if len == 4, comma will be after the first two digits
		lcd_gotoxy(posx,posy); 
		for (uint8_t i=0; i < len; i++){
			if (i==commapos) lcd_putc('.'); 
			lcd_putc(displ_buff[i]);
		}

		lcd_putc(' ');
		lcd_putc(' ');
		//lcd_puts(displ_buff);

}
 
void convert_print_value(uint8_t posx, uint8_t posy, float value, char* displ_buff){
		itoa( value, displ_buff, 10);
		//trick the div by 1000 by placing a comma 
		uint8_t len = strlen(displ_buff);
		//copy the last digits to another buffer for placing them on the display after the comma
        char last_two_digits[2]; 

		if (len >=2 ){ // if there are decimal digits
			last_two_digits[0] = displ_buff[len-2];
			last_two_digits[1] = displ_buff[len-1];
			displ_buff[len-2] = '.';
			displ_buff[len-1] = 0; //end string
		}else if (len ==2){
			displ_buff[0] = '.';
			last_two_digits[0] = displ_buff[len-2];
			last_two_digits[1] = displ_buff[len-1];		
			displ_buff[1] = 0; //end string
		}else{ //len ==1
			displ_buff[0] = '.';
			last_two_digits[0] = displ_buff[len-1];
			last_two_digits[1] = ' ';		
			displ_buff[1] = 0; //end string
		};
		
		lcd_gotoxy(posx,posy); 
		lcd_puts(displ_buff);
		
		//now write the last digits
		lcd_putc(last_two_digits[0]);
        lcd_putc(last_two_digits[1]);
		lcd_putc(' ');
		lcd_putc(' ');
}
