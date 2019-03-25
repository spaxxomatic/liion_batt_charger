
#ifndef _ANALOG_H_
	#define _ANALOG_H_	

	#include <avr/io.h>
	#include <avr/interrupt.h>
		
	//will contain the sum of the latest ACCUMULATE_ADC_SAMPLES adc's. No overflow will occur, since adc is max 10 bits
	
	void adc_init(void);
	
	typedef struct {
		uint8_t channel ;
		uint16_t accumulator ;
		uint16_t value ;
		uint8_t counter ;
		
	} ADC_VAL_STRUCT;

	//measured voltage for max ADC output 
	#define V_REF = 2.68 
	
	extern volatile ADC_VAL_STRUCT adc_val_arr[];
	
	//volatile uint16_t current_value = 0;
	//volatile uint16_t voltage_value = 0;
	//volatile uint16_t discharge_curr_value = 0;
	
	#define NO_OF_ANALOG_INPUTS (sizeof adc_val_arr/sizeof adc_val_arr[0])

#endif //_ADC_H_






