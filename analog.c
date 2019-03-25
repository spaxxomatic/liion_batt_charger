
#include "analog.h"


volatile ADC_VAL_STRUCT adc_val_arr[] = {
	{0,0,0},  //voltage 
	{1,0,0},  //current
	{2,0,0}	   //discharge_curr
	};

#define ANALOG_OFF ADCSRA=0
#define ANALOG_ON  ADCSRA=(1<<ADEN)|(1<<ADSC)|(1<<ADIE)|(1<<ADFR)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0)	

void adc_init(void)
{
//select reference voltage to internal 2,56 V reference
ADMUX = (1<<REFS1)|(1<<REFS0);
//Free Running Mode, Division Factor 128, Interrupt on
//ADCSRA |= (1<<ADSC)|(1<<ADIE)|(1<<ADFR)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);
ANALOG_ON;
}

volatile uint8_t cnt;

ISR (ADC_vect)
{
    ANALOG_OFF; //ADC OFF
	ADC_VAL_STRUCT* thisMeas = &adc_val_arr[cnt];
	
	thisMeas->accumulator += ADC; //collect in the accumulator
	
	if (thisMeas->counter%32 == 0){
		thisMeas->value = (thisMeas->accumulator)>>5 ;	
		//simply shift left the latest 5 bits (corresponds to 32 samples). This is oversampling, we're keeping only the 10 bits, but have removed lot of noise
		thisMeas->accumulator = 0;
	}
	thisMeas->counter += 1;	
	
	cnt++;		
	if (cnt >= NO_OF_ANALOG_INPUTS) cnt = 0;
		
    ADMUX = ((1<<REFS1) | (1<<REFS0)) | adc_val_arr[cnt].channel;
	
	ANALOG_ON;

}

