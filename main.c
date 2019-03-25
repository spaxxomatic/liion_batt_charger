#include <stdlib.h>
#include <avr/io.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <string.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>

#include "lcd.h"
#include "utils.h"

#include "analog.h"

#define BAUDRATE 9600
#include "usart.h"

#define NELEMS(x)  (sizeof(x) / sizeof((x)[0]))

#define false 0
#define true 1
#define bool uint8_t

char* batMenuName = "Battery type:";
char* batMenuItems[7] = {"Auto", "1S", "2S", "3S", "4S", "5S"};

int8_t encoder;  
uint8_t mode;


#define CURRENT_AMP_SLOPE 0.500 //0.500 V/A
#define AD_RESOLUTION 2.441 //mv/step for internal reference and 10bit ad conversion

#define AD_RESOLUTION_VOLTAGEINPUT 24.41 // mv/step this is different from the others because we have a potentiometer in front of the input

#define AD_RESOLUTION_CURRENTINPUT AD_RESOLUTION*CURRENT_AMP_SLOPE // mA/step

#warning "AD_RESOLUTION_CURRENTINPUT is " AD_RESOLUTION_CURRENTINPUT

//#define MAX_ADVAL_CHARGE_CURRENT 3000*AD_RESOLUTION_CURRENTINPUT
//#define MIN_ADVAL_CHARGE_CURRENT 100*AD_RESOLUTION_CURRENTINPUT

#define MAX_ADVAL_CHARGE_CURRENT 0x03FF //10 bit pwm, value is relative to the setting of the current source
#define MIN_ADVAL_CHARGE_CURRENT 0x000F

#define MAX_ADVAL_CELL_VOLTAGE 4400/AD_RESOLUTION_VOLTAGEINPUT
#define MIN_ADVAL_CELL_VOLTAGE 3800/AD_RESOLUTION_VOLTAGEINPUT
#define MAX_OFFSET_VAL AD_RESOLUTION*10

#define MIN_CELL_VOLTAGE 3.00

#define FINAL_CHARGE_CURRENT AD_RESOLUTION_CURRENTINPUT*100 //100mA

#define MODE_BUTTON PC5
#define OK_BUTTON PC4

#define MODE_RELAY PC3

#define ROTARY_IN1 PB6
#define ROTARY_IN2 PB7
#define PHASE_A     (PINB & 1<<ROTARY_IN1)     // an Pinbelegung anpassen
#define PHASE_B     (PINB & 1<<ROTARY_IN2)     // an Pinbelegung anpassen

#define CHARGE_RELAIS_PIN PB3
#define DISCHARGE_RELAIS_PIN PB5

#define SWITCH_CHARGING_ON PORTB |= (1<< CHARGE_RELAIS_PIN)
#define SWITCH_CHARGING_OFF PORTB &=~ (1<< CHARGE_RELAIS_PIN)

#define SWITCH_DISCHARGING_ON PORTB |= (1<< DISCHARGE_RELAIS_PIN)
#define SWITCH_DISCHARGING_OFF PORTB &=~ (1<< DISCHARGE_RELAIS_PIN)


struct s_settings {
	uint8_t batt_type;
	uint16_t target_charging_current;
	uint16_t target_cell_voltage;
	uint16_t zero_offset_val_voltage;
	uint16_t zero_offset_val_current;
} ;

//s_settings EEMEM ee_prev_settings EEMMEM = {5,MIN_ADVAL_CHARGE_CURRENT,MIN_ADVAL_CELL_VOLTAGE,0,0};

struct s_settings ee_settings EEMEM = {.batt_type=5, .target_charging_current=MIN_ADVAL_CHARGE_CURRENT, .target_cell_voltage=MIN_ADVAL_CELL_VOLTAGE
, .zero_offset_val_voltage=0, .zero_offset_val_current=0};
struct s_settings settings;

char target_voltage_str[8] ;

volatile int8_t enc_delta;              // Drehgeberbewegung zwischen zwei Auslesungen im Hauptprogramm
volatile uint8_t mode_pressed;
volatile uint8_t ok_pressed;

// Dekodertabelle für wackeligen Rastpunkt
// halbe Auflösung
const int8_t table[16] PROGMEM = {0,0,-1,0,0,0,0,1,1,0,0,0,0,-1,0,0}; 

#define DEF_BOUNCE_CNT 45
ISR( TIMER1_COMPB_vect )             // 1ms fuer manuelle Eingabe
{
    //read the rotary encoder
	static int8_t last=0;           // alten Wert speichern
	static int8_t bounce_counter;           // debouncing for buttons
    last = (last << 2)  & 0x0F;
    if (PHASE_A) last |=2;
    if (PHASE_B) last |=1;
    enc_delta += pgm_read_byte(&table[last]);
	
	//read and debounce the buttons 
	
	if(bounce_counter== 0 && mode_pressed == 0 && bit_is_clear(PINC, MODE_BUTTON)){ //mode button event processed, and new press detected
	
		mode_pressed = 1;
		bounce_counter = DEF_BOUNCE_CNT;
	}
	if(bounce_counter== 0 && ok_pressed == 0 && bit_is_clear(PINC, OK_BUTTON)){ //ok button event processed, and new press detected
		ok_pressed = 1;	
		bounce_counter = DEF_BOUNCE_CNT;
	}
	if (bounce_counter >0) bounce_counter --;
}

/*
#define T1A_DIVIDER 100
volatile uint8_t t1_div = 0;

ISR(TIMER1_COMPA_vect) //1000 msec timer 
{
	if (t1_div>T1A_DIVIDER/2) {ticker = '*'; }
	else {ticker = ' ';};
	t1_div += 1;
    if (t1_div>T1A_DIVIDER) {ticker=0;}
}
*/

void read_encoder( void )         // Encoder auslesen
{
  // atomarer Variablenzugriff  
  //cli();
  encoder = enc_delta;
  enc_delta = 0;
  //sei();
}

char buff[20];

volatile char adv_cnt = 0;
uint16_t expected_charge_current=0;


bool restart (void){
	//reset

	asm("ldi r30,0"); asm("ldi r31,0"); asm("ijmp");
	return true; //we never reach this
};


bool save_settings(void){
    eeprom_busy_wait();
	eeprom_update_block (&settings, &ee_settings, sizeof(settings));
	eeprom_busy_wait();
    lcd_clrscr();
    lcd_gotoxy(4,0); 
    lcd_puts("Saved. Will restart");
    _delay_ms(1000);
	restart();
	return true; // will never reach here
}

void read_settings(void){
    eeprom_busy_wait();
    eeprom_read_block ((void*)&settings, &ee_settings, sizeof(settings));
}


void print_voltage_current(void){
	float voltage_value = adc_val_arr[0].value*AD_RESOLUTION_VOLTAGEINPUT ;
	float current_value = adc_val_arr[1].value*AD_RESOLUTION_CURRENTINPUT ; 
	convert_print_value_mult_1000(15,0,voltage_value, buff);
	convert_print_value_mult_1000(15,1,current_value, buff);
	lcd_gotoxy(20,0); 
	lcd_puts(" V");
	lcd_gotoxy(20,1); 
	lcd_puts(" mA");
}

#define CHARGING_ABORT 2
#define CHARGING_DONE 1
#define CHARGING_ACTIVE 0

uint8_t charge_mode ;
uint16_t target_voltage;
uint16_t min_batt_voltage;

void f_charge (void){
	lcd_gotoxy(7,0); 
	lcd_puts(batMenuItems[settings.batt_type]);
	
	lcd_gotoxy(0,1); 
	uint16_t batt_voltage = adc_val_arr[0].value;
	uint16_t charge_current = adc_val_arr[1].value;
	
	if (charge_mode == CHARGING_ABORT){
		SWITCH_CHARGING_OFF;
		lcd_puts("CHARGING ABORTED"); 	
		return;
	}
	if (charge_mode == CHARGING_DONE) {
		SWITCH_CHARGING_OFF;
		lcd_puts("CHARGING FINISHED"); 
		return;
	}else{
		SWITCH_CHARGING_ON;
	}
	
	if (batt_voltage >=  target_voltage && charge_current < FINAL_CHARGE_CURRENT){
		//disconnect charging
		SWITCH_CHARGING_OFF;
		charge_mode = CHARGING_DONE;
	}
	if (adc_val_arr[0].value < target_voltage) {
		//keep loading in constant voltage mode
		//show mode on display
		lcd_puts("CC to "); 
        lcd_puts(target_voltage_str);
	}else{			
		lcd_puts("final CV "); 
		//start reducing the charging current
		charge_mode = CHARGING_DONE;
		;
	}
	print_voltage_current();
};

bool charge_ok_btn(void){
	if (charge_mode == CHARGING_DONE){
		mode = 0;
		restart();
		return true;
	} else {
		lcd_clrscr();
		lcd_gotoxy(1,10); 
		lcd_puts("Stop charging"); 
		_delay_ms(200);
		lcd_clrscr();
		charge_mode = CHARGING_ABORT;
		return false;
	}
}

float capacity;

void f_discharge (void){
	uint16_t batt_voltage = adc_val_arr[0].value;
	uint16_t discharge_current = adc_val_arr[1].value;
	capacity += batt_voltage*discharge_current;
	lcd_gotoxy(0,1);
	convert_print_value_mult_1000(0,1,capacity, buff);
	
	//if (batt_voltage <=  min_batt_voltage ){
	if (false){
		SWITCH_DISCHARGING_OFF;
		lcd_gotoxy(10,0);
		lcd_puts("Done! "); 
		return;
	}else{
		SWITCH_DISCHARGING_ON;
	}
	
	print_voltage_current();
	
};

bool discharge_ok_btn(void){
	SWITCH_DISCHARGING_OFF;
	return true;
}

void f_wait_start (void){
	lcd_gotoxy(0,0); 
	lcd_puts("Press OK to charge"); 
};

#define NO_OF_BAT_MENU_OPTS (sizeof batMenuItems/sizeof batMenuItems[0])

void f_set_batt_type (void){
	if (encoder > 0) {
		circular_inc(settings.batt_type, NO_OF_BAT_MENU_OPTS);
	};
	if (encoder < 0) {
		circular_dec(settings.batt_type, NO_OF_BAT_MENU_OPTS);
	}
	
	lcd_gotoxy(2,1); 
	lcd_puts(batMenuItems[settings.batt_type]);
	lcd_puts("        "); //padding to clean previous print
};

#define MAX_PWM 0xFFFF
uint16_t current_pwm = MAX_PWM;

#define CURRENT_STEP 20

void f_set_curr		(void){
	
	if (encoder > 0) {
		if (settings.target_charging_current < MAX_ADVAL_CHARGE_CURRENT-CURRENT_STEP) settings.target_charging_current+=CURRENT_STEP;
	};
	if (encoder < 0) {
		if (settings.target_charging_current > CURRENT_STEP) settings.target_charging_current-=CURRENT_STEP;
	}
	OCR1A = settings.target_charging_current; //set pwm
	//convert_print_value_mult_1000(2,1, ((float) settings.target_charging_current)*AD_RESOLUTION_CURRENTINPUT);
	cheapitoa(settings.target_charging_current, buff);
	lcd_gotoxy(2,1); 
	lcd_puts(buff);
};

void f_set_target_voltage (void){
	
	//convert_print_value_mult_1000(10,0, MIN_ADVAL_CELL_VOLTAGE*AD_RESOLUTION_VOLTAGEINPUT);
	//convert_print_value_mult_1000(16,0, MAX_ADVAL_CELL_VOLTAGE*AD_RESOLUTION_VOLTAGEINPUT);
	if (encoder > 0) {
		if (settings.target_cell_voltage < MAX_ADVAL_CELL_VOLTAGE) settings.target_cell_voltage++;
	};
	if (encoder < 0) {
		if (settings.target_cell_voltage > MIN_ADVAL_CELL_VOLTAGE) settings.target_cell_voltage--;
	}
	convert_print_value_mult_1000(2,1, settings.target_cell_voltage*AD_RESOLUTION_VOLTAGEINPUT, buff);
	//cheapitoa(settings.target_cell_voltage, buff);
	//lcd_gotoxy(2,1); 
	//lcd_puts(buff);
};

void f_restart (void){
	lcd_gotoxy(0,0); 
	lcd_puts("Press OK to restart"); 
};


bool wait_start_ok(void){
	//TODO: autodetect battery
	//
	lcd_clrscr();
    lcd_gotoxy(0,0); 
    if (settings.batt_type == 0) {
		lcd_puts("Try to detect battery .. ");	
		
		if (adc_val_arr[1].value < MIN_ADVAL_CHARGE_CURRENT){
			lcd_puts("Battery not detected");	
			return false;
		}
		settings.batt_type  = 5;
	}
	target_voltage = settings.target_cell_voltage*settings.batt_type;
	
	convert_print_value_mult_1000(0,1,target_voltage*AD_RESOLUTION_VOLTAGEINPUT, target_voltage_str); //keep it as string for display
	charge_mode = CHARGING_ACTIVE;
	mode_pressed = 1; //jump to charging mode
	return true;
}
/*
bool show_done(void){
	lcd_clrscr();
    lcd_gotoxy(4,0); 
    lcd_puts("....");	
	_delay_ms(400);
	return true;
};
*/
typedef struct
{
	//uint8_t mode; 				        
	void(*fp)(void);  	// Function pointer for the handler
	char* descr;
	bool(*finally)(void);  	// Pointer to the function to be executed when ok key is pressed
	
} MODE_STRUCT;	

	
MODE_STRUCT MODES[] = 
{
	//{f_wait_start, "START CHARGING", wait_start_ok}, 
	//{f_charge, "CHARGE", charge_ok_btn}, 
	//{f_discharge, "DISCHARGE", discharge_ok_btn}, 
	{f_set_batt_type, "BATT TYPE", save_settings}, 
	{f_set_curr, "SET CHARGE CURRENT", save_settings}, 
	{f_set_target_voltage, "SET CELL VOLTAGE", save_settings}, 
	//{f_restart, "RESTART", restart_ok}, 
	//{{00},NULL} 
};

void timer_init(void){

  //timer 0 on 1 milisecond
  TCCR1B = (1<<WGM12) | (1<<CS01) | (1<<CS00);     // CTC, XTAL / 64
  OCR1B = (uint8_t)(F_CPU / 64.0 * 1e-3 - 0.5);     // 1ms
  TIMSK |= 1<<OCIE1B;
  
   //OCR1A is used to control the constant current source
   OCR1A = settings.target_charging_current; //pwm 0
   //TIMSK|=(1<<OCIE1A);  //Output compare 1A interrupt enable
  // Set 8 bit Phase correct PWM Mode for channel 1 + 2(A/B)
  
  //PWM OC1A init
  //TCCR1A &= ~(1 << WGM11);  // 8bit Phase correct CH1
  //TCCR1A |=  (1 << WGM10);  // 8bit Phase correct CH1
  TCCR1A |= (1 << WGM11);  // 10bit Phase correct CH1
  TCCR1A |=  (1 << WGM10);  // 10bit Phase correct CH1
  
// Connect output Pin channel 1
  TCCR1A |=  (1 << COM1A1);   
  TCCR1A &= ~(1 << COM1A0);
  
 
}
 
int main(void)
{	
	//usart_init(BAUDRATE); // setup the UART

    /* initialize display, cursor off */
    
	//set inputs
	PORTC |= (1 << MODE_BUTTON) ; // PULLUP für inputs
    DDRC &=~ 1 << MODE_BUTTON ; // Activate input
	PORTC |= (1 << OK_BUTTON) ; // PULLUP für inputs
    DDRC &=~ 1 << OK_BUTTON ; // Activate input
    
	PORTB |=  (1 << ROTARY_IN1) | (1 << ROTARY_IN2);
	DDRB &=~ 1 << ROTARY_IN1 ;
    DDRB &=~ 1 << ROTARY_IN2 ;    
    
    //set OC1A as output
    DDRB |= (1 << PB1) ;
	
	//relaises
	DDRB |=  (1 << CHARGE_RELAIS_PIN) | (1 << DISCHARGE_RELAIS_PIN) ; 
    
	lcd_init(LCD_DISP_ON);
	lcd_clrscr();
	lcd_gotoxy(0,1); 
	
	
	timer_init();
	
	adc_init();
	sei();
	//usart_write_str("INIT OK");
	SWITCH_CHARGING_OFF;
	SWITCH_DISCHARGING_OFF;
	//check value plausibility
	mode = 0;
	read_settings();
	target_voltage = settings.target_cell_voltage*settings.batt_type;
	min_batt_voltage = MIN_CELL_VOLTAGE*AD_RESOLUTION_VOLTAGEINPUT*settings.batt_type;
	
	for (;;) { /* loop forever */
		if (mode_pressed) //mode button pressed
		{
			if (mode < NELEMS(MODES)-1){
                mode+=1;
				lcd_clrscr();                
            }else{
				mode=0;
			}
			mode_pressed = 0;
		}
		if (ok_pressed) //ok button pressed
		{
			ok_pressed = 0;
			bool bDone = false;
			if (MODES[mode].finally != NULL){
				bDone = MODES[mode].finally();
			}
			if (bDone == true){
				_delay_ms(500);
				lcd_clrscr();
				mode = 0;
			}
			
			
		}

		//call the mode handling func
		lcd_gotoxy(0,0); 
		lcd_puts(MODES[mode].descr);
		//read encoder
		read_encoder();  
		//call handling function
		MODES[mode].fp();
        
		lcd_gotoxy(0,1); 
		char* memaddr = &settings;
		lcd_putc( (((int) memaddr)>>4) + 0x30 );
		lcd_putc( (((int) memaddr)&0x0F) + 0x30 );
		for (int i = 0; i < sizeof(settings); i++){
		
			
			lcd_putc( (*(memaddr + i) >> 4) + 0x30);
			lcd_putc( (*(memaddr + i) & 0x0F) + 0x30);
		}
		
		_delay_ms(200);
        /*lcd_gotoxy(0,1); 
		lcd_putc(ticker);
		cheapitoa(adc_val_arr[0].value, buff);
        lcd_gotoxy(3,1); 
        lcd_puts(buff);
		cheapitoa(adc_val_arr[1].value, buff);
        lcd_gotoxy(8,1); 
        lcd_puts(buff);
		cheapitoa(adc_val_arr[2].value, buff);
        lcd_gotoxy(14,1); 
        lcd_puts(buff);        
		*/
        //usart_write_str(buff);
		//usart_write_str("\n\r");
    }
}
