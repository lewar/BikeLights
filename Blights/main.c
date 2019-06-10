/*
 * Blights.c
 *
 * Created: 03/05/2019 21:58:59
 * Author : Krzysztof
 * Description : Simple program to display different LED's animations.
 */ 

#include <avr/io.h>
#define F_CPU 8000000
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

#define LED_L_RED (1<<PB6) //outputs for different color RGB led's
#define LED_L_GREEN (1<<PB7)
#define LED_L_BLUE (1<<PB0)

#define LED_R_RED (1<<PB3)
#define LED_R_GREEN (1<<PB2)
#define LED_R_BLUE (1<<PB1)

#define KEY_PIN (1<<PD4) //set switch to port PD4
#define KEY_PRESSED (!(PIND & KEY_PIN)) //to check is key is pressed

//Function declaration
void hwTimer2Init(void);
void key_press();
void displayEffect(uint8_t *mode, uint8_t *start);

//Variables
uint8_t modeMax=5;
volatile uint8_t timer1;
uint8_t mode, clock;
static uint8_t idx=1;
uint8_t start;

//Arrays with effects, value on position 0 indicate how many steps to display, last value is timer setting
const uint8_t eff0[] PROGMEM={1,0,10};
const uint8_t eff1[] PROGMEM={12,64,0,64,0,64,0,2,0,2,0,2,0,9};
const uint8_t eff2[] PROGMEM={4,64,0,2,0,20};
const uint8_t eff3[] PROGMEM={12,1,0,1,0,1,0,2,0,2,0,2,0,9};
const uint8_t eff4[] PROGMEM={2,72,0,20};
const uint8_t eff5[] PROGMEM={1,72,20};

int main(void)
{
	
	DDRB |= LED_L_RED | LED_L_GREEN | LED_L_BLUE | LED_R_RED | LED_R_GREEN | LED_R_BLUE; //Set PORTB as output
	PORTB &= ~(LED_L_RED | LED_L_GREEN | LED_L_BLUE | LED_R_RED | LED_R_GREEN | LED_R_BLUE); //Set 0 on all outputs
	
	DDRD &= ~(KEY_PIN); //Set PD4 as input
	PORTD |= KEY_PIN; //enable pull up resistors by setting PORTD
	
	hwTimer2Init();

	sei(); //global permission for interrupts
	
    while (1) 
    {
		
		key_press();
		
		if(!timer1){
			
			displayEffect(&mode, &start);
			
		}

		
    }
	return 0;
}

void displayEffect(uint8_t *mode, uint8_t *start){
	const uint8_t *tab;
	uint8_t all;
	
	if(*mode==0) tab=eff0;
	else if(*mode==1) tab=eff1;
	else if(*mode==2) tab=eff2;
	else if(*mode==3) tab=eff3;
	else if(*mode==4) tab=eff4;
	else if(*mode==5) tab=eff5;
	all=pgm_read_byte(&tab[0]);
	
	if(*start){
		PORTB=0;
		idx=1;
		*start=0;
	}
	
	PORTB=pgm_read_byte(&tab[idx++]);
	if(idx>all) idx=1;
	
	timer1=pgm_read_byte(&tab[all+1]);
}

//Configuration of hardware timer/counter 2
void hwTimer2Init(void){
	TCCR2 |=(1<<WGM21); //set timer mode CTC
	TCCR2 |=(1<<CS20) | (1<<CS21) | (1<<CS22); //set prescaler to 1024
	OCR2  = F_CPU/1024ul/156ul; //compare register set to ~50Hz every 20ms F_CPU 8000000 / 1024 / 156 = 50.08
	TIMSK |=(1<<OCF2); //compareMatch interrupt enabled
}

void key_press(void){
	uint8_t key_press=(PIND & KEY_PIN);
	if(!clock  && !key_press){ 
		clock=1;
		mode++;
		start=1;
		if(mode>modeMax) {
			mode=0;
			idx=1;
		}
	}
	else if( clock && key_press ) {//Software delay to avoid debouncing when switch is released
		clock++;
	}
}

ISR(TIMER2_COMP_vect){
	
	uint8_t cnt;
	cnt=timer1;
	if(cnt){
		timer1=--cnt;}
}