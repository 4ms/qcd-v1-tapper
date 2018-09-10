#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
/*
  FUSES =
    {
        .low = 0xe2,
		.high = 0xd7
    };
*/

/***********************
 *  COMPILE SETTINGS   *
 ***********************/

/*
DEBUG
*/
//#define DEBUG



extern uint32_t udiv32(uint32_t divisor);
volatile uint32_t tapouttmr;
volatile uint32_t tapintmr;


/************************
 * Mnuemonics		*
 ************************/




/********************
 * GLOBAL VARIABLES *
 ********************/

#define HOLDTIMECLEAR 800000

/*******************
 * PIN DEFINITIONS *
 *******************/

#ifdef DEBUG
#define DEBUG_pin PB0
#define DEBUG_init DDRB |= (1<<DEBUG_pin)
#define DEBUGFLIP PORTB ^= (1<<DEBUG_pin)
#define DEBUGHIGH PORTB |= (1<<DEBUG_pin)
#define DEBUGLOW PORTB &= ~(1<<DEBUG_pin)
#endif

#define LEDOUT_pin PB0
#define LEDOUT_init DDRB |= (1 << LEDOUT_pin)
#define LEDOUT_ON PORTB |= (1 << LEDOUT_pin)
#define LEDOUT_OFF PORTB &= ~(1 << LEDOUT_pin)

#define TAP_pin PB3
#define TAP_init DDRB &= ~(1<<TAP_pin); PORTB |= (1<<TAP_pin)
#define TAPIN (!(PINB & (1<<TAP_pin)))

#define TAPOUT_pin PB4
#define TAPOUT_init DDRB |= (1 << TAPOUT_pin)
#define TAPOUT_ON PORTB |= (1 << TAPOUT_pin)
#define TAPOUT_OFF PORTB &= ~(1 << TAPOUT_pin)

volatile char timer_overflowed=0;


SIGNAL (TIMER0_OVF_vect){
	tapintmr++;
	tapouttmr++;
}


uint32_t get_tapintmr(void){
	uint32_t result;
	cli();
	result = (tapintmr << 8) | TCNT0;
	sei();
	return result;
}
uint32_t get_tapouttmr(void){
	uint32_t result;
	cli();
	result = (tapouttmr << 8) | TCNT0;
	sei();
	return result;
}


void reset_tapouttmr(void){
	cli();
	tapouttmr=0;
	sei();
}
void reset_tapintmr(void){
	cli();
	tapintmr=0;
	sei();
}



void inittimer(void){
	cli();
	//Normal mode, TOP at 0xFF, OC0A and OC0B disconnected, Prescale @ FCK/8
	TCCR0A=(0<<WGM01) | (0<<WGM00) ;
	TCCR0B= (0<<WGM02) | (0<<CS00) | (1<<CS01) | (0<<CS02);

	TCNT0=0;

	TIMSK |= (1<<TOIE0); 
	tapouttmr=0;
	tapintmr=0;
						// Enable timer overflow interrupt
	sei();
}


void init_pins(void){
	TAP_init;
	TAPOUT_init;
	LEDOUT_init;
#ifdef DEBUG_init
	DEBUG_init;
#endif
}




/***************************************************
 *             MAIN() FUNCTION                     *
 *                                                 *
 ***************************************************/


int main(void){

	uint32_t tapout_clk_time=0;
	uint32_t now=0;
	uint32_t t=0;


	char tapin_down=0, tapin_up=0;

	/** Initialize **/


	inittimer();
	init_pins();

	_delay_ms(5);

	TAPOUT_OFF;
	/** Main loop **/
	while(1){

		if (TAPIN){
			tapin_down=0;
			now=get_tapintmr();

			if (!(tapin_up)){
				tapin_up=1;

				tapout_clk_time=now;
				reset_tapintmr();
				reset_tapouttmr();

				LEDOUT_ON;
				TAPOUT_ON;
			} else {
				if (now > HOLDTIMECLEAR){ //button has been down for more than 2 seconds
					tapout_clk_time=0;
					reset_tapouttmr();
					TAPOUT_OFF;
					LEDOUT_OFF;
//				} else {
//					TAPOUT_ON;
				}
			}
		} else {
			tapin_up=0;
			if (!(tapin_down)){
				TAPOUT_OFF;
				tapin_down=1;
			}
		}


		if (tapout_clk_time){

			now=get_tapouttmr();

			/* Main Tap Out jack/LED */
			if (now>=(tapout_clk_time>>1)){
				TAPOUT_OFF;
				LEDOUT_OFF;
			}
			if (now>tapout_clk_time){
				t=(now-tapout_clk_time)>>8;
				cli();
				tapouttmr=t;
				sei();

				TAPOUT_ON;
				LEDOUT_ON;
			}

		} else {
			TAPOUT_OFF;
			LEDOUT_OFF; 
		}



	} //main loop

} //void main()





