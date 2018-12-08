// Lab 6 - Alarm Clock FM Radio
// final_l6_.c
// Josh Griffin
// 12/7/2018

////////////////////////////////////////////////////////////
// Include all of the important libraries
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdbool.h>
#include "hd44780.h"
#include "twi_master.c"
#include <avr/pgmspace.h>
#include "lm73_functions.h"
#include <stdlib.h>
#include <string.h>
#include "uart_functions.c"
#include "l6.h"
#include "si4734.c"
////////////////////////////////////////////////////////////
// Define Constants for easy use
// number of protocols to check
#define N_Process 10

// define 1 second. 
#define ROLLOVER 59392

// First Station
// College Radio 8870
#define DFLT_STION 8870

// define interrupt update period: 1 ms, less, more etc
#define update_period 4

// Define the value used to select the 7seg digit 1,2,3,4
// (High impedance on push board)
#define dM 	0b00100000
#define d4  0b00000000
#define d3  0b00010000
#define d2  0b00110000
#define d1  0b01000000

// Define the value used to select the push-button board
// This deactivates the high impedance state.
#define pb  0b01110000

// Define the buffersize for the shift-register debouncer
#define buffersize 20

// Define 7-seg representations of 0-9 and blank.
#define zer 0b11000000
#define one 0b11111001
#define two 0b10100100
#define thr 0b10110000
#define fou 0b10011001
#define fiv 0b10010010
#define six 0b10000010
#define sev 0b11111000
#define eig 0b10000000
#define nin 0b10011000
#define aaa 0b10001000
#define bbb 0b10000011
#define ccc 0b11000110
#define ddd 0b10100001
#define eee 0b10000110
#define fff 0b10001110
#define nul 0xFF

////////////////////////////////////////////////////////////
// Initialize global variables

// Indexing variables
uint16_t j;
uint16_t i;
uint8_t r;
uint8_t k;

// Radio Stuff
uint8_t  si4734_wr_buf[9];
// uint8_t  si4734_rd_buf[9];
uint8_t  si4734_tune_status_buf[8];
uint8_t rad_on = 0;

// Initialize previous radio station
uint16_t prev_frq = DFLT_STION;

// initialize clock state variable
uint8_t clk_state[4] = {0,5,0,0};

// initialize alarm state variable
uint8_t alrm_state[4] = {0,5,0,1};

// AM/PM indicator - 0 is am, 1 is pm
bool ampm = 0;
// AM/PM indicator for alarm
bool ampm_alrm = 0;


// ':' on off state
bool colon_on = 0;

// encoder control state - exclusive
// no state 		0
// time set 		1
// alarm set 		2
// volume set 		3
// radio tune		4
uint8_t encoder_function = 0;


// other control states	

// play alarm 
bool play_alrm = 0;
// play radio 
bool play_rad = 1;

// alarm enabled
bool alrm_en = 0;
// military/std 
bool militaryTime = 1;
bool alrmmilitaryTime = 1;

// Temperature local data;
uint8_t tempData[2];
uint16_t tempHighLow;
char lcdStr[16] = "0";

// Temperature Local
char tempL[3] = {'0','0','0'};
// Temperature Remote
char tempR[3] = {'0','0','0'};

// store line state LCD
uint8_t line_1_2 = 1;

// number of temperature bytes
uint8_t byte_cnt = 0x2;

bool lcd_flip_flop = 0;

// ms and s clock. Two clocks will allow
// for precision and lack of drift.
// The clk is approximated as 32 KHz for ms
// and 32.768 KHz for seconds

// This is updated on every 32 clk cycles
uint32_t msCLK			= 0;
// This is updated on every 8×4096 = 32768
// aka, 1024 msCLK.
// uint32_t sCLK			= 0;

// Stores the previous encoder state
uint8_t prev_encoder_state = 0x00;
// Stores the current encoder state
uint8_t encoder_state = 0x00;
// Port A Buffer initialization flag
bool PA_Buffer_Initialized = 0;
// debounced portA value. Triggered once per hold.
uint8_t debounced_PA;
// Indicates what digit will be displayed (7seg)
uint8_t digit_place = 0;
// Value to be written to 7seg
uint16_t display_output = 0;

bool update_EN = 0;

bool up = 0;

// Display state variables
uint16_t dispHIGH = 1;  	// low time
uint16_t dispLOW = 2;		// high time
bool disp_state = 0;
uint16_t disp_cntr = 0;
uint16_t dimLVL = 2;

// Alarm Volume for OCR3A
uint8_t vol = 0xFF>>1;

////////////////////////////////////////////////////////////
// Functions
////////////////////////////////////////////////////////////

// flip alarm pin
void flip_alrm(){

	// Only play the alarm when radio disabled, and alarm on
	if (play_alrm && !play_rad){

		DDRD |= (1<<3);
		if (up){
			PORTD &= ~(1<<3);
			up = 1-up;
		}
		else{
			// update_clock(1);
			PORTD |= (1<<3);
			up = 1-up;
		}
	}
}

// CTC interrupt ISR. Updates ms clock value
ISR(TIMER0_COMP_vect){

	// Increment timing register
	msCLK++;
	// Toggle colon
	if (msCLK%(512 * (32/update_period)) == 0){
		colon_on = 1-colon_on;
	}

	// Rollover timing reg to 0
	if (msCLK == ROLLOVER * (32/update_period)){
		update_EN = 1;
		msCLK = 0;

	}

	// update 7seg display
	if (msCLK % 3 == 0){
		// update_or_display = 1; display
		write_7seg(1);
		// update_EN = update_EN;
	}

	// flip the alarm tone pin to generate tone
	if (msCLK % 20 == 0){
		flip_alrm();
	}

}

// Manually toggle alarm wave input
ISR(TIMER1_COMPA_vect){

	// Functional, but abandoned for superior method

	// uint8_t PD3S = PORTD & (1<<3);

	// update_clock(10);
	// if (play_alrm){
	// 	// if (up){
	// 	PORTD &= ~(1<<3);
	// 		// up = 1-up;
	// 	// }
	// 	// else{
	// 		// update_clock(1);
	// 	PORTD |= (1<<3);
	// 		// up = 1-up;
	// 	// }
	// }
}

// Setup TCNT0 and Interrupt
void init_counter(){
	// Set clock to 1 prescale, 
	TCCR0 |= (1<<CS00);
	TCCR0 &= ~((1<<CS01)|(1<<CS02));
	// CTC mode
	TCCR0 |= (1<<WGM01);
	TCCR0 &= ~(1<<WGM00);
	// Asynchonous 32 KHz clk
	ASSR |= (1<<AS0);
	//Set output compare reg.
	OCR0  = update_period;
	// OCR0  = 255;
	// Enable interrupts
	TIMSK |= (1<<OCIE0);
	sei();
}

// Setup TCNT3 for audio pwm volume
void init_tcnt3_audio_volume(){

	// Set compare to volume reg
	OCR3A = vol;

	// fast pwm
	TCCR3B |= (1<<WGM32);
	TCCR3B &= ~(1<<WGM33);
	TCCR3A &= ~(1<<WGM31);
	TCCR3A |= (1<<WGM30);

	// prescale 1
	TCCR3B |= (1<<CS30);
	TCCR3B &= ~((1<<CS32)|(1<<CS31));

	// Pin 3 as output
	DDRE |= (1<<3);

	// set OC3A ; PE3;
	TCCR3A |= (1<<COM3A1);
	TCCR3A &= ~(1<<COM3A0);
}

// initialize pwm for dimmer
void init_tcnt2_dimmer(){

	// set fast pwm
	TCCR2 |= ((1<<WGM21)|(1<<WGM20));

	TCCR2 |= (1<<CS20);

	TCCR2 |= ((1<<COM21)|(1<<COM20));

	OCR2 = 0x00;
}

// Setup TCNT1 for audio pwm frequency
// PD3
void init_tcnt1_audio_freq(){

	// 440 hz
	OCR1A = 0x8E0B;

	// ctc
	TCCR1B |= (1<<WGM12);
	// TCCR1A |= ((1<<WGM11)|(1<<WGM10));

	// prescale 1
	TCCR1B |= (1<<CS10);
	TCCR1B &= ~((1<<CS12)|(1<<CS11));

	DDRD |= (1<<3);

	TIMSK |= (1<<OCIE1A);

	// set normal port operation. 
	TCCR1A &= ~((1<<COM1A0)|(1<<COM1A1));

	sei();
}

void init_radio(){
	
	//Port E bit 2 is active high reset for radio 
	DDRE |= (1<<PE2); 

	// TWI defaults to 2 wire mode

	// Reset Sequence:
	//hardware reset Si4734
	PORTE |=  (1<<PE2);  
	//hold for 200us, 100us by spec
	_delay_us(200);
	//release reset
	PORTE &= ~(1<<PE2);
	//5us required because of my slow I2C translators I suspect
	_delay_us(30);

	current_fm_freq = DFLT_STION;

}

// Increments the clock array
void update_clock(uint16_t numMS){

	for (j = 1;j<=numMS;j++){

		// Conditionally update clock
		clk_state[3]++;
		if (clk_state[3] == 10){
			clk_state[2]++;
			clk_state[3] = 0;
		}
		if (clk_state[2] == 6){
			clk_state[1]++;
			clk_state[2] = 0;
		}
		if (clk_state[1] == 10){
			clk_state[0]++;
			clk_state[1] = 0;
		}
		// Handle military time
		if (militaryTime){
			if (clk_state[0] == 2 && clk_state[1] == 4){
				for (i = 0; i < 4; i++){
					clk_state[i] = 0;
				}	
			}
		}
		else{
			if (clk_state[0] == 1 && clk_state[1] == 3){
				for (i = 0; i < 4; i++){
					clk_state[i] = 0;
				}
				clk_state[1] = 1;
			}
		}
	
		if(!militaryTime){
			if (clk_state[0] == 1 && clk_state[1] == 2 && clk_state[2]==0 && clk_state[3] == 0){
				ampm = 1-ampm;
			}
		}
	}
}

// Decrements the clock array
void dec_clk(uint16_t numMS){


	if (!militaryTime){
		if (numMS == 1){
			if (clk_state[0] == 1 && clk_state[1] == 2 && clk_state[2]==0 && clk_state[3] == 0){	
				ampm = 1-ampm;
			}
		}
		else{
			if (clk_state[0] == 1 && clk_state[1] == 2){	
				ampm = 1-ampm;
			}	 
		}
	}


	for (j = 1;j<=numMS;j++){
	// for(i = 0;i<4;i++){
	// 	clk_state[i] = 1;
	// }


		if(clk_state[3] == 0){
			clk_state[3] = 9;
			if(clk_state[2] == 0){
				clk_state[2] = 5;
				if (militaryTime){
					if(clk_state[1] == 0){
						if(clk_state[0] != 0){
							clk_state[1] = 9;
							clk_state[0]--;
						}
						else{
							clk_state[0] = 2;
							clk_state[1] = 3;
						}
					}
					else{
						clk_state[1]--;
					}
				}
				else{
					if(clk_state[1] == 1){
						if(clk_state[0] == 0){
							clk_state[0] = 1;
							clk_state[1] = 2;
						}
						else{
							clk_state[0]--;
							clk_state[1] = 9;							
						}
					}
					else{
						clk_state[1]--;
					}
				}
			}
			else{
				clk_state[2]--;
			}
		}
		else{
			clk_state[3]--;
		}
	}
}

// Decrements the alarm array
void dec_alrm(uint16_t numMS){

	if (!militaryTime){
		if (numMS == 1){
			if (alrm_state[0] == 1 && alrm_state[1] == 2 && alrm_state[2]==0 && alrm_state[3] == 0){	
				ampm_alrm = 1-ampm_alrm;
			}
		}
		else{
			if (alrm_state[0] == 1 && alrm_state[1] == 2){	
				ampm_alrm = 1-ampm_alrm;
			}	 
		}
	}


	for (j = 1;j<=numMS;j++){
		// There has to be a better way...
		if(alrm_state[3] == 0){
			alrm_state[3] = 9;
			if(alrm_state[2] == 0){
				alrm_state[2] = 5;
				if (militaryTime){
					if(alrm_state[1] == 0){
						if(alrm_state[0] != 0){
							alrm_state[1] = 9;
							alrm_state[0]--;
						}
						else{
							alrm_state[0] = 2;
							alrm_state[1] = 3;
						}
					}
					else{
						alrm_state[1]--;
					}
				}
				else{
					if(alrm_state[1] == 1){
						if(alrm_state[0] == 0){
							alrm_state[0] = 1;
							alrm_state[1] = 2;
						}
						else{
							alrm_state[0]--;
							alrm_state[1] = 9;							
						}
					}
					else{
						alrm_state[1]--;
					}
				}
			}
			else{
				alrm_state[2]--;
			}
		}
		else{
			alrm_state[3]--;
		}
	}
}

// Increments the alarm array
void update_alrm(uint16_t numMS){

	for (j = 1;j<=numMS;j++){

		alrm_state[3]++;
		if (alrm_state[3] == 10){
			alrm_state[2]++;
			alrm_state[3] = 0;
		}
		if (alrm_state[2] == 6){
			alrm_state[1]++;
			alrm_state[2] = 0;
		}
		if (alrm_state[1] == 10){
			alrm_state[0]++;
			alrm_state[1] = 0;
		}
		if (militaryTime){
			if (alrm_state[0] == 2 && alrm_state[1] == 4){
				for (i = 0; i < 4; i++){
					alrm_state[i] = 0;
				}	
			}
		}
		else{
			if (alrm_state[0] == 1 && alrm_state[1] == 3){
				for (i = 0; i < 4; i++){
					alrm_state[i] = 0;
				}
				alrm_state[1] = 1;
			}
		}	
		if(!militaryTime){
			if (alrm_state[0] == 1 && alrm_state[1] == 2 && alrm_state[2]==0 && alrm_state[3] == 0){
				ampm_alrm = 1-ampm_alrm;
			}
		}
	}
}

// Read and debounce pushbuttons
void read_pushbuttons(){
	// Array of previous PORTA button values for debouncing
	static uint8_t PA_History[buffersize];
	if (PA_Buffer_Initialized){
  		// init Porta history to 0
		for (i = 0;i < buffersize; i++){
			PA_History[i] = 0;
		}  
		PA_Buffer_Initialized = 1;
	}
	// Debounced value of PORTA buttons
	static uint8_t PA_Debounce = 0b00000000;
	// Temporary variable for PA_Debounce
	static uint8_t PA_Temp;
	// Sample of PortA buttons
	static uint8_t PA;

	cli();
	//make PORTA an input port with pullups 
	DDRA = 0x00;	// inputs
    PORTA = 0xFF;	// pullups

    //enable tristate buffer for pushbutton switches
    PORTB = pb;
    _delay_ms(1);

    // Sample PORTA
    PA = ~PINA;
    sei();
    // Shift the shift register of PA history
    for(k=buffersize-1;k>0;k--){
      PA_History[k] = PA_History[k-1];
    }
    // Append the new PA sample
    PA_History[0] = PA;

    // Debounce Porta by shifting through a buffer.
    // only increment when all values are the same except for 
    // the last value.
    PA_Debounce = PA;
    PA_Temp = PA_Debounce;

    // If any of the eight buttons fails the conditions, set PA_temp
    // at the corresponding bit to 1 (no increment)
    for(k=1;k<buffersize-1;k++){
      for (j=0;j<8;j++){
        // Check for inconsistent history ex [110111]
        if ((PA_Debounce & (0x01 << j)) != (PA_History[k] & (0x01 << j))){
          PA_Temp &= ~(0x01 << j);
        }
      }
    }
    for (j=0;j<8;j++){
      // Check the final value so holding the button doesn't cause problems. 
      // ex [111110] (increment) vs [111111] (don't increment)
      if ((PA_Debounce & (0x01 << j)) == (PA_History[buffersize-1] & (0x01 << j))){
        PA_Temp &= ~(0x01 << j);
      }
    }

    debounced_PA = PA_Temp;
}

// Write to the 7seg
void write_7seg(uint8_t update_or_display){

	// init array of digit select keys
	static uint8_t selSet[5] = {d1,d2,d3,d4,dM};
	// static uint8_t selSet[4] = {d1,d2,d3,d4};
	// init array of digit to 7seg code
	static uint8_t nums[11] = {zer,one,two,thr,fou,fiv,six,sev,eig,nin,nul};

    //break up the disp_value to 4, BCD digits in the array: call (segsum)
    // uint16_t tempCount = count;
    // holds each place in an array
    static uint8_t digits[4] = {0,0,0,0};


	if (update_or_display == 0){
		// cli prevents display of leadng zeros
		cli();
		if (encoder_function == 2){
			for (i = 0;i < 4;i++){
				digits[i] = alrm_state[i];
			}
		}
		else if(encoder_function == 4){

			static uint16_t temp;
			temp = current_fm_freq;
			digits[0] = temp/10000;
			temp -= 10000 * digits[0];
			digits[1] = temp/1000;
			temp -= 1000 * digits[1];
			digits[2] = temp/100;
			temp -= 100 * digits[2];
			digits[3] = temp/10;
			temp -= 10 * digits[3];










			// static uint16_t frq_T;
			// frq_T = current_fm_freq;
			// frq_T = frq_T;
			// static char frq_str[4];
			// static uint8_t temp_val;
			// static char tempSTR[] = "";
			// tempSTR[0] = 0;
			// itoa(frq_T,frq_str,10);	
			// for (i = 0;i < 4 ;i++){
			// 	// strcat(tempSTR,frq_str[i]);
			// 	tempSTR[0] = frq_str[i];
			// 	temp_val = atoi(tempSTR);
			// 	digits[i] = temp_val;
			// }
		}
		else{
			for (i = 0;i < 4;i++){
				digits[i] = clk_state[i];
			}
		}
	    for (j = 0;j<3;j++){
	    	if (digits[j] == 0){
	        	digits[j] = 10;
	    	}
	    	else{
	        	break;
		    }
	    }
	    sei();
	}
	else{
		// This gets rid of the dim glow of turned off digits
	    PORTA = 0xFF;

	    // set ddrb to outputs except for mosi
	   	DDRB = 0xF7;

	    //disable tristate buffer for pushbutton switches
	    PORTB &= 0b10001111;
		PORTB |= selSet[digit_place];


	    //make PORTA an output
	    DDRA = 0xFF;

	    // PORTA = nums[digits[digit_place]];
	    //send 7 segment code to LED segments
	    if (digit_place != 4){
		    PORTA = nums[digits[digit_place]];
	    	digit_place++;
	    }
	    else{
	    	if(colon_on){
		    	PORTA = 0x04;
	    	}
	    	else{
	    		PORTA = 0xFF;
	    	}

	    	digit_place = 0;
	    }
	}    
}

// Write 8 bits of data to the bargraph display
void write_bargraph(uint8_t data){
	// Load data reg. This is sent.
	SPDR = data;
	// Wait until the data is sent
	while(bit_is_clear(SPSR,SPIF));
	// toggle the regCLK to display contents of shift register
	PORTD |= (1<<2);
	PORTD &= ~(1<<2);
}

// Initialize SPI
void spi_init(){

	// Set all to output except MISO
	DDRB = 0xF7;

	// Set RCLK to output
	DDRD |= 0x04;

	// SPE - SPI enable ; MSTR - master mode
	SPCR |= (1<<SPE)|(1<<MSTR);
}

// Write to the 7seg
void toggle_colon(){

    // This gets rid of the dim glow of turned off digits
    PORTA = 0xFF;

    // set ddrb to outputs except for mosi
   	DDRB = 0xF7;

    //disable tristate buffer for pushbutton switches
    PORTB &= 0b10001111;
	PORTB |= dM;

    //make PORTA an output
    DDRA = 0xFF;

	if(colon_on){
    	PORTA = 0x04;
	}
	else{
		PORTA = 0xFF;
	}
}

// Decrement display_output by the value specified by the mode.
void mode_decrement(uint8_t LR){		
	//L - 1
	//R - 0

	if (encoder_function == 1){
		dec_clk(1+LR*59);
	}
	else if (encoder_function == 2){
		dec_alrm(1+LR*59);
		// update_alrm(LR*59+1+LR*1439);
	}
	else if (encoder_function == 3){
		if (vol != 0x00){
			vol--;
			OCR3A = vol;
		}
	}
	else if (encoder_function == 4){
		if (current_fm_freq == 8810){
			current_fm_freq = 10790;
		}
		else{
			current_fm_freq = current_fm_freq - 20;
		}
	}
}

// Increment display_output by the value specified by the mode.
void mode_increment(uint8_t LR){
	// init the increment value to 1
		
	//L - 1
	//R - 0

	// if time set
	if (encoder_function == 1){
		update_clock(LR*59+1);	
	}
	// if alrm set
	else if (encoder_function == 2){
		update_alrm(LR*59+1);
	}
	// if volume set
	else if (encoder_function == 3){
		if (vol != 0xFF){
			vol++;
			OCR3A = vol;
		}
	}
	// If radio set:
	else if (encoder_function == 4){
		if (current_fm_freq == 10790){
			current_fm_freq = 8810;
		}
		else{
			current_fm_freq += 20;
		}
	}
}
// read the encoders via SPI
void read_encoders(){
	// Toggle SH_LD to sample encoder state.
	PORTB &= 0xFE;
	PORTB |= 0x01;
	// Load dummy values to initialize transfer
	SPDR = 0x00;
	// Wait for data to transmit/receive
	while(bit_is_clear(SPSR,SPIF)){}	
	// shift the buffer and append new value	
	prev_encoder_state = encoder_state;
	encoder_state = SPDR;
}

// Check the encoders to see if they have changed position
// Incrment or decrement display_output
void encoder_rotation(){
	// initialize counter to track in between states
	static uint8_t encoderL_int = 0x0F;
	static uint8_t encoderR_int = 0x0F;

	// Compare the previous state to the current state
	// states a,b,c,d correspond to 11, 10, 00, 01
	// When turned clockwise, a->b->c->d->a
	// When turned counterclockwise, a->d->c->b->a
	//
	// The following code increments the encoderLR_int to keep track of the transition states
	// Example: a->b then encoderLR_int++   ;   b->a then encoderLR_int--
	// When encoderLR_int goes up or down by four, we know it has completed one click.
	// Now, we increment or decrement.
	//
	// This is implemented with a switch/case statement for each encoder

	// Encoder Left:
	switch (encoder_state & 0b00000011){

		case 0b00000011:
			if((prev_encoder_state & 0b00000011) == 0b00000001){
				encoderL_int++;
			}
			else if((prev_encoder_state & 0b00000011) == 0b00000010){
				encoderL_int--;
			}
			break;
		case 0b00000010:
			if((prev_encoder_state & 0b00000011) == 0b00000011){
				encoderL_int++;
			}
			else if((prev_encoder_state & 0b00000011) == 0b00000000){
				encoderL_int--;
			}
			break;
		case 0b00000000:
			if((prev_encoder_state & 0b00000011) == 0b00000010){
				encoderL_int++;
			}
			else if((prev_encoder_state & 0b00000011) == 0b00000001){
				encoderL_int--;
			}			
			break;
		case 0b00000001:
			if((prev_encoder_state & 0b00000011) == 0b00000000){
				encoderL_int++;
			}
			else if((prev_encoder_state & 0b00000011) == 0b00000011){
				encoderL_int--;
			}
			break;
	}
	// If it has increased by 4, increment
	if (encoderL_int == 0x0F + 4){
		// increment counter upwards as designated by mode
		mode_increment(1);
		// Reset counter
		encoderL_int = 0x0F;
	}
	else if(encoderL_int == 0x0F - 4){
		// Decrement counter as designated by mode
		mode_decrement(1);
		// Reset Counter
		encoderL_int = 0x0F;
	}
	// Reset the counter when the encoder is in a clicked position
	if ((encoder_state & 0b00000011) == 0x03){
		encoderL_int = 0x0F;
	}
	// See above for commenting, they are identical.
	switch ((encoder_state>>2) & 0b00000011){

		case 0b00000011:
			if(((prev_encoder_state>>2) & 0b00000011) == 0b00000001){
				encoderR_int++;
			}
			else if(((prev_encoder_state>>2) & 0b00000011) == 0b00000010){
				encoderR_int--;
			}
			break;
		case 0b00000010:
			if(((prev_encoder_state>>2) & 0b00000011) == 0b00000011){
				encoderR_int++;
			}
			else if(((prev_encoder_state>>2) & 0b00000011) == 0b00000000){
				encoderR_int--;
			}
			break;
		case 0b00000000:
			if(((prev_encoder_state>>2) & 0b00000011) == 0b00000010){
				encoderR_int++;
			}
			else if(((prev_encoder_state>>2) & 0b00000011) == 0b00000001){
				encoderR_int--;
			}			
			break;
		case 0b00000001:
			if(((prev_encoder_state>>2) & 0b00000011) == 0b00000000){
				encoderR_int++;
			}
			else if(((prev_encoder_state>>2) & 0b00000011) == 0b00000011){
				encoderR_int--;
			}
			break;
	}
	if (encoderR_int == 0x0F + 4){
		// increment counter upwards as designated by mode
		mode_increment(0);
		encoderR_int = 0x0F;
	}
	else if(encoderR_int == 0x0F - 4){
		mode_decrement(0);
		encoderR_int = 0x0F;
	}
	if (((encoder_state>>2) & 0b00000011) == 0x03){
		encoderR_int = 0x0F;
	}
}

// Initialize analog to digital converter
void init_adc(){
	// configure ADC
	ADMUX |= (1<<REFS0);
	ADCSRA |= (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);	
}

// Compute change in time since last run
uint16_t delta_Time(uint32_t *prev_msCLK, uint32_t *c_msCLK){

	uint32_t d_msCLK = 0;

	*prev_msCLK 	= *c_msCLK;
	*c_msCLK		= msCLK;

	if (*prev_msCLK <= *c_msCLK){
		d_msCLK		= *c_msCLK - *prev_msCLK;
	}
	else{
		// if current is less than previous, then rollover occured.
		// Rollover occurs at: 
		d_msCLK		= *c_msCLK + (ROLLOVER - *prev_msCLK);
	}
	return(d_msCLK);
}

// Read from the adc
uint16_t readADC(){

  // start single convertion
  // write ’1′ to ADSC
  ADCSRA |= (1<<ADSC);
 
  // wait for conversion to complete
  while(ADCSRA & (1<<ADSC));
 
  return (ADC);
}

// convert military to std time
void military_std(){
	if (militaryTime){
		if (clk_state[0] == 1 && clk_state[1] > 2){
			dec_clk(12*60);
			militaryTime = 1-militaryTime;
			update_clock(12*60);
			ampm = 1;
		}
		else if(clk_state[0] == 2){
			dec_clk(12*60);
			militaryTime = 1-militaryTime;
			update_clock(12*60);
			ampm = 1;
		}
		else if (clk_state[0] == 0 && clk_state[1] == 0){
			clk_state[0] = 1;
			clk_state[1] = 2;
			ampm = 1;
			militaryTime = 1-militaryTime;
		}
		else{
			militaryTime = 1-militaryTime;
		}
	}
	else{
		if (ampm){
			militaryTime = 1-militaryTime;
			update_clock(12*60);
			ampm = 1-ampm;
		}
		else{
			militaryTime = 1-militaryTime;							
		}
	}
	if (alrmmilitaryTime){
		if (alrm_state[0] == 1 && alrm_state[1] > 2){
			dec_alrm(12*60);
			alrmmilitaryTime = 1-alrmmilitaryTime;
			update_alrm(12*60);
			ampm_alrm = 1;
		}
		else if(alrm_state[0] == 2){
			dec_alrm(12*60);
			alrmmilitaryTime = 1-alrmmilitaryTime;
			update_alrm(12*60);
			ampm_alrm = 1;
		}
		else if (alrm_state[0] == 0 && alrm_state[1] == 0){
			alrm_state[0] = 1;
			alrm_state[1] = 2;
			ampm_alrm = 1;
			alrmmilitaryTime = 1-alrmmilitaryTime;
		}
		else{
			alrmmilitaryTime = 1-alrmmilitaryTime;
		}
	}
	else{
		if (ampm_alrm){
			alrmmilitaryTime = 1-alrmmilitaryTime;
			update_alrm(12*60);
			ampm_alrm = 1-ampm_alrm;
		}
		else{
			alrmmilitaryTime = 1-alrmmilitaryTime;							
		}
	}
}


////////////////////////////////////////////////////////////
// Begin Main Function
int main(){

	// Initialize Port register control

	// Initialize SPI control
	spi_init();

	// Initialize interrupt control
	// Set up PWM and timers
	init_counter();
	init_tcnt3_audio_volume();
	// init_tcnt1_audio_freq();
	init_tcnt2_dimmer();

	// initialize adc
	init_adc();

	// init temperature twi
	init_twi();

	// Initialize UART
	uart_init();

	// Initialize LCD
	lcd_init();
	clear_display();

	// initialize radio
	init_radio();

	// initialize timing counters
	static uint32_t	prev_msCLK;
	static uint32_t	c_msCLK;
	static uint16_t d_msCLK;

	// Target Timings
	// toggle ':'				| 	1 second
	uint16_t t_since_lcd =0;
	uint16_t period_lcd = 1024 * (32/update_period);

	// check buttons 			|	2 ms
	uint16_t t_since_but=0;
	uint16_t period_but = 2 * (32/update_period);

	// check encoders			|	1 ms
	uint16_t t_since_enc=0;
	uint16_t period_enc = 1 * (32/update_period);

	// write bargraph			|	20 ms
	uint16_t t_since_bg=0;
	uint16_t period_bg = 20 * (32/update_period);

	// Update 7seg				|	20 ms
	uint16_t t_since_7s=0;
	uint16_t period_7s = 20 * (32/update_period);

	// adjust pwm duty cycles	|	50 ms
	uint16_t t_since_lvl=0;
	uint16_t period_lvl = 50 * (32/update_period);

	// Compare the alarm 	| 	500 ms
	uint16_t t_since_alrm=0;
	uint16_t period_alrm = 500 * (32/update_period);


	// adjust pwm frequency	of alarm	|	100 ms
	uint16_t t_since_frq=0;
	// uint16_t period_frq = 1;
	uint16_t period_frq = 100 * (32/update_period);

	// Read local temp 	| 	1000 ms
	uint16_t t_since_tempL = 0;
	uint16_t period_tempL = 1000 * (32/update_period);

	// Read Remote Temp | 1000 ms
	uint16_t t_since_tempR = 0;
	uint16_t period_tempR = 1000 * (32/update_period);

	// Array of process period and tsince
	uint16_t t_since[N_Process] = {t_since_lcd,t_since_but,t_since_enc,t_since_bg,t_since_7s,t_since_lvl,t_since_alrm,t_since_frq,t_since_tempL,t_since_tempR};
	uint16_t period[N_Process] = {period_lcd,period_but,period_enc,period_bg,period_7s,period_lvl,period_alrm,period_frq,period_tempL,period_tempR};

	// stores index of process that is most important.
	uint8_t most_important_process=0;
	uint8_t max_time_since=0;

	// Photo resistor value
	uint16_t photoVal;

	while(1){

		// Lets make a finite state machine.

		// The transitions between states will be determined
		// on a time priority basis - this will be calculated
		// by finding the maximum process distance from
		// its target period. 

		// At the start of the loop, update all the times
		d_msCLK = delta_Time(&prev_msCLK,&c_msCLK);

		// Now, update t_since variables
		for (i = 0; i < N_Process;i++){
			t_since[i] = t_since[i] + d_msCLK;
		} //for

		// compute a vector of distances from target times
		// bias the difference into the positive region
		most_important_process = 0;
		max_time_since = 0;
		for (i = 0; i < N_Process;i++){
			if(period[i] <= t_since[i]){
				if (t_since[i]-period[i] > max_time_since){
					max_time_since = t_since[i]-period[i];
					most_important_process = i;
				}//if
			}//if
		}//for

		if (update_EN){
			update_clock(1);
			update_EN = 0;
		}

		// Read encoders for maximal time sensitivity
		read_encoders();
		encoder_rotation();

		// Now, switch to the most important process
		switch(most_important_process){

			// write to lcd disp
			case 0:

				if (1-lcd_flip_flop){
					clear_display();
					line_1_2 = 1;
					// Display temperature
					if (line_1_2 == 1){
						// set_cursor(0x2,0x0);
						line2_col1();
						line_1_2 = 2;
					}


					for (i = 0; i < strlen(lcdStr);i++){
						lcdStr[i] = 0;
					}

					// Display Temperature data
					string2lcd(" In:");
					string2lcd(tempL);
					string2lcd(" Out:");
					string2lcd(tempR);

					// display alarm status
					// if(play_alrm){
					if (line_1_2 == 2){
						set_cursor(0x1,0x0);
						line_1_2 = 1;
					}
					string2lcd(" ALARM:");
					if(alrm_en){
						string2lcd("Set~");
						if(play_rad){
							string2lcd("Radio");
						}
						else if(!play_rad){
							string2lcd("Buzz");
						}
					}
					else{
						string2lcd("Off");
					}

					lcd_flip_flop = 1-lcd_flip_flop;
				}

				t_since[0] = 0;
				break;
			// check buttons
			case 1:
				// Scan Pushbuttons
				read_pushbuttons();
				// Update State

				//enable volume control
				if(debounced_PA & (0x01)){
					if (encoder_function == 3){
						encoder_function = 0;
					}
					else{
						encoder_function = 3;
					}
				}
				// Set alarm or radio
				if(debounced_PA & (0x01<<1)){
					play_rad = 1 - play_rad;
				}

				// time set	
				if(debounced_PA & (0x01<<2)){
					if (encoder_function == 1){
						encoder_function = 0;
					}
					else{
						encoder_function = 1;
					}
				}
				// Alarm enable
				if (debounced_PA & (0x01<<3)){
					alrm_en = 1-alrm_en;
				}
				// alarm set
				if(debounced_PA & (0x01<<4)){
					if (encoder_function == 2){
						encoder_function = 0;
					}
					else{
						encoder_function = 2;
					}
				}
				// radio tune
				if (debounced_PA & (1<<5)){
					encoder_function = 4;
				}

				// military/std
				if (debounced_PA & (1<<6)){
						military_std();
				}
				// snooze
				if (debounced_PA & (0x01<<7)){
					if (play_alrm){
						clear_display();
						lcd_flip_flop = 1-lcd_flip_flop;
						play_alrm = 1-play_alrm;
						update_alrm(10);
					}
				}

				t_since[1] = 0;

				break;

			// check encoders
			case 2:
				// Relocated to run every iteration
				// read_encoders();
				// encoder_rotation();
				t_since[2] = 0;			
				break;
			// write bargraph
			case 3:
				if(encoder_function == 2){
					write_bargraph(((play_alrm)|0b11111110)&((1<<2*play_rad)|(1<<3*alrm_en)|(1<<4*alrmmilitaryTime)|(1<<5*ampm_alrm)|(1<<(encoder_function+5))));
				}
				else if(encoder_function == 1){
					write_bargraph(((play_alrm)|0b11111110)&((1<<2*play_rad)|(1<<3*alrm_en)|(1<<4*alrmmilitaryTime)|(1<<5*ampm)|(1<<(encoder_function+5))));
				}
				else{
					write_bargraph(((play_alrm)|0b11111110)&((1<<2*play_rad)|(1<<3*alrm_en)|(1<<4*militaryTime)|(1<<5*ampm)));
				}

				t_since[3] = 0;
				break;
			// write 7seg
			case 4:
				// Update 7 seg (but don't display)
				// Display is done in isr
				write_7seg(0);
				t_since[4] = 0;
				break;

			// adjust 7seg brightness
			case 5:
				// sample brightness
				photoVal = readADC();
				OCR2 = 30*(photoVal>>7)+1;
				t_since[5] = 0;			
				break;

			// compare to the alarm
			case 6:
				if (alrm_en){
					for(i=0;i<4;i++){
						if (clk_state[i] != alrm_state[i]){
							break;
						}
						if (i == 3){
							play_alrm = 1;
						}
					}
				}
				t_since[6] = 0;
				break;
			// Radio Stuff
			case 7:

				// Radio
				if (play_alrm && play_rad && !rad_on){
					fm_pwr_up();        //power up radio
					_delay_ms(300);
					fm_tune_freq();
					rad_on = 1;
				}
				if (!play_alrm && rad_on){
					radio_pwr_dwn();
					rad_on = 0;
				}
				if(!play_rad && rad_on){
					radio_pwr_dwn();
					rad_on = 0;
				}
				if (current_fm_freq != prev_frq){
					fm_tune_freq();
					prev_frq = current_fm_freq;
				}

				t_since[7] = 0;
				break;

			// Read Temperature
			case 8:

				// Get local Temp
				twi_start_rd(LM73_ADDRESS, tempData, byte_cnt);
				tempHighLow = ((uint16_t)tempData[0] << 8) | tempData[1];

				itoa(tempHighLow/128,tempL,10);

				// set lcd write and period reset
				lcd_flip_flop = 0;
				t_since[8] = 0;
				
			// Uart communication with atmega 48
			case 9:
				// Requests data, and polls for returned messages
				uart_putc('y');
				tempR[0] = uart_getc();
				tempR[1] = uart_getc();
				tempR[2] = uart_getc();

				// set lcd write and period reset
				lcd_flip_flop = 0;
				t_since[9] = 0;

		}
	}//while	
	return 0;
}//main
