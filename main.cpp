/*
 * BT_16Bit_PWM.cpp
 *
 * Created: 09.05.2016 20:40:33
 * Author : holzi
 */ 


//Commands
/*
| Identifier | Values (uint_8) | Stop Indicator | Diskription									   |
|     0      |		  4        |      '\n'      | Saves the current slider configuration to mode 0 |
|     1      |		  4        |      '\n'      | Saves the current slider configuration to mode 1 |
|     2      |		  4        |      '\n'      | Saves the current slider configuration to mode 2 |
|     3      |		  4        |      '\n'      | Saves the current slider configuration to mode 3 |
|     4      |		  4        |      '\n'      | Saves the current slider configuration to mode 4 |
|     5      |		  4        |      '\n'      | Saves the current slider configuration to mode 5 |
|     6      |		  1        |      '\n'      | Sets Color to Cold White Side 1				   |
|     7      |		  1        |      '\n'      | Sets Color to Warm White Side 1				   |
|     8      |		  1        |      '\n'      | Sets Color to Cold White Side 2			 	   |
|     9      |		  1        |      '\n'      | Sets Color to Warm White Side 2				   |
|     10     |		  0        |      '\n'      | Forces all OFF								   |
|     11     |		  0        |      '\n'      | Forces all to mode 0 (default)				   |*/


#ifndef F_CPU
#define F_CPU 16000000UL // 16 MHz clock speed
#endif

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include "inttypes.h"
# include <avr/eeprom.h>

#include "USART2.h"
#include "spi.h"
#include "TLC59711.h"
#include "debounce.h"

#define BAUDRATE		51			//19200 -> Refer Datasheet Page 190
#define DIMM_STEP		1			//Step for 1 Repeat of the debounce routine (can be set in debounce.h)
#define SET_MAX_DIM_VAL	255			//Sets the maximum value for Dimming (maximum brightness can be reduced to do leck of cooling possibilities)
#define NUM_OF_MODES		5    		//Number of predefined modeS

#define CW1	1						//Cold White Side 1
#define WW1	2						//Warm White Side 1
#define CW2	1						//Cold White Side 2
#define WW2	2						//Warm White Side 2

#define  ON  1
#define  OFF 0

typedef struct {
	uint8_t coldW1;			//coldWhite 1
	uint8_t warmW1;			//warmWhite 1
	uint8_t coldW2;			//coldWhite 2
	uint8_t warmW2;			//warmWhite 2
} GSvalues;


//Predefined Color sets -> later unimportant
GSvalues EEMEM GSmodes[NUM_OF_MODES];
//MUSS NOCH ANGEPASST WERDEN!!!!!!!!!!!!!!!!!!!
const uint16_t logTable_16[256] =
{
0, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 3,
3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5, 6, 6, 6, 6, 7,
7, 7, 8, 8, 8, 9, 9, 10, 10, 10, 11, 11, 12, 12, 13, 13, 14, 15,
15, 16, 17, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29,
31, 32, 33, 35, 36, 38, 40, 41, 43, 45, 47, 49, 52, 54, 56, 59,
61, 64, 67, 70, 73, 76, 79, 83, 87, 91, 95, 99, 103, 108, 112,
117, 123, 128, 134, 140, 146, 152, 159, 166, 173, 181, 189, 197,
206, 215, 225, 235, 245, 256, 267, 279, 292, 304, 318, 332, 347,
362, 378, 395, 412, 431, 450, 470, 490, 512, 535, 558, 583, 609,
636, 664, 693, 724, 756, 790, 825, 861, 899, 939, 981, 1024, 1069,
1117, 1166, 1218, 1272, 1328, 1387, 1448, 1512, 1579, 1649, 1722,
1798, 1878, 1961, 2048, 2139, 2233, 2332, 2435, 2543, 2656, 2773,
2896, 3025, 3158, 3298, 3444, 3597, 3756, 3922, 4096, 4277, 4467,
4664, 4871, 5087, 5312, 5547, 5793, 6049, 6317, 6596, 6889, 7194,
7512, 7845, 8192, 8555, 8933, 9329, 9742, 10173, 10624, 11094,
11585, 12098, 12634, 13193, 13777, 14387, 15024, 15689, 16384,
17109, 17867, 18658, 19484, 20346, 21247, 22188, 23170, 24196,
25267, 26386, 27554, 28774, 30048, 31378, 32768, 34218, 35733,
37315, 38967, 40693, 42494, 44376, 46340, 48392, 50534, 52772,
55108, 57548, 60096, 62757, 65535
};	//This can be outsourced to Flash memory using PROGMEM if the data memory gehts full


//intital Greyscale Values
GSvalues actGSvalue = {50, 50, 50, 50};
	
bool dimm_direc = true;			//true  = up; false = down
uint8_t mode = 0;				//Color Set
uint8_t max_dim_val = SET_MAX_DIM_VAL;
TLC59711 myChip;


int idx = 0;
uint8_t tmp = 0;
uint8_t command[10];
bool RX_ERROR = false;



ISR(TIMER0_OVF_vect)		// every 10ms for debouncing the switch
{
	static uint8_t ct0, ct1, rpt;
	uint8_t i;

	TCNT0 = (uint8_t)(uint16_t)-(F_CPU / 1024 * 10e-3 + 0.5);	// preload for 10ms

	i = key_state ^ ~(KEY_PIN & 0x08);		// key changed ?
	ct0 = ~( ct0 & i );			// reset or count ct0
	ct1 = ct0 ^ (ct1 & i);		// reset or count ct1
	i &= ct0 & ct1;			// count until roll over ?
	key_state ^= i;			// then toggle debounced state
	key_press |= key_state & i;		// 0->1: key press detect
	key_release |= ~key_state & i; // 1->0: key release detect
	
	if( (key_state & REPEAT_MASK) == 0 )	// check repeat function
	rpt = REPEAT_START;		// start delay
	if( --rpt == 0 ){
		rpt = REPEAT_NEXT;			// repeat delay
		key_rpt |= key_state & REPEAT_MASK;
	}
}

void writeOutGSvalues(GSvalues* GSvalue)
{
	myChip.setGreyScale(3, 0, logTable_16[GSvalue->coldW1] , logTable_16[GSvalue->warmW1]);
	myChip.setGreyScale(2, 0, logTable_16[GSvalue->warmW2], logTable_16[GSvalue->coldW2]);
}


void switch_mode(uint8_t mode)
{
	//One Set of Dimming values is 4 Byte long
	//EEMEM starts with addressing at 0
	//so the first element (mode = 0) is at 4*mode = 0 , second at 4*mode = 4*1 ... usw
	//interrupts should be deactivated during read and write process
	cli();
	eeprom_read_block (( void *) &actGSvalue , ( const void *)(mode*4) , 4);
	writeOutGSvalues(&actGSvalue);
	sei();
}

void process_switch(void)
{
	//Short Press recognized
	if(get_key_short(1<<KEY0))
	{
		PORTB ^= (1<<PORTB0);
		mode++;
		if (mode == NUM_OF_MODES) mode = 0;
				
		switch_mode(mode);
	}
			
	//long press recognized (needed for correct behaviour of get_key_rpt_l() )
	if( get_key_long_r( 1<<KEY0 ))
		asm("nop");			//do nothing

			
	//Pressed an hold -> Dimming called every 50ms
	if(get_key_rpt_l( 1<<KEY0 ))
	{
		//All colors are treated the same
		if(dimm_direc == true) //dimm up
		{
			switch (mode)
			{
				case 0: //only side 1
					if (actGSvalue.coldW1 <= (max_dim_val-DIMM_STEP)) actGSvalue.coldW1 += DIMM_STEP;
					if (actGSvalue.warmW1 <= (max_dim_val-DIMM_STEP)) actGSvalue.warmW1 += DIMM_STEP;
					break;
				case 1: //only side 2
					if (actGSvalue.coldW2 <= (max_dim_val-DIMM_STEP)) actGSvalue.coldW2 += DIMM_STEP;
					if (actGSvalue.warmW2 <= (max_dim_val-DIMM_STEP)) actGSvalue.warmW2 += DIMM_STEP;
					break;
					
				case 4: break; // OFF STATE
					
				default: //both sides the same for mode 2 and 3
					if (actGSvalue.coldW1 <= (max_dim_val-DIMM_STEP)) actGSvalue.coldW1 += DIMM_STEP;
					if (actGSvalue.warmW1 <= (max_dim_val-DIMM_STEP)) actGSvalue.warmW1 += DIMM_STEP;
					if (actGSvalue.coldW2 <= (max_dim_val-DIMM_STEP)) actGSvalue.coldW2 += DIMM_STEP;
					if (actGSvalue.warmW2 <= (max_dim_val-DIMM_STEP)) actGSvalue.warmW2 += DIMM_STEP;
				break;				
			}
			//write out to chip								
			writeOutGSvalues(&actGSvalue);
		}
		else	//Dimm down
		{
			switch (mode)
			{
				case 0: //only side 1
					if (actGSvalue.coldW1 >= DIMM_STEP) actGSvalue.coldW1 -= DIMM_STEP;
					if (actGSvalue.warmW1 >= DIMM_STEP) actGSvalue.warmW1 -= DIMM_STEP;
					break;
				case 1: //only side 2
					if (actGSvalue.coldW2 >= DIMM_STEP) actGSvalue.coldW2 -= DIMM_STEP;
					if (actGSvalue.warmW2 >= DIMM_STEP) actGSvalue.warmW2 -= DIMM_STEP;
					break;
					
				case 4: break;	//off state
				
				default: //both sides the same for mode 2 and 3
					if (actGSvalue.coldW1 >= DIMM_STEP) actGSvalue.coldW1 -= DIMM_STEP;
					if (actGSvalue.warmW1 >= DIMM_STEP) actGSvalue.warmW1 -= DIMM_STEP;
					if (actGSvalue.coldW2 >= DIMM_STEP) actGSvalue.coldW2 -= DIMM_STEP;
					if (actGSvalue.warmW2 >= DIMM_STEP) actGSvalue.warmW2 -= DIMM_STEP;
					break;
			}
			//write out to chip
			writeOutGSvalues(&actGSvalue);
		}
	}
			
	if(get_key_release( (1<<KEY0) ))
	{
		dimm_direc = !dimm_direc;	//toggle dimming direction
	}
}

void save_mode( uint8_t* command)
{
	GSvalues tmpGSValues;
	
	tmpGSValues.coldW1 = *(command + 1 );	//Second element
	tmpGSValues.warmW1 = *(command + 2 );
	tmpGSValues.coldW2 = *(command + 3 );
	tmpGSValues.warmW2 = *(command + 4 );
	eeprom_update_block ((const void *) &tmpGSValues , ( void *)( *(command + 0) * 4) , 4);
}


int main(void)
{
	//BTM222 Reset Configuration (not needed because of internal Pullup of the BTM222)
	/*DDRB |= (1<<PB1);
	PORTB |= (1<<PB1);		//high -> no Reset
	PORTB &= ~(1<<PB1);		//low  -> Reset   */

	SPI_init();
	USART_Init(BAUDRATE);


	//Configuration Timer0 (used for debouncing)
	TCCR0B = (1<<CS02)|(1<<CS00);	// divide by 1024
	TIMSK0 |= (1 << TOIE0);			// enable timer interrupt
	
	DDRB  |= (1 << PORTB0 );		//PB0 is output (LED)
	PORTB |= (1 <<PORTB0);			//Switch LED on
		
	PORTD |= (1<<PORTD3);			//Activate Pullups for PD3 (Switch)
	
	switch_mode(mode);				//Color set 0 ist default at startup
	
	sei();							//Enable Global Interrups (for USART)
		


    while (1) 
    {
			
		process_switch();			//button polling
		
		
		//Parse incomming data on the UART from BTM222
		if (DataInReceiveBuffer())
		{
			
			idx = 0;
			tmp = 0;
			RX_ERROR = false;
			
			for (int a = 0 ; a<9 ; a++)
			{
				command[a] = 0;
			}
			
			while(tmp != '\n')
			{
				//parse until end of command reaced and save in command Array
				if (idx == 10)
				{
					RX_ERROR = true;
					break;		//an Error Occoured -> there is no such long command
				}
				
				tmp = USART_Receive();
				command[idx] = tmp;
				idx++;
			}			
			
			if  (RX_ERROR == false)
			{
				switch (command[0])
				{
					case  0: save_mode(command);								  break;
					case  1: save_mode(command);								  break;
					case  2: save_mode(command);							  	  break;
					case  3: save_mode(command);								  break;
					case  4: save_mode(command);								  break;
					case  5: save_mode(command);								  break;
							
					case  6: actGSvalue.coldW1 = command[1];					  
								myChip.setSingleGS(3, CW1, logTable_16[command[1]]);break;
								     				 
					case  7:
						actGSvalue.warmW1 = command[1]; 
						myChip.setSingleGS(3, WW1, logTable_16[command[1]]); break;
							
					case  8: 
						actGSvalue.coldW2 = command[1];
					myChip.setSingleGS(2, WW2, logTable_16[command[1]]); break;
							
					case  9:

							actGSvalue.warmW2 = command[1];					
					myChip.setSingleGS(2, CW2, logTable_16[command[1]]); break;
							
					case 10: myChip.global_OFF(OFF);					          break;
					case 11: switch_mode(0);									  break; //Default mode is 0						
					default: break;
				}
			}
		}
	}
}
