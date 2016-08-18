/*
 * BT_16Bit_PWM.cpp
 *
 * Created: 09.05.2016 20:40:33
 * Author : holzi
 */ 

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
#define NUM_OF_SETS		5    		//Number of predefined Color Sets


typedef struct {
	uint8_t coldW1;			//coldWhite 1
	uint8_t warmW1;			//warmWhite 1
	uint8_t coldW2;			//coldWhite 2
	uint8_t warmW2;			//warmWhite 2
} GSvalues;


//Predefined Color sets -> later unimportant
GSvalues EEMEM GScSets[NUM_OF_SETS] =  {{ 0,   0,   0,   0 },		//Color Set 1
										{ 10,  10,  10,  10 },		//Color Set 2
										{ 50,  50,  50,  50 },		//Color Set 3
										{ 150, 150, 150, 150 },		//Color Set 4
										{ 255, 255, 255, 255 }};	//Color Set 5
//MUSS NOCH ANGEPASST WERDEN!!!!!!!!!!!!!!!!!!!
const uint16_t logTable_16[256] =
{
0, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2,
2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4,
4, 4, 5, 5, 5, 5, 5, 6, 6, 6, 6, 7, 7, 7, 8, 8,
8, 9, 9, 10, 10, 10, 11, 11, 12, 12, 13, 13, 14,
15, 15, 16, 17, 17, 18, 19, 20, 21, 22, 23, 24,
25, 26, 27, 28, 29, 31, 32, 33, 35, 36, 38, 40,
41, 43, 45, 47, 49, 52, 54, 56, 59, 61, 64, 67,
70, 73, 76, 79, 83, 87, 91, 95, 99, 103, 108, 112,
117, 123, 128, 134, 140, 146, 152, 159, 166, 173,
181, 189, 197, 206, 215, 225, 235, 245, 256, 267,
279, 292, 304, 318, 332, 347, 362, 378, 395, 412,
431, 450, 470, 490, 512, 535, 558, 583, 609, 636,
664, 693, 724, 756, 790, 825, 861, 899, 939, 981,
1024, 1069, 1117, 1166, 1218, 1272, 1328, 1387,
1448, 1512, 1579, 1649, 1722, 1798, 1878, 1961,
2048, 2139, 2233, 2332, 2435, 2543, 2656, 2774,
2896, 3025, 3158, 3298, 3444, 3597, 3756, 3922,
4096, 4277, 4467, 4664, 4871, 5087, 5312, 5547,
5793, 6049, 6317, 6597, 6889, 7194, 7512, 7845,
8192, 8555, 8933, 9329, 9742, 10173, 10624, 11094,
11585, 12098, 12634, 13193, 13777, 14387, 15024, 15689,
16384, 17109, 17867, 18658, 19484, 20347, 21247, 22188,
23170, 24196, 25268, 26386, 27554, 28774, 30048, 31379,
32768, 34219, 35734, 37316, 38968, 40693, 42495, 44376,
46341, 48393, 50535, 52773, 55109, 57549, 60097, 62757,
65535
};	//This can be outsourced to Flash memory using PROGMEM if the data memory gehts full


//intital Greyscale Values
GSvalues actGSvalue = {50, 50, 50, 50};
	
bool dimm_direc = true;			//true  = up; false = down
uint8_t CSet = 0;				//Color Set
uint8_t max_dim_val = SET_MAX_DIM_VAL;




TLC59711 myChip;

char string[10];
uint16_t val;
int tmp;
int i;
int modus;

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
	myChip.setGreyScale(3, logTable_16[GSvalue->coldW1], logTable_16[GSvalue->warmW1], 0x0);
	myChip.setGreyScale(2, logTable_16[GSvalue->coldW2], logTable_16[GSvalue->warmW2], 0x0);
}

void switch_CSet(uint8_t CSet)
{
	//One Set of Dimming values is 4 Byte long
	//EEMEM starts with addressing at 0
	//so the first element (Cset = 0) is at 4*CSet = 0 , second at 4*Cset = 4*1 ... usw
	//interrupts should be deactivated during read and write process
	cli();
	eeprom_read_block (( void *) &actGSvalue , ( const void *)(CSet*4) , 4);
	writeOutGSvalues(&actGSvalue);
	sei();
}

void process_switch(void)
{
	//Short Press recognized
	if(get_key_short(1<<KEY0))
	{
		PORTB ^= (1<<PORTB0);
		CSet++;
		if (CSet == NUM_OF_SETS) CSet = 0;
				
		switch_CSet(CSet);
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
			//overflow prevention
			if((actGSvalue.coldW1 <= (max_dim_val-DIMM_STEP)) && (actGSvalue.warmW1 <= (max_dim_val-DIMM_STEP)) &&
			   (actGSvalue.coldW2 <= (max_dim_val-DIMM_STEP)) && (actGSvalue.warmW2 <= (max_dim_val-DIMM_STEP)))
			{
				actGSvalue.coldW1 += DIMM_STEP;
				actGSvalue.warmW1 += DIMM_STEP;
				actGSvalue.coldW2 += DIMM_STEP;
				actGSvalue.warmW2 += DIMM_STEP;
						
				//write out to chip
				writeOutGSvalues(&actGSvalue);
			}
		}
		else
		{	//dimm down
			if((actGSvalue.coldW1 >= DIMM_STEP) && (actGSvalue.warmW1 >= DIMM_STEP) &&		//overflow prevention
			   (actGSvalue.coldW2 >= DIMM_STEP) && (actGSvalue.warmW2 >= DIMM_STEP))
			{
				actGSvalue.coldW1 -= DIMM_STEP;
				actGSvalue.warmW1 -= DIMM_STEP;
				actGSvalue.coldW2 -= DIMM_STEP;
				actGSvalue.warmW2 -= DIMM_STEP;
						
				//write out to chip
				writeOutGSvalues(&actGSvalue);
			}
		}
	}
			
	if(get_key_release( (1<<KEY0) ))
	{
		dimm_direc = !dimm_direc;	//toggle dimming direction
	}
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
	
	switch_CSet(CSet);				//Color set 0 ist default at startup
	
	sei();							//Enable Global Interrups (for USART)
		
		
		int faktor =257;

	
	
    while (1) 
    {
			
		process_switch();			//collects the polling information from the button
		
		if (DataInReceiveBuffer())
		{
			for (int a = 0 ; a<9 ; a++)
			{
				string[a] = 0;
			}
			
			if(USART_Receive() == 'a')
			{
				USART_Receive();	//space
				i = 0;
				tmp = 0;
				while(tmp != '\n')
				{
					tmp = USART_Receive();
					string[i] = tmp;	
					i++;
				}
				
				string[i] =  '\0';
				
				val = atoi(string);
				val = faktor*val;
				
				myChip.setGreyScale(3, val, val, val);
				myChip.setGreyScale(2, val, val, val);
			}
		}
	}
		
}

/*switch(USART_Receive())
{
	case 'a':
	PORTB |= (1 << PB0);			//Turn on
	myChip.setGlobalBrightness(3, 0x7F);
	myChip.setGreyScale(3, 0xFFFF, 0xFFFF,0xFFFF);
	myChip.setGreyScale(2, 0xFFFF, 0xFFFF,0xFFFF);
	USART_Transmit('a');
	USART_Transmit('n');
	USART_Transmit('\n');
	USART_Transmit('\r');
	break;
	
	case 'b':
	PORTB &= ~(1 << PB0);			//Turn on
	myChip.setGreyScale(3 , 0x0 , 0x0, 0x0);
	myChip.setGreyScale(2 , 0x0 , 0x0, 0x0);
	USART_Transmit('a');
	USART_Transmit('u');
	USART_Transmit('s');
	USART_Transmit('\n');
	USART_Transmit('\r');
	break;
	
	case '0':
	myChip.setGreyScale(3 , 0x0001 , 0x0001, 0x1);
	myChip.setGreyScale(2 , 0x0001 , 0x0001, 0x1);
	USART_Transmit('0');
	USART_Transmit('\n');
	USART_Transmit('\r');
	break;
	
	case '1':
	myChip.setGreyScale(3 , 0x0002 , 0x0002, 0x2);
	myChip.setGreyScale(2 , 0x0002 , 0x0002, 0x2);
	USART_Transmit('1');
	USART_Transmit('\n');
	USART_Transmit('\r');
	break;
	
	case '2':
	myChip.setGreyScale(3 , 0x000F , 0x000F, 0x000F);
	myChip.setGreyScale(2 , 0x000F , 0x000F, 0x000F);
	USART_Transmit('2');
	USART_Transmit('\n');
	USART_Transmit('\r');
	break;
	
	case '3':
	myChip.setGreyScale(3 , 0x00FF , 0x00FF, 0x00FF);
	myChip.setGreyScale(2 , 0x00FF , 0x00FF, 0x00FF);
	USART_Transmit('3');
	USART_Transmit('\n');
	USART_Transmit('\r');
	break;
	
	case '4':
	myChip.setGreyScale(3 , 0 , 0xFFFF, 0);
	myChip.setGreyScale(2 , 0 , 0xFFFF, 0);
	USART_Transmit('4');
	USART_Transmit('\n');
	USART_Transmit('\r');
	break;
	
	case '5':
	myChip.setGreyScale(3 , 0xFFFF , 0 , 0);
	myChip.setGreyScale(2 , 0xFFFF , 0 , 0);
	USART_Transmit('5');
	USART_Transmit('\n');
	USART_Transmit('\r');
	break;
	
	default:
	USART_Transmit('E');
	USART_Transmit('R');
	USART_Transmit('R');
	USART_Transmit('O');
	USART_Transmit('R');
	USART_Transmit('\n');
	USART_Transmit('\r');
	break;
}*/