/*/**********************************************************************/
/*                                                                      */
/*						Debouncing 8 Keys								*/
/*						Sampling 4 Times								*/
/*						With Repeat Function							*/
/*                                                                      */
/*              Author: Peter Dannegger                                 */
/*                      danni@specs.de                                  */
/*                                                                      */
/************************************************************************/

//source: https://www.mikrocontroller.net/topic/48465?goto=1753367#1753367

/* Short discription to the functionality of the functions
Mode 1 ('get_key_press' with 'get_key_rpt')
===========================================

           time ---->
                 __      _________________      __
keystate    ____/  \____/                 \____/  \____
key_press   ----X-------X----------------------X-------
key_rpt     --------------------X--X--X--X-------------
key_release -------X----------------------X-------X----
                        |       |  |  |
                        |       |__|__|
                        |       | \ /
                        |_______|  REPEAT_NEXT
                              \
                               REPEAT_START


Mode 2 ('get_key_short' with 'get_key_long')
============================================
                 __      _________________      __
keystate    ____/  \____/                 \____/  \____
key_short   -------X------------------------------X----
key_long    --------------------X----------------------
key_release -------X----------------------X-------X----



Modus 3 ('get_key_short' with 'get_key_long_r' and 'get_key_rpt_l')
==================================================================
                 __      _________________      __
keystate    ____/  \____/                 \____/  \____
key_short   -------X------------------------------X----
key_long_r  --------------------X----------------------
key_rpt_l   -----------------------X--X--X-------------
key_release -------X----------------------X-------X----

Wichtig zu wissen ist vielleicht noch, dass 'get_key_long_r' und 
'get_key_rpt_l' immer zusammen abgefragt werden müssen,
auch wenn für ein Fall keine Verwendung ist. Für sich alleine liefern 
die Funktionen keine brauchbaren Ergebnisse (auch nicht zusammen mit 
'get_key_short').

'get_key_release' kann man in allen Modi verwenden.

Erlaubte Funktionen einer Taste:

- get_key_press()

- get_key_rpt()

- get_key_press() mit get_key_rpt()

- get_key_short() mit get_key_long()

- get_key_short() mit get_key_long_r() und get_key_rpt_l()*/


#ifndef DEBOUCE_H_
#define DEBOUCE_H_

#define F_CPU 16000000UL //16MHz

#include "inttypes.h"
#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#define KEY_PIN		PIND
#define KEY0		3		//Switch is connected to PD3
//#define KEY0		0
//#define KEY1		1
//#define KEY2		2

//#define LED_DDR		DDRB
//#define LED_PORT	PORTB
//#define LED0		0		//LED connected to PB0
//#define LED0		0
//#define LED1		1
//#define LED2		2

#define REPEAT_MASK	(1<<KEY0)					// repeat: key0
#define REPEAT_START	50						// after 500ms
#define REPEAT_NEXT	1							// every 10ms

extern volatile uint8_t key_state;				// debounced and inverted key state: bit = 1: key pressed
extern volatile uint8_t key_press;				// key press detect
extern volatile uint8_t key_release;			// key release detect
extern volatile uint8_t key_rpt;				// key long press and repeat

uint8_t get_key_press( uint8_t key_mask );		//Indicates if a key-press got deteced (no matter if long or short)
uint8_t get_key_rpt( uint8_t key_mask );		//Indicates "true" repeated if the key stays pressed down (only for use with short press)
uint8_t get_key_short( uint8_t key_mask );		//Indicates a short press (true by falling edge)
uint8_t get_key_long( uint8_t key_mask );		//Indicates a long press (true after a delay from rising edge)
uint8_t get_key_release( uint8_t key_mask );	//Indicates negative edge
uint8_t get_key_long_r( uint8_t key_mask );		//Indicates a long press with use of the repeat function for long press (works only with mutal use of get_key_rpt_l)
uint8_t get_key_rpt_l( uint8_t key_mask );		//Indicates "true" repeated if the key stays pressed down (only for use with get_key_long_r press)
												//Further discription see above
#endif /* DEBOUCE_H_ */




/*_______________________________________________________________________________________
Get-Key-Common	https://www.mikrocontroller.net/topic/48465?goto=1753367#1753367

So, jetzt noch einfacher, mit ner extra Funktion für die 2 Tasten 
Erkennung.
Man kann auch auswählen, ob der Druck beider Tasten innerhalb einer 
bestimmte Zeit erfolgen soll (#define TIMEOUT)

uint8_t get_key_common( uint8_t key_mask )
{
  return get_key_press((key_press & key_mask) == key_mask ? key_mask : 0);
}

// ...

//#define TIMEOUT

  for(;;){                                              // main loop
#ifdef TIMEOUT
    if( get_key_short( 1<<KEY0 ) || get_key_long( 1<<KEY0 ))
      LED_PORT ^= 1<<LED0;

    if( get_key_short( 1<<KEY1 ) || get_key_long( 1<<KEY1 ))
      LED_PORT ^= 1<<LED1;
#else
    if( get_key_short( 1<<KEY0 ))
      LED_PORT ^= 1<<LED0;

    if( get_key_short( 1<<KEY1 ))
      LED_PORT ^= 1<<LED1;
#endif
    if( get_key_common( 1<<KEY0 | 1<<KEY1 ))
      LED_PORT ^= 1<<LED2;
  }
  ______________________________________________________________________________________*/