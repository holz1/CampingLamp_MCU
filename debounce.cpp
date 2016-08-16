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

#include "debounce.h"


volatile uint8_t key_state;				// debounced and inverted key state: bit = 1: key pressed
volatile uint8_t key_press;				// key press detect
volatile uint8_t key_release;			// key release detect
volatile uint8_t key_rpt;				// key long press and repeat


uint8_t get_key_press( uint8_t key_mask )
{
	cli();								// read and clear atomic !
	key_mask &= key_press;              // read key(s)
	key_press ^= key_mask;              // clear key(s)
	sei();
	return key_mask;
}

uint8_t get_key_rpt( uint8_t key_mask )
{
	cli();								// read and clear atomic !
	key_mask &= key_rpt;                // read key(s)
	key_rpt ^= key_mask;                // clear key(s)
	sei();
	return key_mask;
}

uint8_t get_key_short( uint8_t key_mask )
{
	cli();			// read key state and key press atomic !
	return get_key_press( ~key_state & key_mask );
}

uint8_t get_key_long( uint8_t key_mask )
{
	return get_key_press( get_key_rpt( key_mask ));
}

uint8_t get_key_release( uint8_t key_mask )
{
	cli();                     // read and clear atomic !
	key_mask &= key_release;   // read key(s)
	key_release ^= key_mask;   // clear key(s)
	sei();
	return key_mask;
}

uint8_t get_key_long_r( uint8_t key_mask )      // if repeat functionality for long press needed
{
  return get_key_press( get_key_rpt( key_press & key_mask ));
}

uint8_t get_key_rpt_l( uint8_t key_mask )       // if long press function with long press repeat functionality needed
{
  return get_key_rpt( ~key_press & key_mask );
}
