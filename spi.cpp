/*
 * CPPFile1.cpp
 *
 * Created: 09.04.2016 23:44:36
 *  Author: holzi
 */ 

#include "spi.h"

void SPI_init(void)
{
	/* Set MOSI and SCK output, all others input */
	DDRB = (1<<PB3)|(1<<PB5)|(1<<PB2);		//DDRB The Port B Data Direction Register (1 = Output)
	// PB3 = 3 = MOSI ; PB5 = 5 = SCK ; PB2 = SS
	
	/* Enable SPI, set Master, set clock rate fclk/4 */
	SPCR = (1<<SPE)|(1<<MSTR)|(0<<SPR0);
	
	//Set SS to high (not active)
	PORTB |= (1 << PB2);
}


void SPI_transfairnb(uint8_t* cData, int len)
{
	/* Start transmission */
	PORTB &= ~(1<<PB2);								// set SS to low
	
	for (int i = 0 ; i < len ; i++)
	{
		SPDR = *(cData + i);						//SPDR = SPI Data Register
		
		/* Wait for transmission complete */
		while(!(SPSR & (1<<SPIF)));					// SPIF = SPI Interrupt Flag, gets set if the transmit is completed
		*(cData + i) = SPDR;						// cleared by first reading the SPI Status Register with SPIF set, then accessing the SPDR
	}
	
	PORTB |= (1<<PB2);								// set SS to high
}

char SPI_transfairb(char cData)
{
	/* Start transmission */
	PORTB &= ~(1 << PB2);						// set SS to low
	
	
	SPDR = cData;								//SPDR = SPI Data Register
	/* Wait for transmission complete */
	while(!(SPSR & (1<<SPIF) ));				// SPIF = SPI Interrupt Flag, gets set if the transmit is completed
	
	
	PORTB |= (1<<PB2);							// set SS to high
	return (SPDR);								// cleared by first reading the SPI Status Register with SPIF set, then accessing the SPDR
}