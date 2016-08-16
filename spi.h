/*
 * IncFile1.h
 *
 * Created: 09.04.2016 23:45:45
 *  Author: holzi
 */ 


#ifndef SPI_H_
#define SPI_H_

#include <avr/io.h>
#include "inttypes.h"


void SPI_init(void);
void SPI_transfairnb(uint8_t* cData, int len);
char SPI_transfairb(char cData);


#endif /* SPI_H_ */