/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   TLC59711.cpp
 * Author: holzi
 * 
 * Created on 27. März 2016, 10:32
 */

#ifndef F_CPU
#define F_CPU 16000000UL // 16 MHz clock speed
#endif

#include "TLC59711.h"
#include <util/delay.h>

TLC59711::TLC59711(bool OUTMMG,
                   bool EXTGCK,
                   bool TMGRST,
                   bool DSPRPT) 
{
    control_data.write_cmd = 0x25;
    
    control_data.OUTMMG = OUTMMG;
    control_data.EXTGCK = EXTGCK;
    control_data.TMGRST = TMGRST;
    control_data.DSPRPT = DSPRPT;
    control_data.BLANK  = false;
    
    //Global Brightnes control -> all high (7Bit)
    control_data.BCr = 0x7F;
    control_data.BCg = 0x7F;
    control_data.BCb = 0x7F;

    
    //Greyscale control 12 Channels -> all low
    for (int i = 0 ; i <12 ; i++)
    {
        GS[i]  = 0;
    }
}


//TLC59711::~TLC59711() {
    
    //bcm2835_spi_end();
    //bcm2835_close();
//}


void TLC59711::setGreyScale(uint8_t RGBgroup, uint16_t GSb, uint16_t GSg, uint16_t GSr)
{
    switch(RGBgroup)
    {
        case 0:
            GS[0]  = GSb;
            GS[1]  = GSg;
            GS[2]  = GSr;
            break;
        case 1:
            GS[3]  = GSb;
            GS[4]  = GSg;
            GS[5]  = GSr;
            break;
        case 2:
            GS[6]  = GSb;
            GS[7]  = GSg;
            GS[8]  = GSr;
            break;
        case 3:
            GS[9]  = GSb;
            GS[10] = GSg;
            GS[11] = GSr;
            break;
        default:
            break;
    }
    write_to_chip();
}

void TLC59711::write_to_chip()
{

    //Flags
    m_buffer[0] = (control_data.write_cmd << 2) | (control_data.OUTMMG << 1) | control_data.EXTGCK;
    m_buffer[1] = (control_data.TMGRST << 7)    | (control_data.DSPRPT << 6) | (control_data.BLANK << 5);

    //Global brightness - BC
    m_buffer[1] |= control_data.BCb >> 2;
    m_buffer[2] = (control_data.BCb << 6) | control_data.BCg >> 1;
    m_buffer[3] =  control_data.BCg << 7  | control_data.BCr;
    
    //Greyscale - GS
    int a = 11;
    for (int i = 4 ; i < 28 ; i = i+2)
    {
        m_buffer[i] = GS[a] >> 8;
        m_buffer[i+1] = (GS[a]<< 8) >> 8;
        a--;
    }
	
	SPI_transfairnb(m_buffer, 28);
	_delay_us(10);
    
    
    //    //Flags
//    m_buffer[0]  = (conrol_data.write_cmd << 10) | (conrol_data.OUTMMG << 9) | (conrol_data.EXTGCK << 8);  //Leading 8 Bits
//    m_buffer[0] |= (conrol_data.TMGRST << 7)     | (conrol_data.DSPRPT << 6) | (conrol_data.BLANK << 5);   //following 3 Bits
//
//    //Global brightness - BC
//    m_buffer[0] |= conrol_data.BCb >> 2;                                                                   //First 5 Bits of BCb
//    m_buffer[1] = (conrol_data.BCb << 14) | conrol_data.BCg << 7 | conrol_data.BCr;                   //Last  2 Bits of BCb and BCg,BCr
//
//    //Greyscale - GS
//    for (int i = 2 ; i < 14 ; i++)
//    {
//        m_buffer[i] = GS[i-2];
//    }
}

void TLC59711::dimmDown(int time, uint16_t GS_max, uint8_t RGBgroup)//PFUSCH FUNKTION
{
	int a;
	
    for (unsigned int i = 0 ; i < GS_max ; i++)
    {
	    setGreyScale(RGBgroup, GS_max - i, GS_max - i, GS_max - i);
		
		a = 0;
		while( a < (time*1e3)/GS_max)
		{
			a++;
			_delay_us(1);
		}
    }
}

void TLC59711::setGlobalBrightness(uint8_t colorGroup , uint8_t value)
{
    switch(colorGroup)
    {
        case RED  :     control_data.BCr = value;
        case BLUE :     control_data.BCb = value;
        case GREEN:     control_data.BCg = value;
        default: break;
    }
    write_to_chip();
}

void TLC59711::setSingleGS(uint8_t RGBgroup, uint8_t color, uint16_t GSval)
{
	GS[RGBgroup*3 + color] = GSval;
	write_to_chip();
}

void TLC59711::global_OFF(uint8_t MODE)
{
	for (int i = 0 ; i <12 ; i++)
		GS[i]  = 0;
	write_to_chip();
}