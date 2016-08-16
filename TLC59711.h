/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   TLC59711.h
 * Author: holzi
 *
 * Created on 27. März 2016, 10:32
 */

#ifndef TLC59711_H
#define TLC59711_H

#ifndef F_CPU
#define F_CPU 16000000UL // 16 MHz clock speed
#endif

#include "spi.h"
#include <util/delay.h>

#define RED     0
#define GREEN   1
#define BLUE    2

struct control{
    uint8_t write_cmd   : 6;    //

    uint8_t OUTMMG         : 1;    //
    uint8_t EXTGCK         : 1;    //
    uint8_t TMGRST         : 1;    //
    uint8_t DSPRPT         : 1;    //
    uint8_t BLANK          : 1;    //  
    
    uint8_t BCb         : 7;    //Global Brightness Control for OUTB0-3 (7Bit)
    uint8_t BCg         : 7;    //Global Brightness Control for OUTG0-3 (7Bit)
    uint8_t BCr         : 7;    //Global Brightness Control for OUTR0-3 (7Bit)  
};

class TLC59711 {
public:
    TLC59711(bool OUTMMG = 1,
             bool EXTGCK = 0,
             bool TMGRST = 0,
             bool DSPRPT = 1);
    
    //virtual ~TLC59711();
    
    void setGreyScale(uint8_t RGBgroup, uint16_t GSr, uint16_t GSg, uint16_t GSb);      //RGBgroup = one of the 4 RGB Outputs
                                                                                        //GSr = Grayscale control red
                                                                                        //GSr = Grayscale control green
                                                                                        //GSr = Grayscale control blue
    
    void dimmDown(int time = 2,  uint16_t GS_max = 0xFFFF,	//PFUSCH FUNKTION
                                 uint8_t RGBgroup = 0); 
    
    void setGlobalBrightness(uint8_t color, uint8_t value);                              //color = The color to dim (red,green,blue)
   
private:
    void write_to_chip();
    
    struct control control_data; //Stores the control information and Global brightness
    uint16_t GS[12];             //Stores the Greyscale information for all 12 Channels
    uint8_t m_buffer[28];        //SPI send buffer
};



#endif /* TLC59711_H */

