///*
    //TMP Test Code
	//5-31-10
    //Copyright Spark Fun Electronics© 2010
    //Nathan Seidle
	//
	//Example code for the TMP102 11-bit I2C temperature sensor
	//
	//You will need to connect the ADR0 pin to one of four places. This code assumes ADR0 is tied to VCC. 
	//This results in an I2C address of 0x93 for read, and 0x92 for write.
	//
	//This code assumes regular 12 bit readings. If you want the extended mode for higher temperatures, the code
	//will have to be modified slightly.
//
//*/
//#include <stdio.h>
//#include <avr/io.h>
//#include "i2c.h" //Needed  for I2C sensors
//
//#define FALSE	0
//#define TRUE	-1
//
////#define FOSC 16000000 //16MHz external osc
////
////#define SERIAL_BAUD 9600
////#define SERIAL_MYUBRR (((((FOSC * 10) / (16L * SERIAL_BAUD)) + 5) / 10) - 1)
//
////Function definitions
////=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//void sens_t_ioinit(void);
//
//int16_t sens_t_tmp102Read(void);
//
////=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//
////Global variables
////=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//static FILE mystdout = FDEV_SETUP_STREAM(uart_putchar, NULL, _FDEV_SETUP_WRITE);
////=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//


#ifndef TMP102_DRIVER_H_
#define TMP102_DRIVER_H_

#include "asf.h"
#include "drivers/twi.h"

#define SENS_TEMP_ADDR		0b1001000 // = 72 ->Assume ADR0 is tied to GND
#define SENS_TEMP_REG 		0x00
#define SENS_CONFIG_REG 	0x01

twi_packet_t i2c_pck;
uint8_t data_buffer[5];

int16_t sens_t_tmp102_read(void);

int16_t sens_t_tmp102_setup(void);


#endif