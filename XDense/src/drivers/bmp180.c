/*
	SFE_BMP180.cpp
	Bosch BMP180 pressure sensor library for the Arduino microcontroller
	Mike Grusin, SparkFun Electronics

	Uses floating-point equations from the Weather Station Data Logger project
	http://wmrx00.sourceforge.net/
	http://wmrx00.sourceforge.net/Arduino/BMP085-Calcs.pdf

	Forked from BMP085 library by M.Grusin

	version 1.0 2013/09/20 initial version
	Verison 1.1.2 - Updated for Arduino 1.6.4 5/2015

	Our example code uses the "beerware" license. You can do anything
	you like with this code. No really, anything. If you find it useful,
	buy me a (root) beer someday.
*/


#include "bmp180.h"



char SFE_BMP180_begin(){
	double c3,c4,b1;
	
	// Start up the Arduino's "wire" (I2C) library:
	
	// The BMP180 includes factory calibration data stored on the device.
	// Each device has different numbers, these must be retrieved and
	// used in the calculations when taking pressure measurements.

	// Retrieve calibration data from device:
	
	i2c_pck_bmp180.chip = BMP180_ADDR;
	i2c_pck_bmp180.buffer = data_buffer_bmp180;
	
	SFE_BMP180_readInt(0xAA,NULL); //fix: cleanup i2c, otherwise got wrong values (idk why...) - Pedro
	
	if (SFE_BMP180_readInt(0xAA,&AC1) &&
		SFE_BMP180_readInt(0xAC,&AC2) &&
		SFE_BMP180_readInt(0xAE,&AC3) &&
		SFE_BMP180_readUInt(0xB0,&AC4) &&
		SFE_BMP180_readUInt(0xB2,&AC5) &&
		SFE_BMP180_readUInt(0xB4,&AC6) &&
		SFE_BMP180_readInt(0xB6,&VB1) &&
		SFE_BMP180_readInt(0xB8,&VB2) &&
		SFE_BMP180_readInt(0xBA,&MB) &&
		SFE_BMP180_readInt(0xBC,&MC) &&
		SFE_BMP180_readInt(0xBE,&MD))
	{

		// All reads completed successfully!

		// If you need to check your math using known numbers,
		// you can uncomment one of these examples.
		// (The correct results are commented in the below functions.)

		// Example from Bosch datasheet
		// AC1 = 408; AC2 = -72; AC3 = -14383; AC4 = 32741; AC5 = 32757; AC6 = 23153;
		// B1 = 6190; B2 = 4; MB = -32768; MC = -8711; MD = 2868;

		// Example from http://wmrx00.sourceforge.net/Arduino/BMP180-Calcs.pdf
		// AC1 = 7911; AC2 = -934; AC3 = -14306; AC4 = 31567; AC5 = 25671; AC6 = 18974;
		// VB1 = 5498; VB2 = 46; MB = -32768; MC = -11075; MD = 2432;

		/*
		Serial.print("AC1: "); Serial.println(AC1);
		Serial.print("AC2: "); Serial.println(AC2);
		Serial.print("AC3: "); Serial.println(AC3);
		Serial.print("AC4: "); Serial.println(AC4);
		Serial.print("AC5: "); Serial.println(AC5);
		Serial.print("AC6: "); Serial.println(AC6);
		Serial.print("VB1: "); Serial.println(VB1);
		Serial.print("VB2: "); Serial.println(VB2);
		Serial.print("MB: "); Serial.println(MB);
		Serial.print("MC: "); Serial.println(MC);
		Serial.print("MD: "); Serial.println(MD);
		*/
		
		// Compute floating-point polynominals:

		c3 = 160.0 * pow(2,-15) * AC3;
		c4 = pow(10,-3) * pow(2,-15) * AC4;
		b1 = pow(160,2) * pow(2,-30) * VB1;
		c5 = (pow(2,-15) / 160) * AC5;
		c6 = AC6;
		mc = (pow(2,11) / pow(160,2)) * MC;
		md = MD / 160.0;
		x0 = AC1;
		x1 = 160.0 * pow(2,-13) * AC2;
		x2 = pow(160,2) * pow(2,-25) * VB2;
		y00 = c4 * pow(2,15);
		y01 = c4 * c3;
		y02 = c4 * b1;
		p0 = (3791.0 - 8.0) / 1600.0;
		p1 = 1.0 - 7357.0 * pow(2,-20);
		p2 = 3038.0 * 100.0 * pow(2,-36);

		/*
		Serial.println();
		Serial.print("c3: "); Serial.println(c3);
		Serial.print("c4: "); Serial.println(c4);
		Serial.print("c5: "); Serial.println(c5);
		Serial.print("c6: "); Serial.println(c6);
		Serial.print("b1: "); Serial.println(b1);
		Serial.print("mc: "); Serial.println(mc);
		Serial.print("md: "); Serial.println(md);
		Serial.print("x0: "); Serial.println(x0);
		Serial.print("x1: "); Serial.println(x1);
		Serial.print("x2: "); Serial.println(x2);
		Serial.print("y00: "); Serial.println(y00);
		Serial.print("y01: "); Serial.println(y01);
		Serial.print("y02: "); Serial.println(y02);
		Serial.print("p0: "); Serial.println(p0);
		Serial.print("p1: "); Serial.println(p1);
		Serial.print("p2: "); Serial.println(p2);
		*/
		
		// Success!
		return(1);
	}
	else
	{
		// Error reading calibration data; bad component or connection?
		return(0);
	}
}





char SFE_BMP180_readInt(char address, int16_t * value)
// Read a signed integer (two bytes) from device
// address: register to start reading (plus subsequent register)
// value: external variable to store data (function modifies value)
{
	unsigned char data[2];

	data[0] = address;
	if (SFE_BMP180_readBytes(data,2))
	{
		*value = (int16_t)((data[0]<<8)|data[1]);
		//if (*value & 0x8000) *value |= 0xFFFF0000; // sign extend if negative
		return(1);
	}
	*value = 0;
	return(0);
}


char SFE_BMP180_readUInt(char address, uint16_t * value)
// Read an unsigned integer (two bytes) from device
// address: register to start reading (plus subsequent register)
// value: external variable to store data (function modifies value)
{
	unsigned char data[2];

	data[0] = address;
	if (SFE_BMP180_readBytes(data,2))
	{
		*value = (((uint16_t)data[0]<<8)|(uint16_t)data[1]);
		return(1);
	}
	*value = 0;
	return(0);
}




char SFE_BMP180_readBytes(unsigned char *values, char length)
// Read an array of bytes from device
// values: external array to hold data. Put starting register in values[0].
// length: number of bytes to read
{
		char x;
		const portTickType max_block_time_ticks = (portTickType) (200UL / portTICK_RATE_MS) * portTICK_RATE_MS_LESS_1_MS;
		
		i2c_pck_bmp180.length = (uint32_t) length;
		i2c_pck_bmp180.addr[0] = (uint8_t) values[0];
		i2c_pck_bmp180.addr_length = 1;
		// Attempt to write the data to the port referenced by the freertos_twi
		// handle. Wait a maximum of 200ms to get exclusive access to the port
		// (other FreeRTOS tasks will execute during any waiting time).
		if(freertos_twi_read_packet(freertos_twi, &i2c_pck_bmp180, max_block_time_ticks) != STATUS_OK)
		{
			// The data was not written successfully, either because there was
			// an error on the TWI bus, or because exclusive access to the TWI
			// port was not obtained within 200ms.
			x = 0;
		}
		else{
			for(x=0;x<length;x++)
			{
				values[x] = data_buffer_bmp180[x];
			}
			return(1);
		}
		
		return(0);

}


char SFE_BMP180_writeBytes(unsigned char *values, char length)
// Write an array of bytes to device
// values: external array of data to write. Put starting register in values[0].
// length: number of bytes to write
{
	
		const portTickType max_block_time_ticks = (portTickType) ( 200UL / portTICK_RATE_MS) * portTICK_RATE_MS_LESS_1_MS;
		
		data_buffer_bmp180[0]=(uint8_t)  (values[1]);
		i2c_pck_bmp180.length = 1;
		i2c_pck_bmp180.addr[0] = (uint8_t) values[0];
		i2c_pck_bmp180.addr_length = 1;
		// Attempt to write the data to the port referenced by the freertos_twi
		// handle. Wait a maximum of 200ms to get exclusive access to the port
		// (other FreeRTOS tasks will execute during any waiting time).
		if(freertos_twi_write_packet(freertos_twi, &i2c_pck_bmp180, max_block_time_ticks) != STATUS_OK)
		{
			// The data was not written successfully, either because there was
			// an error on the TWI bus, or because exclusive access to the TWI
			// port was not obtained within 200ms.
			return(0);
		}
		else{
			return(1);
		}		
}


char SFE_BMP180_startTemperature(void)
// Begin a temperature reading.
// Will return delay in ms to wait, or 0 if I2C error
{
	unsigned char data[2], result;
	
	data[0] = BMP180_REG_CONTROL;
	data[1] = BMP180_COMMAND_TEMPERATURE;
	result = SFE_BMP180_writeBytes(data, 2);
	if (result) // good write?
		return(5); // return the delay in ms (rounded up) to wait before retrieving data
	else
		return(0); // or return 0 if there was a problem communicating with the BMP
}


char SFE_BMP180_getTemperature(double *T)
// Retrieve a previously-started temperature reading.
// Requires begin() to be called once prior to retrieve calibration parameters.
// Requires startTemperature() to have been called prior and sufficient time elapsed.
// T: external variable to hold result.
// Returns 1 if successful, 0 if I2C error.
{
	unsigned char data[2];
	char result;
	double tu, a;
	
	data[0] = BMP180_REG_RESULT;

	result = SFE_BMP180_readBytes(data, 2);
	if (result) // good read, calculate temperature
	{
		tu = (data[0] * 256.0) + data[1];

		//example from Bosch datasheet
		//tu = 27898;

		//example from http://wmrx00.sourceforge.net/Arduino/BMP085-Calcs.pdf
		//tu = 0x69EC;
		
		a = c5 * (tu - c6);
		*T = (a + (mc / (a + md)));

		/*		
		Serial.println();
		Serial.print("tu: "); Serial.println(tu);
		Serial.print("a: "); Serial.println(a);
		Serial.print("T: "); Serial.println(*T);
		*/
	}
	return(result);
}

char SFE_BMP180_getTemperature_float(float *T)
// Retrieve a previously-started temperature reading.
// Requires begin() to be called once prior to retrieve calibration parameters.
// Requires startTemperature() to have been called prior and sufficient time elapsed.
// T: external variable to hold result.
// Returns 1 if successful, 0 if I2C error.
{
	unsigned char data[2];
	char result;
	double tu, a;
	
	data[0] = BMP180_REG_RESULT;

	result = SFE_BMP180_readBytes(data, 2);
	if (result) // good read, calculate temperature
	{
		tu = (data[0] * 256.0) + data[1];

		//example from Bosch datasheet
		//tu = 27898;

		//example from http://wmrx00.sourceforge.net/Arduino/BMP085-Calcs.pdf
		//tu = 0x69EC;
		
		a = c5 * (tu - c6);
		*T = (float) (a + (mc / (a + md)));

		/*		
		Serial.println();
		Serial.print("tu: "); Serial.println(tu);
		Serial.print("a: "); Serial.println(a);
		Serial.print("T: "); Serial.println(*T);
		*/
	}
	return(result);
}


char SFE_BMP180_startPressure(char oversampling)
// Begin a pressure reading.
// Oversampling: 0 to 3, higher numbers are slower, higher-res outputs.
// Will return delay in ms to wait, or 0 if I2C error.
{
	unsigned char data[2], result, delay;
	
	data[0] = BMP180_REG_CONTROL;

	switch (oversampling)
	{
		case 0:
			data[1] = BMP180_COMMAND_PRESSURE0;
			delay = 5;
		break;
		case 1:
			data[1] = BMP180_COMMAND_PRESSURE1;
			delay = 8;
		break;
		case 2:
			data[1] = BMP180_COMMAND_PRESSURE2;
			delay = 14;
		break;
		case 3:
			data[1] = BMP180_COMMAND_PRESSURE3;
			delay = 26;
		break;
		default:
			data[1] = BMP180_COMMAND_PRESSURE0;
			delay = 5;
		break;
	}
	result = SFE_BMP180_writeBytes(data, 2);
	if (result) // good write?
		return(delay); // return the delay in ms (rounded up) to wait before retrieving data
	else
		return(0); // or return 0 if there was a problem communicating with the BMP
}


char SFE_BMP180_getPressure(double * P, double * T)
// Retrieve a previously started pressure reading, calculate abolute pressure in mbars.
// Requires begin() to be called once prior to retrieve calibration parameters.
// Requires startPressure() to have been called prior and sufficient time elapsed.
// Requires recent temperature reading to accurately calculate pressure.

// P: external variable to hold pressure.
// T: previously-calculated temperature.
// Returns 1 for success, 0 for I2C error.

// Note that calculated pressure value is absolute mbars, to compensate for altitude call sealevel().
{
	unsigned char data[3];
	char result;
	double pu,s,x,y,z;
	
	double aux = 0;
	
	data[0] = BMP180_REG_RESULT;

	result = SFE_BMP180_readBytes(data, 3);
	if (result) // good read, calculate pressure
	{
		pu = (data[0] * 256.0) + data[1] + (data[2]/256.0);

		//example from Bosch datasheet
		//pu = 23843;

		//example from http://wmrx00.sourceforge.net/Arduino/BMP085-Calcs.pdf, pu = 0x982FC0;	
		//pu = (0x98 * 256.0) + 0x2F + (0xC0/256.0);
		
		s = *T - 25.0;
		x = (x2 * pow(s,2)) + (x1 * s) + x0;
		y = (y02 * pow(s,2)) + (y01 * s) + y00;
		z = (pu - x) / y;
		aux = (p2 * pow(z,2)) + (p1 * z) + p0;
		*P = aux;
		
		/*
		Serial.println();
		Serial.print("pu: "); Serial.println(pu);
		Serial.print("T: "); Serial.println(*T);
		Serial.print("s: "); Serial.println(s);
		Serial.print("x: "); Serial.println(x);
		Serial.print("y: "); Serial.println(y);
		Serial.print("z: "); Serial.println(z);
		Serial.print("P: "); Serial.println(*P);
		*/
	}
	return(result);
}

char SFE_BMP180_getPressure_float(float * P, float * T)
// Retrieve a previously started pressure reading, calculate abolute pressure in mbars.
// Requires begin() to be called once prior to retrieve calibration parameters.
// Requires startPressure() to have been called prior and sufficient time elapsed.
// Requires recent temperature reading to accurately calculate pressure.

// P: external variable to hold pressure.
// T: previously-calculated temperature.
// Returns 1 for success, 0 for I2C error.

// Note that calculated pressure value is absolute mbars, to compensate for altitude call sealevel().
{
	unsigned char data[3];
	char result;
	double pu,s,x,y,z;
	
	double aux = 0;
	
	data[0] = BMP180_REG_RESULT;

	result = SFE_BMP180_readBytes(data, 3);
	if (result) // good read, calculate pressure
	{
		pu = (data[0] * 256.0) + data[1] + (data[2]/256.0);

		//example from Bosch datasheet
		//pu = 23843;

		//example from http://wmrx00.sourceforge.net/Arduino/BMP085-Calcs.pdf, pu = 0x982FC0;	
		//pu = (0x98 * 256.0) + 0x2F + (0xC0/256.0);
		
		s = *T - 25.0;
		x = (x2 * pow(s,2)) + (x1 * s) + x0;
		y = (y02 * pow(s,2)) + (y01 * s) + y00;
		z = (pu - x) / y;
		aux = (p2 * pow(z,2)) + (p1 * z) + p0;
		*P = (float) aux;
		
		/*
		Serial.println();
		Serial.print("pu: "); Serial.println(pu);
		Serial.print("T: "); Serial.println(*T);
		Serial.print("s: "); Serial.println(s);
		Serial.print("x: "); Serial.println(x);
		Serial.print("y: "); Serial.println(y);
		Serial.print("z: "); Serial.println(z);
		Serial.print("P: "); Serial.println(*P);
		*/
	}
	return(result);
}



double SFE_BMP180_sealevel(double P, double A)
// Given a pressure P (mb) taken at a specific altitude (meters),
// return the equivalent pressure (mb) at sea level.
// This produces pressure readings that can be used for weather measurements.
{
	return(P/pow(1-(A/44330.0),5.255));
}


double SFE_BMP180_altitude(double P, double P0)
// Given a pressure measurement P (mb) and the pressure at a baseline P0 (mb),
// return altitude (meters) above baseline.
{
	return(44330.0*(1-pow(P/P0,1/5.255)));
}


