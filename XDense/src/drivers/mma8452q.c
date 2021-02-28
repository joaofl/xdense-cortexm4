/******************************************************************************
SparkFun_MMA8452Q.cpp
SparkFun_MMA8452Q Library Source File
Jim Lindblom @ SparkFun Electronics
Original Creation Date: June 3, 2014
https://github.com/sparkfun/MMA8452_Accelerometer

This file implements all functions of the MMA8452Q class. Functions here range
from higher level stuff, like reading/writing MMA8452Q registers to low-level,
hardware I2C reads and writes.

Development environment specifics:
	IDE: Arduino 1.0.5
	Hardware Platform: Arduino Uno

	**Updated for Arduino 1.6.4 5/2015**
	
This code is beerware; if you see me (or any other SparkFun employee) at the
local, and you've found our code helpful, please buy us a round!

Distributed as-is; no warranty is given.
******************************************************************************/


#include "mma8452q.h"



// INITIALIZATION
//	This function initializes the MMA8452Q. It sets up the scale (either 2, 4,
//	or 8g), output data rate, portrait/landscape detection and tap detection.
//	It also checks the WHO_AM_I register to make sure we can communicate with
//	the sensor. Returns a 0 if communication failed, 1 if successful.
uint8_t MMA8452Q_init(MMA8452Q_Scale fsr, MMA8452Q_ODR odr)
{
	address = 0x1C;
	scale = fsr; // Haul fsr into our class variable, scale
	

	i2c_pck_mma8452q.chip = address;
	i2c_pck_mma8452q.buffer = data_buffer_mma8452q;
	
	uint8_t c = MMA8452Q_readRegister(WHO_AM_I);  // Read WHO_AM_I register
	
	if (c != 0x2A) // WHO_AM_I should always be 0x2A
	{
		return 0;
	}
	
	MMA8452Q_standby();  // Must be in standby to change registers
	
	MMA8452Q_setScale(scale);  // Set up accelerometer scale
	MMA8452Q_setODR(odr);  // Set up output data rate
	MMA8452Q_setupPL();  // Set up portrait/landscape detection
	// Multiply parameter by 0.0625g to calculate threshold.
	MMA8452Q_setupTap(0x80, 0x80, 0x08); // Disable x, y, set z to 0.5g
	
	MMA8452Q_active();  // Set to active to start reading
	
	return 1;
}

// READ ACCELERATION DATA
//  This function will read the acceleration values from the MMA8452Q. After
//	reading, it will update two triplets of variables:
//		* int's x, y, and z will store the signed 12-bit values read out
//		  of the acceleromter.
//		* floats cx, cy, and cz will store the calculated acceleration from
//		  those 12-bit values. These variables are in units of g's.
void MMA8452Q_read()
{
	uint8_t rawData[6];  // x/y/z accel register data stored here

	MMA8452Q_readRegisters(OUT_X_MSB, rawData, 6);  // Read the six raw data registers into data array
	
	MMA8452Q_x = ((short)(rawData[0]<<8 | rawData[1])) >> 4;
	MMA8452Q_y = ((short)(rawData[2]<<8 | rawData[3])) >> 4;
	MMA8452Q_z = ((short)(rawData[4]<<8 | rawData[5])) >> 4;
	MMA8452Q_cx = (float) MMA8452Q_x / (float)(1<<11) * (float)(scale);
	MMA8452Q_cy = (float) MMA8452Q_y / (float)(1<<11) * (float)(scale);
	MMA8452Q_cz = (float) MMA8452Q_z / (float)(1<<11) * (float)(scale);
}

// CHECK IF NEW DATA IS AVAILABLE
//	This function checks the status of the MMA8452Q to see if new data is availble.
//	returns 0 if no new data is present, or a 1 if new data is available.
uint8_t MMA8452Q_available()
{
	return (MMA8452Q_readRegister(STATUS) & 0x08) >> 3;
}

// SET FULL-SCALE RANGE
//	This function sets the full-scale range of the x, y, and z axis accelerometers.
//	Possible values for the fsr variable are SCALE_2G, SCALE_4G, or SCALE_8G.
void MMA8452Q_setScale(MMA8452Q_Scale fsr)
{
	// Must be in standby mode to make changes!!!
	uint8_t cfg = MMA8452Q_readRegister(XYZ_DATA_CFG);
	cfg &= 0xFC; // Mask out scale bits
	cfg |= (fsr >> 2);  // Neat trick, see page 22. 00 = 2G, 01 = 4A, 10 = 8G
	MMA8452Q_writeRegister(XYZ_DATA_CFG, cfg);
}

// SET THE OUTPUT DATA RATE
//	This function sets the output data rate of the MMA8452Q.
//	Possible values for the odr parameter are: ODR_800, ODR_400, ODR_200, 
//	ODR_100, ODR_50, ODR_12, ODR_6, or ODR_1
void MMA8452Q_setODR(MMA8452Q_ODR odr)
{
	// Must be in standby mode to make changes!!!
	uint8_t ctrl = MMA8452Q_readRegister(CTRL_REG1);
	ctrl &= 0xCF; // Mask out data rate bits
	ctrl |= (odr << 3);
	MMA8452Q_writeRegister(CTRL_REG1, ctrl);
}

// SET UP TAP DETECTION
//	This function can set up tap detection on the x, y, and/or z axes.
//	The xThs, yThs, and zThs parameters serve two functions:
//		1. Enable tap detection on an axis. If the 7th bit is SET (0x80)
//			tap detection on that axis will be DISABLED.
//		2. Set tap g's threshold. The lower 7 bits will set the tap threshold
//			on that axis.
void MMA8452Q_setupTap(uint8_t xThs, uint8_t yThs, uint8_t zThs)
{
	// Set up single and double tap - 5 steps:
	// for more info check out this app note:
	// http://cache.freescale.com/files/sensors/doc/app_note/AN4072.pdf
	// Set the threshold - minimum required acceleration to cause a tap.
	uint8_t temp = 0;
	if (!(xThs & 0x80)) // If top bit ISN'T set
	{
		temp |= 0x3; // Enable taps on x
		MMA8452Q_writeRegister(PULSE_THSX, xThs);  // x thresh
	}
	if (!(yThs & 0x80))
	{
		temp |= 0xC; // Enable taps on y
		MMA8452Q_writeRegister(PULSE_THSY, yThs);  // y thresh
	}
	if (!(zThs & 0x80))
	{
		temp |= 0x30; // Enable taps on z
		MMA8452Q_writeRegister(PULSE_THSZ, zThs);  // z thresh
	}
	// Set up single and/or double tap detection on each axis individually.
	MMA8452Q_writeRegister(PULSE_CFG, temp | 0x40);
	// Set the time limit - the maximum time that a tap can be above the thresh
	MMA8452Q_writeRegister(PULSE_TMLT, 0x30);  // 30ms time limit at 800Hz odr
	// Set the pulse latency - the minimum required time between pulses
	MMA8452Q_writeRegister(PULSE_LTCY, 0xA0);  // 200ms (at 800Hz odr) between taps min
	// Set the second pulse window - maximum allowed time between end of
	//	latency and start of second pulse
	MMA8452Q_writeRegister(PULSE_WIND, 0xFF);  // 5. 318ms (max value) between taps max
}

// READ TAP STATUS
//	This function returns any taps read by the MMA8452Q. If the function 
//	returns no new taps were detected. Otherwise the function will return the
//	lower 7 bits of the PULSE_SRC register.
uint8_t MMA8452Q_readTap()
{
	uint8_t tapStat = MMA8452Q_readRegister(PULSE_SRC);
	if (tapStat & 0x80) // Read EA bit to check if a interrupt was generated
	{
		return tapStat & 0x7F;
	}
	else
		return 0;
}

// SET UP PORTRAIT/LANDSCAPE DETECTION
//	This function sets up portrait and landscape detection.
void MMA8452Q_setupPL()
{
	// Must be in standby mode to make changes!!!
	// For more info check out this app note:
	//	http://cache.freescale.com/files/sensors/doc/app_note/AN4068.pdf
	// 1. Enable P/L
	MMA8452Q_writeRegister(PL_CFG, MMA8452Q_readRegister(PL_CFG) | 0x40); // Set PL_EN (enable)
	// 2. Set the debounce rate
	MMA8452Q_writeRegister(PL_COUNT, 0x50);  // Debounce counter at 100ms (at 800 hz)
}

// READ PORTRAIT/LANDSCAPE STATUS
//	This function reads the portrait/landscape status register of the MMA8452Q.
//	It will return either PORTRAIT_U, PORTRAIT_D, LANDSCAPE_R, LANDSCAPE_L,
//	or LOCKOUT. LOCKOUT indicates that the sensor is in neither p or ls.
uint8_t MMA8452Q_readPL()
{
	uint8_t plStat = MMA8452Q_readRegister(PL_STATUS);
	
	if (plStat & 0x40) // Z-tilt lockout
		return LOCKOUT;
	else // Otherwise return LAPO status
		return (plStat & 0x6) >> 1;
}

// SET STANDBY MODE
//	Sets the MMA8452 to standby mode. It must be in standby to change most register settings
void MMA8452Q_standby()
{
	uint8_t c = MMA8452Q_readRegister(CTRL_REG1);
	MMA8452Q_writeRegister(CTRL_REG1, c & ~(0x01)); //Clear the active bit to go into standby
}

// SET ACTIVE MODE
//	Sets the MMA8452 to active mode. Needs to be in this mode to output data
void MMA8452Q_active()
{
	uint8_t c = MMA8452Q_readRegister(CTRL_REG1);
	MMA8452Q_writeRegister(CTRL_REG1, c | 0x01); //Set the active bit to begin detection
}

// WRITE A SINGLE REGISTER
// 	Write a single byte of data to a register in the MMA8452Q.
void MMA8452Q_writeRegister(MMA8452Q_Register reg, uint8_t data)
{
	MMA8452Q_writeRegisters(reg, &data, 1);
}

// WRITE MULTIPLE REGISTERS
//	Write an array of "len" bytes ("buffer"), starting at register "reg", and
//	auto-incrementing to the next.
void MMA8452Q_writeRegisters(MMA8452Q_Register reg, uint8_t *buffer, uint8_t len)
{
	int x;
	const portTickType max_block_time_ticks = (portTickType) (200UL / portTICK_RATE_MS) * portTICK_RATE_MS_LESS_1_MS;
			
	
	for(x=0; x<len; x++){
		data_buffer_mma8452q[x]=buffer[x];
	}
	i2c_pck_mma8452q.length = len;
	i2c_pck_mma8452q.addr[0] = reg;
	i2c_pck_mma8452q.addr_length = 1;
	// Attempt to write the data to the port referenced by the freertos_twi
	// handle. Wait a maximum of 200ms to get exclusive access to the port
	// (other FreeRTOS tasks will execute during any waiting time).
	if(freertos_twi_write_packet(freertos_twi, &i2c_pck_mma8452q, max_block_time_ticks) != STATUS_OK)
	{
		// The data was not written successfully, either because there was
		// an error on the TWI bus, or because exclusive access to the TWI
		// port was not obtained within 200ms.
	}
	else{
	}
	return;
	
}

// READ A SINGLE REGISTER
//	Read a uint8_t from the MMA8452Q register "reg".
uint8_t MMA8452Q_readRegister(MMA8452Q_Register reg)
{
	const portTickType max_block_time_ticks = (portTickType) ( 200UL / portTICK_RATE_MS)* portTICK_RATE_MS_LESS_1_MS;
	
	i2c_pck_mma8452q.length = (uint32_t) 1;
	i2c_pck_mma8452q.addr[0] = (uint8_t) reg;
	i2c_pck_mma8452q.addr_length = 1;
	
	if(freertos_twi_read_packet(freertos_twi, &i2c_pck_mma8452q, max_block_time_ticks) != STATUS_OK)
	{
		// The data was not written successfully, either because there was
		// an error on the TWI bus, or because exclusive access to the TWI
		// port was not obtained within 200ms.
		return 0;
	} else{
		return data_buffer_mma8452q[0];
	}
	return;
	
}

// READ MULTIPLE REGISTERS
//	Read "len" bytes from the MMA8452Q, starting at register "reg". Bytes are stored
//	in "buffer" on exit.
void MMA8452Q_readRegisters(MMA8452Q_Register reg, uint8_t *buffer, uint8_t len)
{
	char x;
	const portTickType max_block_time_ticks = (portTickType) ( 200UL / portTICK_RATE_MS) * portTICK_RATE_MS_LESS_1_MS;
	
	i2c_pck_mma8452q.length = (uint32_t) len;
	i2c_pck_mma8452q.addr[0] = (uint8_t) reg;
	i2c_pck_mma8452q.addr_length = 1;
	
	if(freertos_twi_read_packet(freertos_twi, &i2c_pck_mma8452q, max_block_time_ticks) != STATUS_OK)
	{
		// The data was not written successfully, either because there was
		// an error on the TWI bus, or because exclusive access to the TWI
		// port was not obtained within 200ms.
		x = 0;
	}
	else{
		for(x=0;x<len;x++)
		{
			buffer[x] = data_buffer_mma8452q[x];
		}
	}
	return;    
}