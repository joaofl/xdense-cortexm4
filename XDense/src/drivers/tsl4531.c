/*
	SFE_TSL4531 illumination sensor library for Arduino
	Mike Grusin, SparkFun Electronics
	
	This library provides functions to access the TAOS TSL4531
	Illumination Sensor.
	
	Our example code uses the "beerware" license. You can do anything
	you like with this code. No really, anything. If you find it useful,
	buy me a beer someday.

	version 1.0 2013/09/20 MDG initial version
*/

#include "tsl4531.h"
//#include <Wire.h>


//SFE_TSL4531::SFE_TSL4531(void)
	//// SFE_TSL4531 object
//{}


bool sens_l_setPowerUp(void)
	// Turn on TSL4531, begin integrations
	// Returns true (1) if successful, false (0) if there was an I2C error
	// (Also see getError() below)
{
	i2c_pck_tsl4531.chip = TSL4531_ADDR;
	i2c_pck_tsl4531.buffer = data_buffer_tsl4531;
	
	// Write 0x03 to command byte (power on)
	sens_l_writeByte(TSL4531_REG_CONTROL, 0b00000011);
	
	//b3=0 skip sleep cycle; b1:0=10 100 ms sampling rate
	sens_l_writeByte(TSL4531_REG_CONFIG, 0b00001010);
	return true;
}


bool sens_l_setPowerDown(void)
	// Turn off TSL4531
	// Returns true (1) if successful, false (0) if there was an I2C error
	// (Also see getError() below)
{
	// Clear command byte (power off)
	return(sens_l_writeByte(TSL4531_REG_CONTROL,0x00));
}


//bool sens_l_setTiming(bool gain, uint8_t time)
	//// If gain = false (0), device is set to low gain (1X)
	//// If gain = high (1), device is set to high gain (16X)
	//// If time = 0, integration will be 13.7ms
	//// If time = 1, integration will be 101ms
	//// If time = 2, integration will be 402ms
	//// If time = 3, use manual start / stop
	//// Returns true (1) if successful, false (0) if there was an I2C error
	//// (Also see getError() below)
//{
	//uint8_t timing;
//
	//// Get timing byte
	//if (sens_l_readByte(TSL4531_REG_TIMING,timing))
	//{
		//// Set gain (0 or 1)
		//if (gain)
			//timing |= 0x10;
		//else
			//timing &= ~0x10;
//
		//// Set integration time (0 to 3)
		//timing &= ~0x03;
		//timing |= (time & 0x03);
//
		//// Write modified timing byte back to device
		//if (sens_l_writeByte(TSL4531_REG_TIMING,timing))
			//return(true);
	//}
	//return(false);
//}


//boolean sens_l_setTiming(boolean gain, uint8_t time, uint16_t* ms)
	//// If gain = false (0), device is set to low gain (1X)
	//// If gain = high (1), device is set to high gain (16X)
	//// If time = 0, integration will be 13.7ms
	//// If time = 1, integration will be 101ms
	//// If time = 2, integration will be 402ms
	//// If time = 3, use manual start / stop (ms = 0)
	//// ms will be set to integration time
	//// Returns true (1) if successful, false (0) if there was an I2C error
	//// (Also see getError() below)
//{
	//// Calculate ms for user
	//switch (time)
	//{
		//case 0: ms = 14; break;
		//case 1: ms = 101; break;
		//case 2: ms = 402; break;
		//default: ms = 0;
	//}
	//// Set integration using base function
	//return(setTiming(gain,time));
//}


//bool sens_l_manualStart(void)
	//// Starts a manual integration period
	//// After running this command, you must manually stop integration with manualStop()
	//// Internally sets integration time to 3 for manual integration (gain is unchanged)
	//// Returns true (1) if successful, false (0) if there was an I2C error
	//// (Also see getError() below)
//{
	//uint8_t timing;
	//
	//// Get timing byte
	//if (sens_l_readByte(TSL4531_REG_TIMING,timing))
	//{
		//// Set integration time to 3 (manual integration)
		//timing |= 0x03;
//
		//if (sens_l_writeByte(TSL4531_REG_TIMING,timing))
		//{
			//// Begin manual integration
			//timing |= 0x08;
//
			//// Write modified timing byte back to device
			//if (sens_l_writeByte(TSL4531_REG_TIMING,timing))
				//return(true);
		//}
	//}
	//return(false);
//}


//bool sens_l_manualStop(void)
	//// Stops a manual integration period
	//// Returns true (1) if successful, false (0) if there was an I2C error
	//// (Also see getError() below)
//{
	//uint8_t timing;
	//
	//// Get timing byte
	//if (sens_l_readByte(TSL4531_REG_TIMING,timing))
	//{
		//// Stop manual integration
		//timing &= ~0x08;
//
		//// Write modified timing byte back to device
		//if (sens_l_writeByte(TSL4531_REG_TIMING,timing))
			//return(true);
	//}
	//return(false);
//}


double sens_l_getData(void)
	// Retrieve raw integration results
	// data0 and data1 will be set to integration results
	// Returns true (1) if successful, false (0) if there was an I2C error
	// (Also see getError() below)
{
	// Get data0 and data1 out of result registers
	return((double) sens_l_readUInt(TSL4531_REG_DATA_0));
}


bool sens_l_getLux(uint8_t gain, uint16_t ms, uint16_t CH0, uint16_t CH1, double* lux)
	// Convert raw data to lux
	// gain: 0 (1X) or 1 (16X), see setTiming()
	// ms: integration time in ms, from setTiming() or from manual integration
	// CH0, CH1: results from getData()
	// lux will be set to resulting lux calculation
	// returns true (1) if calculation was successful
	// RETURNS false (0) AND lux = 0.0 IF EITHER SENSOR WAS SATURATED (0XFFFF)
{
	double ratio, d0, d1;

	// Determine if either sensor saturated (0xFFFF)
	// If so, abandon ship (calculation will not be accurate)
	if ((CH0 == 0xFFFF) || (CH1 == 0xFFFF))
	{
		*lux = 0.0;
		return(false);
	}

	// Convert from unsigned integer to floating point
	d0 = CH0; d1 = CH1;

	// We will need the ratio for subsequent calculations
	ratio = d1 / d0;

	// Normalize for integration time
	d0 *= (402.0/ms);
	d1 *= (402.0/ms);

	// Normalize for gain
	if (!gain)
	{
		d0 *= 16;
		d1 *= 16;
	}

	// Determine lux per datasheet equations:
	
	if (ratio < 0.5)
	{
		*lux = 0.0304 * d0 - 0.062 * d0 * pow(ratio,1.4);
		return(true);
	}

	if (ratio < 0.61)
	{
		*lux = 0.0224 * d0 - 0.031 * d1;
		return(true);
	}

	if (ratio < 0.80)
	{
		*lux = 0.0128 * d0 - 0.0153 * d1;
		return(true);
	}

	if (ratio < 1.30)
	{
		*lux = 0.00146 * d0 - 0.00112 * d1;
		return(true);
	}

	// if (ratio > 1.30)
	*lux = 0.0;
	return(true);
}


//bool sens_l_setInterruptControl(uint8_t control, uint8_t persist)
	//// Sets up interrupt operations
	//// If control = 0, interrupt output disabled
	//// If control = 1, use level interrupt, see setInterruptThreshold()
	//// If persist = 0, every integration cycle generates an interrupt
	//// If persist = 1, any value outside of threshold generates an interrupt
	//// If persist = 2 to 15, value must be outside of threshold for 2 to 15 integration cycles
	//// Returns true (1) if successful, false (0) if there was an I2C error
	//// (Also see getError() below)
//{
	//// Place control and persist bits into proper location in interrupt control register
	//if (sens_l_writeByte(TSL4531_REG_INTCTL,((control | 0B00000011) << 4) & (persist | 0B00001111)))
		//return(true);
		//
	//return(false);
//}


//bool sens_l_setInterruptThreshold(uint16_t low, uint16_t high)
	//// Set interrupt thresholds (channel 0 only)
	//// low, high: 16-bit threshold values
	//// Returns true (1) if successful, false (0) if there was an I2C error
	//// (Also see getError() below)
//{
	//// Write low and high threshold values
	//if (sens_l_writeUInt(TSL4531_REG_THRESH_L,low) && sens_l_writeUInt(TSL4531_REG_THRESH_H,high))
		//return(true);
		//
	//return(false);
//}


bool sens_l_clearInterrupt(void)
	// Clears an active interrupt
	// Returns true (1) if successful, false (0) if there was an I2C error
	// (Also see getError() below)
{
	// Set up command byte for interrupt clear
	//Wire.beginTransmission(TSL4531_ADDR);
	//Wire.write(TSL4531_CMD_CLEAR);
	//_error = Wire.endTransmission();
	//
//
	//
	//if (_error == 0)
		//return(true);

	return(false);
}


uint8_t sens_l_getID()
	// Retrieves part and revision code from TSL4531
	// Sets ID to part ID (see datasheet)
	// Returns true (1) if successful, false (0) if there was an I2C error
	// (Also see getError() below)
{
	// Get ID byte from ID register
	return ( sens_l_readByte(TSL4531_REG_ID) );
	
}


uint8_t sens_l_getError(void)
	// If any library command fails, you can retrieve an extended
	// error code using this command. Errors are from the wire library: 
	// 0 = Success
	// 1 = Data too long to fit in transmit buffer
	// 2 = Received NACK on transmit of address
	// 3 = Received NACK on transmit of data
	// 4 = Other error
{
	return(_error);
}

// Private functions:

uint8_t sens_l_readByte(uint8_t address)
	// Reads a byte from a TSL4531 address
	// Address: TSL4531 address (0 to 15)
	// Value will be set to stored byte
	// Returns true (1) if successful, false (0) if there was an I2C error
	// (Also see getError() above)
{
	// Set up command byte for read
	//Wire.beginTransmission(TSL4531_ADDR);
	//Wire.write((address & 0x0F) | TSL4531_CMD);
	//_error = Wire.endTransmission();
//
	//// Read requested byte
	//if (_error == 0)
	//{
		//Wire.requestFrom(TSL4531_ADDR,1);
		//if (Wire.available() == 1)
		//{
			//value = Wire.read();
			//return(true);
		//}
	//}
	//return(false);
	
	const portTickType max_block_time_ticks = (portTickType) (200UL / portTICK_RATE_MS) * portTICK_RATE_MS_LESS_1_MS;
	
	i2c_pck_tsl4531.length = 1;
	i2c_pck_tsl4531.addr[0] = (uint8_t) address & 0x0F;
	i2c_pck_tsl4531.addr_length = 1;
	// Attempt to write the data to the port referenced by the freertos_twi
	// handle. Wait a maximum of 200ms to get exclusive access to the port
	// (other FreeRTOS tasks will execute during any waiting time).
	if(freertos_twi_read_packet(freertos_twi, &i2c_pck_tsl4531, max_block_time_ticks) != STATUS_OK)
	{
		// The data was not written successfully, either because there was
		// an error on the TWI bus, or because exclusive access to the TWI
		// port was not obtained within 200ms.
		return(0);
	}

	return(data_buffer_tsl4531[0]);
}


bool sens_l_writeByte(uint8_t address, uint8_t value)
	// Write a byte to a TSL4531 address
	// Address: TSL4531 address (0 to 15)
	// Value: byte to write to address
	// Returns true (1) if successful, false (0) if there was an I2C error
	// (Also see getError() above)
{
	// Set up command byte for write
	//Wire.beginTransmission(TSL4531_ADDR);
	//Wire.write((address & 0x0F) | TSL4531_CMD);
	//Wire.write(value);
	//_error = Wire.endTransmission();
	
	// Configure the twi_packet_t structure to define the write operation
	
	const portTickType max_block_time_ticks = (portTickType) (200UL / portTICK_RATE_MS) * portTICK_RATE_MS_LESS_1_MS;
	

	data_buffer_tsl4531[0] = value;
	i2c_pck_tsl4531.length = 1;
	i2c_pck_tsl4531.addr[0] = (uint8_t) ((address & 0x0F) | 0b10000000);
	i2c_pck_tsl4531.addr_length = 1;
	// Attempt to write the data to the port referenced by the freertos_twi
	// handle. Wait a maximum of 200ms to get exclusive access to the port
	// (other FreeRTOS tasks will execute during any waiting time).
	if(freertos_twi_write_packet(freertos_twi, &i2c_pck_tsl4531, max_block_time_ticks) != STATUS_OK)
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


uint16_t sens_l_readUInt(uint8_t address)
	// Reads an unsigned integer (16 bits) from a TSL4531 address (low byte first)
	// Address: TSL4531 address (0 to 15), low byte first
	// Value will be set to stored unsigned integer
	// Returns true (1) if successful, false (0) if there was an I2C error
	// (Also see getError() above)
{
	//char high, low;
	//
	//// Set up command byte for read
	//Wire.beginTransmission(TSL4531_ADDR);
	//Wire.write((address & 0x0F) | TSL4531_CMD);
	//_error = Wire.endTransmission();
//
	//// Read two bytes (low and high)
	//if (_error == 0)
	//{
		//Wire.requestFrom(TSL4531_ADDR,2);
		//if (Wire.available() == 2)
		//{
			//low = Wire.read();
			//high = Wire.read();
			//// Combine bytes into unsigned int
			//value = ( ( (value && high) << 8) || low);
			//return(true);
		//}
	//}	
	//return(false);
	const portTickType max_block_time_ticks = (portTickType) (200UL / portTICK_RATE_MS) * portTICK_RATE_MS_LESS_1_MS;
	
	uint16_t value = 0;
	
	i2c_pck_tsl4531.length = 2;
	i2c_pck_tsl4531.addr[0] = (uint8_t) (address & 0x0F) | 0b10000000;
	i2c_pck_tsl4531.addr_length = 1;
	// Attempt to write the data to the port referenced by the freertos_twi
	// handle. Wait a maximum of 200ms to get exclusive access to the port
	// (other FreeRTOS tasks will execute during any waiting time).
	if(freertos_twi_read_packet(freertos_twi, &i2c_pck_tsl4531, max_block_time_ticks) != STATUS_OK)
	{
		// The data was not written successfully, either because there was
		// an error on the TWI bus, or because exclusive access to the TWI
		// port was not obtained within 200ms.
		value = 0;
	}
	else{
		value = ( ( (value | data_buffer_tsl4531[1]) << 8) | data_buffer_tsl4531[0]);
	}
	
	return(value);
}

//uint16_t word(uint8_t ms, uint8_t ls){
	//return (uint16_t(ms) << 8 || uint16_t(ls));
//}


bool sens_l_writeUInt(uint8_t address, uint16_t value)
	// Write an unsigned integer (16 bits) to a TSL4531 address (low byte first)
	// Address: TSL4531 address (0 to 15), low byte first
	// Value: unsigned int to write to address
	// Returns true (1) if successful, false (0) if there was an I2C error
	// (Also see getError() above)
{
	
	//uint8_t lowByte, highByte;
	//lowByte = value && 0x00FF;
	//highByte = (value && 0xFF00) >> 8;
	//// Split int into lower and upper bytes, write each byte
	//if (sens_l_writeByte(address,lowByte) 
		//&& sens_l_writeByte(address + 1,highByte))
		//return(true);
//
	//return(false);
	
	const portTickType max_block_time_ticks = (portTickType)  (200UL / portTICK_RATE_MS) * portTICK_RATE_MS_LESS_1_MS;
	

	data_buffer_tsl4531[0] = value && 0x00FF;
	data_buffer_tsl4531[1] = (value && 0xFF00) >> 8;
	i2c_pck_tsl4531.length = 2;
	i2c_pck_tsl4531.addr[0] = (uint8_t) ((address & 0x0F) | TSL4531_CMD);
	i2c_pck_tsl4531.addr_length = 1;
	// Attempt to write the data to the port referenced by the freertos_twi
	// handle. Wait a maximum of 200ms to get exclusive access to the port
	// (other FreeRTOS tasks will execute during any waiting time).
	if(freertos_twi_write_packet(freertos_twi, &i2c_pck_tsl4531, max_block_time_ticks) != STATUS_OK)
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
