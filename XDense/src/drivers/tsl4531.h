/*
	SFE_TSL2561 illumination sensor library for Arduino
	Mike Grusin, SparkFun Electronics
	
	This library provides functions to access the TAOS TSL2561
	Illumination Sensor.
	
	Our example code uses the "beerware" license. You can do anything
	you like with this code. No really, anything. If you find it useful,
	buy me a beer someday.

	version 1.0 2013/09/20 MDG initial version
	
	Ported to Atmel ATSAM4N GCC ARM and TSL45315CL by Jo�o Loureiro
	version 1.0 2014/12/17 initial version
*/

#ifndef SFE_TSL2561_h
#define SFE_TSL2561_h

#include "asf.h"
#include "drivers/twi.h"
#include "math.h"

//#define TSL2561_ADDR_0 0x29 // address with '0' shorted on board
//#define TSL2561_ADDR   0x39 // default address
//#define TSL2561_ADDR_1 0x49 // address with '1' shorted on board


// TSL2561 registers

//#define TSL2561_CMD           0x80
//#define TSL2561_CMD_CLEAR     0xC0
//#define	TSL2561_REG_CONTROL   0x00
//#define	TSL2561_REG_TIMING    0x01
//#define	TSL2561_REG_THRESH_L  0x02
//#define	TSL2561_REG_THRESH_H  0x04
//#define	TSL2561_REG_INTCTL    0x06
//#define	TSL2561_REG_ID        0x0A
//#define	TSL2561_REG_DATA_0    0x0C
//#define	TSL2561_REG_DATA_1    0x0E

#define TSL4531_ADDR  0x29
#define TSL4531_CMD           0x80
#define	TSL4531_REG_CONTROL   0x00
#define	TSL4531_REG_CONFIG    0x01
#define	TSL4531_REG_DATA_0    0x04
#define	TSL4531_REG_DATA_1    0x05
#define	TSL4531_REG_ID        0x0A

twi_packet_t i2c_pck_tsl4531;
uint8_t data_buffer_tsl4531[2];
uint8_t _error;


//public:

		
bool sens_l_setPowerUp(void);
// Turn on TSL2561, begin integration
// Returns true (1) if successful, false (0) if there was an I2C error
// (Also see getError() below)

bool sens_l_setPowerDown(void);
// Turn off TSL2561
// Returns true (1) if successful, false (0) if there was an I2C error
// (Also see getError() below)

bool sens_l_setTiming(bool gain, unsigned char time);
// If gain = false (0), device is set to low gain (1X)
// If gain = high (1), device is set to high gain (16X)
// If time = 0, integration will be 13.7ms
// If time = 1, integration will be 101ms
// If time = 2, integration will be 402ms
// If time = 3, use manual start / stop
// Returns true (1) if successful, false (0) if there was an I2C error
// (Also see getError() below)

//bool sens_l_setTiming(bool gain, uint8_t time, uint16_t* ms);
// Identical to above command, except ms is set to selected integration time
// If gain = false (0), device is set to low gain (1X)
// If gain = high (1), device is set to high gain (16X)
// If time = 0, integration will be 13.7ms
// If time = 1, integration will be 101ms
// If time = 2, integration will be 402ms
// If time = 3, use manual start / stop (ms = 0)
// ms will be set to requested integration time
// Returns true (1) if successful, false (0) if there was an I2C error
// (Also see getError() below)

bool sens_l_manualStart(void);
// Starts a manual integration period
// After running this command, you must manually stop integration with manualStop()
// Internally sets integration time to 3 for manual integration (gain is unchanged)
// Returns true (1) if successful, false (0) if there was an I2C error
// (Also see getError() below)

bool sens_l_manualStop(void);
// Stops a manual integration period
// Returns true (1) if successful, false (0) if there was an I2C error
// (Also see getError() below)

double sens_l_getData(void);
// Retrieve raw integration results
// data0 and data1 will be set to integration results
// Returns true (1) if successful, false (0) if there was an I2C error
// (Also see getError() below)
			
bool sens_l_getLux(uint8_t gain, uint16_t ms, uint16_t CH0, uint16_t CH1, double* lux);
// Convert raw data to lux
// gain: 0 (1X) or 1 (16X), see setTiming()
// ms: integration time in ms, from setTiming() or from manual integration
// CH0, CH1: results from getData()
// lux will be set to resulting lux calculation
// returns true (1) if calculation was successful
// RETURNS false (0) AND lux = 0.0 IF EITHER SENSOR WAS SATURATED (0XFFFF)

bool sens_l_setInterruptControl(uint8_t control, uint8_t persist);
// Sets up interrupt operations
// If control = 0, interrupt output disabled
// If control = 1, use level interrupt, see setInterruptThreshold()
// If persist = 0, every integration cycle generates an interrupt
// If persist = 1, any value outside of threshold generates an interrupt
// If persist = 2 to 15, value must be outside of threshold for 2 to 15 integration cycles
// Returns true (1) if successful, false (0) if there was an I2C error
// (Also see getError() below)

bool sens_l_setInterruptThreshold(uint16_t low, uint16_t high);
// Set interrupt thresholds (channel 0 only)
// low, high: 16-bit threshold values
// Returns true (1) if successful, false (0) if there was an I2C error
// (Also see getError() below)

bool sens_l_clearInterrupt(void);
// Clears an active interrupt
// Returns true (1) if successful, false (0) if there was an I2C error
// (Also see getError() below)

uint8_t sens_l_getID(void);
// Retrieves part and revision code from TSL2561
// Sets ID to part ID (see datasheet)
// Returns true (1) if successful, false (0) if there was an I2C error
// (Also see getError() below)
			
uint8_t sens_l_getError(void);
// If any library command fails, you can retrieve an extended
// error code using this command. Errors are from the wire library: 
// 0 = Success
// 1 = Data too long to fit in transmit buffer
// 2 = Received NACK on transmit of address
// 3 = Received NACK on transmit of data
// 4 = Other error
//


////	private:
//



uint8_t sens_l_readByte(uint8_t address);
// Reads a byte from a TSL2561 address
// Address: TSL2561 address (0 to 15)
// Value will be set to stored byte
// Returns true (1) if successful, false (0) if there was an I2C error
// (Also see getError() above)

bool sens_l_writeByte(uint8_t address, uint8_t value);
// Write a byte to a TSL2561 address
// Address: TSL2561 address (0 to 15)
// Value: byte to write to address
// Returns true (1) if successful, false (0) if there was an I2C error
// (Also see getError() above)

uint16_t sens_l_readUInt(uint8_t address);
// Reads an unsigned integer (16 bits) from a TSL2561 address (low byte first)
// Address: TSL2561 address (0 to 15), low byte first
// Value will be set to stored unsigned integer
// Returns true (1) if successful, false (0) if there was an I2C error
// (Also see getError() above)

bool sens_l_writeUInt(uint8_t address, uint16_t value);
// Write an unsigned integer (16 bits) to a TSL2561 address (low byte first)
// Address: TSL2561 address (0 to 15), low byte first
// Value: unsigned int to write to address
// Returns true (1) if successful, false (0) if there was an I2C error
// (Also see getError() above)



#endif
