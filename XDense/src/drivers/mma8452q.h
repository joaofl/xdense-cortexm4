/******************************************************************************
SparkFun_MMA8452Q.h
SparkFun_MMA8452Q Library Header File
Jim Lindblom @ SparkFun Electronics
Original Creation Date: June 3, 2014
https://github.com/sparkfun/MMA8452_Accelerometer

This file prototypes the MMA8452Q class, implemented in SFE_MMA8452Q.cpp. In
addition, it defines every register in the MMA8452Q.

Development environment specifics:
	IDE: Arduino 1.0.5
	Hardware Platform: Arduino Uno

	**Updated for Arduino 1.6.4 5/2015**
	
This code is beerware; if you see me (or any other SparkFun employee) at the
local, and you've found our code helpful, please buy us a round!

Distributed as-is; no warranty is given.
******************************************************************************/

#ifndef MMA8452Q_h
#define MMA8452Q_h

	#include "asf.h"
	#include "drivers/twi.h"
	#include "math.h"

	///////////////////////////////////
	// MMA8452Q Register Definitions //
	///////////////////////////////////
	typedef enum {
		STATUS = 0x00,
		OUT_X_MSB = 0x01,
		OUT_X_LSB = 0x02,
		OUT_Y_MSB = 0x03,
		OUT_Y_LSB = 0x04,
		OUT_Z_MSB = 0x05,
		OUT_Z_LSB = 0x06,
		SYSMOD = 0x0B,
		INT_SOURCE = 0x0C,
		WHO_AM_I = 0x0D,
		XYZ_DATA_CFG = 0x0E,
		HP_FILTER_CUTOFF = 0x0F,
		PL_STATUS = 0x10,
		PL_CFG = 0x11,
		PL_COUNT = 0x12,
		PL_BF_ZCOMP = 0x13,
		P_L_THS_REG = 0x14,
		FF_MT_CFG = 0x15,
		FF_MT_SRC = 0x16,
		FF_MT_THS = 0x17,
		FF_MT_COUNT = 0x18,
		TRANSIENT_CFG = 0x1D,
		TRANSIENT_SRC = 0x1E,
		TRANSIENT_THS = 0x1F,
		TRANSIENT_COUNT = 0x20,
		PULSE_CFG = 0x21,
		PULSE_SRC = 0x22,
		PULSE_THSX = 0x23,
		PULSE_THSY = 0x24,
		PULSE_THSZ = 0x25,
		PULSE_TMLT = 0x26,
		PULSE_LTCY = 0x27,
		PULSE_WIND = 0x28,
		ASLP_COUNT = 0x29,
		CTRL_REG1 = 0x2A,
		CTRL_REG2 = 0x2B,
		CTRL_REG3 = 0x2C,
		CTRL_REG4 = 0x2D,
		CTRL_REG5 = 0x2E,
		OFF_X = 0x2F,
		OFF_Y = 0x30,
		OFF_Z = 0x31
	} MMA8452Q_Register;

	////////////////////////////////
	// MMA8452Q Misc Declarations //
	////////////////////////////////
	typedef enum {SCALE_2G = 2, SCALE_4G = 4, SCALE_8G = 8} MMA8452Q_Scale; // Possible full-scale settings
	typedef enum {ODR_800, ODR_400, ODR_200, ODR_100, ODR_50, ODR_12, ODR_6, ODR_1} MMA8452Q_ODR; // possible data rates
	// Possible portrait/landscape settings
	#define PORTRAIT_U 0
	#define PORTRAIT_D 1
	#define LANDSCAPE_R 2
	#define LANDSCAPE_L 3
	#define LOCKOUT 0x40
	
	short MMA8452Q_x, MMA8452Q_y, MMA8452Q_z;
	float MMA8452Q_cx, MMA8452Q_cy, MMA8452Q_cz;

	uint8_t address;
	MMA8452Q_Scale scale;
	
	twi_packet_t i2c_pck_mma8452q;
	uint8_t data_buffer_mma8452q[10];

	uint8_t MMA8452Q_init(MMA8452Q_Scale fsr, MMA8452Q_ODR odr);
    void MMA8452Q_read();
	uint8_t MMA8452Q_available();
	uint8_t MMA8452Q_readTap();
	uint8_t MMA8452Q_readPL();
	
	void MMA8452Q_standby();
	void MMA8452Q_active();
	void MMA8452Q_setupPL();
	void MMA8452Q_setupTap(uint8_t xThs, uint8_t yThs, uint8_t zThs);
	void MMA8452Q_setScale(MMA8452Q_Scale fsr);
	void MMA8452Q_setODR(MMA8452Q_ODR odr);
	void MMA8452Q_writeRegister(MMA8452Q_Register reg, uint8_t data);
    void MMA8452Q_writeRegisters(MMA8452Q_Register reg, uint8_t *buffer, uint8_t len);
	uint8_t MMA8452Q_readRegister(MMA8452Q_Register reg);
    void MMA8452Q_readRegisters(MMA8452Q_Register reg, uint8_t *buffer, uint8_t len);

#endif