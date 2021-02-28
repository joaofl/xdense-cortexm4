/*
 * i2c.h
 *
 * Created: 05/11/2014 23:20:38
 *  Author: Joao
 */ 


#ifndef I2C_DRIVER_H_
#define I2C_DRIVER_H_

#include <asf.h>


#define TWI_CLOCK_HZ 100000

// Declare the variables used as parameters to the
// freertos_twi_master_init() function.
// Handle used to access the initialized port by other FreeRTOS ASF functions.
freertos_twi_if freertos_twi;

/**
* \brief Configure Twi comm.
*/
uint8_t twi_initialize(Twi *twi_base);

//uint16_t read_senssor(freertos_twi_if freertos_twi, uint8_t address);
//
//
//uint8_t configure_sensor(freertos_twi_if freertos_twi, uint8_t address);



#endif /* I2C_DRIVER_H_ */