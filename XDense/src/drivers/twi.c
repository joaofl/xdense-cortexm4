/*
 * i2c.c
 *
 * Created: 05/11/2014 23:20:25
 *  Author: Joao
 */ 

#include "drivers/twi.h"

/**
* \brief Configure Twi comm.
*/
uint8_t twi_initialize(Twi *twi_base)
{
	
	// Configuration structure.
	freertos_peripheral_options_t driver_options = {
		// This peripheral is synchronous and so does not need a receive buffer.
		// The receive_buffer value is just set to NULL.
		NULL,
		// There isn't a receive buffer, so the receive_buffer_size value can
		// take any value.
		0,
		// The interrupt priority. The FreeRTOS driver provides the interrupt
		// service routine, and handles all interrupt interactions. The
		// application writer does not need to provide any interrupt handling
		// code, but does need to specify the priority of the DMA interrupt here.
		// IMPORTANT!!! see <a href="http://www.freertos.org/RTOS-Cortex-M3-M4.html">how to set interrupt priorities to work with FreeRTOS</a>
		0x0e,
		// The operation_mode value.
		TWI_I2C_MASTER,
		// Flags set to allow access from multiple tasks, wait in the transmit
		// function until the transmit is complete, and wait in the receive
		// function until reception is complete. Note that other FreeRTOS tasks
		// will execute during the wait period.
		(USE_TX_ACCESS_MUTEX | USE_RX_ACCESS_MUTEX | WAIT_TX_COMPLETE | WAIT_RX_COMPLETE)
	};
	// Call the TWI specific FreeRTOS ASF driver initialization function,
	// storing the return value as the driver handle.
	freertos_twi = freertos_twi_master_init(twi_base, &driver_options);
	if (freertos_twi != NULL) {
		// Calling freertos_twi_master_init() will enable the peripheral
		// clock, and set the TWI into I2C master mode. Other ASF
		// configuration functions, such as twi_set_speed(), can then be
		// called here.
		
		/* Configure the TWI bus parameters.  Do this after calling
		freertos_twi_master_init(). */
		twi_set_speed(twi_base, TWI_CLOCK_HZ, sysclk_get_cpu_hz());
		
		return 1;
	}
	
	return 0;

}


// Write number_of_bytes from data_buffer to freertos_twi.
//
// This examples assumes freertos_twi has already been set by a successful
// call to freertos_twi_master_init(), and that freertos_twi_master_init()
// configured the FreeRTOS TWI driver to use the standard operation mode.

uint8_t configure_sensor(freertos_twi_if freertos_twi, uint8_t address)
{
	twi_packet_t write_parameters;
	uint16_t calculated_address;
	uint8_t data_buffer[10];
	const portTickType max_block_time_ticks = (portTickType) ( 200UL / portTICK_RATE_MS) * portTICK_RATE_MS_LESS_1_MS;
	// Calculate the address being written to
	calculated_address = 1;
	// Configure the twi_packet_t structure to define the write operation
	write_parameters.chip = address;
	write_parameters.buffer = data_buffer;
	write_parameters.length = 1;
	write_parameters.addr[0] = (uint8_t) ((calculated_address >> 8) & 0xff);
	write_parameters.addr[1] = (uint8_t) (calculated_address & 0xff);
	write_parameters.addr_length = 2;
	// Attempt to write the data to the port referenced by the freertos_twi
	// handle. Wait a maximum of 200ms to get exclusive access to the port
	// (other FreeRTOS tasks will execute during any waiting time).
	if(freertos_twi_write_packet(freertos_twi, &write_parameters, max_block_time_ticks) != STATUS_OK)
	{
		// The data was not written successfully, either because there was
		// an error on the TWI bus, or because exclusive access to the TWI
		// port was not obtained within 200ms.
	}
}

