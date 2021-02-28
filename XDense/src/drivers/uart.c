/*
 * uart.c
 *
 * Created: 05/11/2014 22:34:02
 *  Author: Joao
 */ 

#include "uart.h"

freertos_uart_if prepare_uart_port(Uart *uart_base, uint8_t *receive_buffer, uint8_t receive_buffer_size)
{
	// Declare the variables used as parameters to the
	// freertos_uart_serial_init() function.
	// Declare a buffer to be used as the UART receive DMA buffer. The FreeRTOS
	// peripheral control drivers manage this buffer, and use it as a circular
	// buffer.
	//uint8_t receive_buffer[receive_buffer_size_in_bytes];
	// Handle used to access the initialized port by other FreeRTOS ASF functions.
	freertos_uart_if freertos_uart;
	// Configuration structure.
	freertos_peripheral_options_t driver_options = {
		// This peripheral has full duplex asynchronous operation, so the
		// receive_buffer value is set to a valid buffer location (declared
		// above).
		receive_buffer,
		// receive_buffer_size is set to the size, in bytes, of the buffer
		// pointed to by the receive_buffer value.
		receive_buffer_size,
		// The interrupt priority. The FreeRTOS driver provides the interrupt
		// service routine, and handles all interrupt interactions. The
		// application writer does not need to provide any interrupt handling
		// code, but does need to specify the priority of the DMA interrupt here.
		// IMPORTANT!!! see <a href="http://www.freertos.org/RTOS-Cortex-M3-M4.html">how to set interrupt priorities to work with FreeRTOS</a>
		0x0e,
		// The operation_mode value.
		UART_RS232,
		// Flags set to allow access from multiple tasks, and to wait in the
		// transmit function until the transmit is complete. Note that other
		// FreeRTOS tasks will execute during the wait period.
		(USE_TX_ACCESS_MUTEX | USE_RX_ACCESS_MUTEX) //assynchronous mode
		//(USE_TX_ACCESS_MUTEX | USE_RX_ACCESS_MUTEX | WAIT_TX_COMPLETE )
	};
	// The RS232 configuration. This structure, and the values used in its
	// setting, are from the standard ASF UART driver.
	sam_uart_opt_t uart_settings;
	
	uart_settings.ul_baudrate = 115200;
	uart_settings.ul_mck = sysclk_get_peripheral_hz();
	uart_settings.ul_mode = UART_MR_PAR_NO;
	
	
	if (uart_base != UART3)
	{
		freertos_uart = freertos_uart_serial_init(uart_base, &uart_settings, &driver_options);
		//configASSERT(freertos_uart);
		return freertos_uart;
	}
	else if (uart_base == UART3) // in this case I can not use the freeRtos, since there is no driver for ports without PDC (DMA)
	{
		usart_serial_options_t uart_options;
			uart_options.baudrate = 115200;
			uart_options.charlength = US_MR_CHRL_8_BIT;
			uart_options.paritytype = US_MR_PAR_NO;
			uart_options.stopbits = US_MR_NBSTOP_1_BIT;

			usart_serial_init(UART3, &uart_options);
			
			return UART3;
	}
}

//freertos_usart_if prepare_usart_port(Usart *usart_base, uint8_t *receive_buffer, uint8_t receive_buffer_size)
//{
//
	//freertos_peripheral_options_t driver_options = {
//
		//receive_buffer,
		//receive_buffer_size,
		//0x0e,
		//USART_RS232,
		////(USE_TX_ACCESS_MUTEX | USE_RX_ACCESS_MUTEX | WAIT_TX_COMPLETE )
		//(USE_TX_ACCESS_MUTEX | USE_RX_ACCESS_MUTEX)
	//};
	//// The RS232 configuration. This structure, and the values used in its
	//// setting, are from the standard ASF UART driver.
	//sam_usart_opt_t usart_settings;		
	//usart_settings.baudrate = 115200;
	//usart_settings.stop_bits = US_MR_NBSTOP_1_BIT;
	//usart_settings.irda_filter = 0;
	//usart_settings.channel_mode = US_MR_CHMODE_NORMAL;
	//usart_settings.parity_type = US_MR_PAR_NO;
	//usart_settings.char_length = US_MR_CHRL_8_BIT;
//
	//if (usart_base != UART3)
	//{
//
		//// Call the UART specific FreeRTOS ASF driver initialization function,
		//// storing the return value as the driver handle.
		//freertos_usart_if freertos_usart = freertos_usart_serial_init(usart_base, &usart_settings,
		//&driver_options);
	//
		//return freertos_usart;		
	//}
	//else if (usart_base == UART3) // in this case I can not use the freeRtos, since there is no driver for ports without PDC (DMA)
	//{
		//usart_serial_init(usart_base, &usart_settings);
		//return usart_base;
	//}
//
//
//}



status_code_t write_to_uart_port_assync(freertos_uart_if freertos_uart, xSemaphoreHandle notification_semaphore){
	
	// This examples assumes freertos_uart has already been set by a successful
	// call to freertos_uart_serial_init(), and that freertos_uart_serial_init()
	// configured the FreeRTOS ASF driver for standard mode operation.
	// xSemaphoreHandle is a FreeRTOS type used to store a handle to a semaphore.
	// In this example, the semaphore has already been created using a call to
	// the FreeRTOS vSemaphoreCreateBinary() API function, and is being passed in
	// as a function parameter.
	uint8_t write_buffer[5];
	status_code_t result;
	// Send a string to the UART. The string must be in RAM, so copy it
	// into an array. The array must exist for the entire time taken to
	// transmit the string. This can be ensured by making it global, static,
	// or by allocating it on the stack and then ensuring the stack frame
	// does not change until the transmission is complete.
	strcpy(write_buffer, "one");
	// notification_semaphore, passed into the function, is used by the
	// FreeRTOS ASF driver to signal that the transmission has finished.
	// Using a block time of 100 / portTICK_RATE_MS means "don't block any
	// longer than 100ms" to get access to the UART.
	result = freertos_uart_write_packet_async(freertos_uart, write_buffer, strlen("one"), 100 / portTICK_RATE_MS, notification_semaphore);
	
	if(result == STATUS_OK) {
		// Transmission of the string was started successfully.
	}
	// .. other processing can be performed here, while the string is being
	// transmitted ..
	// Another string is going to be sent, but the write_buffer array must
	// not be altered until the original transmission is complete. If the
	// notification semaphore is available, then the transmission is
	// complete and the following function call will return immediately. If
	// the notification semaphore is not available, then the following
	// function call will place this task into the Blocked state for a
	// maximum of 200ms to wait for it to become available (other tasks will
	// execute during the wait).
	xSemaphoreTake(notification_semaphore, 200 / portTICK_RATE_MS);
	strcpy(write_buffer, "two");
	result = freertos_uart_write_packet_async(freertos_uart, write_buffer,
	strlen("two"), 100 / portTICK_RATE_MS, notification_semaphore);
	// .. other processing can be performed here, while the string is being
	// transmitted ..
	// In this example, the array being transmitted is declared on the
	// stack. If this function exits, the array will no longer exist, and
	// if it was still being transmitted, the transmitted data can be
	// corrupted. Therefore, xSemaphoreTake() is used again to ensure the
	// transmission has completely finished before allowing the function to
	// return.
	xSemaphoreTake(notification_semaphore, 200 / portTICK_RATE_MS);
	return result;
}

// This examples assumes freertos_uart has already been set by a successful
// call to freertos_uart_serial_init(), and that freertos_uart_serial_init()
// configured the FreeRTOS UART driver to use the standard operation mode.
status_code_t write_to_uart_port(freertos_uart_if freertos_uart){
	uint8_t write_buffer[13];
	status_code_t result;
	// Send a string to the UART. The string must be in RAM, so copy it into
	// an array.
	strcpy(write_buffer, "5555555555");
	// Using a block time of 10 / portTICK_RATE_MS means "don't block any
	// longer than 10ms" to wait for access to the UART. Other FreeRTOS tasks
	// will execute during the wait period.
	
	if (freertos_uart != UART3)
	{
		result = freertos_uart_write_packet(freertos_uart, write_buffer, strlen("5555555555"), 10 / portTICK_RATE_MS);
			
		//if(result == STATUS_OK) {
		//// freertos_uart_write_packet() does not return until transmission
		//// of the string has completed (other FreeRTOS tasks will execute
		//// while the transmission is in progress), meaning the write_buffer
		//// array can be re-used immediately without any risk of corrupting
		//// the original transmission. Copy the second string to be
		//// transmitted into the RAM buffer.
		//strcpy(write_buffer, "two");
		//result = freertos_uart_write_packet(freertos_uart, write_buffer,
		//strlen("two"), 10 / portTICK_RATE_MS);
		//}
			
		// freertos_uart_write_packet() does not return until transmission of
		// the string has completed (other FreeRTOS task will execute while the
		// transmission is in progress), meaning the function can exit even
		// though the buffer being transmitted is declared on the function’s
		// stack because it is guaranteed that nothing is still using the data
		// the buffer contains.
		return result;
	}
	else if (freertos_uart == UART3)
	{
		usart_serial_write_packet(UART3, write_buffer, strlen("555"));
		
	}
}


void read_from_usart_port(freertos_uart_if freertos_uart)
{
	uint8_t receive_buffer[20];
	uint32_t bytes_received;
	portTickType max_wait_20ms = 20 / portTICK_RATE_MS;
	// Attempt to read 20 bytes from freertos_uart. If fewer than 20 bytes are
	// available, then wait a maximum of 20ms for the rest to arrive.
	bytes_received = freertos_uart_serial_read_packet(freertos_uart,
	receive_buffer, 20, max_wait_20ms);
	if(bytes_received == 20)
	{
		// All the bytes were received. The RTOS task calling this function
		// *may* have been placed into the Blocked state to wait for all the
		// bytes to be available. Other tasks will execute while this task
		// is in the Blocked state.
	}
	else
	{
		// Fewer than the requested number of bytes have been received, so
		// the RTOS task calling this function did enter the blocked state
		// for the full 20 milliseconds before giving up.
	}
}
