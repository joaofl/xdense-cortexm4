/*
 * net_device.c
 *
 * Created: 20/11/2014 16:37:34
 *  Author: Joao
 */ 

//#include "config/conf_board.h"
#include "comm_stack/net_device.h"



uint8_t net_device_initialize(xd_net_device * nd)
{
	//nd->uart;
	
	
	sam_uart_opt_t uart_settings;
			
	uart_settings.ul_baudrate = 3000000; //3000000; //TODO: 500000
	uart_settings.ul_mck = sysclk_get_peripheral_hz(); //sysclk_get_cpu_hz()
	uart_settings.ul_mode = US_MR_CHRL_8_BIT | UART_MR_PAR_NO | US_MR_NBSTOP_1_BIT | US_MR_CHMODE_NORMAL;
	  
	
	if (nd->uart != UART3)
	{
		// Configuration structure.
		freertos_peripheral_options_t driver_options = 
		{
			nd->input_buffer,
			nd->input_buffer_size,
			configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY, // 0x0a max interrupt priority from non-OS interrupt
			UART_RS232,
			(USE_TX_ACCESS_MUTEX | USE_RX_ACCESS_MUTEX | WAIT_TX_COMPLETE | WAIT_RX_COMPLETE)
			//(USE_TX_ACCESS_MUTEX | USE_RX_ACCESS_MUTEX | WAIT_TX_COMPLETE) //asynchronous mode
		};
		
		nd->uart_freertos = freertos_uart_serial_init(nd->uart, &uart_settings, &driver_options);			
		
	}
	else if (nd->uart == UART3) // in this case I can not use the freeRtos, since there is no driver for ports without PDC (DMA)
	{
		//uart3_data_access = xSemaphoreCreateMutex();
	
		//fifo_init(&fifo_desc, fifo_buffer, INPUT_BUFFER_SIZE);		
		//
		
		//input_buffer_uart3 = xQueueCreate(nd->input_buffer_size +16,sizeof(uint8_t));
		//output_buffer_uart3 = xQueueCreate(nd->input_buffer_size +16,sizeof(uint8_t));
		
		//input_buffer_uart3.pointer = 0;
		//input_buffer_uart3.size = INPUT_BUFFER_SIZE;
		//input_buffer_uart3.full = false;
		
		
		#ifndef PORT_NORTH_TXIO_DEBUG_MODE
			uart_init(UART3, &uart_settings);
			uart_enable_interrupt(UART3, UART_IER_RXRDY);   //Interrupt reading ready | UART_IER_ENDRX | UART_IER_RXBUFF | | UART_IER_TXRDY
			NVIC_DisableIRQ(UART3_IRQn);          // toca el NVIC->ICER[IRQn];   ICER = Interrupt Clear Enable Register
			NVIC_ClearPendingIRQ(UART3_IRQn);      // limpio el flag de Interrupt Clear-Pending register
			NVIC_SetPriority(UART3_IRQn,10);      // Toca el NVIC->IP[IRQn]
			NVIC_EnableIRQ(UART3_IRQn);         // toca el NVIC->ISER[IRQn]   ISER = Interrupt Set Enable Register
		#endif
		
		fifo_init(&fifo_read_uart3, fifo_read_buffer_uart3, INPUT_BUFFER_SIZE);
		fifo_init(&fifo_write_uart3, fifo_write_buffer_uart3, INPUT_BUFFER_SIZE + 32);
		
		//uart3_received_successfuly = false;
		//g_uart3_data_received = false;
		mutex_uart3_data_write = xSemaphoreCreateMutex();
		mutex_uart3_data_read = xSemaphoreCreateMutex();
		
		//vSemaphoreCreateBinary(semaphore_write_PORT_NORTH);
		//vSemaphoreCreateBinary(semaphore_write_PORT_SOUTH);
		//vSemaphoreCreateBinary(semaphore_write_PORT_EAST);
		//vSemaphoreCreateBinary(semaphore_write_PORT_WEST);
		//
		//semaphore_write_PORT_NORTH = xSemaphoreCreateMutex();
		//semaphore_write_PORT_SOUTH = xSemaphoreCreateMutex();
		//semaphore_write_PORT_EAST = xSemaphoreCreateMutex();
		//semaphore_write_PORT_WEST = xSemaphoreCreateMutex();
		//vSemaphoreCreateBinary(mutex_uart3_data_read);
		
	}
	return 0;
}


//TODO: GAMBIARRA HERE! FIX THIS LATER.... PEDRO

uint8_t net_device_write_data (xd_net_device * nd, xd_packet * pck)
{
	status_code_t result;
	//portTickType max_wait = PACKET_SIZE / portTICK_RATE_MS;
	//portTickType max_wait = (portTickType) (1 /portTICK_RATE_MS) * portTICK_RATE_MS_LESS_1_MS; 
	portTickType max_wait = (portTickType) 0xFFFFFFFF; 
	
	char pck_out[PACKET_SIZE];
	
	packet_serialize(pck, pck_out);
	
	 // Send a string to the UART.  The string must be in RAM, so copy it
	 // into an array.  The array must exist for the entire time taken to
	 // transmit the string.  This can be ensured by making it global, static,
	 // or by allocating it on the stack and then ensuring the stack frame
	 // does not change until the transmission is complete. IMPORTANT!!!!
	 
	if (nd->uart == UART_NORTH){
		result = uart_serial_write_packet(nd->uart, pck_out, PACKET_SIZE, max_wait);// max_wait);
	}	
	else if (nd->uart == UART_SOUTH){
		result = freertos_uart_write_packet(nd->uart_freertos, pck_out, PACKET_SIZE, max_wait);
	}
	else if (nd->uart == UART_EAST){
		result = freertos_uart_write_packet(nd->uart_freertos, pck_out, PACKET_SIZE, max_wait);
	}
	else if (nd->uart == UART_WEST){
		result = freertos_uart_write_packet(nd->uart_freertos, pck_out, PACKET_SIZE, max_wait);
	} else {
		result = STATUS_ERR_DENIED;
	}
		
	return result;
}


//TODO: GAMBIARRA HERE! FIX THIS LATER.... PEDRO


uint8_t net_device_read_data(xd_net_device * nd, xd_packet * pck)
{
	uint8_t bytes_read = 0;
	char data[PACKET_SIZE];
	portTickType max_wait = 0xFFFFFFFF ; // (portTickType) ( 10000 / portTICK_RATE_MS) * portTICK_RATE_MS_LESS_1_MS;

	if (nd->uart == UART_NORTH){
		//vTaskDelay(max_wait);
		bytes_read = uart_serial_read_packet(nd->uart, data, PACKET_SIZE, max_wait); // max_wait); //my implementation of it...
	}
	else {
		bytes_read = freertos_uart_serial_read_packet(nd->uart_freertos, data, 1, max_wait);
		if (bytes_read) {
			bytes_read += freertos_uart_serial_read_packet(nd->uart_freertos, &data[bytes_read], PACKET_SIZE - bytes_read, (portTickType)  (1 / portTICK_RATE_MS) * portTICK_RATE_MS_LESS_1_MS);
		}
	}
	
	if (bytes_read == PACKET_SIZE)
	{
		if(packet_deserialize(data, pck)){
			pck->port_origin = nd->id;
		} else {
			bytes_read = 0; //BCC error
		}
	}
	
	return bytes_read;
}




//uint8_t net_device_read_data(xd_net_device * nd, xd_packet * pck)
//{
	//uint8_t bytes_read = 0;
	//char data[PACKET_SIZE];
	//portTickType max_wait = 10 / portTICK_RATE_MS;
//
	//if (nd->uart == UART_NORTH){
		////vTaskDelay(max_wait);
		//bytes_read = uart_serial_read_packet(nd->uart, data, PACKET_SIZE, max_wait); //my implementation of it...
	//}
	//else if (nd->uart == UART_SOUTH){
		//bytes_read = freertos_uart_serial_read_packet(nd->uart_freertos, data, PACKET_SIZE, max_wait);
		//if (bytes_read < PACKET_SIZE) {
			//bytes_read += freertos_uart_serial_read_packet(nd->uart_freertos, &data[bytes_read], PACKET_SIZE - bytes_read, max_wait);
		//}
		//if (bytes_read < PACKET_SIZE) {
			//bytes_read += freertos_uart_serial_read_packet(nd->uart_freertos, &data[bytes_read], PACKET_SIZE - bytes_read, max_wait);
		//}
	//}
	//else if (nd->uart == UART_EAST){
		//bytes_read = freertos_uart_serial_read_packet(nd->uart_freertos, data, PACKET_SIZE, max_wait);
		//if (bytes_read < PACKET_SIZE) {
			//bytes_read += freertos_uart_serial_read_packet(nd->uart_freertos, &data[bytes_read], PACKET_SIZE - bytes_read, max_wait);
		//}
		//if (bytes_read < PACKET_SIZE) {
			//bytes_read += freertos_uart_serial_read_packet(nd->uart_freertos, &data[bytes_read], PACKET_SIZE - bytes_read, max_wait);
		//}
	//}
	//else if (nd->uart == UART_WEST){
		//bytes_read = freertos_uart_serial_read_packet(nd->uart_freertos, data, PACKET_SIZE, max_wait);
		//if (bytes_read < PACKET_SIZE) {
			//bytes_read += freertos_uart_serial_read_packet(nd->uart_freertos, &data[bytes_read], PACKET_SIZE - bytes_read, max_wait);
		//}
		//if (bytes_read < PACKET_SIZE) {
			//bytes_read += freertos_uart_serial_read_packet(nd->uart_freertos, &data[bytes_read], PACKET_SIZE - bytes_read, max_wait);
		//}
	//}
	//
	//if (bytes_read == PACKET_SIZE)
	//{
		//if(packet_deserialize(data, pck)){
			//pck->port_origin = nd->id;
		//} else {
			//bytes_read = 0; //BCC error
		//}
	//}
	//
	//return bytes_read;
//}


uint32_t uart_serial_read_packet(Uart * port, uint8_t * data, uint32_t len, portTickType block_time_ticks){
		
	uint8_t data_temp;
	uint8_t i = 0;
	
	if( xSemaphoreTake( mutex_uart3_data_read, block_time_ticks ) == pdTRUE )
    {
		while(fifo_pull_uint8(&fifo_read_uart3, &data_temp) == FIFO_OK && i<len){
			data[i] = data_temp;
			i++;
		}
    }
	
	
	return i;
}


uint32_t uart_serial_write_packet(Uart * port, const uint8_t *data, uint32_t len, portTickType block_time_ticks){
	
	xSemaphoreTake(mutex_uart3_data_write, block_time_ticks);
	uint8_t i;
	
	for (i = 0 ; i < len ; i++)
	{
		//xQueueSendToBack(output_buffer_uart3, &data[i], block_time_ticks);
		fifo_push_uint8(&fifo_write_uart3, data[i]);
	}
	
	uart_enable_interrupt(UART3, UART_IER_TXRDY);
	
	xSemaphoreGive(mutex_uart3_data_write);
}


void UART3_Handler(){
		
	if( uart_is_rx_ready(UART3))
	{
			uint8_t data = (uint8_t) UART3->UART_RHR;			
			fifo_push_uint8(&fifo_read_uart3, data);
			if (fifo_is_full(&fifo_read_uart3)) {
				xSemaphoreGiveFromISR(mutex_uart3_data_read, pdFALSE);
			}		
	}
	else if(uart_is_tx_ready(UART3))
	{
		uint8_t data_out;
		
		if(fifo_pull_uint8(&fifo_write_uart3, &data_out) == FIFO_OK){
			UART3->UART_THR = data_out;	
		} else {
			uart_disable_interrupt(UART3, UART_IDR_TXRDY);
		}
		
		/*
		
		static portBASE_TYPE xHigherPriorityTaskWoken;
		xHigherPriorityTaskWoken = pdFALSE;
			
		if (xQueueReceiveFromISR(output_buffer_uart3, &data_out, xHigherPriorityTaskWoken) == pdPASS)
		{
			UART3->UART_THR = data_out;
			//printf("d=%d\r\n, data_out);
		}
		else
		{
			//printf("IR Disabled\r\n);
			uart_disable_interrupt(UART3, UART_IDR_TXRDY);
		}
		
		//printf("IR Running\r\n);
		*/
	}
	
}
