/*
 * net_device.h
 *
 * Created: 20/11/2014 16:37:22
 *  Author: Joao
 */ 


#ifndef NET_DEVICE_H_
#define NET_DEVICE_H_

#include "asf.h"
#include <string.h>
#include "FreeRTOS.h"
//#include "comm_stack/switch.h"
#include "comm_stack/packet.h"
#include "drivers/led.h"
#include "main.h"

#define INPUT_BUFFER_SIZE 16

typedef struct{
	uint8_t input_buffer_size;// = INPUT_BUFFER_SIZE;
	uint8_t input_buffer[INPUT_BUFFER_SIZE];
	uint8_t id;
	uint8_t mask;
	freertos_uart_if * uart_freertos;
	Uart * uart;
}xd_net_device;


union buffer_element {
	uint8_t  byte;
	uint16_t halfword;
	uint32_t word;
};



union buffer_element fifo_read_buffer_uart3[INPUT_BUFFER_SIZE];
fifo_desc_t fifo_read_uart3;
union buffer_element fifo_write_buffer_uart3[INPUT_BUFFER_SIZE+32];
fifo_desc_t fifo_write_uart3;

//xQueueHandle input_buffer_uart3;
//xQueueHandle output_buffer_uart3;

xSemaphoreHandle mutex_uart3_data_write;
xSemaphoreHandle mutex_uart3_data_read;

xSemaphoreHandle semaphore_write_PORT_NORTH;
xSemaphoreHandle semaphore_write_PORT_SOUTH;
xSemaphoreHandle semaphore_write_PORT_EAST;
xSemaphoreHandle semaphore_write_PORT_WEST;

char pck_global_write_async[PORT_COUNT][PACKET_SIZE];

//bool uart3_received_successfuly;
//bool g_uart3_data_received;
//uint8_t uart3_bytes_received;
//xd_packet uart3_received_pck;
//uint8_t uart3_received_data[PACKET_SIZE];

//uint8_t net_device_input_buffer[4][INPUT_BUFFER_SIZE];

//uint8_t net_device_initialize(Uart *uart_base, uint8_t * receive_buffer, uint8_t receive_buffer_size);

uint8_t net_device_initialize(xd_net_device * nd);

uint8_t net_device_write_data(xd_net_device * nd, xd_packet * pck);

uint8_t net_device_read_data(xd_net_device * nd, xd_packet * pck);

uint32_t uart_serial_read_packet(Uart *, uint8_t *data, uint32_t len, portTickType block_time_ticks);

uint32_t uart_serial_write_packet(Uart * port, const uint8_t *data, uint32_t len, portTickType block_time_ticks);

void uart3_read_data(void);


	


#endif /* NET_DEVICE_H_ */