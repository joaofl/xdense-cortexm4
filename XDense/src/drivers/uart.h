/*
 * uart.h
 *
 * Created: 05/11/2014 22:33:38
 *  Author: Joao
 */ 


#ifndef UART_H_
#define UART_H_

#include <asf.h>
#include <string.h>

freertos_uart_if prepare_uart_port(Uart *uart_base, uint8_t *receive_buffer, uint8_t receive_buffer_size);

freertos_usart_if prepare_usart_port(Usart *usart_base, uint8_t *receive_buffer, uint8_t receive_buffer_size);

status_code_t write_to_uart_port_assync(freertos_uart_if freertos_uart, xSemaphoreHandle notification_semaphore);

status_code_t write_to_uart_port(freertos_uart_if freertos_uart);

void read_from_usart_port(freertos_uart_if freertos_uart);

#endif /* UART_H_ */