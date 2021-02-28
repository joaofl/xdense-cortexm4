/*
 * switch.h
 *
 * Created: 16/11/2014 17:30:41
 *  Author: Joao
 */ 


#ifndef SWITCH_H_
#define SWITCH_H_

#include <asf.h>
#include "comm_stack/packet.h"
#include "comm_stack/net_device.h"
#include "application.h"




//#define PORT_CONSOLE 4

//const static uint8_t PORT_ALL[PORT_COUNT] = {PORT_NORTH, PORT_SOUTH, PORT_EAST, PORT_WEST};
	


uint8_t switch_write_packet(uint8_t ports, xd_packet * pck);
bool	switch_protocol_packet_analysis (uint8_t ports, xd_packet * pck);
uint8_t switch_read_packet(uint8_t ports);
uint8_t switch_initialize(void);
void	switch_add_output_packet_to_queue(uint8_t port, xd_packet * pck_out);
void	switch_send_packet(uint8_t port);

xd_net_device net_devices[PORT_COUNT];
xQueueHandle xQueue_packet_output[PORT_COUNT]; 


#endif /* SWITCH_H_ */