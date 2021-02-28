/*
 * switch.c
 *
 * Created: 16/11/2014 17:30:29
 *  Author: Joao
 */ 


#include "comm_stack/switch.h"

uint8_t switch_initialize(void)
{
	uint8_t errors = 0;
	uint8_t i = 0;
	 
	net_devices[PORT_NORTH].uart = UART_NORTH;
	net_devices[PORT_NORTH].id = PORT_NORTH;
	net_devices[PORT_NORTH].mask = PORT_NORTH_MASK;
	net_devices[PORT_NORTH].input_buffer_size = INPUT_BUFFER_SIZE;
	errors += net_device_initialize(&net_devices[PORT_NORTH]);
	
	net_devices[PORT_SOUTH].uart = UART_SOUTH;
	net_devices[PORT_SOUTH].id = PORT_SOUTH;
	net_devices[PORT_SOUTH].mask = PORT_SOUTH_MASK;
	net_devices[PORT_SOUTH].input_buffer_size = INPUT_BUFFER_SIZE;
	errors += net_device_initialize(&net_devices[PORT_SOUTH]);
	
	net_devices[PORT_EAST].uart = UART_EAST;
	net_devices[PORT_EAST].id = PORT_EAST;
	net_devices[PORT_EAST].mask = PORT_EAST_MASK;
	net_devices[PORT_EAST].input_buffer_size = INPUT_BUFFER_SIZE;
	errors += net_device_initialize(&net_devices[PORT_EAST]);
	
	net_devices[PORT_WEST].uart = UART_WEST;
	net_devices[PORT_WEST].id = PORT_WEST;
	net_devices[PORT_WEST].mask = PORT_WEST_MASK;
	net_devices[PORT_WEST].input_buffer_size = INPUT_BUFFER_SIZE;
	errors += net_device_initialize(&net_devices[PORT_WEST]);
	
	for (i=0;i<PORT_COUNT; i++){
		xQueue_packet_output[i] = xQueueCreate( PACKET_QUEUE_N, sizeof( xd_packet ) );	
	}	
	
	return errors;
}

uint8_t switch_write_packet(uint8_t ports, xd_packet * pck)
{

	/*Transmit through this port last since it has not DMA (PDC), and the
	processor can be busy, while the other packets are being sent*/
	if ((PORT_NORTH_MASK & ports) >> 0 == 1) {
		//net_device_write_data(&net_devices[PORT_NORTH], pck);
		switch_add_output_packet_to_queue(PORT_NORTH, pck);
		//if (g_is_led_debugging) 
		leds[PORT_EAST].g_blink_duration = 1 * LED_BLINK_MIN_PERIOD;
	}	

	if ((PORT_SOUTH_MASK & ports) >> 1 == 1) {
		//net_device_write_data(&net_devices[PORT_SOUTH], pck);
		switch_add_output_packet_to_queue(PORT_SOUTH, pck);
		//if (g_is_led_debugging) 
		leds[PORT_EAST].g_blink_duration = 1 * LED_BLINK_MIN_PERIOD;
	}
	if ((PORT_EAST_MASK & ports) >> 2 == 1) {
		//net_device_write_data(&net_devices[PORT_EAST], pck);
		switch_add_output_packet_to_queue(PORT_EAST, pck);
		//if (g_is_led_debugging) 
		leds[PORT_EAST].g_blink_duration = 1 * LED_BLINK_MIN_PERIOD;
	}
	if ((PORT_WEST_MASK & ports) >> 3 == 1) {
		//net_device_write_data(&net_devices[PORT_WEST], pck);
		switch_add_output_packet_to_queue(PORT_WEST, pck);
		//if (g_is_led_debugging) 
		leds[PORT_EAST].g_blink_duration = 1 * LED_BLINK_MIN_PERIOD;
	}				
	return 0;
}

uint8_t switch_redirect_packet_unicast(xd_packet * pck){
	
	if( pck->app_packet_type == APP_PROTOCOL_REQUEST){
		if (pck->d_x > 0) //Travel x-
			switch_write_packet(PORT_EAST_MASK, pck);
		else if (pck->d_x < 0) ////Travel x+
			switch_write_packet(PORT_WEST_MASK, pck);
		else { // = 0, travel Y {
			if (pck->d_y > 0) //Travel y-
				switch_write_packet(PORT_SOUTH_MASK, pck);
			else if (pck->d_y < 0) //Travel y+	
				switch_write_packet(PORT_NORTH_MASK, pck);
		}
	} else if( pck->app_packet_type ==  APP_PROTOCOL_RESPONSE) {
		if (pck->d_y > 0) //Travel y-
			switch_write_packet(PORT_SOUTH_MASK, pck);
		else if (pck->d_y < 0) //Travel y+
			switch_write_packet(PORT_NORTH_MASK, pck);	
		else { // = 0, travel Y {
			if (pck->d_x > 0) //Travel x-
				switch_write_packet(PORT_EAST_MASK, pck);
			else if (pck->d_x < 0) ////Travel x+
				switch_write_packet(PORT_WEST_MASK, pck);
		}
	} 
			
		//if(pck->payload[0] & PROTOCOL_REQUEST){
			//if (pck->d_x >= 0 && pck->d_y < 0) {
				//switch_write_packet(PORT_NORTH_MASK, pck);
			//} else if (pck->d_x <= 0 && pck->d_y > 0) {
				//switch_write_packet(PORT_SOUTH_MASK, pck);
			//} else if (pck->d_x > 0 && pck->d_y >= 0 ) {
				//switch_write_packet(PORT_EAST_MASK, pck);
			//} else if (pck->d_x < 0 && pck->d_y <= 0) {
				//switch_write_packet(PORT_WEST_MASK, pck);
			//}
		//} else if(pck->payload[0] & PROTOCOL_RESPONSE){
			//if (pck->d_x >= 0 && pck->d_y < 0) {
				//switch_write_packet(PORT_WEST_MASK, pck);
			//} else if (pck->d_x <= 0 && pck->d_y > 0) {
				//switch_write_packet(PORT_EAST_MASK, pck);
			//} else if (pck->d_x > 0 && pck->d_y >= 0 ) {
				//switch_write_packet(PORT_SOUTH_MASK, pck);
			//} else if (pck->d_x < 0 && pck->d_y <= 0) {
				//switch_write_packet(PORT_NORTH_MASK, pck);
			//}
		//}

		return 0;
}

uint8_t switch_redirect_packet_multicast_1(xd_packet * pck){
	
	pck->protocol = PROTOCOL_UNICAST;
	
	pck->d_x=0;	pck->d_y=-1;
	switch_write_packet(PORT_NORTH_MASK, pck);
	pck->d_x=0;	pck->d_y=1;
	switch_write_packet(PORT_SOUTH_MASK, pck);
	pck->d_x=-1;	pck->d_y=0;
	switch_write_packet(PORT_WEST_MASK, pck);
	pck->d_x=1;	pck->d_y=0;
	switch_write_packet(PORT_EAST_MASK, pck);
	return 0;
}

void switch_redirect_packet_broadcast (xd_packet * pck){
	if(pck->s_x <= 0 && pck->s_y <= 0) {
		switch_write_packet(PORT_SOUTH_MASK, pck);
	} 
	if(pck->s_x >= 0 && pck->s_y <= 0) {
		switch_write_packet(PORT_WEST_MASK, pck);
	} 
	if(pck->s_x >= 0 && pck->s_y >= 0) {
		switch_write_packet(PORT_NORTH_MASK, pck);
	} 
	if(pck->s_x <= 0 && pck->s_y >= 0) {
		switch_write_packet(PORT_EAST_MASK, pck);
	}
	
	return;	
}

bool switch_protocol_packet_analysis (uint8_t port, xd_packet * pck){
	
	bool accept_packet=false;
	
	switch(pck->protocol){
		case PROTOCOL_UNICAST:
			if (pck->d_x == 0 && pck->d_y == 0){
				accept_packet = true;
				application_add_packet_to_queue(pck);
			} else {
				accept_packet = false;
				switch_redirect_packet_unicast(pck);
			}
		break;
		case PROTOCOL_MULTICAST_1:
			if(pck->d_x == 0 && pck->d_y == 0){
				accept_packet = false;
				switch_redirect_packet_multicast_1(pck);
			} else {
				accept_packet = false;
				switch_redirect_packet_unicast(pck); //redirect as unicast packet
				//accept_packet = true;
				//application_packet_received(pck);
			}
			
		break;
		case PROTOCOL_BROADCAST:
			accept_packet = true;
			switch_redirect_packet_broadcast(pck);
			application_add_packet_to_queue(pck);			
		break;		
	}
	
	return accept_packet;
}


uint8_t switch_read_packet(uint8_t port)
{
	int bytes_received;
	
	xd_packet pckt;
	
	bytes_received = net_device_read_data(&net_devices[port], &pckt);
	
	if (bytes_received == PACKET_SIZE)
	{
		leds[PORT_WEST].r_blink_duration = 1 * LED_BLINK_MIN_PERIOD;	
		
		if (port == PORT_NORTH && pckt.d_y > -126) {
			pckt.s_y -= 1;
			pckt.d_y -= 1;
		} else if (port == PORT_SOUTH &&  pckt.d_y < 126) {
			pckt.s_y += 1;
			pckt.d_y += 1;
		} else if (port == PORT_EAST && pckt.d_x  < 126) {
			pckt.s_x += 1; // -1 ?
			pckt.d_x += 1; // -1 ?
		} else if (port == PORT_WEST && pckt.d_x > -126) {
			pckt.s_x -= 1; // -1 ?
			pckt.d_x -= 1; // -1 ?
		} else
		{
			bytes_received = 0; //packet dropped since the counter have exceeded its limits
			return 0;
		}
		
		if ( switch_protocol_packet_analysis(port, &pckt) == false ){
			bytes_received = 0; //packet redirected, not for this node
		}

	}
	
	//free_pck_id_packet_queue(pckt);
	
	return bytes_received;
	
	
	/*
	if ((PORT_NORTH_MASK & ports) >> 0 == 1)
	{
		bytes_received[PORT_NORTH] = net_device_read_data(&net_devices[PORT_NORTH], &pck_input[PORT_NORTH]);
		if (bytes_received[PORT_NORTH] == PACKET_SIZE)
		{
			if (pck_input[PORT_NORTH].d_y > -126)
			{
				pck_input[PORT_NORTH].s_y -= 1;
				pck_input[PORT_NORTH].d_y -= 1;
				
				//if (g_is_led_debugging) 
				leds[PORT_WEST].r_blink_duration = 1 * LED_BLINK_MIN_PERIOD;			
				
				if ( switch_protocol_packet_analysis(PORT_NORTH, &pck_input[PORT_NORTH]) == false ){
					bytes_received[PORT_NORTH] = 0; //packet redirected, not for this node
				}
				
			}
			else
			{
				bytes_received[PORT_NORTH] = 0; //packet dropped since the counter have exceeded its limits
			}

		}

	}
	
	if ((PORT_SOUTH_MASK & ports) >> 1 == 1)
	{
		bytes_received[PORT_SOUTH] = net_device_read_data(&net_devices[PORT_SOUTH], &pck_input[PORT_SOUTH]);
		//result = freertos_uart_write_packet(UART_SOUTH, pck_str, PACKET_SIZE, 10 / portTICK_RATE_MS);
		if (bytes_received[PORT_SOUTH] == PACKET_SIZE)
		{
			if (pck_input[PORT_SOUTH].d_y < 126)
			{
				pck_input[PORT_SOUTH].s_y += 1;
				pck_input[PORT_SOUTH].d_y += 1;
			
				//if (g_is_led_debugging) 
				leds[PORT_WEST].r_blink_duration = 1 * LED_BLINK_MIN_PERIOD;
			
				if ( switch_protocol_packet_analysis(PORT_SOUTH, &pck_input[PORT_SOUTH]) == false){
					bytes_received[PORT_SOUTH] = 0; //packet redirected, not for this node
				}
			
			}
			else
			{
				bytes_received[PORT_SOUTH] = 0; //packet dropped since the counter have exceeded its limits
			}
		}

	}
	
	if ((PORT_EAST_MASK & ports) >> 2 == 1)
	{
		bytes_received[PORT_EAST] = net_device_read_data(&net_devices[PORT_EAST], &pck_input[PORT_EAST]);
		//result = freertos_uart_write_packet(UART_EAST, pck_str, PACKET_SIZE, 10 / portTICK_RATE_MS);
		if (bytes_received[PORT_EAST] == PACKET_SIZE)
		{
			if (pck_input[PORT_EAST].d_x  < 126) //> -126? //down to -127 max
			{
				pck_input[PORT_EAST].s_x += 1; // -1 ? 
				pck_input[PORT_EAST].d_x += 1; // -1 ?
			
				//if (g_is_led_debugging) 
				leds[PORT_WEST].r_blink_duration = 1 * LED_BLINK_MIN_PERIOD;
			
				if ( switch_protocol_packet_analysis(PORT_EAST, &pck_input[PORT_EAST]) == false){
					bytes_received[PORT_EAST] = 0; //packet redirected, not for this node
				}
			
			}
			else
			{
				bytes_received[PORT_EAST] = 0; //packet dropped since the counter have exceeded its limits
			}
		}

	}
	
	if ((PORT_WEST_MASK & ports) >> 3 == 1)
	{
		bytes_received[PORT_WEST] = net_device_read_data(&net_devices[PORT_WEST], &pck_input[PORT_WEST]);
		//usart_serial_write_packet(UART_WEST, pck_str, PACKET_SIZE);
		if (bytes_received[PORT_WEST] == PACKET_SIZE)
		{
			if (pck_input[PORT_WEST].d_x > -126)
			{
				pck_input[PORT_WEST].s_x -= 1; // -1 ?
				pck_input[PORT_WEST].d_x -= 1; // -1 ?
			
				//if (g_is_led_debugging)
				leds[PORT_WEST].r_blink_duration = 1 * LED_BLINK_MIN_PERIOD;
			
				if ( switch_protocol_packet_analysis(PORT_WEST, &pck_input[PORT_WEST]) == false ){
					bytes_received[PORT_WEST] = 0; //packet redirected, not for this node
				}
			}
			else
			{
				bytes_received[PORT_WEST] = 0; //packet dropped since the counter have exceeded its limits
			}
		}

	}
	*/
	
}

void switch_add_output_packet_to_queue(uint8_t port, xd_packet * pck_out){
	xQueueSendToBack( xQueue_packet_output[port], pck_out,  ( portTickType ) 100 / portTICK_RATE_MS * portTICK_RATE_MS_LESS_1_MS );
	return;
}

void switch_send_packet(uint8_t port){
	xd_packet pck_out = {0};
	
	if( xQueueReceive( xQueue_packet_output[port],  &pck_out, (portTickType) (10000 / portTICK_RATE_MS) * portTICK_RATE_MS_LESS_1_MS ) ) {
		net_device_write_data(&net_devices[port], &pck_out);
	}
	return;
}


