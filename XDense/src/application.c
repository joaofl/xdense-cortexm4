/*
 * application.c
 *
 * Created: 20/01/2015 17:31:21
 *  Author: Joao
 */ 


#include "application.h"
#include "math_lib.h"

#define SYST_CVR ( * ( ( volatile unsigned long * ) 0xe000e018 ) ) // SysTick Current Value Register

union int_float_bits {
	int32_t int_bits;
	float float_bits;
};

float intBitsToFloat(int32_t x_value_convert)
{
	union int_float_bits bits;
	bits.int_bits = x_value_convert;
	return bits.float_bits;
}

void application_initialize(void){
	uint8_t i;
	
	//initialize variables
	for (i = 0 ; i < PORT_COUNT ; i++)
	{
		active_neighbors[i] = false;
		//pck_serial_number[i] = 0;
	}
	
	for ( i = 0 ; i < MAX_SINKS ; i++ )
	{
		sink_list[i].x = 0;
		sink_list[i].y = 0;
		sink_list[i].serial_number = -1; //255... so that the serial number 0 is new in the first round
		sink_list[i].active = false;
	}
	
	xQueue_packet_input = xQueueCreate( PACKET_QUEUE_N, sizeof( xd_packet ) );
	
	xQueue_packet_OBSERVE = xQueueCreate( 1 , sizeof( xd_packet ) );
	
	application_packet_observe_cycle_time = 0;
}



void application_add_packet_to_queue(xd_packet * pck_in){
	xQueueSendToBack( xQueue_packet_input, pck_in, ( portTickType ) 0xFFFFFFFF ); //( 1 / portTICK_RATE_MS ) * portTICK_RATE_MS_LESS_1_MS );
	return;
}	

void application_process_packet_received(){
	static xd_packet pck_in = {0};
	
	if( xQueueReceive( xQueue_packet_input,  &pck_in,(portTickType) 0xFFFFFFFF)) { //( 10000 / portTICK_RATE_MS ) * portTICK_RATE_MS_LESS_1_MS ) 
		if (pck_in.app_packet_type){ // IS A RESPONSE
			switch (pck_in.app_resource){
				case APP_RESOURCE_PING:
					application_ping_response_received(&pck_in);
				break;
				case APP_RESOURCE_ND:
					//application_method_not_available(&pck_in);
				break;
				case APP_RESOURCE_CONFIG:
					application_config_response_received(&pck_in);
				break;
				case APP_RESOURCE_SENSOR_LIGHT:
					application_va_response_received(&pck_in, 'L');
				break;
				case APP_RESOURCE_SENSOR_PRESSURE:
					application_va_response_received(&pck_in, 'P');
				break;
				case APP_RESOURCE_SENSOR_ACCELERATION:
					application_va_response_received(&pck_in, 'A');
				break;
				case APP_RESOURCE_SENSOR_TEMPERATURE:
					application_va_response_received(&pck_in, 'T');
				break;
				case APP_RESOURCE_ACTUATOR_LED:
					application_ping_response_received(&pck_in);
				break;
				default:
					//application_method_not_available(&pck_in);
				break;
			}
		} else { //IS A REQUEST
			switch (pck_in.app_resource){
				case APP_RESOURCE_PING:
					application_ping_request_received(&pck_in);
				break;
				case APP_RESOURCE_ND:
					//application_method_not_available(&pck_in);
				break;
				case APP_RESOURCE_CONFIG:
					application_config_request_received(&pck_in);
				break;
				case APP_RESOURCE_SENSOR_LIGHT:
					application_va_request_received(&pck_in, 'L');
				break;
				case APP_RESOURCE_SENSOR_PRESSURE:
					application_va_request_received(&pck_in, 'P');
				break;
				case APP_RESOURCE_SENSOR_ACCELERATION:
					application_va_request_received(&pck_in, 'A');
				break;
				case APP_RESOURCE_SENSOR_TEMPERATURE:
					application_va_request_received(&pck_in, 'T');
				break;
				case APP_RESOURCE_ACTUATOR_LED:
					//application_method_not_available(&pck_in);
				break;
				default:
					//application_method_not_available(&pck_in);
				break;
			}
		}
	}
	return;
}

void application_process_periodically(){
	
	int i, j;
	portTickType t_now = xTaskGetTickCount(); //16 bits var
	
	
	if(g_config.g_periodically_app_neighbor_lookup_radius == APP_NEIGHBOR_LOOKUP_TYPE_SQUARE){
		for(i = - g_config.g_periodically_app_neighbor_lookup_radius ; i <= g_config.g_periodically_app_neighbor_lookup_radius ; i++){
			for(j = - g_config.g_periodically_app_neighbor_lookup_radius ; j <= g_config.g_periodically_app_neighbor_lookup_radius ; j++){
				 switch(g_config.g_periodically_sensor_type){
					 case APP_RESOURCE_SENSOR_LIGHT:
						application_va(PROTOCOL_UNICAST, i, j, 'L');
					 break;
					 case APP_RESOURCE_SENSOR_PRESSURE:
						application_va(PROTOCOL_UNICAST, i, j, 'P');
					 break;
					 case APP_RESOURCE_SENSOR_ACCELERATION:
						application_va(PROTOCOL_UNICAST, i, j, 'A');
					 break;
					 case APP_RESOURCE_SENSOR_TEMPERATURE:
						application_va(PROTOCOL_UNICAST, i, j, 'T');
					 break;
				 }
			}	
		}
		g_config.g_periodically_timestamp = t_now;
		g_config.g_periodically_update_status = 1;
	} 
	
	return;
}

void application_add_packet_to_OBSERVE_queue(xd_packet * pck_in, int obs_time){
	xQueueSendToBack( xQueue_packet_OBSERVE, pck_in,  ( portTickType ) (100 / portTICK_RATE_MS) * portTICK_RATE_MS_LESS_1_MS);
	application_packet_observe_cycle_time = 300;
	return;
}
void application_remove_packet_from_OBSERVE_queue(){
	xQueueReset( xQueue_packet_OBSERVE);
	application_packet_observe_cycle_time = 0;
	return;
}
void application_process_observe(){
	xd_packet pck_observe = {0};
	
	if( xQueuePeek( xQueue_packet_OBSERVE,  &pck_observe,  ( portTickType ) ( 10 / portTICK_RATE_MS) * portTICK_RATE_MS_LESS_1_MS) ) {
		application_add_packet_to_queue(&pck_observe);
	}
	return;
}

void application_build_header_response_packet(uint8_t protocol, xd_packet * pck_in, xd_packet * pck_out){
	
	pck_out->protocol = protocol; // pck_in->protocol; //uni, multi, broadcast
	pck_out->s_x = 0;
	pck_out->s_y = 0;
	pck_out->d_x = pck_in->s_x;
	pck_out->d_y = pck_in->s_y;
	
	pck_out->app_resource	= pck_in->app_resource;
	pck_out->app_method		= pck_in->app_method;
	pck_out->app_packet_type	= APP_PROTOCOL_RESPONSE;
	
	return;
}

void application_config(int protocol, int d_x, int d_y, char method /*(GET or SET)*/, int g_periodically_configured, int g_periodically_cycle_time, int g_periodically_sensor_type, int g_periodically_app_neighbor_lookup_type, int g_periodically_app_neighbor_lookup_radius)
{
	xd_packet pck_out = {0};

	pck_out.protocol = protocol; //uni, multi, broadcast
	pck_out.s_x = 0;
	pck_out.s_y = 0;
	pck_out.d_x = d_x;
	pck_out.d_y = d_y;
	//pck_out.serial = pck_serial_number[PROTOCOL_PING_SEQ];
	
	pck_out.app_packet_type	= APP_PROTOCOL_REQUEST;
	pck_out.app_resource	= APP_RESOURCE_CONFIG;
	pck_out.app_method		= method;
	
	if(method == APP_METHOD_SET){
		pck_out.payload[1] = g_config.g_periodically_configured;
		pck_out.payload[2] = (g_config.g_periodically_cycle_time & 0x00FF);
		pck_out.payload[3] = (g_config.g_periodically_cycle_time & 0xFF00) >> 8;
		pck_out.payload[4] = g_config.g_periodically_sensor_type;
		pck_out.payload[5] = g_config.g_periodically_app_neighbor_lookup_type;
		pck_out.payload[6] = g_config.g_periodically_app_neighbor_lookup_radius;
	}
	
	pck_out.port_origin = PORT_LOCAL;
	
	//pck_serial_number[PROTOCOL_PING_SEQ]++;
	
	switch_protocol_packet_analysis(PORT_LOCAL, &pck_out);
}
void application_config_request_received(xd_packet * pck_in)
{
	
	xd_packet pck_out = {0};
	
	application_build_header_response_packet(PROTOCOL_UNICAST, pck_in, &pck_out);
	
	if(pck_in->app_method == APP_METHOD_GET){
		pck_out.payload[0] = APP_OK;
		
		pck_out.payload[1] = g_config.g_periodically_configured;
		pck_out.payload[2] = (g_config.g_periodically_cycle_time & 0x00FF);
		pck_out.payload[3] = (g_config.g_periodically_cycle_time & 0xFF00) >> 8;
		pck_out.payload[4] = g_config.g_periodically_sensor_type;
		pck_out.payload[5] = g_config.g_periodically_app_neighbor_lookup_type;
		pck_out.payload[6] = g_config.g_periodically_app_neighbor_lookup_radius;
			
	} else if(pck_in->app_method == APP_METHOD_SET){
		pck_out.payload[0] = APP_OK;
		
		g_config.g_periodically_configured					= pck_in->payload[1];
		g_config.g_periodically_cycle_time					= pck_in->payload[2];
		g_config.g_periodically_cycle_time					+= (pck_in->payload[3] << 8);
		g_config.g_periodically_sensor_type					= pck_in->payload[4];
		g_config.g_periodically_app_neighbor_lookup_type	= pck_in->payload[5];
		g_config.g_periodically_app_neighbor_lookup_radius	= pck_in->payload[6];
		
		pck_out.payload[1] = pck_in->payload[1];
		pck_out.payload[2] = pck_in->payload[2];
		pck_out.payload[3] = pck_in->payload[3];
		pck_out.payload[4] = pck_in->payload[4];
		pck_out.payload[5] = pck_in->payload[5];
		pck_out.payload[6] = pck_in->payload[6];
		
	} else if(pck_in->app_method == APP_METHOD_RESET){
		pck_out.payload[0] = APP_OK;
		g_config.g_periodically_configured					= 0;
	
	} else {
		pck_out.payload[0]		= APP_METHOD_ERROR;
	}
	
	switch_protocol_packet_analysis(pck_in->port_origin, &pck_out);
	
	return;
}
void application_config_response_received(xd_packet * pck_in)
{	
	if (pck_in->payload[0] == APP_OK){
		printf("Node (s_x=%d s_y=%d): Settings received: periodically=%d cycle_time=%d sensor_type=%d lookup_type=%d  lookup_radius=%d [PORT=%d]\n\r", pck_in->s_x, pck_in->s_y, pck_in->payload[1],  pck_in->payload[2] & ( pck_in->payload[3] << 8),  pck_in->payload[4], pck_in->payload[5], pck_in->payload[6], pck_in->port_origin);
	} else {
		printf("Config failed! from (s_x=%d s_y=%d) ERROR: %d [PORT=%d]\n\r" , pck_in->s_x, pck_in->s_y, pck_in->payload[0], pck_in->port_origin);
	}
}

void application_ping(int protocol, int d_x, int d_y)
{
	portTickType time_ms_freertos = xTaskGetTickCount(); //16 bits var
	unsigned long time_ns_arm = SYST_CVR;
	
	xd_packet pck_out = {0};
	
	pck_out.protocol = protocol; //uni, multi, broadcast
	pck_out.s_x = 0;
	pck_out.s_y = 0;
	pck_out.d_x = d_x;
	pck_out.d_y = d_y;
	//pck_out.serial = pck_serial_number[PROTOCOL_PING_SEQ];
	
	pck_out.app_resource	= APP_RESOURCE_PING;
	pck_out.app_method		= APP_METHOD_GET;
	pck_out.app_packet_type	= APP_PROTOCOL_REQUEST;
	
	pck_out.payload[0]		= 0; //ping serial;
	
	pck_out.payload[1] = 0x000000FF & time_ms_freertos;
	pck_out.payload[2] = (time_ms_freertos & 0x0000FF00) >> 8;
	pck_out.payload[3] = (time_ms_freertos & 0x00FF0000) >> 16;
	pck_out.payload[4] = (time_ms_freertos & 0xFF000000) >> 24;
	
	pck_out.payload[5] = 0x000000FF & time_ns_arm;
	pck_out.payload[6] = (time_ns_arm & 0x0000FF00) >> 8;
	pck_out.payload[7] = (time_ns_arm & 0x00FF0000) >> 16;
	pck_out.payload[8] = (time_ns_arm & 0xFF000000) >> 24;
	
	pck_out.port_origin = PORT_LOCAL;
	
	//pck_serial_number[PROTOCOL_PING_SEQ]++;
	
	switch_protocol_packet_analysis(PORT_LOCAL, &pck_out);	
}
void application_ping_request_received(xd_packet * pck_in){
	xd_packet pck_out = {0};
		
	application_build_header_response_packet(PROTOCOL_UNICAST, pck_in, &pck_out);
	
	if(pck_in->app_method == APP_METHOD_GET){
			pck_out.payload[0]		= APP_OK;			
			pck_out.payload[1] =  pck_in->payload[1]; //time
			pck_out.payload[2] =  pck_in->payload[2]; //time
			pck_out.payload[3] =  pck_in->payload[3]; //time
			pck_out.payload[4] =  pck_in->payload[4]; //time
			pck_out.payload[5] =  pck_in->payload[5]; //time
			pck_out.payload[6] =  pck_in->payload[6]; //time
			pck_out.payload[7] =  pck_in->payload[7]; //time
			pck_out.payload[8] =  pck_in->payload[8]; //time
	} else {
			pck_out.payload[0]		= APP_METHOD_ERROR;
	}
	
	switch_protocol_packet_analysis(pck_in->port_origin, &pck_out); //send_packet
	
	return;
}
void application_ping_response_received(xd_packet * pck_in)
{
	char output[50] = {0};

	portTickType time_ms_freertos_now = xTaskGetTickCount() ; //16 bits var
	unsigned long time_ns_arm_now = SYST_CVR;
	
	portTickType time_ms_freertos_prev = 0;
	unsigned long time_ns_arm_prev = 0;
	
	
	unsigned long elapsed_time;
	
	
	if (pck_in->payload[0] == APP_OK){
		time_ms_freertos_prev = 0x000000FF & pck_in->payload[1];
		time_ms_freertos_prev |= (0x000000FF & pck_in->payload[2]) << 8; //TODO: get the time and subtract it to check the delay.
		time_ms_freertos_prev |= (0x000000FF & pck_in->payload[3]) << 16;
		time_ms_freertos_prev |= (0x000000FF & pck_in->payload[4]) << 24;
				
		time_ns_arm_prev = 0x000000FF & pck_in->payload[5];
		time_ns_arm_prev |= (0x000000FF & pck_in->payload[6]) << 8; //TODO: get the time and subtract it to check the delay.
		time_ns_arm_prev |= (0x000000FF & pck_in->payload[7]) << 16;
		time_ns_arm_prev |= (0x000000FF & pck_in->payload[8]) << 24;
		
		
		time_ns_arm_now = ((100000/ portTICK_RATE_MS_LESS_1_MS) - time_ns_arm_now) * 10; //SYSTICK ns
		time_ns_arm_prev = ((100000/ portTICK_RATE_MS_LESS_1_MS) - time_ns_arm_prev) * 10; //SYSTICK ns  
		
		active_neighbors[pck_in->port_origin] = true;		
		ping_delay[pck_in->port_origin] = (((time_ms_freertos_now * 1000000) / portTICK_RATE_MS_LESS_1_MS) + time_ns_arm_now ) -  (((time_ms_freertos_prev * 1000000) / portTICK_RATE_MS_LESS_1_MS) + time_ns_arm_prev );
		
		//if(ping_delay[pck_in->port_origin] > 1000000){
		//	printf("ERROR %lu %lu %lu %lu", time_ms_freertos_prev, time_ns_arm_prev, time_ms_freertos_now, time_ns_arm_now);
		//}
		
		
		//if (g_is_info_debugging)
		sprintf(output, "%llu", ping_delay[pck_in->port_origin]);
		printf("Reply from (%d,%d) [SYST_CVR+xTaskGetTickCount=%s] [PORT=%d]\n\r", pck_in->s_x, pck_in->s_y, output, pck_in->port_origin);
		//printf("[SYST_CVR] time start= %d time end= %d -> diff = %d \n\r", t_prev, t_now, t_now - t_prev);	
		//printf("[xTaskGetTickCount] time start= %d time end= %d -> diff = %d\n\r", t_prev_2, t_now_2, t_now_2 - t_prev_2);	
		
		//printf("Reply from (s_x=%d s_y=%d) time= %d ms [PORT=%d]\n\r", pck_in->s_x, pck_in->s_y, ping_delay[pck_in->port_origin], pck_in->port_origin);
		
				
		
	} else {
		printf("Reply from (s_x=%d s_y=%d) ERROR: %d [PORT=%d]\n\r", pck_in->s_x, pck_in->s_y, pck_in->payload[0], pck_in->port_origin);
	}
}

void application_va(int protocol, int d_x, int d_y, char va_type)
{
	xd_packet pck_out = {0};
	
	portTickType t_now = xTaskGetTickCount(); //16 bits var

	pck_out.protocol = protocol; //uni, multi, broadcast
	pck_out.s_x = 0;
	pck_out.s_y = 0;
	pck_out.d_x = d_x;
	pck_out.d_y = d_y;
	//pck_out.serial = pck_serial_number[PROTOCOL_PING_SEQ];
	
	pck_out.app_packet_type	= APP_PROTOCOL_REQUEST;
	
	switch(va_type){
		case 'L':
			pck_out.app_resource	= APP_RESOURCE_SENSOR_LIGHT;
		break;
		case 'P':
			pck_out.app_resource	= APP_RESOURCE_SENSOR_PRESSURE;
		break;
		case 'A':
			pck_out.app_resource	= APP_RESOURCE_SENSOR_ACCELERATION;
		break;
		case 'T':
			pck_out.app_resource	= APP_RESOURCE_SENSOR_TEMPERATURE;
		break;
	}
	
	pck_out.app_method		= APP_METHOD_GET;

	//pck_out.payload[0]		= APP_DATAPROCESSING_NONE;
	//pck_out.payload[1]		= APP_NEIGHBOR_LOOKUP_TYPE_SQUARE;
	//pck_out.payload[2]		= APP_NEIGHBOR_LOOKUP_RADIUS_NONE;
	
	
	
	pck_out.port_origin = PORT_LOCAL;
	
	//pck_serial_number[PROTOCOL_PING_SEQ]++;
	
	switch_protocol_packet_analysis(PORT_LOCAL, &pck_out);
}
void application_va_request_received(xd_packet * pck_in, char va_type)
{
	portTickType t_now = xTaskGetTickCount(); //32 bits var
	xd_packet pck_out = {0};
	xd_packet pck_out_2 = {0};
	xd_packet pck_out_3 = {0};
	
	application_build_header_response_packet(PROTOCOL_UNICAST, pck_in, &pck_out);
	application_build_header_response_packet(PROTOCOL_UNICAST, pck_in, &pck_out_2);
	application_build_header_response_packet(PROTOCOL_UNICAST, pck_in, &pck_out_3);
	
	Byte * sensor_value_raw;
	float sensor_value_float;
	
	if(pck_in->app_method == APP_METHOD_GET){
		switch(va_type){
			case 'L':
				if(resources.sensor.light.configured){
					pck_out.payload[0] = APP_OK;
					
					sensor_value_float = (float) resources.sensor.light.value;
					sensor_value_raw = (Byte *) &sensor_value_float;
					
					pck_out.payload[1] = sensor_value_raw[0];
					pck_out.payload[2] = sensor_value_raw[1];
					pck_out.payload[3] = sensor_value_raw[2];
					pck_out.payload[4] = sensor_value_raw[3];

						
				} else {
					pck_out.payload[0] = APP_SENSOR_NOT_AVAILABLE;
				}
				switch_protocol_packet_analysis(pck_in->port_origin, &pck_out);
			break;
			case 'P':
				if(resources.sensor.pressure.configured){
					pck_out.payload[0] = APP_OK;
					
					sensor_value_float = (float) resources.sensor.pressure.value;
					sensor_value_raw = (Byte *) &sensor_value_float;
					
					pck_out.payload[1] = sensor_value_raw[0];
					pck_out.payload[2] = sensor_value_raw[1];
					pck_out.payload[3] = sensor_value_raw[2];
					pck_out.payload[4] = sensor_value_raw[3];
					
				} else {
					pck_out.payload[0]  = APP_SENSOR_NOT_AVAILABLE;
				}
				switch_protocol_packet_analysis(pck_in->port_origin, &pck_out);
			break;
			case 'A':
				if(resources.sensor.acceleration.configured){			
					sensor_value_float = (float) resources.sensor.acceleration.cx;
					sensor_value_raw = (Byte *) &sensor_value_float;
					
					pck_out.payload[0] = 1 << 4 | APP_OK; //packet n 1 [X]
					pck_out.payload[1] = sensor_value_raw[0];
					pck_out.payload[2] = sensor_value_raw[1];
					pck_out.payload[3] = sensor_value_raw[2];
					pck_out.payload[4] = sensor_value_raw[3];
					
					switch_protocol_packet_analysis(pck_in->port_origin, &pck_out);
					vTaskDelay(1);

					
					sensor_value_float = (float) resources.sensor.acceleration.cy;
					sensor_value_raw = (Byte *) &sensor_value_float;
				
					pck_out_2.payload[0] = 2 << 4 | APP_OK; //packet n 2 [Y]
					pck_out_2.payload[1] = sensor_value_raw[0];
					pck_out_2.payload[2] = sensor_value_raw[1];
					pck_out_2.payload[3] = sensor_value_raw[2];
					pck_out_2.payload[4] = sensor_value_raw[3];

					switch_protocol_packet_analysis(pck_in->port_origin, &pck_out_2);
					vTaskDelay(1);
				
					sensor_value_float = (float) resources.sensor.acceleration.cz;
					sensor_value_raw = (Byte *) &sensor_value_float;
					
					pck_out_3.payload[0] = 3 << 4 | APP_OK; //packet n 3 [Z]
					pck_out_3.payload[1] = sensor_value_raw[0];
					pck_out_3.payload[2] = sensor_value_raw[1];
					pck_out_3.payload[3] = sensor_value_raw[2];
					pck_out_3.payload[4] = sensor_value_raw[3];

					switch_protocol_packet_analysis(pck_in->port_origin, &pck_out_3);
					vTaskDelay(1);
				} else {
					pck_out.payload[0]  = APP_SENSOR_NOT_AVAILABLE;
					switch_protocol_packet_analysis(pck_in->port_origin, &pck_out);
				}
				
			break;
			case 'T':
				if(resources.sensor.temperature.configured){
					pck_out.payload[0] = APP_OK;
					
					sensor_value_float = (float) resources.sensor.temperature.value;
					sensor_value_raw = (Byte *) &sensor_value_float;
					
					pck_out.payload[1] = sensor_value_raw[0];
					pck_out.payload[2] = sensor_value_raw[1];
					pck_out.payload[3] = sensor_value_raw[2];
					pck_out.payload[4] = sensor_value_raw[3];

					} else {
					pck_out.payload[0]	= APP_SENSOR_NOT_AVAILABLE;
				}
				switch_protocol_packet_analysis(pck_in->port_origin, &pck_out);
			break;
		}		
	} else if(pck_in->app_method == APP_METHOD_OBSERVE){
		pck_out.payload[0] = APP_OK;
		
		pck_in->app_method = APP_METHOD_GET;
		application_add_packet_to_OBSERVE_queue(pck_in, pck_in->payload[3]);
		
	} else if(pck_in->app_method == APP_METHOD_RESET){
		pck_out.payload[0] = APP_OK;
		application_remove_packet_from_OBSERVE_queue();
	
	} else {
		pck_out.payload[0]		= APP_METHOD_ERROR;
		switch_protocol_packet_analysis(pck_in->port_origin, &pck_out);
	}	
	
	return;
}
void application_va_response_received(xd_packet * pck_in, char va_type)
{
	float value_float = 0;
	int32_t value_int32_t = 0;
	char type_aux;
	char output[10];
		
	if (pck_in->payload[0] & 0x0F == APP_OK){
		switch(va_type){
			case 'L':
				value_int32_t = (int32_t) ((pck_in->payload[4] & 0xff) << 24) | ((pck_in->payload[3] & 0xff) << 16) | ((pck_in->payload[2] & 0xff) << 8) | (pck_in->payload[1] & 0xff);
				value_float = intBitsToFloat(value_int32_t);
				
				printf("Value [TYPE: %c] from (s_x=%d s_y=%d) V:%d [PORT=%d]\n\r", va_type, pck_in->s_x, pck_in->s_y, (int) value_float, pck_in->port_origin);
				
				if (g_config.g_periodically_configured && g_config.g_periodically_sensor_type==APP_RESOURCE_SENSOR_LIGHT && abs(pck_in->s_x) < (MAX_neighboor_N - 1 / 2) && abs(pck_in->s_y) < (MAX_neighboor_N - 1 / 2) ){
						neighborhood[(MAX_neighboor_N - 1 / 2) + pck_in->s_x][(MAX_neighboor_N - 1 / 2) + pck_in->s_y].configured = true;
						neighborhood[(MAX_neighboor_N - 1 / 2) + pck_in->s_x][(MAX_neighboor_N - 1 / 2) + pck_in->s_y].sensor_type = APP_RESOURCE_SENSOR_LIGHT;
						neighborhood[(MAX_neighboor_N - 1 / 2) + pck_in->s_x][(MAX_neighboor_N - 1 / 2) + pck_in->s_y].value = value_float;
				}
				
			break;
			case 'P':
				
				value_float = intBitsToFloat((int32_t) ((pck_in->payload[4] & 0xff) << 24) | ((pck_in->payload[3] & 0xff) << 16) | ((pck_in->payload[2] & 0xff) << 8) | (pck_in->payload[1] & 0xff));
				
				printf("Value [TYPE: %c] from (s_x=%d s_y=%d) V:%d.%d [PORT=%d]\n\r", va_type, pck_in->s_x, pck_in->s_y, (int) value_float, (int) ((value_float - (int) value_float) * 1000), pck_in->port_origin);
				
				if (g_config.g_periodically_configured && g_config.g_periodically_sensor_type==APP_RESOURCE_SENSOR_PRESSURE && abs(pck_in->s_x) < (MAX_neighboor_N - 1 / 2) && abs(pck_in->s_y) < (MAX_neighboor_N - 1 / 2) ){
					neighborhood[((MAX_neighboor_N - 1) / 2) + pck_in->s_x][((MAX_neighboor_N - 1) / 2) + pck_in->s_y].configured = true;
					neighborhood[((MAX_neighboor_N - 1) / 2) + pck_in->s_x][((MAX_neighboor_N - 1) / 2) + pck_in->s_y].sensor_type = APP_RESOURCE_SENSOR_PRESSURE;
					neighborhood[((MAX_neighboor_N - 1) / 2) + pck_in->s_x][((MAX_neighboor_N - 1) / 2) + pck_in->s_y].value = value_float;
				}
				
			break;
			case 'A': 
				
				value_float = intBitsToFloat((int32_t) ((pck_in->payload[4] & 0xff) << 24) | ((pck_in->payload[3] & 0xff) << 16) | ((pck_in->payload[2] & 0xff) << 8) | (pck_in->payload[1] & 0xff));
				
				if ((pck_in->payload[0] & 0xF0) >> 4 == 1 ){
					type_aux = 'x';
				} else if ((pck_in->payload[0] & 0xF0) >> 4 == 2) {
					type_aux = 'y';
				} else if ((pck_in->payload[0] & 0xF0) >> 4 == 3) {
					type_aux = 'z';
				}
				
				printf("Value [TYPE: %c] [%c] from (s_x=%d s_y=%d) V:",  va_type, type_aux , pck_in->s_x, pck_in->s_y);
				sprintf(output, "%f", value_float); printf(output);
				printf(" [PORT=%d]\n\r", pck_in->port_origin);
				
				if (g_config.g_periodically_configured && g_config.g_periodically_sensor_type==APP_RESOURCE_SENSOR_ACCELERATION && abs(pck_in->s_x) < (MAX_neighboor_N - 1 / 2) && abs(pck_in->s_y) < (MAX_neighboor_N - 1 / 2) ){
					neighborhood[((MAX_neighboor_N - 1) / 2) + pck_in->s_x][((MAX_neighboor_N - 1) / 2) + pck_in->s_y].configured = true;
					neighborhood[((MAX_neighboor_N - 1) / 2) + pck_in->s_x][((MAX_neighboor_N - 1) / 2) + pck_in->s_y].sensor_type = APP_RESOURCE_SENSOR_ACCELERATION;
					neighborhood[((MAX_neighboor_N - 1) / 2) + pck_in->s_x][((MAX_neighboor_N - 1) / 2) + pck_in->s_y].value = 0;
				}
				
				
			break;
			case 'T':
				
				value_float = intBitsToFloat((int32_t) ((pck_in->payload[4] & 0xff) << 24) | ((pck_in->payload[3] & 0xff) << 16) | ((pck_in->payload[2] & 0xff) << 8) | (pck_in->payload[1] & 0xff));
				
				printf("Value [TYPE: %c] from (s_x=%d s_y=%d) V:%d [PORT=%d]\n\r", va_type, pck_in->s_x, pck_in->s_y, (int) value_float, pck_in->port_origin);
				
				if (g_config.g_periodically_configured && g_config.g_periodically_sensor_type==APP_RESOURCE_SENSOR_TEMPERATURE && abs(pck_in->s_x) < (MAX_neighboor_N - 1 / 2) && abs(pck_in->s_y) < (MAX_neighboor_N - 1 / 2) ){
					neighborhood[((MAX_neighboor_N - 1) / 2) + pck_in->s_x][((MAX_neighboor_N - 1) / 2) + pck_in->s_y].configured = true;
					neighborhood[((MAX_neighboor_N - 1) / 2) + pck_in->s_x][((MAX_neighboor_N - 1) / 2) + pck_in->s_y].sensor_type = APP_RESOURCE_SENSOR_TEMPERATURE;
					neighborhood[((MAX_neighboor_N - 1) / 2) + pck_in->s_x][((MAX_neighboor_N - 1) / 2) + pck_in->s_y].value = value_float;
				}
				
			break;
		}
	} else if (pck_in->payload[0] == APP_SENSOR_NOT_AVAILABLE){
		printf("Value [TYPE: %c] from (s_x=%d s_y=%d) ERROR SENSOR_NOT_AVAILABLE [PORT=%d]\n\r", va_type, pck_in->s_x, pck_in->s_y, pck_in->port_origin);
	} else {
		printf("Value [TYPE: %c] from (s_x=%d s_y=%d) ERROR: %d [PORT=%d]\n\r", va_type, pck_in->s_x, pck_in->s_y, pck_in->payload[0], pck_in->port_origin);
	}
}

void application_poly_2(int protocol, int d_x, int d_y)
{
	portTickType t_now = xTaskGetTickCount(); //16 bits var
	
	xd_packet pck_out = {0};
	
	pck_out.protocol = protocol; //uni, multi, broadcast
	pck_out.s_x = 0;
	pck_out.s_y = 0;
	pck_out.d_x = d_x;
	pck_out.d_y = d_y;
	//pck_out.serial = pck_serial_number[PROTOCOL_PING_SEQ];
	
	pck_out.app_resource	= APP_RESOURCE_POLY_REGRESSION_D2;
	pck_out.app_method		= APP_METHOD_GET;
	pck_out.app_packet_type	= APP_PROTOCOL_REQUEST;
	
	pck_out.payload[0]		= 0; //ping serial;
	
	pck_out.payload[1] = 0x000000FF & t_now;
	pck_out.payload[2] = (t_now & 0x0000FF00) >> 8;
	pck_out.payload[3] = (t_now & 0x00FF0000) >> 16;
	pck_out.payload[4] = (t_now & 0xFF000000) >> 24;
	
	pck_out.port_origin = PORT_LOCAL;
	
	//pck_serial_number[PROTOCOL_PING_SEQ]++;
	
	switch_protocol_packet_analysis(PORT_LOCAL, &pck_out);
}
void application_poly_2_request_received(xd_packet * pck_in)
{
	portTickType t_now = xTaskGetTickCount(); //32 bits var
	
	xd_packet pck_out = {0};
	Byte * coeff_value_raw;
	
	application_build_header_response_packet(PROTOCOL_UNICAST, pck_in, &pck_out);
	
	
	if(pck_in->app_method == APP_METHOD_GET){
		char output[50];
		int i, j;
		
		float * x = (float *) pvPortMalloc(sizeof(float) * g_config.g_periodically_app_neighbor_lookup_radius * g_config.g_periodically_app_neighbor_lookup_radius);
		float * y = (float *) pvPortMalloc(sizeof(float) * g_config.g_periodically_app_neighbor_lookup_radius * g_config.g_periodically_app_neighbor_lookup_radius);
		float * v = (float *) pvPortMalloc(sizeof(float) * g_config.g_periodically_app_neighbor_lookup_radius * g_config.g_periodically_app_neighbor_lookup_radius);
		
		float coeff[6] = {0};
		float ms_error = 0;
		
		//TODO: improve error check!
		
		if (!g_config.g_periodically_app_neighbor_lookup_radius){
			pck_out.payload[0] = APP_NEIGHBOR_LOOKUP_RADIUS_ERROR;
			switch_protocol_packet_analysis(pck_in->port_origin, &pck_out);
			return;
		}
		
		
		for (i = - g_config.g_periodically_app_neighbor_lookup_radius; i <= g_config.g_periodically_app_neighbor_lookup_radius; i++){
			for (j = - g_config.g_periodically_app_neighbor_lookup_radius; j <= g_config.g_periodically_app_neighbor_lookup_radius; j++){
				x[(j + g_config.g_periodically_app_neighbor_lookup_radius) + ( g_config.g_periodically_app_neighbor_lookup_radius + i)] = j;
				y[(j + g_config.g_periodically_app_neighbor_lookup_radius) + ( g_config.g_periodically_app_neighbor_lookup_radius + i)] = i;
			}
		}
		
		for(i=0; i< (g_config.g_periodically_app_neighbor_lookup_radius * g_config.g_periodically_app_neighbor_lookup_radius); i++){
			v[i] = (neighborhood[(int8_t) (((MAX_neighboor_N - 1) / 2) + x[i]) ][ (int8_t) (((MAX_neighboor_N - 1) / 2) + y[i])].value);
		}
		
		polyfit2indepentvars_vlm(x, y, v, g_config.g_periodically_app_neighbor_lookup_radius * g_config.g_periodically_app_neighbor_lookup_radius, coeff, &ms_error);
		
		pck_out.payload[0] = 1 << 4 | APP_OK; //packet n 1 
		coeff_value_raw = (Byte *) &coeff[0]; //float to bytes! -> coeff [C0] + C1 * x + C2 * y + C3 * x^2 + C4 * y^2 + C5 * x.y
		pck_out.payload[1] = coeff_value_raw[0];
		pck_out.payload[2] = coeff_value_raw[1];
		pck_out.payload[3] = coeff_value_raw[2];
		pck_out.payload[4] = coeff_value_raw[3];
		coeff_value_raw = (Byte *) &coeff[1]; //float to bytes! -> coeff C0 + [C1] * x + C2 * y + C3 * x^2 + C4 * y^2 + C5 * x.y
		pck_out.payload[5] = coeff_value_raw[0];
		pck_out.payload[6] = coeff_value_raw[1];
		pck_out.payload[7] = coeff_value_raw[2];
		pck_out.payload[8] = coeff_value_raw[3];
		switch_protocol_packet_analysis(pck_in->port_origin, &pck_out);
		
		pck_out.payload[0] = 2 << 4 | APP_OK; //packet n 2
		coeff_value_raw = (Byte *) &coeff[2]; //float to byte! -> coeff C0 + C1 * x + [C2] * y + C3 * x^2 + C4 * y^2 + C5 * x.y
		pck_out.payload[1] = coeff_value_raw[0];
		pck_out.payload[2] = coeff_value_raw[1];
		pck_out.payload[3] = coeff_value_raw[2];
		pck_out.payload[4] = coeff_value_raw[3];
		coeff_value_raw = (Byte *) &coeff[3]; //float to byte! -> coeff C0 + C1 * x + C2 * y + [C3] * x^2 + C4 * y^2 + C5 * x.y
		pck_out.payload[5] = coeff_value_raw[0];
		pck_out.payload[6] = coeff_value_raw[1];
		pck_out.payload[7] = coeff_value_raw[2];
		pck_out.payload[8] = coeff_value_raw[3];
		switch_protocol_packet_analysis(pck_in->port_origin, &pck_out);
		
		pck_out.payload[0] = 3 << 4 | APP_OK; //packet n 3
		coeff_value_raw = (Byte *) &coeff[4]; //float to byte! -> coeff C0 + C1 * x + C2 * y + C3 * x^2 + [C4] * y^2 + C5 * x.y
		pck_out.payload[1] = coeff_value_raw[0];
		pck_out.payload[2] = coeff_value_raw[1];
		pck_out.payload[3] = coeff_value_raw[2];
		pck_out.payload[4] = coeff_value_raw[3];
		coeff_value_raw = (Byte *) &coeff[5]; //float to byte! -> coeff [C0] + C1 * x + C2 * y + C3 * x^2 + C4 * y^2 + [C5] * x.y
		pck_out.payload[5] = coeff_value_raw[0];
		pck_out.payload[6] = coeff_value_raw[1];
		pck_out.payload[7] = coeff_value_raw[2];
		pck_out.payload[8] = coeff_value_raw[3];
		switch_protocol_packet_analysis(pck_in->port_origin, &pck_out);
		
		pck_out.payload[0] = 4 << 4 | APP_OK; //packet n 4
		coeff_value_raw = (Byte *) &ms_error; //float to byte! -> Mean square error!
		pck_out.payload[1] = coeff_value_raw[0];
		pck_out.payload[2] = coeff_value_raw[1];
		pck_out.payload[3] = coeff_value_raw[2];
		pck_out.payload[4] = coeff_value_raw[3];
		coeff_value_raw = NULL;				// empty
		pck_out.payload[5] = 0;
		pck_out.payload[6] = 0;
		pck_out.payload[7] = 0;
		pck_out.payload[8] = 0;
		switch_protocol_packet_analysis(pck_in->port_origin, &pck_out);
		
		vPortFree(x);
		vPortFree(y);
		vPortFree(v);
		
			
	} else if(pck_in->app_method == APP_METHOD_OBSERVE){
		
		pck_out.payload[0] = APP_OK;
		pck_in->app_method = APP_METHOD_GET;
		application_remove_packet_from_OBSERVE_queue();
		application_add_packet_to_OBSERVE_queue(pck_in, pck_in->payload[3]);
		switch_protocol_packet_analysis(pck_in->port_origin, &pck_out);
		
	} else if(pck_in->app_method == APP_METHOD_RESET){
		pck_out.payload[0] = APP_OK;
		application_remove_packet_from_OBSERVE_queue();
		switch_protocol_packet_analysis(pck_in->port_origin, &pck_out);
		
	} else {
		pck_out.payload[0]		= APP_METHOD_ERROR;
		switch_protocol_packet_analysis(pck_in->port_origin, &pck_out);
	}
	

	
	return;
}
void application_poly_2_response_received(xd_packet * pck_in)
{
	char output[50];
	float coeff_float[2];
	
	
	if (pck_in->payload[0] & 0x0F == APP_OK){
		switch(pck_in->payload[0] & 0xF0 >> 4){
			case 1:

				coeff_float[0] = intBitsToFloat((int32_t) (pck_in->payload[4] & 0xff << 24) | ((pck_in->payload[3] & 0xff) << 16) | ((pck_in->payload[2] & 0xff) << 8) | (pck_in->payload[1] & 0xff));
				coeff_float[1] = intBitsToFloat((int32_t) (pck_in->payload[8] & 0xff << 24) | ((pck_in->payload[7] & 0xff) << 16) | ((pck_in->payload[6] & 0xff) << 8) | (pck_in->payload[5] & 0xff));
			
				printf("Regression from (s_x=%d s_y=%d) [PORT=%d] y= ", pck_in->s_x, pck_in->s_y, pck_in->port_origin);
			
				sprintf(output, "%c %f ",		coeff_float[0] > 0 ? '+' : ' ', coeff_float[0]);	printf(output);
				sprintf(output, "%c %f * x ",	coeff_float[1] > 0 ? '+' : ' ', coeff_float[1]);	printf(output);
						
			break;
			case 2:
				coeff_float[0] = intBitsToFloat((int32_t) (pck_in->payload[4] & 0xff << 24) | ((pck_in->payload[3] & 0xff) << 16) | ((pck_in->payload[2] & 0xff) << 8) | (pck_in->payload[1] & 0xff));
				coeff_float[1] = intBitsToFloat((int32_t) (pck_in->payload[8] & 0xff << 24) | ((pck_in->payload[7] & 0xff) << 16) | ((pck_in->payload[6] & 0xff) << 8) | (pck_in->payload[5] & 0xff));
											
				sprintf(output, "%c %f * y ",	coeff_float[0] > 0 ? '+' : ' ', coeff_float[0]);	printf(output);
				sprintf(output, "%c %f * x^2",	coeff_float[1] > 0 ? '+' : ' ', coeff_float[1]);	printf(output);

			break;
			case 3:
				coeff_float[0] = intBitsToFloat((int32_t) (pck_in->payload[4] & 0xff << 24) | ((pck_in->payload[3] & 0xff) << 16) | ((pck_in->payload[2] & 0xff) << 8) | (pck_in->payload[1] & 0xff));
				coeff_float[1] = intBitsToFloat((int32_t) (pck_in->payload[8] & 0xff << 24) | ((pck_in->payload[7] & 0xff) << 16) | ((pck_in->payload[6] & 0xff) << 8) | (pck_in->payload[5] & 0xff));
			
				sprintf(output, "%c %f * y^2",	coeff_float[0] > 0 ? '+' : ' ', coeff_float[0]);	printf(output);
				sprintf(output, "%c %f * x.y \n\r",	coeff_float[1] > 0 ? '+' : ' ', coeff_float[1]);	printf(output);
			
			
			break;
			case 4:
				coeff_float[0] = intBitsToFloat((int32_t) (pck_in->payload[4] & 0xff << 24) | ((pck_in->payload[3] & 0xff) << 16) | ((pck_in->payload[2] & 0xff) << 8) | (pck_in->payload[1] & 0xff));
					
				printf("Regression from (s_x=%d s_y=%d) [PORT=%d] Mean square error = ", pck_in->s_x, pck_in->s_y, pck_in->port_origin);
			
				sprintf(output, "%c %f \n\r",		coeff_float[0] > 0 ? '+' : ' ', coeff_float[0]);	printf(output);
	
			break;
		}
	} else if (pck_in->payload[0] & 0x0F == APP_NEIGHBOR_LOOKUP_RADIUS_ERROR){
		printf("Regression from (s_x=%d s_y=%d) APP_NEIGHBOR_LOOKUP_RADIUS_ERROR [PORT=%d]\n\r", pck_in->s_x, pck_in->s_y, pck_in->port_origin);
	} else {
		printf("Regression from (s_x=%d s_y=%d) ERROR: %d [PORT=%d]\n\r", pck_in->s_x, pck_in->s_y, pck_in->payload[0], pck_in->port_origin);
	}
}




/*
void application_nd(uint8_t ports)
{
		
	pck_out.protocol = PROTOCOL_ND;
	pck_out.x = 0; //myself
	pck_out.y = 0; //myself
	pck_out.payload[0] = g_nodes_config;
	pck_out.payload[1] = g_nodes_config_sampling_period;
	pck_out.payload[2] = 0;
	pck_out.payload[3] = 0;
	
	pck_out.serial = pck_serial_number[PROTOCOL_ND_SEQ]++;
	
	switch_write_packet(ports, &pck_out);
	
	if (g_is_led_debugging) leds[PORT_SOUTH].b_blink_duration = 1 * LED_BLINK_MIN_PERIOD;
}

void application_nd_received(xd_packet * pck_in)
{
	uint8_t i;
	i = 0;
	
	uint8_t config = 0;
	
	if (sink_list[i].active == false || sink_list[i].serial_number != pck_in->serial) //if the slot was not assigned yet
	{
		sink_list[i].active = true;
		sink_list[i].serial_number = pck_in->serial;
		sink_list[i].x = pck_in->x;
		sink_list[i].y = pck_in->y;
		
		if (g_is_led_debugging) leds[PORT_SOUTH].g_blink_duration = 10 * LED_BLINK_MIN_PERIOD;
		if (g_is_debugging) printf("Sink at (%d,%d) added\r\n", sink_list[i].x, sink_list[i].y);
		
		config = pck_in->payload[0];
		
		//Extract the configuration

		g_is_debugging		= 0b00000001	& (config >> 4);
		g_is_led_debugging	= 0b00000001	& (config >> 3);
		g_ea_periodically	= 0b00000001	& (config >> 2);
		g_nd_periodically	= 0b00000001	& (config >> 1);
		g_va_periodically	= 0b00000001	& (config >> 0);
		
		if (pck_in->payload[1] > 0)
		{
			g_va_period = pck_in->payload[1];
		}
		
				
		uint8_t ports = PORT_ALL_MASK &  ~port_to_mask(pck_in->port_origin); //All except the the port it came from
		
		//application_ea(); // in this way, it works like pooling based, so the sink keeps sending nd, to get data back
		
		switch_write_packet(ports, pck_in); //forward the packet to the remaining ports so it continue in the network
		
		if (g_is_led_debugging_min) leds[PORT_SOUTH].g_blink_duration = 5 * LED_BLINK_MIN_PERIOD;
	}
}

void application_va(uint8_t ports)
{
	pck_out.protocol = PROTOCOL_VA_LIGHT;
	pck_out.x = 0; //myself
	pck_out.y = 0; //myself
	
	pck_out.payload[0] = resources.sensor.light.value & 0x00FF;
	pck_out.payload[1] = (resources.sensor.light.value & 0xFF00) >> 8;
	pck_out.payload[2] = 0;
	pck_out.payload[3] = 0;
	
	pck_out.serial = pck_serial_number[PROTOCOL_VA_LIGHT_SEQ]++;
	
	switch_write_packet(ports, &pck_out);
	
	if (g_is_led_debugging) leds[PORT_SOUTH].b_blink_duration = 1 * LED_BLINK_MIN_PERIOD;
}

void application_va_received(xd_packet * pck_in)
{
	uint8_t i = pck_in->port_origin;
	uint16_t sensor_value = 0;
	uint16_t delta = 0;
	uint16_t delta_new = 0;
	uint8_t ports = 0;
	
	
	if (neighbors_list[i].serial_number != pck_in->serial) //if the slot was not assigned yet
	{
		neighbors_list[i].serial_number = pck_in->serial;
		neighbors_list[i].x = pck_in->x;
		neighbors_list[i].y = pck_in->y;
		neighbors_list[i].active = true;
		neighbors_list[i].payload[0] = pck_in->payload[0];
		neighbors_list[i].payload[1] = pck_in->payload[1];
		neighbors_list[i].payload[2] = pck_in->payload[2];		
		
		uint8_t ports = PORT_ALL_MASK &  ~port_to_mask(pck_in->port_origin); //All except the the port it came from
		
		//switch_write_packet(ports, pck_in); //forward the packet to the remaining ports so it continue in the network
		
		
		
		if (g_is_debugging) 
		{
			//printf("Neighbor added\r\n);
		}
		
		if (g_is_led_debugging) leds[PORT_SOUTH].g_blink_duration = 1 * LED_BLINK_MIN_PERIOD;
	}
	
	
	for (i = 0 ; i < PORT_COUNT ; i++)
	{
		if (neighbors_list[i].active == true)
		{
			uint16_t a, b;
			a = (uint16_t) resources.sensor.light.value;
			b = (neighbors_list[i].payload[1] << 8) | neighbors_list[i].payload[0];
			delta_new = calc_major(a, b) - calc_minor(a,b);
		
			//if (g_is_debugging) printf("d=%d\r\n", delta_new);
			
			
		
			if (delta_new > THRESHOLD_EDGE && b == calc_minor(a,b)) //if they are in the valey
			{
				ports |= port_to_mask(i);
				if (delta_new > delta) delta = delta_new;
				if (g_is_led_debugging_min) leds[PORT_SOUTH].r_blink_duration = LED_ERROR * LED_BLINK_MIN_PERIOD;
			}
		}
	}
	
	if (ports != 0)// && (delta > delta_previous * 1.1 || delta < delta_previous * 0.9))
	{
		//application_ea(PROTOCOL_EA_STEP, delta, ports);
		delta_previous = delta;
	}
	else if (ports == 0 && delta_previous > delta * 20) // nothing detected, but previously yes
	{
		//application_ea(PROTOCOL_EA_STEP, delta, ports);
		delta_previous = delta;		
	}
}

void application_ea(uint8_t protocol, uint16_t data, uint8_t port)
{
	pck_out.protocol = protocol;
	//pck_out.x = sink_list[0].x; //my nearest sink
	//pck_out.y = sink_list[0].y; 
	
	pck_out.x = 0; //my nearest sink
	pck_out.y = 0;
	
	if(data > 0b0000111111111111) data = 0b0000111111111111;
	
	pck_out.payload[0] = ((uint16_t) resources.sensor.light.value) & 0x00FF;
	pck_out.payload[1] = (((uint16_t) resources.sensor.light.value) & 0xFF00) >> 8;
	pck_out.payload[2] = data & 0x00FF;
	pck_out.payload[3] = (data & 0xFF00) >> 8;
	//pck_out.payload[3] &= 0b00111111;
	pck_out.payload[3] = (pck_out.payload[3] & 0x0F) | (port_to_mask(port) << 4);
	
	
	
	if (sink_list[0].active)
	{
		uint8_t ports = route_to(sink_list[0].x, sink_list[0].y);
	
		pck_out.serial = pck_serial_number[protocol]++;
	
		switch_write_packet(ports, &pck_out);	
		//packet_print(&pck_out);
		if (g_is_led_debugging_min) leds[PORT_SOUTH].b_blink_duration = 1 * LED_BLINK_MIN_PERIOD;	
	}
	else
	{
		if (g_is_led_debugging) leds[PORT_SOUTH].r_blink_duration = LED_ERROR * LED_BLINK_MIN_PERIOD;
	}
	
	
}

void application_ea_received(xd_packet * pck_in)
{
	uint8_t i = pck_in->port_origin;
	
	//sink_list[0].active = true;
	//sink_list[0].x = -2;
	//sink_list[0].y = 0;
	
	if (sink_list[0].active && g_is_sink == false)
	{
		uint8_t port = route_to(sink_list[0].x, sink_list[0].y);
		//port = PORT_ALL_MASK &  ~port_to_mask(pck_in->port_origin); //for debugging
		switch_write_packet(port, pck_in); //forward the packet to the remaining ports so it continue in the network
		if (g_is_led_debugging_min) leds[PORT_SOUTH].g_blink_duration = 1 * LED_BLINK_MIN_PERIOD;
	}
	else if (g_is_sink)
	{
		if (g_is_led_debugging_min) leds[PORT_SOUTH].g_blink_duration = 1 * LED_BLINK_MIN_PERIOD;
		packet_print(pck_in);
		if(g_is_debugging) switch_write_packet(PORT_SOUTH_MASK, pck_in);
	}
	else
	{
		if (g_is_led_debugging) leds[PORT_SOUTH].r_blink_duration = LED_ERROR * LED_BLINK_MIN_PERIOD;
	}

}
*/
uint8_t port_to_mask(uint8_t in)
{
	return 0b00000001 << in;
}

uint8_t route_to(uint8_t x, uint8_t y)
{
	uint8_t ports = 0;
	
	if ( y > 0)
	{
		return PORT_SOUTH_MASK;
	}
	
	if ( y < 0)
	{
		return PORT_NORTH_MASK;
	}
	
	if ( x > 0)
	{
		return PORT_WEST_MASK;
	}
	if (x < 0)
	{
		return PORT_EAST_MASK;
	}
	
	return ports;
}

uint16_t calc_major (uint16_t a, uint16_t b)
{
	if (a>=b)
		return a;
	else
		return b;
}

uint16_t calc_minor (uint16_t a, uint16_t b)
{
	if (a<b)
		return a;
	else
		return b;
}

//uint8_t add_to_list(node * nodes_list, uint8_t node_list_size, node node_to_add)
//{
//uint8_t i;
//
//for (i = 0 ; i < node_list_size ; i++)
//{
//
//
//if (nodes_list[i]->active == false) //if the slot was not assigned yet
//{
//nodes_list[i]->active = true;
//nodes_list[i]->serial_number = node_to_add.serial_number;
//nodes_list[i]->x = node_to_add.x;
//nodes_list[i]->y = node_to_add.y;
//
//
//}
//else
//{
//if ( nodes_list[i]->x == node_to_add.x && nodes_list[i]->y == node_to_add.y && nodes_list[i]->serial_number != node_to_add.serial_number)
////the same sink, but an updated packet, maybe with some new operational config
//{
//
//}
//}
//
//}
//}