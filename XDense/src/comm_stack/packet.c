/*
 * packet.c
 *
 * Created: 20/11/2014 15:05:27
 *  Author: Joao
 */ 

#include "comm_stack/packet.h"

void init_packet_queue(){
	//fifo_init(&fifo_packet_queue, fifo_buffer_packet_queue, PACKET_QUEUE_N);
}

xd_packet * get_pck_id_packet_queue(){
	//fifo_pull_uint8(&fifo_packet_queue, &data_out);
	xd_packet * pck = (xd_packet *) pvPortMalloc(sizeof(xd_packet));
	return pck;
}
void free_pck_id_packet_queue(xd_packet * pck){
	//fifo_push_uint8(&fifo_packet_queue, data);
	vPortFree(pck);
	return;
}


Bool BCC_check(char *buff, int len){
	int i;
	char BCC=0x00;
	for(i=0; i < len; i++){
		BCC^=buff[i];
	}
	if(BCC!=0x00){
		return false;
	} else {
		return true;	
	}
}
char BCC_calc(char *buff, int len){
	int i;
	char BCC=0x00;
	for(i=0; i < len-1; i++){
		BCC^=buff[i];
	}
	return BCC;
}

uint8_t app_settings_to_byte(uint8_t resource, uint8_t method, uint8_t packet_type){
	
	return (resource<<3 | method << 1 | packet_type);
}

void app_settings_from_byte(uint8_t byte, uint8_t * resource, uint8_t * method, uint8_t * packet_type){
	
	*resource		= ( byte & (0b11111000)) >> 3;
	*method			= ( byte & (0b00000110)) >> 1;
	*packet_type	= ( byte & (0b00000001));
	
	return;
}

void packet_serialize(xd_packet * input_pck, char * output_char)
{	
	output_char[0] = input_pck->protocol;
	output_char[1] = input_pck->s_x;
	output_char[2] = input_pck->s_y;
	output_char[3] = input_pck->d_x;
	output_char[4] = input_pck->d_y;
	output_char[5] = app_settings_to_byte(input_pck->app_resource, input_pck->app_method, input_pck->app_packet_type);
	output_char[6] = input_pck->payload[0]; //REQUEST: (resource<<3 | method << 1 | packet_type) // RESPONSE: status/error message
	output_char[7] = input_pck->payload[1]; 
	output_char[8] = input_pck->payload[2];
	output_char[9] = input_pck->payload[3];
	output_char[10] = input_pck->payload[4];
	output_char[11] = input_pck->payload[5];
	output_char[12] = input_pck->payload[6];
	output_char[13] = input_pck->payload[7];
	output_char[14] = input_pck->payload[8];	
	output_char[15] = BCC_calc(output_char, PACKET_SIZE);
	
	return;
}
Bool packet_deserialize(char * input_char, xd_packet * output_pck)
{
	char char_start, char_end;
	char_start = input_char[0];
	char_end = input_char[PACKET_SIZE-1];
	

	
	if (BCC_check(input_char, PACKET_SIZE)){ //TODO: enhance error checking and data reconstruction, like md5 crc etc...
		
		output_pck->protocol = input_char[0];
		
		//output_pck->serial = input_char[1];
		
		output_pck->s_x = input_char[1];
		output_pck->s_y = input_char[2];
		
		output_pck->d_x = input_char[3];
		output_pck->d_y = input_char[4];
		
		app_settings_from_byte(input_char[5], &(output_pck->app_resource), &(output_pck->app_method), &(output_pck->app_packet_type));
		
		output_pck->payload[0] = input_char[6];
		output_pck->payload[1] = input_char[7];
		output_pck->payload[2] = input_char[8];
		output_pck->payload[3] = input_char[9];
		output_pck->payload[4] = input_char[10];
		output_pck->payload[5] = input_char[11];
		output_pck->payload[6] = input_char[12];
		output_pck->payload[7] = input_char[13];
		output_pck->payload[8] = input_char[14];
		
		return true;
	} else {
		return false;
	}

}

/*
void packet_print(xd_packet * pck)
{
	//xTaskSuspendScheduler();
	
	portTickType t_now = xTaskGetTickCount(); //16 bits var
	
	printf("t=%05d;@%03d;%03d;%03d;%03d;%03d;%03d;%03d;%03d;%03d#\r\n",
	t_now,
	pck->protocol,
	pck->serial,
	pck->s_x, pck->s_y,
	pck->payload[0], pck->payload[1], pck->payload[2], pck->payload[3],
	pck->port_origin
	);

	//printf("p\n");
	
	//char s[12];
	//
	//packet_deserialize(s , pck);
	//
	//s[10] = pck->port_origin;
	//s[11] = NULL;
	//
	//puts(s);
}*/