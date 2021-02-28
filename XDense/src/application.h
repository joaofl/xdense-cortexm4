/*
 * application.h
 *
 * Created: 20/01/2015 17:32:34
 *  Author: Joao
 */ 


#ifndef APPLICATION_H_
#define APPLICATION_H_

#include "comm_stack/packet.h"
#include "comm_stack/switch.h"
#include "main.h"

#define NEIGHBORHOOD_SIZE				1
#define NEIGHBORHOOD_COUNT				4 //2 * NEIGHBORHOOD_SIZE * (NEIGHBORHOOD_SIZE - 1)
#define MAX_SINKS						1

#define THRESHOLD_EDGE					2500

typedef struct  
{
	uint8_t x;
	uint8_t y;
	uint8_t serial_number;
	uint8_t payload[4];
	bool active;
}node;

uint16_t delta_previous;

bool active_neighbors[PORT_COUNT];

//uint16_t previous

static node neighbors_list[NEIGHBORHOOD_COUNT];

unsigned long long ping_delay[PORT_COUNT];

static node sink_list[MAX_SINKS];

//static uint8_t pck_serial_number[PROTOCOL_COUNT];

static uint16_t calc_major (uint16_t a, uint16_t b);

static uint16_t calc_minor (uint16_t a, uint16_t b);


xQueueHandle xQueue_packet_input; 
xQueueHandle xQueue_packet_OBSERVE;

int application_packet_observe_cycle_time;

void application_initialize(void);
void application_add_packet_to_queue(xd_packet * pck_in);
void application_process_packet_received();

void application_process_periodically();

void application_add_packet_to_OBSERVE_queue(xd_packet * pck_in, int obs_time);
void application_remove_packet_from_OBSERVE_queue();
void application_process_observe();

void application_config(int protocol, int d_x, int d_y, char method /*(GET or SET)*/, int g_periodically_configured, int g_periodically_cycle_time, int g_periodically_sensor_type, int g_periodically_app_neighbor_lookup_type, int g_periodically_app_neighbor_lookup_radius);
void application_config_request_received(xd_packet * pck_in);
void application_config_response_received(xd_packet * pck_in);

void application_ping(int protocol, int d_x, int d_y);
void application_ping_request_received(xd_packet * pck_in);
void application_ping_response_received(xd_packet * pck_in);

void application_va(int protocol, int d_x, int d_y, char va_type);
void application_va_request_received(xd_packet * pck_in, char va_type);
void application_va_response_received(xd_packet * pck_in, char va_type);

void application_nd(uint8_t ports);
void application_nd_received(xd_packet * pck_in);
void application_ea(uint8_t, uint16_t data, uint8_t port_mask);
void application_ea_received(xd_packet * pck_in);
uint8_t port_to_mask(uint8_t in);
uint8_t route_to(uint8_t destination_x, uint8_t destination_y);


#endif /* APPLICATION_H_ */