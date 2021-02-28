/*
 * main.h
 *
 * Created: 21/01/2015 20:22:28
 *  Author: Joao
 */ 


#ifndef MAIN_H_
#define MAIN_H_


/* Globals */
//packet pck;

bool g_is_debugging;
bool g_is_led_debugging;
bool g_is_led_debugging_min;
bool g_is_info_debugging;
bool g_is_sink;
bool g_va_periodically;
bool g_nd_periodically;
bool g_ea_periodically;
bool g_do_nothing;

uint8_t g_va_period;

#define CONFIG_VA_PERIODICALLY_MASK		0b00000001
#define CONFIG_ND_PERIODICALLY_MASK		0b00000010
#define CONFIG_EA_PERIODICALLY_MASK		0b00000100
#define CONFIG_LED_DEBUGGING_MASK		0b00001000
#define CONFIG_DEBUGGING_MASK			0b00010000
#define CONFIG_LED_DEBUGGING_MIN_MASK	0b00100000

uint8_t g_nodes_config;
uint8_t g_nodes_config_sampling_period;

bool g_comm_unit_test;


//CONFIGS
typedef struct{
	uint8_t		g_periodically_configured;
	uint8_t		g_periodically_update_status;
	uint16_t	g_periodically_cycle_time;
	int			g_periodically_sensor_type;
	int			g_periodically_app_neighbor_lookup_type;
	int			g_periodically_app_neighbor_lookup_radius;
	double		g_periodically_timestamp;
} g_configs;

g_configs g_config;


//NEIGHBOR
#define MAX_neighboor_N 9

typedef struct{
	bool configured;
	bool update_status;
	int sensor_type;
	float value;
	double timestamp;
} g_neighboor;

g_neighboor neighborhood[MAX_neighboor_N][MAX_neighboor_N];

//SENSORS
typedef struct{
	bool configured;
	bool update_status;
	double value;
	double timestamp;
} g_sensor_light;
typedef struct{
	bool configured;
	bool update_status;
	double value;
	double timestamp;
} g_sensor_pressure;
typedef struct{
	bool configured;
	bool update_status;
	double x, y, z; // raw values
	double cx, cy, cz; //calculated values (g)
	double timestamp;
} g_sensor_acceleration;
typedef struct{
	bool configured;
	bool update_status;
	double value;
	double timestamp;
} g_sensor_temperature;

typedef struct{
	g_sensor_light			light;
	g_sensor_temperature	temperature;
	g_sensor_pressure		pressure;
	g_sensor_acceleration	acceleration;
} g_sensors;

//ACTUATORS
typedef struct{
	//TODO: Leds
} g_actuators;

typedef struct{
	g_sensors	sensor;
	g_actuators actuator;
} g_resources;

g_resources resources;


void scan_i2c(void);
void simulate_polyfit_regression(int size, int type);

#endif /* MAIN_H_ */