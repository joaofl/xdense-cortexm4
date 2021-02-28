/**
 * \file
 *
 * \brief Empty user application template
 *
 */

/**
 * \mainpage User Application template doxygen documentation
 *
 * \par Empty user application template
 *
 * Bare minimum empty user application template
 *
 * \par Content
 *
 * -# Include the ASF header files (through asf.h)
 * -# "Insert system clock initialization code here" comment
 * -# Minimal main function that starts with a call to board_init()
 * -# "Insert application code here" comment
 *
 */


/*
 * Include header files for all drivers that have been imported from
 * Atmel Software Framework (ASF).
 */

//TODO: Processador 50Mhz

#include "asf.h"
#include "application.h"
#include "drivers\led.h"
#include "drivers\uart.h"
#include "drivers\twi.h"
#include "drivers\tmp102.h"
#include "drivers\tsl4531.h"
#include "drivers\bmp180.h"
#include "drivers\mma8452q.h"

#include "comm_stack\switch.h"
#include "comm_stack\net_device.h"
#include "comm_stack\packet.h"

#include "tasks.h"

#include "math_lib.h"

#include "main.h"

#if configUSE_TRACE_FACILITY
	#include "trcRecorder.h"
#endif


#define TASKS_STACK_SIZE					(2048/sizeof(portSTACK_TYPE))
#define TASK_LED_STACK_SIZE					(300/sizeof(portSTACK_TYPE))
#define TASK_RECEIVE_DATA_STACK_SIZE		(2048/sizeof(portSTACK_TYPE)) //6144
#define TASK_SEND_DATA_STACK_SIZE		(500/sizeof(portSTACK_TYPE)) //6144

#define TASKS_STACK_PRIORITY				(tskIDLE_PRIORITY)
#define TASKS_SENSOR_PRIORITY				(tskIDLE_PRIORITY + 1)
#define TASKS_DATA_PROC_PRIORITY			(tskIDLE_PRIORITY + 2)
#define TASK_SEND_DATA_STACK_PRIORITY		(tskIDLE_PRIORITY + 3)
#define TASK_RECEIVE_DATA_STACK_PRIORITY	(tskIDLE_PRIORITY + 3)

#define PERIOD_SENSOR_READING 100
#define PERIOD_EA 100
#define INITIAL_DELAY 1000


#define configUSE_TRACE 0

/*
 * FreeRTOS hook (or callback) functions that are defined in this file.
 */
void vApplicationMallocFailedHook(void);
void vApplicationIdleHook(void);
void vApplicationStackOverflowHook(xTaskHandle pxTask, signed char *pcTaskName);
void vApplicationTickHook(void);

/*--------------------------------------------------------------------------------*/


static void led_task_blink(void *pvParameters)
{
	UNUSED(pvParameters);
	

	//for (uint8_t j = 0 ; j < PORT_COUNT ; j++){
		//leds[j].r = true;
		//leds[j].g = true;
		//leds[j].b = true;
	//}
	
	//portTickType xLastWakeTime;
  //
	//xLastWakeTime = xTaskGetTickCount();
	
	led_set_all(0x0000);
	
	//uint16_t v = 0b0000000001000000;
		
	for (;;) 
	{
		
		//vTaskDelayUntil(&xLastWakeTime, LED_BLINK_MIN_PERIOD);
		
		//led_set_all(0xFFFF); //turn all on
		//led_on(DIR_SOUTH, COLOR_R;)
		//
		//led_on(PORT_WEST, COLOR_R); //turn all on
		//vTaskDelay(25);
		//led_off(PORT_WEST, COLOR_R);
		//vTaskDelay(100);
		//led_on(PORT_WEST, COLOR_R); //turn all on
		//vTaskDelay(25);
		//led_off(PORT_WEST, COLOR_R);
		
				
		led_blink();
		#if configUSE_TRACE
			vTraceInstanceFinishedNow();
		#endif
		vTaskDelay((portTickType) (LED_BLINK_MIN_PERIOD / portTICK_RATE_MS) * portTICK_RATE_MS_LESS_1_MS);
		//v = v << 1;
		
		//if (v == 0b0001000000000000) v = 1;
	}
	
  //portTickType xLastWakeTime;
  //
  //xLastWakeTime = xTaskGetTickCount();
  //
  //while (true)
  //{
	  //vTaskDelayUntil(&xLastWakeTime, configTSK_BRINK_PERIOD);
	  //
	  //if (gpio_get_pin_value(AVR32_PIN_PX16))
	  //{
		  //gpio_set_pin_low(AVR32_PIN_PX16);
	  //}
	  //else
	  //{
		  //gpio_set_pin_high(AVR32_PIN_PX16);
	  //}
  //}	
}


void scan_i2c(void){
	uint8_t data_received[1];
	uint8_t a, f;
	f= 0;
	a= 1;
	twi_package_t packet_read = {
		.addr = 0, // TWI slave memory address data
		.addr_length = 1, // TWI slave memory address data size
		.chip = a, // TWI slave bus address
		.buffer = data_received, // transfer data destination buffer
		.length = 1 // transfer data size (bytes)
	};
	for (a = 1 ; a <= 0b01111111 ; a++){
		packet_read.chip = a;
		// Perform a multi-byte read access then check the result.
		if(twi_master_read(TWI0, &packet_read) == TWI_SUCCESS){
			printf("Sensor found at address %02d (%#02X)\r\n", a, a);
			f++;
		}
		
	}
	if (f == 0) printf("No sensors were found|\n\r");
}


//TODO: sens_t //if (sens_t_tmp102_setup() == 0){ //printf("Failed to configure sens_t\n\r"); //}
//t = sens_t_tmp102_read();
//printf("t=%d\n\r",t);


static void task_read_light_sensor(void *pvParameters){	
	portTickType xLastExecutionTime;
	UNUSED(pvParameters);
	vTaskDelay((portTickType)((INITIAL_DELAY/6)/portTICK_RATE_MS) * portTICK_RATE_MS_LESS_1_MS);
	
	#if configUSE_TRACE
		char name[30] = "LightSensor";
	#endif
	xLastExecutionTime = xTaskGetTickCount();
	
	char status;
	uint8_t blink = 0;	
	
	#if configUSE_TRACE
		vTraceStoreUserEventChannelName(name);
	#endif
	sens_l_setPowerUp();

	if ( sens_l_getID() == 0) {
		resources.sensor.light.configured = false;
		printf("[ERROR] [task_read_light_sensor] sens_l_getID failed!\r\n");
		printf("[INFO] [task_read_light_sensor] Kill task!\r\n");
		vTaskDelete( NULL );		
	} else {
		resources.sensor.light.configured=true;
		resources.sensor.light.value=0;
	}
	
	while(true)	{
		resources.sensor.light.value = sens_l_getData();
			if (blink == 19){
				if (g_is_led_debugging_min)
				leds[PORT_WEST].b_blink_duration = 1 * LED_BLINK_MIN_PERIOD;
				blink = 0;
			}
			blink++;
		#if configUSE_TRACE 
			vTracePrintF(name , "Read value = %d", resources.sensor.light.value);
			vTraceInstanceFinishedNow();
			//printf("[INFO] [task_read_light_sensor] Read value = %d\r\n", resources.sensor.light.value);
		#endif
		vTaskDelayUntil( &xLastExecutionTime, (portTickType) (PERIOD_SENSOR_READING / portTICK_RATE_MS ) * portTICK_RATE_MS_LESS_1_MS);
	}
}
static void task_read_pressure_sensor(void *pvParameters){
	portTickType xLastExecutionTime;
	UNUSED(pvParameters);
	vTaskDelay((portTickType)((INITIAL_DELAY/3)/portTICK_RATE_MS) * portTICK_RATE_MS_LESS_1_MS);

	char status;
	uint8_t blink = 0;
	
	status = SFE_BMP180_begin();
	if ( status == 0) {
		resources.sensor.pressure.configured = false;
		printf("[ERROR] [task_read_pressure_sensor] SFE_BMP180_begin failed! (Error Code: %d)\r\n", status);
		printf("[INFO] [task_read_pressure_sensor] Kill task!\r\n");
		vTaskDelete( NULL );
	} else {
		resources.sensor.pressure.configured = true;
		resources.sensor.temperature.configured = true;
		resources.sensor.pressure.value = 0;
		resources.sensor.temperature.value = 0;
	}
	
	xLastExecutionTime = xTaskGetTickCount();
	
	while(true)	{
		status = SFE_BMP180_startTemperature();
		if (status != 0)
		{
			vTaskDelay(status);
			status = SFE_BMP180_getTemperature(&(resources.sensor.temperature.value));
			//printf("T: %d\r\n", (int) resources.sensor.temperature.value);
			if (status != 0)
			{
				status = SFE_BMP180_startPressure(3);
				if (status != 0)
				{
					// Wait for the measurement to complete:
					vTaskDelay(status);
					status = SFE_BMP180_getPressure(&(resources.sensor.pressure.value),&(resources.sensor.temperature.value));
					if(status != 0){
						if (blink == 19){
							if (g_is_led_debugging_min)
							leds[PORT_SOUTH].b_blink_duration = 1 * LED_BLINK_MIN_PERIOD;
							blink = 0;
						}
						blink++;
						//printf("P: %d T: %d\r\n", (int) resources.sensor.pressure.value, (int) resources.sensor.temperature.value);
					} else printf("[ERROR] [task_read_pressure_sensor] SFE_BMP180_getPressure failed! (Error Code: %d)\r\n", status);
				} else printf("[ERROR] [task_read_pressure_sensor] SFE_BMP180_startPressure failed! (Error Code: %d)\r\n", status);
			} else printf("[ERROR] [task_read_pressure_sensor] SFE_BMP180_getTemperature failed! (Error Code: %d)\r\n", status);
		} else printf("[ERROR] [task_read_pressure_sensor] SFE_BMP180_startTemperature failed! (Error Code: %d)\r\n", status);
		
		
		vTaskDelayUntil( &xLastExecutionTime, (portTickType) (PERIOD_SENSOR_READING / portTICK_RATE_MS) * portTICK_RATE_MS_LESS_1_MS);
	}
}
static void task_read_accelerometer_sensor(void *pvParameters){
	portTickType xLastExecutionTime;
	UNUSED(pvParameters);
	vTaskDelay((portTickType) (INITIAL_DELAY/portTICK_RATE_MS) * portTICK_RATE_MS_LESS_1_MS);

	char status;
	uint8_t blink = 0;
	
	MMA8452Q_init(SCALE_2G, ODR_800);
	vTaskDelay(50);
	status = MMA8452Q_available();
	if ( status == 0){
		resources.sensor.acceleration.configured = false;
		printf("[ERROR] [task_read_accelerometer_sensor] MMA8452Q_available failed!\r\n");
		printf("[INFO] [task_read_accelerometer_sensor] Kill task!\r\n");
		vTaskDelete( NULL );
	} else {
		resources.sensor.acceleration.configured = true;
		resources.sensor.acceleration.cx = 0;
		resources.sensor.acceleration.cy = 0;
		resources.sensor.acceleration.cz = 0;
		
		resources.sensor.acceleration.x = 0;
		resources.sensor.acceleration.y = 0;
		resources.sensor.acceleration.z = 0;
	}
	
	xLastExecutionTime = xTaskGetTickCount();
	
	while(true)	{
		
		MMA8452Q_read();
		vTaskDelay(50);
		
		resources.sensor.acceleration.cx = MMA8452Q_cx;
		resources.sensor.acceleration.cy = MMA8452Q_cy;
		resources.sensor.acceleration.cz = MMA8452Q_cz;
		
		resources.sensor.acceleration.x = MMA8452Q_x;
		resources.sensor.acceleration.y = MMA8452Q_y;
		resources.sensor.acceleration.z = MMA8452Q_z;
		
		/*
		uint8_t pl = MMA8452Q_readPL();
		switch (pl)
		{
			case PORTRAIT_U:
			printf("Portrait Up\r\n");
			break;
			case PORTRAIT_D:
			printf("Portrait Down\r\n");
			break;
			case LANDSCAPE_R:
			printf("Landscape Right\r\n");
			break;
			case LANDSCAPE_L:
			printf("Landscape Left\r\n");
			break;
			case LOCKOUT:
			printf("Flat\r\n");
			break;
		}*/
			
		if (blink == 19){
			if (g_is_led_debugging_min)
			leds[PORT_EAST].b_blink_duration = 1 * LED_BLINK_MIN_PERIOD;
			blink = 0;
		}
		blink++;
		
		vTaskDelayUntil( &xLastExecutionTime,(portTickType) (PERIOD_SENSOR_READING / portTICK_RATE_MS) * portTICK_RATE_MS_LESS_1_MS );
	}
}
static void task_send_data(void *pvParameters)
{
	UNUSED(pvParameters);
	
	vTaskDelay((portTickType) (INITIAL_DELAY/portTICK_RATE_MS) * portTICK_RATE_MS_LESS_1_MS);
	
	portTickType xLastExecutionTime;
	
	xLastExecutionTime = xTaskGetTickCount();
	
	while(true)
	{
		

		if (g_va_periodically == true && g_is_sink == false)
			//application_va(PORT_ALL_MASK);
			
		if (g_nd_periodically == true && g_is_sink == true)
			//application_nd(PORT_ALL_MASK);
			
		if (g_ea_periodically == true && g_is_sink == false)
		{
			//application_ea(PROTOCOL_EA_RAW, 0, 0);
		}
		
		if(g_is_sink)
		{
			if (g_is_led_debugging_min) leds[PORT_WEST].r_blink_duration = 1 * LED_BLINK_MIN_PERIOD;
		}
		
			
		vTaskDelayUntil( &xLastExecutionTime, (portTickType) (g_va_period / portTICK_RATE_MS) * portTICK_RATE_MS_LESS_1_MS );
		
	}

}
static void comm_test(void *pvParameters)
{
	UNUSED(pvParameters);
	
	uint16_t i = 0;
	
	portTickType xLastExecutionTime;
	
	xLastExecutionTime = xTaskGetTickCount();	
	
	while (true)
	{
		if (g_comm_unit_test == true)
		{
			
			for (i = 0 ; i < 1000 ; i++)
			{
				
				//application_ea(PROTOCOL_EA_STEP, 0, 0);
				printf("i=%d\n\r", i);
				vTaskDelayUntil( &xLastExecutionTime, (portTickType) (100 / portTICK_RATE_MS) * portTICK_RATE_MS_LESS_1_MS );
				if (g_comm_unit_test == false) break;
			}
			
			g_comm_unit_test = false;
		}
		
		vTaskDelayUntil( &xLastExecutionTime, (portTickType) (1000 /portTICK_RATE_MS) * portTICK_RATE_MS_LESS_1_MS );
	}
	
}
void task_receive_data(void *pvParameters)
{
	uint8_t port = (uint8_t) pvParameters;
	
	#if configUSE_TRACE
		char name[30] = "task_receive_data[X]\0";
		name[18] = '0' + port;
		vTraceStoreUserEventChannelName(name);
	#endif
	
	if (port == PORT_NORTH_MASK)
		vTaskDelay((portTickType) ((INITIAL_DELAY / 4) / portTICK_RATE_MS) * portTICK_RATE_MS_LESS_1_MS);
	else if (port == PORT_SOUTH_MASK) 
		vTaskDelay((portTickType)((INITIAL_DELAY / 2) / portTICK_RATE_MS) * portTICK_RATE_MS_LESS_1_MS);
	else if (port == PORT_EAST_MASK)
		vTaskDelay((portTickType)((INITIAL_DELAY / 1.5) / portTICK_RATE_MS) * portTICK_RATE_MS_LESS_1_MS);
	else if (port == PORT_WEST_MASK) 
		vTaskDelay((portTickType) ((INITIAL_DELAY ) / portTICK_RATE_MS) * portTICK_RATE_MS_LESS_1_MS);
	
	while (true)
	{
		switch_read_packet(port);
		
		#if configUSE_TRACE
			vTraceInstanceFinishedNow();
		#endif
	}
}
void task_send_packets(void *pvParameters)
{
	uint8_t port = (uint8_t) pvParameters;
	
	#if configUSE_TRACE
		char name[30] = "task_receive_data[X]\0";
		name[18] = '0' + port;
		vTraceStoreUserEventChannelName(name);
	#endif
	
	if (port == PORT_NORTH_MASK)
		vTaskDelay((portTickType)((INITIAL_DELAY / 4) / portTICK_RATE_MS) * portTICK_RATE_MS_LESS_1_MS);
	else if (port == PORT_SOUTH_MASK)
		vTaskDelay((portTickType)((INITIAL_DELAY / 2) / portTICK_RATE_MS) * portTICK_RATE_MS_LESS_1_MS);
	else if (port == PORT_EAST_MASK)
		vTaskDelay((portTickType)((INITIAL_DELAY / 1.5) / portTICK_RATE_MS) * portTICK_RATE_MS_LESS_1_MS);
	else if (port == PORT_WEST_MASK)
		vTaskDelay((portTickType)(INITIAL_DELAY  / portTICK_RATE_MS) * portTICK_RATE_MS_LESS_1_MS);
	
	while (true)
	{
		switch_send_packet(port);
		
		#if configUSE_TRACE
		vTraceInstanceFinishedNow();
		#endif
	}
}


void task_process_packets_received(void *pvParameters)
{
	UNUSED(pvParameters);
	while (true)
	{
		application_process_packet_received();
	}
}

void task_observe_data (void *pvParameters)
{
	UNUSED(pvParameters);
	while (true)
	{
		if(application_packet_observe_cycle_time){
			application_process_observe();
			vTaskDelay(application_packet_observe_cycle_time);
		} else {
			vTaskDelay((portTickType)( 500 /portTICK_RATE_MS) * portTICK_RATE_MS_LESS_1_MS);
		}
			
	}
}

void task_periodically (void *pvParameters)
{
	UNUSED(pvParameters);
	while (true)
	{
		if(g_config.g_periodically_configured && g_config.g_periodically_cycle_time){
			//application_process_observe();
			
			application_process_periodically();
			vTaskDelay(g_config.g_periodically_cycle_time);
		} else {
			vTaskDelay((portTickType)(500 /portTICK_RATE_MS) * portTICK_RATE_MS_LESS_1_MS);
		}
		
	}
}



static void configure_console(void)
{
	const usart_serial_options_t usart_serial_options = {
		.baudrate     = CONSOLE_USART_BAUDRATE,
		.charlength   = CONSOLE_USART_CHAR_LENGTH,
		.paritytype   = CONSOLE_USART_PARITY,
		.stopbits     = CONSOLE_USART_STOP_BIT
	};

	/* Configure console UART. */
	stdio_serial_init(CONSOLE_USART, &usart_serial_options);

	/* Specify that stdout should not be buffered. */
	setbuf(stdout, NULL);
	
	create_usart_cli_task(CONSOLE_USART, mainUART_CLI_TASK_STACK_SIZE, mainUART_CLI_TASK_PRIORITY);

}



void vApplicationMallocFailedHook(void)
{
	/* vApplicationMallocFailedHook() will only be called if
	configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h.  It is a hook
	function that will get called if a call to pvPortMalloc() fails.
	pvPortMalloc() is called internally by the kernel whenever a task, queue,
	timer or semaphore is created.  It is also called by various parts of the
	demo application.  If heap_1.c or heap_2.c are used, then the size of the
	heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
	FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
	to query the size of free heap space that remains (although it does not
	provide information on how the remaining heap might be fragmented). */
	taskDISABLE_INTERRUPTS();
	printf("Application malloc failed!\n\r");
	for (;;) {
	}
}

void vApplicationIdleHook(void)
{
	/* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
	to 1 in FreeRTOSConfig.h.  It will be called on each iteration of the idle
	task.  It is essential that code added to this hook function never attempts
	to block in any way (for example, call xQueueReceive() with a block time
	specified, or call vTaskDelay()).  If the application makes use of the
	vTaskDelete() API function (as this demo application does) then it is also
	important that vApplicationIdleHook() is permitted to return to its calling
	function, because it is the responsibility of the idle task to clean up
	memory allocated by the kernel to any task that has since been deleted. */
}

void vApplicationStackOverflowHook(xTaskHandle pxTask, signed char *pcTaskName) {
	(void) pcTaskName;
	(void) pxTask;

	/* Run time stack overflow checking is performed if
	configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
	function is called if a stack overflow is detected. */
	taskDISABLE_INTERRUPTS();
	printf("\n\nStack Overflow! %s\n\r\n", pcTaskName);
	for (;;) {
	}
}

void vApplicationTickHook(void)
{
	/* This function will be called by each tick interrupt if
	configUSE_TICK_HOOK is set to 1 in FreeRTOSConfig.h.  User code can be
	added here, but the tick hook is called from an interrupt context, so
	code must not attempt to block, and only the interrupt safe FreeRTOS API
	functions can be used (those that end in FromISR()). */
}



void global_vars_init(void)
{
	g_is_debugging = false;
	g_is_info_debugging = false;
	g_is_led_debugging = false;
	g_is_led_debugging_min = true;
	g_is_sink = false;
	g_va_periodically = false;
	g_nd_periodically = false;
	g_ea_periodically = false;
	g_va_period = PERIOD_EA;
	
	g_comm_unit_test = false;
	
	g_nodes_config = CONFIG_VA_PERIODICALLY_MASK | CONFIG_LED_DEBUGGING_MASK;
	g_nodes_config_sampling_period = g_va_period;
	
	
	g_config.g_periodically_configured = 0;
	g_config.g_periodically_cycle_time = 0;
	g_config.g_periodically_sensor_type = 0;
	g_config.g_periodically_app_neighbor_lookup_type = 0;
	g_config.g_periodically_app_neighbor_lookup_radius = 0;
	
/*	
	g_sensors.l_configured = false;
	g_sensors.p_configured = false;
	g_sensors.t_configured = false;
	g_sensors.a_configured = false;
*/
}

void force_mutiple_pingm(void *pvParameters){
	UNUSED(pvParameters);
	while(1){
		application_ping(PROTOCOL_MULTICAST_1, 0 , 0);
		vTaskDelay((portTickType)( 1000/portTICK_RATE_MS) * portTICK_RATE_MS_LESS_1_MS);
	}
}
 
void simulate_polyfit_regression(int size, int type){
	char output[50];
	int i, j;
	
	float * x = (float *) pvPortMalloc(sizeof(float) * size * size);
	float * y = (float *) pvPortMalloc(sizeof(float) * size * size);
	float * v = (float *) pvPortMalloc(sizeof(float) * size * size);
	
	float coeff[6] = {0};
	float ms_error = 0;
	

	for (i = 0; i< size; i++){
		for (j = 0; j< size; j++){
			x[j + (size * i)] = j + 1;
			y[j + (size * i)] = i + 1;
		}
	}
		
	for(i=0; i< (size * size); i++){
		v[i] = rand()%40 + 10;
	}
	
	#ifdef PORT_NORTH_TXIO_DEBUG_MODE
		ioport_set_pin_level(PIO_PB11_IDX, IOPORT_PIN_LEVEL_HIGH);
	#endif
	
	if(type==0){
		polyfit2indepentvars_vlm(x, y, v, size * size, coeff, &ms_error);
	} else if (type==1){
		//polyfit2indepentvars_lm(x, y, v, size * size, coeff, &ms_error);
	}
	
	#ifdef PORT_NORTH_TXIO_DEBUG_MODE
		ioport_set_pin_level(PIO_PB11_IDX, IOPORT_PIN_LEVEL_LOW);
	#endif
	
	//print equation
	printf("\n\rf(x,y)= ");
	
	sprintf(output, "%c %f ",		coeff[0] > 0 ? '+' : ' ', coeff[0]);	printf(output);	
	sprintf(output, "%c %f * x ",	coeff[1] > 0 ? '+' : ' ', coeff[1]);	printf(output);	
	sprintf(output, "%c %f * y ",	coeff[2] > 0 ? '+' : ' ', coeff[2]);	printf(output);	
	sprintf(output, "%c %f * x^2",	coeff[3] > 0 ? '+' : ' ', coeff[3]);	printf(output);	
	sprintf(output, "%c %f * y^2",	coeff[4] > 0 ? '+' : ' ', coeff[4]);	printf(output);	
	sprintf(output, "%c %f * x.y",	coeff[5] > 0 ? '+' : ' ', coeff[5]);	printf(output);
	printf("\n\r");
	
	sprintf(output,"Mean Square Error = %f", ms_error);  printf(output);
	printf("\n\r");
	
	vPortFree(x);
	vPortFree(y);
	vPortFree(v);
	
	return;
}

int main (void)
{
	// Insert system clock initialization code here (sysclk_init()).
	sysclk_init();
	board_init();
	configure_console();
	application_initialize();
	global_vars_init();
	
	led_set_all(0xFFFF); //turn all on
	led_set_all(0x0000); //All off
	switch_initialize();
	
	void* objectHandle = NULL;
	
	#if configUSE_TRACE 
		//SEGGER_SYSVIEW_Conf();
		//SEGGER_RTT_WriteString(0, "Hello World from SEGGER!\r\n");
		Trace_Init();
		//vTraceInitTraceData();
		//uiTraceStart();
		vTraceStoreUserEventChannelName("Messages");
		vTracePrint("Messages", "XDense starting...");
		//vTracePrint("Messages", "vTraceUserEvent creates basic User Events.");
		//vTracePrint("Messages", "vTracePrintF creates advanced user events, like printf");
		//vTracePrintF("Messages", "A float: %f (should be 1)", (float)1.0);
		//vTracePrintF("Messages", "A double: %lf (should be 1)", (double)1.0);
		//vTracePrintF("Messages", "A signed 8-bit value: %bd (should be -1)", -1);
	#endif
	
	
	//RX
	 	
	/* Create task that receives data */
	#ifndef PORT_NORTH_TXIO_DEBUG_MODE
		if (xTaskCreate(task_receive_data, "Rx North", TASK_RECEIVE_DATA_STACK_SIZE, (void*) PORT_NORTH, TASK_RECEIVE_DATA_STACK_PRIORITY,  &objectHandle) != pdPASS) {
			printf("Failed to create Receive Rx North task\r\n");
		}
	#endif
	/* Create task that receives data */
	if (xTaskCreate(task_receive_data, "Rx South", TASK_RECEIVE_DATA_STACK_SIZE, (void*) PORT_SOUTH, TASK_RECEIVE_DATA_STACK_PRIORITY,  &objectHandle) != pdPASS) {
		printf("Failed to create Receive Rx South task\r\n");
	}
	/* Create task that receives data */
	if (xTaskCreate(task_receive_data, "Rx East", TASK_RECEIVE_DATA_STACK_SIZE, (void*) PORT_EAST, TASK_RECEIVE_DATA_STACK_PRIORITY,  &objectHandle) != pdPASS) {
		printf("Failed to create Receive Rx East task\r\n");
	}
	/* Create task that receives data */
	if (xTaskCreate(task_receive_data, "Rx West", TASK_RECEIVE_DATA_STACK_SIZE, (void*) PORT_WEST, TASK_RECEIVE_DATA_STACK_PRIORITY,  &objectHandle) != pdPASS) {
		printf("Failed to create Receive Rx West task\r\n");
	}
	
	//TX
	
	#ifndef PORT_NORTH_TXIO_DEBUG_MODE
	if (xTaskCreate(task_send_packets, "Tx North", TASK_SEND_DATA_STACK_SIZE, (void*) PORT_NORTH, TASK_SEND_DATA_STACK_PRIORITY,  &objectHandle) != pdPASS) {
		printf("Failed to create Receive Tx North task\r\n");
	}
	#endif
	/* Create task that receives data */
	if (xTaskCreate(task_send_packets, "Tx South", TASK_SEND_DATA_STACK_SIZE, (void*) PORT_SOUTH, TASK_SEND_DATA_STACK_PRIORITY,  &objectHandle) != pdPASS) {
		printf("Failed to create Receive Tx South task\r\n");
	}
	/* Create task that receives data */
	if (xTaskCreate(task_send_packets, "Tx East", TASK_SEND_DATA_STACK_SIZE, (void*) PORT_EAST, TASK_SEND_DATA_STACK_PRIORITY,  &objectHandle) != pdPASS) {
		printf("Failed to create Receive Tx East task\r\n");
	}
	/* Create task that receives data */
	if (xTaskCreate(task_send_packets, "Tx West", TASK_SEND_DATA_STACK_SIZE, (void*) PORT_WEST, TASK_SEND_DATA_STACK_PRIORITY,  &objectHandle) != pdPASS) {
		printf("Failed to create Receive Tx West task\r\n");
	}
	
	
	
	
	/* Create task to send sensors data */
	//if (xTaskCreate(task_receive_data_uart3, "Data Rx U3", TASKS_STACK_SIZE, NULL, TASKS_STACK_PRIORITY, NULL) != pdPASS) {
		//printf("Failed to create Read U3 task\r\n");
	//}
	/* Create task to send sensors data */
	/*if (xTaskCreate(task_send_data, "Data Tx", TASKS_STACK_SIZE, NULL, TASKS_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create Send Data task\r\n");
	}*/
	
	
	/*  Task to process OBSERVE packets */
	//if (xTaskCreate(task_observe_data, "OBSERVE", TASKS_STACK_SIZE, NULL, TASKS_STACK_PRIORITY, NULL) != pdPASS) {
		//printf("Failed to create OBSERVE Data task\r\n");
	//}
	//
	/* Task to process packets received */
	if (xTaskCreate(task_process_packets_received, "Packet Process", TASK_RECEIVE_DATA_STACK_SIZE, NULL, TASKS_DATA_PROC_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create Packet Process task\r\n");
	}
	
	///*  Task to process periodically tasks */
	//if (xTaskCreate(task_periodically, "Periodically", TASKS_STACK_SIZE, NULL, TASKS_STACK_PRIORITY, NULL) != pdPASS) {
		//printf("Failed to create OBSERVE Data task\r\n");
	//}
	//
	/*Create task to make led blink */
	if (xTaskCreate(led_task_blink, "Led", TASK_LED_STACK_SIZE, NULL, TASKS_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create test led task\r\n");
	}
	/*
	if (xTaskCreate(task_terminal, "Terminal", TASKS_STACK_SIZE, NULL, TASKS_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create terminal task\r\n");
	}*/
	
	/*if (xTaskCreate(comm_test, "Comm test", TASKS_STACK_SIZE, NULL, TASKS_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create Comm test task\r\n");
	}*/
	
	
	/* Create task to read the sensors */
	//if (twi_initialize(TWI0) == 0){
		//printf("Failed to configure I2C0\n\r");
	//} else {
		//
		//if (xTaskCreate(task_read_pressure_sensor, "SensorP", TASKS_STACK_SIZE, NULL, TASKS_SENSOR_PRIORITY, NULL) != pdPASS) {
			//printf("Failed to create Read Pressure Sensor task\r\n");
		//}
		//if (xTaskCreate(task_read_light_sensor, "SensorL", TASKS_STACK_SIZE, NULL, TASKS_SENSOR_PRIORITY, NULL) != pdPASS) {
			//printf("Failed to create Read Light Sensor task\r\n");
		//} 
		//if (xTaskCreate(task_read_accelerometer_sensor, "SensorAcc", TASKS_STACK_SIZE, NULL, TASKS_SENSOR_PRIORITY, NULL) != pdPASS) {
			//printf("Failed to create Read Accelerometer Sensor task\r\n");
		//}
	//}
	
	
	
	/* Create task that process received data */
	//if (xTaskCreate(task_process_data, "Process Data", TASK_LED_STACK_SIZE, NULL, TASK_LED_STACK_PRIORITY, NULL) != pdPASS) {
		//printf("Failed to create Process Data task\r\n");
	//}
	
	//DEBUG task
	//if (xTaskCreate(force_mutiple_pingm, "force pingm", TASKS_STACK_SIZE, NULL, TASKS_STACK_PRIORITY, NULL) != pdPASS) {
	//	printf("Failed to create test led force_mutiple_pingm\r\n");
	//}
	//////////////////////////////////////////////////////////////////////////

	/* Start the scheduler. */
	vTaskStartScheduler();
	
	
}

