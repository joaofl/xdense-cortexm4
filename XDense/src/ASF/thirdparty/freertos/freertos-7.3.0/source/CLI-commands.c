/**
 *
 * \file
 *
 * \brief FreeRTOS+CLI command examples
 *
 *
 * Copyright (c) 2012 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel microcontroller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 */

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"

/* Standard includes. */
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/* FreeRTOS+CLI includes. */
#include "FreeRTOS_CLI.h"

#include "tasks.h"
#include "comm_stack/packet.h"
#include "application.h"
#include "main.h"

/*
 * Implements the run-time-stats command.
 */
static portBASE_TYPE task_stats_command(int8_t *pcWriteBuffer,
		size_t xWriteBufferLen,
		const int8_t *pcCommandString);

/*
 * Implements the task-stats command.
 */
static portBASE_TYPE run_time_stats_command(int8_t *pcWriteBuffer,
		size_t xWriteBufferLen,
		const int8_t *pcCommandString);

/*
 * Implements the echo-three-parameters command.
 */
static portBASE_TYPE three_parameter_echo_command(int8_t *pcWriteBuffer,
		size_t xWriteBufferLen,
		const int8_t *pcCommandString);

/*
 * Implements the echo-parameters command.
 */
static portBASE_TYPE multi_parameter_echo_command(int8_t *pcWriteBuffer,
		size_t xWriteBufferLen,
		const int8_t *pcCommandString);

/*
 * Implements the create-task command.
 */
static portBASE_TYPE create_task_command(int8_t *pcWriteBuffer,
		size_t xWriteBufferLen,
		const int8_t *pcCommandString);

/*
 * Implements the delete-task command.
 */
static portBASE_TYPE delete_task_command(int8_t *pcWriteBuffer,
		size_t xWriteBufferLen,
		const int8_t *pcCommandString);

/*
 * Implements the ping-task command.
 */
static portBASE_TYPE ping_command(int8_t *pcWriteBuffer,
		size_t xWriteBufferLen,
		const int8_t *pcCommandString);
		
/*
 * Implements the led-task command.
 */
static portBASE_TYPE led_command(int8_t *pcWriteBuffer,
		size_t xWriteBufferLen,
		const int8_t *pcCommandString);
		
/*
 * Implements the led-task command.
 */
static portBASE_TYPE value_command(int8_t *pcWriteBuffer,
		size_t xWriteBufferLen,
		const int8_t *pcCommandString);
		
/*
 * Implements the s_regression-task command.
 */
static portBASE_TYPE s_regression_command(int8_t *pcWriteBuffer,
		size_t xWriteBufferLen,
		const int8_t *pcCommandString);
		
/*
 * Implements the config_p_set-task command.
 */
static portBASE_TYPE config_p_set_command(int8_t *pcWriteBuffer,
		size_t xWriteBufferLen,
		const int8_t *pcCommandString);
		
/*
 * Implements the regression-task command.
 */
static portBASE_TYPE regression_command(int8_t *pcWriteBuffer,
		size_t xWriteBufferLen,
		const int8_t *pcCommandString);

/*
 * The task that is created by the create-task command.
 */
void created_task(void *pvParameters);

/*
 * Holds the handle of the task created by the create-task command.
 */
static xTaskHandle created_task_handle = NULL;

/* Structure that defines the "run-time-stats" command line command.
This generates a table that shows how much run time each task has */
static const CLI_Command_Definition_t run_time_stats_command_definition =
{
	(const int8_t *const) "run-time-stats", /* The command string to type. */
	(const int8_t *const) "run-time-stats:\r\n Displays a table showing how much processing time each FreeRTOS task has used\r\n\r\n",
	run_time_stats_command, /* The function to run. */
	0 /* No parameters are expected. */
};

/* Structure that defines the "task-stats" command line command.  This generates
a table that gives information on each task in the system. */
static const CLI_Command_Definition_t task_stats_command_definition =
{
	(const int8_t *const) "task-stats", /* The command string to type. */
	(const int8_t *const) "task-stats:\r\n Displays a table showing the state of each FreeRTOS task\r\n\r\n",
	task_stats_command, /* The function to run. */
	0 /* No parameters are expected. */
};

/* Structure that defines the "echo_3_parameters" command line command.  This
takes exactly three parameters that the command simply echos back one at a
time. */
static const CLI_Command_Definition_t three_parameter_echo_command_definition =
{
	(const int8_t *const) "echo-3-parameters",
	(const int8_t *const) "echo-3-parameters <param1> <param2> <param3>:\r\n Expects three parameters, echos each in turn\r\n\r\n",
	three_parameter_echo_command, /* The function to run. */
	3 /* Three parameters are expected, which can take any value. */
};

/* Structure that defines the "echo_parameters" command line command.  This
takes a variable number of parameters that the command simply echos back one at
a time. */
static const CLI_Command_Definition_t multi_parameter_echo_command_definition =
{
	(const int8_t *const) "echo-parameters",
	(const int8_t *const) "echo-parameters <...>:\r\n Take variable number of parameters, echos each in turn\r\n\r\n",
	multi_parameter_echo_command, /* The function to run. */
	-1 /* The user can enter any number of commands. */
};

/* Structure that defines the "create-task" command line command.  This takes a
single parameter that is passed into a newly created task.  The task then
periodically writes to the console.  The parameter must be a numerical value. */
static const CLI_Command_Definition_t create_task_command_definition =
{
	(const int8_t *const) "create-task",
	(const int8_t *const) "create-task <param>:\r\n Creates a new task that periodically writes the parameter to the CLI output\r\n\r\n",
	create_task_command, /* The function to run. */
	1 /* A single parameter should be entered. */
};

/* Structure that defines the "delete-task" command line command.  This deletes
the task that was previously created using the "create-command" command. */
static const CLI_Command_Definition_t delete_task_command_definition =
{
	(const int8_t *const) "delete-task",
	(const int8_t *const) "delete-task:\r\n Deletes the task created by the create-task command\r\n\r\n",
	delete_task_command, /* The function to run. */
	0 /* A single parameter should be entered. */
};

/* Structure that defines the "ping" command line command.  This test the reachability of a node. */
static const CLI_Command_Definition_t ping_command_definition =
{
	(const int8_t *const) "ping",
	(const int8_t *const) "ping <protocol(u/m/b)> <x> <y>: \r\n Test the reachability of a node.\r\n\r\n",
	ping_command, /* The function to run. */
	3 /* Three parameters are expected, which can take any value. */
};


/* Structure that defines the "ping" command line command.  This test the reachability of a node. */
static const CLI_Command_Definition_t led_command_definition =
{
	(const int8_t *const) "led",
	(const int8_t *const) "led <position(a/n/s/e/w)> <color(r/g/b)> <state(0/1)>: \r\n Debug RGB LEDs.\r\n\r\n",
	led_command, /* The function to run. */
	3 /* Three parameters are expected, which can take any value. */
};

/* Structure that defines the "ping" command line command.  This test the reachability of a node. */
static const CLI_Command_Definition_t value_command_definition =
{
	(const int8_t *const) "value",
	(const int8_t *const) "value <type(L/A/P/T)> <x> <y>: \r\n Request sensor values.\r\n\r\n",
	value_command, /* The function to run. */
	3 /* Two parameters are expected, which can take any value. */
};

/* Structure that defines the "s_regression" command line command.  This simulate polynomial regression. */
static const CLI_Command_Definition_t s_regression_command_definition =
{
	(const int8_t *const) "s_regression",
	(const int8_t *const) "s_regression <size> <type(0=dynamic/1=static)>: \r\n Simulate polynomial regression.\r\n\r\n",
	s_regression_command, /* The function to run. */
	2 /* One parameters are expected, which can take any value. */
};

/* Structure that defines the "config_p_set" command line command.  Config the periodically task (Needed to make poly regression). */
static const CLI_Command_Definition_t config_p_set_command_definition =
{
	(const int8_t *const) "config_p_set",
	(const int8_t *const) "config_p_set <x> <y> <enable(0=No/1=Yes)> <cycle_time(ms)> <sensor_type(L/A/P/T)> <square_radius>: \r\n  Config periodically task (Needed to make polynomial regression)\r\n\r\n",
	config_p_set_command, /* The function to run. */
	6 /* One parameters are expected, which can take any value. */
};

/* Structure that defines the "s_regression" command line command.  This simulate polynomial regression. */
static const CLI_Command_Definition_t regression_command_definition =
{
	(const int8_t *const) "regression",
	(const int8_t *const) "regression <x> <y>: \r\n Polynomial regression.\r\n\r\n",
	regression_command, /* The function to run. */
	2 /* One parameters are expected, which can take any value. */
};

/*-----------------------------------------------------------*/

void vRegisterCLICommands(void)
{
	/* Register all the command line commands defined immediately above. */
	FreeRTOS_CLIRegisterCommand(&task_stats_command_definition);
	FreeRTOS_CLIRegisterCommand(&run_time_stats_command_definition);
	FreeRTOS_CLIRegisterCommand(&three_parameter_echo_command_definition);
	FreeRTOS_CLIRegisterCommand(&multi_parameter_echo_command_definition);
	FreeRTOS_CLIRegisterCommand(&create_task_command_definition);
	FreeRTOS_CLIRegisterCommand(&delete_task_command_definition);
	FreeRTOS_CLIRegisterCommand(&ping_command_definition);
	FreeRTOS_CLIRegisterCommand(&led_command_definition);
	FreeRTOS_CLIRegisterCommand(&value_command_definition);
	FreeRTOS_CLIRegisterCommand(&s_regression_command_definition);
	FreeRTOS_CLIRegisterCommand(&config_p_set_command_definition);
	FreeRTOS_CLIRegisterCommand(&regression_command_definition);
}

/*-----------------------------------------------------------*/

static portBASE_TYPE task_stats_command(int8_t *pcWriteBuffer,
		size_t xWriteBufferLen,
		const int8_t *pcCommandString)
{
	const int8_t *const task_table_header = (int8_t *) "Task          State  Priority  Stack	#\r\n************************************************\r\n";

	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	(void) pcCommandString;
	(void) xWriteBufferLen;
	configASSERT(pcWriteBuffer);

	/* Generate a table of task stats. */
	strcpy((char *) pcWriteBuffer, (char *) task_table_header);
	vTaskList(pcWriteBuffer + strlen((char *) task_table_header));

	/* There is no more data to return after this single string, so return
	pdFALSE. */
	return pdFALSE;
}

/*-----------------------------------------------------------*/

static portBASE_TYPE run_time_stats_command(int8_t *pcWriteBuffer,
		size_t xWriteBufferLen,
		const int8_t *pcCommandString)
{
	const int8_t *const stats_table_header = (int8_t *) "Task            Abs Time      % Time\r\n****************************************\r\n";

	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	(void) pcCommandString;
	(void) xWriteBufferLen;
	configASSERT(pcWriteBuffer);

	/* Generate a table of task stats. */
	//strcpy((char *) pcWriteBuffer, (char *) stats_table_header);
	//vTaskGetRunTimeStats(pcWriteBuffer + strlen(
			//(char *) stats_table_header));

	/* There is no more data to return after this single string, so return
	pdFALSE. */
	return pdFALSE;
}

/*-----------------------------------------------------------*/

static portBASE_TYPE three_parameter_echo_command(int8_t *pcWriteBuffer,
		size_t xWriteBufferLen,
		const int8_t *pcCommandString)
{
	int8_t *parameter_string;
	portBASE_TYPE parameter_string_length, return_value;
	static portBASE_TYPE parameter_number = 0;

	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	(void) pcCommandString;
	(void) xWriteBufferLen;
	configASSERT(pcWriteBuffer);

	if (parameter_number == 0) {
		/* The first time the function is called after the command has been
		entered just a header string is returned. */
		sprintf((char *) pcWriteBuffer,
				"The three parameters were:\r\n");

		/* Next time the function is called the first parameter will be echoed
		back. */
		parameter_number = 1L;

		/* There is more data to be returned as no parameters have been echoed
		back yet. */
		return_value = pdPASS;
	} else {
		/* Obtain the parameter string. */
		parameter_string = (int8_t *) FreeRTOS_CLIGetParameter
									(
										pcCommandString,		/* The command string itself. */
										parameter_number,		/* Return the next parameter. */
										&parameter_string_length	/* Store the parameter string length. */
									);

		/* Sanity check something was returned. */
		configASSERT(parameter_string);

		/* Return the parameter string. */
		memset(pcWriteBuffer, 0x00, xWriteBufferLen);
		sprintf((char *) pcWriteBuffer, "%ld: ", parameter_number);
		strncat((char *) pcWriteBuffer, (const char *) parameter_string,
				parameter_string_length);
		strncat((char *) pcWriteBuffer, "\r\n", strlen("\r\n"));

		/* If this is the last of the three parameters then there are no more
		strings to return after this one. */
		if (parameter_number == 3L) {
			/* If this is the last of the three parameters then there are no more
			strings to return after this one. */
			return_value = pdFALSE;
			parameter_number = 0L;
		} else {
			/* There are more parameters to return after this one. */
			return_value = pdTRUE;
			parameter_number++;
		}
	}

	return return_value;
}

/*-----------------------------------------------------------*/

static portBASE_TYPE multi_parameter_echo_command(int8_t *pcWriteBuffer,
		size_t xWriteBufferLen,
		const int8_t *pcCommandString)
{
	int8_t *parameter_string;
	portBASE_TYPE parameter_string_length, return_value;
	static portBASE_TYPE parameter_number = 0;

	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	(void) pcCommandString;
	(void) xWriteBufferLen;
	configASSERT(pcWriteBuffer);

	if (parameter_number == 0) {
		/* The first time the function is called after the command has been
		entered just a header string is returned. */
		sprintf((char *) pcWriteBuffer, "The parameters were:\r\n");

		/* Next time the function is called the first parameter will be echoed
		back. */
		parameter_number = 1L;

		/* There is more data to be returned as no parameters have been echoed
		back yet. */
		return_value = pdPASS;
	} else {
		/* Obtain the parameter string. */
		parameter_string = (int8_t *) FreeRTOS_CLIGetParameter
									(
										pcCommandString,		/* The command string itself. */
										parameter_number,		/* Return the next parameter. */
										&parameter_string_length	/* Store the parameter string length. */
									);

		if (parameter_string != NULL) {
			/* Return the parameter string. */
			memset(pcWriteBuffer, 0x00, xWriteBufferLen);
			sprintf((char *) pcWriteBuffer, "%ld: ", parameter_number);
			strncat((char *) pcWriteBuffer, (const char *) parameter_string, parameter_string_length);
			strncat((char *) pcWriteBuffer, "\r\n", strlen("\r\n"));

			/* There might be more parameters to return after this one. */
			return_value = pdTRUE;
			parameter_number++;
		} else {
			/* No more parameters were found.  Make sure the write buffer does
			not contain a valid string. */
			pcWriteBuffer[0] = 0x00;

			/* No more data to return. */
			return_value = pdFALSE;

			/* Start over the next time this command is executed. */
			parameter_number = 0;
		}
	}

	return return_value;
}

/*-----------------------------------------------------------*/

static portBASE_TYPE create_task_command(int8_t *pcWriteBuffer,
		size_t xWriteBufferLen,
		const int8_t *pcCommandString)
{
	int8_t *parameter_string;
	portBASE_TYPE parameter_string_length;
	static const int8_t *success_message = (int8_t *) "Task created\r\n";
	static const int8_t *failure_message = (int8_t *) "Task not created\r\n";
	static const int8_t *task_already_created_message = (int8_t *) "The task has already been created. Execute the delete-task command first.\r\n";
	int32_t parameter_value;

	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	(void) xWriteBufferLen;
	configASSERT(pcWriteBuffer);

	/* Obtain the parameter string. */
	parameter_string = (int8_t *) FreeRTOS_CLIGetParameter(
									pcCommandString,		/* The command string itself. */
									1,						/* Return the first parameter. */
									&parameter_string_length	/* Store the parameter string length. */
								);

	/* Turn the parameter into a number. */
	parameter_value = (int32_t) atol((const char *) parameter_string);

	/* Attempt to create the task. */
	if (created_task_handle != NULL) {
		strcpy((char *) pcWriteBuffer,
				(const char *) task_already_created_message);
	} else {
		if (xTaskCreate(created_task, (const signed char *) "Created",
				configMINIMAL_STACK_SIZE,
				(void *) parameter_value, tskIDLE_PRIORITY,
				&created_task_handle) == pdPASS) {
			strcpy((char *) pcWriteBuffer,
					(const char *) success_message);
		} else {
			strcpy((char *) pcWriteBuffer,
					(const char *) failure_message);
		}
	}

	/* There is no more data to return after this single string, so return
	pdFALSE. */
	return pdFALSE;
}

/*-----------------------------------------------------------*/

static portBASE_TYPE delete_task_command(int8_t *pcWriteBuffer,
		size_t xWriteBufferLen,
		const int8_t *pcCommandString)
{
	static const int8_t *success_message = (int8_t *) "Task deleted\r\n";
	static const int8_t *failure_message = (int8_t *) "The task was not running.  Execute the create-task command first.\r\n";

	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	(void) pcCommandString;
	(void) xWriteBufferLen;
	configASSERT(pcWriteBuffer);

	/* See if the task is running. */
	if (created_task_handle != NULL) {
		vTaskDelete(created_task_handle);
		created_task_handle = NULL;
		strcpy((char *) pcWriteBuffer, (const char *) success_message);
	} else {
		strcpy((char *) pcWriteBuffer, (const char *) failure_message);
	}

	/* There is no more data to return after this single string, so return
	 * pdFALSE. */
	return pdFALSE;
}


/*-----------------------------------------------------------*/

void created_task(void *pvParameters)
{
	int32_t parameter_value;
	static uint8_t local_buffer[60];

	/* Cast the parameter to an appropriate type. */
	parameter_value = (int32_t)pvParameters;

	memset((void *) local_buffer, 0x00, sizeof(local_buffer));
	sprintf((char *) local_buffer,
			"Created task running.  Received parameter %ld\r\n\r\n",
			(long) parameter_value);

	/* Cannot yet tell which CLI interface is in use, but both output functions
	guard check the port is initialised before it is used. */
#if (defined confINCLUDE_USART_CLI)
	usart_cli_output(local_buffer);
#endif

#if (defined confINCLUDE_CDC_CLI)
	cdc_cli_output(local_buffer);
#endif

	for (;;) {
		vTaskDelay(portMAX_DELAY);
	}
}

/*-----------------------------------------------------------*/

static portBASE_TYPE ping_command(int8_t *pcWriteBuffer,
		size_t xWriteBufferLen,
		const int8_t *pcCommandString)
{
	int8_t *parameter_string;
	
	uint8_t protocol;
	int8_t d_x;
	int8_t d_y;
	
	portBASE_TYPE parameter_string_length_d_x, parameter_string_length_d_y, parameter_string_length_protocol, return_value;
	static portBASE_TYPE parameter_number = 0;

	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	(void) pcCommandString;
	(void) xWriteBufferLen;
	configASSERT(pcWriteBuffer);
	
	parameter_string = (int8_t *) FreeRTOS_CLIGetParameter
	(
		pcCommandString,				/* The command string itself. */
		1,								/* Return the x parameter. */
		&parameter_string_length_protocol	/* Store the parameter string length. */
	);
	
	switch(parameter_string[0]){
		case 'u':
			protocol = PROTOCOL_UNICAST;
		break;
		case 'm':
			protocol = PROTOCOL_MULTICAST_1;
		break;
		case 'b':
			protocol = PROTOCOL_BROADCAST;
		break;
		default:
			protocol = PROTOCOL_BROADCAST;
		break;
	}
	
		
	parameter_string = (int8_t *) FreeRTOS_CLIGetParameter
	(
		pcCommandString,				/* The command string itself. */
		2,								/* Return the x parameter. */
		&parameter_string_length_d_x	/* Store the parameter string length. */
	);
	
	d_x = (int8_t) atol((const char *) parameter_string);
	
	
	parameter_string = (int8_t *) FreeRTOS_CLIGetParameter
	(
		pcCommandString,				/* The command string itself. */
		3,								/* Return the x parameter. */
		&parameter_string_length_d_y	/* Store the parameter string length. */
	);
	d_y = (int8_t) atol((const char *) parameter_string);

	
	
	//sprintf((char *) pcWriteBuffer,	"Ping sent to d_x: %d d_y: %d node with %d bytes of data:\r\n",   d_x , d_y, PACKET_SIZE);
	//strncat((char *) pcWriteBuffer, "\r\n", strlen("\r\n"));
	
	
	application_ping(protocol, d_x , d_y);
	
	return_value = pdFALSE;

	return return_value;
}

static portBASE_TYPE led_command(int8_t *pcWriteBuffer,
		size_t xWriteBufferLen,
		const int8_t *pcCommandString)
{
	int8_t *parameter_string;
	char pos;
	char color;
	int8_t state; 
	portBASE_TYPE parameter_string_length_pos, parameter_string_length_color, parameter_string_length_state, return_value;
	static portBASE_TYPE parameter_number = 0;

	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	(void) pcCommandString;
	(void) xWriteBufferLen;
	configASSERT(pcWriteBuffer);
	
		
	parameter_string = (int8_t *) FreeRTOS_CLIGetParameter
	(
		pcCommandString,				/* The command string itself. */
		1,								/* Return the x parameter. */
		&parameter_string_length_pos	/* Store the parameter string length. */
	);
	
	pos = parameter_string[0];
	
	parameter_string = (int8_t *) FreeRTOS_CLIGetParameter
	(
		pcCommandString,				/* The command string itself. */
		2,								/* Return the x parameter. */
		&parameter_string_length_color	/* Store the parameter string length. */
	);	
	
	switch(parameter_string[0]){
		case 'r':
			color = COLOR_R;
		break;
		case 'g':
			color = COLOR_G;
		break;
		case 'b':
			color = COLOR_B;
		break;
		
	}	
	
	parameter_string = (int8_t *) FreeRTOS_CLIGetParameter
	(
		pcCommandString,				/* The command string itself. */
		3,								/* Return the x parameter. */
		&parameter_string_length_state	/* Store the parameter string length. */
	);
	
	state = (int8_t) atol((const char *) parameter_string);
		
	
	if( state == 1 ){
		switch(pos){
			case 'n':
				led_on(PORT_NORTH, color);
			break;
			case 's':
				led_on(PORT_SOUTH, color);
			break;
			case 'e':
				led_on(PORT_EAST, color);
			break;
			case 'w':
				led_on(PORT_WEST, color);
			break;
			case 'a':
				led_on(PORT_NORTH, color);
				led_on(PORT_SOUTH, color);
				led_on(PORT_EAST, color);
				led_on(PORT_WEST, color);
			break;
			
		}
		sprintf((char *) pcWriteBuffer,	"LED [%c] color [%d] set on!\r\n", pos, color);
	} else {
		switch(pos){
			case 'n':
			led_off(PORT_NORTH, color);
			break;
			case 's':
			led_off(PORT_SOUTH, color);
			break;
			case 'e':
			led_off(PORT_EAST, color);
			break;
			case 'w':
			led_off(PORT_WEST, color);
			break;
			case 'a':
				led_off(PORT_NORTH, color);
				led_off(PORT_SOUTH, color);
				led_off(PORT_EAST, color);
				led_off(PORT_WEST, color);
			break;
		}
		sprintf((char *) pcWriteBuffer,	"LED [%c] color [%d] set off!\r\n", pos, color);
	}

	strncat((char *) pcWriteBuffer, "\r\n", strlen("\r\n"));
	
	return_value = pdFALSE;

	return return_value;
}

static portBASE_TYPE value_command(int8_t *pcWriteBuffer,
		size_t xWriteBufferLen,
		const int8_t *pcCommandString)
{
	int8_t *parameter_string;
	char type;
	int8_t x; 
	int8_t y; 
	portBASE_TYPE parameter_string_length_type, parameter_string_length_x, parameter_string_length_y, return_value;
	static portBASE_TYPE parameter_number = 0;

	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	(void) pcCommandString;
	(void) xWriteBufferLen;
	configASSERT(pcWriteBuffer);
	
		
	parameter_string = (int8_t *) FreeRTOS_CLIGetParameter
	(
		pcCommandString,				/* The command string itself. */
		1,								/* Return the x parameter. */
		&parameter_string_length_type	/* Store the parameter string length. */
	);
	
	type = parameter_string[0];
	
	parameter_string = (int8_t *) FreeRTOS_CLIGetParameter
	(
		pcCommandString,				/* The command string itself. */
		2,								/* Return the x parameter. */
		&parameter_string_length_x	/* Store the parameter string length. */
	);	
	
	x = (int8_t) atol((const char *) parameter_string);
	
	parameter_string = (int8_t *) FreeRTOS_CLIGetParameter
	(
		pcCommandString,				/* The command string itself. */
		3,								/* Return the x parameter. */
		&parameter_string_length_y	/* Store the parameter string length. */
	);
	
	y = (int8_t) atol((const char *) parameter_string);
		
	
	application_va(PROTOCOL_UNICAST, x, y, type);
	
	sprintf((char *) pcWriteBuffer,	"Value [type=%c] [d_x=%d d_y=%d] requested!\r\n", type, x, y);
	strncat((char *) pcWriteBuffer, "\r\n", strlen("\r\n"));
	
	
	
	return_value = pdFALSE;

	return return_value;
}


static portBASE_TYPE s_regression_command(int8_t *pcWriteBuffer,
		size_t xWriteBufferLen,
		const int8_t *pcCommandString)
{
	int8_t *parameter_string;
	int8_t size, type; 
	portBASE_TYPE parameter_string_length_size, parameter_string_length_type, return_value;
	static portBASE_TYPE parameter_number = 0;

	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	(void) pcCommandString;
	(void) xWriteBufferLen;
	configASSERT(pcWriteBuffer);
	
		
	parameter_string = (int8_t *) FreeRTOS_CLIGetParameter
	(
		pcCommandString,				/* The command string itself. */
		1,								/* Return the x parameter. */
		&parameter_string_length_size	/* Store the parameter string length. */
	);
	
	size = (int8_t) atol((const char *) parameter_string);
	
	parameter_string = (int8_t *) FreeRTOS_CLIGetParameter
	(
		pcCommandString,				/* The command string itself. */
		2,								/* Return the x parameter. */
		&parameter_string_length_type	/* Store the parameter string length. */
	);
	
	type = (int8_t) atol((const char *) parameter_string);
	
	simulate_polyfit_regression(size, type);
	
	sprintf((char *) pcWriteBuffer,	"Simulation started! [size=%d] requested!\r\n", size);
	strncat((char *) pcWriteBuffer, "\r\n", strlen("\r\n"));
	
	
	return_value = pdFALSE;

	return return_value;
}


static portBASE_TYPE config_p_set_command(int8_t *pcWriteBuffer,
		size_t xWriteBufferLen,
		const int8_t *pcCommandString)
{
	int8_t *parameter_string;
	char protocol;
	int8_t protocol_define;
	int8_t x;
	int8_t y;
	int8_t enable;
	char type;
	int16_t radius;
	portBASE_TYPE parameter_string_length_protocol, parameter_string_length_x, parameter_string_length_y, parameter_string_length_enable, parameter_string_length_type, parameter_string_length_radius, return_value;
	static portBASE_TYPE parameter_number = 0;

	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	(void) pcCommandString;
	(void) xWriteBufferLen;
	configASSERT(pcWriteBuffer);
	
	//application_config(int protocol, int d_x, int d_y, char method /*(GET or SET)*/, int g_periodically_configured, int g_periodically_cycle_time, int g_periodically_sensor_type, int g_periodically_app_neighbor_lookup_type, int g_periodically_app_neighbor_lookup_radius)
	//config_p_set <x> <y> <enable(0=No/1=Yes)> <cycle_time(ms)> <sensor_type(L/A/P/T)> <square_radius>: \r\n  Config periodically task (Needed to make polynomial regression)\r\n\r\n",
	//config u 1 0 0 100 L 1 
		
	parameter_string = (int8_t *) FreeRTOS_CLIGetParameter
	(
		pcCommandString,				/* The command string itself. */
		1,								/* Return the x parameter. */
		&parameter_string_length_protocol	/* Store the parameter string length. */
	);
	
	protocol = parameter_string[0];
	
	if(protocol=='b'){
		protocol_define=PROTOCOL_BROADCAST;
	} else if(protocol=='m'){
		protocol_define=PROTOCOL_MULTICAST_1;
	} else {
		protocol_define=PROTOCOL_UNICAST;
	}
	
	parameter_string = (int8_t *) FreeRTOS_CLIGetParameter
	(
		pcCommandString,				/* The command string itself. */
		2,								/* Return the x parameter. */
		&parameter_string_length_x	/* Store the parameter string length. */
	);
	
	x = (int8_t) atol((const char *) parameter_string);
	
	parameter_string = (int8_t *) FreeRTOS_CLIGetParameter
	(
		pcCommandString,				/* The command string itself. */
		3,								/* Return the x parameter. */
		&parameter_string_length_y	/* Store the parameter string length. */
	);
	
	y = (int8_t) atol((const char *) parameter_string);
	
	parameter_string = (int8_t *) FreeRTOS_CLIGetParameter
	(
		pcCommandString,				/* The command string itself. */
		4,								/* Return the x parameter. */
		&parameter_string_length_enable	/* Store the parameter string length. */
	);
		
	enable = (int8_t) atol((const char *) parameter_string);
	
	parameter_string = (int8_t *) FreeRTOS_CLIGetParameter
	(
		pcCommandString,				/* The command string itself. */
		5,								/* Return the x parameter. */
		&parameter_string_length_type	/* Store the parameter string length. */
	);
		
	type = parameter_string[0];
	
	parameter_string = (int8_t *) FreeRTOS_CLIGetParameter
	(
		pcCommandString,				/* The command string itself. */
		6,								/* Return the x parameter. */
		&parameter_string_length_enable	/* Store the parameter string length. */
	);
	
	radius = (int16_t) atol((const char *) parameter_string);
	
	
	//application_config(protocol, x, y, 'set', int g_periodically_configured, int g_periodically_cycle_time, int g_periodically_sensor_type, int g_periodically_app_neighbor_lookup_type, int g_periodically_app_neighbor_lookup_radius);
	
	
	
	//simulate_polyfit_regression(size, type);
	
	//sprintf((char *) pcWriteBuffer,	"Simulation started! [size=%d] requested!\r\n", size);
	//strncat((char *) pcWriteBuffer, "\r\n", strlen("\r\n"));
	
	
	return_value = pdFALSE;

	return return_value;
}

static portBASE_TYPE regression_command(int8_t *pcWriteBuffer,
		size_t xWriteBufferLen,
		const int8_t *pcCommandString)
{
	int8_t *parameter_string;
	int8_t size, type; 
	portBASE_TYPE parameter_string_length_size, parameter_string_length_type, return_value;
	static portBASE_TYPE parameter_number = 0;

	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	(void) pcCommandString;
	(void) xWriteBufferLen;
	configASSERT(pcWriteBuffer);
	
		
	parameter_string = (int8_t *) FreeRTOS_CLIGetParameter
	(
		pcCommandString,				/* The command string itself. */
		1,								/* Return the x parameter. */
		&parameter_string_length_size	/* Store the parameter string length. */
	);
	
	size = (int8_t) atol((const char *) parameter_string);
	
	parameter_string = (int8_t *) FreeRTOS_CLIGetParameter
	(
		pcCommandString,				/* The command string itself. */
		2,								/* Return the x parameter. */
		&parameter_string_length_type	/* Store the parameter string length. */
	);
	
	type = (int8_t) atol((const char *) parameter_string);
	
	simulate_polyfit_regression(size, type);
	
	sprintf((char *) pcWriteBuffer,	"Simulation started! [size=%d] requested!\r\n", size);
	strncat((char *) pcWriteBuffer, "\r\n", strlen("\r\n"));
	
	
	return_value = pdFALSE;

	return return_value;
}




/*-----------------------------------------------------------*/