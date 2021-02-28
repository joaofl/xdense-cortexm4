/*
 * conf_tasks.h
 *
 * Created: 13/11/2014 16:37:58
 *  Author: Joao
 */ 


#ifndef CONF_TASKS_H_
#define CONF_TASKS_H_


///////////////// TASKS ///////////////////////

#define BOARD_USART	USART0

/* Comment/Uncomment the following definitions to enable/disable to corresponding tasks. */
/* Note: only the listed tasks have hardware support. */

//#define confINCLUDE_USART_ECHO_TASKS
#define confINCLUDE_USART_CLI
//#define confINCLUDE_CDC_CLI


#endif /* CONF_TASKS_H_ */