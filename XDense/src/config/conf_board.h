/**
 * \file
 *
 * \brief User board configuration template
 *
 */

#ifndef CONF_BOARD_H
#define CONF_BOARD_H

#define DEBUG

//#define PORT_NORTH_TXIO_DEBUG_MODE //USING TX PIN OF PORT NORTH FOR DEBUG, communication by this port will not be possible

//#define confINCLUDE_USART_CLI

// Config CONSOLE at USART0
#define CONSOLE_USART				  USART0
#define CONSOLE_USART_ID			  ID_USART0
#define CONSOLE_USART_BAUDRATE		  1382400
#define CONSOLE_USART_PARITY		  US_MR_PAR_NO
#define CONSOLE_USART_STOP_BIT		  US_MR_NBSTOP_1_BIT
#define CONSOLE_USART_CHAR_LENGTH     US_MR_CHRL_8_BIT

#define CONF_BOARD_USART_RXD
#define CONF_BOARD_USART_TXD

#define CONF_BOARD_UART0 
#define CONF_BOARD_UART1
#define CONF_BOARD_UART2
#ifndef PORT_NORTH_TXIO_DEBUG_MODE
	#define CONF_BOARD_UART3
#endif

#define CONF_BOARD_TWI0



/* The priorities at which various tasks will get created. */
#define mainUART_CLI_TASK_PRIORITY              (tskIDLE_PRIORITY + 1)

/* The stack sizes allocated to the various tasks. */
//#define mainUART_CLI_TASK_STACK_SIZE    (configMINIMAL_STACK_SIZE * 2)
#define mainUART_CLI_TASK_STACK_SIZE    (5048/sizeof(portSTACK_TYPE)) //2048



#define configUSE_TICKLESS_IDLE 0

// External oscillator settings.
// Uncomment and set correct values if external oscillator is used.

// External oscillator frequency
//#define BOARD_XOSC_HZ          8000000

// External oscillator type.
//!< External clock signal
//#define BOARD_XOSC_TYPE        XOSC_TYPE_EXTERNAL
//!< 32.768 kHz resonator on TOSC
//#define BOARD_XOSC_TYPE        XOSC_TYPE_32KHZ
//!< 0.4 to 16 MHz resonator on XTALS
//#define BOARD_XOSC_TYPE        XOSC_TYPE_XTAL

// External oscillator startup time
//#define BOARD_XOSC_STARTUP_US  500000

#define BOARD_FREQ_SLCK_XTAL      (32768U)
#define BOARD_FREQ_SLCK_BYPASS    (32768U)
#define BOARD_FREQ_MAINCK_XTAL    (12000000U)
#define BOARD_FREQ_MAINCK_BYPASS  (12000000U)
#define BOARD_MCK                 CHIP_FREQ_CPU_MAX
#define BOARD_OSC_STARTUP_US      15625



#endif // CONF_BOARD_H
