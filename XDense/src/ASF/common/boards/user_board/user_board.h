/**
 * \file
 *
 * \brief User board definition template
 *
 */

 /* This file is intended to contain definitions and configuration details for
 * features and devices that are available on the board, e.g., frequency and
 * startup time for an external crystal, external memory devices, LED and USART
 * pins.
 */

#ifndef USER_BOARD_H
#define USER_BOARD_H

#include <conf_board.h>

#define BOARD_NAME "XDense-SN"

#define PORT_NORTH 0
#define PORT_SOUTH 1
#define PORT_EAST  2
#define PORT_WEST  3
#define PORT_LOCAL  4

#define PORT_LOCAL_MASK 0b0000
#define PORT_NORTH_MASK 0b0001
#define PORT_SOUTH_MASK 0b0010
#define PORT_EAST_MASK  0b0100
#define PORT_WEST_MASK  0b1000
#define PORT_ALL_MASK   0b1111

#define PORT_COUNT 5



#define UART_NORTH UART3
#define UART_SOUTH UART2
#define UART_EAST  UART1
#define UART_WEST  UART0
//const static uint8_t PORT_ALL[PORT_COUNT] = {PORT_NORTH, PORT_SOUTH, PORT_EAST, PORT_WEST, PORT_LOCAL};

#define COLOR_R 0
#define COLOR_G 1
#define COLOR_B 2

/** UART0 pins (UTXD0 and URXD0) definitions, PA9,10. */
#define PINS_UART0         (PIO_PA9A_URXD0 | PIO_PA10A_UTXD0)
#define PINS_UART0_FLAGS   (PIO_PERIPH_A | PIO_DEFAULT)

#define PINS_UART0_MASK    PIO_PA9A_URXD0|PIO_PA10A_UTXD0
#define PINS_UART0_PIO     PIOA
#define PINS_UART0_ID      ID_PIOA
#define PINS_UART0_TYPE    PIO_PERIPH_A
#define PINS_UART0_ATTR    PIO_DEFAULT

/** UART1 pins (UTXD1 and URXD1) definitions, PB2,PB3. */
#define PINS_UART1         (PIO_PB2A_URXD1 | PIO_PB3A_UTXD1)
#define PINS_UART1_FLAGS   (PIO_PERIPH_A | PIO_DEFAULT)

#define PINS_UART1_MASK    PIO_PB2A_URXD1 | PIO_PB3A_UTXD1
#define PINS_UART1_PIO     PIOB
#define PINS_UART1_ID      ID_PIOB
#define PINS_UART1_TYPE    PIO_PERIPH_A
#define PINS_UART1_ATTR    PIO_DEFAULT

/** UART2 pins (UTXD3 and URXD3) definitions, PB2,3. */
#define PINS_UART2        (PIO_PA16A_URXD2 | PIO_PA15A_UTXD2)
#define PINS_UART2_FLAGS  (PIO_PERIPH_A | PIO_DEFAULT)

#define PINS_UART2_MASK   PIO_PA16A_URXD2 | PIO_PA15A_UTXD2
#define PINS_UART2_PIO    PIOA
#define PINS_UART2_ID     ID_PIOA
#define PINS_UART2_TYPE   PIO_PERIPH_A
#define PINS_UART2_ATTR   PIO_DEFAULT

/** UART3 pins (UTXD3 and URXD3) definitions, PB10,11. */
#define PINS_UART3        (PIO_PB10B_URXD3 | PIO_PB11B_UTXD3)
#define PINS_UART3_FLAGS  (PIO_PERIPH_B | PIO_DEFAULT)

#define PINS_UART3_MASK   PIO_PB10B_URXD3 | PIO_PB11B_UTXD3
#define PINS_UART3_PIO    PIOB
#define PINS_UART3_ID     ID_PIOB
#define PINS_UART3_TYPE   PIO_PERIPH_B
#define PINS_UART3_ATTR   PIO_DEFAULT

//#define PINS_UART3        (PIO_PB10B_URXD3 | PIO_PB11B_UTXD3)
//#define PINS_UART3_FLAGS  (IOPORT_MODE_MUX_B)
//
//#define PINS_UART3_PORT   IOPORT_PIOB
//#define PINS_UART3_MASK   (PIO_PB10B_URXD3 | PIO_PB11B_UTXD3)
//#define PINS_UART3_PIO    PIOB
//#define PINS_UART3_ID     ID_PIOB
//#define PINS_UART3_TYPE   PIO_PERIPH_B
//#define PINS_UART3_ATTR   PIO_DEFAULT




/** USART0 pin RX */
#define PIN_USART0_RXD        {PIO_PA5A_RXD0, PIOA, ID_PIOA, PIO_PERIPH_A, \
PIO_DEFAULT}
#define PIN_USART0_RXD_IDX    (PIO_PA5_IDX)
#define PIN_USART0_RXD_FLAGS  (IOPORT_MODE_MUX_A)
/** USART0 pin TX */
#define PIN_USART0_TXD        {PIO_PA6A_TXD0, PIOA, ID_PIOA, PIO_PERIPH_A, \
PIO_DEFAULT}
#define PIN_USART0_TXD_IDX    (PIO_PA6_IDX)
#define PIN_USART0_TXD_FLAGS  (IOPORT_MODE_MUX_A)
/** USART0 pin CTS */
#define PIN_USART0_CTS        {PIO_PA8A_CTS0, PIOA, ID_PIOA, PIO_PERIPH_A, \
PIO_DEFAULT}
#define PIN_USART0_CTS_IDX    (PIO_PA8_IDX)
#define PIN_USART0_CTS_FLAGS  (IOPORT_MODE_MUX_A)
/** USART0 pin RTS */
#define PIN_USART0_RTS        {PIO_PA7A_RTS0, PIOA, ID_PIOA, PIO_PERIPH_A, \
PIO_DEFAULT}
#define PIN_USART0_RTS_IDX    (PIO_PA7_IDX)
#define PIN_USART0_RTS_FLAGS  (IOPORT_MODE_MUX_A)
/** USART0 pin SCK */
#define PIN_USART0_SCK        {PIO_PA2B_SCK0, PIOA, ID_PIOA, PIO_PERIPH_B, \
PIO_DEFAULT}
#define PIN_USART0_SCK_IDX    (PIO_PA2_IDX)
#define PIN_USART0_SCK_FLAGS  (IOPORT_MODE_MUX_B)

//#define CONF_BOARD_KEEP_WATCHDOG_AT_INIT


//////////////////////// TWO WIRE - I2C //////////////////////
/** TWI0 pin definitions */
#define TWI0_DATA_GPIO   PIO_PA3_IDX
#define TWI0_DATA_FLAGS  IOPORT_MODE_MUX_A
#define TWI0_CLK_GPIO    PIO_PA4_IDX
#define TWI0_CLK_FLAGS   IOPORT_MODE_MUX_A


//! \name LED0 definitions
//@{
	
//#define LED_PORT_A (0)
//#define LED_MASK_A 0xFFFFFFFF//(0b010001111110000)
//#define LED_PORT_B (1)
//#define LED_MASK_B 0xFFFFFFFF//(0b110100000000100)

////////////////////// L4 - NORTH //////////////////////////////////
#define LEDNR_GPIO						 PIO_PB5_IDX
#define LEDNR_ACTIVE_LEVEL               false
#define LEDNR_INACTIVE_LEVEL             !LEDNR_ACTIVE_LEVEL

#define LEDNG_GPIO						 PIO_PB7_IDX
#define LEDNG_ACTIVE_LEVEL               false
#define LEDNG_INACTIVE_LEVEL             !LEDNG_ACTIVE_LEVEL

#define LEDNB_GPIO						 PIO_PB6_IDX
#define LEDNB_ACTIVE_LEVEL               false
#define LEDNB_INACTIVE_LEVEL             !LEDNB_ACTIVE_LEVEL

/////////////////////////////////////////////////////////////

////////////////////// L3 - EAST //////////////////////////////////
#define LEDER_GPIO						 PIO_PB12_IDX
#define LEDER_ACTIVE_LEVEL               false
#define LEDER_INACTIVE_LEVEL             !LEDER_ACTIVE_LEVEL

#define LEDEG_GPIO						 PIO_PB9_IDX
#define LEDEG_ACTIVE_LEVEL               false
#define LEDEG_INACTIVE_LEVEL             !LEDEG_ACTIVE_LEVEL

#define LEDEB_GPIO						 PIO_PB8_IDX
#define LEDEB_ACTIVE_LEVEL               false
#define LEDEB_INACTIVE_LEVEL             !LEDEB_ACTIVE_LEVEL

/////////////////////////////////////////////////////////////

////////////////////// L2 - SOUTH //////////////////////////////////
#define LEDSR_GPIO						 PIO_PA13_IDX
#define LEDSR_ACTIVE_LEVEL               false
#define LEDSR_INACTIVE_LEVEL             !LEDSR_ACTIVE_LEVEL

#define LEDSG_GPIO						 PIO_PA14_IDX
#define LEDSG_ACTIVE_LEVEL               false
#define LEDSG_INACTIVE_LEVEL             !LEDSG_ACTIVE_LEVEL

#define LEDSB_GPIO						 PIO_PA2_IDX
#define LEDSB_ACTIVE_LEVEL               false
#define LEDSB_INACTIVE_LEVEL             !LEDSB_ACTIVE_LEVEL

/////////////////////////////////////////////////////////////

////////////////////// L1 - WEST //////////////////////////////////
#define LEDWR_GPIO						 PIO_PA11_IDX
#define LEDWR_ACTIVE_LEVEL               false
#define LEDWR_INACTIVE_LEVEL             !LEDWR_ACTIVE_LEVEL

#define LEDWG_GPIO						 PIO_PB4_IDX
#define LEDWG_ACTIVE_LEVEL               false
#define LEDWG_INACTIVE_LEVEL             !LEDWG_ACTIVE_LEVEL

#define LEDWB_GPIO						 PIO_PA12_IDX
#define LEDWB_ACTIVE_LEVEL               false
#define LEDWB_INACTIVE_LEVEL             !LEDWB_ACTIVE_LEVEL

/////////////////////////////////////////////////////////////

#define LED_COUNT 12

static const uint8_t LEDS[LED_COUNT] = {LEDNR_GPIO, LEDNG_GPIO, LEDNB_GPIO, 
										LEDSR_GPIO, LEDSG_GPIO, LEDSB_GPIO,
										LEDER_GPIO, LEDEG_GPIO, LEDEB_GPIO, 
										LEDWR_GPIO, LEDWG_GPIO, LEDWB_GPIO};



#endif // USER_BOARD_H
