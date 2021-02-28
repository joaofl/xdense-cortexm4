/**
 * \file
 *
 * \brief User board initialization template
 *
 */

#include <asf.h>
#include <board.h>
#include <conf_board.h>

/**
 * \brief Set peripheral mode for IOPORT pins.
 * It will configure port mode and disable pin mode (but enable peripheral).
 * \param port IOPORT port to configure
 * \param masks IOPORT pin masks to configure
 * \param mode Mode masks to configure for the specified pin (\ref ioport_modes)
 */
#define ioport_set_port_peripheral_mode(port, masks, mode) \
	do {\
		ioport_set_port_mode(port, masks, mode);\
		ioport_disable_port(port, masks);\
	} while (0)

/**
 * \brief Set peripheral mode for one single IOPORT pin.
 * It will configure port mode and disable pin mode (but enable peripheral).
 * \param pin IOPORT pin to configure
 * \param mode Mode masks to configure for the specified pin (\ref ioport_modes)
 */
#define ioport_set_pin_peripheral_mode(pin, mode) \
	do {\
		ioport_set_pin_mode(pin, mode);\
		ioport_disable_pin(pin);\
	} while (0)


void board_init(void)
{
	/* This function is meant to contain board-specific initialization code
	 * for, e.g., the I/O pins. The initialization can rely on application-
	 * specific board configuration, found in conf_board.h.
	 */
	
	#ifndef CONF_BOARD_KEEP_WATCHDOG_AT_INIT
		/* Disable the watchdog */
		WDT->WDT_MR = WDT_MR_WDDIS;
	#endif	
	
	/* Initialize IOPORT */
	ioport_init();
	
	/* Set output direction on the given LED IOPORTs */
	//ioport_set_port_dir(LED_PORT_A, LED_MASK_A,	IOPORT_DIR_OUTPUT);
	//ioport_set_port_dir(LED_PORT_B, LED_MASK_B,	IOPORT_DIR_OUTPUT);
	
	/* Initialize LEDS, turned on */
	//The ones using pins shared with the JTAG have to have their MUX set
	//ioport_set_pin_mode(LEDNR_GPIO, IOPORT_MODE_MUX_BIT0);
	//ioport_set_pin_dir(LEDNR_GPIO, IOPORT_DIR_OUTPUT);
	
	//uint32_t r = matrix_get_system_io();
	
	#ifndef DEBUG
		//If not debugging, disable the debugging pins and use them as IO's for the LEDs.
		matrix_set_system_io(CCFG_SYSIO_SYSIO4 | CCFG_SYSIO_SYSIO5 | CCFG_SYSIO_SYSIO6 | CCFG_SYSIO_SYSIO7 | CCFG_SYSIO_SYSIO12);
	#endif

	/*North*/
	ioport_enable_pin(LEDNR_GPIO);
	ioport_set_pin_dir(LEDNR_GPIO, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(LEDNR_GPIO, IOPORT_PIN_LEVEL_LOW);
	
	ioport_enable_pin(LEDNG_GPIO);
	ioport_set_pin_dir(LEDNG_GPIO, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(LEDNG_GPIO, IOPORT_PIN_LEVEL_LOW);
	
	ioport_enable_pin(LEDNB_GPIO);
	ioport_set_pin_dir(LEDNB_GPIO, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(LEDNB_GPIO, IOPORT_PIN_LEVEL_LOW);

	/*South*/
	ioport_enable_pin(LEDSR_GPIO);
	ioport_set_pin_dir(LEDSR_GPIO, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(LEDSR_GPIO, IOPORT_PIN_LEVEL_LOW);
	
	ioport_enable_pin(LEDSG_GPIO);
	ioport_set_pin_dir(LEDSG_GPIO, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(LEDSG_GPIO, IOPORT_PIN_LEVEL_LOW);
	
	ioport_enable_pin(LEDSB_GPIO);
	ioport_set_pin_dir(LEDSB_GPIO, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(LEDSB_GPIO, IOPORT_PIN_LEVEL_LOW);
	
	/*East*/
	ioport_enable_pin(LEDER_GPIO);
	ioport_set_pin_dir(LEDER_GPIO, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(LEDER_GPIO, IOPORT_PIN_LEVEL_LOW);
	
	ioport_enable_pin(LEDEG_GPIO);
	ioport_set_pin_dir(LEDEG_GPIO, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(LEDEG_GPIO, IOPORT_PIN_LEVEL_LOW);
	
	ioport_enable_pin(LEDEB_GPIO);
	ioport_set_pin_dir(LEDEB_GPIO, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(LEDEB_GPIO, IOPORT_PIN_LEVEL_LOW);
	
	/*West*/
	ioport_enable_pin(LEDWR_GPIO);
	ioport_set_pin_dir(LEDWR_GPIO, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(LEDWR_GPIO, IOPORT_PIN_LEVEL_LOW);
	
	ioport_enable_pin(LEDWG_GPIO);
	ioport_set_pin_dir(LEDWG_GPIO, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(LEDWG_GPIO, IOPORT_PIN_LEVEL_LOW);
	
	ioport_enable_pin(LEDWB_GPIO);
	ioport_set_pin_dir(LEDWB_GPIO, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(LEDWB_GPIO, IOPORT_PIN_LEVEL_LOW);
	
	
	#ifdef CONF_BOARD_USART_RXD
		/* Configure USART RXD pin */
		ioport_set_pin_peripheral_mode(PIN_USART0_RXD_IDX, PIN_USART0_RXD_FLAGS);
	#endif

	#ifdef CONF_BOARD_USART_TXD
		/* Configure USART TXD pin */
		ioport_set_pin_peripheral_mode(PIN_USART0_TXD_IDX, PIN_USART0_TXD_FLAGS);
	#endif


	#ifdef CONF_BOARD_UART0
		/* Configure UART0 */
		//ioport_set_port_peripheral_mode(PINS_UART0_PORT, PINS_UART0, PINS_UART0_FLAGS);
		gpio_configure_group(PINS_UART0_PIO, PINS_UART0, PINS_UART0_FLAGS);
	#endif	

	#ifdef CONF_BOARD_UART1
		/* Configure UART0 */
		//ioport_set_port_peripheral_mode(PINS_UART1_PORT, PINS_UART1, PINS_UART1_FLAGS);
		gpio_configure_group(PINS_UART1_PIO, PINS_UART1, PINS_UART1_FLAGS);
	#endif

	#ifdef CONF_BOARD_UART2
		/* Configure UART0 */
		//ioport_set_port_peripheral_mode(PINS_UART2_PORT, PINS_UART2, PINS_UART2_FLAGS);
		gpio_configure_group(PINS_UART2_PIO, PINS_UART2, PINS_UART2_FLAGS);
	#endif

	#ifdef CONF_BOARD_UART3
		/* Configure UART0 */
		//ioport_set_pin_peripheral_mode(PINS_UART3, PINS_UART3_FLAGS);
		gpio_configure_group(PINS_UART3_PIO, PINS_UART3, PINS_UART3_FLAGS);
		//pio_configure(PINS_UART3_PIO, PINS_UART3_TYPE, PINS_UART3_MASK, PINS_UART3_ATTR);
		//enable the uart peripherial clock
		pmc_enable_periph_clk(ID_UART3);
	#else
		ioport_set_pin_dir(PIO_PB11_IDX, IOPORT_DIR_OUTPUT); //ALERT!!! Test mode
		ioport_set_pin_level(PIO_PB11_IDX, IOPORT_PIN_LEVEL_LOW);
	#endif
	
	#ifdef CONF_BOARD_TWI0
		ioport_set_pin_peripheral_mode(TWI0_DATA_GPIO, TWI0_DATA_FLAGS);
		ioport_set_pin_peripheral_mode(TWI0_CLK_GPIO, TWI0_CLK_FLAGS);
	#endif
}


