/*
 * led.c
 *
 * Created: 11/11/2014 17:27:02
 *  Author: Joao
 */ 



#include "led.h"

void led_init(void){
	
	for (uint8_t i = 0 ; i < PORT_COUNT ; i++){


		leds[i].r = true; //initialize it off
		leds[i].r_intensity = PWM_FULL_SCALE/3;
	
		leds[i].g = true; //initialize it off
		leds[i].g_intensity = PWM_FULL_SCALE/3;		
		
		leds[i].b = true; //initialize it off
		leds[i].b_intensity = PWM_FULL_SCALE/3;


		leds[i].r_blink = false; //initialize it off
		leds[i].r_blink_duration = 0;
		
		leds[i].g_blink = false; //initialize it off
		leds[i].g_blink_duration = 0;
		
		leds[i].b_blink = false; //initialize it off
		leds[i].b_blink_duration = 0;
	}

	pwm_counter = 0;
}

void led_set(uint8_t led, uint8_t color, uint8_t intensity){
	
}



void led_on(uint8_t led, uint8_t color){
	
	switch (led)
	{
	case PORT_NORTH:
		switch (color)
		{
		case COLOR_R:
			ioport_set_pin_level(LEDNR_GPIO, IOPORT_PIN_LEVEL_LOW);
			break;
		case COLOR_G:
			ioport_set_pin_level(LEDNG_GPIO, IOPORT_PIN_LEVEL_LOW);
			break;
		case COLOR_B:
			ioport_set_pin_level(LEDNB_GPIO, IOPORT_PIN_LEVEL_LOW);
			break;
		}
		break;
		
	case PORT_SOUTH:
		switch (color)
		{
			case COLOR_R:
			ioport_set_pin_level(LEDSR_GPIO, IOPORT_PIN_LEVEL_LOW);
			break;
			case COLOR_G:
			ioport_set_pin_level(LEDSG_GPIO, IOPORT_PIN_LEVEL_LOW);
			break;
			case COLOR_B:
			ioport_set_pin_level(LEDSB_GPIO, IOPORT_PIN_LEVEL_LOW);
			break;
		}
		break;
	
	case PORT_EAST:
		switch (color)
		{
			case COLOR_R:
			ioport_set_pin_level(LEDER_GPIO, IOPORT_PIN_LEVEL_LOW);
			break;
			case COLOR_G:
			ioport_set_pin_level(LEDEG_GPIO, IOPORT_PIN_LEVEL_LOW);
			break;
			case COLOR_B:
			ioport_set_pin_level(LEDEB_GPIO, IOPORT_PIN_LEVEL_LOW);
			break;
		}
		break;
	
	case PORT_WEST:
		switch (color)
		{
			case COLOR_R:
			ioport_set_pin_level(LEDWR_GPIO, IOPORT_PIN_LEVEL_LOW);
			break;
			case COLOR_G:
			ioport_set_pin_level(LEDWG_GPIO, IOPORT_PIN_LEVEL_LOW);
			break;
			case COLOR_B:
			ioport_set_pin_level(LEDWB_GPIO, IOPORT_PIN_LEVEL_LOW);
			break;
		}
		
		break;
	}
	
	//ioport_set_pin_level(LED0_GPIO, IOPORT_PIN_LEVEL_LOW);
}

void led_set_all(uint16_t leds){
	// rgb rgb rgb rgb
	//  N   S   E   W
	
	uint16_t i = 0;
	uint16_t led = 0;
	uint16_t mask;
	
	//mask = 0b000000000001 << i;
	//led = leds & mask;
	//
	//if (led == 0)
	//ioport_set_pin_level(LEDSB_GPIO, IOPORT_PIN_LEVEL_LOW);
	//else if (led == 1)
	//ioport_set_pin_level(LEDSB_GPIO, IOPORT_PIN_LEVEL_HIGH);
	
	//TODO: change this to parallel set on the ports.
	//keep track of the leds currently on, and only set the required ones using a mask

	while (i < LED_COUNT){
		mask = 0b000000000001 << i;
		led = (leds & mask) >> i;
		if (led == 0)
			ioport_set_pin_level(LEDS[i], IOPORT_PIN_LEVEL_HIGH);
		else if (led == 1)
			ioport_set_pin_level(LEDS[i], IOPORT_PIN_LEVEL_LOW);
		
		i++;
	}
}

void led_get_all(uint16_t mask){
	
}

void led_off(uint8_t led, uint8_t color){
	switch(led)
	{
		case PORT_NORTH:
			switch (color)
			{
				case COLOR_R:
				ioport_set_pin_level(LEDNR_GPIO, IOPORT_PIN_LEVEL_HIGH);
				break;
				case COLOR_G:
				ioport_set_pin_level(LEDNG_GPIO, IOPORT_PIN_LEVEL_HIGH);
				break;
				case COLOR_B:
				ioport_set_pin_level(LEDNB_GPIO, IOPORT_PIN_LEVEL_HIGH);
				break;
			}
			break;
	
		case PORT_SOUTH:
			switch (color)
			{
				case COLOR_R:
				ioport_set_pin_level(LEDSR_GPIO, IOPORT_PIN_LEVEL_HIGH);
				break;
				case COLOR_G:
				ioport_set_pin_level(LEDSG_GPIO, IOPORT_PIN_LEVEL_HIGH);
				break;
				case COLOR_B:
				ioport_set_pin_level(LEDSB_GPIO, IOPORT_PIN_LEVEL_HIGH);
				break;
			}
			break;
	
		case PORT_EAST:
			switch (color)
			{
				case COLOR_R:
				ioport_set_pin_level(LEDER_GPIO, IOPORT_PIN_LEVEL_HIGH);
				break;
				case COLOR_G:
				ioport_set_pin_level(LEDEG_GPIO, IOPORT_PIN_LEVEL_HIGH);
				break;
				case COLOR_B:
				ioport_set_pin_level(LEDEB_GPIO, IOPORT_PIN_LEVEL_HIGH);
				break;
			}
			break;
	
		case PORT_WEST:
			switch (color)
			{
				case COLOR_R:
				ioport_set_pin_level(LEDWR_GPIO, IOPORT_PIN_LEVEL_HIGH);
				break;
				case COLOR_G:
				ioport_set_pin_level(LEDWG_GPIO, IOPORT_PIN_LEVEL_HIGH);
				break;
				case COLOR_B:
				ioport_set_pin_level(LEDWB_GPIO, IOPORT_PIN_LEVEL_HIGH);
				break;
			}
			break;
	}
	
}

void led_toggle(uint8_t led, uint8_t color, uint8_t intensity){
	//ioport_toggle_pin_level(LED0_GPIO);
}



void led_control(void){
	
	//portTickType xTimeNow, xLastTime;
	
	xTimeNow = xTaskGetTickCount();
	
	
	for (uint8_t i = 0 ; i < PORT_COUNT ; i++){
		
		//TODO: Set all in a single operation, in parallel.
		
		////////////
		if (leds[i].r_intensity < pwm_counter || leds[i].r == false){
			/*Keep it on until  the counter exceeds the desirable intensity, to compose the pwm*/
			ioport_set_pin_level(LEDS[i*3 + 0], IOPORT_PIN_LEVEL_HIGH); 
		}
		else if(leds[i].r_intensity >= pwm_counter && leds[i].r == true){
			ioport_set_pin_level(LEDS[i*3 + 0], IOPORT_PIN_LEVEL_LOW);
		}
		////////////
		if (leds[i].g_intensity < pwm_counter || leds[i].g == false){
			/*Keep it on until  the counter exceeds the desirable intensity, to compose the pwm*/
			ioport_set_pin_level(LEDS[i*3 + 1], IOPORT_PIN_LEVEL_HIGH); 
		}
		else if(leds[i].g_intensity >= pwm_counter && leds[i].g == true){
			ioport_set_pin_level(LEDS[i*3 + 1], IOPORT_PIN_LEVEL_LOW);
		}
		//////////////
		if (leds[i].b_intensity < pwm_counter || leds[i].b == false){
			/*Keep it on until  the counter exceeds the desirable intensity, to compose the pwm*/
			ioport_set_pin_level(LEDS[i*3 + 2], IOPORT_PIN_LEVEL_HIGH); 
		}
		else if(leds[i].b_intensity >= pwm_counter && leds[i].b == true){
			ioport_set_pin_level(LEDS[i*3 + 2], IOPORT_PIN_LEVEL_LOW);
		}
		/////////////

	}
	
	//TODO: change it to time comparison instead, and put it as a background task
	pwm_counter++;
	if (pwm_counter > PWM_FULL_SCALE) pwm_counter = 0;
}

void led_blink(void){
		uint8_t i;
		for (i = 0 ; i < PORT_COUNT ; i++){
			
			//TODO: Set all in a single operation, in parallel.
			
			////////////
			if (leds[i].r_blink_duration > 0){
				/*Keep it on until  the counter exceeds the desirable intensity, to compose the pwm*/
				leds[i].r_blink_duration -= LED_BLINK_MIN_PERIOD;
				ioport_set_pin_level(LEDS[i*3 + 0], IOPORT_PIN_LEVEL_LOW);
			}
			else{
				ioport_set_pin_level(LEDS[i*3 + 0], IOPORT_PIN_LEVEL_HIGH);
			}
			////////////
			if (leds[i].g_blink_duration > 0){
				/*Keep it on until  the counter exceeds the desirable intensity, to compose the pwm*/
				leds[i].g_blink_duration -= LED_BLINK_MIN_PERIOD;
				ioport_set_pin_level(LEDS[i*3 + 1], IOPORT_PIN_LEVEL_LOW);
			}
			else{
				ioport_set_pin_level(LEDS[i*3 + 1], IOPORT_PIN_LEVEL_HIGH);
			}
			//////////////
			if (leds[i].b_blink_duration > 0){
				/*Keep it on until  the counter exceeds the desirable intensity, to compose the pwm*/
				leds[i].b_blink_duration -= LED_BLINK_MIN_PERIOD;
				ioport_set_pin_level(LEDS[i*3 + 2], IOPORT_PIN_LEVEL_LOW);
			}
			else{
				ioport_set_pin_level(LEDS[i*3 + 2], IOPORT_PIN_LEVEL_HIGH);
			}
			/////////////

		}
}

void led_blink_all(uint32_t period){

}