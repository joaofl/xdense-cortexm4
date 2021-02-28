/*
 * led.h
 *
 * Created: 11/11/2014 17:27:13
 *  Author: Joao
 */ 


#ifndef LED_H_
#define LED_H_

#include "FreeRTOS.h"
#include "asf.h"
//#include "comm_stack/switch.h"

#define PWM_FULL_SCALE 10
#define PWM_PERIOD 40 //Human eye = 25fps = 0.04s = 40ms period

uint16_t pwm_counter;
portTickType xTimeNow, xLastTime, xPwmPeriod;

void led_set(uint8_t led, uint8_t color, uint8_t intensity);
void led_on(uint8_t led, uint8_t color);
void led_off(uint8_t led, uint8_t color);
void led_toggle(uint8_t led, uint8_t color, uint8_t intensity);

void led_set_all(uint16_t leds);

void led_get_all(uint16_t leds);

void led_control(void);

void led_init(void);

void led_blink(void);

void led_blink_all(uint32_t period);

typedef struct  
{
	uint8_t r_intensity;// = -1; //default max intensity
	bool r;// = false; //default turned off
	uint8_t g_intensity;// = -1;
	bool g;// = false;
	uint8_t b_intensity;// = -1;
	bool b;// = false;
	
	uint8_t r_blink_duration;// = -1; //default max intensity
	bool r_blink;// = false; //default turned off
	uint8_t g_blink_duration;// = -1;
	bool g_blink;// = false;
	uint8_t b_blink_duration;// = -1;
	bool b_blink;// = false;
}led;

led leds[PORT_COUNT];

#define LED_BLINK_MIN_PERIOD 5 //blinks should be multiple of 10 ms
#define LED_ERROR 100
#define LED_SUCCESS 1

#endif /* LED_H_ */