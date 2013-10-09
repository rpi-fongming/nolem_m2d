/* Copyright (c) 2009 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

/** @file
* @brief Example project on GPIO usage to drive LEDs
* @defgroup gpio_example_pca10001 GPIO example
* @{
* @ingroup nrf_examples_pca10001
*
* @brief Example of GPIO output usage.
*
* This example shows how to configure GPIO pins as outputs which can also be used to drive LEDs.
* Each LED is set on one at a time and each state lasts 100 milliseconds.
*/

#include <stdbool.h>
#include <stdint.h>
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "boards.h"

#define DEBOUNCE_TIME_IN_MS (50U) //!< Debounce timer in milliseconds
#define DEBOUNCE_INPUT_SAMPLING_FREQ (60U) //!< Input sampling frequency in Hertz

#define TIMER0_PRESCALER (9UL) /*!< Timer 0 prescaler */
#define TIMER0_CLOCK (SystemCoreClock >> TIMER0_PRESCALER) /*!< Timer clock frequency */

#define MS_TO_TIMER0_TICKS(ms) ((1000000UL * ms) / (TIMER0_CLOCK)) /*!< Converts milliseconds to timer ticks */

static uint_fast16_t timer0_cc0_period; /*!< Period between debouncer input reads. */

/** Initializes Timer 0 peripheral.
 */
static void timer0_init(void);

/** Timer 0 peripheral interrupt handler.
 */
void TIMER0_IRQHandler(void)
{
	static unsigned int tCnt = 0;
	
  if ((NRF_TIMER0->EVENTS_COMPARE[0] != 0) && ((NRF_TIMER0->INTENSET & TIMER_INTENSET_COMPARE0_Msk) != 0))
  {
    NRF_TIMER0->EVENTS_COMPARE[0] = 0;
    NRF_TIMER0->CC[0] += timer0_cc0_period;
		if (tCnt==0)
		{
			tCnt = 1;
//			nrf_gpio_port_write(NRF_GPIO_PORT_SELECT_PORT0, nrf_gpio_port_read(NRF_GPIO_PORT_SELECT_PORT0) ^ 0xff); 	
		}
		else
		{
			tCnt -= 1;
		}
			nrf_gpio_port_write(NRF_GPIO_PORT_SELECT_PORT0, nrf_gpio_port_read(NRF_GPIO_PORT_SELECT_PORT0) ^ 0xff); 	
	

  }
}

static void timer0_init(void)
{
  NRF_TIMER0->MODE = TIMER_MODE_MODE_Timer; // Set the timer in Timer Mode
  NRF_TIMER0->PRESCALER = TIMER0_PRESCALER;
  NRF_TIMER0->BITMODE = TIMER_BITMODE_BITMODE_24Bit;  // 24-bit mode

  // Enable interrupt for COMPARE[0]
  NRF_TIMER0->INTENSET = (1UL << TIMER_INTENSET_COMPARE0_Pos);
  NRF_TIMER0->CC[0] = timer0_cc0_period;
  NRF_TIMER0->TASKS_START = 1; // Start clocks
}

uint8_t key_scan(uint8_t tKey)
{
		static uint8_t pkey=0;
		uint8_t tMode=0;
		
		if (tKey==0)
		{
				tMode = pkey;
				pkey = 0;
		}
		else
		{
				tMode = pkey | 0x01;		
				pkey = 0x02;
		}

		return (tMode & 0x03);
	
}


/**
 * main() function
 * @return 0. int return type required by ANSI/ISO standard.
 */
int main(void)
{
	uint8_t tCnt=0;
	uint8_t tKey=0;
	uint8_t tMode=4;
	
	
  // Configure LED-pins as outputs
	nrf_gpio_cfg_output(LED_BLUE);
	nrf_gpio_cfg_output(LED_RED);
	nrf_gpio_cfg_output(NeuroSky_VCC);
	nrf_gpio_cfg_input(KEY_1,NRF_GPIO_PIN_PULLUP);
	nrf_gpio_cfg_input(Battery_AD,NRF_GPIO_PIN_PULLUP);
	nrf_gpio_cfg_input(Battery_Check,NRF_GPIO_PIN_PULLUP);

  // DEBOUNCE_INPUT_SAMPLING_FREQ is in hertz so we need to multiply by 1000 to get milliseconds
  timer0_cc0_period = MS_TO_TIMER0_TICKS((1000 * 1 / DEBOUNCE_INPUT_SAMPLING_FREQ));
  timer0_init();

//  NVIC_EnableIRQ(TIMER0_IRQn); // Enable Interrupt for the timer in the core
//  __enable_irq();
	
	nrf_gpio_port_set(NRF_GPIO_PORT_SELECT_PORT0, 1<<LED_RED);
	nrf_gpio_port_clear(NRF_GPIO_PORT_SELECT_PORT0, 1<<LED_BLUE);
	
	tKey = key_scan (nrf_gpio_pin_read(KEY_1));
	while(true)
  {
    nrf_delay_ms(100);
	  tCnt ++;
		tKey = key_scan (nrf_gpio_pin_read(KEY_1));
	
		switch (tKey)
		{
			case 0:	// key : no change at "0"	-> Hold
				break;
			case 1:	// key : change from "0" to "1"	-> released
				break;
			case 2:	// key : change from "1" to "0" -> pressed
				tMode ++;
				if (tMode >=6)
						tMode = 0;
				break;
			case 3:	// key : no change at "1" -> no action
				break;
		}

		switch (tMode)
		{
			case 0:
						nrf_gpio_port_clear(NRF_GPIO_PORT_SELECT_PORT0, 1<<LED_RED);
						nrf_gpio_port_clear(NRF_GPIO_PORT_SELECT_PORT0, 1<<LED_BLUE);
						nrf_gpio_port_set(NRF_GPIO_PORT_SELECT_PORT0, 1<<NeuroSky_VCC);
						break;
				
			case 1:
						nrf_gpio_port_set(NRF_GPIO_PORT_SELECT_PORT0, 1<<LED_RED);
						nrf_gpio_port_clear(NRF_GPIO_PORT_SELECT_PORT0, 1<<LED_BLUE);
						nrf_gpio_port_clear(NRF_GPIO_PORT_SELECT_PORT0, 1<<NeuroSky_VCC);
						break;

			case 2:
						nrf_gpio_port_clear(NRF_GPIO_PORT_SELECT_PORT0, 1<<LED_RED);
						nrf_gpio_port_set(NRF_GPIO_PORT_SELECT_PORT0, 1<<LED_BLUE);
						nrf_gpio_port_set(NRF_GPIO_PORT_SELECT_PORT0, 1<<NeuroSky_VCC);
						break;
				
			case 3:
						nrf_gpio_port_set(NRF_GPIO_PORT_SELECT_PORT0, 1<<LED_RED);
						nrf_gpio_port_set(NRF_GPIO_PORT_SELECT_PORT0, 1<<LED_BLUE);
						nrf_gpio_port_clear(NRF_GPIO_PORT_SELECT_PORT0, 1<<NeuroSky_VCC);
						break;			
			case 4:
						if ((tCnt & 0x0f) == 0x00)
						{
							nrf_gpio_port_set(NRF_GPIO_PORT_SELECT_PORT0, 1<<LED_RED);
							nrf_gpio_port_set(NRF_GPIO_PORT_SELECT_PORT0, 1<<LED_BLUE);
						}			
						if ((tCnt & 0x0f) == 0x08)
						{
							nrf_gpio_port_clear(NRF_GPIO_PORT_SELECT_PORT0, 1<<LED_RED);
							nrf_gpio_port_clear(NRF_GPIO_PORT_SELECT_PORT0, 1<<LED_BLUE);
						}			
					break;		
			case 5:
						if ((tCnt & 0x0f) == 0x00)
						{
							nrf_gpio_port_set(NRF_GPIO_PORT_SELECT_PORT0, 1<<LED_RED);
							nrf_gpio_port_clear(NRF_GPIO_PORT_SELECT_PORT0, 1<<LED_BLUE);
						}			
						if ((tCnt & 0x0f) == 0x08)
						{
							nrf_gpio_port_clear(NRF_GPIO_PORT_SELECT_PORT0, 1<<LED_RED);
							nrf_gpio_port_set(NRF_GPIO_PORT_SELECT_PORT0, 1<<LED_BLUE);
						}			
					break;		

		}		

	}
		
}

/**
 *@}
 **/
