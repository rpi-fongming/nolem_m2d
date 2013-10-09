/* Copyright (c) 2012 Nordic Semiconductor. All Rights Reserved.
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
#ifndef MELON_H
#define MELON_H


// PORT0
#define Battery_AD		 01
#define LED_BLUE       04
#define KEY_1        	 05
#define NeuroSky_VCC   06
#define LED_RED        07
#define LED_PORT_BR		 NRF_GPIO_PORT_SELECT_PORT0		// Port 0 (GPIO pin 0-7)

#define Battery_Check  8


#define RX_PIN_NUMBER  11
#define TX_PIN_NUMBER  9
#define CTS_PIN_NUMBER 10
#define RTS_PIN_NUMBER 8
#define HWFC           true

#define BLINKY_STATE_MASK   0x01

#endif
