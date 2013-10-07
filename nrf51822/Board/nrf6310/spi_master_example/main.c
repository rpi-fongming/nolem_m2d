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
* @brief SPI Master loopback example.
* @defgroup spi_master_example SPI master loopback usage example
* @{
* @ingroup nrf_examples_nrf6310
*
* @brief SPI master example.
*
* This example needs that the slave is configured to transmit the received bytes. That is the slave
* behaves as a loopback device for the master. The loopback can also be achieved without using a slave device at all by wiring MOSI and
* MISO pins of the spi master together. @ref TX_RX_MSG_LENGTH number of bytes are transmitted through the master and the received bytes are
* verified to be the same as transmitted. IF there is an error, gpio pin for relevant spi module is set high to show the error @sa ERROR_PIN_SPI0,
* @sa ERROR_PIN_SPI1. If there is error from both modules that is if both pins are set high, then this application loops for ever
*
*/

#include "spi_master.h"
#include "nrf_delay.h"
#include "common.h"
#include "spi_master_config.h"

static uint8_t tx_data[TX_RX_MSG_LENGTH]; /*!< SPI TX buffer */
static uint8_t rx_data[TX_RX_MSG_LENGTH]; /*!< SPI RX buffer */

#define DELAY_MS               100        /*!< Timer Delay in milli-seconds */

/** Tests SPI master.
 * @param lsb_first If true, least significant bits are transferred first
 * @param mod_num spi module to be used, either SPI0 or SPI1 from enumeration SPIModuleNumber
 * @retval true 
 * @retval false Error occurred
 */
static bool test_spi_tx_rx(SPIModuleNumber mod_num, uint8_t lsb_first)
{
  // Use SPI0, mode0 with lsb shifted as requested
  uint32_t *spi_base_address = spi_master_init(mod_num, SPI_MODE0, (bool)lsb_first);
  if (spi_base_address == 0)
  {
    return false;
  }
  
/** @def debug_init
 * If debug is enabled @ref DEBUG, then this function will configure @ref DEBUG_EVENT_READY_PIN to toggle (using GPIOTE) everytime
 * READY_EVENTS are generated in the SPI
 * @note This flag will configure GPIOTE CONFIG0 and PPI channel 0, do not enable DEBUG while using two spi modules in parallel
 */
#ifdef DEBUG
    *(uint32_t *)0x40000504 = 0xC007FFDF; // Workaround for PAN_028 rev1.1 anomaly 23 - System: Manual setup is required to enable use of peripherals


    if ( NRF_SPI0_BASE == (uint32_t)spi_base_address )
    {
        nrf_gpio_cfg_output(DEBUG_EVENT_READY_PIN0);

        /*lint -e{845} // A zero has been given as right argument to operator '|'" */
        NRF_GPIOTE->CONFIG[0] = (GPIOTE_CONFIG_POLARITY_Toggle << GPIOTE_CONFIG_POLARITY_Pos) |
                                (DEBUG_EVENT_READY_PIN0 << GPIOTE_CONFIG_PSEL_Pos) |
                                (GPIOTE_CONFIG_MODE_Task << GPIOTE_CONFIG_MODE_Pos);

        NRF_PPI->CH[0].EEP = (uint32_t)&(((NRF_SPI_Type *)spi_base_address)->EVENTS_READY);
        NRF_PPI->CH[0].TEP = (uint32_t)&NRF_GPIOTE->TASKS_OUT[0];
        NRF_PPI->CHEN |= (PPI_CHEN_CH0_Enabled << PPI_CHEN_CH0_Pos);
    }

    if ( NRF_SPI1_BASE == (uint32_t)spi_base_address )
    {
        nrf_gpio_cfg_output(DEBUG_EVENT_READY_PIN1);

        /*lint -e{845} // A zero has been given as right argument to operator '|'" */
        NRF_GPIOTE->CONFIG[1] = (GPIOTE_CONFIG_POLARITY_Toggle << GPIOTE_CONFIG_POLARITY_Pos) |
                                (DEBUG_EVENT_READY_PIN1 << GPIOTE_CONFIG_PSEL_Pos) |
                                (GPIOTE_CONFIG_MODE_Task << GPIOTE_CONFIG_MODE_Pos);

        NRF_PPI->CH[1].EEP = (uint32_t)&(((NRF_SPI_Type *)spi_base_address)->EVENTS_READY);
        NRF_PPI->CH[1].TEP = (uint32_t)&NRF_GPIOTE->TASKS_OUT[1];
        NRF_PPI->CHEN |= (PPI_CHEN_CH1_Enabled << PPI_CHEN_CH1_Pos);
    }
#endif /* DEBUG */

  // Fill tx_data with some simple pattern, rx is filled with zero's so that after receiving from
  // slave we verify rx_Data is same as tx_data
  for(uint8_t i = 0; i < TX_RX_MSG_LENGTH; i++)
  {
    tx_data[i] = i;
    rx_data[i] = 0;
  }

  // Transmit TX_RX_MSG_LENGTH bytes from tx_data and receive same number of bytes and data into rx_data
  if(!spi_master_tx_rx(spi_base_address, TX_RX_MSG_LENGTH, (const uint8_t *)tx_data, rx_data) )
    return false;

  // Validate that we got all transmitted bytes back in the exact order
  for(uint8_t i = 0; i < TX_RX_MSG_LENGTH; i++)
  {
    if( tx_data[i] != rx_data[i] )
      return false;
  }
  return true;
}

/**
 * main() function
 * @return 0. int return type required by ANSI/ISO standard. 
 */
int main(void)
{
  bool ret0, ret1;

  NRF_GPIO->DIRSET = (1UL << ERROR_PIN_SPI0);
  NRF_GPIO->DIRSET = (1UL << ERROR_PIN_SPI1);
  while(true)
  {
    // SPI0
    ret0 = test_spi_tx_rx(SPI0, 1 );   /*!< test with shift Lsb first mode 0 */
    if (ret0)
    {
      // previous tx/rx was successful with lsb shifted first, try the same with lsb shifted last
      nrf_delay_ms(DELAY_MS);   /*!< This delay is just fot the events ready signal to be visually clear if connted to LED if DEBUG flag is enabled in spi_master library */
      ret0 = test_spi_tx_rx(SPI0, 0 );
    }

    if (!ret0)
    {
      // Set gpio pin number ERROR_PIN to convey error, this pin can be connected to LED for visual debug
      NRF_GPIO->OUTSET = (1UL << ERROR_PIN_SPI0);
    }

    // SPI1
    ret1 = test_spi_tx_rx(SPI1, 1 );   /*!< test with shift Lsb first mode 0 */
    if (ret1)
    {
      // previous tx/rx was successful with lsb shifted first, try the same with lsb shifted last
      nrf_delay_ms(DELAY_MS);   /*!< This delay is just fot the events ready signal to be visually clear if connted to LED if DEBUG flag is enabled in spi_master library */
      ret1 = test_spi_tx_rx(SPI1, 0 );
    }

    if (!ret1)
    {
      // Set gpio pin number ERROR_PIN to convey error, this pin can be connected to LED for visual debug
      NRF_GPIO->OUTSET = (1UL << ERROR_PIN_SPI1);
    }

    if (!ret0 && !ret1 )
    {
      while(true)
      {
          // Loop forever
      }
    }
  }
}

/**
 *@}
 **/
