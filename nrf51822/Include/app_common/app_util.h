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

/** @file
 *
 * @defgroup app_util Utility Functions and Definitions
 * @{
 * @ingroup app_common
 *
 * @brief Various types and definitions available to all applications.
 */

#ifndef APP_UTIL_H__
#define APP_UTIL_H__

#include <stdint.h>
#include "compiler_abstraction.h"
#include "nrf51.h"
#include "app_error.h"

/**@brief The interrupt priorities available to the application while the softdevice is active. */
typedef enum {
    APP_IRQ_PRIORITY_HIGH = 1,
    APP_IRQ_PRIORITY_LOW  = 3
} app_irq_priority_t;

#define NRF_APP_PRIORITY_THREAD    4                    /**< "Interrupt level" when running in Thread Mode. */

/**@cond NO_DOXYGEN */
#define EXTERNAL_INT_VECTOR_OFFSET 16
/**@endcond */

#define PACKED(TYPE) __packed TYPE

/**@brief Macro for doing static (i.e. compile time) assertion.
 *
 * @note If the assertion fails when compiling using Keil, the compiler will report error message
 *       "error: #94: the size of an array must be greater than zero" (while gcc will list the
 *       symbol static_assert_failed, making the error message more readable).
 *       If the supplied expression can not be evaluated at compile time, Keil will report
 *       "error: #28: expression must have a constant value".
 *
 * @note The macro is intentionally implemented not using do while(0), allowing it to be used
 *       outside function blocks (e.g. close to global type- and variable declarations).
 *       If used in a code block, it must be used before any executable code in this block.
 *
 * @param[in]   EXPR   Constant expression to be verified.
 */
#define STATIC_ASSERT(EXPR) extern char static_assert_failed[(EXPR) ? 1 : -1]

/**@brief type for holding an encoded (i.e. little endian) 16 bit unsigned integer. */
typedef uint8_t uint16_le_t[2];

/**@brief type for holding an encoded (i.e. little endian) 32 bit unsigned integer. */
typedef uint8_t uint32_le_t[4];

/**@brief Byte array type. */
typedef struct
{
    uint16_t  size;                 /**< Number of array entries. */
    uint8_t * p_data;               /**< Pointer to array entries. */
} uint8_array_t;

/**@cond NO_DOXYGEN */
#define INTERN_GPIO_PIN_CONFIG(PIN_NO, IO_DIR, IO_INPUT, IO_PULL, IO_DRIVE, IO_SENSE) \
    do{                                                                               \
        NRF_GPIO->PIN_CNF[(PIN_NO)] =                                                 \
            ((IO_DIR)   << GPIO_PIN_CNF_DIR_Pos)   |                                  \
            ((IO_INPUT) << GPIO_PIN_CNF_INPUT_Pos) |                                  \
            ((IO_PULL)  << GPIO_PIN_CNF_PULL_Pos)  |                                  \
            ((IO_DRIVE) << GPIO_PIN_CNF_DRIVE_Pos) |                                  \
            ((IO_SENSE) << GPIO_PIN_CNF_SENSE_Pos);                                   \
    } while (0)
/**@endcond */

/**@brief Macro to configure an I/O.
 *
 * @details This macro configures a given I/O to input or output with pullup/buffer configuration.
 *
 * @param[in]  PIN_NO     I/O pin to configure.
 * @param[in]  IO_DIR     I/O pin direction.
 * @param[in]  IO_INPUT   Connect or disconnect input path.
 * @param[in]  IO_PULL    Pull-up or -down configuration.
 * @param[in]  IO_DRIVE   Drive configuration.
 * @param[in]  IO_SENSE   Pin sensing mechanism.
 */
#define GPIO_PIN_CONFIG(PIN_NO, IO_DIR, IO_INPUT, IO_PULL, IO_DRIVE, IO_SENSE) \
            INTERN_GPIO_PIN_CONFIG(PIN_NO, IO_DIR, IO_INPUT, IO_PULL, IO_DRIVE, IO_SENSE)

/**@brief Configure a PIN to be used as a LED.
 *
 * @param[in]  PIN_NO  Input pin to configure.
 */
#define GPIO_LED_CONFIG(PIN_NO)                       \
    do                                                \
    {                                                 \
        GPIO_PIN_CONFIG(PIN_NO,                       \
                        GPIO_PIN_CNF_DIR_Output,      \
                        GPIO_PIN_CNF_INPUT_Disconnect,\
                        GPIO_PIN_CNF_PULL_Disabled,   \
                        GPIO_PIN_CNF_DRIVE_S0S1,      \
                        GPIO_PIN_CNF_SENSE_Disabled); \
    } while (0)

/**@brief Configure a PIN to be used as a debug pin (e.g. to be monitored using a logic analyzer).
 *
 * @param[in]  PIN_NO  Input pin to configure.
 */
#define GPIO_DEBUG_PIN_CONFIG(PIN_NO)                 \
    do                                                \
    {                                                 \
        GPIO_PIN_CONFIG(PIN_NO,                       \
                        GPIO_PIN_CNF_DIR_Output,      \
                        GPIO_PIN_CNF_INPUT_Disconnect,\
                        GPIO_PIN_CNF_PULL_Disabled,   \
                        GPIO_PIN_CNF_DRIVE_S0S1,      \
                        GPIO_PIN_CNF_SENSE_Disabled); \
    } while (0)

/**@brief Configure a PIN to be used as a Button.
 *
 * @param[in]  PIN_NO  Pin to configure.
 */
#define GPIO_BUTTON_CONFIG(PIN_NO)                    \
    do                                                \
    {                                                 \
        GPIO_PIN_CONFIG(PIN_NO,                       \
                        GPIO_PIN_CNF_DIR_Input,       \
                        GPIO_PIN_CNF_INPUT_Connect,   \
                        GPIO_PIN_CNF_PULL_Disabled,   \
                        GPIO_PIN_CNF_DRIVE_S0S1,      \
                        GPIO_PIN_CNF_SENSE_Disabled); \
    } while (0)

/**@brief Configure a PIN to be used as a Button and a Wakeup source.
 *
 * @param[in]  PIN_NO  Pin to configure.
 */
#define GPIO_WAKEUP_BUTTON_CONFIG(PIN_NO)             \
    do                                                \
    {                                                 \
        GPIO_PIN_CONFIG(PIN_NO,                       \
                        GPIO_PIN_CNF_DIR_Input,       \
                        GPIO_PIN_CNF_INPUT_Connect,   \
                        GPIO_PIN_CNF_PULL_Disabled,   \
                        GPIO_PIN_CNF_DRIVE_S0S1,      \
                        GPIO_PIN_CNF_SENSE_Low);      \
    } while (0)

/**@brief Configure a PIN to be used as a Button, with a pullup resistor enabled.
 *
 * @param[in]  PIN_NO  Pin to configure.
 */
#define GPIO_BUTTON_WITH_PULLUP_CONFIG(PIN_NO)        \
    do                                                \
    {                                                 \
        GPIO_PIN_CONFIG(PIN_NO,                       \
                        GPIO_PIN_CNF_DIR_Input,       \
                        GPIO_PIN_CNF_INPUT_Connect,   \
                        GPIO_PIN_CNF_PULL_Pullup,     \
                        GPIO_PIN_CNF_DRIVE_S0S1,      \
                        GPIO_PIN_CNF_SENSE_Disabled); \
    } while (0)

/**@brief Configure a PIN to be used as a Button and a Wakeup source, with a pullup resistor
 *        enabled.
 *
 * @param[in]  PIN_NO  Pin to configure.
 */
#define GPIO_WAKEUP_BUTTON_WITH_PULLUP_CONFIG(PIN_NO) \
    do                                                \
    {                                                 \
        GPIO_PIN_CONFIG(PIN_NO,                       \
                        GPIO_PIN_CNF_DIR_Input,       \
                        GPIO_PIN_CNF_INPUT_Connect,   \
                        GPIO_PIN_CNF_PULL_Pullup,     \
                        GPIO_PIN_CNF_DRIVE_S0S1,      \
                        GPIO_PIN_CNF_SENSE_Low);      \
    } while (0)

/**@brief Macro for entering a critical region.
 *
 * @note Due to implementation details, there must exist one and only one call to
 *       CRITICAL_REGION_EXIT() for each call to CRITICAL_REGION_ENTER(), and they must be located
 *       in the same scope.
 */
#define CRITICAL_REGION_ENTER()                                                             \
    {                                                                                       \
        uint8_t IS_NESTED_CRITICAL_REGION = 0;                                              \
        uint32_t CURRENT_INT_PRI = current_int_priority_get();                              \
        if (CURRENT_INT_PRI != APP_IRQ_PRIORITY_HIGH)                                       \
        {                                                                                   \
            uint32_t ERR_CODE = sd_nvic_critical_region_enter(&IS_NESTED_CRITICAL_REGION); \
            APP_ERROR_CHECK(ERR_CODE);                                                      \
        }        
    
/**@brief Macro for leaving a critical region.
 *
 * @note Due to implementation details, there must exist one and only one call to
 *       CRITICAL_REGION_EXIT() for each call to CRITICAL_REGION_ENTER(), and they must be located
 *       in the same scope.
 */
#define CRITICAL_REGION_EXIT()                                                              \
        if (CURRENT_INT_PRI != APP_IRQ_PRIORITY_HIGH)                                       \
        {                                                                                   \
            uint32_t ERR_CODE = sd_nvic_critical_region_exit(IS_NESTED_CRITICAL_REGION);   \
            APP_ERROR_CHECK(ERR_CODE);                                                      \
        }                                                                                   \
    }
    
/**@brief Perform rounded integer division (as opposed to truncating the result).
 *
 * @param[in]   A   Numerator.
 * @param[in]   B   Denominator.
 *
 * @return      Rounded (integer) result of dividing A by B.
 */
#define ROUNDED_DIV(A, B) (((A) + ((B) / 2)) / (B))

/**@brief Check if the integer provided is a power of two.
 *
 * @param[in]   A   Number to be tested.
 *
 * @return      true if value is power of two.
 * @return      false if value not power of two.
 */
#define IS_POWER_OF_TWO(A) ( ((A) != 0) && ((((A) - 1) & (A)) == 0) )

/**@brief Perform integer division, making sure the result is rounded up.
 *
 * @details One typical use for this is to compute the number of objects with size B is needed to
 *          hold A number of bytes.
 *
 * @param[in]   A   Numerator.
 * @param[in]   B   Denominator.
 *
 * @return      Integer result of dividing A by B, rounded up.
 */
#define CEIL_DIV(A, B)      \
    /*lint -save -e573 */   \
    ((((A) - 1) / (B)) + 1) \
    /*lint -restore */

/**@brief Inline function for encoding a uint16 value.
 *
 * @param[in]   value            Value to be encoded.
 * @param[out]  p_encoded_data   Buffer where the encoded data is to be written.
 *
 * @return      Number of bytes written.
 */
static __INLINE uint8_t uint16_encode(uint16_t value, uint8_t * p_encoded_data)
{
    p_encoded_data[0] = (uint8_t) ((value & 0x00FF) >> 0);
    p_encoded_data[1] = (uint8_t) ((value & 0xFF00) >> 8);
    return sizeof(uint16_t);
}
    
/**@brief Inline function for encoding a uint32 value.
 *
 * @param[in]   value            Value to be encoded.
 * @param[out]  p_encoded_data   Buffer where the encoded data is to be written.
 *
 * @return      Number of bytes written.
 */
static __INLINE uint8_t uint32_encode(uint32_t value, uint8_t * p_encoded_data)
{
    p_encoded_data[0] = (uint8_t) ((value & 0x000000FF) >> 0);
    p_encoded_data[1] = (uint8_t) ((value & 0x0000FF00) >> 8);
    p_encoded_data[2] = (uint8_t) ((value & 0x00FF0000) >> 16);
    p_encoded_data[3] = (uint8_t) ((value & 0xFF000000) >> 24);
    return sizeof(uint32_t);
}

/**@brief Inline function for decoding a uint16 value.
 *
 * @param[in]   p_encoded_data   Buffer where the encoded data is stored.
 *
 * @return      Decoded value.
 */
static __INLINE uint16_t uint16_decode(const uint8_t * p_encoded_data)
{
        return ( (((uint16_t)((uint8_t *)p_encoded_data)[0])) | 
                 (((uint16_t)((uint8_t *)p_encoded_data)[1]) << 8 ));
}

/**@brief Inline function for decoding a uint32 value.
 *
 * @param[in]   p_encoded_data   Buffer where the encoded data is stored.
 *
 * @return      Decoded value.
 */
static __INLINE uint32_t uint32_decode(const uint8_t * p_encoded_data)
{
    return ( (((uint32_t)((uint8_t *)p_encoded_data)[0]) << 0)  |
             (((uint32_t)((uint8_t *)p_encoded_data)[1]) << 8)  |
             (((uint32_t)((uint8_t *)p_encoded_data)[2]) << 16) |
             (((uint32_t)((uint8_t *)p_encoded_data)[3]) << 24 ));
}

    
/**@brief Inline function for finding the current interrupt level.
 *
 * @return   Current interrupt level.
 * @retval   APP_IRQ_PRIORITY_HIGH    We are running in Application High interrupt level.
 * @retval   APP_IRQ_PRIORITY_LOW     We are running in Application Low interrupt level.
 * @retval   APP_IRQ_PRIORITY_THREAD  We are running in Thread Mode.
 */
static __INLINE uint8_t current_int_priority_get(void)
{
    uint32_t isr_vector_num = (SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk);
    if (isr_vector_num > 0)
    {
        int32_t irq_type = ((int32_t)isr_vector_num - EXTERNAL_INT_VECTOR_OFFSET);
        return (NVIC_GetPriority((IRQn_Type)irq_type) & 0xFF);
    }
    else
    {
        return NRF_APP_PRIORITY_THREAD;
    }
}

/** @brief Converts the input voltage (in milli volts) into percentage of 3.0 Volts.
 *
 *  @details The calculation is based on a linearized version of the battery's discharge
 *           curve. 3.0V returns 100% battery level. The limit for power failure is 2.1V and
 *           is considered to be the lower boundary.
 *
 *           The discharge curve for CR2032 is non-linear. In this model it is split into
 *           4 linear sections:
 *           - Section 1: 3.0V - 2.9V = 100% - 42% (58% drop on 100 mV)
 *           - Section 2: 2.9V - 2.74V = 42% - 18% (24% drop on 160 mV)
 *           - Section 3: 2.74V - 2.44V = 18% - 6% (12% drop on 300 mV)
 *           - Section 4: 2.44V - 2.1V = 6% - 0% (6% drop on 340 mV)
 *
 *           These numbers are by no means accurate. Temperature and
 *           load in the actual application is not accounted for!
 *
 *  @param[in] mvolts The voltage in mV
 *
 *  @return    Battery level in percent.
*/
static __INLINE uint8_t battery_level_in_percent(const uint16_t mvolts)
{
    uint8_t battery_level;

    if (mvolts >= 3000)
    {
        battery_level = 100;
    }
    else if (mvolts > 2900)
    {
        battery_level = 100 - ((3000 - mvolts) * 58) / 100;
    }
    else if (mvolts > 2740)
    {
        battery_level = 42 - ((2900 - mvolts) * 24) / 160;
    }
    else if (mvolts > 2440)
    {
        battery_level = 18 - ((2740 - mvolts) * 12) / 300;
    }
    else if (mvolts > 2100)
    {
        battery_level = 6 - ((2440 - mvolts) * 6) / 340;
    }
    else
    {
        battery_level = 0;
    }

    return battery_level;
}

/**@brief Checks if a pointer value is aligned to a 4 byte boundary.
 *
 * @param[in]   p   Pointer value to be checked.
 *
 * @return      TRUE if pointer is aligned to a 4 byte boundary, FALSE otherwise.
 */
static __INLINE bool is_word_aligned(void * p)
{
    return (((uint32_t)p & 0x00000003) == 0);
}

#endif // APP_UTIL_H__

/** @} */
