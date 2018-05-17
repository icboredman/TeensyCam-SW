/***********************************************************************************************************************
 * This file was generated by the MCUXpresso Config Tools. Any manual edits made to this file
 * will be overwritten if the respective MCUXpresso Config Tools is used to update this file.
 **********************************************************************************************************************/

#ifndef _PIN_MUX_H_
#define _PIN_MUX_H_

/***********************************************************************************************************************
 * Definitions
 **********************************************************************************************************************/

/*! @brief Direction type  */
typedef enum _pin_mux_direction
{
    kPIN_MUX_DirectionInput = 0U,        /* Input direction */
    kPIN_MUX_DirectionOutput = 1U,       /* Output direction */
    kPIN_MUX_DirectionInputOrOutput = 2U /* Input or output direction */
} pin_mux_direction_t;

/*!
 * @addtogroup pin_mux
 * @{
 */

/***********************************************************************************************************************
 * API
 **********************************************************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @brief Calls initialization functions.
 *
 */
void BOARD_InitBootPins(void);

/*! @name PORTC5 (coord D8), J1[12]/I2S0_RXD0/FB_AD10/FTM0_CH2
  @{ */
#define BOARD_INITPINS_LED_PERIPHERAL GPIOC                          /*!<@brief Device name: GPIOC */
#define BOARD_INITPINS_LED_SIGNAL GPIO                               /*!<@brief GPIOC signal: GPIO */
#define BOARD_INITPINS_LED_GPIO GPIOC                                /*!<@brief GPIO device name: GPIOC */
#define BOARD_INITPINS_LED_GPIO_PIN 5U                               /*!<@brief PORTC pin index: 5 */
#define BOARD_INITPINS_LED_PORT PORTC                                /*!<@brief PORT device name: PORTC */
#define BOARD_INITPINS_LED_PIN 5U                                    /*!<@brief PORTC pin index: 5 */
#define BOARD_INITPINS_LED_CHANNEL 5                                 /*!<@brief GPIOC GPIO channel: 5 */
#define BOARD_INITPINS_LED_PIN_NAME PTC5                             /*!<@brief Pin name */
#define BOARD_INITPINS_LED_LABEL "J1[12]/I2S0_RXD0/FB_AD10/FTM0_CH2" /*!<@brief Label */
#define BOARD_INITPINS_LED_NAME "LED"                                /*!<@brief Identifier name */
#define BOARD_INITPINS_LED_DIRECTION kPIN_MUX_DirectionOutput        /*!<@brief Direction */
                                                                     /* @} */

/*!
 * @brief Configures pin routing and optionally pin electrical features.
 *
 */
void BOARD_InitPins(void);

#if defined(__cplusplus)
}
#endif

/*!
 * @}
 */
#endif /* _PIN_MUX_H_ */

/***********************************************************************************************************************
 * EOF
 **********************************************************************************************************************/