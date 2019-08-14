/*!
 * \file      board-config.h
 *
 * \brief     Board configuration
 *
 * \copyright Revised BSD License, see section \ref LICENSE.
 *
 * \code
 *                ______                              _
 *               / _____)             _              | |
 *              ( (____  _____ ____ _| |_ _____  ____| |__
 *               \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 *               _____) ) ____| | | || |_| ____( (___| | | |
 *              (______/|_____)_|_|_| \__)_____)\____)_| |_|
 *              (C)2013-2017 Semtech
 *
 *               ___ _____ _   ___ _  _____ ___  ___  ___ ___
 *              / __|_   _/_\ / __| |/ / __/ _ \| _ \/ __| __|
 *              \__ \ | |/ _ \ (__| ' <| _| (_) |   / (__| _|
 *              |___/ |_/_/ \_\___|_|\_\_| \___/|_|_\\___|___|
 *              embedded.connectivity.solutions===============
 *
 * \endcode
 *
 * \author    Miguel Luis ( Semtech )
 *
 * \author    Gregory Cristian ( Semtech )
 *
 * \author    Daniel Jaeckle ( STACKFORCE )
 *
 * \author    Johannes Bruder ( STACKFORCE )
 */
#ifndef __BOARD_CONFIG_H__
#define __BOARD_CONFIG_H__

/*!
 * Defines the time required for the TCXO to wakeup [ms].
 */

#define BOARD_TCXO_WAKEUP_TIME                      3

/*!
 * Board MCU pins definitions
 */
#define RADIO_RESET                                 PC_6

#define RADIO_MOSI                                  PB_15
#define RADIO_MISO                                  PB_14
#define RADIO_SCLK                                  PB_13

#define RADIO_NSS                                   PB_12
#define RADIO_BUSY                                  PC_7
#define RADIO_DIO_1                                 PC_8

#define RADIO_ANT_SWITCH_POWER                      PC_9
// #define RADIO_FREQ_SEL                              PA_1
// #define RADIO_XTAL_SEL                              PB_0
// #define RADIO_DEVICE_SEL                            PA_4

#define LED_1                                       PC_1
#define LED_2                                       PC_0

#define OSC_LSE_IN                                  PC_14
#define OSC_LSE_OUT                                 PC_15

#define OSC_HSE_IN                                  PH_0
#define OSC_HSE_OUT                                 PH_1

#define SWCLK                                       PA_14
#define SWDAT                                       PA_13

#define I2C_SCL                                     PB_8
#define I2C_SDA                                     PB_9

#define UART_TX                                     PA_9
#define UART_RX                                     PA_10

// Debug pins definition.
#define RADIO_DBG_PIN_TX                            NC
#define RADIO_DBG_PIN_RX                            NC

#define UBLOXM8_EXINT								PB_7
#define BATTERY_CTRL_SWITCH							PC_13
#define BATTERY_VOLTAGE_IN							PC_3
#define BATTERY_CURRENT_IN							PC_2
#define BATTERY_VOLTAGE_CHANNEL						ADC_CHANNEL_13
#define BATTERY_CURRENT_CHANNEL						ADC_CHANNEL_12
#define ADC_REFERENCE_CHANNEL						ADC_CHANNEL_17

#define GPS_PPS                                     PB_3
#define GPS_TX_READY                                PB_4
#define ACC_INT1                                    PB_5
#define ACC_INT2                                    PB_6

#endif // __BOARD_CONFIG_H__
