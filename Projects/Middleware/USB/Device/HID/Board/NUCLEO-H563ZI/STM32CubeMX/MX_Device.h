/******************************************************************************
 * File Name   : MX_Device.h
 * Date        : 13/07/2023 14:23:24
 * Description : STM32Cube MX parameter definitions
 * Note        : This file is generated by STM32CubeMX (DO NOT EDIT!)
 ******************************************************************************/

#ifndef __MX_DEVICE_H
#define __MX_DEVICE_H

/*---------------------------- Clock Configuration ---------------------------*/

#define MX_LSI_VALUE                                32000
#define MX_LSE_VALUE                                32768
#define MX_HSI_VALUE                                64000000
#define MX_HSICalibrationValue                      64
#define MX_HSE_VALUE                                8000000
#define MX_SYSCLKFreq_VALUE                         250000000
#define MX_HCLKFreq_Value                           250000000
#define MX_FCLKCortexFreq_Value                     250000000
#define MX_CortexFreq_Value                         250000000
#define MX_AHBFreq_Value                            250000000
#define MX_APB1Freq_Value                           250000000
#define MX_APB2Freq_Value                           250000000
#define MX_APB1TimFreq_Value                        250000000
#define MX_APB2TimFreq_Value                        250000000
#define MX_ADCFreq_Value                            250000000
#define MX_I2C1Freq_Value                           250000000
#define MX_I2C2Freq_Value                           250000000
#define MX_I2C3Freq_Value                           250000000
#define MX_CSI_VALUE                                4000000
#define MX_SPI1Freq_Value                           250000000
#define MX_SPI2Freq_Value                           250000000
#define MX_SPI3Freq_Value                           250000000
#define MX_USBFreq_Value                            48000000
#define MX_WatchDogFreq_Value                       32000
#define MX_LPTIM1Freq_Value                         250000000
#define MX_LPTIM2Freq_Value                         250000000
#define MX_RTCFreq_Value                            32000
#define MX_PWRFreq_Value                            250000000
#define MX_MCO1PinFreq_Value                        64000000
#define MX_CRSFreq_Value                            48000000
#define MX_SWPMI1Freq_Value                         4000000
#define MX_SAI1Freq_Value                           258000000
#define MX_SAI2Freq_Value                           258000000
#define MX_LPUART1Freq_Value                        250000000
#define MX_UART4Freq_Value                          250000000
#define MX_UART5Freq_Value                          250000000
#define MX_USART1Freq_Value                         250000000
#define MX_USART2Freq_Value                         250000000
#define MX_USART3Freq_Value                         250000000

/*-------------------------------- ADC1       --------------------------------*/

#define MX_ADC1                                     1

/* GPIO Configuration */

/* Pin PA4 */
#define MX_ADCx_INP18_Pin                           PA4
#define MX_ADCx_INP18_GPIOx                         GPIOA
#define MX_ADCx_INP18_GPIO_PuPd                     GPIO_NOPULL
#define MX_ADCx_INP18_GPIO_Pin                      GPIO_PIN_4
#define MX_ADCx_INP18_GPIO_Mode                     GPIO_MODE_ANALOG

/*-------------------------------- CORTEX_M33_NS --------------------------------*/

#define MX_CORTEX_M33_NS                            1

/*-------------------------------- ETH        --------------------------------*/

#define MX_ETH                                      1

/* GPIO Configuration */

/* Pin PA1 */
#define MX_ETH_REF_CLK_GPIO_Speed                   GPIO_SPEED_FREQ_LOW
#define MX_ETH_REF_CLK_Pin                          PA1
#define MX_ETH_REF_CLK_GPIOx                        GPIOA
#define MX_ETH_REF_CLK_GPIO_PuPd                    GPIO_NOPULL
#define MX_ETH_REF_CLK_GPIO_Pin                     GPIO_PIN_1
#define MX_ETH_REF_CLK_GPIO_AF                      GPIO_AF11_ETH
#define MX_ETH_REF_CLK_GPIO_Mode                    GPIO_MODE_AF_PP

/* Pin PA7 */
#define MX_ETH_CRS_DV_GPIO_Speed                    GPIO_SPEED_FREQ_LOW
#define MX_ETH_CRS_DV_Pin                           PA7
#define MX_ETH_CRS_DV_GPIOx                         GPIOA
#define MX_ETH_CRS_DV_GPIO_PuPd                     GPIO_NOPULL
#define MX_ETH_CRS_DV_GPIO_Pin                      GPIO_PIN_7
#define MX_ETH_CRS_DV_GPIO_AF                       GPIO_AF11_ETH
#define MX_ETH_CRS_DV_GPIO_Mode                     GPIO_MODE_AF_PP

/* Pin PC4 */
#define MX_ETH_RXD0_GPIO_Speed                      GPIO_SPEED_FREQ_LOW
#define MX_ETH_RXD0_Pin                             PC4
#define MX_ETH_RXD0_GPIOx                           GPIOC
#define MX_ETH_RXD0_GPIO_PuPd                       GPIO_NOPULL
#define MX_ETH_RXD0_GPIO_Pin                        GPIO_PIN_4
#define MX_ETH_RXD0_GPIO_AF                         GPIO_AF11_ETH
#define MX_ETH_RXD0_GPIO_Mode                       GPIO_MODE_AF_PP

/* Pin PC5 */
#define MX_ETH_RXD1_GPIO_Speed                      GPIO_SPEED_FREQ_LOW
#define MX_ETH_RXD1_Pin                             PC5
#define MX_ETH_RXD1_GPIOx                           GPIOC
#define MX_ETH_RXD1_GPIO_PuPd                       GPIO_NOPULL
#define MX_ETH_RXD1_GPIO_Pin                        GPIO_PIN_5
#define MX_ETH_RXD1_GPIO_AF                         GPIO_AF11_ETH
#define MX_ETH_RXD1_GPIO_Mode                       GPIO_MODE_AF_PP

/* Pin PG11 */
#define MX_ETH_TX_EN_GPIO_Speed                     GPIO_SPEED_FREQ_LOW
#define MX_ETH_TX_EN_Pin                            PG11
#define MX_ETH_TX_EN_GPIOx                          GPIOG
#define MX_ETH_TX_EN_GPIO_PuPd                      GPIO_NOPULL
#define MX_ETH_TX_EN_GPIO_Pin                       GPIO_PIN_11
#define MX_ETH_TX_EN_GPIO_AF                        GPIO_AF11_ETH
#define MX_ETH_TX_EN_GPIO_Mode                      GPIO_MODE_AF_PP

/* Pin PA2 */
#define MX_ETH_MDIO_GPIO_Speed                      GPIO_SPEED_FREQ_LOW
#define MX_ETH_MDIO_Pin                             PA2
#define MX_ETH_MDIO_GPIOx                           GPIOA
#define MX_ETH_MDIO_GPIO_PuPd                       GPIO_NOPULL
#define MX_ETH_MDIO_GPIO_Pin                        GPIO_PIN_2
#define MX_ETH_MDIO_GPIO_AF                         GPIO_AF11_ETH
#define MX_ETH_MDIO_GPIO_Mode                       GPIO_MODE_AF_PP

/* Pin PB15 */
#define MX_ETH_TXD1_GPIO_Speed                      GPIO_SPEED_FREQ_LOW
#define MX_ETH_TXD1_Pin                             PB15
#define MX_ETH_TXD1_GPIOx                           GPIOB
#define MX_ETH_TXD1_GPIO_PuPd                       GPIO_NOPULL
#define MX_ETH_TXD1_GPIO_Pin                        GPIO_PIN_15
#define MX_ETH_TXD1_GPIO_AF                         GPIO_AF11_ETH
#define MX_ETH_TXD1_GPIO_Mode                       GPIO_MODE_AF_PP

/* Pin PG13 */
#define MX_ETH_TXD0_GPIO_Speed                      GPIO_SPEED_FREQ_LOW
#define MX_ETH_TXD0_Pin                             PG13
#define MX_ETH_TXD0_GPIOx                           GPIOG
#define MX_ETH_TXD0_GPIO_PuPd                       GPIO_NOPULL
#define MX_ETH_TXD0_GPIO_Pin                        GPIO_PIN_13
#define MX_ETH_TXD0_GPIO_AF                         GPIO_AF11_ETH
#define MX_ETH_TXD0_GPIO_Mode                       GPIO_MODE_AF_PP

/* Pin PC1 */
#define MX_ETH_MDC_GPIO_Speed                       GPIO_SPEED_FREQ_LOW
#define MX_ETH_MDC_Pin                              PC1
#define MX_ETH_MDC_GPIOx                            GPIOC
#define MX_ETH_MDC_GPIO_PuPd                        GPIO_NOPULL
#define MX_ETH_MDC_GPIO_Pin                         GPIO_PIN_1
#define MX_ETH_MDC_GPIO_AF                          GPIO_AF11_ETH
#define MX_ETH_MDC_GPIO_Mode                        GPIO_MODE_AF_PP

/*-------------------------------- ICACHE     --------------------------------*/

#define MX_ICACHE                                   1

/*-------------------------------- LPUART1    --------------------------------*/

#define MX_LPUART1                                  1

/* GPIO Configuration */

/* Pin PB7 */
#define MX_LPUART1_RX_GPIO_Speed                    GPIO_SPEED_FREQ_LOW
#define MX_LPUART1_RX_Pin                           PB7
#define MX_LPUART1_RX_GPIOx                         GPIOB
#define MX_LPUART1_RX_GPIO_PuPd                     GPIO_NOPULL
#define MX_LPUART1_RX_GPIO_Pin                      GPIO_PIN_7
#define MX_LPUART1_RX_GPIO_AF                       GPIO_AF8_LPUART1
#define MX_LPUART1_RX_GPIO_FM7                      __NULL
#define MX_LPUART1_RX_GPIO_Mode                     GPIO_MODE_AF_PP

/* Pin PB6 */
#define MX_LPUART1_TX_GPIO_Speed                    GPIO_SPEED_FREQ_LOW
#define MX_LPUART1_TX_GPIO_FM6                      __NULL
#define MX_LPUART1_TX_Pin                           PB6
#define MX_LPUART1_TX_GPIOx                         GPIOB
#define MX_LPUART1_TX_GPIO_PuPd                     GPIO_NOPULL
#define MX_LPUART1_TX_GPIO_Pin                      GPIO_PIN_6
#define MX_LPUART1_TX_GPIO_AF                       GPIO_AF8_LPUART1
#define MX_LPUART1_TX_GPIO_Mode                     GPIO_MODE_AF_PP

/*-------------------------------- USART3     --------------------------------*/

#define MX_USART3                                   1

#define MX_USART3_VM                                VM_ASYNC

/* GPIO Configuration */

/* Pin PD8 */
#define MX_USART3_TX_GPIO_ModeDefaultPP             GPIO_MODE_AF_PP
#define MX_USART3_TX_GPIO_Speed                     GPIO_SPEED_FREQ_LOW
#define MX_USART3_TX_Pin                            PD8
#define MX_USART3_TX_GPIOx                          GPIOD
#define MX_USART3_TX_GPIO_PuPd                      GPIO_NOPULL
#define MX_USART3_TX_GPIO_Pin                       GPIO_PIN_8
#define MX_USART3_TX_GPIO_AF                        GPIO_AF7_USART3

/* Pin PD9 */
#define MX_USART3_RX_GPIO_ModeDefaultPP             GPIO_MODE_AF_PP
#define MX_USART3_RX_GPIO_Speed                     GPIO_SPEED_FREQ_LOW
#define MX_USART3_RX_Pin                            PD9
#define MX_USART3_RX_GPIOx                          GPIOD
#define MX_USART3_RX_GPIO_PuPd                      GPIO_NOPULL
#define MX_USART3_RX_GPIO_Pin                       GPIO_PIN_9
#define MX_USART3_RX_GPIO_AF                        GPIO_AF7_USART3

/*-------------------------------- PWR        --------------------------------*/

#define MX_PWR                                      1

/*-------------------------------- SYS        --------------------------------*/

#define MX_SYS                                      1

/*-------------------------------- USB        --------------------------------*/

#define MX_USB                                      1

/* GPIO Configuration */

/* Pin PA11 */
#define MX_USB_DM_GPIO_Speed                        GPIO_SPEED_FREQ_HIGH
#define MX_USB_DM_Pin                               PA11
#define MX_USB_DM_GPIOx                             GPIOA
#define MX_USB_DM_GPIO_PuPd                         GPIO_NOPULL
#define MX_USB_DM_GPIO_Pin                          GPIO_PIN_11
#define MX_USB_DM_GPIO_AF                           GPIO_AF10_USB
#define MX_USB_DM_GPIO_Mode                         GPIO_MODE_AF_PP

/* Pin PA12 */
#define MX_USB_DP_GPIO_Speed                        GPIO_SPEED_FREQ_HIGH
#define MX_USB_DP_Pin                               PA12
#define MX_USB_DP_GPIOx                             GPIOA
#define MX_USB_DP_GPIO_PuPd                         GPIO_NOPULL
#define MX_USB_DP_GPIO_Pin                          GPIO_PIN_12
#define MX_USB_DP_GPIO_AF                           GPIO_AF10_USB
#define MX_USB_DP_GPIO_Mode                         GPIO_MODE_AF_PP

/* NVIC Configuration */

/* NVIC USB_DRD_FS_IRQn */
#define MX_USB_DRD_FS_IRQn_interruptPremptionPriority 0
#define MX_USB_DRD_FS_IRQn_PriorityGroup            NVIC_PRIORITYGROUP_4
#define MX_USB_DRD_FS_IRQn_Subriority               0

/*-------------------------------- NVIC       --------------------------------*/

#define MX_NVIC                                     1

/*-------------------------------- GPIO       --------------------------------*/

#define MX_GPIO                                     1

/* GPIO Configuration */

/* Pin PC13 - USER_BUTTON */
#define MX_PC13_Pin                                 PC13
#define MX_PC13_GPIOx                               GPIOC
#define MX_PC13_GPIO_PuPd                           GPIO_NOPULL
#define MX_PC13_GPIO_Pin                            GPIO_PIN_13
#define MX_PC13_GPIO_ModeDefaultEXTI                GPIO_MODE_IT_RISING
#define MX_PC13_Privilege                           EXTI_LINE_NPRIV
#define MX_PC13_EXTI_Line                           EXTI_LINE_13

#define MX_USER_BUTTON_Pin                          MX_PC13_Pin
#define MX_USER_BUTTON_GPIOx                        MX_PC13_GPIOx
#define MX_USER_BUTTON_GPIO_PuPd                    MX_PC13_GPIO_PuPd
#define MX_USER_BUTTON_GPIO_Pin                     MX_PC13_GPIO_Pin
#define MX_USER_BUTTON_GPIO_ModeDefaultEXTI         MX_PC13_GPIO_ModeDefaultEXTI
#define MX_USER_BUTTON_Privilege                    MX_PC13_Privilege
#define MX_USER_BUTTON_EXTI_Line                    MX_PC13_EXTI_Line

/* Pin PG7 - UCPD_FLT */
#define MX_PG7_Pin                                  PG7
#define MX_PG7_GPIOx                                GPIOG
#define MX_PG7_GPIO_PuPd                            GPIO_NOPULL
#define MX_PG7_GPIO_Pin                             GPIO_PIN_7
#define MX_PG7_GPIO_ModeDefaultEXTI                 GPIO_MODE_IT_RISING
#define MX_PG7_Privilege                            EXTI_LINE_NPRIV
#define MX_PG7_EXTI_Line                            EXTI_LINE_7

#define MX_UCPD_FLT_Pin                             MX_PG7_Pin
#define MX_UCPD_FLT_GPIOx                           MX_PG7_GPIOx
#define MX_UCPD_FLT_GPIO_PuPd                       MX_PG7_GPIO_PuPd
#define MX_UCPD_FLT_GPIO_Pin                        MX_PG7_GPIO_Pin
#define MX_UCPD_FLT_GPIO_ModeDefaultEXTI            MX_PG7_GPIO_ModeDefaultEXTI
#define MX_UCPD_FLT_Privilege                       MX_PG7_Privilege
#define MX_UCPD_FLT_EXTI_Line                       MX_PG7_EXTI_Line

/* Pin PG4 - LED3_RED */
#define MX_PG4_GPIO_Speed                           GPIO_SPEED_FREQ_LOW
#define MX_PG4_Pin                                  PG4
#define MX_PG4_GPIOx                                GPIOG
#define MX_PG4_PinState                             GPIO_PIN_RESET
#define MX_PG4_GPIO_PuPd                            GPIO_NOPULL
#define MX_PG4_GPIO_Pin                             GPIO_PIN_4
#define MX_PG4_GPIO_ModeDefaultOutputPP             GPIO_MODE_OUTPUT_PP

#define MX_LED3_RED_GPIO_Speed                      MX_PG4_GPIO_Speed
#define MX_LED3_RED_Pin                             MX_PG4_Pin
#define MX_LED3_RED_GPIOx                           MX_PG4_GPIOx
#define MX_LED3_RED_PinState                        MX_PG4_PinState
#define MX_LED3_RED_GPIO_PuPd                       MX_PG4_GPIO_PuPd
#define MX_LED3_RED_GPIO_Pin                        MX_PG4_GPIO_Pin
#define MX_LED3_RED_GPIO_ModeDefaultOutputPP        MX_PG4_GPIO_ModeDefaultOutputPP

/* Pin PF4 - LED2_YELLOW */
#define MX_PF4_GPIO_Speed                           GPIO_SPEED_FREQ_LOW
#define MX_PF4_Pin                                  PF4
#define MX_PF4_GPIOx                                GPIOF
#define MX_PF4_PinState                             GPIO_PIN_RESET
#define MX_PF4_GPIO_PuPd                            GPIO_NOPULL
#define MX_PF4_GPIO_Pin                             GPIO_PIN_4
#define MX_PF4_GPIO_ModeDefaultOutputPP             GPIO_MODE_OUTPUT_PP

#define MX_LED2_YELLOW_GPIO_Speed                   MX_PF4_GPIO_Speed
#define MX_LED2_YELLOW_Pin                          MX_PF4_Pin
#define MX_LED2_YELLOW_GPIOx                        MX_PF4_GPIOx
#define MX_LED2_YELLOW_PinState                     MX_PF4_PinState
#define MX_LED2_YELLOW_GPIO_PuPd                    MX_PF4_GPIO_PuPd
#define MX_LED2_YELLOW_GPIO_Pin                     MX_PF4_GPIO_Pin
#define MX_LED2_YELLOW_GPIO_ModeDefaultOutputPP     MX_PF4_GPIO_ModeDefaultOutputPP

/* Pin PB0 - LED1_GREEN */
#define MX_PB0_GPIO_Speed                           GPIO_SPEED_FREQ_LOW
#define MX_PB0_Pin                                  PB0
#define MX_PB0_GPIOx                                GPIOB
#define MX_PB0_PinState                             GPIO_PIN_RESET
#define MX_PB0_GPIO_PuPd                            GPIO_NOPULL
#define MX_PB0_GPIO_Pin                             GPIO_PIN_0
#define MX_PB0_GPIO_ModeDefaultOutputPP             GPIO_MODE_OUTPUT_PP

#define MX_LED1_GREEN_GPIO_Speed                    MX_PB0_GPIO_Speed
#define MX_LED1_GREEN_Pin                           MX_PB0_Pin
#define MX_LED1_GREEN_GPIOx                         MX_PB0_GPIOx
#define MX_LED1_GREEN_PinState                      MX_PB0_PinState
#define MX_LED1_GREEN_GPIO_PuPd                     MX_PB0_GPIO_PuPd
#define MX_LED1_GREEN_GPIO_Pin                      MX_PB0_GPIO_Pin
#define MX_LED1_GREEN_GPIO_ModeDefaultOutputPP      MX_PB0_GPIO_ModeDefaultOutputPP

#endif  /* __MX_DEVICE_H */

