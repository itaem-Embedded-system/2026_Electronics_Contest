/*
 * Copyright (c) 2023, Texas Instruments Incorporated - http://www.ti.com
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ============ ti_msp_dl_config.h =============
 *  Configured MSPM0 DriverLib module declarations
 *
 *  DO NOT EDIT - This file is generated for the MSPM0G350X
 *  by the SysConfig tool.
 */
#ifndef ti_msp_dl_config_h
#define ti_msp_dl_config_h

#define CONFIG_MSPM0G350X
#define CONFIG_MSPM0G3507

#if defined(__ti_version__) || defined(__TI_COMPILER_VERSION__)
#define SYSCONFIG_WEAK __attribute__((weak))
#elif defined(__IAR_SYSTEMS_ICC__)
#define SYSCONFIG_WEAK __weak
#elif defined(__GNUC__)
#define SYSCONFIG_WEAK __attribute__((weak))
#endif

#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>
#include <ti/driverlib/m0p/dl_core.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 *  ======== SYSCFG_DL_init ========
 *  Perform all required MSP DL initialization
 *
 *  This function should be called once at a point before any use of
 *  MSP DL.
 */


/* clang-format off */

#define POWER_STARTUP_DELAY                                                (16)



#define CPUCLK_FREQ                                                     80000000
/* Defines for SYSPLL_ERR_01 Workaround */
/* Represent 1.000 as 1000 */
#define FLOAT_TO_INT_SCALE                                               (1000U)
#define FCC_EXPECTED_RATIO                                                  2500
#define FCC_UPPER_BOUND                       (FCC_EXPECTED_RATIO * (1 + 0.003))
#define FCC_LOWER_BOUND                       (FCC_EXPECTED_RATIO * (1 - 0.003))

bool SYSCFG_DL_SYSCTL_SYSPLL_init(void);


/* Defines for PWM_motor_L */
#define PWM_motor_L_INST                                                   TIMG6
#define PWM_motor_L_INST_IRQHandler                             TIMG6_IRQHandler
#define PWM_motor_L_INST_INT_IRQN                               (TIMG6_INT_IRQn)
#define PWM_motor_L_INST_CLK_FREQ                                       80000000
/* GPIO defines for channel 0 */
#define GPIO_PWM_motor_L_C0_PORT                                           GPIOB
#define GPIO_PWM_motor_L_C0_PIN                                   DL_GPIO_PIN_26
#define GPIO_PWM_motor_L_C0_IOMUX                                (IOMUX_PINCM57)
#define GPIO_PWM_motor_L_C0_IOMUX_FUNC               IOMUX_PINCM57_PF_TIMG6_CCP0
#define GPIO_PWM_motor_L_C0_IDX                              DL_TIMER_CC_0_INDEX
/* GPIO defines for channel 1 */
#define GPIO_PWM_motor_L_C1_PORT                                           GPIOB
#define GPIO_PWM_motor_L_C1_PIN                                   DL_GPIO_PIN_27
#define GPIO_PWM_motor_L_C1_IOMUX                                (IOMUX_PINCM58)
#define GPIO_PWM_motor_L_C1_IOMUX_FUNC               IOMUX_PINCM58_PF_TIMG6_CCP1
#define GPIO_PWM_motor_L_C1_IDX                              DL_TIMER_CC_1_INDEX

/* Defines for PWM_motor_R */
#define PWM_motor_R_INST                                                   TIMG8
#define PWM_motor_R_INST_IRQHandler                             TIMG8_IRQHandler
#define PWM_motor_R_INST_INT_IRQN                               (TIMG8_INT_IRQn)
#define PWM_motor_R_INST_CLK_FREQ                                       40000000
/* GPIO defines for channel 0 */
#define GPIO_PWM_motor_R_C0_PORT                                           GPIOB
#define GPIO_PWM_motor_R_C0_PIN                                   DL_GPIO_PIN_10
#define GPIO_PWM_motor_R_C0_IOMUX                                (IOMUX_PINCM27)
#define GPIO_PWM_motor_R_C0_IOMUX_FUNC               IOMUX_PINCM27_PF_TIMG8_CCP0
#define GPIO_PWM_motor_R_C0_IDX                              DL_TIMER_CC_0_INDEX
/* GPIO defines for channel 1 */
#define GPIO_PWM_motor_R_C1_PORT                                           GPIOB
#define GPIO_PWM_motor_R_C1_PIN                                   DL_GPIO_PIN_11
#define GPIO_PWM_motor_R_C1_IOMUX                                (IOMUX_PINCM28)
#define GPIO_PWM_motor_R_C1_IOMUX_FUNC               IOMUX_PINCM28_PF_TIMG8_CCP1
#define GPIO_PWM_motor_R_C1_IDX                              DL_TIMER_CC_1_INDEX



/* Defines for TIMER_STATS */
#define TIMER_STATS_INST                                                (TIMG12)
#define TIMER_STATS_INST_IRQHandler                            TIMG12_IRQHandler
#define TIMER_STATS_INST_INT_IRQN                              (TIMG12_INT_IRQn)
#define TIMER_STATS_INST_LOAD_VALUE                                (1073739999U)
/* Defines for TIMER_Ultrasonic */
#define TIMER_Ultrasonic_INST                                            (TIMA0)
#define TIMER_Ultrasonic_INST_IRQHandler                        TIMA0_IRQHandler
#define TIMER_Ultrasonic_INST_INT_IRQN                          (TIMA0_INT_IRQn)
#define TIMER_Ultrasonic_INST_LOAD_VALUE                                (62499U)
/* Defines for IMU_dt */
#define IMU_dt_INST                                                      (TIMG7)
#define IMU_dt_INST_IRQHandler                                  TIMG7_IRQHandler
#define IMU_dt_INST_INT_IRQN                                    (TIMG7_INT_IRQn)
#define IMU_dt_INST_LOAD_VALUE                                          (49999U)



/* Defines for UART_1 */
#define UART_1_INST                                                        UART1
#define UART_1_INST_FREQUENCY                                           40000000
#define UART_1_INST_IRQHandler                                  UART1_IRQHandler
#define UART_1_INST_INT_IRQN                                      UART1_INT_IRQn
#define GPIO_UART_1_RX_PORT                                                GPIOB
#define GPIO_UART_1_TX_PORT                                                GPIOB
#define GPIO_UART_1_RX_PIN                                         DL_GPIO_PIN_5
#define GPIO_UART_1_TX_PIN                                         DL_GPIO_PIN_4
#define GPIO_UART_1_IOMUX_RX                                     (IOMUX_PINCM18)
#define GPIO_UART_1_IOMUX_TX                                     (IOMUX_PINCM17)
#define GPIO_UART_1_IOMUX_RX_FUNC                      IOMUX_PINCM18_PF_UART1_RX
#define GPIO_UART_1_IOMUX_TX_FUNC                      IOMUX_PINCM17_PF_UART1_TX
#define UART_1_BAUD_RATE                                                (115200)
#define UART_1_IBRD_40_MHZ_115200_BAUD                                      (21)
#define UART_1_FBRD_40_MHZ_115200_BAUD                                      (45)
/* Defines for UART_2 */
#define UART_2_INST                                                        UART2
#define UART_2_INST_FREQUENCY                                           40000000
#define UART_2_INST_IRQHandler                                  UART2_IRQHandler
#define UART_2_INST_INT_IRQN                                      UART2_INT_IRQn
#define GPIO_UART_2_RX_PORT                                                GPIOB
#define GPIO_UART_2_TX_PORT                                                GPIOB
#define GPIO_UART_2_RX_PIN                                        DL_GPIO_PIN_16
#define GPIO_UART_2_TX_PIN                                        DL_GPIO_PIN_15
#define GPIO_UART_2_IOMUX_RX                                     (IOMUX_PINCM33)
#define GPIO_UART_2_IOMUX_TX                                     (IOMUX_PINCM32)
#define GPIO_UART_2_IOMUX_RX_FUNC                      IOMUX_PINCM33_PF_UART2_RX
#define GPIO_UART_2_IOMUX_TX_FUNC                      IOMUX_PINCM32_PF_UART2_TX
#define UART_2_BAUD_RATE                                                (115200)
#define UART_2_IBRD_40_MHZ_115200_BAUD                                      (21)
#define UART_2_FBRD_40_MHZ_115200_BAUD                                      (45)
/* Defines for UART_3 */
#define UART_3_INST                                                        UART3
#define UART_3_INST_FREQUENCY                                           80000000
#define UART_3_INST_IRQHandler                                  UART3_IRQHandler
#define UART_3_INST_INT_IRQN                                      UART3_INT_IRQn
#define GPIO_UART_3_RX_PORT                                                GPIOA
#define GPIO_UART_3_TX_PORT                                                GPIOA
#define GPIO_UART_3_RX_PIN                                        DL_GPIO_PIN_13
#define GPIO_UART_3_TX_PIN                                        DL_GPIO_PIN_14
#define GPIO_UART_3_IOMUX_RX                                     (IOMUX_PINCM35)
#define GPIO_UART_3_IOMUX_TX                                     (IOMUX_PINCM36)
#define GPIO_UART_3_IOMUX_RX_FUNC                      IOMUX_PINCM35_PF_UART3_RX
#define GPIO_UART_3_IOMUX_TX_FUNC                      IOMUX_PINCM36_PF_UART3_TX
#define UART_3_BAUD_RATE                                                (115200)
#define UART_3_IBRD_80_MHZ_115200_BAUD                                      (43)
#define UART_3_FBRD_80_MHZ_115200_BAUD                                      (26)
/* Defines for UART_ZDT */
#define UART_ZDT_INST                                                      UART0
#define UART_ZDT_INST_FREQUENCY                                         40000000
#define UART_ZDT_INST_IRQHandler                                UART0_IRQHandler
#define UART_ZDT_INST_INT_IRQN                                    UART0_INT_IRQn
#define GPIO_UART_ZDT_RX_PORT                                              GPIOB
#define GPIO_UART_ZDT_TX_PORT                                              GPIOA
#define GPIO_UART_ZDT_RX_PIN                                       DL_GPIO_PIN_1
#define GPIO_UART_ZDT_TX_PIN                                      DL_GPIO_PIN_10
#define GPIO_UART_ZDT_IOMUX_RX                                   (IOMUX_PINCM13)
#define GPIO_UART_ZDT_IOMUX_TX                                   (IOMUX_PINCM21)
#define GPIO_UART_ZDT_IOMUX_RX_FUNC                    IOMUX_PINCM13_PF_UART0_RX
#define GPIO_UART_ZDT_IOMUX_TX_FUNC                    IOMUX_PINCM21_PF_UART0_TX
#define UART_ZDT_BAUD_RATE                                              (115200)
#define UART_ZDT_IBRD_40_MHZ_115200_BAUD                                    (21)
#define UART_ZDT_FBRD_40_MHZ_115200_BAUD                                    (45)




/* Defines for SPI_0 */
#define SPI_0_INST                                                         SPI1
#define SPI_0_INST_IRQHandler                                   SPI1_IRQHandler
#define SPI_0_INST_INT_IRQN                                       SPI1_INT_IRQn
#define GPIO_SPI_0_PICO_PORT                                              GPIOB
#define GPIO_SPI_0_PICO_PIN                                       DL_GPIO_PIN_8
#define GPIO_SPI_0_IOMUX_PICO                                   (IOMUX_PINCM25)
#define GPIO_SPI_0_IOMUX_PICO_FUNC                   IOMUX_PINCM25_PF_SPI1_PICO
#define GPIO_SPI_0_POCI_PORT                                              GPIOB
#define GPIO_SPI_0_POCI_PIN                                       DL_GPIO_PIN_7
#define GPIO_SPI_0_IOMUX_POCI                                   (IOMUX_PINCM24)
#define GPIO_SPI_0_IOMUX_POCI_FUNC                   IOMUX_PINCM24_PF_SPI1_POCI
/* GPIO configuration for SPI_0 */
#define GPIO_SPI_0_SCLK_PORT                                              GPIOB
#define GPIO_SPI_0_SCLK_PIN                                       DL_GPIO_PIN_9
#define GPIO_SPI_0_IOMUX_SCLK                                   (IOMUX_PINCM26)
#define GPIO_SPI_0_IOMUX_SCLK_FUNC                   IOMUX_PINCM26_PF_SPI1_SCLK



/* Port definition for Pin Group GPIO_KEY */
#define GPIO_KEY_PORT                                                    (GPIOB)

/* Defines for S4: GPIOB.2 with pinCMx 15 on package pin 50 */
#define GPIO_KEY_S4_PIN                                          (DL_GPIO_PIN_2)
#define GPIO_KEY_S4_IOMUX                                        (IOMUX_PINCM15)
/* Port definition for Pin Group GPIO_LED */
#define GPIO_LED_PORT                                                    (GPIOB)

/* Defines for Freertos_LED: GPIOB.22 with pinCMx 50 on package pin 21 */
#define GPIO_LED_Freertos_LED_PIN                               (DL_GPIO_PIN_22)
#define GPIO_LED_Freertos_LED_IOMUX                              (IOMUX_PINCM50)
/* Port definition for Pin Group GPIO_BEEP */
#define GPIO_BEEP_PORT                                                   (GPIOA)

/* Defines for PIN_0: GPIOA.12 with pinCMx 34 on package pin 5 */
#define GPIO_BEEP_PIN_0_PIN                                     (DL_GPIO_PIN_12)
#define GPIO_BEEP_PIN_0_IOMUX                                    (IOMUX_PINCM34)
/* Port definition for Pin Group GPIO_IMU */
#define GPIO_IMU_PORT                                                    (GPIOB)

/* Defines for CS: GPIOB.6 with pinCMx 23 on package pin 58 */
#define GPIO_IMU_CS_PIN                                          (DL_GPIO_PIN_6)
#define GPIO_IMU_CS_IOMUX                                        (IOMUX_PINCM23)
/* Defines for INT1: GPIOB.0 with pinCMx 12 on package pin 47 */
// groups represented: ["ENCODER_A","GPIO_Ultrasonic","GPIO_IMU"]
// pins affected: ["LA","PIN_Echo","INT1"]
#define GPIO_MULTIPLE_GPIOB_INT_IRQN                            (GPIOB_INT_IRQn)
#define GPIO_MULTIPLE_GPIOB_INT_IIDX            (DL_INTERRUPT_GROUP1_IIDX_GPIOB)
#define GPIO_IMU_INT1_IIDX                                   (DL_GPIO_IIDX_DIO0)
#define GPIO_IMU_INT1_PIN                                        (DL_GPIO_PIN_0)
#define GPIO_IMU_INT1_IOMUX                                      (IOMUX_PINCM12)
/* Defines for LA: GPIOB.13 with pinCMx 30 on package pin 1 */
#define ENCODER_A_LA_PORT                                                (GPIOB)
#define ENCODER_A_LA_IIDX                                   (DL_GPIO_IIDX_DIO13)
#define ENCODER_A_LA_PIN                                        (DL_GPIO_PIN_13)
#define ENCODER_A_LA_IOMUX                                       (IOMUX_PINCM30)
/* Defines for RA: GPIOA.9 with pinCMx 20 on package pin 55 */
#define ENCODER_A_RA_PORT                                                (GPIOA)
// pins affected by this interrupt request:["RA"]
#define ENCODER_A_GPIOA_INT_IRQN                                (GPIOA_INT_IRQn)
#define ENCODER_A_GPIOA_INT_IIDX                (DL_INTERRUPT_GROUP1_IIDX_GPIOA)
#define ENCODER_A_RA_IIDX                                    (DL_GPIO_IIDX_DIO9)
#define ENCODER_A_RA_PIN                                         (DL_GPIO_PIN_9)
#define ENCODER_A_RA_IOMUX                                       (IOMUX_PINCM20)
/* Defines for LB: GPIOB.12 with pinCMx 29 on package pin 64 */
#define ENCODER_B_LB_PORT                                                (GPIOB)
#define ENCODER_B_LB_PIN                                        (DL_GPIO_PIN_12)
#define ENCODER_B_LB_IOMUX                                       (IOMUX_PINCM29)
/* Defines for RB: GPIOA.8 with pinCMx 19 on package pin 54 */
#define ENCODER_B_RB_PORT                                                (GPIOA)
#define ENCODER_B_RB_PIN                                         (DL_GPIO_PIN_8)
#define ENCODER_B_RB_IOMUX                                       (IOMUX_PINCM19)
/* Port definition for Pin Group GPIO_Ultrasonic */
#define GPIO_Ultrasonic_PORT                                             (GPIOB)

/* Defines for PIN_Trig: GPIOB.3 with pinCMx 16 on package pin 51 */
#define GPIO_Ultrasonic_PIN_Trig_PIN                             (DL_GPIO_PIN_3)
#define GPIO_Ultrasonic_PIN_Trig_IOMUX                           (IOMUX_PINCM16)
/* Defines for PIN_Echo: GPIOB.23 with pinCMx 51 on package pin 22 */
#define GPIO_Ultrasonic_PIN_Echo_IIDX                       (DL_GPIO_IIDX_DIO23)
#define GPIO_Ultrasonic_PIN_Echo_PIN                            (DL_GPIO_PIN_23)
#define GPIO_Ultrasonic_PIN_Echo_IOMUX                           (IOMUX_PINCM51)
/* Defines for OUT1: GPIOA.27 with pinCMx 60 on package pin 31 */
#define GPIO_GRAY_OUT1_PORT                                              (GPIOA)
#define GPIO_GRAY_OUT1_PIN                                      (DL_GPIO_PIN_27)
#define GPIO_GRAY_OUT1_IOMUX                                     (IOMUX_PINCM60)
/* Defines for OUT2: GPIOA.26 with pinCMx 59 on package pin 30 */
#define GPIO_GRAY_OUT2_PORT                                              (GPIOA)
#define GPIO_GRAY_OUT2_PIN                                      (DL_GPIO_PIN_26)
#define GPIO_GRAY_OUT2_IOMUX                                     (IOMUX_PINCM59)
/* Defines for OUT3: GPIOA.25 with pinCMx 55 on package pin 26 */
#define GPIO_GRAY_OUT3_PORT                                              (GPIOA)
#define GPIO_GRAY_OUT3_PIN                                      (DL_GPIO_PIN_25)
#define GPIO_GRAY_OUT3_IOMUX                                     (IOMUX_PINCM55)
/* Defines for OUT4: GPIOA.24 with pinCMx 54 on package pin 25 */
#define GPIO_GRAY_OUT4_PORT                                              (GPIOA)
#define GPIO_GRAY_OUT4_PIN                                      (DL_GPIO_PIN_24)
#define GPIO_GRAY_OUT4_IOMUX                                     (IOMUX_PINCM54)
/* Defines for OUT5: GPIOB.25 with pinCMx 56 on package pin 27 */
#define GPIO_GRAY_OUT5_PORT                                              (GPIOB)
#define GPIO_GRAY_OUT5_PIN                                      (DL_GPIO_PIN_25)
#define GPIO_GRAY_OUT5_IOMUX                                     (IOMUX_PINCM56)
/* Defines for OUT6: GPIOB.24 with pinCMx 52 on package pin 23 */
#define GPIO_GRAY_OUT6_PORT                                              (GPIOB)
#define GPIO_GRAY_OUT6_PIN                                      (DL_GPIO_PIN_24)
#define GPIO_GRAY_OUT6_IOMUX                                     (IOMUX_PINCM52)
/* Defines for OUT7: GPIOB.20 with pinCMx 48 on package pin 19 */
#define GPIO_GRAY_OUT7_PORT                                              (GPIOB)
#define GPIO_GRAY_OUT7_PIN                                      (DL_GPIO_PIN_20)
#define GPIO_GRAY_OUT7_IOMUX                                     (IOMUX_PINCM48)
/* Port definition for Pin Group GPIO_OLED */
#define GPIO_OLED_PORT                                                   (GPIOA)

/* Defines for SDA: GPIOA.0 with pinCMx 1 on package pin 33 */
#define GPIO_OLED_SDA_PIN                                        (DL_GPIO_PIN_0)
#define GPIO_OLED_SDA_IOMUX                                       (IOMUX_PINCM1)
/* Defines for SCL: GPIOA.1 with pinCMx 2 on package pin 34 */
#define GPIO_OLED_SCL_PIN                                        (DL_GPIO_PIN_1)
#define GPIO_OLED_SCL_IOMUX                                       (IOMUX_PINCM2)
/* Port definition for Pin Group GPIO_HC05 */
#define GPIO_HC05_PORT                                                   (GPIOA)

/* Defines for HC05_EN: GPIOA.15 with pinCMx 37 on package pin 8 */
#define GPIO_HC05_HC05_EN_PIN                                   (DL_GPIO_PIN_15)
#define GPIO_HC05_HC05_EN_IOMUX                                  (IOMUX_PINCM37)
/* Defines for HC05_STATE: GPIOA.16 with pinCMx 38 on package pin 9 */
#define GPIO_HC05_HC05_STATE_PIN                                (DL_GPIO_PIN_16)
#define GPIO_HC05_HC05_STATE_IOMUX                               (IOMUX_PINCM38)


/* clang-format on */

void SYSCFG_DL_init(void);
void SYSCFG_DL_initPower(void);
void SYSCFG_DL_GPIO_init(void);
void SYSCFG_DL_SYSCTL_init(void);

bool SYSCFG_DL_SYSCTL_SYSPLL_init(void);
void SYSCFG_DL_PWM_motor_L_init(void);
void SYSCFG_DL_PWM_motor_R_init(void);
void SYSCFG_DL_TIMER_STATS_init(void);
void SYSCFG_DL_TIMER_Ultrasonic_init(void);
void SYSCFG_DL_IMU_dt_init(void);
void SYSCFG_DL_UART_1_init(void);
void SYSCFG_DL_UART_2_init(void);
void SYSCFG_DL_UART_3_init(void);
void SYSCFG_DL_UART_ZDT_init(void);
void SYSCFG_DL_SPI_0_init(void);


bool SYSCFG_DL_saveConfiguration(void);
bool SYSCFG_DL_restoreConfiguration(void);

#ifdef __cplusplus
}
#endif

#endif /* ti_msp_dl_config_h */
