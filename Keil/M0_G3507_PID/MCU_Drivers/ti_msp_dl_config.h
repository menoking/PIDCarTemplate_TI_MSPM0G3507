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



#define CPUCLK_FREQ                                                     32000000



/* Defines for PWM_0 */
#define PWM_0_INST                                                         TIMA0
#define PWM_0_INST_IRQHandler                                   TIMA0_IRQHandler
#define PWM_0_INST_INT_IRQN                                     (TIMA0_INT_IRQn)
#define PWM_0_INST_CLK_FREQ                                             16000000
/* GPIO defines for channel 0 */
#define GPIO_PWM_0_C0_PORT                                                 GPIOA
#define GPIO_PWM_0_C0_PIN                                         DL_GPIO_PIN_21
#define GPIO_PWM_0_C0_IOMUX                                      (IOMUX_PINCM46)
#define GPIO_PWM_0_C0_IOMUX_FUNC                     IOMUX_PINCM46_PF_TIMA0_CCP0
#define GPIO_PWM_0_C0_IDX                                    DL_TIMER_CC_0_INDEX
/* GPIO defines for channel 1 */
#define GPIO_PWM_0_C1_PORT                                                 GPIOA
#define GPIO_PWM_0_C1_PIN                                         DL_GPIO_PIN_22
#define GPIO_PWM_0_C1_IOMUX                                      (IOMUX_PINCM47)
#define GPIO_PWM_0_C1_IOMUX_FUNC                     IOMUX_PINCM47_PF_TIMA0_CCP1
#define GPIO_PWM_0_C1_IDX                                    DL_TIMER_CC_1_INDEX



/* Defines for TIMER_0 */
#define TIMER_0_INST                                                     (TIMA1)
#define TIMER_0_INST_IRQHandler                                 TIMA1_IRQHandler
#define TIMER_0_INST_INT_IRQN                                   (TIMA1_INT_IRQn)
#define TIMER_0_INST_LOAD_VALUE                                            (15U)




/* Defines for I2C_0 */
#define I2C_0_INST                                                          I2C1
#define I2C_0_INST_IRQHandler                                    I2C1_IRQHandler
#define I2C_0_INST_INT_IRQN                                        I2C1_INT_IRQn
#define I2C_0_BUS_SPEED_HZ                                                100000
#define GPIO_I2C_0_SDA_PORT                                                GPIOA
#define GPIO_I2C_0_SDA_PIN                                        DL_GPIO_PIN_16
#define GPIO_I2C_0_IOMUX_SDA                                     (IOMUX_PINCM38)
#define GPIO_I2C_0_IOMUX_SDA_FUNC                      IOMUX_PINCM38_PF_I2C1_SDA
#define GPIO_I2C_0_SCL_PORT                                                GPIOA
#define GPIO_I2C_0_SCL_PIN                                        DL_GPIO_PIN_17
#define GPIO_I2C_0_IOMUX_SCL                                     (IOMUX_PINCM39)
#define GPIO_I2C_0_IOMUX_SCL_FUNC                      IOMUX_PINCM39_PF_I2C1_SCL


/* Defines for UART_0 */
#define UART_0_INST                                                        UART0
#define UART_0_INST_IRQHandler                                  UART0_IRQHandler
#define UART_0_INST_INT_IRQN                                      UART0_INT_IRQn
#define GPIO_UART_0_RX_PORT                                                GPIOA
#define GPIO_UART_0_TX_PORT                                                GPIOA
#define GPIO_UART_0_RX_PIN                                        DL_GPIO_PIN_11
#define GPIO_UART_0_TX_PIN                                        DL_GPIO_PIN_10
#define GPIO_UART_0_IOMUX_RX                                     (IOMUX_PINCM22)
#define GPIO_UART_0_IOMUX_TX                                     (IOMUX_PINCM21)
#define GPIO_UART_0_IOMUX_RX_FUNC                      IOMUX_PINCM22_PF_UART0_RX
#define GPIO_UART_0_IOMUX_TX_FUNC                      IOMUX_PINCM21_PF_UART0_TX
#define UART_0_BAUD_RATE                                                  (9600)
#define UART_0_IBRD_32_MHZ_9600_BAUD                                       (208)
#define UART_0_FBRD_32_MHZ_9600_BAUD                                        (21)





/* Port definition for Pin Group BEEP */
#define BEEP_PORT                                                        (GPIOB)

/* Defines for PIN_0: GPIOB.17 with pinCMx 43 on package pin 36 */
#define BEEP_PIN_0_PIN                                          (DL_GPIO_PIN_17)
#define BEEP_PIN_0_IOMUX                                         (IOMUX_PINCM43)
/* Port definition for Pin Group LED */
#define LED_PORT                                                         (GPIOA)

/* Defines for LED1: GPIOA.4 with pinCMx 9 on package pin 10 */
#define LED_LED1_PIN                                             (DL_GPIO_PIN_4)
#define LED_LED1_IOMUX                                            (IOMUX_PINCM9)
/* Defines for LED2: GPIOA.3 with pinCMx 8 on package pin 9 */
#define LED_LED2_PIN                                             (DL_GPIO_PIN_3)
#define LED_LED2_IOMUX                                            (IOMUX_PINCM8)
/* Defines for LED3: GPIOA.2 with pinCMx 7 on package pin 8 */
#define LED_LED3_PIN                                             (DL_GPIO_PIN_2)
#define LED_LED3_IOMUX                                            (IOMUX_PINCM7)
/* Port definition for Pin Group KEY */
#define KEY_PORT                                                         (GPIOA)

/* Defines for KEY1: GPIOA.13 with pinCMx 35 on package pin 28 */
#define KEY_KEY1_PIN                                            (DL_GPIO_PIN_13)
#define KEY_KEY1_IOMUX                                           (IOMUX_PINCM35)
/* Defines for KEY2: GPIOA.14 with pinCMx 36 on package pin 29 */
#define KEY_KEY2_PIN                                            (DL_GPIO_PIN_14)
#define KEY_KEY2_IOMUX                                           (IOMUX_PINCM36)
/* Defines for KEY3: GPIOA.18 with pinCMx 40 on package pin 33 */
#define KEY_KEY3_PIN                                            (DL_GPIO_PIN_18)
#define KEY_KEY3_IOMUX                                           (IOMUX_PINCM40)
/* Port definition for Pin Group OLED */
#define OLED_PORT                                                        (GPIOA)

/* Defines for SCL: GPIOA.24 with pinCMx 54 on package pin 44 */
#define OLED_SCL_PIN                                            (DL_GPIO_PIN_24)
#define OLED_SCL_IOMUX                                           (IOMUX_PINCM54)
/* Defines for SDA: GPIOA.23 with pinCMx 53 on package pin 43 */
#define OLED_SDA_PIN                                            (DL_GPIO_PIN_23)
#define OLED_SDA_IOMUX                                           (IOMUX_PINCM53)
/* Port definition for Pin Group Encoder */
#define Encoder_PORT                                                     (GPIOB)

/* Defines for A: GPIOB.6 with pinCMx 23 on package pin 20 */
// pins affected by this interrupt request:["A","B","C","D"]
#define Encoder_INT_IRQN                                        (GPIOB_INT_IRQn)
#define Encoder_INT_IIDX                        (DL_INTERRUPT_GROUP1_IIDX_GPIOB)
#define Encoder_A_IIDX                                       (DL_GPIO_IIDX_DIO6)
#define Encoder_A_PIN                                            (DL_GPIO_PIN_6)
#define Encoder_A_IOMUX                                          (IOMUX_PINCM23)
/* Defines for B: GPIOB.7 with pinCMx 24 on package pin 21 */
#define Encoder_B_IIDX                                       (DL_GPIO_IIDX_DIO7)
#define Encoder_B_PIN                                            (DL_GPIO_PIN_7)
#define Encoder_B_IOMUX                                          (IOMUX_PINCM24)
/* Defines for C: GPIOB.8 with pinCMx 25 on package pin 22 */
#define Encoder_C_IIDX                                       (DL_GPIO_IIDX_DIO8)
#define Encoder_C_PIN                                            (DL_GPIO_PIN_8)
#define Encoder_C_IOMUX                                          (IOMUX_PINCM25)
/* Defines for D: GPIOB.9 with pinCMx 26 on package pin 23 */
#define Encoder_D_IIDX                                       (DL_GPIO_IIDX_DIO9)
#define Encoder_D_PIN                                            (DL_GPIO_PIN_9)
#define Encoder_D_IOMUX                                          (IOMUX_PINCM26)
/* Defines for AIN1: GPIOA.7 with pinCMx 14 on package pin 13 */
#define Motor_Ctrl_AIN1_PORT                                             (GPIOA)
#define Motor_Ctrl_AIN1_PIN                                      (DL_GPIO_PIN_7)
#define Motor_Ctrl_AIN1_IOMUX                                    (IOMUX_PINCM14)
/* Defines for AIN2: GPIOB.2 with pinCMx 15 on package pin 14 */
#define Motor_Ctrl_AIN2_PORT                                             (GPIOB)
#define Motor_Ctrl_AIN2_PIN                                      (DL_GPIO_PIN_2)
#define Motor_Ctrl_AIN2_IOMUX                                    (IOMUX_PINCM15)
/* Defines for BIN1: GPIOB.3 with pinCMx 16 on package pin 15 */
#define Motor_Ctrl_BIN1_PORT                                             (GPIOB)
#define Motor_Ctrl_BIN1_PIN                                      (DL_GPIO_PIN_3)
#define Motor_Ctrl_BIN1_IOMUX                                    (IOMUX_PINCM16)
/* Defines for BIN2: GPIOB.14 with pinCMx 31 on package pin 24 */
#define Motor_Ctrl_BIN2_PORT                                             (GPIOB)
#define Motor_Ctrl_BIN2_PIN                                     (DL_GPIO_PIN_14)
#define Motor_Ctrl_BIN2_IOMUX                                    (IOMUX_PINCM31)
/* Defines for IN1: GPIOB.24 with pinCMx 52 on package pin 42 */
#define Huidu_IN1_PORT                                                   (GPIOB)
#define Huidu_IN1_PIN                                           (DL_GPIO_PIN_24)
#define Huidu_IN1_IOMUX                                          (IOMUX_PINCM52)
/* Defines for IN2: GPIOB.20 with pinCMx 48 on package pin 41 */
#define Huidu_IN2_PORT                                                   (GPIOB)
#define Huidu_IN2_PIN                                           (DL_GPIO_PIN_20)
#define Huidu_IN2_IOMUX                                          (IOMUX_PINCM48)
/* Defines for IN3: GPIOA.27 with pinCMx 60 on package pin 47 */
#define Huidu_IN3_PORT                                                   (GPIOA)
#define Huidu_IN3_PIN                                           (DL_GPIO_PIN_27)
#define Huidu_IN3_IOMUX                                          (IOMUX_PINCM60)
/* Defines for IN4: GPIOB.19 with pinCMx 45 on package pin 38 */
#define Huidu_IN4_PORT                                                   (GPIOB)
#define Huidu_IN4_PIN                                           (DL_GPIO_PIN_19)
#define Huidu_IN4_IOMUX                                          (IOMUX_PINCM45)
/* Defines for IN5: GPIOB.18 with pinCMx 44 on package pin 37 */
#define Huidu_IN5_PORT                                                   (GPIOB)
#define Huidu_IN5_PIN                                           (DL_GPIO_PIN_18)
#define Huidu_IN5_IOMUX                                          (IOMUX_PINCM44)
/* Defines for IN6: GPIOA.12 with pinCMx 34 on package pin 27 */
#define Huidu_IN6_PORT                                                   (GPIOA)
#define Huidu_IN6_PIN                                           (DL_GPIO_PIN_12)
#define Huidu_IN6_IOMUX                                          (IOMUX_PINCM34)
/* Defines for IN7: GPIOB.16 with pinCMx 33 on package pin 26 */
#define Huidu_IN7_PORT                                                   (GPIOB)
#define Huidu_IN7_PIN                                           (DL_GPIO_PIN_16)
#define Huidu_IN7_IOMUX                                          (IOMUX_PINCM33)

/* clang-format on */

void SYSCFG_DL_init(void);
void SYSCFG_DL_initPower(void);
void SYSCFG_DL_GPIO_init(void);
void SYSCFG_DL_SYSCTL_init(void);
void SYSCFG_DL_PWM_0_init(void);
void SYSCFG_DL_TIMER_0_init(void);
void SYSCFG_DL_I2C_0_init(void);
void SYSCFG_DL_UART_0_init(void);


bool SYSCFG_DL_saveConfiguration(void);
bool SYSCFG_DL_restoreConfiguration(void);

#ifdef __cplusplus
}
#endif

#endif /* ti_msp_dl_config_h */
