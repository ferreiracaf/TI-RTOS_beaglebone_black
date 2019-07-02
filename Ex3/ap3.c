/**
 *  \file   main_eeprom_read.c
 *
 *  \brief  Example application main file. This application will read the data
 *          from eeprom and compares it with the known data.
 *
 */

/*
 * Copyright (C) 2014 - 2017 Texas Instruments Incorporated - http://www.ti.com/
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the
 * distribution.
 *
 * Neither the name of Texas Instruments Incorporated nor the names of
 * its contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef BARE_METAL
/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/cfg/global.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Error.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#endif

#include <stdio.h>
#include <string.h>

/* TI-RTOS Header files */
#include <ti/drv/i2c/I2C.h>
#include <ti/drv/i2c/soc/I2C_soc.h>
#include <ti/drv/i2c/test/eeprom_read/src/I2C_log.h>
#include <ti/drv/i2c/test/eeprom_read/src/I2C_board.h>

#include <ti/drv/gpio/GPIO.h>
#include <ti/drv/gpio/soc/GPIO_soc.h>
#include <ti/csl/src/ip/gpio/V1/gpio_v2.h>

#include <ti/drv/gpio/test/led_blink/src/GPIO_log.h>
#include <ti/drv/gpio/test/led_blink/src/GPIO_board.h>

#include <ti/board/board.h>


#include <ti/sysbios/family/arm/a8/intcps/Hwi.h>
#include <ti/sysbios/knl/Swi.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ftoa.h>
#include <convert.h>
#include <math.h>
#include "mylib.h"


#if defined (SOC_AM335X) || defined (SOC_AM437x)
/* EEPROM data -Board specific */
extern char eepromData[I2C_EEPROM_RX_LENGTH];
#endif

/*
 *
 *
 *  O TICK FOI DEFINIDO NO .cfg COMO 10000 (10ms)
 *
 *
 * */

/**********************************************************************
 ************************** Macros ************************************
 **********************************************************************/

#define I2C_TRANSACTION_TIMEOUT         (2000U)

#define I2C_MPU_6065_INSTANCE   1
#define MPU_6065_SLAVE_ADDR     0x68
#define PWR_MGMT_1              0x6b
#define WHOAMI                  0x75

#define ACCEL_CONFIG            0x1c
#define ACCEL_XOUT_H            0x3b
#define ACCEL_XOUT_L            0x3c
#define ACCEL_YOUT_H            0x3d
#define ACCEL_YOUT_L            0x3e
#define ACCEL_ZOUT_H            0x3f
#define ACCEL_ZOUT_L            0x40

#define GYRO_CONFIG             0x1b
#define GYRO_XOUT_H             0x43
#define GYRO_XOUT_L             0x44
#define GYRO_YOUT_H             0x45
#define GYRO_YOUT_L             0x46
#define GYRO_ZOUT_H             0x47
#define GYRO_ZOUT_L             0x48

#define CONFIG                  0x1a
#define SMPLRT_DIV              0x19
#define FIFO_EN                 0x23

#define LSB_SENSITIVITY_ACCEL_2G       16384 //LSB/g
#define LSB_SENSITIVITY_ACCEL_4G       8192  //LSB/g
#define LSB_SENSITIVITY_ACCEL_8G       4096  //LSB/g
#define LSB_SENSITIVITY_ACCEL_16G      2048  //LSB/g

#define LSB_SENSITIVITY_GYRO_250       131   //LSBº/s
#define LSB_SENSITIVITY_GYRO_500       65.5  //LSBº/s
#define LSB_SENSITIVITY_GYRO_1000      38.8  //LSBº/s
#define LSB_SENSITIVITY_GYRO_2000      16.4  //LSBº/s

#define g   9.81

#define sensorFL    21   // 3 21 - 25 p9
#define sensorFR    17   // 1 17 - 23 p9
#define sensorRL    16   // 1 16 - 15 p9
#define sensorRR    28   // 1 28 - 12 p9

#define frd1    8   // 2  8  - 43 p8
#define fld1    11  // 2  11 - 42 p8
#define rrd1    6   // 2  6  - 45 p8
#define rld1    13  // 2  13 - 40 p8

#define frd2    9   // 2  9  - 44 p8
#define fld2    10  // 2  10 - 41 p8
#define rrd2    7   // 2  7  - 46 p8
#define rld2    12  // 2  12 - 39 p8

#define button  27  // 0  27 - 17 p8

#define intNumber   96

#define I2C_PCA_96685_INSTANCE      2
#define PCA_96685_SLAVE_ADDR     0x40

#define PCA9685_MODE1       0x00     /**< Mode Register 1 */
#define PCA9685_PRESCALE    0xFE    /**< Prescaler for PWM output frequency */

#define LED0_ON_L           0x06     /**< LED0 output and brightness control byte 0 */
#define LED0_ON_H           0x07     /**< LED0 output and brightness control byte 1 */
#define LED0_OFF_L          0x08     /**< LED0 output and brightness control byte 2 */
#define LED0_OFF_H          0x09     /**< LED0 output and brightness control byte 3 */

#define LED1_ON_L           0x0A
#define LED1_ON_H           0x0B
#define LED1_OFF_L          0x0C
#define LED1_OFF_H          0x0D

#define LED2_ON_L           0x0E
#define LED2_ON_H           0x0F
#define LED2_OFF_L          0x10
#define LED2_OFF_H          0x11

#define LED3_ON_L           0x12
#define LED3_ON_H           0x13
#define LED3_OFF_L          0x14
#define LED3_OFF_H          0x15

#define LED4_ON_L           0x16
#define LED4_ON_H           0x17
#define LED4_OFF_L          0x18
#define LED4_OFF_H          0x19

/**********************************************************************
 ************************** Global Variables **************************
 **********************************************************************/

char buff_GYRO_Z[100];

Clock_Handle myClock;
Clock_Handle myButClock;
Clock_Handle clkInterrupt;
Clock_Handle clkRoute;

Semaphore_Handle semTask;
Semaphore_Handle semTaskControlRoute;
Semaphore_Handle semTest;


Swi_Handle swi0;

Task_Handle taskConfig;
Task_Handle taskControlRoute;
Task_Handle task2;
Task_Handle task3;

/* Create Tasks */
Task_Params taskConfigParams;
Task_Params taskControlRouteParams;
Task_Params task02Params;
Task_Params task03Params;
Task_Params taskParams;
Task_Params confParams;


float offsetAx;
float offsetAy;
float offsetAz;
float offsetGx;
float offsetGy;
float offsetGz;

int route = 0;

/**********************************************************************
 ************************** Internal functions ************************
 **********************************************************************/

/* Data compare function */
bool CompareData(char *expData, char *rxData, unsigned int length);
/*      COMUNICAÇÃO COM O MPU      */
void    writeSensor(I2C_Handle h, uint8_t reg, uint8_t val);
uint8_t readSensor(I2C_Handle h, uint8_t reg);
/*      COMUNICAÇÃO COM O PCA      */
void    writeI2C(I2C_Handle h, uint8_t reg, uint8_t val);
uint8_t readI2C(I2C_Handle h, uint8_t reg);
/*      FUNÇÃO DE CONFIGURAÇÃO      */
void    myFunc();
void    clkFunc();
void    isrFunc();
void    swiFunc();
void    clkButFunc();
void    clkInterruptFunc();
void    route1();
void    route2();
void    route3();
void    route4();
void    routeControl();
float   getZg();

void setSpeedL0(uint8_t high, uint8_t low){

    UART_printf("roda 0\n");
    I2C_Params i2cParams;
    I2C_Handle handle = NULL;

    I2C_Params_init(&i2cParams);
    handle = I2C_open(I2C_PCA_96685_INSTANCE, &i2cParams);
    writeI2C(handle, PCA9685_MODE1, 0x10);
    Task_sleep(1);
    writeI2C(handle, PCA9685_PRESCALE, 0x78);
    writeI2C(handle, LED0_ON_H, 0x00);
    writeI2C(handle, LED0_ON_L, 0x00);
    if(high == 0 && low == 0){
        writeI2C(handle, LED0_OFF_H, 0x04);
        writeI2C(handle, LED0_OFF_L, 0x00);
    }
    else{
        writeI2C(handle, LED0_OFF_H, high);
        writeI2C(handle, LED0_OFF_L, low);
    }
    writeI2C(handle, PCA9685_MODE1, 0x00);

    I2C_close(handle);
}

void setSpeedL1(uint8_t high, uint8_t low){
    UART_printf("roda 1\n");
    I2C_Params i2cParams;
    I2C_Handle handle = NULL;

    I2C_Params_init(&i2cParams);
    handle = I2C_open(I2C_PCA_96685_INSTANCE, &i2cParams);
    writeI2C(handle, PCA9685_MODE1, 0x10);
    Task_sleep(1);
    writeI2C(handle, PCA9685_PRESCALE, 0x78);
    writeI2C(handle, LED1_ON_H, 0x00);
    writeI2C(handle, LED1_ON_L, 0x00);
    if(high == 0 && low == 0){
        writeI2C(handle, LED1_OFF_H, 0x04);
        writeI2C(handle, LED1_OFF_L, 0x00);
    }
    else{
        writeI2C(handle, LED0_OFF_H, high);
        writeI2C(handle, LED0_OFF_L, low);
    }
    writeI2C(handle, PCA9685_MODE1, 0x00);

    I2C_close(handle);
}

void setSpeedL2(uint8_t high, uint8_t low){
    UART_printf("roda 2\n");
    I2C_Params i2cParams;
    I2C_Handle handle = NULL;

    I2C_Params_init(&i2cParams);
    handle = I2C_open(I2C_PCA_96685_INSTANCE, &i2cParams);
    writeI2C(handle, PCA9685_MODE1, 0x10);
    Task_sleep(1);
    writeI2C(handle, PCA9685_PRESCALE, 0x78);
    writeI2C(handle, LED2_ON_H, 0x00);
    writeI2C(handle, LED2_ON_L, 0x00);
    if(high == 0 && low == 0){
        writeI2C(handle, LED2_OFF_H, 0x04);
        writeI2C(handle, LED2_OFF_L, 0x00);
    }
    else{
        writeI2C(handle, LED0_OFF_H, high);
        writeI2C(handle, LED0_OFF_L, low);
    }
    writeI2C(handle, PCA9685_MODE1, 0x00);

    I2C_close(handle);
}

void setSpeedL3(uint8_t high, uint8_t low){
    UART_printf("roda 3\n");
    I2C_Params i2cParams;
    I2C_Handle handle = NULL;

    I2C_Params_init(&i2cParams);
    handle = I2C_open(I2C_PCA_96685_INSTANCE, &i2cParams);
    writeI2C(handle, PCA9685_MODE1, 0x10);
    Task_sleep(1);
    writeI2C(handle, PCA9685_PRESCALE, 0x78);
    writeI2C(handle, LED3_ON_H, 0x00);
    writeI2C(handle, LED3_ON_L, 0x00);
    if(high == 0 && low == 0){
        writeI2C(handle, LED3_OFF_H, 0x04);
        writeI2C(handle, LED3_OFF_L, 0x00);
    }
    else{
        writeI2C(handle, LED0_OFF_H, high);
        writeI2C(handle, LED0_OFF_L, low);
    }
    writeI2C(handle, PCA9685_MODE1, 0x00);

    I2C_close(handle);
}


void frente(){
    GPIOPinWrite(SOC_GPIO_2_REGS, frd1, GPIO_PIN_HIGH);
    GPIOPinWrite(SOC_GPIO_2_REGS, fld1, GPIO_PIN_LOW);
    GPIOPinWrite(SOC_GPIO_2_REGS, rrd1, GPIO_PIN_HIGH);
    GPIOPinWrite(SOC_GPIO_2_REGS, rld1, GPIO_PIN_LOW);
    GPIOPinWrite(SOC_GPIO_2_REGS, frd2, GPIO_PIN_LOW);
    GPIOPinWrite(SOC_GPIO_2_REGS, fld2, GPIO_PIN_HIGH);
    GPIOPinWrite(SOC_GPIO_2_REGS, rrd2, GPIO_PIN_LOW);
    GPIOPinWrite(SOC_GPIO_2_REGS, rld2, GPIO_PIN_HIGH);

}

void esquerda(){
    GPIOPinWrite(SOC_GPIO_2_REGS, frd1, GPIO_PIN_HIGH);
    GPIOPinWrite(SOC_GPIO_2_REGS, fld1, GPIO_PIN_HIGH);
    GPIOPinWrite(SOC_GPIO_2_REGS, rrd1, GPIO_PIN_HIGH);
    GPIOPinWrite(SOC_GPIO_2_REGS, rld1, GPIO_PIN_HIGH);
    GPIOPinWrite(SOC_GPIO_2_REGS, frd2, GPIO_PIN_LOW);
    GPIOPinWrite(SOC_GPIO_2_REGS, fld2, GPIO_PIN_LOW);
    GPIOPinWrite(SOC_GPIO_2_REGS, rrd2, GPIO_PIN_LOW);
    GPIOPinWrite(SOC_GPIO_2_REGS, rld2, GPIO_PIN_LOW);
}

void direita(){
    GPIOPinWrite(SOC_GPIO_2_REGS, frd1, GPIO_PIN_LOW);
    GPIOPinWrite(SOC_GPIO_2_REGS, fld1, GPIO_PIN_LOW);
    GPIOPinWrite(SOC_GPIO_2_REGS, rrd1, GPIO_PIN_LOW);
    GPIOPinWrite(SOC_GPIO_2_REGS, rld1, GPIO_PIN_LOW);
    GPIOPinWrite(SOC_GPIO_2_REGS, frd2, GPIO_PIN_HIGH);
    GPIOPinWrite(SOC_GPIO_2_REGS, fld2, GPIO_PIN_HIGH);
    GPIOPinWrite(SOC_GPIO_2_REGS, rrd2, GPIO_PIN_HIGH);
    GPIOPinWrite(SOC_GPIO_2_REGS, rld2, GPIO_PIN_HIGH);
}

void para(){
    GPIOPinWrite(SOC_GPIO_2_REGS, frd1, GPIO_PIN_LOW);
    GPIOPinWrite(SOC_GPIO_2_REGS, fld1, GPIO_PIN_LOW);
    GPIOPinWrite(SOC_GPIO_2_REGS, rrd1, GPIO_PIN_LOW);
    GPIOPinWrite(SOC_GPIO_2_REGS, rld1, GPIO_PIN_LOW);
    GPIOPinWrite(SOC_GPIO_2_REGS, frd2, GPIO_PIN_LOW);
    GPIOPinWrite(SOC_GPIO_2_REGS, fld2, GPIO_PIN_LOW);
    GPIOPinWrite(SOC_GPIO_2_REGS, rrd2, GPIO_PIN_LOW);
    GPIOPinWrite(SOC_GPIO_2_REGS, rld2, GPIO_PIN_LOW);
}

#if defined (idkAM572x)
char eepromData[I2C_EEPROM_TEST_LENGTH] = {0x55, 0x33, 0xEE, 0x41, 0x4D, 0x35, 0x37, 0x32,
                              0x49, 0x44};
#elif defined(idkAM574x)
char eepromData[I2C_EEPROM_TEST_LENGTH] = {0x55, 0x33, 0xEE, 0x41, 0x4D, 0x35, 0x37, 0x34,
                              0x49, 0x44};
#elif defined (idkAM571x)
char eepromData[I2C_EEPROM_TEST_LENGTH] = {0x55, 0x33, 0xEE, 0x41, 0x4D, 0x35, 0x37, 0x31,
                              0x49, 0x44};
#elif defined (evmAM572x)
char eepromData[I2C_EEPROM_TEST_LENGTH] = {0x55, 0x33, 0xEE, 0x41, 0x4d, 0x35, 0x37, 0x32,
                              0x50, 0x4d};
#elif defined (evmDRA72x)
char eepromData[I2C_EEPROM_TEST_LENGTH] = {0x33, 0x55, 0xAA, 0x4A, 0x36, 0x45, 0x43, 0x4F,
                              0x43, 0x50};
#elif defined (evmDRA75x)
char eepromData[I2C_EEPROM_TEST_LENGTH] = {0xEE, 0x33, 0x55, 0xAA, 0x35, 0x37, 0x37, 0x37,
                              0x78, 0x43};
#elif defined (evmDRA78x)
char eepromData[I2C_EEPROM_TEST_LENGTH] = {0x33, 0x55, 0xAA, 0x41, 0x44, 0x41, 0x53, 0x2D,
                              0x4C, 0x4F};
#elif defined (evmK2H) || defined (evmK2K) || defined (evmK2E) || defined (evmK2L) || defined (evmK2G) || defined (evmC6678) || defined (evmC6657) || defined (iceK2G) || defined (evmOMAPL137)
char eepromData[I2C_EEPROM_TEST_LENGTH] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                              0x00, 0x00};
#else
#endif

/*
 *  ======== Board_initI2C ========
 */
bool Board_initI2C(void)
{
    Board_initCfg boardCfg;
    Board_STATUS  boardStatus;
#if defined (idkAM571x)
    Board_IDInfo  id;
#endif
    I2C_HwAttrs   i2c_cfg;
#if defined (evmK2G)
    Board_SoCInfo socInfo;
#endif

    /* Get the default I2C init configurations */
    I2C_socGetInitCfg(I2C_EEPROM_INSTANCE, &i2c_cfg);

    /* Modify the default I2C configurations if necessary */

    /* Set the default I2C init configurations */
    I2C_socSetInitCfg(I2C_EEPROM_INSTANCE, &i2c_cfg);

    I2C_init();

#if defined(evmK2E) || defined(evmC6678)
    boardCfg = BOARD_INIT_MODULE_CLOCK |
        BOARD_INIT_UART_STDIO;
#else
    boardCfg = BOARD_INIT_PINMUX_CONFIG |
        BOARD_INIT_MODULE_CLOCK |
        BOARD_INIT_UART_STDIO;
#endif
    boardStatus = Board_init(boardCfg);
    if (boardStatus != BOARD_SOK)
    {
        return (false);
    }

#if defined (idkAM571x)
    boardStatus = Board_getIDInfo(&id);
    if (boardStatus != BOARD_SOK)
    {
        return (false);
    }
    memcpy(eepromData, &id.header[I2C_EEPROM_TEST_ADDR],
           BOARD_EEPROM_HEADER_LENGTH - I2C_EEPROM_TEST_ADDR);
    memcpy(&eepromData[BOARD_EEPROM_HEADER_LENGTH - I2C_EEPROM_TEST_ADDR],
           id.boardName,
           I2C_EEPROM_TEST_LENGTH - BOARD_EEPROM_HEADER_LENGTH + I2C_EEPROM_TEST_ADDR);
#endif

#if defined (evmK2G)
    /* Read the SoC info to get the System clock value */
    Board_getSoCInfo(&socInfo);
    if(socInfo.sysClock != BOARD_SYS_CLK_DEFAULT)
    {
        /* Get the default I2C init configurations */
        I2C_socGetInitCfg(I2C_EEPROM_INSTANCE, &i2c_cfg);
        /* Update the I2C functional clock based on CPU clock - 1G or 600MHz */
        i2c_cfg.funcClk = socInfo.sysClock/I2C_MODULE_CLOCK_DIVIDER;
        /* Set the default I2C init configurations */
        I2C_socSetInitCfg(I2C_EEPROM_INSTANCE, &i2c_cfg);
    }
#endif

    return (true);
}

/*
 *  ======== Board_initGPIO ========
 */
static void Board_initGPIO(void)
{
    Board_initCfg boardCfg;

#if defined(SOC_K2H) || defined(SOC_K2K) || defined(SOC_K2E) || defined(SOC_K2L) || defined(SOC_K2G) || defined(SOC_C6678) || defined(SOC_C6657) || defined(SOC_OMAPL137) || defined(SOC_OMAPL138) || defined(SOC_AM65XX) || defined(SOC_J721E)
    GPIO_v0_HwAttrs gpio_cfg;

    /* Get the default SPI init configurations */
    GPIO_socGetInitCfg(GPIO_LED0_PORT_NUM, &gpio_cfg);


#if defined(SOC_K2G)
    /* Setup GPIO interrupt configurations */
    GPIO_socSetIntMux(GPIO_LED0_PORT_NUM, GPIO_LED0_PIN_NUM, NULL, GPIO_MUX_SEL);
#endif
#if defined(SOC_OMAPL137) || defined(SOC_OMAPL138)
    /* Setup GPIO interrupt configurations */
    GPIO_socSetBankInt(GPIO_LED0_PORT_NUM, GPIO_LED0_PIN_NUM, NULL);
#endif
#endif

#if defined(evmK2E) || defined(evmC6678)
    boardCfg = BOARD_INIT_MODULE_CLOCK |
        BOARD_INIT_UART_STDIO;
#else
    boardCfg = BOARD_INIT_PINMUX_CONFIG |
        BOARD_INIT_MODULE_CLOCK |
        BOARD_INIT_UART_STDIO;
#endif
    Board_init(boardCfg);

#if defined(idkAM572x) || defined(idkAM574x)
    GPIOApp_UpdateBoardInfo();
#endif

    /* Modify the default GPIO configurations if necessary */
#if defined (am65xx_evm) || defined (am65xx_idk) || defined (j721e_sim)

    GPIO_configIntRouter(GPIO_LED0_PORT_NUM, GPIO_LED0_PIN_NUM, 0, &gpio_cfg);

    /* Set the default GPIO init configurations */
    GPIO_socSetInitCfg(GPIO_LED0_PORT_NUM, &gpio_cfg);

#endif
}


/*
 *  ======== main ========
 */
int main(void)
{
    if (Board_initI2C() == false)
    {
        return (0);
    }

#if defined (SOC_AM335X) || defined (SOC_AM437x) || defined (SOC_OMAPL137)

	Error_Block eb;
    Error_init(&eb);

    /*
     *
     * */

    Board_initGPIO();

    GPIODirModeSet(SOC_GPIO_1_REGS, 21, GPIO_CFG_OUTPUT);
    GPIODirModeSet(SOC_GPIO_1_REGS, 22, GPIO_CFG_OUTPUT);
    GPIODirModeSet(SOC_GPIO_1_REGS, 23, GPIO_CFG_OUTPUT);
    GPIODirModeSet(SOC_GPIO_1_REGS, 24, GPIO_CFG_OUTPUT);

    GPIODirModeSet(SOC_GPIO_3_REGS, sensorFL, GPIO_CFG_INPUT);
    GPIODirModeSet(SOC_GPIO_1_REGS, sensorFR, GPIO_CFG_INPUT);
    GPIODirModeSet(SOC_GPIO_1_REGS, sensorRL, GPIO_CFG_INPUT);
    GPIODirModeSet(SOC_GPIO_1_REGS, sensorRR, GPIO_CFG_INPUT);

    GPIODirModeSet(SOC_GPIO_2_REGS, frd1, GPIO_CFG_OUTPUT);
    GPIODirModeSet(SOC_GPIO_2_REGS, fld1, GPIO_CFG_OUTPUT);
    GPIODirModeSet(SOC_GPIO_2_REGS, rrd1, GPIO_CFG_OUTPUT);
    GPIODirModeSet(SOC_GPIO_2_REGS, rld1, GPIO_CFG_OUTPUT);

    GPIODirModeSet(SOC_GPIO_2_REGS, frd2, GPIO_CFG_OUTPUT);
    GPIODirModeSet(SOC_GPIO_2_REGS, fld2, GPIO_CFG_OUTPUT);
    GPIODirModeSet(SOC_GPIO_2_REGS, rrd2, GPIO_CFG_OUTPUT);
    GPIODirModeSet(SOC_GPIO_2_REGS, rld2, GPIO_CFG_OUTPUT);

    /* Create Hwi */
    Hwi_Params hwiParams;
    Hwi_Params_init(&hwiParams);
    hwiParams.enableInt = FALSE;
    Hwi_create(intNumber, isrFunc, &hwiParams, NULL);
    Hwi_enableInterrupt(intNumber);

    GPIODirModeSet(SOC_GPIO_0_REGS, button, GPIO_CFG_INPUT);
    GPIOIntTypeSet(SOC_GPIO_0_REGS, button, GPIO_INT_TYPE_RISE_EDGE);
    GPIOPinIntEnable(SOC_GPIO_0_REGS, GPIO_INT_LINE_1, button);

    /* Create Swi */
    Swi_Params swiParams;
    Swi_Params_init(&swiParams);
    swi0 = Swi_create(swiFunc, &swiParams, NULL);
    Swi_enable();


    /*      TASK DE CONFIGURAÇÃO      */
    Task_Params_init(&taskConfigParams);
    taskConfigParams.stackSize = 0x1400;
    taskConfigParams.priority = 10;
    taskConfig = Task_create(myFunc, &taskConfigParams, &eb);

    /*      TASK DE CONTROLE DE ROTA      */
    Error_Block eb1;
    Error_init(&eb1);
    Task_Params_init(&taskControlRouteParams);
    taskControlRouteParams.stackSize = 0x1400;
    taskControlRouteParams.priority = 10;
    taskControlRoute = Task_create(routeControl, &taskControlRouteParams, &eb1);

    /*      CLOCK DE CONTROLE DA CONFIGURAÇÃO      */
    Clock_Params clkParams;
    Clock_Params_init(&clkParams);
    clkParams.startFlag = TRUE;
    clkParams.period = 100;
    myClock = Clock_create(clkFunc, 1, &clkParams, NULL);

    /*      CLOCK DE CONTROLE PARA INICIAR A ROTA ESCOLHIDA      */
    Clock_Params clkButParams;
    Clock_Params_init(&clkButParams);
    clkButParams.startFlag = FALSE;
    clkButParams.period = 200;
    myButClock = Clock_create(clkButFunc, 200, &clkButParams, NULL);

    /*      CLOCK DE CONTROLE PARA ESCOLHA DA ROTA      */
    Clock_Params clkInterruptParams;
    Clock_Params_init(&clkInterruptParams);
    clkInterruptParams.startFlag = FALSE;
    clkInterruptParams.period = 50;
    clkInterrupt = Clock_create(clkInterruptFunc, 50, &clkInterruptParams, NULL);

    /*      SEMAFORO DE CONTROLE TASK DE CONFIGURAÇÃO       */
    Semaphore_Params semParams;
    Semaphore_Params_init(&semParams);
    semParams.mode = Semaphore_Mode_BINARY;
    semTask = Semaphore_create(0, &semParams, NULL);

    /*      SEMAFORO DE CONTROLE TASK DE ROTA       */
    Semaphore_Params semParams1;
    Semaphore_Params_init(&semParams1);
    semParams1.mode = Semaphore_Mode_BINARY;
    semTaskControlRoute = Semaphore_create(0, &semParams1, NULL);


    /*
     *
     * */


#endif

    /* Start BIOS */
    BIOS_start();
    return (0);
}

/*
 *  ======== CompareData ========
 */

bool CompareData(char *expData, char *rxData, unsigned int length)
{
    uint32_t idx = 0;
    uint32_t match = 1;
    bool retVal = false;

    for(idx = 0; ((idx < length) && (match != 0)); idx++)
    {
        if(*expData != *rxData) match = 0;
        expData++;
        rxData++;
    }

    if(match == 1) retVal = true;

    return retVal;
}

uint8_t readSensor(I2C_Handle h, uint8_t reg){
    uint8_t rxData = 0;
    uint8_t txData = 0;

    I2C_Transaction t;
    int16_t transferStatus;

    I2C_transactionInit(&t);

    memset(&txData, 0x00, sizeof(txData));

    t.slaveAddress = MPU_6065_SLAVE_ADDR;
    t.writeBuf = &txData;
    t.writeCount = 1;
    t.readBuf = &rxData;
    t.readCount = 1;
    t.timeout = 1000U;
    txData = reg;

    transferStatus = I2C_transfer(h, &t);

    if (I2C_STS_SUCCESS != transferStatus){
        UART_printf("\n Data Transfer failed with transfer status %d \n", transferStatus);
    }

    return rxData;
}

void writeSensor(I2C_Handle h, uint8_t reg, uint8_t val){
    uint8_t txData[2] = {0, 0};
    I2C_Transaction t;
    int16_t transferStatus;

    I2C_transactionInit(&t);

    memset(&txData, 0x00, sizeof(txData));

    t.slaveAddress = MPU_6065_SLAVE_ADDR;
    t.writeBuf = &txData;
    t.writeCount = 2;
    t.readCount = 0;
    t.timeout = 1000U;
    txData[0] = reg;
    txData[1] = val;

    transferStatus = I2C_transfer(h, &t);

    if (I2C_STS_SUCCESS != transferStatus){
        UART_printf("\n Data Transfer failed with transfer status %d \n", transferStatus);
    }

}

uint8_t readI2C(I2C_Handle h, uint8_t reg){
    uint8_t rxData = 0;
    uint8_t txData = 0;

    I2C_Transaction t;
    int16_t transferStatus;

    I2C_transactionInit(&t);

    memset(&txData, 0x00, sizeof(txData));

    t.slaveAddress = PCA_96685_SLAVE_ADDR;
    t.writeBuf = &txData;
    t.writeCount = 1;
    t.readBuf = &rxData;
    t.readCount = 1;
    t.timeout = 1000U;
    txData = reg;

    transferStatus = I2C_transfer(h, &t);

    if (I2C_STS_SUCCESS != transferStatus){
        UART_printf("\n Data Transfer failed with transfer status %d \n", transferStatus);
    }

    return rxData;
}

void writeI2C(I2C_Handle h, uint8_t reg, uint8_t val){
    uint8_t txData[2] = {0, 0};
    I2C_Transaction t;
    int16_t transferStatus;

    I2C_transactionInit(&t);

    memset(&txData, 0x00, sizeof(txData));

    t.slaveAddress = PCA_96685_SLAVE_ADDR;
    t.writeBuf = &txData;
    t.writeCount = 2;
    t.readCount = 0;
    t.timeout = 1000U;
    txData[0] = reg;
    txData[1] = val;

    transferStatus = I2C_transfer(h, &t);

    if (I2C_STS_SUCCESS != transferStatus){
        UART_printf("\n Data Transfer failed with transfer status %d \n", transferStatus);
    }
}

void myFunc(){

    I2C_Params i2cParams;
    I2C_Handle handle = NULL;
    I2C_Params_init(&i2cParams);

    handle = I2C_open(I2C_MPU_6065_INSTANCE, &i2cParams);
    UART_printf("Init Acc/Gyro Setup... \n");

    UART_printf("Device Reset... \n");
    writeSensor(handle, PWR_MGMT_1, 0x00);

    UART_printf("Accelerometer config... \n");
    writeSensor(handle, ACCEL_CONFIG, 0x10);

    UART_printf("Gyro config... \n");
    writeSensor(handle, GYRO_CONFIG, 0x10);

//    I2C_close(handle);
    UART_printf("Acc/Gyro Setup Ended... \n\n");

    int i = 0;
    int len = 10;

    float xa[len];
    float ya[len];
    float za[len];
    float xg[len];
    float yg[len];
    float zg[len];


    while(1){
        Semaphore_pend(semTask, BIOS_WAIT_FOREVER);
        GPIOPinWrite(SOC_GPIO_1_REGS, 21, GPIO_PIN_HIGH);
        if(i == len){
            I2C_close(handle);
            Clock_stop(myClock);

        }
        else if(i < len){
            UART_printf("Calibration step %d\n", i+1);
            xa[i] = convertValRegToFloat((readSensor(handle, ACCEL_XOUT_H)<<8) + readSensor(handle, ACCEL_XOUT_L), LSB_SENSITIVITY_ACCEL_8G);
            ya[i] = convertValRegToFloat((readSensor(handle, ACCEL_YOUT_H)<<8) + readSensor(handle, ACCEL_YOUT_L), LSB_SENSITIVITY_ACCEL_8G);
            za[i] = convertValRegToFloat((readSensor(handle, ACCEL_ZOUT_H)<<8) + readSensor(handle, ACCEL_ZOUT_L), LSB_SENSITIVITY_ACCEL_8G);
            xg[i] = convertValRegToFloat((readSensor(handle, GYRO_XOUT_H) <<8) + readSensor(handle, GYRO_XOUT_L), LSB_SENSITIVITY_GYRO_1000);
            yg[i] = convertValRegToFloat((readSensor(handle, GYRO_YOUT_H) <<8) + readSensor(handle, GYRO_YOUT_L), LSB_SENSITIVITY_GYRO_1000);
            zg[i] = convertValRegToFloat((readSensor(handle, GYRO_ZOUT_H) <<8) + readSensor(handle, GYRO_ZOUT_L), LSB_SENSITIVITY_GYRO_1000);

            if(i+1 == len){

                /*      CALCULO PARA CALIBRAÇÃO (DEFINIÇÃO DOS OFFSETS PARA DIMINUIÇAO DO ERRO NAS LEITURAS)      */
                offsetAx = roundfl((xa[0]+xa[1]+xa[2]+xa[3]+xa[4]+xa[5]+xa[6]+xa[7]+xa[8]+xa[9])/len) - ((xa[0]+xa[1]+xa[2]+xa[3]+xa[4]+xa[5]+xa[6]+xa[7]+xa[8]+xa[9])/len);
                offsetAy = roundfl((ya[0]+ya[1]+ya[2]+ya[3]+ya[4]+ya[5]+ya[6]+ya[7]+ya[8]+ya[9])/len) - ((ya[0]+ya[1]+ya[2]+ya[3]+ya[4]+ya[5]+ya[6]+ya[7]+ya[8]+ya[9])/len);
                offsetAz = roundfl((za[0]+za[1]+za[2]+za[3]+za[4]+za[5]+za[6]+za[7]+za[8]+za[9])/len) - ((za[0]+za[1]+za[2]+za[3]+za[4]+za[5]+za[6]+za[7]+za[8]+za[9])/len);
                offsetGx = 0 - ((xg[0]+xg[1]+xg[2]+xg[3]+xg[4]+xg[5]+xg[6]+xg[7]+xg[8]+xg[9])/len);
                offsetGy = 0 - ((yg[0]+yg[1]+yg[2]+yg[3]+yg[4]+yg[5]+yg[6]+yg[7]+yg[8]+yg[9])/len);
                offsetGz = 0 - ((zg[0]+zg[1]+zg[2]+zg[3]+zg[4]+zg[5]+zg[6]+zg[7]+zg[8]+zg[9])/len);

                /*      CONFIGURAÇÃO DOS PWMs DAS 4 RODAS      */
                setSpeedL0(0,0);
                setSpeedL1(0,0);
                setSpeedL2(0,0);
                setSpeedL3(0,0);
            }
            i++;
        }
        GPIOPinWrite(SOC_GPIO_1_REGS, 21, GPIO_PIN_LOW);
    }
}

void clkFunc(){
    Semaphore_post(semTask);
}

void isrFunc(){
    UART_printStatus("\n ISR triggered. \n");

    Swi_post(swi0);
    Clock_start(clkInterrupt);
    Clock_start(myButClock);

    GPIOPinIntClear(SOC_GPIO_0_REGS, GPIO_INT_LINE_1, button);
}

void swiFunc(){
    UART_printStatus("\n Swi triggered. \n");
    Hwi_disableInterrupt(intNumber);
}

void clkInterruptFunc(){
    Clock_stop(clkInterrupt);
    route = route + 1;
    if(route == 1){
        GPIOPinWrite(SOC_GPIO_1_REGS, 21, GPIO_PIN_HIGH);
        GPIOPinWrite(SOC_GPIO_1_REGS, 22, GPIO_PIN_LOW);
        GPIOPinWrite(SOC_GPIO_1_REGS, 23, GPIO_PIN_LOW);
        GPIOPinWrite(SOC_GPIO_1_REGS, 24, GPIO_PIN_LOW);
    }
    else if(route == 2){
        GPIOPinWrite(SOC_GPIO_1_REGS, 21, GPIO_PIN_LOW);
        GPIOPinWrite(SOC_GPIO_1_REGS, 22, GPIO_PIN_HIGH);
        GPIOPinWrite(SOC_GPIO_1_REGS, 23, GPIO_PIN_LOW);
        GPIOPinWrite(SOC_GPIO_1_REGS, 24, GPIO_PIN_LOW);
    }
    else if(route == 3){
        GPIOPinWrite(SOC_GPIO_1_REGS, 21, GPIO_PIN_LOW);
        GPIOPinWrite(SOC_GPIO_1_REGS, 22, GPIO_PIN_LOW);
        GPIOPinWrite(SOC_GPIO_1_REGS, 23, GPIO_PIN_HIGH);
        GPIOPinWrite(SOC_GPIO_1_REGS, 24, GPIO_PIN_LOW);
    }
    else if(route == 4){
        GPIOPinWrite(SOC_GPIO_1_REGS, 21, GPIO_PIN_LOW);
        GPIOPinWrite(SOC_GPIO_1_REGS, 22, GPIO_PIN_LOW);
        GPIOPinWrite(SOC_GPIO_1_REGS, 23, GPIO_PIN_LOW);
        GPIOPinWrite(SOC_GPIO_1_REGS, 24, GPIO_PIN_HIGH);
    }
    else{
        route = 0;
    }
    Hwi_enableInterrupt(intNumber);
    GPIOPinIntClear(SOC_GPIO_0_REGS, GPIO_INT_LINE_1, button);
}

void clkButFunc(){
    UART_printStatus("\n Clk Button Func triggered. r = ");
    UART_printf("%d\n", route);
    GPIOPinWrite(SOC_GPIO_1_REGS, 21, GPIO_PIN_LOW);
    GPIOPinWrite(SOC_GPIO_1_REGS, 22, GPIO_PIN_LOW);
    GPIOPinWrite(SOC_GPIO_1_REGS, 23, GPIO_PIN_LOW);
    GPIOPinWrite(SOC_GPIO_1_REGS, 24, GPIO_PIN_LOW);
    Semaphore_post(semTaskControlRoute);
    Clock_stop(myButClock);
}

void route1(){
    GPIOPinWrite(SOC_GPIO_1_REGS, 21, GPIO_PIN_HIGH);
    frente();
    int conta = 0;
    Task_sleep(50);
    while(conta != 12){
        if(!GPIOPinRead(SOC_GPIO_1_REGS, sensorFR)){
            conta = conta + 1;
            UART_printf("%d\n", conta);
            Task_sleep(10);
        }
    }
    para();
    GPIOPinWrite(SOC_GPIO_1_REGS, 21, GPIO_PIN_LOW);
}

void route2(){
    GPIOPinWrite(SOC_GPIO_1_REGS, 22, GPIO_PIN_HIGH);
    frente();
    int conta = 0;
    Task_sleep(50);
    while(conta != 25){
        if(!GPIOPinRead(SOC_GPIO_1_REGS, sensorFR)){
            conta = conta + 1;
            UART_printf("%d\n", conta);
            Task_sleep(10);
        }
    }
    para();
    GPIOPinWrite(SOC_GPIO_1_REGS, 22, GPIO_PIN_LOW);
}

void route3(){
    GPIOPinWrite(SOC_GPIO_1_REGS, 23, GPIO_PIN_HIGH);
    int conta = 0;
    float soma = 0;
    float atual;
    float base;
    frente();
    Task_sleep(50);

    frente();
    while(conta != 16){
        if(!GPIOPinRead(SOC_GPIO_1_REGS, sensorFR)){
            conta = conta + 1;
            UART_printf("%d\n", conta);
            Task_sleep(10);
        }
    }
    para();
    // logica pra salvar o anterior
    base = getZg();
    if(base < 0){
        base = (base*(-1));
    }
    soma = 0;
    esquerda();
    while(soma < 90){
        Task_sleep(40);
        atual = getZg();
        if(atual < 0){
            atual = (atual*(-1));
        }
        ftoa(atual, buff_GYRO_Z, 4);
        UART_printf("%s\n", buff_GYRO_Z);
        if(atual > (base + 1)){
            soma = soma + (atual*0.7);
        }
    }
    para();

    GPIOPinWrite(SOC_GPIO_1_REGS, 23, GPIO_PIN_LOW);
}

void route4(){
    GPIOPinWrite(SOC_GPIO_1_REGS, 24, GPIO_PIN_HIGH);

    int conta = 0;
    float soma = 0;
    float atual;
    float base;

    frente();
//    Task_sleep(50);
    // anda 30cm
    frente();
    while(conta != 6){
        if(!GPIOPinRead(SOC_GPIO_1_REGS, sensorFR)){
            conta = conta + 1;
            UART_printf("%d\n", conta);
            Task_sleep(10);
        }
    }
    para();

    // curva pra direita 45º
    base = getZg();
    if(base < 0){
        base = (base*(-1));
    }
    soma = 0;
    direita();
    while(soma < 45){
        Task_sleep(40);
        atual = getZg();
        if(atual < 0){
            atual = (atual*(-1));
        }
        ftoa(atual, buff_GYRO_Z, 4);
        UART_printf("%s\n", buff_GYRO_Z);
        if(atual > (base + 1)){
            soma = soma + (atual*0.7);
        }
    }
    para();
    // anda 42cm
    conta = 0;
    frente();
    while(conta != 10){
        if(!GPIOPinRead(SOC_GPIO_1_REGS, sensorFR)){
            conta = conta + 1;
            UART_printf("%d\n", conta);
            Task_sleep(10);
        }
    }
    para();
    // curva pra direita 45º
    base = getZg();
    if(base < 0){
        base = (base*(-1));
    }
    soma = 0;
    direita();
    while(soma < 45){
        Task_sleep(40);
        atual = getZg();
        if(atual < 0){
            atual = (atual*(-1));
        }
        ftoa(atual, buff_GYRO_Z, 4);
        UART_printf("%s\n", buff_GYRO_Z);
        if(atual > (base + 1)){
            soma = soma + (atual*0.7);
        }
    }
    para();
    //anda 30cm
    conta = 0;
    frente();
    while(conta != 6){
        if(!GPIOPinRead(SOC_GPIO_1_REGS, sensorFR)){
            conta = conta + 1;
            UART_printf("%d\n", conta);
            Task_sleep(10);
        }
    }
    para();
    // curva pra direita 90º
    base = getZg();
    if(base < 0){
        base = (base*(-1));
    }
    soma = 0;
    direita();
    while(soma < 90){
        Task_sleep(40);
        atual = getZg();
        if(atual < 0){
            atual = (atual*(-1));
        }
        ftoa(atual, buff_GYRO_Z, 4);
        UART_printf("%s\n", buff_GYRO_Z);
        if(atual > (base + 1)){
            soma = soma + (atual*0.65);
        }
    }
    para();
    // anda 30cm
    conta = 0;
    frente();
    while(conta != 6){
        if(!GPIOPinRead(SOC_GPIO_1_REGS, sensorFR)){
            conta = conta + 1;
            UART_printf("%d\n", conta);
            Task_sleep(10);
        }
    }
    para();
    // curva pra direita 90º
    base = getZg();
    if(base < 0){
        base = (base*(-1));
    }
    soma = 0;
    direita();
    while(soma < 90){
        Task_sleep(40);
        atual = getZg();
        if(atual < 0){
            atual = (atual*(-1));
        }
        ftoa(atual, buff_GYRO_Z, 4);
        UART_printf("%s\n", buff_GYRO_Z);
        if(atual > (base + 1)){
            soma = soma + (atual*0.7);
        }
    }
    para();
    // anda 30cm
    conta = 0;
    frente();
    while(conta != 6){
        if(!GPIOPinRead(SOC_GPIO_1_REGS, sensorFR)){
            conta = conta + 1;
            UART_printf("%d\n", conta);
            Task_sleep(10);
        }
    }
    para();
    // curva pra esquerda 45
    base = getZg();
    if(base < 0){
        base = (base*(-1));
    }
    soma = 0;
    esquerda();
    while(soma < 45){
        Task_sleep(40);
        atual = getZg();
        if(atual < 0){
            atual = (atual*(-1));
        }
        ftoa(atual, buff_GYRO_Z, 4);
        UART_printf("%s\n", buff_GYRO_Z);
        if(atual > (base + 1)){
            soma = soma + (atual*0.7);
        }
    }
    para();
    // anda 42cm
    conta = 0;
    frente();
    while(conta != 10){
        if(!GPIOPinRead(SOC_GPIO_1_REGS, sensorFR)){
            conta = conta + 1;
            UART_printf("%d\n", conta);
            Task_sleep(10);
        }
    }
    para();
    GPIOPinWrite(SOC_GPIO_1_REGS, 24, GPIO_PIN_LOW);
}

void routeControl(){
    while(1){
        Semaphore_pend(semTaskControlRoute, BIOS_WAIT_FOREVER);
        switch (route) {
            case 1:
                route1();
                break;
            case 2:
                route2();
                break;
            case 3:
                route3();
                route3();
                route3();
                route3();
                break;
            case 4:
                route4();
                break;

            default:
                break;
        }

        route = 0;
    }
}

float getZg(){
    I2C_Params i2cParams;
    I2C_Handle handle = NULL;
    I2C_Params_init(&i2cParams);

    handle = I2C_open(I2C_MPU_6065_INSTANCE, &i2cParams);

    float value = convertValRegToFloat((((readSensor(handle, GYRO_ZOUT_H)<<8) + readSensor(handle, GYRO_ZOUT_L))), LSB_SENSITIVITY_GYRO_1000) + offsetGz;

    I2C_close(handle);

    return value;
}
