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



#if defined (SOC_AM335X) || defined (SOC_AM437x)
/* EEPROM data -Board specific */
extern char eepromData[I2C_EEPROM_RX_LENGTH];
#endif

/**********************************************************************
 ************************** Macros ************************************
 **********************************************************************/

#define I2C_TRANSACTION_TIMEOUT         (2000U)


/**********************************************************************
 ************************** Internal functions ************************
 **********************************************************************/

/* Data compare function */
bool CompareData(char *expData, char *rxData, unsigned int length);

/**********************************************************************
 ************************** Global Variables **************************
 **********************************************************************/

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


#define I2C_MPU_6065_INSTANCE   2
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


/*
 * I2C1_SLC  17 p9
 * I2C1_SDA  18 p9
 */

// FUNTIONS
void IMUSetUp();
void writeSensor(I2C_Handle h, uint8_t reg, uint8_t val);
uint8_t readSensor(I2C_Handle h, uint8_t reg);
void myFunc();
void clkFunc();
float convertValRegToFloat(int val,int LSB);
float convertValRegToFloatGYRO(int val,int LSB);
int roundfl(float num);
float absf(float num);
static void Board_initGPIO(void);

// GLOBAL VARIABLES

I2C_Handle handle;
Clock_Handle myClock;
Semaphore_Handle semTask;
Task_Params taskParams;
Task_Params confParams;
I2C_Params i2cParams;

float offsetAx;
float offsetAy;
float offsetAz;
float offsetGx;
float offsetGy;
float offsetGz;


const unsigned CheckIfSigned[33]=
{0x00000000,
0x00000001,0x00000002,0x00000004,0x00000008,
0x00000010,0x00000020,0x00000040,0x00000080,
0x00000100,0x00000200,0x00000400,0x00000800,
0x00001000,0x00002000,0x00004000,0x00008000,
0x00010000,0x00020000,0x00040000,0x00080000,
0x00100000,0x00200000,0x00400000,0x00800000,
0x01000000,0x02000000,0x04000000,0x08000000,
0x10000000,0x20000000,0x40000000,0x80000000};
const unsigned ConvertToSigned[32]=
{0xffffffff,
0xfffffffe,0xfffffffc,0xfffffff8,0xfffffff0,
0xffffffe0,0xffffffc0,0xffffff80,0xffffff00,
0xfffffe00,0xfffffc00,0xfffff800,0xfffff000,
0xffffe000,0xffffc000,0xffff8000,0xffff0000,
0xfffe0000,0xfffc0000,0xfff80000,0xfff00000,
0xffe00000,0xffc00000,0xff800000,0xff000000,
0xfe000000,0xfc000000,0xf8000000,0xf0000000,
0xe0000000,0xc0000000,0x80000000};
const unsigned digits2bits[33]=
{0x00000000,
0x00000001,0x00000003,0x00000007,0x0000000f,
0x0000001f,0x0000003f,0x0000007f,0x000000ff,
0x000001ff,0x000003ff,0x000007ff,0x00000fff,
0x00001fff,0x00003fff,0x00007fff,0x0000ffff,
0x0001ffff,0x0003ffff,0x0007ffff,0x000fffff,
0x001fffff,0x003fffff,0x007fffff,0x00ffffff,
0x01ffffff,0x03ffffff,0x07ffffff,0x0fffffff,
0x1fffffff,0x3fffffff,0x7fffffff,0xffffffff};





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
//    I2C_socGetInitCfg(I2C_MPU_6065_INSTANCE, &i2c_cfg);

    /* Modify the default I2C configurations if necessary */

    /* Set the default I2C init configurations */
    I2C_socSetInitCfg(I2C_EEPROM_INSTANCE, &i2c_cfg);
//    I2C_socSetInitCfg(I2C_MPU_6065_INSTANCE, &i2c_cfg);

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
 *  ======== test function ========
 */
void i2c_test(UArg arg0, UArg arg1)
{
    I2C_Params i2cParams;
    I2C_Handle handle = NULL;
    I2C_Transaction i2cTransaction;
    char txBuf[I2C_EEPROM_TEST_LENGTH + I2C_EEPROM_ADDR_SIZE] = {0x00, };
    char rxBuf[I2C_EEPROM_TEST_LENGTH] = {0x00, };
    bool status,test_pass=FALSE;
    int16_t transferStatus;

    /* Set the I2C EEPROM write/read address */
    txBuf[0] = (I2C_EEPROM_TEST_ADDR >> 8) & 0xff; /* EEPROM memory high address byte */
    txBuf[1] = I2C_EEPROM_TEST_ADDR & 0xff;        /* EEPROM memory low address byte */

    I2C_init();

    I2C_Params_init(&i2cParams);

    handle = I2C_open(I2C_EEPROM_INSTANCE, &i2cParams);

    I2C_transactionInit(&i2cTransaction);
    i2cTransaction.slaveAddress = I2C_EEPROM_ADDR;
    i2cTransaction.writeBuf = (uint8_t *)&txBuf[0];
    i2cTransaction.writeCount = I2C_EEPROM_ADDR_SIZE;
    i2cTransaction.readBuf = (uint8_t *)&rxBuf[0];
    i2cTransaction.readCount = I2C_EEPROM_TEST_LENGTH;
    i2cTransaction.timeout   = I2C_TRANSACTION_TIMEOUT;
    transferStatus = I2C_transfer(handle, &i2cTransaction);

    if(I2C_STS_SUCCESS != transferStatus)
    {
        I2C_log("\n Data Transfer failed with transfer status %d \n",transferStatus);
        test_pass=FALSE;
        goto exit_test;
    }

    I2C_close(handle);

#if defined (evmK2H) || defined (evmK2K) || defined (evmK2E) || defined (evmK2L) || defined (evmK2G) || defined (iceK2G) || defined (evmC6678)  || defined (evmC6657) || defined (iceK2G) || defined (evmOMAPL137)
    /* EEPROM not programmed on K2 EVMs, copy rx data to eepromData
       so it can pass the test */
    memcpy(eepromData, rxBuf, I2C_EEPROM_TEST_LENGTH);
#endif
    status = CompareData(&eepromData[0], &rxBuf[0], I2C_EEPROM_TEST_LENGTH);
    if(TRUE == status)
    {
        I2C_log("\n EEPROM data matched \n");
       test_pass=TRUE;
    }
    else
    {
       test_pass=FALSE;
    }

exit_test:

   if(TRUE == test_pass)
   {
       UART_printStatus("\n All tests have passed. \n");
    }
    else
    {
        UART_printStatus("\n Some tests have failed. \n");
    }

    while (1) {

    }
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

	Task_Handle task;
//	Task_Handle conf;
	Error_Block eb;

    Error_init(&eb);

    /*
     *
     * */

    Board_initGPIO();

    GPIODirModeSet(SOC_GPIO_1_REGS, 21, GPIO_CFG_OUTPUT);

//    Task_Params_init(&confParams);
//    confParams.stackSize = 0x1400;
//    confParams.priority = 15;
//    conf = Task_create(IMUSetUp, &taskParams, &eb);

    Task_Params_init(&taskParams);
    taskParams.stackSize = 0x1400;
    taskParams.priority = 10;
    task = Task_create(myFunc, &taskParams, &eb);

    Clock_Params clkParams;
    Clock_Params_init(&clkParams);
    clkParams.startFlag = TRUE;
    clkParams.period = 1;
    myClock = Clock_create(clkFunc, 3, &clkParams, NULL);

    /* Create Semaphore */
    Semaphore_Params semParams;
    Semaphore_Params_init(&semParams);
    semParams.mode = Semaphore_Mode_BINARY;
    semTask = Semaphore_create(0, &semParams, NULL);

//    task = Task_create(i2c_test, NULL, &eb);
//    task = Task_create(IMUSetUp, &taskParams    , &eb);

    /*
     *
     * */

    if (task == NULL) {
        System_printf("Task_create() failed!\n");
        BIOS_exit(0);
    }
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

void IMUSetUp(){

    handle = NULL;

    I2C_Params_init(&i2cParams);

    handle = I2C_open(I2C_MPU_6065_INSTANCE, &i2cParams); //UART_printf("depois\n");

    UART_printf("Init Acc/Gyro Setup... \n");

    UART_printf("Device Reset... \n");
    writeSensor(handle, PWR_MGMT_1, 0x00);

    UART_printf("Accelerometer config... \n");
    writeSensor(handle, ACCEL_CONFIG, 0x10);

    UART_printf("Gyro config... \n\n");
    writeSensor(handle, GYRO_CONFIG, 0x10);

    I2C_close(handle);
    UART_printf("Acc/Gyro Setup Ended... \n\n");
}

void myFunc(){

    handle = NULL;
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

    char buff_ACCEL_X[100];
    char buff_ACCEL_Y[100];
    char buff_ACCEL_Z[100];
    char buff_GYRO_X[100];
    char buff_GYRO_Y[100];
    char buff_GYRO_Z[100];

    float Vx, Vx_prev = 0;
    float Vy, Vy_prev = 0;
    float Vz, Vz_prev = 0;
//    float aIncX;
//    float aIncY;
//    float aIncZ;

    char buff_V_X[100];
    char buff_V_Y[100];
    char buff_V_Z[100];

    while(1){
        Semaphore_pend(semTask, BIOS_WAIT_FOREVER);
        GPIOPinWrite(SOC_GPIO_1_REGS, 21, GPIO_PIN_HIGH);
        if(i == len){
            float Xa = convertValRegToFloat((((readSensor(handle, ACCEL_XOUT_H)<<8) + readSensor(handle, ACCEL_XOUT_L))), LSB_SENSITIVITY_ACCEL_8G) + offsetAx;
            float Ya = convertValRegToFloat((((readSensor(handle, ACCEL_YOUT_H)<<8) + readSensor(handle, ACCEL_YOUT_L))), LSB_SENSITIVITY_ACCEL_8G) + offsetAy;
            float Za = convertValRegToFloat((((readSensor(handle, ACCEL_ZOUT_H)<<8) + readSensor(handle, ACCEL_ZOUT_L))), LSB_SENSITIVITY_ACCEL_8G) + offsetAz;
            float Xg = convertValRegToFloat((((readSensor(handle, GYRO_XOUT_H)<<8) + readSensor(handle, GYRO_XOUT_L))), LSB_SENSITIVITY_GYRO_1000) + offsetGx;
            float Yg = convertValRegToFloat((((readSensor(handle, GYRO_YOUT_H)<<8) + readSensor(handle, GYRO_YOUT_L))), LSB_SENSITIVITY_GYRO_1000) + offsetGy;
            float Zg = convertValRegToFloat((((readSensor(handle, GYRO_ZOUT_H)<<8) + readSensor(handle, GYRO_ZOUT_L))), LSB_SENSITIVITY_GYRO_1000) + offsetGz;

            ftoa(Xa, buff_ACCEL_X, 4);
            ftoa(Ya, buff_ACCEL_Y, 4);
            ftoa(Za, buff_ACCEL_Z, 4);
            ftoa(Xg, buff_GYRO_X, 4);
            ftoa(Yg, buff_GYRO_Y, 4);
            ftoa(Zg, buff_GYRO_Z, 4);

            float aux = sqrtf(powf(Xa, 2) + powf(Ya, 2) + powf(Za, 2));
            if(aux >= 0.95 && aux <= 1.05){
                Vx = 0;
                Vy = 0;
                Vz = 0;
            }
            else{
                Vx = Vx_prev + Xa*g;
                Vy = Vy_prev + Ya*g;
                Vz = Vz_prev + Za*g;
            }

            Vx_prev = Vx;
            Vy_prev = Vy;
            Vz_prev = Vz;

            ftoa(Vx, buff_V_X, 4);
            ftoa(Vy, buff_V_Y, 4);
            ftoa(Vz, buff_V_Z, 4);

//            float aux2 = sqrtf(powf(Xg, 2) + powf(Yg, 2) + powf(Zg, 2));
//            if(aIncX >= -0.05 && aIncX <= 0.05){
//                aIncX
//            }
//            if(aIncY >= -0.05 && aIncY <= 0.05){
//
//            }
//            if(aIncZ >= -0.05 && aIncZ <= 0.05){
//
//            }



            UART_printf("{\"ACCEL\":{\"x\":\"%s\",\"y\":\"%s\",\"z\":\"%s\"},\"GYRO\":{\"x\":\"%s\",\"y\":\"%s\",\"z\":\"%s\"},\"VELO\":{\"x\":\"%s\",\"y\":\"%s\",\"z\":\"%s\"}}\n",
                        buff_ACCEL_X, buff_ACCEL_Y, buff_ACCEL_Z, buff_GYRO_X, buff_GYRO_Y, buff_GYRO_Z, buff_V_X, buff_V_Y, buff_V_Z);

        }
        else if(i < len){
            UART_printf("Calibration step %d\n", i+1);
            xa[i] = convertValRegToFloat((readSensor(handle, ACCEL_XOUT_H)<<8) + readSensor(handle, ACCEL_XOUT_L), LSB_SENSITIVITY_ACCEL_8G);
            ya[i] = convertValRegToFloat((readSensor(handle, ACCEL_YOUT_H)<<8) + readSensor(handle, ACCEL_YOUT_L), LSB_SENSITIVITY_ACCEL_8G);
            za[i] = convertValRegToFloat((readSensor(handle, ACCEL_ZOUT_H)<<8) + readSensor(handle, ACCEL_ZOUT_L), LSB_SENSITIVITY_ACCEL_8G);
            xg[i] = convertValRegToFloat((readSensor(handle, GYRO_XOUT_H)<<8) + readSensor(handle, GYRO_XOUT_L), LSB_SENSITIVITY_GYRO_1000);
            yg[i] = convertValRegToFloat((readSensor(handle, GYRO_YOUT_H)<<8) + readSensor(handle, GYRO_YOUT_L), LSB_SENSITIVITY_GYRO_1000);
            zg[i] = convertValRegToFloat((readSensor(handle, GYRO_ZOUT_H)<<8) + readSensor(handle, GYRO_ZOUT_L), LSB_SENSITIVITY_GYRO_1000);

            if(i+1 == 10){
                offsetAx = roundfl((xa[0]+xa[1]+xa[2]+xa[3]+xa[4]+xa[5]+xa[6]+xa[7]+xa[8]+xa[9])/len) - ((xa[0]+xa[1]+xa[2]+xa[3]+xa[4]+xa[5]+xa[6]+xa[7]+xa[8]+xa[9])/len);
                offsetAy = roundfl((ya[0]+ya[1]+ya[2]+ya[3]+ya[4]+ya[5]+ya[6]+ya[7]+ya[8]+ya[9])/len) - ((ya[0]+ya[1]+ya[2]+ya[3]+ya[4]+ya[5]+ya[6]+ya[7]+ya[8]+ya[9])/len);
                offsetAz = roundfl((za[0]+za[1]+za[2]+za[3]+za[4]+za[5]+za[6]+za[7]+za[8]+za[9])/len) - ((za[0]+za[1]+za[2]+za[3]+za[4]+za[5]+za[6]+za[7]+za[8]+za[9])/len);
                offsetGx = 0 - ((xg[0]+xg[1]+xg[2]+xg[3]+xg[4]+xg[5]+xg[6]+xg[7]+xg[8]+xg[9])/len);
                offsetGy = 0 - ((yg[0]+yg[1]+yg[2]+yg[3]+yg[4]+yg[5]+yg[6]+yg[7]+yg[8]+yg[9])/len);
                offsetGz = 0 - ((zg[0]+zg[1]+zg[2]+zg[3]+zg[4]+zg[5]+zg[6]+zg[7]+zg[8]+zg[9])/len);
            }
            i++;
        }
        GPIOPinWrite(SOC_GPIO_1_REGS, 21, GPIO_PIN_LOW);
    }
}

void clkFunc(){
    Semaphore_post(semTask);

}

float convertValRegToFloat(int val,int LSB){
    int temp;
    temp = val & CheckIfSigned[16];
    if (temp != 0)
        temp = (val | ConvertToSigned[16-1]);
    else
        temp=val;

    float ret = (temp+0.0)/LSB;

    return ret;
}

float absf(float num){
    if(num < 0)
        num *=-1;
    return num;
}

int roundfl(float num){
    int l = num;
    int h = l+1;
    if(absf(num - l) < absf(num - h))
        return l;
    return h;
}

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

