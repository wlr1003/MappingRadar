/*
 * lsm6dsl.h
 *
 *  Created on: Jan 9, 2023
 *      Author: William Ralston
 *
 *  Adapted from
 * @file    lsm6dsl.h
 * @author  MCD Application Team
 * @brief   LSM6DSL header driver file
 */

#ifndef INC_LSM6DSL_H_
#define INC_LSM6DSL_H_

#include "stm32g4xx_hal.h"

/* Registers Address Mapping
 * data sheet pg. 48
 */

#define LSM6DSL_FUNC_CFG_ACCESS	   0x01 //Embedded functions configuration register
#define LSM6DSL_SENSOR_SYNC_TIME   0x04
#define LSM6DSL_SENSOR_RES_RATIO   0x05
#define LSM6DSL_FIFO_CTRL1         0x06
#define LSM6DSL_FIFO_CTRL2         0x07
#define LSM6DSL_FIFO_CTRL3         0x08
#define LSM6DSL_FIFO_CTRL4         0x09
#define LSM6DSL_FIFO_CTRL5         0x0A
#define LSM6DSL_DRDY_PULSE_CFG_G   0x0B
#define LSM6DSL_INT1_CTRL          0x0D
#define LSM6DSL_INT2_CTRL          0x0E
#define LSM6DSL_WHO_AM_I           0x0F
#define LSM6DSL_CTRL1_XL           0x10
#define LSM6DSL_CTRL2_G            0x11
#define LSM6DSL_CTRL3_C            0x12
#define LSM6DSL_CTRL4_C            0x13
#define LSM6DSL_CTRL5_C            0x14
#define LSM6DSL_CTRL6_C            0x15
#define LSM6DSL_CTRL7_G            0x16
#define LSM6DSL_CTRL8_XL           0x17
#define LSM6DSL_CTRL9_XL           0x18
#define LSM6DSL_CTRL10_C           0x19
#define LSM6DSL_I2C_MASTER_CONFIG  0x1A
#define LSM6DSL_WAKE_UP_SRC        0x1B
#define LSM6DSL_TAP_SRC            0x1C
#define LSM6DSL_D6D_SRC            0x1D
#define LSM6DSL_STATUS_REG         0x1E
#define LSM6DSL_OUT_TEMP_L         0x20
#define LSM6DSL_OUT_TEMP_H         0x21
#define LSM6DSL_OUTX_L_G           0x22
#define LSM6DSL_OUTX_H_G           0x23
#define LSM6DSL_OUTY_L_G           0x24
#define LSM6DSL_OUTY_H_G           0x25
#define LSM6DSL_OUTZ_L_G           0x26
#define LSM6DSL_OUTZ_H_G           0x27
#define LSM6DSL_OUTX_L_XL          0x28
#define LSM6DSL_OUTX_H_XL          0x29
#define LSM6DSL_OUTY_L_XL          0x2A
#define LSM6DSL_OUTY_H_XL          0x2B
#define LSM6DSL_OUTZ_L_XL          0x2C
#define LSM6DSL_OUTZ_H_XL          0x2D
#define LSM6DSL_SENSORHUB1_REG     0x2E
#define LSM6DSL_SENSORHUB2_REG     0x2F
#define LSM6DSL_SENSORHUB3_REG     0x30
#define LSM6DSL_SENSORHUB4_REG     0x31
#define LSM6DSL_SENSORHUB5_REG     0x32
#define LSM6DSL_SENSORHUB6_REG     0x33
#define LSM6DSL_SENSORHUB7_REG     0x34
#define LSM6DSL_SENSORHUB8_REG     0x35
#define LSM6DSL_SENSORHUB9_REG     0x36
#define LSM6DSL_SENSORHUB10_REG    0x37
#define LSM6DSL_SENSORHUB11_REG    0x38
#define LSM6DSL_SENSORHUB12_REG    0x39
#define LSM6DSL_FIFO_STATUS1       0x3A
#define LSM6DSL_FIFO_STATUS2       0x3B
#define LSM6DSL_FIFO_STATUS3       0x3C
#define LSM6DSL_FIFO_STATUS4       0x3D
#define LSM6DSL_FIFO_DATA_OUT_L    0x3E
#define LSM6DSL_FIFO_DATA_OUT_H    0x3F
#define LSM6DSL_TIMESTAMP0_REG     0x40
#define LSM6DSL_TIMESTAMP1_REG     0x41
#define LSM6DSL_TIMESTAMP2_REG     0x42
#define LSM6DSL_TIMESTAMP_L        0x49
#define LSM6DSL_TIMESTAMP_H        0x4A
#define LSM6DSL_STEP_COUNTER_L     0x4B
#define LSM6DSL_STEP_COUNTER_H     0x4C
#define LSM6DSL_SENSORHUB13_REG    0x4D
#define LSM6DSL_SENSORHUB14_REG    0x4E
#define LSM6DSL_SENSORHUB15_REG    0x4F
#define LSM6DSL_SENSORHUB16_REG    0x50
#define LSM6DSL_SENSORHUB17_REG    0x51
#define LSM6DSL_SENSORHUB18_REG    0x52
#define LSM6DSL_FUNC_SRC           0x53
#define LSM6DSL_TAP_CFG1           0x58
#define LSM6DSL_TAP_THS_6D         0x59
#define LSM6DSL_INT_DUR2           0x5A
#define LSM6DSL_WAKE_UP_THS        0x5B
#define LSM6DSL_WAKE_UP_DUR        0x5C
#define LSM6DSL_FREE_FALL          0x5D
#define LSM6DSL_MD1_CFG            0x5E
#define LSM6DSL_MD2_CFG            0x5F
#define LSM6DSL_X_OFS_USR          0x73
#define LSM6DSL_Y_OFS_USR          0x74
#define LSM6DSL_Z_OFS_USR          0x75

/* Accelerometer Full Scale Selection */
#define LSM6DSL_ACC_FULLSCALE_2G          ((uint8_t)0x00) /* 2 g */
#define LSM6DSL_ACC_FULLSCALE_4G          ((uint8_t)0x08) /* 4 g */
#define LSM6DSL_ACC_FULLSCALE_8G          ((uint8_t)0x0C) /* 8 g */
#define LSM6DSL_ACC_FULLSCALE_16G         ((uint8_t)0x04) /* 16 g */

/* Accelerometer Full Scale Sensitivity */
#define LSM6DSL_ACC_SENSITIVITY_2G     ((float)0.061f)  /* accelerometer sensitivity with 2 g full scale  [mgauss/LSB] */
#define LSM6DSL_ACC_SENSITIVITY_4G     ((float)0.122f)  /* accelerometer sensitivity with 4 g full scale  [mgauss/LSB] */
#define LSM6DSL_ACC_SENSITIVITY_8G     ((float)0.244f)  /* accelerometer sensitivity with 8 g full scale  [mgauss/LSB] */
#define LSM6DSL_ACC_SENSITIVITY_16G    ((float)0.488f)  /* accelerometer sensitivity with 16 g full scale [mgauss/LSB] */

/* Accelerometer Power Mode selection */
#define LSM6DSL_LP_XL_DISABLED     ((uint8_t)0x00) /* LP disabled*/
#define LSM6DSL_LP_XL_ENABLED      ((uint8_t)0x10) /* LP enabled*/

/* Output Data Rate */
#define LSM6DSL_ODR_BITPOSITION      ((uint8_t)0xF0)  /*!< Output Data Rate bit position */
#define LSM6DSL_ODR_POWER_DOWN       ((uint8_t)0x00) /* Power Down mode       */
#define LSM6DSL_ODR_13Hz             ((uint8_t)0x10) /* Low Power mode        */
#define LSM6DSL_ODR_26Hz             ((uint8_t)0x20) /* Low Power mode        */
#define LSM6DSL_ODR_52Hz             ((uint8_t)0x30) /* Low Power mode        */
#define LSM6DSL_ODR_104Hz            ((uint8_t)0x40) /* Normal mode           */
#define LSM6DSL_ODR_208Hz            ((uint8_t)0x50) /* Normal mode           */
#define LSM6DSL_ODR_416Hz            ((uint8_t)0x60) /* High Performance mode */
#define LSM6DSL_ODR_833Hz            ((uint8_t)0x70) /* High Performance mode */
#define LSM6DSL_ODR_1660Hz           ((uint8_t)0x80) /* High Performance mode */
#define LSM6DSL_ODR_3330Hz           ((uint8_t)0x90) /* High Performance mode */
#define LSM6DSL_ODR_6660Hz           ((uint8_t)0xA0) /* High Performance mode */

/* Gyroscope Full Scale Selection */
#define LSM6DSL_GYRO_FS_245          ((uint8_t)0x00)
#define LSM6DSL_GYRO_FS_500          ((uint8_t)0x04)
#define LSM6DSL_GYRO_FS_1000         ((uint8_t)0x08)
#define LSM6DSL_GYRO_FS_2000         ((uint8_t)0x0C)

/* Gyroscope Full Scale Sensitivity */
#define LSM6DSL_GYRO_SENSITIVITY_245DPS            ((float)8.750f) /**< Sensitivity value for 245 dps full scale  [mdps/LSB] */
#define LSM6DSL_GYRO_SENSITIVITY_500DPS            ((float)17.50f) /**< Sensitivity value for 500 dps full scale  [mdps/LSB] */
#define LSM6DSL_GYRO_SENSITIVITY_1000DPS           ((float)35.00f) /**< Sensitivity value for 1000 dps full scale [mdps/LSB] */
#define LSM6DSL_GYRO_SENSITIVITY_2000DPS           ((float)70.00f) /**< Sensitivity value for 2000 dps full scale [mdps/LSB] */

/* Gyroscope Power Mode selection */
#define LSM6DSL_ACC_GYRO_LP_G_DISABLED     ((uint8_t)0x00) /* LP disabled*/
#define LSM6DSL_ACC_GYRO_LP_G_ENABLED      ((uint8_t)0x80) /* LP enabled*/

/* Block Data Update */
#define LSM6DSL_BDU_CONTINUOS               ((uint8_t)0x00)
#define LSM6DSL_BDU_BLOCK_UPDATE            ((uint8_t)0x40)

/* Auto-increment */
#define LSM6DSL_IF_INC_DISABLED    ((uint8_t)0x00)
#define LSM6DSL_IF_INC_ENABLED     ((uint8_t)0x04)


typedef struct {
	/* Acceleration data (X,Y,Z) in m/s^2 */
	float acceleration_mps2[3];

	/* Gyroscope data (X,Y,Z)  */
	float gyro_mps2[3];

	/* Temperature data in degree C */
	foat temp_C;
} LSM6DSL;


/*
 * Initialization
 */
uint8_t LSM6DSL_Initialize(LSM6DSL *ptr);

/*
 * Data Acquisition
 */
HAL_StatusTypeDef LSM6DSL_ReadTemperature(LSM6DSL *ptr);
HAL_StatusTypeDef LSM6DSL_ReadAcceleration(LSM6DSL *ptr);
HAL_StatusTypeDef LSM6DSL_ReadGyroscope(LSM6DSL *ptr);

/*
 * Low Level Functions
 */

HAL_StatusTypeDef LSM6DSL_ReadRegister(LSM6DSL *ptr, uint8_t reg, uint8_t *data);
HAL_StatusTypeDef LSM6DSL_ReadRegisters(LSM6DSL *ptr, uint8_t reg, uint8_t *data, uint8_t length);

HAL_StatusTypeDef LSM6DSL_WriteRegister(LSM6DSL *ptr, uint8_t reg, uint8_t *data);

/* below copied from stm driver
///* LSM6DSL_AccExported_Functions ACCELEROMETER Exported functions */
//
//void    LSM6DSL_AccInit(uint16_t InitStruct);
//void    LSM6DSL_AccDeInit(void);
//uint8_t LSM6DSL_AccReadID(void);
//void    LSM6DSL_AccLowPower(uint16_t status);
//void    LSM6DSL_AccReadXYZ(int16_t* pData);
///**
//  * @}
//  */
//
///** @defgroup LSM6DSL_AccImported_Globals  ACCELEROMETER Imported Globals
//  * @{
//  */
//extern ACCELERO_DrvTypeDef Lsm6dslAccDrv;
///**
//  * @}
//  */
//
///** @defgroup LSM6DSL_GyroExported_Functions GYROSCOPE Exported functions
//  * @{
//  */
///* Sensor Configuration Functions */
//void    LSM6DSL_GyroInit(uint16_t InitStruct);
//void    LSM6DSL_GyroDeInit(void);
//uint8_t LSM6DSL_GyroReadID(void);
//void    LSM6DSL_GyroLowPower(uint16_t status);
//void    LSM6DSL_GyroReadXYZAngRate(float *pfData);
///**
//  * @}
//  */
//
///** @defgroup LSM6DSL_GyroImported_Globals  GYROSCOPE Imported Globals
//  * @{
//  */
///* Gyroscope driver structure */
//extern GYRO_DrvTypeDef Lsm6dslGyroDrv;
//
///**
//  * @}
//  */
//
///** @defgroup LSM6DSL_Imported_Functions LSM6DSL Imported Functions
// * @{
// */
///* IO functions */
//extern void     SENSOR_IO_Init(void);
//extern void     SENSOR_IO_Write(uint8_t Addr, uint8_t Reg, uint8_t Value);
//extern uint8_t  SENSOR_IO_Read(uint8_t Addr, uint8_t Reg);
//extern uint16_t SENSOR_IO_ReadMultiple(uint8_t Addr, uint8_t Reg, uint8_t *Buffer, uint16_t Length);
//extern void     SENSOR_IO_WriteMultiple(uint8_t Addr, uint8_t Reg, uint8_t *Buffer, uint16_t Length);


#endif /* INC_LSM6DSL_H_ */
