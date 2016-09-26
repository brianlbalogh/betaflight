/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "exti.h"

// RA = Register Address

#define MPU_RA_PRODUCT_ID       0x0C    // Product ID Register
#define ITG_RA_XG_OFFS_TC_H     0x04    //[9:8] XG_OFFS_TC_H
#define ITG_RA_XG_OFFS_TC_L     0x05
#define ITG_RA_YG_OFFS_TC_H     0x07    //[9:8] YG_OFFS_TC_H
#define ITG_RA_YG_OFFS_TC_L     0x08
#define ITG_RA_ZG_OFFS_TC_H     0x0A    //[9:8] ZG_OFFS_TC_H
#define ITG_RA_ZG_OFFS_TC_L     0x0B
#define ITG_RA_XG_OFFS_USRH     0x13    //[15:0] XG_OFFS_USR
#define ITG_RA_XG_OFFS_USRL     0x14
#define ITG_RA_YG_OFFS_USRH     0x15    //[15:0] YG_OFFS_USR
#define ITG_RA_YG_OFFS_USRL     0x16
#define ITG_RA_ZG_OFFS_USRH     0x17    //[15:0] ZG_OFFS_USR
#define ITG_RA_ZG_OFFS_USRL     0x18
#define ITG_RA_SMPLRT_DIV       0x19
#define ITG_RA_CONFIG           0x1A
#define ITG_RA_GYRO_CONFIG      0x1B
#define ITG_RA_FIFO_EN          0x23
#define ITG_RA_INT_PIN_CFG      0x37
#define ITG_RA_INT_ENABLE       0x38
#define ITG_RA_INT_STATUS       0x3A
#define ITG_RA_TEMP_OUT_H       0x41
#define ITG_RA_TEMP_OUT_L       0x42
#define ITG_RA_GYRO_XOUT_H      0x43
#define ITG_RA_GYRO_XOUT_L      0x44
#define ITG_RA_GYRO_YOUT_H      0x45
#define ITG_RA_GYRO_YOUT_L      0x46
#define ITG_RA_GYRO_ZOUT_H      0x47
#define ITG_RA_GYRO_ZOUT_L      0x48
#define ITG_RA_USER_CTRL        0x6A
#define ITG_RA_PWR_MGMT_1       0x6B
#define ITG_RA_PWR_MGMT_2       0x6C
#define ITG_RA_FIFO_COUNTH      0x72
#define ITG_RA_FIFO_COUNTL      0x73
#define ITG_RA_FIFO_R_W         0x74
#define ITG_RA_WHO_AM_I         0x75

#define MPU_RA_I2C_MST_CTRL     0x24
#define MPU_RA_I2C_SLV0_ADDR    0x25
#define MPU_RA_I2C_SLV0_REG     0x26
#define MPU_RA_I2C_SLV0_CTRL    0x27
#define MPU_RA_I2C_SLV1_ADDR    0x28
#define MPU_RA_I2C_SLV1_REG     0x29
#define MPU_RA_I2C_SLV1_CTRL    0x2A
#define MPU_RA_I2C_SLV2_ADDR    0x2B
#define MPU_RA_I2C_SLV2_REG     0x2C
#define MPU_RA_I2C_SLV2_CTRL    0x2D
#define MPU_RA_I2C_SLV3_ADDR    0x2E
#define MPU_RA_I2C_SLV3_REG     0x2F
#define MPU_RA_I2C_SLV3_CTRL    0x30
#define MPU_RA_I2C_SLV4_ADDR    0x31
#define MPU_RA_I2C_SLV4_REG     0x32
#define MPU_RA_I2C_SLV4_DO      0x33
#define MPU_RA_I2C_SLV4_CTRL    0x34
#define MPU_RA_I2C_SLV4_DI      0x35
#define MPU_RA_I2C_MST_STATUS   0x36
#define MPU_RA_I2C_SLV0_DO      0x63
#define MPU_RA_I2C_SLV1_DO      0x64
#define MPU_RA_I2C_SLV2_DO      0x65
#define MPU_RA_I2C_SLV3_DO      0x66
#define MPU_RA_I2C_MST_DELAY_CTRL   0x67
#define MPU_RA_SIGNAL_PATH_RESET    0x68
#define MPU_RA_MOT_DETECT_CTRL      0x69
#define MPU_RA_BANK_SEL         0x6D
#define MPU_RA_MEM_START_ADDR   0x6E
#define MPU_RA_MEM_R_W          0x6F

// RF = Register Flag
#define ITG_RF_DATA_RDY_EN (1 << 0)

typedef bool (*itgReadRegisterFunc)(uint8_t reg, uint8_t length, uint8_t* data);
typedef bool (*itgWriteRegisterFunc)(uint8_t reg, uint8_t data);
typedef void(*itgResetFuncPtr)(void);  

typedef struct itgConfiguration_s {
    uint8_t gyroReadXRegister; // Y and Z must registers follow this, 2 words each
    itgReadRegisterFunc read;
    itgWriteRegisterFunc write;
    itgReadRegisterFunc slowread;
    itgWriteRegisterFunc verifywrite;
    itgResetFuncPtr reset;
} itgConfiguration_t;

extern itgConfiguration_t itgConfiguration;

typedef enum {
    ITG_NONE,
    ITG_1010,
    ITG_3701,
    ITG_1010_SPI,
    ITG_3701_SPI,
    ITG_1010_I2C,
    ITG_3701_I2C
} detectedITGSensor_e;

typedef struct itgDetectionResult_s {
    detectedITGSensor_e sensor;
} itgDetectionResult_t;

extern itgDetectionResult_t itgDetectionResult;

void configureItgDataReadyInterruptHandling(void);
void itgIntExtiInit(void);
bool itgGyroRead(int16_t *gyroADC);
itgDetectionResult_t *detectItg(const extiConfig_t *configToUse);
bool checkItgDataReady(void);
