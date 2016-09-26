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


/*
 * Authors:
 * Dominic Clifton - Cleanflight implementation
 * John Ihlein - Initial FF32 code
*/

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "common/axis.h"
#include "common/maths.h"

#include "system.h"
#include "io.h"
#include "exti.h"
#include "bus_spi.h"

#include "gyro_sync.h"

#include "sensor.h"
#include "accgyro.h"
#include "accgyro_mpu.h"
#include "accgyro_itg.h"

#if defined(USE_GYRO_SPI_ITG1010)

#include "accgyro_spi_itg1010.h"

static void itg1010GyroInit(void);

static bool itg1010SpiInitDone = false;


// Bits
#define BIT_SLEEP                   0x40
#define BIT_H_RESET                 0x80
#define BIT_I2C_IF_DIS              0x10
#define BIT_SIG_COND_RESET          0x01
#define BIT_TEMP_DIS                0x08
#define BITS_CLKSEL                 0x07
#define ITG_CLK_SEL_PLLGYROX        0x01
#define ITG_CLK_SEL_PLLGYROZ        0x03
#define MPU_EXT_SYNC_GYROX          0x02
#define BITS_FS_250DPS              0x00
#define BITS_FS_500DPS              0x08
#define BITS_FS_1000DPS             0x10
#define BITS_FS_2000DPS             0x18
#define BITS_FS_2G                  0x00
#define BITS_FS_4G                  0x08
#define BITS_FS_8G                  0x10
#define BITS_FS_16G                 0x18
#define BITS_FS_MASK                0x18
#define BITS_DLPF_CFG_256HZ         0x00
#define BITS_DLPF_CFG_188HZ         0x01
#define BITS_DLPF_CFG_98HZ          0x02
#define BITS_DLPF_CFG_42HZ          0x03
#define BITS_DLPF_CFG_20HZ          0x04
#define BITS_DLPF_CFG_10HZ          0x05
#define BITS_DLPF_CFG_5HZ           0x06
#define BITS_DLPF_CFG_2100HZ_NOLPF  0x07
#define BITS_DLPF_CFG_MASK          0x07
#define BIT_INT_ANYRD_2CLEAR        0x10
#define BIT_RAW_RDY_EN              0x01
#define BIT_I2C_IF_DIS              0x10
#define BIT_INT_STATUS_DATA         0x01
#define BIT_GYRO                    3
#define BIT_TEMP                    1

#define DISABLE_ITG1010       IOHi(itg1010SpiCsPin)
#define ENABLE_ITG1010        IOLo(itg1010SpiCsPin)

static IO_t itg1010SpiCsPin = IO_NONE;

bool itg1010WriteRegister(uint8_t reg, uint8_t data)
{
    ENABLE_ITG1010;
    spiTransferByte(ITG1010_SPI_INSTANCE, reg);
    spiTransferByte(ITG1010_SPI_INSTANCE, data);
    DISABLE_ITG1010;

    return true;
}

bool itg1010ReadRegister(uint8_t reg, uint8_t length, uint8_t *data)
{
    ENABLE_ITG1010;
    spiTransferByte(ITG1010_SPI_INSTANCE, reg | 0x80); // read transaction
    spiTransfer(ITG1010_SPI_INSTANCE, data, NULL, length);
    DISABLE_ITG1010;

    return true;
}

void itg1010SpiGyroInit(uint8_t lpf)
{
    itgIntExtiInit();

    itg1010GyroInit();

    spiSetDivisor(ITG1010_SPI_INSTANCE, SPI_CLOCK_INITIALIZATON);

    // Gyro DLPF Setting
    itg1010WriteRegister(ITG1010_CONFIG, lpf);
    delayMicroseconds(1);

    spiSetDivisor(ITG1010_SPI_INSTANCE, SPI_CLOCK_FAST);  // 18 MHz SPI clock

    int16_t data[3];
    itgGyroRead(data);

    if (((int8_t)data[1]) == -1 && ((int8_t)data[0]) == -1) {
        failureMode(FAILURE_GYRO_INIT_FAILED);
    }
}

bool itg1010SpiDetect(void)
{
    uint8_t in;
    uint8_t attemptsRemaining = 5;

#ifdef ITG1010_CS_PIN
    itg1010SpiCsPin = IOGetByTag(IO_TAG(ITG1010_CS_PIN));
#endif
    IOInit(itg1010SpiCsPin, OWNER_MPU, RESOURCE_SPI_CS, 0);
    IOConfigGPIO(itg1010SpiCsPin, SPI_IO_CS_CFG);

    spiSetDivisor(ITG1010_SPI_INSTANCE, SPI_CLOCK_INITIALIZATON);

    itg1010WriteRegister(ITG_RA_PWR_MGMT_1, BIT_H_RESET);

    do {
        delay(150);

        itg1010ReadRegister(ITG_RA_WHO_AM_I, 1, &in);
        if (in == ITG1010_WHO_AM_I_CONST) {
            break;
        }
        if (!attemptsRemaining) {
            return false;
        }
    } while (attemptsRemaining--);


    return true;
}

static void itg1010GyroInit(void) {

    if (itg1010SpiInitDone) {
        return;
    }

    spiSetDivisor(ITG1010_SPI_INSTANCE, SPI_CLOCK_INITIALIZATON);

    // Device Reset
    itg1010WriteRegister(ITG_RA_PWR_MGMT_1, BIT_H_RESET);
    delay(150);

    itg1010WriteRegister(ITG_RA_USER_CTRL, BIT_I2C_IF_DIS|BIT_SIG_COND_RESET);
    delay(150);

    // Clock Source PPL with Z axis gyro reference
    itg1010WriteRegister(ITG_RA_PWR_MGMT_1, ITG_CLK_SEL_PLLGYROZ);
    delayMicroseconds(15);

    itg1010WriteRegister(ITG_RA_PWR_MGMT_2, 0x00);
    delayMicroseconds(15);

    // Accel Sample Rate 1kHz
    // Gyroscope Output Rate =  1kHz when the DLPF is enabled
    itg1010WriteRegister(ITG_RA_SMPLRT_DIV, gyroMPU6xxxGetDividerDrops());
    delayMicroseconds(15);

    // Gyro +/- 1000 DPS Full Scale
    itg1010WriteRegister(ITG_RA_GYRO_CONFIG, INV_FSR_2000DPS << 3);
    delayMicroseconds(15);

    itg1010WriteRegister(ITG_RA_INT_PIN_CFG, 0 << 7 | 0 << 6 | 0 << 5 | 1 << 4 | 0 << 3 | 0 << 2 | 0 << 1 | 0 << 0);  // INT_ANYRD_2CLEAR
    delayMicroseconds(15);

#ifdef USE_ITG_DATA_READY_SIGNAL
    itg1010WriteRegister(ITG_RA_INT_ENABLE, ITG_RF_DATA_RDY_EN);
    delayMicroseconds(15);
#endif

    spiSetDivisor(ITG1010_SPI_INSTANCE, SPI_CLOCK_FAST);
    delayMicroseconds(1);

    itg1010SpiInitDone = true;
}

bool itg1010SpiGyroDetect(gyro_t *gyro)
{
    if (itgDetectionResult.sensor != ITG_1010_SPI) {
        return false;
    }

    gyro->init = itg1010SpiGyroInit;
    gyro->read = itgGyroRead;
    gyro->intStatus = checkItgDataReady;
    // 16.4 dps/lsb scalefactor
    gyro->scale = 1.0f / 16.4f;

    return true;
}

#endif
