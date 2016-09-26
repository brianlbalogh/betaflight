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

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "platform.h"
#include "build_config.h"
#include "debug.h"

#include "common/maths.h"

#include "nvic.h"

#include "system.h"
#include "gpio.h"
#include "exti.h"

#include "sensor.h"
#include "accgyro.h"
#include "accgyro_mpu.h"
#include "accgyro_itg.h"
#include "accgyro_spi_itg1010.h"

//#define DEBUG_MPU_DATA_READY_INTERRUPT

static volatile bool itgDataReady;

#ifdef USE_SPI
static bool detectSPISensorsAndUpdateDetectionResult(void);
#endif

itgDetectionResult_t itgDetectionResult;

itgConfiguration_t itgConfiguration;
static const extiConfig_t *itgIntExtiConfig = NULL;

#define ITG_ADDRESS             0x68

// WHO_AM_I register contents for ITG1010
#define ITG1010_WHO_AM_I_CONST              (0x68)

#define ITG_INQUIRY_MASK   0x7E

itgDetectionResult_t *detectItg(const extiConfig_t *configToUse)
{
    memset(&itgDetectionResult, 0, sizeof(itgDetectionResult));
    memset(&itgConfiguration, 0, sizeof(itgConfiguration));

    itgIntExtiConfig = configToUse;

    // MPU datasheet specifies 30ms.
    delay(35);

    bool detectedSpiSensor = detectSPISensorsAndUpdateDetectionResult();
    UNUSED(detectedSpiSensor);

    return &itgDetectionResult;
}

static bool detectSPISensorsAndUpdateDetectionResult(void)
{
#ifdef USE_GYRO_SPI_ITG1010
    if (itg1010SpiDetect()) {
        itgDetectionResult.sensor = ITG_1010_SPI;
        itgConfiguration.gyroReadXRegister = ITG_RA_GYRO_XOUT_H;
        itgConfiguration.read = itg1010ReadRegister;
        itgConfiguration.write = itg1010WriteRegister;
        return true;
    }
#endif

    return false;
}

extiCallbackRec_t itgIntCallbackRec;

void itgIntExtiHandler(extiCallbackRec_t *cb)
{
    UNUSED(cb);
    itgDataReady = true;

#ifdef DEBUG_ITG_DATA_READY_INTERRUPT
    static uint32_t lastCalledAt = 0;
    uint32_t now = micros();
    uint32_t callDelta = now - lastCalledAt;
    debug[0] = callDelta;
    lastCalledAt = now;
#endif
}

void itgIntExtiInit(void)
{
    static bool itgExtiInitDone = false;

    if (itgExtiInitDone || !itgIntExtiConfig) {
        return;
    }

#if defined(USE_ITG_DATA_READY_SIGNAL) && defined(USE_EXTI)

    IO_t itgIntIO = IOGetByTag(itgIntExtiConfig->tag);

#ifdef ENSURE_ITG_DATA_READY_IS_LOW
    uint8_t status = IORead(itgIntIO);
    if (status) {
        return;
    }
#endif

    IOInit(itgIntIO, OWNER_MPU, RESOURCE_EXTI, 0);
    IOConfigGPIO(itgIntIO, IOCFG_IN_FLOATING);   // TODO - maybe pullup / pulldown ?

    EXTIHandlerInit(&itgIntCallbackRec, itgIntExtiHandler);
    EXTIConfig(itgIntIO, &itgIntCallbackRec, NVIC_PRIO_MPU_INT_EXTI, EXTI_Trigger_Rising);
    EXTIEnable(itgIntIO, true);
#endif

    itgExtiInitDone = true;
}

bool itgGyroRead(int16_t *gyroADC)
{
    uint8_t data[6];

    bool ack = itgConfiguration.read(itgConfiguration.gyroReadXRegister, 6, data);
    if (!ack) {
        return false;
    }

    gyroADC[0] = (int16_t)((data[0] << 8) | data[1]);
    gyroADC[1] = (int16_t)((data[2] << 8) | data[3]);
    gyroADC[2] = (int16_t)((data[4] << 8) | data[5]);

    return true;
}

bool checkItgDataReady(void)
{
    bool ret;
    if (itgDataReady) {
        ret = true;
        itgDataReady= false;
    } else {
        ret = false;
    }
    return ret;
}
