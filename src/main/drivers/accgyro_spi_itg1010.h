
#pragma once

#define ITG1010_CONFIG              0x1A

#define BITS_DLPF_CFG_250HZ         0x00
#define BITS_DLPF_CFG_184HZ         0x01
#define BITS_DLPF_CFG_92HZ          0x02
#define BITS_DLPF_CFG_41HZ          0x03

#define GYRO_SCALE_FACTOR  0.00053292f  // (4/131) * pi/180   (32.75 LSB = 1 DPS)

#define ITG1010_WHO_AM_I_CONST              (0x68)

// RF = Register Flag
#define ITG_RF_DATA_RDY_EN (1 << 0)

bool itg1010SpiDetect(void);

bool itg1010SpiGyroDetect(gyro_t *gyro);

bool itg1010WriteRegister(uint8_t reg, uint8_t data);
bool itg1010ReadRegister(uint8_t reg, uint8_t length, uint8_t *data);
