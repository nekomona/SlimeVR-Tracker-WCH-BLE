/*
    SlimeVR Code is placed under the MIT license
    Copyright (c) 2021 S.J. Remington & SlimeVR contributors

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in
    all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
    THE SOFTWARE.
*/

#include "sensor/bmi160sensor.h"
#include "sensor/qmc5883l.h"

#include <CH58x_common.h>

#define delay DelayMs
#define delayMicroseconds DelayUs

void BMI160Sensor::initQMC(BMI160MagRate magRate) {
    /* Configure MAG interface and setup mode */
    /* Set MAG interface normal power mode */
    imu.setRegister(BMI160_RA_CMD, BMI160_CMD_MAG_MODE_NORMAL);
    delay(60);

    imu.setRegister(BMI160_RA_CMD, BMI160_EN_PULL_UP_REG_1);
    imu.setRegister(BMI160_RA_CMD, BMI160_EN_PULL_UP_REG_2);
    imu.setRegister(BMI160_RA_CMD, BMI160_EN_PULL_UP_REG_3);
    imu.setRegister(BMI160_7F, BMI160_EN_PULL_UP_REG_4);
    imu.setRegister(BMI160_7F, BMI160_EN_PULL_UP_REG_5);

    /* Enable MAG interface */
    imu.setRegister(BMI160_RA_IF_CONF, BMI160_IF_CONF_MODE_PRI_AUTO_SEC_MAG | BMI160_IF_CONF_SPI3);
    delay(1);

    imu.setMagDeviceAddress(QMC_DEVADDR);
    delay(3);
    imu.setRegister(BMI160_RA_MAG_IF_1_MODE, BMI160_MAG_SETUP_MODE);
    delay(3);

    /* Configure QMC5883L Sensor */
    imu.setMagRegister(QMC_RA_RESET, 1);
    delay(3);
    imu.setMagRegister(QMC_RA_CONTROL, QMC_CFG_MODE_CONTINUOUS | QMC_CFG_ODR_200HZ | QMC_CFG_RNG_8G | QMC_CFG_OSR_512);

    imu.setRegister(BMI160_RA_MAG_IF_2_READ_RA, QMC_RA_DATA);
    imu.setRegister(BMI160_RA_MAG_CONF, magRate);
    delay(3);
    imu.setRegister(BMI160_RA_MAG_IF_1_MODE, BMI160_MAG_DATA_MODE_6);
}

void BMI160Sensor::motionSetup() {
    // initialize device
    imu.getDeviceID();

    imu.initialize(
        devid,
        BMI160_GYRO_RATE,
        BMI160_GYRO_RANGE,
        BMI160_GYRO_FILTER_MODE,
        BMI160_ACCEL_RATE,
        BMI160_ACCEL_RANGE,
        BMI160_ACCEL_FILTER_MODE
    );

    initQMC(BMI160_MAG_RATE);

    
    if (!imu.testConnection()) {
        PRINT("Can't connect to BMI160 (reported device ID 0x%02x) at address 0x%02x", imu.getDeviceID(), devid);
        return;
    }

    PRINT("Connected to BMI160 (reported device ID 0x%02x) at address 0x%02x", imu.getDeviceID(), devid);


    imu.setFIFOHeaderModeEnabled(false);
    imu.setGyroFIFOEnabled(true);
    imu.setAccelFIFOEnabled(true);
    imu.setMagFIFOEnabled(true);
    delay(4);
    imu.resetFIFO();
    delay(2);

    uint8_t err;
    if (imu.getErrReg(&err)) {
        if (err & BMI160_ERR_MASK_CHIP_NOT_OPERABLE) {
            PRINT("Fatal error: chip not operable");
            return;
        } else if (err & BMI160_ERR_MASK_ERROR_CODE) {
            PRINT("Error code 0x%02x", err);
        } else {
            PRINT("Initialized");
        }
    } else {
        PRINT("Failed to get error register value");
    }

    fusion.setup();
}

#define FIFO_PACKET_LEN 20

void BMI160Sensor::motionLoop() {
    readFIFO();

    for (int i = 0; i < fifo.length; i += FIFO_PACKET_LEN) {
        BMI160Packet9Axis *pack = (BMI160Packet9Axis *) &fifo.data[i];

        fusion.mag.axis.x = pack->mag[0];
        fusion.mag.axis.y = pack->mag[1];
        fusion.mag.axis.z = pack->mag[2];
        fusion.gyro.axis.x = pack->gyro[0] * BMI160_GSCALE;
        fusion.gyro.axis.y = pack->gyro[1] * BMI160_GSCALE;
        fusion.gyro.axis.z = pack->gyro[2] * BMI160_GSCALE;
        fusion.accel.axis.x = pack->accel[0] * BMI160_ASCALE;
        fusion.accel.axis.y = pack->accel[1] * BMI160_ASCALE;
        fusion.accel.axis.z = pack->accel[2] * BMI160_ASCALE;

        fusion.update();
    }
    fifo.length = 0;
}

void BMI160Sensor::readFIFO() {
    if (!imu.getFIFOCount(&fifo.length)) {
        fifo.length = 0;
        return;
    }

    if (fifo.length <= 1) return;
    if (fifo.length > sizeof(fifo.data)) {
        imu.getFIFOBytes(fifo.data, FIFO_PACKET_LEN);
        fifo.length = FIFO_PACKET_LEN;
        imu.resetFIFO();
        return;
    }
    if (!imu.getFIFOBytes(fifo.data, fifo.length)) {
        return;
    }
}
