/*
    SlimeVR Code is placed under the MIT license
    Copyright (c) 2022 SlimeVR Contributors

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

#ifndef SENSORS_BMI160SENSOR_H
#define SENSORS_BMI160SENSOR_H

#include <BMI160.h>
#include <algorithm>
using namespace std;

#define CONST_EARTH_GRAVITY 9.80665f
#define PI 3.14159f

#define BMI160_GYRO_RATE BMI160_GYRO_RATE_200HZ
#define BMI160_GYRO_RANGE BMI160_GYRO_RANGE_1000
#define BMI160_GYRO_FILTER_MODE BMI160_DLPF_MODE_NORM

#define BMI160_ACCEL_RATE BMI160_ACCEL_RATE_200HZ
#define BMI160_ACCEL_RANGE BMI160_ACCEL_RANGE_4G
#define BMI160_ACCEL_FILTER_MODE BMI160_DLPF_MODE_NORM

// note: if changing ODR or filter modes - adjust rest detection params and buffer size

#define BMI160_TIMESTAMP_RESOLUTION_MICROS 39.0625f
// #define BMI160_TIMESTAMP_RESOLUTION_MICROS 39.0f
#define BMI160_MAP_ODR_MICROS(micros) ((uint16_t)((micros) / BMI160_TIMESTAMP_RESOLUTION_MICROS) * BMI160_TIMESTAMP_RESOLUTION_MICROS)
constexpr float BMI160_ODR_GYR_HZ = 25.0f * (1 << (BMI160_GYRO_RATE - 6));
constexpr float BMI160_ODR_ACC_HZ = 12.5f * (1 << (BMI160_ACCEL_RATE - 5));
constexpr float BMI160_ODR_GYR_MICROS = BMI160_MAP_ODR_MICROS(1.0f / BMI160_ODR_GYR_HZ * 1e6f);
constexpr float BMI160_ODR_ACC_MICROS = BMI160_MAP_ODR_MICROS(1.0f / BMI160_ODR_ACC_HZ * 1e6f);
#define BMI160_MAG_RATE BMI160_MAG_RATE_200HZ
constexpr float BMI160_ODR_MAG_HZ = (25.0f/32.0f) * (1 << (BMI160_MAG_RATE - 1));
constexpr float BMI160_ODR_MAG_MICROS = BMI160_MAP_ODR_MICROS(1.0f / BMI160_ODR_MAG_HZ * 1e6f);

constexpr uint16_t BMI160_SETTINGS_MAX_ODR_HZ = max(max(BMI160_ODR_GYR_HZ, BMI160_ODR_ACC_HZ), BMI160_ODR_MAG_HZ);
constexpr uint16_t BMI160_SETTINGS_MAX_ODR_MICROS = BMI160_MAP_ODR_MICROS(1.0f / BMI160_SETTINGS_MAX_ODR_HZ * 1e6f);

constexpr float BMI160_FIFO_AVG_DATA_FRAME_LENGTH = (
    BMI160_SETTINGS_MAX_ODR_HZ * 1 +
    BMI160_ODR_GYR_HZ * BMI160_FIFO_G_LEN +
    BMI160_ODR_ACC_HZ * BMI160_FIFO_A_LEN +
    BMI160_ODR_MAG_HZ * BMI160_FIFO_M_LEN
) / BMI160_SETTINGS_MAX_ODR_HZ;
constexpr float BMI160_FIFO_READ_BUFFER_SIZE_MICROS = 30000;
constexpr float BMI160_FIFO_READ_BUFFER_SIZE_SAMPLES =
    BMI160_SETTINGS_MAX_ODR_HZ * BMI160_FIFO_READ_BUFFER_SIZE_MICROS / 1e6f;
constexpr uint16_t BMI160_FIFO_MAX_LENGTH = 1024;
constexpr uint16_t BMI160_FIFO_READ_BUFFER_SIZE_BYTES = min(
    (float)BMI160_FIFO_MAX_LENGTH - 64,
    BMI160_FIFO_READ_BUFFER_SIZE_SAMPLES * BMI160_FIFO_AVG_DATA_FRAME_LENGTH * 1.25f
);

// Typical sensitivity at 25C
// See p. 9 of https://www.mouser.com/datasheet/2/783/BST-BMI160-DS000-1509569.pdf
// #define BMI160_GYRO_TYPICAL_SENSITIVITY_LSB 16.4f  // 2000 deg  0
// #define BMI160_GYRO_TYPICAL_SENSITIVITY_LSB 32.8f  // 1000 deg  1
// #define BMI160_GYRO_TYPICAL_SENSITIVITY_LSB 65.6f  // 500 deg   2
// #define BMI160_GYRO_TYPICAL_SENSITIVITY_LSB 131.2f // 250 deg   3 
// #define BMI160_GYRO_TYPICAL_SENSITIVITY_LSB 262.4f // 125 deg   4
constexpr double BMI160_GYRO_TYPICAL_SENSITIVITY_LSB = (16.4f * (1 << BMI160_GYRO_RANGE));

constexpr std::pair<uint8_t, float> BMI160_ACCEL_SENSITIVITY_LSB_MAP[] = {
    {BMI160_ACCEL_RANGE_2G, 16384.0f},
    {BMI160_ACCEL_RANGE_4G, 8192.0f},
    {BMI160_ACCEL_RANGE_8G, 4096.0f},
    {BMI160_ACCEL_RANGE_16G, 2048.0f}
};
constexpr double BMI160_ACCEL_TYPICAL_SENSITIVITY_LSB = BMI160_ACCEL_SENSITIVITY_LSB_MAP[BMI160_ACCEL_RANGE / 4].second;
constexpr double BMI160_ASCALE = CONST_EARTH_GRAVITY / BMI160_ACCEL_TYPICAL_SENSITIVITY_LSB;

// Scale conversion steps: LSB/°/s -> °/s -> step/°/s -> step/rad/s
constexpr double BMI160_GSCALE = ((32768. / BMI160_GYRO_TYPICAL_SENSITIVITY_LSB) / 32768.) * (PI / 180.0);

constexpr float targetSampleRateMs = 10.0f;
constexpr uint32_t targetSampleRateMicros = (uint32_t)targetSampleRateMs * 1e3;

class BMI160Sensor{
    public:
        BMI160Sensor(uint8_t devid) : devid(devid) {};
        ~BMI160Sensor(){};
        void initQMC(BMI160MagRate magRate);

        void motionSetup();
        void motionLoop();
    private:
        void readFIFO();

        BMI160 imu {};
        uint8_t devid;

        float qwxyz[4] {1.0f, 0.0f, 0.0f, 0.0f};

        struct BMI160FIFO {
            uint8_t data[BMI160_FIFO_READ_BUFFER_SIZE_BYTES];
            uint16_t length;
        } fifo {};
        float Gxyz[3] = {0};
        float Axyz[3] = {0};
        float Mxyz[3] = {0};
        float lastAxyz[3] = {0};
        bool fusionUpdated = false;
};

#endif
