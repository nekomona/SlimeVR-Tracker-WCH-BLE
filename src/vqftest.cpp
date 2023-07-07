#include <CH58x_common.h>
#include <vqf.h>
#include <Fusion.h>

struct BMI160VQFParams: VQFParams {
    BMI160VQFParams() : VQFParams() {
        #ifndef VQF_NO_MOTION_BIAS_ESTIMATION
        motionBiasEstEnabled = false;
        #endif
        tauAcc = 2.0f;
        restMinT = 2.0f;
        restThGyr = 0.6f; // 400 norm
        restThAcc = 0.06f; // 100 norm
    }
};

BMI160VQFParams vqfParams {};
VQF vqf(vqfParams, 0.01f, 0.01f, 0.01f);

float fake_gyr[3] = {0.0f, 0.0f, 0.1f};
float fake_acc[3] = {0.54f, 0.72f, 0.0f};
float fake_mag[3] = {0.3f, 0.4f, 0.1f};
float quat_out[4];

// Define calibration (replace with actual calibration data if available)
FusionMatrix gyroscopeMisalignment = {1.0f, 0.1f, 0.1f, 0.1f, 1.0f, 0.1f, 0.1f, 0.1f, 1.0f};
FusionVector gyroscopeSensitivity = {1.0f, 1.0f, 1.1f};
FusionVector gyroscopeOffset = {0.1f, 0.1f, 0.1f};
FusionMatrix accelerometerMisalignment = {1.0f, 0.1f, 0.1f, 0.1f, 1.0f, 0.1f, 0.1f, 0.1f, 1.0f};
FusionVector accelerometerSensitivity = {1.0f, 1.1f, 1.0f};
FusionVector accelerometerOffset = {0.1f, 0.1f, 0.1f};
FusionMatrix softIronMatrix = {1.0f, 0.1f, 0.1f, 0.1f, 1.0f, 0.1f, 0.1f, 0.1f, 1.0f};
FusionVector hardIronOffset = {0.1f, 0.1f, 0.1f};

// Initialise algorithms
FusionOffset offset;
FusionAhrs ahrs;

extern "C"
void vqftest_setup() {

    FusionOffsetInitialise(&offset, 100);
    FusionAhrsInitialise(&ahrs);

    // Set AHRS algorithm settings
    const FusionAhrsSettings settings = {
            .convention = FusionConventionNwu,
            .gain = 0.5f,
            .accelerationRejection = 10.0f,
            .magneticRejection = 20.0f,
            .rejectionTimeout = 5 * 100, /* 5 seconds */
    };
    FusionAhrsSetSettings(&ahrs, &settings);


    vqf.resetState();

    GPIOB_ResetBits(GPIO_Pin_4 | GPIO_Pin_1 | GPIO_Pin_3);
    GPIOB_ModeCfg(GPIO_Pin_4 | GPIO_Pin_1 | GPIO_Pin_2, GPIO_ModeOut_PP_20mA);
}

extern "C"
void vqftest_run() {
    GPIOB_SetBits(GPIO_Pin_4 | GPIO_Pin_1);
    vqf.updateGyr(fake_gyr, 0.01f);
    GPIOB_SetBits(GPIO_Pin_2);
    vqf.updateGyr(fake_gyr, 0.01f);
    vqf.updateGyr(fake_gyr, 0.01f);
    GPIOB_ResetBits(GPIO_Pin_2);
    vqf.updateGyr(fake_gyr, 0.01f);
    GPIOB_SetBits(GPIO_Pin_2);
    vqf.updateAcc(fake_acc);
    GPIOB_ResetBits(GPIO_Pin_2);
    vqf.updateMag(fake_mag);
    GPIOB_SetBits(GPIO_Pin_2);
    vqf.getQuat9D(quat_out);
    GPIOB_ResetBits(GPIO_Pin_4 | GPIO_Pin_1 | GPIO_Pin_2);



    FusionVector gyroscope = {0.0f, 0.0f, 0.1f}; // replace this with actual gyroscope data in degrees/s
    FusionVector accelerometer = {0.54f, 0.72f, 0.0f}; // replace this with actual accelerometer data in g
    FusionVector magnetometer = {0.3f, 0.4f, 0.1f}; // replace this with actual magnetometer data in arbitrary units

    // Apply calibration
    gyroscope = FusionCalibrationInertial(gyroscope, gyroscopeMisalignment, gyroscopeSensitivity, gyroscopeOffset);
    accelerometer = FusionCalibrationInertial(accelerometer, accelerometerMisalignment, accelerometerSensitivity, accelerometerOffset);
    magnetometer = FusionCalibrationMagnetic(magnetometer, softIronMatrix, hardIronOffset);

    GPIOB_SetBits(GPIO_Pin_2);

    // Update gyroscope offset correction algorithm
    gyroscope = FusionOffsetUpdate(&offset, gyroscope);

    GPIOB_ResetBits(GPIO_Pin_2);

    // Update gyroscope AHRS algorithm
    FusionAhrsUpdate(&ahrs, gyroscope, accelerometer, magnetometer, 0.01f);

    GPIOB_SetBits(GPIO_Pin_2);

    // Print algorithm outputs
    const FusionQuaternion quat = FusionAhrsGetQuaternion(&ahrs);
    const FusionVector earth = FusionAhrsGetEarthAcceleration(&ahrs);
    
    GPIOB_ResetBits(GPIO_Pin_2);
}