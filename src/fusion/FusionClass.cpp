#include "fusion/FusionClass.h"

// Initialise algorithms

FusionClass::FusionClass() {
    setup();
}

void FusionClass::setup() {
    FusionOffsetInitialise(&offset, 200);

    // Set AHRS algorithm settings
    const FusionAhrsSettings settings = {
            .convention = FusionConventionNwu,
            .gain = 0.5f,
            .accelerationRejection = 10.0f,
            .magneticRejection = 20.0f,
            .rejectionTimeout = 5 * 100, /* 5 seconds */
    };
    FusionAhrsSetSettings(&ahrs, &settings);
    FusionAhrsReset(&ahrs);
}

void FusionClass::update() {
    // Update gyroscope AHRS algorithm
    // FusionAhrsUpdateNoMagnetometer(&ahrs, FusionOffsetUpdate(&offset, gyro), accel, (1.0f / 200.0f) );
    FusionAhrsUpdate(&ahrs, FusionOffsetUpdate(&offset, gyro), accel, mag, (1.0f / 200.0f) );
}

#define INT16_TO_UINT16(x) ((uint16_t)32768 + (uint16_t)(x))
#define TO_FIXED_14(x) ((int16_t)((x) * (1 << 14)))
#define TO_FIXED_10(x) ((int16_t)((x) * (1 << 10)))

void FusionClass::getResult(FusionReport *obuf) {
    // Print algorithm outputs
    // const FusionQuaternion quat = FusionAhrsGetQuaternion(&ahrs);
    const FusionVector earth = FusionAhrsGetLinearAcceleration(&ahrs);

	obuf->q[0] = INT16_TO_UINT16(TO_FIXED_14(ahrs.quaternion.element.w));
	obuf->q[1] = INT16_TO_UINT16(TO_FIXED_14(ahrs.quaternion.element.x));
	obuf->q[2] = INT16_TO_UINT16(TO_FIXED_14(ahrs.quaternion.element.y));
	obuf->q[3] = INT16_TO_UINT16(TO_FIXED_10(ahrs.quaternion.element.z));
	obuf->a[0] = INT16_TO_UINT16(TO_FIXED_10(earth.axis.x));
	obuf->a[1] = INT16_TO_UINT16(TO_FIXED_10(earth.axis.y));
	obuf->a[2] = INT16_TO_UINT16(TO_FIXED_10(earth.axis.z));
}