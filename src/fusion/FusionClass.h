#ifndef FUSIONCLASS_H_
#define FUSIONCLASS_H_

#include "Fusion.h"

// Use 2B pack, transmit 19B with the first one dropped
#pragma pack(2)
struct FusionReport {
	uint16_t q[4]; // w, x, y, z
	uint16_t a[3]; // x, y, z
};

class FusionClass {
public:
    FusionClass();

    FusionVector gyro;
    FusionVector accel;
    FusionVector mag;
    FusionQuaternion quat;
    FusionVector earth;

    void setup();
    void update();
    void getResult(FusionReport *obuf);

private:
    FusionOffset offset;
    FusionAhrs ahrs;
};


#endif // FUSIONCLASS_H_
