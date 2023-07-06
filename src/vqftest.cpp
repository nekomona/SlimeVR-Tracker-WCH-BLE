#include <CH58x_common.h>
#include <vqf.h>

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

extern "C"
void vqftest_setup() {
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
}