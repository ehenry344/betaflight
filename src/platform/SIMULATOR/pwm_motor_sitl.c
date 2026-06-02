
#include "drivers/motor.h"
#include "drivers/motor_impl.h"

#include "drivers/pwm_output.h"
#include "drivers/pwm_output_impl.h"

static int16_t motorsPwm[MAX_SUPPORTED_MOTORS];


pwmOutputPort_t *pwmGetMotors(void)
{
    return pwmMotors;
}

static float pwmConvertFromExternal(uint16_t externalValue)
{
    return (float)externalValue;
}

static uint16_t pwmConvertToExternal(float motorValue)
{
    return (uint16_t)motorValue;
}

static void pwmDisableMotors(void)
{
    // NOOP
}

static void pwmWriteMotor(uint8_t index, float value)
{
    if (index < MAX_SUPPORTED_MOTORS) {
        motorsPwm[index] = value - idlePulse;
    }

    if (index < pwmRawPkt.motorCount) {
        pwmRawPkt.pwm_output_raw[index] = value;
    }
}

static void pwmWriteMotorInt(uint8_t index, uint16_t value)
{
    pwmWriteMotor(index, (float)value);
}

static void pwmShutdownPulsesForAllMotors(void)
{
    // NOOP
}

static void pwmCompleteMotorUpdate(void)
{
    // send to simulator
    // for gazebo8 ArduCopterPlugin remap, normal range = [0.0, 1.0], 3D rang = [-1.0, 1.0]

    double outScale = 1000.0;
    if (featureIsEnabled(FEATURE_3D)) {
        outScale = 500.0;
    }

    for (int i = 0; i < 4; i++) {
        pwmPkt.motor_speed[i] = motorsPwm[i] / outScale;
    }

    // get one "fdm_packet" can only send one "servo_packet"!!
    if (pthread_mutex_trylock(&updateLock) != 0) return;
    udpSend(&pwmLink, &pwmPkt, sizeof(servo_packet));
    udpSend(&pwmRawLink, &pwmRawPkt, sizeof(servo_packet_raw));
}


static const motorVTable_t vTable = {
    .postInit = motorPostInitNull,
    .convertExternalToMotor = pwmConvertFromExternal,
    .convertMotorToExternal = pwmConvertToExternal,
    .enable = pwmEnableMotors,
    .disable = pwmDisableMotors,
    .write = pwmWriteMotor,
    .writeInt = pwmWriteMotorInt,
    .updateComplete = pwmCompleteMotorUpdate,
    .shutdown = pwmShutdownPulsesForAllMotors,
    .requestTelemetry = NULL,
    .isMotorIdle = NULL,
    .getMotorIO = NULL,
};
