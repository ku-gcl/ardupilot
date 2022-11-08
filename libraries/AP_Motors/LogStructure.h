#pragma once

#include <AP_Logger/LogStructure.h>

#define LOG_IDS_FROM_MOTORS \
    LOG_MOT_MSG

// @LoggerMessage: THR
// @Description: Thrust input
// @Field: TimeUS: Time since system startup
// @Field: SampleUS: time since system startup this sample was taken
// @Field: RollThr: roll thrust input value, +/- 1.0
// @Field: PitchThr: pitch thrust input value, +/- 1.0
// @Field: YawThr: yaw thrust input value, +/- 1.0
// @Field: ThroThr: throttle thrust input value, 0.0 - 1.0
// @Field: CompGain: compensation for battery voltage and altitude
struct PACKED log_THRUST {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint64_t sample_us;
    int16_t roll_thrust;
    int16_t pitch_thrust;
    int16_t yaw_thrust;
    int16_t throttle_thrust;
    int16_t compensation_gain;
};

#define LOG_STRUCTURE_FROM_MOTROS
{LOG_MOT_MSG, sizeof(log_THRUST), 
 "THR", "QQhhhhh", "TimeUS,SampleUS,RollThr,PitchThr,YawThr,ThroThr,CompGain", "ss-----", "FF-----", true},