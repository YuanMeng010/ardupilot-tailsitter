/// @file	AP_MotorsTailsitter.h
/// @brief	Motor control class for tailsitters and bicopters
#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include "AP_MotorsMulticopter.h"

/// @class      AP_MotorsTailsitter
class AP_MotorsTailsitter : public AP_MotorsMulticopter {
public:

    // 2.1 横滚矢量增益：0 关闭，1 满舵
    void set_roll_vect_gain(float gain) { _roll_vect_gain = gain; }

    /// Constructor
    AP_MotorsTailsitter(uint16_t speed_hz = AP_MOTORS_SPEED_DEFAULT);

    // init
    void init(motor_frame_class frame_class, motor_frame_type frame_type) override;

    // set frame class (i.e. quad, hexa, heli) and type (i.e. x, plus)
    void set_frame_class_and_type(motor_frame_class frame_class, motor_frame_type frame_type) override {}

    // set update rate to motors - a value in hertz
    void set_update_rate( uint16_t speed_hz ) override;

    // output_to_motors - sends output to named servos
    void output_to_motors() override;

    // get_motor_mask - returns a bitmask of which outputs are being used for motors or servos (1 means being used)
    //  this can be used to ensure other pwm outputs (i.e. for servos) do not conflict
    uint32_t get_motor_mask() override;

    // output_motor_mask is not supported by the tailsitter backend, override to ensure no action.
    void output_motor_mask(float thrust, uint32_t mask, float rudder_dt) override {};

    // Set by tailsitters using diskloading minumum outflow velocity limit
    void set_min_throttle(float val) {_external_min_throttle = val;}

protected:
    // 2.2横滚增益
    float _roll_vect_gain = 1.0f;

    // calculate motor outputs
    void output_armed_stabilizing() override;

    const char* _get_frame_string() const override { return "TAILSITTER"; }

    // spin a motor at the pwm value specified
    void _output_test_seq(uint8_t motor_seq, int16_t pwm) override;

    // calculated outputs
    float _throttle; // 0..1
    float _tilt_left;  // -1..1
    float _tilt_right;  // -1..1
    float _tilt_l2;
    float _tilt_r2;
    float _thrust_left;  // 0..1
    float _thrust_right;  // 0..1

    // Set by tailsitters using diskloading minumum outflow velocity limit
    float _external_min_throttle;

    // true if differential thrust is available
    bool _has_diff_thrust;

};
