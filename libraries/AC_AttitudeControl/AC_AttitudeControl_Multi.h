#pragma once

/// @file    AC_AttitudeControl_Multi.h
/// @brief   ArduCopter attitude control library

#include "AC_AttitudeControl.h"
#include <AP_Motors/AP_MotorsMulticopter.h>

// default rate controller PID gains
#ifndef AC_ATC_MULTI_RATE_RP_P
  # define AC_ATC_MULTI_RATE_RP_P           0.135f
#endif
#ifndef AC_ATC_MULTI_RATE_RP_I
  # define AC_ATC_MULTI_RATE_RP_I           0.135f
#endif
#ifndef AC_ATC_MULTI_RATE_RP_D
  # define AC_ATC_MULTI_RATE_RP_D           0.0036f
#endif
#ifndef AC_ATC_MULTI_RATE_RP_IMAX
 # define AC_ATC_MULTI_RATE_RP_IMAX         0.5f
#endif
#ifndef AC_ATC_MULTI_RATE_RPY_FILT_HZ
 # define AC_ATC_MULTI_RATE_RPY_FILT_HZ      20.0f
#endif
#ifndef AC_ATC_MULTI_RATE_YAW_P
 # define AC_ATC_MULTI_RATE_YAW_P           0.180f
#endif
#ifndef AC_ATC_MULTI_RATE_YAW_I
 # define AC_ATC_MULTI_RATE_YAW_I           0.018f
#endif
#ifndef AC_ATC_MULTI_RATE_YAW_D
 # define AC_ATC_MULTI_RATE_YAW_D           0.0f
#endif
#ifndef AC_ATC_MULTI_RATE_YAW_IMAX
 # define AC_ATC_MULTI_RATE_YAW_IMAX        0.5f
#endif
#ifndef AC_ATC_MULTI_RATE_YAW_FILT_HZ
 # define AC_ATC_MULTI_RATE_YAW_FILT_HZ     2.5f
#endif


class AC_AttitudeControl_Multi : public AC_AttitudeControl {
public:
	AC_AttitudeControl_Multi(AP_AHRS_View &ahrs, const AP_MultiCopter &aparm, AP_MotorsMulticopter& motors);

	// empty destructor to suppress compiler warning
	virtual ~AC_AttitudeControl_Multi() {}

    // pid accessors
    AC_PID& get_rate_roll_pid() override { return _pid_rate_roll; }
    AC_PID& get_rate_pitch_pid() override { return _pid_rate_pitch; }
    AC_PID& get_rate_yaw_pid() override { return _pid_rate_yaw; }
    const AC_PID& get_rate_roll_pid() const override { return _pid_rate_roll; }
    const AC_PID& get_rate_pitch_pid() const override { return _pid_rate_pitch; }
    const AC_PID& get_rate_yaw_pid() const override { return _pid_rate_yaw; }

    // Update Alt_Hold angle maximum
    void update_althold_lean_angle_max(float throttle_in) override;

    // Set output throttle
    void set_throttle_out(float throttle_in, bool apply_angle_boost, float filt_cutoff) override;

    // calculate total body frame throttle required to produce the given earth frame throttle
    float get_throttle_boosted(float throttle_in);

    // set desired throttle vs attitude mixing (actual mix is slewed towards this value over 1~2 seconds)
    //  low values favour pilot/autopilot throttle over attitude control, high values favour attitude control over throttle
    //  has no effect when throttle is above hover throttle
    void set_throttle_mix_min() override { _throttle_rpy_mix_desired = _thr_mix_min; }
    void set_throttle_mix_man() override { _throttle_rpy_mix_desired = _thr_mix_man; }
    void set_throttle_mix_max(float ratio) override;
    void set_throttle_mix_value(float value) override { _throttle_rpy_mix_desired = _throttle_rpy_mix = value; }
    float get_throttle_mix(void) const override { return _throttle_rpy_mix; }

    // are we producing min throttle?
    bool is_throttle_mix_min() const override { return (_throttle_rpy_mix < 1.25f * _thr_mix_min); }

    // run lowest level body-frame rate controller and send outputs to the motors
    void rate_controller_run_dt(const Vector3f& gyro, float dt) override;
    void rate_controller_target_reset() override;
    void rate_controller_run() override;

    // sanity check parameters.  should be called once before take-off
    void parameter_sanity_check() override;

    // set the PID notch sample rates
    void set_notch_sample_rate(float sample_rate) override;

    // enable/disable strain-based inner acceleration loop calculations
    void set_strain_inner_loop_enabled(bool enabled) override { _strain_inner_loop_enabled = enabled; }
    bool get_strain_inner_loop_enabled() const override { return _strain_inner_loop_enabled; }

    // enable/disable using strain output for motor control (calculations still happen when disabled for logging)
    void set_use_strain_output(bool use_output) override { _use_strain_output = use_output; }
    bool get_use_strain_output() const override { return _use_strain_output; }

    // update strain acceleration feedback (rad/s^2)
    void set_strain_acceleration(float roll_accel, float pitch_accel, uint32_t timestamp_ms) override {
        _strain_roll_accel = roll_accel;
        _strain_pitch_accel = pitch_accel;
        _strain_last_update_ms = timestamp_ms;
    }

    // get strain inner loop outputs for logging
    float get_strain_roll_target() const override { return _strain_roll_target; }
    float get_strain_pitch_target() const override { return _strain_pitch_target; }
    float get_strain_roll_output() const override { return _strain_roll_output; }
    float get_strain_pitch_output() const override { return _strain_pitch_output; }

    // user settable parameters
    static const struct AP_Param::GroupInfo var_info[];

protected:

    // boost angle_p/pd each cycle on high throttle slew
    void update_throttle_gain_boost();

    // update_throttle_rpy_mix - updates thr_low_comp value towards the target
    void update_throttle_rpy_mix();

    // get maximum value throttle can be raised to based on throttle vs attitude prioritisation
    float get_throttle_avg_max(float throttle_in);

    AP_MotorsMulticopter& _motors_multi;
    AC_PID                _pid_rate_roll {
        AC_PID::Defaults{
            .p         = AC_ATC_MULTI_RATE_RP_P,
            .i         = AC_ATC_MULTI_RATE_RP_I,
            .d         = AC_ATC_MULTI_RATE_RP_D,
            .ff        = 0.0f,
            .imax      = AC_ATC_MULTI_RATE_RP_IMAX,
            .filt_T_hz = AC_ATC_MULTI_RATE_RPY_FILT_HZ,
            .filt_E_hz = 0.0f,
            .filt_D_hz = AC_ATC_MULTI_RATE_RPY_FILT_HZ,
            .srmax     = 0,
            .srtau     = 1.0
        }
    };
    AC_PID                _pid_rate_pitch{
        AC_PID::Defaults{
            .p         = AC_ATC_MULTI_RATE_RP_P,
            .i         = AC_ATC_MULTI_RATE_RP_I,
            .d         = AC_ATC_MULTI_RATE_RP_D,
            .ff        = 0.0f,
            .imax      = AC_ATC_MULTI_RATE_RP_IMAX,
            .filt_T_hz = AC_ATC_MULTI_RATE_RPY_FILT_HZ,
            .filt_E_hz = 0.0f,
            .filt_D_hz = AC_ATC_MULTI_RATE_RPY_FILT_HZ,
            .srmax     = 0,
            .srtau     = 1.0
        }
    };

    AC_PID                _pid_rate_yaw{
        AC_PID::Defaults{
            .p         = AC_ATC_MULTI_RATE_YAW_P,
            .i         = AC_ATC_MULTI_RATE_YAW_I,
            .d         = AC_ATC_MULTI_RATE_YAW_D,
            .ff        = 0.0f,
            .imax      = AC_ATC_MULTI_RATE_YAW_IMAX,
            .filt_T_hz = AC_ATC_MULTI_RATE_RPY_FILT_HZ,
            .filt_E_hz = AC_ATC_MULTI_RATE_YAW_FILT_HZ,
            .filt_D_hz = AC_ATC_MULTI_RATE_RPY_FILT_HZ,
            .srmax     = 0,
            .srtau     = 1.0
        }
    };

    // Strain-based inner loop PID controllers
    AC_PID                _pid_strain_roll{
        AC_PID::Defaults{
            .p         = 0.01f,
            .i         = 0.005f,
            .d         = 0.0f,
            .ff        = 0.0f,
            .imax      = 0.1f,
            .filt_T_hz = 20.0f,
            .filt_E_hz = 0.0f,
            .filt_D_hz = 20.0f,
            .srmax     = 0,
            .srtau     = 1.0
        }
    };
    AC_PID                _pid_strain_pitch{
        AC_PID::Defaults{
            .p         = 0.01f,
            .i         = 0.005f,
            .d         = 0.0f,
            .ff        = 0.0f,
            .imax      = 0.1f,
            .filt_T_hz = 20.0f,
            .filt_E_hz = 0.0f,
            .filt_D_hz = 20.0f,
            .srmax     = 0,
            .srtau     = 1.0
        }
    };

    AP_Float              _thr_mix_man;     // throttle vs attitude control prioritisation used when using manual throttle (higher values mean we prioritise attitude control over throttle)
    AP_Float              _thr_mix_min;     // throttle vs attitude control prioritisation used when landing (higher values mean we prioritise attitude control over throttle)
    AP_Float              _thr_mix_max;     // throttle vs attitude control prioritisation used during active flight (higher values mean we prioritise attitude control over throttle)

    // angle_p/pd boost multiplier
    AP_Float              _throttle_gain_boost;

    // strain inner loop state
    bool                  _strain_inner_loop_enabled{false};  // enable calculations (for logging)
    bool                  _use_strain_output{false};          // use strain output for motor control
    float                 _strain_roll_target{0.0f};          // target roll angular acceleration from outer loop
    float                 _strain_pitch_target{0.0f};         // target pitch angular acceleration from outer loop
    float                 _strain_roll_output{0.0f};          // roll correction from strain inner loop
    float                 _strain_pitch_output{0.0f};         // pitch correction from strain inner loop
    float                 _strain_roll_accel{0.0f};           // measured roll angular acceleration from strain (rad/s^2)
    float                 _strain_pitch_accel{0.0f};          // measured pitch angular acceleration from strain (rad/s^2)
    uint32_t              _strain_last_update_ms{0};          // timestamp of last strain data update
    uint32_t              _strain_last_pid_update_ms{0};      // timestamp of last PID update
    float                 _prev_roll_out{0.0f};               // previous time step's total roll output
    float                 _prev_pitch_out{0.0f};              // previous time step's total pitch output
};
