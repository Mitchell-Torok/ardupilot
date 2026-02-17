#include "Copter.h"

#if MODE_TURTLE_ENABLED

#define CRASH_FLIP_EXPO 35.0f
#define CRASH_FLIP_STICK_MINF 0.15f
#define power3(x) ((x) * (x) * (x))

bool ModeTurtle::init(bool ignore_checks)
{
    // do not enter the mode when already armed or when flying
    if (motors->armed() || SRV_Channels::get_dshot_esc_type() == 0) {
        return false;
    }

    // perform minimal arming checks
    if (!copter.mavlink_motor_control_check(*gcs().chan(0), true, "Turtle Mode")) {
        return false;
    }


    // turn on notify leds
    AP_Notify::flags.esc_calibration = true;

    return true;
}

void ModeTurtle::arm_motors()
{
    if (hal.util->get_soft_armed()) {
        return;
    }

    motors->set_spoolup_block(false);
    hal.rcout->disable_channel_mask_updates();
    motors->armed(true);
    hal.util->set_soft_armed(true);
}

bool ModeTurtle::allows_arming(AP_Arming::Method method) const
{
    return true;
}

void ModeTurtle::exit()
{
    disarm_motors();
    AP_Notify::flags.esc_calibration = false;
}

void ModeTurtle::disarm_motors()
{
    if (!hal.util->get_soft_armed()) {
        return;
    }

    // disarm
    motors->armed(false);
    hal.util->set_soft_armed(false);
    hal.rcout->enable_channel_mask_updates();
}

void ModeTurtle::change_motor_direction(bool reverse)
{
    AP_HAL::RCOutput::BLHeliDshotCommand direction = reverse ? AP_HAL::RCOutput::DSHOT_REVERSE : AP_HAL::RCOutput::DSHOT_NORMAL;
    AP_HAL::RCOutput::BLHeliDshotCommand inverse_direction = reverse ? AP_HAL::RCOutput::DSHOT_NORMAL : AP_HAL::RCOutput::DSHOT_REVERSE;

    if (!hal.rcout->get_reversed_mask()) {
        hal.rcout->send_dshot_command(direction, AP_HAL::RCOutput::ALL_CHANNELS, 0, 10, true);
    } else {
        for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; ++i) {
            if (!motors->is_motor_enabled(i)) {
                continue;
            }

            if ((hal.rcout->get_reversed_mask() & (1U << i)) == 0) {
                hal.rcout->send_dshot_command(direction, i, 0, 10, true);
            } else {
                hal.rcout->send_dshot_command(inverse_direction, i, 0, 10, true);
            }
        }
    }
}

void ModeTurtle::run()
{



}



void ModeTurtle::output_to_motors()
{
    // 1. Physical Constants
    const float mass = 1.3f;      
    const float gravity_mss = 9.81f;
    const float kc = 1.2e-06f;
    const float max_omega = 1634.0f;


    if (copter.failsafe.radio || rc().channel(4)->get_radio_in() < 1800) {

        uint16_t neutral_pwm = 1500;
        motors->rc_write(0, neutral_pwm);
        motors->rc_write(1, neutral_pwm);
        motors->rc_write(2, neutral_pwm);
        motors->rc_write(3, neutral_pwm);


        gcs().send_text(MAV_SEVERITY_INFO, "Disarmed: failsafe=%d RC5=%d", copter.failsafe.radio, rc().channel(4)->get_radio_in());
        disarm_motors();
        return;
    }

    // 2. State & Safety
    const float pitch_deg = ahrs.get_pitch();
    const float roll_deg  = ahrs.get_roll();
    const float yaw_deg = ahrs.get_yaw();
    const Vector3f& gyro  = ahrs.get_gyro(); 

    const float pitch_rad = radians(pitch_deg);
    const float roll_rad  = radians(roll_deg);

    const float pitch_rate_dps = degrees(gyro.y); 
    const float roll_rate_dps  = degrees(gyro.x); 
    const float yaw_rate_dps = degrees(gyro.z);


    arm_motors();

    // Deadband helper lambda
    auto apply_deadband = [](float input, float deadband) {
        if (fabsf(input) < deadband) return 0.0f;
        return input;
    };

    // ---------------------------------------------------------
    // 3. PITCH AXIS CONTROL (Motors 1 & 4 / Index 0 & 3)
    // ---------------------------------------------------------
    float stick_pitch = rc().channel(1)->norm_input();
    stick_pitch = apply_deadband(stick_pitch, 0.05f);

    target_pitch_integrated += stick_pitch * 0.001f; 
    target_pitch_integrated = constrain_float(target_pitch_integrated, -60.0f, 60.0f);

    float force_p = mass * gravity_mss * sinf(pitch_rad);
    float omega_p_total = 0.0f;
    if (fabsf(force_p) > 0.0001f) {
        omega_p_total = sqrtf(fabsf(force_p) / kc) * (force_p > 0 ? 1.0f : -1.0f);
    }
    float omega_per_motor_p = omega_p_total / 1.4142f; 
    float ff_p = omega_per_motor_p / (max_omega * 2.0f);

    const float kp_p = 0.1f;
    const float kd_p = 0.01f;
    float pd_p = ((target_pitch_integrated - pitch_deg) * kp_p) + ((0.0f - pitch_rate_dps) * kd_p);

    float out_p = constrain_float(0.5f + ff_p + pd_p, 0.0f, 1.0f);
    
    // Counter-rotating pitch outputs (Index 0 CCW, Index 3 CW mirrored)
    float out_p_ccw = out_p;
    float out_p_cw  = 1.0f - out_p;

    // ---------------------------------------------------------
    // 4. ROLL & YAW AXIS CONTROL (Motors 2 & 3 / Index 1 & 2)
    // ---------------------------------------------------------
    float stick_roll = rc().channel(0)->norm_input();
    stick_roll = apply_deadband(stick_roll, 0.05f);

    target_roll_integrated += stick_roll * 0.001f; 
    target_roll_integrated = constrain_float(target_roll_integrated, -60.0f, 60.0f);

    float force_r = mass * gravity_mss * sinf(roll_rad);
    float omega_r_total = 0.0f;
    if (fabsf(force_r) > 0.0001f) {
        omega_r_total = sqrtf(fabsf(force_r) / kc) * (force_r > 0 ? 1.0f : -1.0f);
    }
    float omega_per_motor_r = omega_r_total / 1.4142f; 
    float ff_r = omega_per_motor_r / (max_omega * 2.0f);

    const float kp_r = 0.1f; 
    const float kd_r = 0.01f;
    float pd_r = ((target_roll_integrated - roll_deg) * kp_r) + ((0.0f - roll_rate_dps) * kd_r);

    float stick_yaw = rc().channel(3)->norm_input();
    stick_yaw = apply_deadband(stick_yaw, 0.05f);

    target_yaw_integrated += stick_yaw * 0.01f; 
    target_yaw_integrated = constrain_float(target_yaw_integrated, -60.0f, 60.0f);

    const float kp_y = 0.5f;
    const float kd_y = 0.01f;
    float pd_y = ((target_yaw_integrated - yaw_deg) * kp_y) + ((0.0f - yaw_rate_dps) * kd_y);

    float out_r = constrain_float(0.5f + ff_r + pd_r, 0.0f, 1.0f);
    float out_r_ccw = out_r - pd_y;           
    float out_r_cw  = 1.0f - out_r - pd_y;    

    // ---------------------------------------------------------
    // 5. OUTPUT TO MOTORS
    // ---------------------------------------------------------
    int16_t pwm_min = motors->get_pwm_output_min();
    int16_t pwm_range = motors->get_pwm_output_max() - pwm_min;

    // Pitch Motors
    motors->rc_write(0, pwm_min + (pwm_range * constrain_float(out_p_ccw , 0.0f, 1.0f)));
    motors->rc_write(1, pwm_min + (pwm_range * constrain_float(out_p_cw, 0.0f, 1.0f)));

    // Roll Motors
    motors->rc_write(2, pwm_min + (pwm_range * constrain_float(out_r_ccw, 0.0f, 1.0f)));
    motors->rc_write(3, pwm_min + (pwm_range * constrain_float(out_r_cw, 0.0f, 1.0f)));



    gcs().send_text(MAV_SEVERITY_INFO, "P:%.3f R:%.3f Y:%.3f", (double)pd_p, (double)pd_r, (double)pd_y);
}

#endif










//OMNICOPTER



/*


#include "Copter.h"

#if MODE_TURTLE_ENABLED

#define CRASH_FLIP_EXPO 35.0f
#define CRASH_FLIP_STICK_MINF 0.15f
#define power3(x) ((x) * (x) * (x))

bool ModeTurtle::init(bool ignore_checks)
{
    // do not enter the mode when already armed or when flying
    if (motors->armed() || SRV_Channels::get_dshot_esc_type() == 0) {
        return false;
    }

    // perform minimal arming checks
    if (!copter.mavlink_motor_control_check(*gcs().chan(0), true, "Turtle Mode")) {
        return false;
    }


    // turn on notify leds
    AP_Notify::flags.esc_calibration = true;

    return true;
}

void ModeTurtle::arm_motors()
{
    if (hal.util->get_soft_armed()) {
        return;
    }

    motors->set_spoolup_block(false);
    hal.rcout->disable_channel_mask_updates();
    motors->armed(true);
    hal.util->set_soft_armed(true);
}

bool ModeTurtle::allows_arming(AP_Arming::Method method) const
{
    return true;
}

void ModeTurtle::exit()
{
    disarm_motors();
    AP_Notify::flags.esc_calibration = false;
}

void ModeTurtle::disarm_motors()
{
    if (!hal.util->get_soft_armed()) {
        return;
    }

    // disarm
    motors->armed(false);
    hal.util->set_soft_armed(false);
    hal.rcout->enable_channel_mask_updates();
}

void ModeTurtle::change_motor_direction(bool reverse)
{
    AP_HAL::RCOutput::BLHeliDshotCommand direction = reverse ? AP_HAL::RCOutput::DSHOT_REVERSE : AP_HAL::RCOutput::DSHOT_NORMAL;
    AP_HAL::RCOutput::BLHeliDshotCommand inverse_direction = reverse ? AP_HAL::RCOutput::DSHOT_NORMAL : AP_HAL::RCOutput::DSHOT_REVERSE;

    if (!hal.rcout->get_reversed_mask()) {
        hal.rcout->send_dshot_command(direction, AP_HAL::RCOutput::ALL_CHANNELS, 0, 10, true);
    } else {
        for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; ++i) {
            if (!motors->is_motor_enabled(i)) {
                continue;
            }

            if ((hal.rcout->get_reversed_mask() & (1U << i)) == 0) {
                hal.rcout->send_dshot_command(direction, i, 0, 10, true);
            } else {
                hal.rcout->send_dshot_command(inverse_direction, i, 0, 10, true);
            }
        }
    }
}

void ModeTurtle::run()
{



}

// actually write values to the motors
void ModeTurtle::output_to_motors()
{


    // Print attitude (deg) and body rates (deg/s) using public getters
    const float roll  = ahrs.get_roll();
    const float pitch = ahrs.get_pitch();
    const float yaw   = ahrs.get_yaw();

    const Vector3f& gyro = ahrs.get_gyro();  // rad/s, body frame

    gcs().send_text( MAV_SEVERITY_INFO, "ATT R=%.1f P=%.1f Y=%.1f deg", (double)roll, (double)pitch, (double)yaw);
    gcs().send_text( MAV_SEVERITY_INFO, "GYRO p=%.1f q=%.1f r=%.1f dps", (double)gyro.x, (double)gyro.y, (double)gyro.z );



    // Print RSSI value
    int16_t rssi_value = hal.rcin->get_rssi();
    gcs().send_text(MAV_SEVERITY_INFO, "RSSI: %d", rssi_value);

    // Check for failsafe conditions
    if (copter.failsafe.radio) {
        //gcs().send_text(MAV_SEVERITY_WARNING, "Failsafe: Radio");
        disarm_motors();
        return;
    }

    const int16_t rc_channel_5_value = rc().channel(4)->get_radio_in(); // RC channel 5 corresponds to index 4 (0-based index)

    if (rc_channel_5_value < 1800) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Failsafe: RC Channel 5 Low");
        disarm_motors();
        return;
    }

    arm_motors();
    const bool allow_output = motors->armed() && motors->get_interlock();

    for (uint8_t i = 0; i < 8; ++i) {
        if (!motors->is_motor_enabled(i)) {
            continue;
        }
        if (!allow_output) {
            motors->rc_write(i, motors->get_pwm_output_min());
            continue;
        }

        // Map motor index to RC channel
        uint8_t rc_channel_index;
        if (i < 4) {
            rc_channel_index = i; // Motors 1-4: RC channels 0-3
        } else {
            rc_channel_index = i + 1; // Motors 5-8: RC channels 5-8
        }

        RC_Channel* rc_chan = rc().channel(rc_channel_index);
        if (rc_chan == nullptr) {
            motors->rc_write(i, motors->get_pwm_output_min());
            continue;
        }

        const int16_t rc_channel_i_value = rc_chan->get_radio_in();
        float normalized_input = (rc_channel_i_value - 988.0f) / (2012.0f - 988.0f);
        normalized_input = constrain_float(normalized_input, 0.0f, 1.0f);
        int16_t pwm = motors->get_pwm_output_min() + (motors->get_pwm_output_max() - motors->get_pwm_output_min()) * fabsf(normalized_input);

        motors->rc_write(i, pwm);
    }



     // ---- Cycle-time & Hz (timestamp recorded at END) ----
    //static uint32_t prev_us = 0;      // persists across calls
    //static float ema_hz = 0.0f;       // optional smoothing
    //const uint32_t now_us = AP_HAL::micros();

    //if (prev_us != 0) {
    //    const uint32_t dt_us = now_us - prev_us;           // uint32 wrap-safe
        //const float    hz    = (dt_us > 0) ? (1.0e6f / (float)dt_us) : 0.0f;

        // Simple EMA to steady the printout (tweak alpha or remove if not wanted)
        //const float alpha = 0.2f;
        //ema_hz = (ema_hz == 0.0f) ? hz : (alpha * hz + (1.0f - alpha) * ema_hz);

        //gcs().send_text( MAV_SEVERITY_INFO, "Loop: dt=%lu us  Hz=%.1f (EMA=%.1f)  RSSI=%d", (unsigned long)dt_us, (double)hz, (double)ema_hz, (int)rssi_value );
    //}
    //prev_us = now_us;


}

#endif
*/




/*


// actually write values to the motors
void ModeTurtle::output_to_motors()
{


    // Print attitude (deg) and body rates (deg/s) using public getters
    const float roll  = ahrs.get_roll();
    const float pitch = ahrs.get_pitch();
    const float yaw   = ahrs.get_yaw();

    const Vector3f& gyro = ahrs.get_gyro();  // rad/s, body frame

    //gcs().send_text( MAV_SEVERITY_INFO, "ATT R=%.1f P=%.1f Y=%.1f deg", (double)roll, (double)pitch, (double)yaw);
    gcs().send_text( MAV_SEVERITY_INFO, "GYRO p=%.1f q=%.1f r=%.1f dps", (double)gyro.x, (double)gyro.y, (double)gyro.z );



    // Print RSSI value
    int16_t rssi_value = hal.rcin->get_rssi();
    //gcs().send_text(MAV_SEVERITY_INFO, "RSSI: %d", rssi_value);

    // Check for failsafe conditions
    if (copter.failsafe.radio) {
        //gcs().send_text(MAV_SEVERITY_WARNING, "Failsafe: Radio");
        disarm_motors();
        return;
    }

    const int16_t rc_channel_5_value = rc().channel(4)->get_radio_in(); // RC channel 5 corresponds to index 4 (0-based index)

    if (rc_channel_5_value < 1800) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Failsafe: RC Channel 5 Low");
        disarm_motors();
        return;
    }

    arm_motors();
    const bool allow_output = motors->armed() && motors->get_interlock();

    for (uint8_t i = 0; i < 8; ++i) {
        if (!motors->is_motor_enabled(i)) {
            continue;
        }
        if (!allow_output) {
            motors->rc_write(i, motors->get_pwm_output_min());
            continue;
        }

        // Map motor index to RC channel
        uint8_t rc_channel_index;
        if (i < 4) {
            rc_channel_index = i; // Motors 1-4: RC channels 0-3
        } else {
            rc_channel_index = i + 1; // Motors 5-8: RC channels 5-8
        }

        RC_Channel* rc_chan = rc().channel(rc_channel_index);
        if (rc_chan == nullptr) {
            motors->rc_write(i, motors->get_pwm_output_min());
            continue;
        }

        const int16_t rc_channel_i_value = rc_chan->get_radio_in();
        float normalized_input = (rc_channel_i_value - 988.0f) / (2012.0f - 988.0f);
        normalized_input = constrain_float(normalized_input, 0.0f, 1.0f);
        int16_t pwm = motors->get_pwm_output_min() + (motors->get_pwm_output_max() - motors->get_pwm_output_min()) * fabsf(normalized_input);

        motors->rc_write(i, pwm);
    }



     // ---- Cycle-time & Hz (timestamp recorded at END) ----
    static uint32_t prev_us = 0;      // persists across calls
    static float ema_hz = 0.0f;       // optional smoothing
    const uint32_t now_us = AP_HAL::micros();

    if (prev_us != 0) {
        const uint32_t dt_us = now_us - prev_us;           // uint32 wrap-safe
        const float    hz    = (dt_us > 0) ? (1.0e6f / (float)dt_us) : 0.0f;

        // Simple EMA to steady the printout (tweak alpha or remove if not wanted)
        const float alpha = 0.2f;
        ema_hz = (ema_hz == 0.0f) ? hz : (alpha * hz + (1.0f - alpha) * ema_hz);

        //gcs().send_text( MAV_SEVERITY_INFO, "Loop: dt=%lu us  Hz=%.1f (EMA=%.1f)  RSSI=%d", (unsigned long)dt_us, (double)hz, (double)ema_hz, (int)rssi_value );
    }
    prev_us = now_us;


}

#endif
*/