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

    // stop the spoolup block activating
    motors->set_spoolup_block(false);

    // reverse the motors
    hal.rcout->disable_channel_mask_updates();
    //change_motor_direction(true);

    // disable throttle and gps failsafe
    //g.failsafe_throttle.set(FS_THR_DISABLED);
    //g.failsafe_gcs.set(FS_GCS_DISABLED);
    //g.fs_ekf_action.set(0);

    // arm
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

    // turn off notify leds
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

    // un-reverse the motors
    //change_motor_direction(false);
    hal.rcout->enable_channel_mask_updates();

    // re-enable failsafes
    //g.failsafe_throttle.load();
    //g.failsafe_gcs.load();
    //g.fs_ekf_action.load();
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
    // Print RSSI value
    int16_t rssi_value = hal.rcin->get_rssi();
    gcs().send_text(MAV_SEVERITY_INFO, "RSSI: %d", rssi_value);
    // Check for failsafe conditions
    if (copter.failsafe.radio) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Failsafe: Radio");
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
    for (uint8_t i = 0; i < 4; ++i) {
        if (!motors->is_motor_enabled(i)) {
            return;
        }
        if (!allow_output) {
            motors->rc_write(i, motors->get_pwm_output_min());
            return;
        }
        const int16_t rc_channel_i_value = rc().channel(i)->get_radio_in();
        float normalized_input = (rc_channel_i_value - 988.0f) / (2012.0f - 988.0f);
        normalized_input = constrain_float(normalized_input, 0.0f, 1.0f);
        int16_t pwm = motors->get_pwm_output_min() + (motors->get_pwm_output_max() - motors->get_pwm_output_min()) * fabsf(normalized_input);

        motors->rc_write(i, pwm);
    }

}

#endif
