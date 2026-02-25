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
    // 1. SAFETY
    if (copter.failsafe.radio || rc().channel(4)->get_radio_in() < 1800) {


        gcs().send_text(MAV_SEVERITY_INFO, "Turtle Mode: OFF %2.f", (double)rc().channel(4)->get_radio_in());
        motors->rc_write(0, 0.0f);
        motors->rc_write(1, 0.0f);
        disarm_motors();
        return;
    }
    arm_motors();

    // 2. READ INPUTS
    // Fix: Use get_gyro() for rate and get_roll()/get_pitch() for angles
    const Vector3f& gyro = ahrs.get_gyro(); 
    
    // CORRECTION HERE: access these as functions
    float current_roll  = ahrs.get_roll();
    float current_pitch = ahrs.get_pitch();
    //float current_yaw   = ahrs.get_yaw();


    
    float enable_fault_tolerant = rc().channel(5)->norm_input(); 



    float roll_stick  = rc().channel(0)->norm_input(); 
    float pitch_stick = rc().channel(1)->norm_input(); 
    float yaw_stick   = rc().channel(3)->norm_input();
        
    // Throttle (0.0 to 1.0)
    float throttle_in = (rc().channel(2)->norm_input() + 1.0f) / 2.0f;
    if (throttle_in < 0.05f) throttle_in = 0.0f;



    // 5. OUTPUT
    int16_t pwm_min = motors->get_pwm_output_min();
    int16_t pwm_max = motors->get_pwm_output_max();
    int16_t pwm_range = pwm_max - pwm_min;
    int16_t pwm_mid = (pwm_max + pwm_min) / 2;



    // --- FAULT TOLERANT BICOPTER CONTROLLER ---
    // Assumes: 
    // - S0 is the Bad Servo (right)
    // - S1 is the Good Servo (left)
    // - Motor 0 is right, Motor 1 is left

    if (enable_fault_tolerant > 0.5f) {

        // --- A. CONFIGURATION & CONSTANTS ---
        float arm_length    = 0.25f;      
        float max_thrust_N  = 30.0f;    // Max thrust per motor
        float target_spin_rate = 3.0f;  // Rad/s (Slow, stable spin)
        float s0_stuck_val = 0.00f;      // Current stuck position of S0
        float Izz = 0.043f;

        // --- B. SLIDING MODE ALTITUDE CONTROLLER ---
        float current_alt_cm = inertial_nav.get_position_z_up_cm();
        float current_vel_z  = inertial_nav.get_velocity_z_up_cms();
        
        float target_alt_cm = 100.0f; // 10 Meters
        float error_alt     = target_alt_cm - current_alt_cm;
        
        // SMC Gains
        float lambda = 1.5f;   
        float K_gain = 0.4f;   
        float phi    = 50.0f;  
        float hover_throt = 0.3f; 

        float S = (lambda * error_alt) - current_vel_z;
        float saturation = constrain_float(S / phi, -1.0f, 1.0f);
        float smc_output = hover_throt + (K_gain * saturation);
        
        smc_output = constrain_float(smc_output, 0.1f, 0.9f);

        // Convert SMC 0-1 range to total Newtons
        float f_total_req_N = smc_output * (2.0f * max_thrust_N);

        float current_rate = fabsf(gyro.z);
        
        // 1. Calculate Desired Angular Acceleration (Alpha)
        // P-loop: How fast we want to change spin speed (rad/s^2)
        float alpha_cmd = 10.0f * (target_spin_rate - current_rate); 
        

        float total_torque_req = (Izz * alpha_cmd);

        float torque_term = (2.0f * total_torque_req) / MAX(arm_length * f_total_req_N, 0.5f);
        float tan_theta_1 = tanf(s0_stuck_val) + torque_term;
        float theta_1_raw = atanf(tan_theta_1);
        float theta_1_ff = -1.0f * constrain_float(theta_1_raw, -1.0f, 1.0f);


        // --- D. SOLVE FOR THRUST SPLIT (Body-Roll Balance) ---
        float base_lift = f_total_req_N / 2.0f;

        float roll_error = 0.0f - current_roll;
        float roll_correction = roll_error * 0.01f;
        
        float f0_lift = base_lift + roll_correction;
        float f1_lift = base_lift - roll_correction;

        float f0_total = f0_lift / MAX(cosf(s0_stuck_val), 0.1f);
        float f1_total = f1_lift / MAX(cosf(theta_1_ff), 0.1f);

        float m0_out = f0_total / max_thrust_N;
        float m1_out = f1_total / max_thrust_N;


        // --- F. FINAL OUTPUT ---
        // Servo 0: Locked
        motors->rc_write(2, pwm_mid + (int16_t)(s0_stuck_val * pwm_range)); 
        motors->rc_write(3, pwm_mid + (int16_t)(constrain_float(theta_1_ff, -1.0f, 1.0f) * pwm_range)); 

        motors->rc_write(0, pwm_min + (int16_t)(constrain_float(m0_out, 0.0f, 1.0f) * pwm_range)); 
        motors->rc_write(1, pwm_min + (int16_t)(constrain_float(m1_out, 0.0f, 1.0f) * pwm_range)); 

        gcs().send_text(MAV_SEVERITY_INFO, "Alt:%.0f T1:%.2f M0:%.2ff M0:%.2ff",  (double)alpha_cmd, (double)theta_1_ff, (double)gyro.z, (double) current_roll);

    } else {
        gcs().send_text(MAV_SEVERITY_INFO, "Bicopter Mode: NORMAL MODE");



        // 3. CONTROL LOGIC
        // Set max tilt angle (e.g. 45 degrees in radians)
        const float MAX_ANGLE = radians(45.0f); 
        const float MAX_YAW_RATE = radians(200.0f);

        // --- PITCH (Collective Servo Tilt) ---
        // 1. Angle Controller (Outer Loop)
        float target_pitch_angle = pitch_stick * MAX_ANGLE;
        float pitch_angle_error = target_pitch_angle - current_pitch;
        
        float K_p_angle_pitch = 4.5f; 
        float target_pitch_rate = pitch_angle_error * K_p_angle_pitch;

        // 2. Rate Controller (Inner Loop)
        float pitch_rate_error = target_pitch_rate - gyro.y;
        float K_p_rate_pitch = 0.30f; 
        
        float pitch_out = pitch_rate_error * K_p_rate_pitch;

        // --- ROLL (Differential Thrust) ---
        float target_roll_angle = roll_stick * MAX_ANGLE;
        float roll_angle_error = target_roll_angle - current_roll;
        
        float K_p_angle_roll = 4.5f;
        float target_roll_rate = roll_angle_error * K_p_angle_roll;

        float roll_rate_error = target_roll_rate - gyro.x;
        float K_p_rate_roll = 0.20f;
        float roll_out = roll_rate_error * K_p_rate_roll;

        // --- YAW (Differential Servo Tilt) ---
        // Yaw usually stays as a Rate controller
        float target_yaw_rate = yaw_stick * MAX_YAW_RATE;
        float yaw_error = target_yaw_rate - gyro.z;
        float K_p_yaw = 0.30f;
        float yaw_out = yaw_error * K_p_yaw;

        // 4. MIXER
        float motor_left  = throttle_in - roll_out;
        float motor_right = throttle_in + roll_out;

        float servo_left  = pitch_out + yaw_out;
        float servo_right = pitch_out - yaw_out;



        motors->rc_write(0, pwm_min + (int16_t)(constrain_float(motor_left, 0.0f, 1.0f) * pwm_range));
        motors->rc_write(1, pwm_min + (int16_t)(constrain_float(motor_right, 0.0f, 1.0f) * pwm_range));

        float servo_scaler = 0.6f; 
        
        int16_t s0_pwm = pwm_mid + (int16_t)(constrain_float(servo_left, -1.0f, 1.0f) * pwm_range * servo_scaler);
        int16_t s1_pwm = pwm_mid + (int16_t)(constrain_float(servo_right, -1.0f, 1.0f) * pwm_range * servo_scaler);

        motors->rc_write(2, s0_pwm);
        motors->rc_write(3, s1_pwm);

    }

    // Optional: Log errors to Ground Station for tuning
    //gcs().send_text(MAV_SEVERITY_INFO, "P_Ang:%.2f P_Rate:%.2f",  (double)button_1, (double)button_2);
}
#endif






//SWUNG UAV

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
    const float yaw_deg   = ahrs.get_yaw();
    const Vector3f& gyro  = ahrs.get_gyro(); 

    const float pitch_rad = radians(pitch_deg);
    const float roll_rad  = radians(roll_deg);

    const float pitch_rate_dps = degrees(gyro.y); 
    const float roll_rate_dps  = degrees(gyro.x); 
    const float yaw_rate_dps   = degrees(gyro.z);

    arm_motors();

    // Deadband helper lambda
    auto apply_deadband = [](float input, float deadband) {
        if (fabsf(input) < deadband) return 0.0f;
        return input;
    };

    // ---------------------------------------------------------
    // EARTH-FRAME SETPOINT ACCUMULATION
    // ---------------------------------------------------------
    // Stick inputs are in body frame. We rotate them into earth frame
    // for accumulation so that the target tilt direction is fixed in
    // the world regardless of yaw.
    float stick_pitch = rc().channel(1)->norm_input();
    stick_pitch = apply_deadband(stick_pitch, 0.05f);
    float stick_roll = rc().channel(0)->norm_input();
    stick_roll = apply_deadband(stick_roll, 0.05f);

    const float cy = cosf(yaw_deg);
    const float sy = sinf(yaw_deg);

    // Body-to-earth rotation for stick inputs
    target_earth_x += (stick_pitch * cy + stick_roll * sy) * 0.001f;
    target_earth_y += (stick_pitch * sy - stick_roll * cy) * 0.001f;

    // Clamp earth-frame magnitude
    const float max_tilt = 60.0f;
    target_earth_x = constrain_float(target_earth_x, -max_tilt, max_tilt);
    target_earth_y = constrain_float(target_earth_y, -max_tilt, max_tilt);

    // Earth-to-body rotation to get body-frame targets for PD control
    // Matches user's sign convention: pitch=-0.47,yaw=0 → pitch=0,roll=+0.47 at yaw=-π/2
    float body_pitch_target = target_earth_x * cy + target_earth_y * sy;
    float body_roll_target  = target_earth_x * sy - target_earth_y * cy;

    // ---------------------------------------------------------
    // 3. PITCH AXIS CONTROL (Motors 1 & 4 / Index 0 & 3)
    // ---------------------------------------------------------
    // Gravity feedforward (original working approach)
    float force_p = mass * gravity_mss * sinf(pitch_rad);
    
    float omega_p_total = 0.0f;
    if (fabsf(force_p) > 0.0001f) {
        omega_p_total = sqrtf(fabsf(force_p) / kc) * (force_p > 0 ? 1.0f : -1.0f);
    }
    float omega_per_motor_p = omega_p_total / 1.4142f; 
    float ff_p = omega_per_motor_p / (max_omega * 2.0f);

    const float kp_p = 0.1f;
    const float kd_p = 0.01f;
    float pd_p = ((body_pitch_target - pitch_deg) * kp_p) + ((0.0f - pitch_rate_dps) * kd_p);

    float out_p = constrain_float(0.5f + ff_p + pd_p, 0.0f, 1.0f);
    
    float out_p_ccw = out_p;
    float out_p_cw  = 1.0f - out_p;

    // ---------------------------------------------------------
    // 4. ROLL & YAW AXIS CONTROL (Motors 2 & 3 / Index 1 & 2)
    // ---------------------------------------------------------
    // Gravity feedforward (original working approach)
    float force_r = mass * gravity_mss * sinf(roll_rad);

    float omega_r_total = 0.0f;
    if (fabsf(force_r) > 0.0001f) {
        omega_r_total = sqrtf(fabsf(force_r) / kc) * (force_r > 0 ? 1.0f : -1.0f);
    }
    float omega_per_motor_r = omega_r_total / 1.4142f; 
    float ff_r = omega_per_motor_r / (max_omega * 2.0f);

    const float kp_r = 0.1f; 
    const float kd_r = 0.01f;
    float pd_r = ((body_roll_target - roll_deg) * kp_r) + ((0.0f - roll_rate_dps) * kd_r);

    float stick_yaw = rc().channel(3)->norm_input();
    stick_yaw = apply_deadband(stick_yaw, 0.05f);

    target_yaw_integrated += stick_yaw * 0.01f; 
    target_yaw_integrated = constrain_float(target_yaw_integrated, -60.0f, 60.0f);

    const float kp_y = 0.25f;
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

    gcs().send_text(MAV_SEVERITY_INFO, "P:%.3f R:%.3f Y:%.3f", (double)pitch_deg, (double)roll_deg, (double)yaw_deg);
}
#endif

*/












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