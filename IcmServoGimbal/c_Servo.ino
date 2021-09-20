void servoSetup()
{
    for (int i = 0; i < kNumServos; ++i)
    {
        servos_[i].attach(servo_pins_[i]);
    }
}

void moveServos()
{
    for (int i = 0; i < kNumServos; ++i)
    {
        float axis_error_rad = -attitude_rad_[i];

        float servo_correction = axis_error_rad * gimbal_gain_scaled_[i];
        debugPrintln("Servo " + String(i) + "corr: " + String(servo_correction));

        servo_target_pw_us_[i] += round(servo_correction);
        servo_target_pw_us_[i] = constrain(servo_target_pw_us_[i], 1000, 2000);

        servos_[i].writeMicroseconds(servo_target_pw_us_[i]);

        debugPrintln("servo " + String(i) + " raw: " + String(servo_target_pw_us_[i]));
    }
}