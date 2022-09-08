#ifndef FUNCTIONS_H
#define FUNCTIONS_H

void check_abort()
{
    if (state == 2)
    {
        if ((x_global_ori > (flight_abort_angle) && x_global_ori < 90))
        {
            flight_abort = 1;
        }
        if ((x_global_ori < (flight_abort_angle * -1) && x_global_ori > -90))
        {
            flight_abort = 1;
        }
        if ((y_global_ori > (flight_abort_angle) && y_global_ori < 90))
        {
            flight_abort = 1;
        }
        if ((y_global_ori < (flight_abort_angle * -1) && y_global_ori > -90))
        {
            flight_abort = 1;
        }
    }
}

void check_mass()
{
    current_mass = starting_mass;           // check whats right idk
    if (flight_time_sec <= motor_burn_time) // Statment keeps currentmass as finalmass after burnout
    {
        // current_mass = (((final_mass - starting_mass) / motor_burn_time) * flight_time_sec) + starting_mass; // y=mx+b form to estimate mass linearly
        current_mass = starting_mass;
    }
}

void log_sdcard()
{
    if ((time_micros - last_log_time) >= log_interval)
    {
        last_log_time = time_micros;
        char data_string[512] = "";
        sprintf(data_string, "%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f",
                (double)time_micros, time_sec,
                (double)flight_time_micros, flight_time_sec,
                dt_millis,
                x_gyro_deg, y_gyro_deg, z_gyro_deg,
                x_gyro_bias, y_gyro_bias, z_gyro_bias,
                x_accel, y_accel, z_accel,
                x_accel_bias, y_accel_bias, z_accel_bias,
                gnss_lat, gnss_long,
                altitude_baro,
                x_global_ori, y_global_ori, z_global_ori,
                x_global_accel, y_global_accel, z_global_accel,
                y_accel_local_ori, z_accel_local_ori,
                y_pos_gnss, z_pos_gnss,
                current_thrust, throttle, current_mass,
                servo_value_y, servo_value_z, servo_value_y2, servo_value_z2,
                tvc_angle_y, tvc_angle_z, tvc_angle_y2, tvc_angle_z2,
                x_force, y_force, z_force,
                y_ori_setpoint, z_ori_setpoint,
                x_pos_cum_error, y_pos_cum_error, z_pos_cum_error,
                //x_pos, y_pos, z_pos,
                //x_vel, y_vel, z_vel,
                x_torque, y_torque, z_torque,
                y_rotated_torque, z_rotated_torque,
                x_cum_error, y_cum_error, z_cum_error,
                x_ori_deriv, y_ori_deriv, z_ori_deriv);
        data_file.println(data_string);
        data_file.flush();
    }
}

void calibrate_sensors()
{
    net_gyro = sqrt(powf(x_gyro_deg, 2) + powf(y_gyro_deg, 2) + powf(z_gyro_deg, 2));
    if (net_gyro <= -1.2 || net_gyro >= 1.2)
    {
        moving = 1;
        integrated_x_gyro = 0;
        integrated_y_gyro = 0;
        integrated_z_gyro = 0;
        last_move_time = time_millis;
        move_timer = 0;
    }
    else
    {
        moving = 0;
        move_timer = time_millis - last_move_time;
    }

    if (move_timer != 0 && move_timer < 8000)
    {
        integrated_x_gyro += x_gyro_rad * dt_millis;
        integrated_y_gyro += y_gyro_rad * dt_millis;
        integrated_z_gyro += z_gyro_rad * dt_millis;
    }

    if (move_timer >= 8000)
    {
        x_gyro_bias = x_gyro_bias + integrated_x_gyro / -8000;
        y_gyro_bias = y_gyro_bias + integrated_y_gyro / -8000;
        z_gyro_bias = z_gyro_bias + integrated_z_gyro / -8000;
        integrated_x_gyro = 0;
        integrated_y_gyro = 0;
        integrated_z_gyro = 0;
        lat_center = gnss_lat;
        long_center = gnss_long;
        altitude_offset = raw_altitude_baro;
        last_move_time = time_millis;
    }
}

void finalize()
{
    if (finalized == 0) // Single run
    {
        finalized = 1;
        data_file.close();
    }
    servo_y.writeMicroseconds(pos_y);
    servo_z.writeMicroseconds(pos_z);
    servo_y2.writeMicroseconds(pos_y2);
    servo_z2.writeMicroseconds(pos_z2);
}

#endif