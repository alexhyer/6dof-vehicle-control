#ifndef DEBUG_H
#define DEBUG_H

void print()
{

    // Serial.print(x_pos_imu);
    // Serial.print("\t");
    // Serial.print(y_pos_imu);
    // Serial.print("\t");
    // Serial.print(z_pos_imu);
    // Serial.print("\t");
    // Serial.print(x_vel_imu);
    // Serial.print("\t");
    // Serial.print(y_vel_imu);
    // Serial.print("\t");
    // Serial.print(z_vel_imu);
    // Serial.print("\t");

    // Serial.print(gnss_lat);
    // Serial.print("\t");
    // Serial.print(gnss_long);
    // Serial.print("\t");
    // Serial.print(y_pos_gnss);
    // Serial.print("\t");
    // Serial.print(z_pos_gnss);
    // Serial.print("\t");
    // Serial.print(y_vel_gnss);
    // Serial.print("\t");
    // Serial.print(z_vel_gnss);
    // Serial.print("\t");

    // Serial.print(y_ori_error);
    // Serial.print("\t");
    // Serial.print(z_ori_error);
    // Serial.print("\t");

    // Serial.print(altitude_baro);
    // Serial.print("\t");
    // Serial.print(dt_micros/1e6, 5);
    // Serial.print("\t");
    // Serial.print(time_sec,4);
    // Serial.print("\t");
    // Serial.print(mass);
    // Serial.print("\t");

    // Serial.print(x_global_ori);
    // Serial.print("\t");
    // Serial.print(y_global_ori);
    // Serial.print("\t");
    // Serial.print(z_global_ori);
    // Serial.print("\t");

    // Serial.print(x_global_accel);
    // Serial.print("\t");
    // Serial.print(y_global_accel);
    // Serial.print("\t");
    // Serial.print(z_global_accel);
    // Serial.print("\t");

    // Serial.print(x_gyro_rad);
    // Serial.print("\t");
    // Serial.print(y_gyro_rad);
    // Serial.print("\t");
    // Serial.print(z_gyro_rad);
    // Serial.print("\t");
    // Serial.print(x_gyro_deg);
    // Serial.print("\t");
    // Serial.print(y_gyro_deg);
    // Serial.print("\t");
    // Serial.print(z_gyro_deg);
    // Serial.print("\t");

    // Serial.print(x_accel);
    // Serial.print("\t");
    // Serial.print(y_accel);
    // Serial.print("\t");
    // Serial.print(z_accel);

    // Serial.print(x_accel_global_ori);
    // Serial.print("\t");
    // Serial.print(y_accel_local_ori);
    // Serial.print("\t");
    // Serial.print(z_accel_local_ori);

    // Serial.print(y_global_ori);
    // Serial.print("\t");
    // Serial.print(z_global_ori);
    // Serial.print("\t");
    // Serial.print(state);
    // Serial.print("\t");
    // Serial.print(flight_time_micros);
    // Serial.print("\t");
    // Serial.print(flight_time_sec);
    // Serial.print("\t");
    // Serial.print(dt_millis);
    // Serial.print("\t");
    // Serial.print(move_timer);
    // Serial.print("\t");
    // Serial.print(x_gyro_bias);
    // Serial.print("\t");
    // Serial.print(y_gyro_bias);
    // Serial.print("\t");
    // Serial.print(z_gyro_bias);
    // Serial.print("\t");

    Serial.println();
    Serial.send_now();
}

#endif