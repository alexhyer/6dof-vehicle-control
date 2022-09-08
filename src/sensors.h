#ifndef SENSORS_H
#define SENSORS_H

double y_pos_gnss, z_pos_gnss;
double y_vel_gnss, z_vel_gnss;
double y_last_pos_gnss, z_last_pos_gnss;
double gnss_lat, gnss_long;
double pressure_baro;
double altitude_baro;
double raw_altitude_baro;
double altitude_offset;
double current_thrust;
double x_pos_imu, y_pos_imu, z_pos_imu;
double x_vel_imu, y_vel_imu, z_vel_imu;

Orientation ori;
Quaternion quat;
Quaternion localAccelQuat;
Quaternion worldAccelQuat;
Quaternion orientation_quat(1, 0, 0, 0);
EulerAngles gyroMeasure;
EulerAngles oriMeasure;
Gnss gnss;

//Quat orientation lib comes from perry found here https://github.com/peregrine-developments/Orientation
//BMI088 lib comes from bolderflight found here https://github.com/bolderflight/bmi088-arduino

uint16_t serialFreq = 512;
uint16_t serialCounter = 0;
double x_gyro_rad, y_gyro_rad, z_gyro_rad;
double x_gyro_deg, y_gyro_deg, z_gyro_deg;
double x_accel, y_accel, z_accel;
double x_global_ori, y_global_ori, z_global_ori;
double x_global_accel, y_global_accel, z_global_accel;
double y_accel_local_ori, z_accel_local_ori;
void check_time()
{
    flight_time_micros = time_micros - liftoff_time_micros;
    flight_time_sec = (double)flight_time_micros / 1e6;
    time_micros = micros();
    time_millis = (double)time_micros / 1000;
    time_sec = (double)time_micros / 1e6;
    dt_micros = time_micros - last_time_micros;
    dt_millis = (double)dt_micros / 1000;
    dt_sec = (double)dt_micros / 1e6;
    last_time_micros = time_micros;

    // Derivation rate limiting:
    if (derive_bool == 1) // Reset bool
    {
        derive_bool = 0;
    }
    if ((time_micros - last_derive_time) >= derive_interval) // Rate limiting
    {
        derive_dt_micros = time_micros - last_derive_time;
        last_derive_time = time_micros;
        derive_bool = 1;
    }
}

void read_sensors()
{
    pressure_baro = bmp.readPressure();
    raw_altitude_baro = bmp.readAltitude(1013.25);

    gyro.readSensor();
    accel.readSensor();

    x_gyro_rad = gyro.getGyroZ_rads() + x_gyro_bias;
    y_gyro_rad = -gyro.getGyroY_rads() + y_gyro_bias;
    z_gyro_rad = -gyro.getGyroX_rads() + z_gyro_bias;

    x_gyro_deg = x_gyro_rad * (180 / pi);
    y_gyro_deg = y_gyro_rad * (180 / pi);
    z_gyro_deg = z_gyro_rad * (180 / pi);

    gyroMeasure.roll = -x_gyro_rad; // Body X
    gyroMeasure.pitch = y_gyro_rad; // Body Y
    gyroMeasure.yaw = z_gyro_rad;   // Body Z

    ori.update(gyroMeasure, (float)dt_micros / 1e6);

    x_accel = -accel.getAccelZ_mss() + x_accel_bias;
    y_accel = -accel.getAccelY_mss() + y_accel_bias;
    z_accel = -accel.getAccelX_mss() + z_accel_bias;
}

void process_sensors()
{
    altitude_baro = raw_altitude_baro - altitude_offset;
    current_thrust = current_mass * (sqrt(powf(x_accel, 2) + powf(y_accel, 2) + powf(z_accel, 2))); // F=ma   accel is magnitude of local accel

    oriMeasure = ori.toEuler();
    x_global_ori = oriMeasure.roll * -RAD_TO_DEG;
    y_global_ori = oriMeasure.pitch * RAD_TO_DEG;
    z_global_ori = oriMeasure.yaw * RAD_TO_DEG;

    float unit_ori = sqrt(powf(x_gyro_rad, 2) + powf(y_gyro_rad, 2) + powf(z_gyro_rad, 2));
    unit_ori = max(abs(unit_ori), 1e-12);

    orientation_quat *= Quaternion::from_axis_angle((float)(dt_micros / 1e6) * unit_ori, -x_gyro_rad / unit_ori, y_gyro_rad / unit_ori, z_gyro_rad / unit_ori);
    localAccelQuat = Quaternion(0.0, x_accel, y_accel, z_accel);
    worldAccelQuat = orientation_quat.rotate(localAccelQuat);

    x_global_accel = worldAccelQuat.b;
    y_global_accel = worldAccelQuat.c;
    z_global_accel = worldAccelQuat.d;

    y_accel_local_ori = atan2(z_accel, x_accel) * (180 / pi);
    z_accel_local_ori = atan2(-y_accel, x_accel) * (180 / pi);

    x_vel_imu += (((x_global_accel - 9.805)) * dt_micros) / (1e6);
    y_vel_imu += ((y_global_accel)*dt_micros) / (1e6);
    z_vel_imu += ((z_global_accel)*dt_micros) / (1e6);

    x_pos_imu += ((x_vel_imu)*dt_micros) / (1e6);
    y_pos_imu += ((y_vel_imu)*dt_micros) / (1e6);
    z_pos_imu += ((z_vel_imu)*dt_micros) / (1e6);
    
    if (gnss_derive_bool == 1) // Reset bool
    {
        gnss_derive_bool = 0;
    }
    if (gnss.processGPS()) // Running at gnss 10Hz Make sure no flashes to low rates when outside
    {
        gnss_derive_bool = 1;
        gnss_interval = (double)time_micros - last_gnss_time;
        gnss_lat = gnss.posllh.lat;
        gnss_long = gnss.posllh.lon;
        y_pos_gnss = ((gnss_long)-long_center) * .0081176;      // Long coefficients change with latitude: CO - .008445  WI - .0081176
        z_pos_gnss = ((gnss_lat)-lat_center) * .011132 * -1;    // Lat coefficient: .011132
        y_vel_gnss = (y_pos_gnss - y_last_pos_gnss) / (gnss_interval / 1e6);
        z_vel_gnss = (z_pos_gnss - z_last_pos_gnss) / (gnss_interval / 1e6);
        y_last_pos_gnss = y_pos_gnss;
        z_last_pos_gnss = z_pos_gnss;
        last_gnss_time = time_micros;
    }
}
#endif