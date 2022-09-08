#ifndef GNC_H
#define GNC_H

double x_ori_setpoint;
double y_ori_setpoint;
double z_ori_setpoint;

double x_pos_setpoint;
double y_pos_setpoint;
double z_pos_setpoint;

double x_ori_error;
double y_ori_error;
double z_ori_error;
double x_ori_last_error;
double y_ori_last_error;
double z_ori_last_error;

double x_pos_error;
double y_pos_error;
double z_pos_error;
double x_pos_last_error;
double y_pos_last_error;
double z_pos_last_error;

double x_torque;
double y_torque;
double z_torque;
double y_rotated_torque;
double z_rotated_torque;

double x_force;
double y_force;
double z_force;

double x_ori_deriv;
double y_ori_deriv;
double z_ori_deriv;

double x_cum_error;
double y_cum_error;
double z_cum_error;

double x_pos_cum_error;
double y_pos_cum_error;
double z_pos_cum_error;

double x_last_ori;
double y_last_ori;
double z_last_ori;
double x_last_pos;
double y_last_pos;
double z_last_pos;

double tvc_angle_y;
double tvc_angle_z;
double tvc_angle_y2;
double tvc_angle_z2;

double servo_value_z;
double servo_value_y;
double servo_value_z2;
double servo_value_y2;

double x_vel;
double y_vel;
double z_vel;

double x_pos; // Will be output of kalman filter
double y_pos;
double z_pos;

void orientation_pid()
{
    x_ori_error = x_ori_setpoint - x_global_ori;
    y_ori_error = y_ori_setpoint - y_global_ori;
    z_ori_error = z_ori_setpoint - z_global_ori;

    x_cum_error += ((x_ori_error)*dt_micros) / (1e6);
    y_cum_error += ((y_ori_error)*dt_micros) / (1e6);
    z_cum_error += ((z_ori_error)*dt_micros) / (1e6);

    if (derive_bool == 1)
    {
        x_ori_last_error = x_ori_setpoint - x_last_ori;
        y_ori_last_error = y_ori_setpoint - y_last_ori;
        z_ori_last_error = z_ori_setpoint - z_last_ori;
        x_ori_deriv = (x_ori_error - x_ori_last_error) / (derive_dt_micros / (1e6));
        y_ori_deriv = (y_ori_error - y_ori_last_error) / (derive_dt_micros / (1e6));
        z_ori_deriv = (z_ori_error - z_ori_last_error) / (derive_dt_micros / (1e6));
        x_last_ori = x_global_ori;
        y_last_ori = y_global_ori;
        z_last_ori = z_global_ori;
    }
    x_torque = p_ori_x * x_ori_error + i_ori_x * x_cum_error + d_ori_x * x_ori_deriv;
    y_torque = p_ori * y_ori_error + i_ori * y_cum_error + d_ori * y_ori_deriv;
    z_torque = p_ori * z_ori_error + i_ori * z_cum_error + d_ori * z_ori_deriv;
}

void position_pid()
{
    y_pos = y_pos_gnss;
    z_pos = z_pos_gnss;
    y_vel = y_vel_gnss;
    z_vel = z_vel_gnss;

    x_pos_error = x_pos_setpoint - x_pos;
    y_pos_error = y_pos_setpoint - y_pos;
    z_pos_error = z_pos_setpoint - z_pos;

    x_pos_cum_error += ((x_pos_error)*dt_micros) / (1e6);
    y_pos_cum_error += ((y_pos_error)*dt_micros) / (1e6);
    z_pos_cum_error += ((z_pos_error)*dt_micros) / (1e6);

    if (derive_bool == 1)
    {
        x_pos_last_error = x_pos_setpoint - x_last_pos;
        y_pos_last_error = y_pos_setpoint - y_last_pos; // why i do this idk, rethink later
        z_pos_last_error = z_pos_setpoint - z_last_pos;
        // x_vel = (x_pos - x_pos_last_error) / (derive_dt_micros / (1e6));
        // y_vel = (y_pos - y_pos_last_error) / (derive_dt_micros / (1e6));
        // z_vel = (z_pos - z_pos_last_error) / (derive_dt_micros / (1e6));
        x_last_pos = x_pos;
        y_last_pos = y_pos;
        z_last_pos = z_pos;
    }
    // x_force = (p_pos_x * x_pos_error + i_pos_x * x_pos_cum_error + d_pos_x * x_vel) * current_mass;
    x_force = 0;
    y_force = (p_pos * y_pos_error + i_pos * y_pos_cum_error + d_pos * -y_vel) * current_mass;
    z_force = (p_pos * z_pos_error + i_pos * z_pos_cum_error + d_pos * -z_vel) * current_mass;

    throttle = x_force; // Force/throttle transfer function goes here. Need to collect static force data
    x_ori_setpoint = 0;
    y_ori_setpoint = asin((z_force / current_thrust) * (pi / 180)) * (180 / pi); // control y pos with z ori, z pos with y ori
    z_ori_setpoint = asin((y_force / current_thrust) * (pi / 180)) * (180 / pi); //https://drive.google.com/file/d/1mANkLke7mAjCwUEb6_7WpxMwtRImbzFx/view
}

void write_servos()
{
    double cs = cos(x_global_ori * (pi / 180));
    double sn = sin(x_global_ori * (pi / 180));
    y_rotated_torque = y_torque * cs - z_torque * sn;
    z_rotated_torque = y_torque * sn + z_torque * cs; // Deroll(global to local)  NEED TO TEST DEROLL AND INCORPERATE ROLL CONTROL debug all this first

    tvc_angle_y = (asin((y_rotated_torque / (moment_arm * current_thrust)) * (pi / 180)) * (180 / pi)) + (asin((x_torque / (moment_arm_x * current_thrust)) * (pi / 180)) * (180 / pi));
    tvc_angle_z = asin((z_rotated_torque / (moment_arm * current_thrust)) * (pi / 180)) * (180 / pi);
    tvc_angle_y2 = (asin((y_rotated_torque / (moment_arm * current_thrust)) * (pi / 180)) * (180 / pi)) + (asin((x_torque / (moment_arm_x * current_thrust)) * (pi / 180)) * (-180 / pi));
    tvc_angle_z2 = asin((z_rotated_torque / (moment_arm * current_thrust)) * (pi / 180)) * (180 / pi);

    servo_value_y = pos_y + ((tvc_angle_y * gear_ratio) * deg_to_micros);
    servo_value_z = pos_z + ((tvc_angle_z * gear_ratio) * deg_to_micros); // Need to account for roll pid
    servo_value_y2 = pos_y2 + ((tvc_angle_y2 * gear_ratio) * deg_to_micros);
    servo_value_z2 = pos_z2 + ((tvc_angle_z2 * gear_ratio) * deg_to_micros);

    servo_value_y = min(servo_value_y, (pos_y + 420));
    servo_value_z = min(servo_value_z, (pos_z + 420));
    servo_value_y = max(servo_value_y, (pos_y - 420));
    servo_value_z = max(servo_value_z, (pos_z - 420));
    servo_value_y2 = min(servo_value_y2, (pos_y2 + 420));
    servo_value_z2 = min(servo_value_z2, (pos_z2 + 420));
    servo_value_y2 = max(servo_value_y2, (pos_y2 - 420));
    servo_value_z2 = max(servo_value_z2, (pos_z2 - 420));

    servo_y.writeMicroseconds(servo_value_y);
    servo_z.writeMicroseconds(servo_value_z);
    servo_y2.writeMicroseconds(servo_value_y2);
    servo_z2.writeMicroseconds(servo_value_z2);
}

#endif