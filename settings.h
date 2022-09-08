#ifndef SETTINGS_H
#define SETTINGS_H

int pos_z;
int pos_y;
int pos_z2;
int pos_y2;
int led_bright;
int half_led_bright;
int flight_abort_angle;
int rled;
int gled;
int bled;
int buzz;
int led;
int chip_select;
double derive_interval; //micros
double gnss_interval;   //micros
double log_interval;
double flush_interval;
double starting_mass;
double final_mass;
double current_mass;
double motor_burn_time;
double liftoff_threshold;
double moment_arm, moment_arm_x;
double gear_ratio;
double deg_to_micros;
double throttle;
double p_ori;
double i_ori;
double d_ori;
double p_ori_x;
double i_ori_x;
double d_ori_x;
double p_pos;
double i_pos;
double d_pos;
double p_pos_x;
double i_pos_x;
double d_pos_x;
double x_accel_bias;
double y_accel_bias;
double z_accel_bias;
double integrated_x_gyro, integrated_y_gyro, integrated_z_gyro;
double pi = 3.14159265;
bool derive_bool;
bool gnss_derive_bool;
bool flight_abort;
bool moving;
bool finalized;
File data_file;

double x_gyro_bias = 0;     // 0
double y_gyro_bias = 0.001; // 0.001
double z_gyro_bias = 0.003; // 0.003

double lat_center = 405835566; // WI: lat-431250150  long-884757620
double long_center = -1051010440;

void load_settings()
{
    pos_z = 1400;
    pos_y = 1550;
    pos_z2 = 1500;
    pos_y2 = 1550;

    led_bright = 255;
    half_led_bright = 150;
    flight_abort_angle = 40;

    starting_mass = 1.1;
    final_mass = 1.1;
    motor_burn_time = 15; // Neglegable for Conk(s)
    liftoff_threshold = 1.5;
    derive_interval = 4000; // Derive rate in micros
    log_interval = 30000;   // Sd card log rate in micros 30000=33.33Hz 12500=80Hz

    moment_arm = 0.2; // 0.2m
    moment_arm_x = 0.067; // 0.067m
    gear_ratio = 5.2; // 5.2
    deg_to_micros = 11.05; //11.05

    p_ori = 2.1; // z,y orientation pid values p: 2.7 i: 2.5 d: 0.95  NEW: P:2.4 I:2.5 D:1.4
    i_ori = 2.5;
    d_ori = 2;

    p_ori_x = 0.35; // roll orientation pid values p:0.35 i: 0.15 d: 0.15   NEW: P:0.35 I:0.2 D:0.12
    i_ori_x = 0.1;
    d_ori_x = 0.12;

    p_pos = 25; // z,y position pid values p: 30 i: 20 d: 60
    i_pos = 40;
    d_pos = 60;

    p_pos_x = 0; // Altitude control pid values p: 15 i: 15 d: 15
    i_pos_x = 0;
    d_pos_x = 0;

    x_accel_bias = -0.00833; //-0.00833
    y_accel_bias = -0.02467; //-0.02467
    z_accel_bias = -0.13234; //-0.13234

    rled = 10; // Red LED
    gled = 9;  // Green LED
    bled = 6;  // Blue LED
    buzz = 5;  // Buzzer
    led = 13;  // Teensy LED
    chip_select = 29;
    pi = 3.14159265;
}
#endif