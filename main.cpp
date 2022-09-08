#include <Arduino.h>
#include <Orientation.h>
#include <Servo.h>
#include <BMI088.h>
#include <Adafruit_BMP280.h>
#include <SD.h>
#include <SPI.h>
#include <gnss.h>
#include <settings.h>
#include <startup.h>
#include <sensors.h>
#include <control.h>
#include <statemachine.h>
#include <debug.h>
#include <functions.h>

void setup()
{
  load_settings();
  initialize();
  startup();
  log_settings_sdcard();
}

void loop()
{
  load_settings();
  check_time();
  check_mass();
  check_state();
  print();

  if (state == 0) // Startup
  {
    read_sensors();
    process_sensors();
    calculate_kalman();
    calibrate_sensors();
  }

  if (state == 1) // Ready for flight
  {
    read_sensors();
    process_sensors();
    calculate_kalman();
    calibrate_sensors();
  }

  if (state == 2) // In flight
  {
    check_abort();
    read_sensors();
    process_sensors();
    calculate_kalman();
    orientation_pid();
    position_pid();
    write_servos();
    log_sdcard();
  }

  if (state == 3) // Post flight
  {
    read_sensors();
    process_sensors();
    finalize();
  }
  
}
