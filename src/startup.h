#ifndef STARTUP_H
#define STARTUP_H

double time_sec = 0;
double time_millis = 0;
double flight_time_sec = 0;
double dt_millis = 0;
double dt_sec = 0;
double net_gyro = 0;
double last_move_time = 0;
double move_timer = 0;
uint64_t time_micros = 0;
uint64_t last_time_micros = 0;
uint64_t dt_micros = 0;
uint64_t derive_dt_micros = 0;
uint64_t liftoff_time_micros = 0;
uint64_t flight_time_micros = 0;
uint64_t last_derive_time = 0;
uint64_t last_gnss_time = 0;
uint64_t last_log_time = 0;
uint64_t last_flush_time = 0;

Servo servo_z;
Servo servo_y;
Servo servo_z2;
Servo servo_y2;
Adafruit_BMP280 bmp;
//BMI088 lib comes from bolderflight found here https://github.com/bolderflight/bmi088-arduino
Bmi088Accel accel(Wire, 0x18);
Bmi088Gyro gyro(Wire, 0x68);

void initialize()
{
    Wire.begin();
    Serial3.begin(9600);
    Serial.begin(115200);
    servo_z.attach(20);
    servo_y.attach(21);
    servo_z2.attach(22);
    servo_y2.attach(23);
    pinMode(rled, OUTPUT);
    pinMode(gled, OUTPUT);
    pinMode(bled, OUTPUT);
    pinMode(buzz, OUTPUT);
    pinMode(led, OUTPUT);
    pinMode(chip_select, OUTPUT);
    int status;
    //  while (!Serial)
    //  {
    //  }
    delay(500);
    status = accel.begin();
    if (status < 0)
    {
        Serial.println("Accel Initialization Error");
        tone(buzz, 100);
        Serial.println(status);
        while (1)
        {
        }
    }
    status = gyro.begin();
    if (status < 0)
    {
        Serial.println("Gyro Initialization Error");
        tone(buzz, 100);
        Serial.println(status);
        while (1)
        {
        }
    }
    if (!bmp.begin())
    {
        Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
        tone(buzz, 100);
        while (1)
            ;
    }
    if (!SD.begin(chip_select))
    {
        Serial.println("Card failed, or not present");
        tone(buzz, 100);
        while (1)
            ;
    }
    accel.setOdr(Bmi088Accel::ODR_200HZ_BW_20HZ);
    gyro.setOdr(Bmi088Gyro::ODR_200HZ_BW_23HZ);
    accel.setRange(Bmi088Accel::RANGE_12G);
    gyro.setRange(Bmi088Gyro::RANGE_2000DPS);
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,   /* Operating Mode. */
                    Adafruit_BMP280::SAMPLING_X2,   /* Temp. oversampling */
                    Adafruit_BMP280::SAMPLING_X16,  /* Pressure oversampling */
                    Adafruit_BMP280::FILTER_X2,     /* Filtering. */
                    Adafruit_BMP280::STANDBY_MS_1); /* Standby time. */
    delay(300);
}

void startup()
{
    data_file = SD.open("datalog.txt", FILE_WRITE);
    servo_z.writeMicroseconds(pos_z);
    servo_y.writeMicroseconds(pos_y);
    servo_z2.writeMicroseconds(pos_z2);
    servo_y2.writeMicroseconds(pos_y2);
    tone(buzz, 500);
    analogWrite(rled, led_bright);
    delay(200);
    noTone(buzz);
    delay(200);
    analogWrite(rled, 0);
    tone(buzz, 700);
    analogWrite(gled, led_bright);
    delay(200);
    noTone(buzz);
    delay(200);
    analogWrite(gled, 0);
    tone(buzz, 900);
    analogWrite(bled, led_bright);
    delay(400);
    noTone(buzz);
    analogWrite(bled, 0);
    delay(300);

    servo_y.writeMicroseconds(pos_y - 420);
    servo_y2.writeMicroseconds(pos_y2 - 420);
    delay(250);
    servo_y.writeMicroseconds(pos_y);
    servo_y2.writeMicroseconds(pos_y2);
    delay(250);
    servo_y.writeMicroseconds(pos_y + 420);
    servo_y2.writeMicroseconds(pos_y2 + 420);
    delay(250);
    servo_y.writeMicroseconds(pos_y);
    servo_y2.writeMicroseconds(pos_y2);
    delay(250);
    servo_z.writeMicroseconds(pos_z - 420);
    servo_z2.writeMicroseconds(pos_z2 - 420);
    delay(250);
    servo_z.writeMicroseconds(pos_z);
    servo_z2.writeMicroseconds(pos_z2);
    delay(250);
    servo_z.writeMicroseconds(pos_z + 420);
    servo_z2.writeMicroseconds(pos_z2 + 420);
    delay(250);
    servo_z.writeMicroseconds(pos_z);
    servo_z2.writeMicroseconds(pos_z2);
    delay(250);
    servo_y.writeMicroseconds(pos_y - 420);
    servo_y2.writeMicroseconds(pos_y2 + 420);
    delay(250);
    servo_y.writeMicroseconds(pos_y);
    servo_y2.writeMicroseconds(pos_y2);
    delay(250);
    servo_y.writeMicroseconds(pos_y + 420);
    servo_y2.writeMicroseconds(pos_y2 - 420);
    delay(250);
    servo_y.writeMicroseconds(pos_y);
    servo_y2.writeMicroseconds(pos_y2);


    servo_z.writeMicroseconds(pos_z);
    servo_y.writeMicroseconds(pos_y);
    servo_z2.writeMicroseconds(pos_z2);
    servo_y2.writeMicroseconds(pos_y2);
    last_time_micros = micros();
}
#endif