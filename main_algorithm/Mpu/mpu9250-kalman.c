#include "Kalman.h" /* Source: https://github.com/TKJElectronics/KalmanFilter */
#include <wiringPiI2C.h>
#include <wiringPi.h>
#include <stdio.h>
#include <math.h>

/* MPU6050 */
#define MPU6050_I2C_DEVICE_ADDRESS     0x68
#define REGISTER_FOR_POWER_MANAGEMENT  0x6B  /* PWR_MGMT_1 */
#define REGISTER_FOR_SAMPLE_RATE       0x19  /* SMPLRT_DIV */
#define REGISTER_FOR_ACCEL_XOUT_H      0x3B
#define REGISTER_FOR_ACCEL_YOUT_H      0x3D
#define REGISTER_FOR_ACCEL_ZOUT_H      0x3F
#define REGISTER_FOR_GYRO_XOUT_H       0x43
#define REGISTER_FOR_GYRO_YOUT_H       0x45
#define REGISTER_FOR_GYRO_ZOUT_H       0x47
#define REGISTER_FOR_TEMP_OUT_H        0x41
#define SLEEP_MODE_DISABLED            0x00

/* Different math and print constants */
#define RAD_TO_DEG                     (180.0 / M_PI)
#define DRIFT_MAX_DEGREES              180

/* To restrict roll instead of pitch to ±90 degrees, comment out the following line */
#define PITCH_RESTRICT_90_DEG

/* AK8963 (Magnetometer) */
#define AK8963_I2C_DEVICE_ADDRESS      0x0C
#define AK8963_REGISTER_CNTL1          0x0A  /* Control register 1 */
#define AK8963_REGISTER_HXL            0x03  /* Magnetometer data start register */
#define AK8963_MODE_CONTINUOUS_8HZ     0x02  /* Continuous measurement mode at 8Hz */
#define AK8963_MODE_CONTINUOUS_100HZ   0x06  /* Continuous measurement mode at 100Hz */
#define AK8963_REGISTER_ST1            0x02  /* Status register 1 */

/* Magnetometer Variables */
int mag_device_handler;
double magX;
double magY;
double magZ;

/* MPU6050 variables */
int gyro_device_handler;
double accX;
double accY;
double accZ;
double gyroX;
double gyroY;
double gyroZ;
double temp_raw;

double temp_degrees_c;
double roll_gyro;
double roll;
double roll_kalman;         /* Angle exposed to a Kalman filter */
double roll_complementary;  /* Angle exposed to a Complementary filter */
double pitch;
double pitch_gyro;
double pitch_kalman;        /* Angle exposed to a Kalman filter */
double pitch_complementary; /* Angle exposed to a Complementary filter */

int read_word_2c(int register_h)
{
    int val;
    val = wiringPiI2CReadReg8(gyro_device_handler, register_h);
    val = val << 8;
    val += wiringPiI2CReadReg8(gyro_device_handler, register_h+1);
    if (val >= 0x8000)
        val = -(65536 - val);
    return val;
}

void read_sensor_data()
{
    accX     = read_word_2c(REGISTER_FOR_ACCEL_XOUT_H);
    accY     = read_word_2c(REGISTER_FOR_ACCEL_YOUT_H);
    accZ     = read_word_2c(REGISTER_FOR_ACCEL_ZOUT_H);
    gyroX    = read_word_2c(REGISTER_FOR_GYRO_XOUT_H);
    gyroY    = read_word_2c(REGISTER_FOR_GYRO_YOUT_H);
    gyroZ    = read_word_2c(REGISTER_FOR_GYRO_ZOUT_H);
    temp_raw = read_word_2c(REGISTER_FOR_TEMP_OUT_H);
}

double convert_to_deg_per_sec(double a)
{
    return a / 131.0;
}

double distance(double a, double b)
{
    return sqrt((a*a) + (b*b));
}

double atan2_deg(double a, double b)
{
    return atan2(a,b) * RAD_TO_DEG;
}

double atan_deg(double a, double b, double c)
{
    return atan(a / distance(b, c)) * RAD_TO_DEG;
}

double max_drift_correction(double gyro, double kalman)
{
    if (gyro < -DRIFT_MAX_DEGREES || gyro > DRIFT_MAX_DEGREES)
        return kalman;
    else
        return gyro;
}

double max_90_deg_correction(double rate, double kalman)
{
    if (abs(kalman) > 90)
        return -rate;
    else
        return rate;
}

void Gyro(double* roll_p, double* pitch_p){
    int timer;
    double seconds_passed;
    double roll_gyro_rate_deg_per_sec;
    double pitch_gyro_rate_deg_per_sec;

    Kalman kalman_roll;
    Kalman kalman_pitch;

    /* Set the gyro starting angles */
    read_sensor_data();

#ifdef PITCH_RESTRICT_90_DEG
    roll  = atan2_deg(accY, accZ);
    pitch = atan_deg(-accX, accY, accZ);
#else
    roll  = atan_deg(accY, accX, accZ);
    pitch = atan2_deg(-accX, accZ);
#endif

    /* Set some more initial values */
    kalman_roll.setAngle(roll);
    roll_gyro           = roll;
    roll_complementary  = roll;  /* Angle exposed to a complementary filter */
    kalman_pitch.setAngle(pitch);
    pitch_gyro          = pitch;
    pitch_complementary = pitch; /* Angle exposed to a complementary filter */
    timer               = micros();

    read_sensor_data();

    temp_degrees_c              = ((double)temp_raw / 340.0) + 36.53;
    seconds_passed              = (double)(micros() - timer) / 1000000;
    timer                       = micros();
    roll_gyro_rate_deg_per_sec  = convert_to_deg_per_sec(gyroX);
    pitch_gyro_rate_deg_per_sec = convert_to_deg_per_sec(gyroY);

#ifdef PITCH_RESTRICT_90_DEG

    /* Eq. 25 and 26 from source for equations */
    roll  = atan2_deg(accY,accZ);
    pitch = atan_deg(-accX, accY, accZ);

    /* Let pitch have -90 and 90 degrees to be the continuous (and roll ±180) */
    if ( abs(roll)<= 90 || abs(roll_kalman)<= 90 )
    {
        /* Calculate roll_kalman */
        roll_kalman = kalman_roll.getAngle(roll, roll_gyro_rate_deg_per_sec, seconds_passed);
    }
    else
    {
        kalman_roll.setAngle(roll);
        roll_complementary = roll;
        roll_kalman        = roll;
        roll_gyro          = roll;
    }
    pitch_gyro_rate_deg_per_sec = max_90_deg_correction(pitch_gyro_rate_deg_per_sec, roll_kalman);
    pitch_kalman                = kalman_pitch.getAngle(pitch, pitch_gyro_rate_deg_per_sec, seconds_passed);

#else
    /* Eq. 28 and 29 from source for equations */
    roll  = atan_deg(accY, accX, accZ);
    pitch = atan2_deg(-accX,accZ);

    /* Let roll have -90 and 90 degrees to be the continuous (and pitch ±180) */
    if ( abs(pitch)<= 90 || abs(pitch_kalman)<= 90 )
    {
        /* Calculate pitch_kalman */
        pitch_kalman = kalman_pitch.getAngle(pitch, pitch_gyro_rate_deg_per_sec, seconds_passed);
    }
    else
    {
        kalman_pitch.setAngle(pitch);
        pitch_complementary = pitch;
        pitch_kalman        = pitch;
        pitch_gyro          = pitch;
    }
    roll_gyro_rate_deg_per_sec = max_90_deg_correction(roll_gyro_rate_deg_per_sec, pitch_kalman);
    roll_kalman                = kalman_roll.getAngle(roll, roll_gyro_rate_deg_per_sec, seconds_passed);
#endif

    /* Calculate gyro angles without any filter */
    roll_gyro  += roll_gyro_rate_deg_per_sec * seconds_passed;
    pitch_gyro += pitch_gyro_rate_deg_per_sec * seconds_passed;

    /* Calculate gyro angle using the unbiased rate */
    //roll_gyro += kalman_roll.getRate() * seconds_passed;
    //pitch_gyro += kalman_pitch.getRate() * seconds_passed;

    roll_gyro  = max_drift_correction(roll_gyro, roll_kalman);
    pitch_gyro = max_drift_correction(pitch_gyro, pitch_kalman);

    /* Calculate the angle using a Complimentary filter */
    roll_complementary  = 0.93 * (roll_complementary + roll_gyro_rate_deg_per_sec * seconds_passed) + 0.07 * roll;
    pitch_complementary = 0.93 * (pitch_complementary + pitch_gyro_rate_deg_per_sec * seconds_passed) + 0.07 * pitch;

    *roll_p = roll_kalman;
    *pitch_p = pitch_kalman;
}

/* Function to initialize the magnetometer */
void init_magnetometer() {
    // Enable I2C bypass mode on MPU9250
    wiringPiI2CWriteReg8(gyro_device_handler, 0x37, 0x02);  // INT_PIN_CFG register
    delay(10);

    // Initialize the AK8963 magnetometer
    mag_device_handler = wiringPiI2CSetup(AK8963_I2C_DEVICE_ADDRESS);
    wiringPiI2CWriteReg8(mag_device_handler, AK8963_REGISTER_CNTL1, AK8963_MODE_CONTINUOUS_8HZ);
    delay(10);

    int who_am_i = wiringPiI2CReadReg8(mag_device_handler, 0x00);
    printf("AK8963 WHO_AM_I: 0x%02X\n", who_am_i);
}

/* Function to read data from the magnetometer */
void read_magnetometer_data() {
    int magX_low, magX_high, magY_low, magY_high, magZ_low, magZ_high;

    // Wait for data to be ready
    if (!(wiringPiI2CReadReg8(mag_device_handler, AK8963_REGISTER_ST1) & 0x01)) {
        return;
    }

    // Read magnetometer data
    magX_low  = wiringPiI2CReadReg8(mag_device_handler, AK8963_REGISTER_HXL);
    magX_high = wiringPiI2CReadReg8(mag_device_handler, AK8963_REGISTER_HXL + 1);
    magY_low  = wiringPiI2CReadReg8(mag_device_handler, AK8963_REGISTER_HXL + 2);
    magY_high = wiringPiI2CReadReg8(mag_device_handler, AK8963_REGISTER_HXL + 3);
    magZ_low  = wiringPiI2CReadReg8(mag_device_handler, AK8963_REGISTER_HXL + 4);
    magZ_high = wiringPiI2CReadReg8(mag_device_handler, AK8963_REGISTER_HXL + 5);

    // Combine high and low bytes
    magX = ((magX_high << 8) | magX_low);
    magY = ((magY_high << 8) | magY_low);
    magZ = ((magZ_high << 8) | magZ_low);

    // Apply two's complement adjustment
    if (magX >= 0x8000) magX -= 65536;
    if (magY >= 0x8000) magY -= 65536;
    if (magZ >= 0x8000) magZ -= 65536;
}


int main()
{
    gyro_device_handler = wiringPiI2CSetup(MPU6050_I2C_DEVICE_ADDRESS);
    wiringPiI2CWriteReg8(gyro_device_handler,REGISTER_FOR_POWER_MANAGEMENT,SLEEP_MODE_DISABLED);

    /* Wait for sensor to stabilize */
    delay(150);

    init_magnetometer();

    double roll_p;
    double pitch_p;

    while(1){
        Gyro(&roll_p, &pitch_p);
        read_magnetometer_data();
        printf("Roll: %.4f, Pitch: %.4f\n", roll_p, pitch_p);
        printf("MagX: %.4f, MagY: %.4f, MagZ: %.4f\n", magX, magY, magZ);
    }

}
