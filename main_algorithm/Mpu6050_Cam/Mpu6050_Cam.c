#include "Kalman.h" /* Source: https://github.com/TKJElectronics/KalmanFilter */
#include <wiringPiI2C.h>
#include <wiringPi.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>
#include <sys/mman.h>
#include <time.h>
#include <jpeglib.h>

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

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

#define IMAGE_WIDTH 1280
#define IMAGE_HEIGHT 720

#define f_img_per 100

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

int rep = 0;
int channels  = 3;

typedef struct {
    long double x;
    long double y;
} Point;

typedef struct {
    Point LU;
    Point RU;
    Point LD;
    Point RD;
} Section;


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

int set_control(int fd, int control_id, int value, const char *control_name) {
    struct v4l2_control control;
    control.id = control_id;
    control.value = value;

    if (ioctl(fd, VIDIOC_S_CTRL, &control) == -1) {
        perror("Error al configurar control");
        fprintf(stderr, "No se pudo establecer el valor de %s (ID: %d) en %d\n", control_name, control_id, value);
        return -1;
    }
    printf("%s configurado en %d\n", control_name, value);
    return 0;
}

// Function to decode MJPEG to RGB using libjpeg
int decode_mjpeg_to_rgb(unsigned char *mjpeg_data, size_t mjpeg_size, unsigned char *rgb_buffer) {
    struct jpeg_decompress_struct cinfo;
    struct jpeg_error_mgr jerr;

    cinfo.err = jpeg_std_error(&jerr);
    jpeg_create_decompress(&cinfo);

    jpeg_mem_src(&cinfo, mjpeg_data, mjpeg_size);
    if (jpeg_read_header(&cinfo, TRUE) != JPEG_HEADER_OK) {
        fprintf(stderr, "Error reading JPEG header\n");
        return -1;
    }

    jpeg_start_decompress(&cinfo);

    if (cinfo.output_width != IMAGE_WIDTH || cinfo.output_height != IMAGE_HEIGHT) {
        fprintf(stderr, "Unexpected image dimensions: %dx%d\n",
                cinfo.output_width, cinfo.output_height);
        return -1;
    }

    while (cinfo.output_scanline < cinfo.output_height) {
        unsigned char *row_pointer = rgb_buffer + cinfo.output_scanline * cinfo.output_width * 3;
        jpeg_read_scanlines(&cinfo, &row_pointer, 1);
    }

    jpeg_finish_decompress(&cinfo);
    jpeg_destroy_decompress(&cinfo);

    return 0;
}


// Función para resolver un sistema de ecuaciones lineales usando factorización LU
void solveLU(long double (*A)[8][8], long double *b, long double *x) {
    long double L[8][8] = {0.0}, U[8][8] = {0.0};
    int i, j, k;

    // Factorización LU
    for (i = 0; i < 8; i++) {
        for (j = i; j < 8; j++) {
            U[i][j] = (*A)[i][j];
            for (k = 0; k < i; k++) {
                U[i][j] -= L[i][k] * U[k][j];
            }
        }
        for (j = i; j < 8; j++) {
            if (i == j) {
                L[j][i] = 1.0;
            } else {
                L[j][i] = (*A)[j][i];
                for (k = 0; k < i; k++) {
                    L[j][i] -= L[j][k] * U[k][i];
                }
                if (U[i][i] != 0) {
                    L[j][i] /= U[i][i];
                } else if (i == 1 && j == 4 && U[i][i] == 0) {
                    L[j][i] /= 0.000000000001;
                } else if (i == 3 && j == 5 && U[i][i] == 0) {
                    L[j][i] /= 0.000001;
                } else if (i == 3 && j == 7 && U[i][i] == 0) {
                    L[j][i] /= 1;
                } else if (i == 5 && U[i][i] == 0) {
                    L[j][i] /= 1;
                }
            }
        }
    }

    // Resolver L * y = b (sustitución hacia adelante)
    long double y[8];
    for (i = 0; i < 8; i++) {
        y[i] = b[i];
        for (j = 0; j < i; j++) {
            y[i] -= L[i][j] * y[j];
        }
    }

    // Resolver U * x = y (sustitución hacia atrás)
    for (i = 7; i >= 0; i--) {
        x[i] = y[i];
        for (j = i + 1; j < 8; j++) {
            x[i] -= U[i][j] * x[j];
        }
        if (U[i][i] != 0) {
            x[i] /= U[i][i];
        } else {
            L[j][i] /= 1;
        }
    }
}

// Función para calcular la matriz de transformación perspectiva
void P2Coefs(Section *source, Section *dest, long double (*m)[9]) {
    // Matriz del sistema de ecuaciones
    long double A[8][8] = {
        {source->LU.x, source->LU.y, 1.0, 0, 0, 0, -source->LU.x * dest->LU.x, -source->LU.y * dest->LU.x},
        {source->RU.x, source->RU.y, 1.0, 0, 0, 0, -source->RU.x * dest->RU.x, -source->RU.y * dest->RU.x},
        {source->LD.x, source->LD.y, 1.0, 0, 0, 0, -source->LD.x * dest->LD.x, -source->LD.y * dest->LD.x},
        {source->RD.x, source->RD.y, 1.0, 0, 0, 0, -source->RD.x * dest->RD.x, -source->RD.y * dest->RD.x},
        {0, 0, 0, source->LU.x, source->LU.y, 1.0, -source->LU.x * dest->LU.y, -source->LU.y * dest->LU.y},
        {0, 0, 0, source->RU.x, source->RU.y, 1.0, -source->RU.x * dest->RU.y, -source->RU.y * dest->RU.y},
        {0, 0, 0, source->LD.x, source->LD.y, 1.0, -source->LD.x * dest->LD.y, -source->LD.y * dest->LD.y},
        {0, 0, 0, source->RD.x, source->RD.y, 1.0, -source->RD.x * dest->RD.y, -source->RD.y * dest->RD.y}
    };

    // Vector de términos independientes
    long double b[8] = {dest->LU.x, dest->RU.x, dest->LD.x, dest->RD.x, dest->LU.y, dest->RU.y, dest->LD.y, dest->RD.y};

    // Solución del sistema
    long double x[8];
    solveLU(&A, b, x);

    // Rellenar la matriz de transformación
    for (int i = 0; i < 8; i++) {
        (*m)[i] = x[i];
    }
    (*m)[8] = 1.0;
}

typedef struct {
    float rho;
    float theta;
    float slope;
    float intercept;
    bool is_vertical;
    bool is_valid = false;
} Line;

void apliFil(unsigned char *img_in, unsigned char *img_out, int width, int height, int img_tam) {
    float kernel[5][5] = {
        {0.01195524648676667, 0.023285640551474758, 0.02908024586668896, 0.023285640551474758, 0.01195524648676667},
        {0.023285640551474758, 0.04535423476987057, 0.05664058479678963, 0.04535423476987057, 0.023285640551474758},
        {0.02908024586668896,  0.05664058479678963,  0.0707355302630646,  0.05664058479678963,  0.02908024586668896},
        {0.023285640551474758, 0.04535423476987057, 0.05664058479678963, 0.04535423476987057, 0.023285640551474758},
        {0.01195524648676667, 0.023285640551474758, 0.02908024586668896, 0.023285640551474758, 0.01195524648676667}
    };

    int k_rad = 2;

    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            float sum = 0.0;

            for (int ky = -k_rad; ky <= k_rad; ky++) {
                for (int kx = -k_rad; kx <= k_rad; kx++) {
                    int pX = x + kx;
                    int pY = y + ky;

                    if (pX < 0) pX = 0;
                    if (pX >= width) pX = width - 1;
                    if (pY < 0) pY = 0;
                    if (pY >= height) pY = height - 1;

                    sum += img_in[pY * width + pX] * kernel[ky + k_rad][kx + k_rad];
                }
            }

            img_out[y * width + x] = (unsigned char)(sum);
        }
    }
}

void apliSobel(unsigned char *img_in, unsigned char *img_out, unsigned char *dir_img, int width, int height, int img_tam) {
    short kernel_x[3][3] = {
        {1,  0,  -1},
        {2,  0,  -2},
        {1,  0,  -1}
    };

    short kernel_y[3][3] = {
        {1,  2,  1},
        {0,  0,  0},
        {-1, -2, -1}
    };

    unsigned char k_rad = 1;

    float dir = 0;

    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            float sum_x = 0.0;
            float sum_y = 0.0;

            for (int ky = -k_rad; ky <= k_rad; ky++) {
                for (int kx = -k_rad; kx <= k_rad; kx++) {
                    int pX = x + kx;
                    int pY = y + ky;

                    if (pX < 0) pX = 0;
                    if (pX >= width) pX = width - 1;
                    if (pY < 0) pY = 0;
                    if (pY >= height) pY = height - 1;

                    sum_x += img_in[pY * width + pX] * kernel_x[ky + k_rad][kx + k_rad];
                    sum_y += img_in[pY * width + pX] * kernel_y[ky + k_rad][kx + k_rad];
                }
            }

            img_out[y * width + x] = (unsigned char)(sqrt(pow(sum_x, 2) + pow(sum_y, 2)));

            dir = atan2(sum_y, sum_x) * 180.0 / M_PI;

            dir_img[y * width + x] = (unsigned char)((dir < 0) ? dir+180 : dir);
        }
    }
}

void apliNonMaxSup(unsigned char *img_in, unsigned char *img_out, unsigned char *dir_img, int width, int height, int img_tam) {
    for (int y = 1; y < height - 1; y++) {
        for (int x = 1; x < width - 1; x++) {
            int index = y * width + x;

            unsigned char c_mag = img_in[index];
            unsigned char c_dir = dir_img[index];

            unsigned char mag1 = 0, mag2 = 0;

            // Determine which neighbors to compare based on gradient direction
            // Use rounded angles (0, 45, 90, 135) to decide direction
            if ((c_dir >= 0 && c_dir < 22) || (c_dir >= 157 && c_dir <= 180)) {
                // Gradient direction is approximately 0 degrees (horizontal)
                mag1 = img_in[y * width + (x + 1)];  // right neighbor
                mag2 = img_in[y * width + (x - 1)];  // left neighbor
            }
            else if (c_dir >= 22 && c_dir < 67) {
                // Gradient direction is approximately 45 degrees
                mag1 = img_in[(y - 1) * width + (x + 1)];  // top-right neighbor
                mag2 = img_in[(y + 1) * width + (x - 1)];  // bottom-left neighbor
            }
            else if (c_dir >= 67 && c_dir < 112) {
                // Gradient direction is approximately 90 degrees (vertical)
                mag1 = img_in[(y - 1) * width + x];  // top neighbor
                mag2 = img_in[(y + 1) * width + x];  // bottom neighbor
            }
            else if (c_dir >= 112 && c_dir < 157) {
                // Gradient direction is approximately 135 degrees
                mag1 = img_in[(y - 1) * width + (x - 1)];  // top-left neighbor
                mag2 = img_in[(y + 1) * width + (x + 1)];  // bottom-right neighbor
            }

            // Suppress the current pixel if it's not a local maximum
            if (c_mag >= mag1 && c_mag >= mag2) {
                img_out[index] = c_mag;  // Preserve the pixel value
            } else {
                img_out[index] = 0;  // Suppress the pixel
            }
        }
    }
}

void apliHyste(unsigned char *img_in, unsigned char *img_out, int width, int height, int img_tam, unsigned char low_thre, unsigned char high_thre) {
    for (int y = 1; y < height - 1; y++) {
        for (int x = 1; x < width - 1; x++) {
            int index = y * width + x;

            if (img_in[index] >= high_thre) {
                img_out[index] = 255;
            } else if (img_in[index] >= low_thre) {
                bool is_cse = false;
                for (int i = -1; i <= 1; i++) {
                    for (int j = -1; j <= 1; j++) {
                        int neir_index = (y + i) * width + (x + j);
                        if (img_out[neir_index] == 255) {
                            is_cse = true;
                            break;
                        }
                    }
                    if (is_cse) break;
                }
                if (is_cse) {
                    img_out[index] = 255;
                } else {
                    img_out[index] = 0;
                }
            } else {
                img_out[index] = 0;
            }
        }
    }
}

void apliHoughTrans(unsigned char *img_in, int width, int height, int img_tam, float res_theta, unsigned int thre, Line* lines) {
    unsigned int steps_theta = (unsigned int) (M_PI/res_theta);
    unsigned int max_rho = (unsigned int)(sqrt(width * width + height * height));
    unsigned int acum_size = ((2*max_rho)+1) * steps_theta;

    unsigned int *acum = (unsigned int *) malloc(acum_size * sizeof(unsigned int));

    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            if (img_in[y * width + x] == 255) {
                float theta = 0;
                for (unsigned int theta_index = 0; theta < M_PI; theta+=res_theta, theta_index++) {
                    int rho = (int)(x * cos(theta) + y * sin(theta));

                    acum[(rho + max_rho) * steps_theta + theta_index]++;
                }
            }
        }
    }

    unsigned int pos = 0;
    for (unsigned int i = 0; i < acum_size; i++) {
        if (acum[i] >= thre) {
            Line line;
            unsigned int theta_index = i % steps_theta;
            unsigned int rho_index = (unsigned int) (i / steps_theta);

            line.is_valid = true;

            line.rho = (int)(rho_index - max_rho);
            line.theta = theta_index * (M_PI / steps_theta);

            if (fabs(line.theta - M_PI / 2) < 0.01) {
                line.is_vertical = true;
                line.slope = 1;
                line.intercept = line.rho;
            } else {
                line.is_vertical = false;
                line.slope = -cos(line.theta) / sin(line.theta);
                line.intercept = line.rho / sin(line.theta);
            }

            if (pos > 0){
                if (fabs(lines[pos-1].theta - line.theta) <= 20.0 && fabs(lines[pos-1].rho - line.rho) <= 1.9) {
                    lines[pos-1].theta = (lines[pos-1].theta + line.theta)/2;
                    lines[pos-1].rho = (lines[pos-1].rho + line.rho)/2;
                    lines[pos-1].slope = (lines[pos-1].slope + line.slope)/2;
                    lines[pos-1].intercept = (lines[pos-1].intercept + line.intercept)/2;
                } else {
                    lines[pos] = line;
                    pos++;
                }
            } else {
                lines[pos] = line;
                pos++;
            }

            /*printf("Line(rho=%.2f, theta=%.2f, slope=%.2f, intercept=%.2f, is_vertical=%s, is_valid=%s, pos=%d, abs=%.4f)\n",
                 line.rho,
                 line.theta,
                 line.slope,
                 line.intercept,
                 line.is_vertical ? "true" : "false",
                 line.is_valid ? "true" : "false",
                 pos,
                 fabs(lines[pos-1].theta - line.theta) <= 5.0 && fabs(lines[pos-1].rho - line.rho) <= 0.2);*/
        }
    }

    free(acum);
}

void set_pixel(unsigned char* image, int width, int height, int channels, int x, int y, unsigned char r, unsigned char g, unsigned char b) {
    if (x >= 0 && x < width && y >= 0 && y < height) {
        int index = (y * width + x) * channels;
        image[index] = r;
        image[index + 1] = g;
        image[index + 2] = b;
    }
}

// Bresenham's line algorithm for drawing lines
void draw_line(unsigned char* image, int width, int height, int channels, int x1, int y1, int x2, int y2, unsigned char r, unsigned char g, unsigned char b) {
    int dx = abs(x2 - x1);
    int dy = abs(y2 - y1);
    int sx = (x1 < x2) ? 1 : -1;
    int sy = (y1 < y2) ? 1 : -1;
    int err = dx - dy;

    while (1) {
        set_pixel(image, width, height, channels, x1, y1, r, g, b);

        if (x1 == x2 && y1 == y2) break;
        int e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            x1 += sx;
        }
        if (e2 < dx) {
            err += dx;
            y1 += sy;
        }
    }
}

// Function to draw lines on the image
void draw_lines(unsigned char* image, int width, int height, int channels, Line* lines, int num_lines) {
    for (int i = 0; i < num_lines; i++) {
        if (lines[i].is_valid) {
            Line line = lines[i];

            if (line.is_vertical) {
                // Draw vertical line at x = line.intercept
                int x = (int)line.intercept;
                for (int y = 0; y < height; y++) {
                    set_pixel(image, width, height, channels, x, y, 255, 0, 0);  // Red color for the line
                }
            } else {
                // Calculate intersection points with image boundaries (top, bottom)
                int y1 = 0;  // Intersection with the top of the image
                int x1 = (int)((y1 - line.intercept) / line.slope);

                int y2 = height - 1;  // Intersection with the bottom of the image
                int x2 = (int)((y2 - line.intercept) / line.slope);

                // Draw the line using Bresenham's line algorithm
                draw_line(image, width, height, channels, x1, y1, x2, y2, 255, 0, 0);  // Red color for the line
            }
        }
    }
}

// Función para buscar líneas en una línea de píxeles específica y actualizar el buffer
int buscar_primer_pixel_rojo_con_buffer(unsigned char *img, int width, int height, int channels, int linea_y, char *buffer) {
    if (linea_y < 0 || linea_y >= height || channels < 3 || buffer == NULL) {
        printf("Error: línea fuera de rango, imagen inválida o buffer no proporcionado.\n");
        return -1;
    }

    int centro_x = width / 2;
    int primer_rojo_izq = -1;
    int primer_rojo_der = -1;

    // Recorrer desde el centro hacia la izquierda
    for (int x = centro_x; x >= 0; x--) {
        int index = (linea_y * width + x) * channels;
        if (img[index] == 255 && img[index + 1] == 0 && img[index + 2] == 0) {
            primer_rojo_izq = x;
            break;
        }
    }

    // Recorrer desde el centro hacia la derecha
    for (int x = centro_x; x < width; x++) {
        int index = (linea_y * width + x) * channels;
        if (img[index] == 255 && img[index + 1] == 0 && img[index + 2] == 0) {
            primer_rojo_der = x;
            break;
        }
    }

    // Inicializar el buffer
    memset(buffer, '0', 10);
    buffer[10] = '\0';

    // Si no se encuentran líneas de ninguno de los lados
    if (primer_rojo_izq == -1 && primer_rojo_der == -1) {
        memset(buffer, '1', 10);
        return 0;
    }

    // Calcular las posiciones relativas en el buffer
    int distancia_izq = (primer_rojo_izq != -1) ? (centro_x - primer_rojo_izq) / (width / 10) : 5;
    int distancia_der = (primer_rojo_der != -1) ? (primer_rojo_der - centro_x) / (width / 10) : 5;

    // Actualizar el buffer
    if (primer_rojo_izq != -1) {
        buffer[5 - distancia_izq] = '1';
    } else {
        // No encontró línea, encender desde el extremo izquierdo
        memset(buffer, '1', 5);
    }

    if (primer_rojo_der != -1) {
        buffer[5 + distancia_der] = '1';
    } else {
        // No encontró línea, encender desde el extremo derecho
        memset(buffer + 5, '1', 5);
    }

    return 0;
}

int main()
{
    gyro_device_handler = wiringPiI2CSetup(MPU6050_I2C_DEVICE_ADDRESS);
    wiringPiI2CWriteReg8(gyro_device_handler,REGISTER_FOR_POWER_MANAGEMENT,SLEEP_MODE_DISABLED);

    /* Wait for sensor to stabilize */
    delay(150);

    double roll_p;
    double pitch_p;

    Section org_im = {
        .LU = { .x = 255.0, .y = 458.0 },
        .RU = { .x = 323.0, .y = 458.0 },
        .LD = { .x = 143.0, .y = 633.0 },
        .RD = { .x = 418.0, .y = 633.0 }
    };

    long double m[9];

    unsigned char *img = (unsigned char *)malloc(IMAGE_WIDTH * IMAGE_HEIGHT * 3);
    if (!img) {
        perror("Error al asignar memoria para la imagen");
        munmap(buffer, buf.length);
        close(fd);
        return 1;
    }

    size_t gimg_size = IMAGE_WIDTH * IMAGE_HEIGHT;
    size_t img_tam = IMAGE_WIDTH * IMAGE_HEIGHT * channels;

    unsigned char *gimg = (unsigned char *) malloc(gimg_size);

    if (gimg == NULL) {
        printf("Error alocando memoria\n");
        exit(1);
    }

    unsigned char *fimg = (unsigned char *) malloc(gimg_size);

    if (fimg == NULL) {
        printf("Error alocando memoria\n");
        exit(1);
    }

    //Sobel
    //unsigned char *simg = (unsigned char *) malloc(gimg_size);
    unsigned char *dimg = (unsigned char *) malloc(gimg_size);

    /*if (simg == NULL) {
        printf("Error alocando memoria\n");
        exit(1);
    }

    unsigned char *nmsimg = (unsigned char *) malloc(gimg_size);

    if (nmsimg == NULL) {
        printf("Error alocando memoria\n");
        exit(1);
    }

    unsigned char *hysimg = (unsigned char *) malloc(gimg_size);

    if (hysimg == NULL) {
        printf("Error alocando memoria\n");
        exit(1);
    }*/

    clock_t t;
    double time_taken;

    while(rep == 0){
        t = clock();

        const char *device = "/dev/video0";
        int fd = open(device, O_RDWR);
        if (fd == -1) {
            perror("Error al abrir el dispositivo de video");
            return 1;
        }

        // Consultar capacidades del dispositivo
        struct v4l2_capability cap;
        if (ioctl(fd, VIDIOC_QUERYCAP, &cap) == -1) {
            perror("Error al consultar capacidades del dispositivo");
            close(fd);
            return 1;
        }

        // Configurar formato de video
        struct v4l2_format fmt;
        memset(&fmt, 0, sizeof(fmt));
        fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        fmt.fmt.pix.width = IMAGE_WIDTH;
        fmt.fmt.pix.height = IMAGE_HEIGHT;
        fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG; // Formato MJPEG
        fmt.fmt.pix.field = V4L2_FIELD_NONE;

        if (ioctl(fd, VIDIOC_S_FMT, &fmt) == -1) {
            perror("Error al configurar el formato de video");
            close(fd);
            return 1;
        }

        // Solicitar buffers de memoria
        struct v4l2_requestbuffers req;
        memset(&req, 0, sizeof(req));
        req.count = 1;  // Usar un solo buffer
        req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        req.memory = V4L2_MEMORY_MMAP;

        if (ioctl(fd, VIDIOC_REQBUFS, &req) == -1) {
            perror("Error al solicitar buffers");
            close(fd);
            return 1;
        }

        // Mapear el buffer
        struct v4l2_buffer buf;
        memset(&buf, 0, sizeof(buf));
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = 0;

        if (ioctl(fd, VIDIOC_QUERYBUF, &buf) == -1) {
            perror("Error al consultar buffer");
            close(fd);
            return 1;
        }

        unsigned char *buffer = (unsigned char*)mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, buf.m.offset);
        if (buffer == MAP_FAILED) {
            perror("Error al mapear buffer");
            close(fd);
            return 1;
        }

        // Iniciar la captura de video
        if (ioctl(fd, VIDIOC_STREAMON, &buf.type) == -1) {
            perror("Error al iniciar la transmisión");
            close(fd);
            return 1;
        }

        // Poner el buffer en cola
        if (ioctl(fd, VIDIOC_QBUF, &buf) == -1) {
            perror("Error al poner el buffer en cola");
            close(fd);
            return 1;
        }

        // Esperar y sacar el buffer de la cola
        Gyro(&roll_p, &pitch_p);
        if (ioctl(fd, VIDIOC_DQBUF, &buf) == -1) {
            perror("Error al sacar el buffer de la cola");
            close(fd);
            return 1;
        }

        // Guardar los datos capturados (ya en formato JPEG) en un archivo
        /*FILE *file = fopen("captura.jpg", "wb");
        if (!file) {
            perror("Error al abrir el archivo para guardar la imagen");
            munmap(buffer, buf.length);
            close(fd);
            return 1;
        }
        printf("Imagen guardada como 'captura.jpg'\n");

        fwrite(buffer, buf.bytesused, 1, file);
        fclose(file);*/

        if (decode_mjpeg_to_rgb(buffer, buf.bytesused, img) == -1) {
            fprintf(stderr, "Error decoding MJPEG frame\n");
            free(img);
            munmap(buffer, buf.length);
            close(fd);
            return 1;
        }

        if (ioctl(fd, VIDIOC_STREAMOFF, &buf.type) == -1) {
            perror("Error al iniciar la transmisión");
            close(fd);
            return 1;
        }

        munmap(buffer, buf.length);
        close(fd);

        //stbi_write_jpg("pruebaimg.jpg", IMAGE_WIDTH, IMAGE_HEIGHT, 3, img, 100);

        printf("Roll: %.4f, Pitch: %.4f\n", roll_p, pitch_p);

        Section im = {
            .LU = { .x = org_im.LD.x + ((org_im.LU.x - org_im.LD.x) * f_img_per * 0.01),
                    .y = org_im.LD.y + ((org_im.LU.y - org_im.LD.y) * f_img_per * 0.01) },
            .RU = { .x = org_im.RD.x + ((org_im.RU.x - org_im.RD.x) * f_img_per * 0.01),
                    .y = org_im.RD.y + ((org_im.RU.y - org_im.RD.y) * f_img_per * 0.01) },
            .LD = { .x = org_im.LD.x, .y = org_im.LD.y },
            .RD = { .x = org_im.RD.x, .y = org_im.RD.y }
        };

        int f_height = im.LD.y - im.LU.y;
        int f_width = ((im.RU.x - im.LU.x) + (im.RD.x - im.LD.x))/2;

        Section per = {
            .LU = { .x = 0.0, .y = 0.0 },
            .RU = { .x = (long double)f_width, .y = 0.0 },
            .LD = { .x = 0.0, .y = (long double)f_height },
            .RD = { .x = (long double)f_width, .y = (long double)f_height }
        };

        /*P2Coefs(&im, &per, &m);

        printf("%.8Lf, %.8Lf, %.8Lf, %.8Lf, %.8Lf, %.8Lf, %.8Lf, %.8Lf, %.8Lf\n", m[0], m[1], m[2], m[3], m[4], m[5], m[6], m[7], m[8]);

        int ptimg_size = f_width * f_height;

        unsigned char *ptimg = (unsigned char *) malloc(ptimg_size);

        if (ptimg == NULL) {
            printf("Error alocando memoria\n");
            exit(1);
        }

        for(unsigned char *p = ptimg; p != ptimg + ptimg_size; p++) {
            *p = 255;
        }

        int nwidth = IMAGE_WIDTH*channels;

        int yme = (im.LU.y <= im.RU.y) ? im.LU.y : im.RU.y;

        int yma = (im.LD.y >= im.RD.y) ? im.LD.y : im.RD.y;

        for (int i = 0; i < IMAGE_HEIGHT; i++) {
            for (int j = 0; j < nwidth; j += channels) {
                if (j >= ((int)im.LD.x)*channels && j <= ((int)im.RD.x)*channels && i >= yme && i <= yma) {
                    int jj = j/channels;
                    double denom = (m[6]*jj + m[7]*i + m[8]);
                    int u = (int)((m[0]*jj + m[1]*i + m[2])/denom);
                    int v = (int)((m[3]*jj + m[4]*i + m[5])/denom);
                    if (u >= 0 && u < f_width && v >= 0 && v < f_height) {
                        ptimg[v*f_width + u] = (unsigned char)((img[i*nwidth + j] + img[i*nwidth + j + 1] + img[i*nwidth + j + 2])/3);
                    }
                }
            }
        }

        unsigned char prev = 0;

        for (unsigned char *p = ptimg; p < ptimg + ptimg_size; p++) {
            if (*p == 0) {
                *p = prev;
            } else {
                prev = *p;
            }
        }*/

        for (unsigned char *p = img, *pg = gimg; p != img + img_tam; p += channels, pg += 1){
            *pg = (uint8_t)((*p + *(p+1) + *(p+2))/3.0);
        }

        apliFil(gimg, fimg, IMAGE_WIDTH, IMAGE_HEIGHT, img_tam);

        apliSobel(fimg, gimg, dimg, IMAGE_WIDTH, IMAGE_HEIGHT, img_tam);

        apliNonMaxSup(gimg, fimg, dimg, IMAGE_WIDTH, IMAGE_HEIGHT, img_tam);

        apliHyste(fimg, gimg, IMAGE_WIDTH, IMAGE_HEIGHT, img_tam, 20, 50);

        //Hough Transform
        Line lines[100];

        apliHoughTrans(gimg, IMAGE_WIDTH, IMAGE_HEIGHT, img_tam, 0.2, 50, lines);

        draw_lines(img, IMAGE_WIDTH, IMAGE_HEIGHT-40, channels, lines, 100);

        char chbuffer[11]; // 10 caracteres más el terminador nulo

        buscar_primer_pixel_rojo_con_buffer(img, IMAGE_WIDTH, IMAGE_HEIGHT, channels, IMAGE_HEIGHT - 50, chbuffer);

        // Imprimir el contenido del buffer
        printf("Buffer generado: %s\n", chbuffer);

        // Construir el comando
        char command[256];
        snprintf(command, sizeof(command), "sudo aleds/bin/python3 leds.py %c %c %c %c %c %c %c %c %c %c",
                chbuffer[0], chbuffer[1], chbuffer[2], chbuffer[3], chbuffer[4], chbuffer[5], chbuffer[6], chbuffer[7], chbuffer[8], chbuffer[9]);

        // Ejecutar el comando
        system(command);

        t = clock() - t;
        time_taken = ((double)t)/CLOCKS_PER_SEC;
        printf("Tardo %f segundos en procesar.\n", time_taken);
        printf("FPS: %f.\n", 1/(time_taken));
    }

    //printf("Se guardo la imagen con ancho: %dpx, alto: %dpx\n", f_width, f_height);
    //stbi_write_jpg("cpimg.jpg", f_width, f_height, 1, ptimg, 100);

    printf("No lane detected");

    //free(ptimg);

    t = clock() - t;
    time_taken = ((double)t)/CLOCKS_PER_SEC;
    printf("Tardo %f segundos en guardar la imgajen.\n", time_taken);


    // Liberar recursos
    free(img);
    free(gimg);
    free(dimg);
    /*free(fimg);
    free(simg);
    free(dimg);
    free(nmsimg);
    free(hysimg);*/

}
