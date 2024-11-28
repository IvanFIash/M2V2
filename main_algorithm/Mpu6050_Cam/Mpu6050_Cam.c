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

#define channels 3
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

int ws = 1280;
int hs = 720;
int ch = 3;

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

int main()
{
    gyro_device_handler = wiringPiI2CSetup(MPU6050_I2C_DEVICE_ADDRESS);
    wiringPiI2CWriteReg8(gyro_device_handler,REGISTER_FOR_POWER_MANAGEMENT,SLEEP_MODE_DISABLED);

    /* Wait for sensor to stabilize */
    delay(150);

    double roll_p;
    double pitch_p;

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

    Section org_im = {
        .LU = { .x = 255.0, .y = 458.0 },
        .RU = { .x = 323.0, .y = 458.0 },
        .LD = { .x = 143.0, .y = 633.0 },
        .RD = { .x = 418.0, .y = 633.0 }
    };

    long double m[9];

    clock_t t;
    t = clock();

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

    printf("Imagen guardada como 'captura.jpg'\n");

    /*unsigned char *img = (unsigned char *)malloc(IMAGE_WIDTH * IMAGE_HEIGHT * 3);
    if (!img) {
        perror("Error al asignar memoria para la imagen");
        munmap(buffer, buf.length);
        close(fd);
        return 1;
    }

    memcpy(img, buffer, buf.bytesused);*/

    unsigned char *decoded_img = (unsigned char*)stbi_load_from_memory((unsigned char*)buffer, ws*hs*3, &ws, &hs, &ch, 3);
    if (!decoded_img) {
        fprintf(stderr, "Error al decodificar los datos JPEG\n");
        munmap(buffer, buf.length);
        close(fd);
        return 1;
    }
    printf("La imagen decodificada tiene ancho: %dpx, alto: %dpx\n", ws, hs);

    stbi_write_jpg("img.jpg", ws, hs, 1, decoded_img, 100);

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

    P2Coefs(&im, &per, &m);

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
                    ptimg[v*f_width + u] = (unsigned char)((buffer[i*nwidth + j] + buffer[i*nwidth + j + 1] + buffer[i*nwidth + j + 2])/3);
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
    }

    t = clock() - t;
    double time_taken = ((double)t)/CLOCKS_PER_SEC;
    printf("Tardo %f segundos en procesar.\n", time_taken);
    printf("FPS: %f.\n", 1/(time_taken));

    // Guardar los datos capturados (ya en formato JPEG) en un archivo
    FILE *file = fopen("captura.jpg", "wb");
    if (!file) {
        perror("Error al abrir el archivo para guardar la imagen");
        munmap(buffer, buf.length);
        close(fd);
        return 1;
    }

    printf("Se guardo la imagen con ancho: %dpx, alto: %dpx\n", f_width, f_height);
    stbi_write_jpg("cpimg.jpg", f_width, f_height, 1, ptimg, 100);

    free(ptimg);

    fwrite(buffer, buf.bytesused, 1, file);
    fclose(file);

    t = clock() - t;
    time_taken = ((double)t)/CLOCKS_PER_SEC;
    printf("Tardo %f segundos en guardar la imgajen.\n", time_taken);


    // Liberar recursos
    free(decoded_img);
    munmap(buffer, buf.length);
    close(fd);

}
