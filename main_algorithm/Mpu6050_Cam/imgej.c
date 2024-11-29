#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

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

            // printf("Line(rho=%.2f, theta=%.2f, slope=%.2f, intercept=%.2f, is_vertical=%s, is_valid=%s, pos=%d, abs=%.4f)\n",
            //     line.rho,
            //     line.theta,
            //     line.slope,
            //     line.intercept,
            //     line.is_vertical ? "true" : "false",
            //     line.is_valid ? "true" : "false",
            //     pos,
            //     fabs(lines[pos-1].theta - line.theta) <= 5.0 && fabs(lines[pos-1].rho - line.rho) <= 0.2);
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

int main(void) {
    clock_t t;
    t = clock();

    int width, height, channels;
    unsigned char *img = stbi_load("cpimg.jpg", &width, &height, &channels, 0);

    if (img == NULL) {
        printf("Error\n");
        exit(1);
    }

    printf("Se cargo la imagen con ancho: %dpx, alto: %dpx y %d canales\n", width, height, channels);

    //Convertir a escala de grises
    size_t img_tam = width * height * channels;
    size_t gimg_size = width * height;

    unsigned char *gimg = (unsigned char *) malloc(gimg_size);

    if (gimg == NULL) {
        printf("Error alocando memoria\n");
        exit(1);
    }

    for (unsigned char *p = img, *pg = gimg; p != img + img_tam; p += channels, pg += 1){
        *pg = (uint8_t)((*p + *(p+1) + *(p+2))/3.0);
    }

    //Filtro Gausiano 5*5
    unsigned char *fimg = (unsigned char *) malloc(gimg_size);

    if (fimg == NULL) {
        printf("Error alocando memoria\n");
        exit(1);
    }

    apliFil(gimg, fimg, width, height, img_tam);

    //Sobel
    unsigned char *simg = (unsigned char *) malloc(gimg_size);
    unsigned char *dimg = (unsigned char *) malloc(gimg_size);

    if (simg == NULL) {
        printf("Error alocando memoria\n");
        exit(1);
    }

    apliSobel(fimg, simg, dimg, width, height, img_tam);

    //NMS
    unsigned char *nmsimg = (unsigned char *) malloc(gimg_size);

    if (nmsimg == NULL) {
        printf("Error alocando memoria\n");
        exit(1);
    }

    apliNonMaxSup(simg, nmsimg, dimg, width, height, img_tam);

    //Edge detection Hysteresis
    unsigned char *hysimg = (unsigned char *) malloc(gimg_size);

    if (hysimg == NULL) {
        printf("Error alocando memoria\n");
        exit(1);
    }

    apliHyste(nmsimg, hysimg, width, height, img_tam, 20, 50);

    //Hough Transform
    Line lines[100];

    apliHoughTrans(hysimg, width, height, img_tam, 0.000001, 0, lines);

    draw_lines(img, width, height, channels, lines, 100);

    t = clock() - t;
    double time_taken = ((double)t)/CLOCKS_PER_SEC;
    printf("Tardo %f segundos en procesar.\n", time_taken);
    printf("FPS: %f.\n", 1/(time_taken));

    stbi_write_jpg("lineimg.jpg", width, height, channels, img, 100);

    stbi_write_jpg("gimg.jpg", width, height, 1, gimg, 100);

    stbi_write_jpg("fimg.jpg", width, height, 1, fimg, 100);

    stbi_write_jpg("simg.jpg", width, height, 1, simg, 100);

    stbi_write_jpg("dimg.jpg", width, height, 1, dimg, 100);

    stbi_write_jpg("nmsimg.jpg", width, height, 1, nmsimg, 100);

    stbi_write_jpg("hysimg.jpg", width, height, 1, hysimg, 100);

    stbi_image_free(img);
    free(gimg);
    free(fimg);
    free(simg);
    free(dimg);
    free(nmsimg);
    free(hysimg);
}
