#include "image.h"

#include <iostream>
#include <fstream>
#include <string>
#include <cstring>
#include <cmath>

using namespace std;

// float core[(q*2+1)*(q*2+1)]
void gaussCore(float* core, uint16_t q) {
    auto msize = q * 2 + 1;
    for (uint16_t i = 0; i <= q; ++i) {
        for (uint16_t j = i; j <= q; ++j) {
            float v = exp(-(i * i + j * j) / (float)q);
            core[q + i + (q + j) * msize] = v;
            core[q + j + (q + i) * msize] = v;
            core[q - i + (q + j) * msize] = v;
            core[q - j + (q + i) * msize] = v;
            core[q + i + (q - j) * msize] = v;
            core[q + j + (q - i) * msize] = v;
            core[q - i + (q - j) * msize] = v;
            core[q - j + (q - i) * msize] = v;
        }
    }
}

void applyGaussFilter(uint32_t width, uint32_t height, float* img, uint16_t* dimg, uint8_t q, float* core) {
    const uint32_t csize = q * 2 + 1;

    for (uint32_t y = 0; y < height; ++y) {
        for (uint32_t x = 0; x < width; ++x) {
            uint32_t baseIdx = x + y * width;

            float sum = 0;
            float weights = 0;

            uint32_t x1 = (x >= q) ? 0 : (q - x);
            uint32_t y1 = (y >= q) ? 0 : (q - y);
            uint32_t x2 = (x + q < width) ? csize : (width + q - x);
            uint32_t y2 = (y + q < height) ? csize : (height + q - y);

            for (uint32_t my = y1; my < y2; ++my) {
                uint32_t s = baseIdx - q + width * (my - q);
                uint32_t s2 = my * csize;
                for (uint32_t mx = x1; mx < x2; ++ mx) {
                    auto v = img[s + mx];
                    if (v > 0) {
                        auto w = core[s2 + mx];
                        sum += v * w;
                        weights += w;
                    }
                }
            }
            dimg[baseIdx] = sum / weights;
        }
    }
}

// float data[width*height]
// uint32_t buffer[width*height]

void flip(const uint32_t width, const uint32_t height, const float* data, uint16_t* buffer) {
    uint32_t i = 0;
    for (uint32_t y = 0; y < height; ++y) {
        for (uint32_t x = 0; x < width; ++x) {
            buffer[x + width * (height - y - 1)] = (uint16_t)data[i++];
        }
    }

}

// float data[width * height]
// uint38_t img[width * height * 3]

void BWtoRGB(const uint32_t width, const uint32_t height, const float* data, uint8_t* img) {

    uint32_t size = width * height;

    float min = 1000000000;
    float max = -100000;

    for (uint32_t i = 0; i < size; ++i) {
        auto v = data[i];
        if (v > 0) {
            if (v < min ) {
                min = v;
            }
            if(v > max) {
                max = v;
            }
        }
    }
    double k = 255.0 / (max - min);


    int j = 0;
    int i = 0;
    for (uint32_t y = 0; y < height; ++y) {
        for (uint32_t x = 0; x < width; ++x) {
            auto v = data[j++];
            if (v <= 0) {
                img[i++] = 255;
                img[i++] = 128;
                img[i++] = 128;

            } else {
                uint8_t v2 = (uint8_t)((v - min) * k);
                img[i++] = v2;
                img[i++] = v2;
                img[i++] = v2;
            }
        }
    }
}

uint32_t fy(uint32_t d){
    float c[4] = { 5.89494380e-06, -7.03269136e-03,  3.00940632e+00, -3.28471368e+02};
    float v = 0;
    for (int k = 0; k < 4; ++k) {
        v = v * d + c[k];
    }
    return (uint32_t)v;
}


uint32_t FY[2000];

void generateFy(uint32_t dheight) {
    for (uint32_t d = 0; d < dheight; ++d) {
        FY[d] = fy(d);
    }
}

void generateMap(const uint32_t width, const uint32_t height, const float* img, const uint32_t dheight, uint16_t* dimg) {
    memset(dimg, 0, width * dheight * sizeof(uint16_t));

    for (uint32_t y = 0; y < height; ++y) {
        auto x0 = width * (height - y - 1);
        for (uint32_t x = 0; x < width; ++x) {
            float d = img[x + x0];
            if ((d >= 0) && (d < dheight)) {
                uint32_t di = (uint32_t)d;
                uint32_t yd = FY[di];
                uint32_t h;
                if (yd < y) {
                    h = (y - yd) * di / 250;
                    if (h > 1000) {
                        h = 1000;
                    }
                } else {
                    h = 1;
                }
                auto ptr = &dimg[x + di * width];
                if (h > *ptr) {
                    *ptr = (uint16_t)h;
                }
            }

        }
    }

}