#include "image.h"

#include <iostream>
#include <fstream>
#include <string>

// float data[width*height]
// uint32_t buffer[width*height]

void flip(const uint32_t width, const uint32_t height, const float* data, uint32_t* buffer) {
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