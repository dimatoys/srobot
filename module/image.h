#ifndef _IMAGE_H
#define _IMAGE_H

#include <cstdint>

void flip(const uint32_t width, const uint32_t height, const float* data, uint32_t* buffer);
void BWtoRGB(const uint32_t width, const uint32_t height, const float* data, uint8_t* img);

void gaussCore(float* core, uint16_t q);
void applyGaussFilter(uint32_t width, uint32_t height, float* img, uint16_t* dimg, uint8_t q, float* core);


#endif
