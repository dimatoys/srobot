#ifndef _IMAGE_H
#define _IMAGE_H

#include <cstdint>

void flip(const uint32_t width, const uint32_t height, const float* data, uint32_t* buffer);
void BWtoRGB(const uint32_t width, const uint32_t height, const float* data, uint8_t* img);

#endif
