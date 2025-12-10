#ifndef _IMAGE_H
#define _IMAGE_H

#include <cstdint>

void flip(const uint32_t width, const uint32_t height, const float* data, uint16_t* buffer);
void BWtoRGB(const uint32_t width, const uint32_t height, const float* data, uint8_t* img);
void BWtoRGB(const uint32_t width, const uint32_t height, const float* data, uint8_t* img, bool cross);
void BWtoRGB(const uint32_t width, const uint32_t height, const double* data, uint8_t* img, bool cross);

void gaussCore(float* core, uint16_t q);
void applyGaussFilter(uint32_t width, uint32_t height, float* img, uint16_t* dimg, uint8_t q, float* core);

void generateFy(uint32_t dheight);
void generateMap(const uint32_t width, const uint32_t height, const float* img, const uint32_t dheight, uint16_t* dimg);

void extract_walls(const uint32_t width, const uint32_t height, const float* img, const uint32_t parts, double* wall_dist);

double count_sf(const double* img,
                const int32_t width,
                const int32_t height,
                const int32_t r,
                const int32_t x,
                const int32_t y,
                const int32_t d,
                double* sf);

#endif
