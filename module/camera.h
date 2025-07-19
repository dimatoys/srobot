#ifndef _CAMERA_H
#define _CAMERA_H

#include <string>

int write_jpeg_file(const char *filename,
                    void *raw_image,
                    int width,
                    int height,
                    int bytes_per_pixel);

void makePictures(std::string depthFile, std::string colorFile);
void makePictures2(std::string depthFile, std::string colorFile, std::string irFile);

#endif