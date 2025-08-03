#ifndef _CAMERA_H
#define _CAMERA_H

#include <string>

struct TCamera {
    static const uint32_t PARTS = 10;
    double WallDist[PARTS];

    uint32_t Width;
    uint32_t Height;
    int32_t MaxRange;

    static TCamera* Camera;
    static TCamera* getCamera();

    virtual ~TCamera(){}

    virtual void makePicture(std::string depthFile, std::string colorFile)=0;
    virtual int GetWall()=0;
};


int write_jpeg_file(const char *filename,
                    void *raw_image,
                    int width,
                    int height,
                    int bytes_per_pixel);

#endif