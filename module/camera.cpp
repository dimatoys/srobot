#include <iostream>
#include <fstream>

#include <jpeglib.h>

#include "image.h"
#include "camera.h"

using namespace std;

#define CAMERA_ARDUCAM_TOF 1

//#define CAMERA_ASTRA_MINI

int write_jpeg_file(const char *filename,
                    void *raw_image,
                    int width,
                    int height,
                    int bytes_per_pixel)
{
	J_COLOR_SPACE color_space = JCS_RGB;

	struct jpeg_compress_struct cinfo;
	struct jpeg_error_mgr jerr;
	
	/* this is a pointer to one row of image data */
	JSAMPROW row_pointer[1];
        //printf("jpeg:%s\n", filename);
	FILE *outfile = fopen( filename, "wb" );
	
	if ( !outfile )
	{
		printf("Error opening output jpeg file %s\n!", filename );
		return -1;
	}
	cinfo.err = jpeg_std_error( &jerr );
	jpeg_create_compress(&cinfo);
	jpeg_stdio_dest(&cinfo, outfile);

	/* Setting the parameters of the output file here */
	cinfo.image_width = width;	
	cinfo.image_height = height;
	cinfo.input_components = bytes_per_pixel;
	cinfo.in_color_space = color_space;
    /* default compression parameters, we shouldn't be worried about these */
	jpeg_set_defaults( &cinfo );
	/* Now do the compression .. */
	jpeg_set_quality(&cinfo, 100, FALSE);
	jpeg_start_compress( &cinfo, TRUE );
	/* like reading a file, this time write one row at a time */
	while( cinfo.next_scanline < cinfo.image_height )
	{
		row_pointer[0] = &((unsigned char*)raw_image)[ cinfo.next_scanline * cinfo.image_width *  cinfo.input_components];
		jpeg_write_scanlines( &cinfo, row_pointer, 1 );
	}
	/* similar to read file, clean up after we're done compressing */
	jpeg_finish_compress( &cinfo );
	jpeg_destroy_compress( &cinfo );
	fclose( outfile );
	/* success code is 1! */
	return 1;
}

int write_jpeg_file_flip(const char *filename,
                    void *raw_image,
                    int width,
                    int height,
                    int bytes_per_pixel)
{
	J_COLOR_SPACE color_space = JCS_RGB;

	struct jpeg_compress_struct cinfo;
	struct jpeg_error_mgr jerr;
	
	/* this is a pointer to one row of image data */
	JSAMPROW row_pointer[1];
        //printf("jpeg:%s\n", filename);
	FILE *outfile = fopen( filename, "wb" );
	
	if ( !outfile )
	{
		printf("Error opening output jpeg file %s\n!", filename );
		return -1;
	}
	cinfo.err = jpeg_std_error( &jerr );
	jpeg_create_compress(&cinfo);
	jpeg_stdio_dest(&cinfo, outfile);

	/* Setting the parameters of the output file here */
	cinfo.image_width = width;	
	cinfo.image_height = height;
	cinfo.input_components = bytes_per_pixel;
	cinfo.in_color_space = color_space;
    /* default compression parameters, we shouldn't be worried about these */
	jpeg_set_defaults( &cinfo );
	/* Now do the compression .. */
	jpeg_set_quality(&cinfo, 100, FALSE);
	jpeg_start_compress( &cinfo, TRUE );
	/* like reading a file, this time write one row at a time */
	while( cinfo.next_scanline < cinfo.image_height )
	{
		row_pointer[0] = &((unsigned char*)raw_image)[ (height - cinfo.next_scanline - 1) * cinfo.image_width *  cinfo.input_components];
		jpeg_write_scanlines( &cinfo, row_pointer, 1 );
	}
	/* similar to read file, clean up after we're done compressing */
	jpeg_finish_compress( &cinfo );
	jpeg_destroy_compress( &cinfo );
	fclose( outfile );
	/* success code is 1! */
	return 1;
}

void saveDump(const char* fileName, int size, const void* data) {
    ofstream file (fileName);
    file.write ((const char*)data, size );
    file.close();
}

void saveDepths(const char* fileName, const uint32_t width, const uint32_t height, const uint16_t* data) {
    uint8_t img[width * height * 3];

    uint32_t size = width * height;

    uint16_t min = 65535;
    uint16_t max = 0;

    for (uint32_t i = 0; i < size; ++i) {
        auto v = data[i];
        if (v != 0) {
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
            if (v == 0) {
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
    write_jpeg_file(fileName, img, width, height, 3);
}

uint8_t d2c(double v) {
    if (v < -1) {
        return 0;
    }
    if (v > 1) {
        return 255;
    }
    return (uint8_t)(v * 127 + 127);
}

void saveColorUYVY(const char* fileName, const uint32_t width, const uint32_t height, const uint8_t* data) {
    uint8_t img[width * height * 3];
    int j = 0;
    int i = 0;
    for (uint32_t y = 0; y < height; ++y) {
        //cout << "y=" << y << endl;
        for (uint32_t x = 0; x < width; x+=2) {
            double U = ((double)data[j++] -127) / 127.0;
            double Y0 = ((double)data[j++] -127) / 127.0;
            double V = ((double)data[j++] -127) / 127.0;
            double Y1 = ((double)data[j++] -127) / 127.0;

            img[i++] = d2c(1.164 * Y0             + 1.596 * V);
            img[i++] = d2c(1.164 * Y0 - 0.392 * U - 0.813 * V);
            img[i++] = d2c(1.164 * Y0 + 2.017 * U);

            img[i++] = d2c(1.164 * Y1             + 1.596 * V);
            img[i++] = d2c(1.164 * Y1 - 0.392 * U - 0.813 * V);
            img[i++] = d2c(1.164 * Y1 + 2.017 * U);

            //cout << x << " (" << U << ", " << Y0 << ", " << V << ", " << Y1 << ") -> [" << (uint32_t)img[i-6] << ", "
            //     << (uint32_t)img[i-5] << ", " << (uint32_t)img[i-4] << "] [" << (uint32_t)img[i-3] << ", " << (uint32_t)img[i-2] << ", " << (uint32_t)img[i-1] << "]" << endl;

        }
    }
    write_jpeg_file(fileName, img, width, height, 3);
}

void saveColorRGB(const char* fileName, const uint32_t width, const uint32_t height, uint8_t* data) {
    write_jpeg_file(fileName, data, width, height, 3);
}

void saveBW(string fileName, const uint32_t width, const uint32_t height, float* data) {
    uint8_t img[width * height * 3];

    BWtoRGB(width, height, data, img);
    write_jpeg_file_flip(fileName.c_str(), img, width, height, 3);

}

void saveDump(string dumpFile, uint32_t width, uint32_t height, const float* data) {
    auto size = width * height;
    uint16_t buffer[size];
    flip(width, height, data, buffer);
    saveDump(dumpFile.c_str(), size * 2, buffer);

}

TCamera* TCamera::Camera = NULL;

#ifdef CAMERA_ASTRA_MINI

#include "libobsensor/ObSensor.hpp"
#include "libobsensor/hpp/Error.hpp"

struct TAstraMiniCamera : public TCamera {

    ob::Pipeline Pipe;

    bool MakeColor;
    bool MakeDepth;

    std::string DepthFile;
    std::string DepthDumpFile;
    std::string ColorFile;
    std::string ColorDumpFile;
    
    TAstraMiniCamera() {
        MakeColor = false;
        MakeDepth = false;
        start();
    }

    virtual ~TAstraMiniCamera() {
        stop();
    }

    void start();
    void stop();
    void makePicture(std::string depthFile, std::string colorFile);
};

void TAstraMiniCamera::start() {
    auto color_profiles = Pipe.getStreamProfileList(OB_SENSOR_COLOR);
    auto depth_profiles = Pipe.getStreamProfileList(OB_SENSOR_DEPTH);

    std::shared_ptr<ob::Config> config = std::make_shared<ob::Config>();
    auto depth_profile = depth_profiles->getProfile(0)
    config->enableStream(depth_profile);
    config->enableStream(color_profiles->getProfile(3));

    auto vdepth = depth_profile->as<ob::VideoStreamProfile>();
    Width = vdepth->width();
    Height = vdepth->height();
    MaxRange = 1000;

    ob::FrameSetCallback callback = [this](std::shared_ptr<ob::FrameSet> frameSet) {
        if (MakeDepth) {
            auto depthFrame = frameSet->depthFrame();
            if (depthFrame) {
                MakeDepth = false;
                saveDepths(DepthFile.c_str(), depthFrame->width(), depthFrame->height(), (uint16_t *)depthFrame->data());
                saveDump(DepthDumpFile.c_str(), depthFrame->width()* depthFrame->height() * 2, depthFrame->data());
            }
        }
        if (MakeColor) {
            auto colorFrame = frameSet->colorFrame();
            if (colorFrame) {
                MakeColor = false;
                saveColorRGB(ColorFile.c_str(), colorFrame->width(), colorFrame->height(), (uint8_t *)colorFrame->data());
                saveDump(ColorDumpFile.c_str(), colorFrame->width()* colorFrame->height() * 3, colorFrame->data());
            }
        }
    };

    Pipe.start(config, callback);
}

void TAstraMiniCamera::stop() {
    Pipe.stop();
}

void TAstraMiniCamera::makePicture(std::string depthFile, std::string colorFile) {

    DepthFile = depthFile;
    ColorFile = colorFile;
    DepthDumpFile = DepthFile + std::string(".dump");
    ColorDumpFile = ColorFile + std::string(".dump");
    MakeDepth = true;
    MakeColor = true;
}


TCamera* TCamera::getCamera() {
    if (Camera == NULL) {
        Camera = new TAstraMiniCamera();
    }
    return Camera;
}

#endif

#ifdef CAMERA_ARDUCAM_TOF

#include "ArducamTOFCamera.hpp"

using namespace Arducam;

struct TArducamTOFCamera : public TCamera {

    ArducamTOFCamera Tof;
    const int MODE = 2000;
    
    TArducamTOFCamera() {
        MaxRange = 0;
        start();
    }

    virtual ~TArducamTOFCamera() {
        stop();
    }

    void start();
    void stop();
    void makePicture(std::string depthFile, std::string colorFile);
};

void TArducamTOFCamera::start() {
    if (Tof.open(Connection::CSI, 0)) {
        std::cerr << "Failed to open camera" << std::endl;
        return;
    }

    if (Tof.start(FrameType::DEPTH_FRAME)) {
        std::cerr << "Failed to start camera" << std::endl;
        return;
    }
    //  Modify the range also to modify the MAX_DISTANCE
    Tof.setControl(Control::RANGE, MODE);
    Tof.getControl(Control::RANGE, &MaxRange);
    auto info = Tof.getCameraInfo();
    Width = info.width;
    Height = info.height;
    std::cout << "open camera with (" << Width << "x" << Height << ") max_range=" << MaxRange
              << " bits=" << info.bit_width << " bpp=" << info.bpp << std::endl;
}

void TArducamTOFCamera::stop() {
    if (Tof.stop()) {
        cerr << "Error stop camera" << endl;
        return;
    }

    if (Tof.close()) {
        cerr << "Error close camera" << endl;
    }
    
}

void TArducamTOFCamera::makePicture(std::string depthFile, std::string colorFile) {
    ArducamFrameBuffer* frame = Tof.requestFrame(200);
    if (frame == nullptr) {
        cerr << "no frame" << endl;
        return;
    }

    float* depth_ptr = (float*)frame->getData(FrameType::DEPTH_FRAME);
    float* confidence_ptr = (float*)frame->getData(FrameType::CONFIDENCE_FRAME);

    if (depth_ptr != nullptr) {
        string depthDumpFile = depthFile + std::string(".dump");
        saveBW(depthFile, Width, Height, depth_ptr);
        saveDump(depthDumpFile, Width, Height, depth_ptr);
    }
    if (confidence_ptr != nullptr) {
        string colorDumpFile = colorFile + std::string(".dump");
        saveBW(colorFile, Width, Height, confidence_ptr);
        saveDump(colorDumpFile, Width, Height, confidence_ptr);
    }
    Tof.releaseFrame(frame);
}

TCamera* TCamera::getCamera() {
    if (Camera == NULL) {
        Camera = new TArducamTOFCamera();
    }
    return Camera;
}

#endif