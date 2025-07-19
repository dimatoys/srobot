#include <iostream>
#include <fstream>

#include "libobsensor/ObSensor.hpp"
#include "libobsensor/hpp/Error.hpp"
#include <jpeglib.h>

#include "camera.h"

using namespace std;

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

void saveDump(const char* fileName, int size, const void* data) {
    ofstream file (fileName);
    file.write ((const char*)data, size );
    file.close();
}

void saveDepths(const char* fileName, const uint32_t width, const uint32_t height, const uint16_t* data) {
    uint8_t img[width * height * 3];
    int j = 0;
    int i = 0;
    for (uint32_t y = 0; y < height; ++y) {
        for (uint32_t x = 0; x < width; ++x) {
            uint8_t v = (uint8_t)(data[j++] / 8);
            img[i++] = v;
            img[i++] = v;
            img[i++] = v;
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

void saveColor(const char* fileName, const uint32_t width, const uint32_t height, const uint8_t* data) {
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



void makePictures(string depthFile, string colorFile) {
    // Create a pipeline with default device
    ob::Pipeline pipe;

    bool color = false;
    bool depth = false;

    ob::FrameSetCallback callback = [&color, &depth, depthFile, colorFile](std::shared_ptr<ob::FrameSet> frameSet) {
        auto colorFrame = frameSet->colorFrame();
        auto depthFrame = frameSet->depthFrame();

        if (depthFrame) {
            saveDepths(depthFile.c_str(), depthFrame->width(), depthFrame->height(), (uint16_t *)depthFrame->data());
            char name[100];
            sprintf(name, "%s.dump", depthFile.c_str());
            saveDump(name, depthFrame->width()* depthFrame->height() * 2, depthFrame->data());
            depth = true;
        }

        if (colorFrame) {
            saveColor(colorFile.c_str(), colorFrame->width(), colorFrame->height(), (uint8_t *)colorFrame->data());
            char name[100];
            sprintf(name, "%s.dump", colorFile.c_str());
            saveDump(name, colorFrame->width()* colorFrame->height() * 2, colorFrame->data());
            color = true;
        }
    };
    
    // Start the pipeline with default config, more info please refer to the `misc/config/OrbbecSDKConfig_v1.0.xml`
    pipe.start(nullptr, callback);

    while(!(depth&color)) {}

    // Stop the Pipeline, no frame data will be generated
    pipe.stop();
}

void makePictures2(string depthFile, string colorFile, string irFile) {
    // Create a pipeline with default device
    ob::Pipeline pipe;

    auto ir_profiles = pipe.getStreamProfileList(OB_SENSOR_IR);
    auto color_profiles = pipe.getStreamProfileList(OB_SENSOR_COLOR);
    auto depth_profiles = pipe.getStreamProfileList(OB_SENSOR_DEPTH);

    std::shared_ptr<ob::Config> config = std::make_shared<ob::Config>();
    //config->enableStream(ir_profiles->getProfile(0));
    config->enableStream(depth_profiles->getProfile(0));
    config->enableStream(color_profiles->getProfile(3));

    bool color = false;
    bool depth = false;
    bool ir = false;

    ob::FrameSetCallback callback = [&color, &depth, &ir, depthFile, colorFile, irFile](std::shared_ptr<ob::FrameSet> frameSet) {
        auto colorFrame = frameSet->colorFrame();
        auto depthFrame = frameSet->depthFrame();
        auto irFrame = frameSet->irFrame();

        if (depthFrame) {
            saveDepths(depthFile.c_str(), depthFrame->width(), depthFrame->height(), (uint16_t *)depthFrame->data());
            char name[100];
            sprintf(name, "%s.dump", depthFile.c_str());
            saveDump(name, depthFrame->width()* depthFrame->height() * 2, depthFrame->data());
            depth = true;
        }

        if (irFrame) {
            saveDepths(irFile.c_str(), irFrame->width(), irFrame->height(), (uint16_t *)irFrame->data());
            char name[100];
            sprintf(name, "%s.dump", irFile.c_str());
            saveDump(name, irFrame->width()* irFrame->height() * 2, irFrame->data());
            ir = true;
        }

        if (colorFrame) {
            saveColorRGB(colorFile.c_str(), colorFrame->width(), colorFrame->height(), (uint8_t *)colorFrame->data());
            char name[100];
            sprintf(name, "%s.dump", colorFile.c_str());
            saveDump(name, colorFrame->width()* colorFrame->height() * 2, colorFrame->data());
            color = true;
        }
    };
    
    // Start the pipeline with default config, more info please refer to the `misc/config/OrbbecSDKConfig_v1.0.xml`
    pipe.start(config, callback);

    while(!(depth&color)) {}

    // Stop the Pipeline, no frame data will be generated
    pipe.stop();
}
