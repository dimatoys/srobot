#include <iostream>
#include <chrono>

#include <jpeglib.h>

#include "libobsensor/ObSensor.hpp"
#include "libobsensor/hpp/Error.hpp"

/*

Astra mini S, PID: 0x0407, SN/ID:
Device name: Astra mini S
Device pid: 1031 vid: 11205 uid: 1-1.1-3
Firmware version: RD2410
Serial number: AEAW223001A
ConnectionType: USB2.0
Sensor types:
        IR sensor
        Color sensor
        Depth sensor

*/
int test1() try {
    // Print the sdk version number, the sdk version number is divided into major version number, minor version number and revision number
    std::cout << "SDK version: " << ob::Version::getMajor() << "." << ob::Version::getMinor() << "." << ob::Version::getPatch() << std::endl;
    // Print sdk stage version
    std::cout << "SDK stage version: " << ob::Version::getStageVersion() << std::endl;

    // Create a Context.
    ob::Context ctx;

    // Query the list of connected devices
    auto devList = ctx.queryDeviceList();

    // Get the number of connected devices
    if(devList->deviceCount() == 0) {
        std::cerr << "Device not found!" << std::endl;
        return -1;
    }

    // Create a device, 0 means the index of the first device
    auto dev = devList->getDevice(0);

    // Get device information
    auto devInfo = dev->getDeviceInfo();

    // Get the name of the device
    std::cout << "Device name: " << devInfo->name() << std::endl;

    // Get the pid, vid, uid of the device
    std::cout << "Device pid: " << devInfo->pid() << " vid: " << devInfo->vid() << " uid: " << devInfo->uid() << std::endl;

    // By getting the firmware version number of the device
    auto fwVer = devInfo->firmwareVersion();
    std::cout << "Firmware version: " << fwVer << std::endl;

    // By getting the serial number of the device
    auto sn = devInfo->serialNumber();
    std::cout << "Serial number: " << sn << std::endl;

    // By getting the connection type of the device
    auto connectType = devInfo->connectionType();
    std::cout << "ConnectionType: " << connectType << std::endl;

    // Get the list of supported sensors
    std::cout << "Sensor types: " << std::endl;
    auto sensorList = dev->getSensorList();
    for(uint32_t i = 0; i < sensorList->count(); i++) {
        auto sensor = sensorList->getSensor(i);
        switch(sensor->type()) {
        case OB_SENSOR_COLOR:
            std::cout << "\tColor sensor" << std::endl;
            break;
        case OB_SENSOR_DEPTH:
            std::cout << "\tDepth sensor" << std::endl;
            break;
        case OB_SENSOR_IR:
            std::cout << "\tIR sensor" << std::endl;
            break;
        case OB_SENSOR_IR_LEFT:
            std::cout << "\tIR Left sensor" << std::endl;
            break;
        case OB_SENSOR_IR_RIGHT:
            std::cout << "\tIR Right sensor" << std::endl;
            break;
        case OB_SENSOR_GYRO:
            std::cout << "\tGyro sensor" << std::endl;
            break;
        case OB_SENSOR_ACCEL:
            std::cout << "\tAccel sensor" << std::endl;
            break;
        default:
            break;
        }
    }


    return 0;
}
catch(ob::Error &e) {
    std::cerr << "function:" << e.getName() << "\nargs:" << e.getArgs() << "\nmessage:" << e.getMessage() << "\ntype:" << e.getExceptionType() << std::endl;
    exit(EXIT_FAILURE);
}

void test2() try {
    // Create a pipeline with default device
    ob::Pipeline pipe;

    auto config = pipe.getConfig();
    for (uint32_t j = 0; j < config->getEnabledStreamProfileList()->count(); ++j) {
        auto profile = config->getEnabledStreamProfileList()->getProfile(j);
        auto type = profile->type();
        if (type == OB_STREAM_DEPTH) {
            auto depth = profile->as<ob::VideoStreamProfile>();
            //OB_FORMAT_Y11        = 11,   /**< Y11 format, 11-bit per pixel, single-channel (SDK will unpack into Y16 by default) */
            // 160 x 120
            std::cout << "Depth:" << depth->width() << " x " << depth->height() << " format=" << profile->format() << std::endl;
        } else {
            if (type == OB_STREAM_COLOR) {
                auto color = profile->as<ob::VideoStreamProfile>();
                // OB_FORMAT_UYVY       = 2,    /**< UYVY format */
                // 320 x 240
                std::cout << "Color:" << color->width() << " x " << color->height() << " format=" << profile->format() << std::endl;
            }
        }
    }
    
    auto lastTime = std::chrono::high_resolution_clock::now();
    int i = 10;

    ob::FrameSetCallback callback = [&lastTime, &i](std::shared_ptr<ob::FrameSet> frameSet) {
        auto now = std::chrono::high_resolution_clock::now();
        auto colorFrame = frameSet->colorFrame();
        auto depthFrame = frameSet->depthFrame();
        std::cout << std::chrono::duration_cast<std::chrono::nanoseconds>(now - lastTime).count() << (colorFrame ? " color" : " -") << (depthFrame ? " depth" : " -") << std::endl;
        lastTime = now;
    };
    
    // Start the pipeline with default config, more info please refer to the `misc/config/OrbbecSDKConfig_v1.0.xml`
    pipe.start(nullptr, callback);

    while(i > 0) {}

    // Stop the Pipeline, no frame data will be generated
    pipe.stop();
}
catch(ob::Error &e) {
    std::cerr << "function:" << e.getName() << "\nargs:" << e.getArgs() << "\nmessage:" << e.getMessage() << "\ntype:" << e.getExceptionType() << std::endl;
    exit(EXIT_FAILURE);
}

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

void test3() {

    const int width = 100;
    const int height = 100;

    uint8_t img[width * height * 3];

    int i = 0;
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            img[i++] = 255;
            img[i++] = 127;
            img[i++] = 127;
        }
    }

    write_jpeg_file("test3.jpg", img, width, height, 3);

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
    if (v <= 0) {
        return 0;
    }
    if (v > 255) {
        return 255;
    }
    return (uint8_t)v;
}

void saveColor(const char* fileName, const uint32_t width, const uint32_t height, const uint8_t* data) {
    uint8_t img[width * height * 3];
    int j = 0;
    int i = 0;
    for (uint32_t y = 0; y < height; ++y) {
        for (uint32_t x = 0; x < width; x+=2) {
            uint8_t U = data[j++];
            uint8_t Y0 = data[j++];
            uint8_t V = data[j++];
            uint8_t Y1 = data[j++];

            img[i++] = d2c(1.164 * Y0             + 1.596 * V);
            img[i++] = d2c(1.164 * Y0 - 0.392 * U - 0.813 * V);
            img[i++] = d2c(1.164 * Y0 + 2.017 * U);

            img[i++] = d2c(1.164 * Y1             + 1.596 * V);
            img[i++] = d2c(1.164 * Y1 - 0.392 * U - 0.813 * V);
            img[i++] = d2c(1.164 * Y1 + 2.017 * U);
        }
    }
    write_jpeg_file(fileName, img, width, height, 3);
}

void test4() try {
    // Create a pipeline with default device
    ob::Pipeline pipe;

    int i = 50;

    ob::FrameSetCallback callback = [&i](std::shared_ptr<ob::FrameSet> frameSet) {
        auto colorFrame = frameSet->colorFrame();
        auto depthFrame = frameSet->depthFrame();
        /*
        if (depthFrame) {
            if(--i == 1) {
                saveDepths("depths1.jpg", depthFrame->width(), depthFrame->height(), (uint16_t *)depthFrame->data());
                std::cout << "saved" << std::endl;
            }
        }
        */
        if (colorFrame) {
            if(--i == 1) {
                saveColor("color1.jpg", colorFrame->width(), colorFrame->height(), (uint8_t *)colorFrame->data());
                std::cout << "saved" << std::endl;
            }
        }

    };
    
    // Start the pipeline with default config, more info please refer to the `misc/config/OrbbecSDKConfig_v1.0.xml`
    pipe.start(nullptr, callback);

    while(i > 0) {}

    // Stop the Pipeline, no frame data will be generated
    pipe.stop();
}
catch(ob::Error &e) {
    std::cerr << "function:" << e.getName() << "\nargs:" << e.getArgs() << "\nmessage:" << e.getMessage() << "\ntype:" << e.getExceptionType() << std::endl;
    exit(EXIT_FAILURE);
}


int main(int argc, char *argv[]) {
	
    std::cout << "start" << std::endl;

	//test1();
    //test2();
    //test3();
    test4();

    std::cout << "exit" << std::endl;

}
