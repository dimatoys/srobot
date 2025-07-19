#include <iostream>
#include <chrono>

#include <jpeglib.h>

#include "libobsensor/ObSensor.hpp"
#include "libobsensor/hpp/Error.hpp"

#include "camera.h"

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

/*
IR profiles:3
0 IR:320 x 240 format=10         < Y10 format, 10-bit per pixel, single-channel(SDK will unpack into Y16 by default)
1 IR:640 x 480 format=10
2 IR:1280 x 1024 format=10
Color profiles:6
0 Color:320 x 240 format=2        < UYVY format
1 Color:640 x 480 format=2
2 Color:1280 x 960 format=2
3 Color:320 x 240 format=22       < RGB format (actual RGB888)
4 Color:640 x 480 format=22
5 Color:1280 x 960 format=22
Depth profiles:4
0 Depth:160 x 120 format=11      < Y11 format, 11-bit per pixel, single-channel (SDK will unpack into Y16 by default)
1 Depth:320 x 240 format=11
2 Depth:640 x 480 format=11
3 Depth:1280 x 1024 format=11
*/
    auto ir_profiles = pipe.getStreamProfileList(OB_SENSOR_IR);
    auto cnt = ir_profiles->count();
    std::cout << "IR profiles:" << cnt << std::endl;
    for (uint32_t j = 0 ; j < cnt; ++j) {
        auto profile = ir_profiles->getProfile(j);
        auto ir = profile->as<ob::VideoStreamProfile>();
        std::cout << "IR:" << ir->width() << " x " << ir->height() << " format=" << ir->format() << std::endl;
    }

    auto color_profiles = pipe.getStreamProfileList(OB_SENSOR_COLOR);
    cnt = color_profiles->count();
    std::cout << "Color profiles:" << cnt << std::endl;
    for (uint32_t j = 0 ; j < cnt; ++j) {
        auto profile = color_profiles->getProfile(j);
        auto ir = profile->as<ob::VideoStreamProfile>();
        std::cout << "Color:" << ir->width() << " x " << ir->height() << " format=" << ir->format() << std::endl;
    }

    auto depth_profiles = pipe.getStreamProfileList(OB_SENSOR_DEPTH);
    cnt = depth_profiles->count();
    std::cout << "Depth profiles:" << cnt << std::endl;
    for (uint32_t j = 0 ; j < cnt; ++j) {
        auto profile = depth_profiles->getProfile(j);
        auto ir = profile->as<ob::VideoStreamProfile>();
        std::cout << "Depth:" << ir->width() << " x " << ir->height() << " format=" << ir->format() << std::endl;
    }

    std::shared_ptr<ob::Config> config2 = std::make_shared<ob::Config>();
    //config2->enableStream(ir_profiles->getProfile(0));
    config2->enableStream(depth_profiles->getProfile(3));
    config2->enableStream(color_profiles->getProfile(5));
/*
    auto config = pipe.getConfig();
    for (uint32_t j = 0; j < config->getEnabledStreamProfileList()->count(); ++j) {
        auto profile = config->getEnabledStreamProfileList()->getProfile(j);
        auto type = profile->type();
        if (type == OB_STREAM_DEPTH) {
            auto depth = profile->as<ob::VideoStreamProfile>();
            //OB_FORMAT_Y11        = 11,   < Y11 format, 11-bit per pixel, single-channel (SDK will unpack into Y16 by default)
            // 160 x 120
            std::cout << "Depth:" << depth->width() << " x " << depth->height() << " format=" << profile->format() << std::endl;
            config2->enableStream(depth);
        } else {
            if (type == OB_STREAM_COLOR) {
                auto color = profile->as<ob::VideoStreamProfile>();
                // OB_FORMAT_UYVY       = 2,    < UYVY format
                // 320 x 240
                std::cout << "Color:" << color->width() << " x " << color->height() << " format=" << profile->format() << std::endl;
                config2->enableStream(color);
            } else {
                if (type == OB_STREAM_IR) {
                    auto ir = profile->as<ob::VideoStreamProfile>();
                    std::cout << "IR:" << ir->width() << " x " << ir->height() << " format=" << ir->format() << std::endl;
                }
            }
        }
    }
*/    
    auto lastTime = std::chrono::high_resolution_clock::now();
    int i = 10;

    ob::FrameSetCallback callback = [&lastTime, &i](std::shared_ptr<ob::FrameSet> frameSet) {
        auto now = std::chrono::high_resolution_clock::now();
        auto colorFrame = frameSet->colorFrame();
        auto depthFrame = frameSet->depthFrame();
        auto irFrame = frameSet->irFrame();
        std::cout << std::chrono::duration_cast<std::chrono::nanoseconds>(now - lastTime).count() << (colorFrame ? " color" : " -") << (depthFrame ? " depth" : " -") << (irFrame ? " ir" : " -") << std::endl;
        lastTime = now;
        --i;
    };
    
    // Start the pipeline with default config, more info please refer to the `misc/config/OrbbecSDKConfig_v1.0.xml`
    pipe.start(config2, callback);

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
    test2();

    std::cout << "exit" << std::endl;

}
