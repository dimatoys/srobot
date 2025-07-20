#include "ArducamTOFCamera.hpp"

#include <iostream>
#include <chrono>

#include <jpeglib.h>

using namespace Arducam;
using namespace std;

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
		row_pointer[0] = &((unsigned char*)raw_image)[ (height - cinfo.next_scanline) * cinfo.image_width *  cinfo.input_components];
		jpeg_write_scanlines( &cinfo, row_pointer, 1 );
	}
	/* similar to read file, clean up after we're done compressing */
	jpeg_finish_compress( &cinfo );
	jpeg_destroy_compress( &cinfo );
	fclose( outfile );
	/* success code is 1! */
	return 1;
}


void test1() {
    ArducamTOFCamera tof;
    if (tof.open(Connection::CSI, 0)) {
        std::cerr << "Failed to open camera" << std::endl;
        return;
    }

    //if (tof.start(FrameType::RAW_FRAME)) {
    if (tof.start(FrameType::DEPTH_FRAME)) {
        std::cerr << "Failed to start camera" << std::endl;
        return;
    }
    //  Modify the range also to modify the MAX_DISTANCE
    int max_range = 0;
    tof.setControl(Control::RANGE,2000);
    tof.getControl(Control::RANGE, &max_range);
    auto info = tof.getCameraInfo();
    std::cout << "open camera with (" << info.width << "x" << info.height << ") max_range=" << max_range << std::endl;

    int cnt = 20;
    for (;;) {
        Arducam::FrameFormat format;
        ArducamFrameBuffer* frame = tof.requestFrame(200);
        if (frame == nullptr) {
            continue;
        }

        // frame: (240x180)
        frame->getFormat(FrameType::DEPTH_FRAME, format);

        // frame: (240x720)
        // frame->getFormat(FrameType::RAW_FRAME, format);
        auto max_height = format.height;
        auto max_width = format.width;
        float* depth_ptr = (float*)frame->getData(FrameType::DEPTH_FRAME);
        float* confidence_ptr = (float*)frame->getData(FrameType::CONFIDENCE_FRAME);
        int16_t* raw_ptr = (int16_t*)frame->getData(FrameType::RAW_FRAME);

        int i = 0;
        float dmin = 10000000000;
        float dmax = -10000000000; 
        float cmin = 10000000000;
        float cmax = -10000000000; 
        uint16_t rmin = 65535;
        uint16_t rmax = 0; 

        for (uint32_t y = 0; y < max_height;++y) {
            for (uint32_t x = 0; x < max_width; ++x) {
                if (depth_ptr != nullptr) {
                    auto d = depth_ptr[i];
                    if (d < dmin) { dmin = d; }
                    if (d > dmax) { dmax = d; }
                }
                if (confidence_ptr != nullptr) {
                    auto c = confidence_ptr[i];
                    if (c < cmin) { cmin = c; }
                    if (c > cmax) { cmax = c; }
                }
                if (raw_ptr != nullptr) {
                    auto r = raw_ptr[i];
                    if (r < rmin) { rmin = r; }
                    if (r > rmax) { rmax = r; }
                }
                ++i;
            }
        }


        i = 0;
        int j = 0;
        uint8_t* depths = new uint8_t[max_width*max_height*3];
        uint8_t* conf = new uint8_t[max_width*max_height*3];
        uint8_t* raw = new uint8_t[max_width*max_height*3];
        for (uint32_t y = 0; y < max_height;++y) {
            for (uint32_t x = 0; x < max_width; ++x) {
                if (dmin < dmax) {
                    auto d = depth_ptr[i];
                    if (d > 0) {
                        uint8_t v = (uint8_t)((d - dmin) * 255 / (dmax - dmin));
                        depths[j] = v;
                        depths[j+1] = v;
                        depths[j+2] = v;
                    } else {
                        depths[j] = 255;
                        depths[j+1] = 128;
                        depths[j+2] = 128;
                    }
                }
                if (cmin < cmax) {
                    auto c = confidence_ptr[i];
                    uint8_t v = (uint8_t)((c - cmin) * 255 / (cmax - cmin));
                    conf[j] = v;
                    conf[j+1] = v;
                    conf[j+2] = v;
                }
                if (rmin < rmax) {
                    auto r = raw_ptr[i];
                    if (r > 0) {
                        uint8_t v = (uint8_t)((r - rmin) * 255 / (rmax - rmin));
                        raw[j] = v;
                        raw[j+1] = v;
                        raw[j+2] = v;
                    } else {
                        raw[j] = 255;
                        raw[j+1] = 128;
                        raw[j+2] = 128;
                    }
                }
                ++i;
                j += 3;
            }
        }

        if (dmin < dmax) {
            write_jpeg_file_flip("adepth.jpg",
                    depths,
                    max_width,
                    max_height,
                    3);
        }

        if (cmin < cmax) {
            write_jpeg_file_flip("aconf.jpg",
                    conf,
                    max_width,
                    max_height,
                    3);
        }

        if (rmin < rmax) {
            write_jpeg_file_flip("araw.jpg",
                    raw,
                    max_width,
                    max_height,
                    3);
        }

        delete depths;
        delete conf;
        delete raw;


        std::cout << "frame: (" << max_width << "x" << max_height
                     << ") depth=" << depth_ptr << " (" << dmin << ", " << dmax
                     << ") conf=" << confidence_ptr << " (" << cmin << ", " << cmax
                     << ") row=" << raw_ptr << " (" << rmin << ", " << rmax
                     << ")" << std::endl;

        tof.releaseFrame(frame);

        if (--cnt <= 0) {
            break;
        }

    }

    if (tof.stop()) {
        cerr << "Error stop" << endl;
        return;
    }

    if (tof.close()) {
        cerr << "Error close" << endl;
    }

}

int main(int argc, char *argv[]) {
	
    std::cout << "start" << std::endl;

	test1();

    std::cout << "exit" << std::endl;

}
