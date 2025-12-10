# include "image.h"

#include <iostream>
#include <iomanip>
#include <unistd.h>
#include <cmath>

using namespace std;

void test1() {
    const uint8_t q = 3;
    const auto msize = q * 2 + 1;

    float core[msize * msize];

    gaussCore(core, q);

    for (int y = 0; y < msize; ++y) {
        for (int x = 0; x < msize; ++ x) {
            cout << " " << core[x + y * msize];
        }
        cout  << endl;
    }
    
}

void loadDump(string filepath, float* dst, const uint32_t size) {
    uint16_t buffer[size];
    FILE *f = fopen(filepath.c_str(), "rb+");
    if (f) {
        fread(buffer, 2, size, f);
        fclose(f);

        for (uint32_t i = 0; i < size; ++i) {
            dst[i] = (float)buffer[i];
        }
    } else {
        cout << "cannot open file" << endl;
    }
}

void loadDump(string filepath, double* dst, const uint32_t size) {
    FILE *f = fopen(filepath.c_str(), "rb+");
    if (f) {
        fread(dst, sizeof(double), size, f);
        fclose(f);
    } else {
        cout << "cannot open file" << endl;
    }
}

void saveDump(string filepath, uint16_t* buffer, uint32_t size) {
    FILE *f = fopen(filepath.c_str(), "wb");
    if (f) {
        fwrite(buffer, 2, size, f);
        fclose(f);
    } else {
        cout << "cannot write file" << endl;
    }    
}

void test2() {
    const uint32_t width = 240;
    const uint32_t height = 180;

    const uint32_t size = width * height;

    float img[size];

    loadDump("analysis/depth_sobj100.dump", img, size);
    
    const uint8_t q = 5;

    float core[(q* 2 + 1) * (q* 2 + 1)];
    gaussCore(core, q);

    uint16_t dimg[size];
    applyGaussFilter(width, height, img, dimg, q, core);

    saveDump("test1.dump", dimg, size);
}

void test3() {
    const uint32_t width = 240;
    const uint32_t height = 180;

    const uint32_t size = width * height;

    float img[size];

    
    //loadDump("analysis/depth_bigbox30.dump", img, size);
    loadDump("analysis/depth_stay30.dump", img, size);

    const uint32_t dheight = 600;
    uint16_t dimg[width * dheight];
    generateFy(dheight);
    generateMap(width, height, img, dheight, dimg);

    saveDump("test1.dump", dimg, width * dheight);
}

void test4() {
    const uint32_t width = 240;
    const uint32_t height = 180;

    const uint32_t size = width * height;

    float img[size];

    const uint32_t parts = 10;
    double wall_dist[parts];


    //loadDump("analysis/v4/depth_wall40.dump", img, size);
    //loadDump("analysis/v4/depth_wall70.dump", img, size);
    //loadDump("analysis/v4/depth_wall70_move1.dump", img, size);
    //loadDump("analysis/v4/depth_wall70_move2.dump", img, size);
    //loadDump("analysis/v4/depth_wall50_move2.dump", img, size);
    //loadDump("analysis/v4/depth_wall50_move1.dump", img, size);
    
    const char* files[] = { 
                            "analysis/v4/depth_wall20_move1.dump",
                            "analysis/v4/depth_wall20_move2.dump",
                            "analysis/v4/depth_wall30_60.dump",
                            "analysis/v4/depth_wall30_move1.dump",
                            "analysis/v4/depth_wall30_move2.dump",
                            "analysis/v4/depth_wall40.dump",
                            "analysis/v4/depth_wall40_30.dump",
                            "analysis/v4/depth_wall40_45.dump",
                            "analysis/v4/depth_wall40_60.dump",
                            "analysis/v4/depth_wall40_move1.dump",
                            "analysis/v4/depth_wall40_move2.dump",
                            "analysis/v4/depth_wall50_60.dump",
                            "analysis/v4/depth_wall50_move1.dump",
                            "analysis/v4/depth_wall50_move2.dump",
                            "analysis/v4/depth_wall70.dump",
                            "analysis/v4/depth_wall70_move1.dump",
                            "analysis/v4/depth_wall70_move2.dump",
                            "analysis/v3/depth_side30.dump"};

    for (size_t i = 0; i < sizeof(files) / sizeof(files[0]); ++i) {
        loadDump(files[i], img, size);
        cout << files[i] << "\t";
        extract_walls(width, height, img, parts, wall_dist);
        for (uint32_t i = 0; i < parts; ++i) {
            cout << std::fixed << std::setprecision(2) << std::setw(5) << wall_dist[i] << ", ";
        }
        cout << endl;
    }
}

void test5() {

    const uint32_t width = 240;
    const uint32_t height = 180;

    const uint32_t size = width * height;

    double img[size];

    loadDump("analysis/v7/depth_box500ff.dump", img, size);

    uint32_t x = 120;
    uint32_t y = 90;

    const uint32_t d = 3;
    const uint32_t r = 4;

    double sf[d];

    auto sdiff = count_sf(img, width, height, r, x, y, d, sf);

    cout << "sdiff=[" << sf[0] << ", " << sf[1] << ", " << sf[2] << "]" << endl;

    cout << "sdiff=" << sdiff << endl;
}

int main(int argc, char *argv[]) {
	
	//test1();
    //test2();
    //test3();
    //test4();
    test5();
}
