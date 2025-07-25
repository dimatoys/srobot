# include "image.h"

#include <iostream>
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

int main(int argc, char *argv[]) {
	
	//test1();
    test2();
}
