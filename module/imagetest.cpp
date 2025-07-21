# include "image.h"

#include <iostream>
#include <unistd.h>
#include <cmath>

using namespace std;


// float core[(q+1)*(q+1)]
void gaussCore(float* core, uint16_t q) {
    auto msize = q  +1;
    for (uint16_t i = 0; i <= q; ++i) {
        for (uint16_t j = i; j <= q; ++j) {
            float v = exp(-(i*i + j*j) / (float)q);
            core[i + j * msize] = v;
            core[j + i * msize] = v;
        }
    }
}

void test1() {
    const uint8_t q = 3;
    const auto msize = q + 1;

    float core[msize * msize];

    gaussCore(core, q);

    for (int y = 0; y < msize; ++y) {
        for (int x = 0; x < msize; ++ x) {
            cout << " " << core[x + y * msize];
        }
        cout  << endl;
    }
    
}

void loadDump(string filepath, uint16_t* buffer, uint32_t size) {
    FILE *f = fopen(filepath.c_str(), "rb+");
    if (f) {
        fread(buffer, 1, size, f);
        fclose(f);
    } else {
        cout << "cannot open file" << endl;
    }
}

void saveDump(string filepath, uint16_t* buffer, uint32_t size) {
    FILE *f = fopen(filepath.c_str(), "wb");
    if (f) {
        fwrite(buffer, 1, size, f);
        fclose(f);
    } else {
        cout << "cannot write file" << endl;
    }    
}

void test2() {
    const uint32_t width = 240;
    const uint32_t height = 180;

    const uint32_t size = width * height;

    uint16_t img[size];

    loadDump("analysis/depth_sobj100.dump", img, size);
}

int main(int argc, char *argv[]) {
	
	//test1();
    test2();
}
