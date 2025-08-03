#include <cstdint>
#include "camera.h"

extern "C" {

struct TModuleObject {
    void* Skeleton;
    void* Move;
    void* Camera;
    uint32_t CameraWidth;
    uint32_t CameraHeight;
    int32_t CameraMaxRange;
    uint32_t Parts;
    double WallDist[TCamera::PARTS];
};

int init(TModuleObject* module);
int run_cmd(TModuleObject* module, const char* cmd, const char* arg);

}