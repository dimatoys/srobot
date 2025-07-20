extern "C" {

#include <cstdint>

struct TModuleObject {
    void* Skeleton;
    void* Move;
    void* Camera;
    uint32_t CameraWidth;
    uint32_t CameraHeight;
    int32_t CameraMaxRange;
};

int init(TModuleObject* module);
int run_cmd(TModuleObject* module, const char* cmd, const char* arg);

}