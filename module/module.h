extern "C" {

struct TModuleObject {
    void* Skeleton;
    void* Move;
};

int init(TModuleObject* module);
int run_cmd(TModuleObject* module, const char* cmd, const char* arg);

}