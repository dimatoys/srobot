extern "C" {

struct TModuleObject {
    void* Robot;
};

int init(TModuleObject* module);
void shutdown(TModuleObject* module);
int run_cmd(TModuleObject* module, char* cmd, char* arg);

}