#include <px4_platform_common/log.h>

extern "C" __EXPORT int hello_px4_main(int argc, char *argv[])
{
    PX4_INFO("Hello PX4");

    return 0;
}