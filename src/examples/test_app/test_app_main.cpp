#include <px4_platform_common/log.h>

extern "C" __EXPORT int test_app_main(int argc, char *argv[]);

int test_app_main(int argc, char *argv[])
{
    PX4_INFO("Hello, PX4!");
    return 0;
}
