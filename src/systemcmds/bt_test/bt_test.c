#include <px4_platform_common/log.h>

#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <stdio.h>

int bt_test_main(int argc, char *argv[])
{
    const char *device = "/dev/ttyS1";
    const char *cmd = "AT\r\n";

    char buffer[64] = {0};

    /* 打开 USART2 对应的设备节点
     * O_RDWR     : 可读可写
     * O_NONBLOCK : 非阻塞，没数据时 read() 直接返回
     */
    int fd = open(device, O_RDWR | O_NONBLOCK);

    if (fd < 0) {
        PX4_ERR("open %s failed", device);
        return -1;
    }

    PX4_INFO("opened %s", device);

    /* 发送 AT 命令 */
    int written = write(fd, cmd, strlen(cmd));

    if (written < 0) {
        PX4_ERR("write failed");
        close(fd);
        return -1;
    }

    PX4_INFO("TX %d bytes: AT\\r\\n", written);

    bool received = false;

    /* 每 10 ms 检查一次
     * 最多检查 300 次
     * 总监听时间约 3 秒
     */
    for (int i = 0; i < 300; i++) {

        int n = read(fd, buffer, sizeof(buffer) - 1);

        if (n > 0) {

            received = true;

            /* 末尾补 '\0'，方便按照字符串打印 */
            buffer[n] = '\0';

            PX4_INFO("RX %d bytes", n);

            /* ASCII形式打印 */
            PX4_INFO("RX ASCII: %s", buffer);

            /* HEX形式打印 */
            printf("RX HEX: ");

            for (int j = 0; j < n; j++) {
                printf("%02X ", (unsigned char)buffer[j]);
            }

            printf("\n");

            break;
        }

        /* 等待 10 ms */
        usleep(10000);
    }

    if (!received) {
        PX4_WARN("no response within 3 seconds");
    }

    close(fd);

    PX4_INFO("test finished");

    return 0;
}
