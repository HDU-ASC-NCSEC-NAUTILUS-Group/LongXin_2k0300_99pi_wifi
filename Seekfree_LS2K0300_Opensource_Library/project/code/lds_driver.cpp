#include "lds_driver.h"
#include <cstring>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <cerrno>
#include <cmath>

LDSDriver::LDSDriver() : fd_(-1) {}

LDSDriver::~LDSDriver() { end(); }

bool LDSDriver::begin(const char* port, int baudrate) {
    fd_ = open(port, O_RDWR | O_NOCTTY);
    if (fd_ < 0) return false;

    struct termios options;
    tcgetattr(fd_, &options);
    
    speed_t baud;
    switch (baudrate) {
        case 230400: baud = B230400; break;
        case 115200: baud = B115200; break;
        default: baud = B230400;
    }
    cfsetispeed(&options, baud);
    cfsetospeed(&options, baud);

    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_cflag &= ~CRTSCTS;

    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    options.c_iflag &= ~(INLCR | ICRNL | IGNCR);
    options.c_oflag &= ~OPOST;

    options.c_cc[VMIN] = 1;   // 至少读取1字节
    options.c_cc[VTIME] = 10; // 100ms超时

    tcsetattr(fd_, TCSANOW, &options);
    tcflush(fd_, TCIOFLUSH);
    return true;
}

void LDSDriver::end() {
    if (fd_ >= 0) {
        close(fd_);
        fd_ = -1;
    }
}

bool LDSDriver::readBytes(uint8_t* buf, size_t len) {
    size_t total = 0;
    while (total < len) {
        ssize_t ret = read(fd_, buf + total, len - total);
        if (ret <= 0) {
            if (errno == EAGAIN || errno == EINTR) continue;
            return false;
        }
        total += ret;
    }
    return true;
}

// 解析一个数据块（具体协议需根据LDRobot文档调整，此处为示例格式）
bool LDSDriver::parsePacket(const uint8_t* data, size_t len, std::vector<LaserPoint>& scan) {
    // 假设数据格式：起始标志 0x54 0x2C，后面跟着点云数据，每点3字节（距离2字节+强度1字节）
    // 一圈数据由多个这样的包组成，通过角度判断开始和结束。
    // 此函数需要维护状态机，这里仅示意，您需根据实际协议实现。
    // 为了简化，假设外部通过其他方式已经获得一圈所有点。
    return false;
}

size_t LDSDriver::getFullScan(std::vector<LaserPoint>& out_scan, uint32_t timeout_ms) {
    out_scan.clear();
    // 实际实现需要读取串口数据，解析出完整一圈的点云。
    // 由于协议未知，此处无法提供完整代码。
    // 建议：参考LDRobot SDK或使用其提供的动态库。
    return 0;
}
