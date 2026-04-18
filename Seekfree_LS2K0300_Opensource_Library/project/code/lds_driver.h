#ifndef LDS_DRIVER_H
#define LDS_DRIVER_H

#include <cstdint>
#include <vector>

struct LaserPoint {
    float    angle_deg;    // 角度（度）
    uint16_t distance_mm;  // 距离（毫米）
    uint8_t  intensity;    // 信号强度（0-255）
};

class LDSDriver {
public:
    LDSDriver();
    ~LDSDriver();

    // 打开串口
    bool begin(const char* port, int baudrate = 230400);
    // 关闭串口
    void end();
    // 读取一圈完整扫描数据（阻塞直到一圈完成），返回点云数量
    size_t getFullScan(std::vector<LaserPoint>& out_scan, uint32_t timeout_ms = 2000);

private:
    int fd_;
    bool readBytes(uint8_t* buf, size_t len);
    bool parsePacket(const uint8_t* data, size_t len, std::vector<LaserPoint>& scan);
};

#endif
