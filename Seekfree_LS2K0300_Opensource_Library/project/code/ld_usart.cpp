#include "zf_common_headfile.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>

static int uart_fd = -1;
LiDARFrameTypeDef g_lidar_frame;
volatile bool g_lidar_frame_valid = false;

typedef struct {
    float angle;            // 平均角度 (度)
    uint16_t distance;      // 平均距离 (mm)
} PointData_t;

// 假设全局定义了 50 个元素的环形缓冲，供避障决策使用
extern PointData_t PointDataProcess[50];

// CRC-8 表（与官方手册完全一致）
static const uint8_t crc8_table[256] = {
    0x00,0x4d,0x9a,0xd7,0x79,0x34,0xe3,0xa6,
    0xf2,0xbf,0x68,0x25,0x8b,0xc6,0x11,0x5c,
    0xa9,0xe4,0x33,0x7e,0xd0,0x9d,0x4a,0x07,
    0x5b,0x16,0xc1,0x8c,0x22,0x6f,0xb8,0xf5,
    0x1f,0x52,0x85,0xc8,0x66,0x2b,0xfc,0xb1,
    0xed,0xa0,0x77,0x3a,0x94,0xd9,0x0e,0x43,
    0xbb,0xfb,0x2c,0x61,0xcf,0x82,0x55,0x18,
    0x44,0x09,0xde,0x93,0x3d,0x70,0xa7,0xea,
    0x3e,0x73,0xa4,0xe9,0x47,0x0a,0xdd,0x90,
    0xcc,0x81,0x56,0x1b,0xb5,0xf8,0x2f,0x62,
    0x97,0xda,0x0d,0x40,0xee,0xa3,0x74,0x39,
    0x65,0x28,0xff,0xb2,0x1c,0x51,0x86,0xcb,
    0x21,0x6c,0xbb,0xf6,0x58,0x15,0xc2,0x8f,
    0xd3,0x9e,0x49,0x04,0xaa,0xe7,0x30,0x7d,
    0x88,0xc5,0x12,0x5f,0xf1,0xbc,0x6b,0x26,
    0x7a,0x37,0xe0,0xad,0x03,0x4e,0x99,0xd4,
    0x7c,0x31,0xe6,0xab,0x05,0x48,0x9f,0xd2,
    0x8e,0xc3,0x14,0x59,0xf7,0xba,0x6d,0x20,
    0xd5,0x98,0x4f,0x02,0xac,0xe1,0x36,0x7b,
    0x27,0x6a,0xbd,0xf0,0x5e,0x13,0xc4,0x89,
    0x63,0x2e,0xf9,0xb4,0x1a,0x57,0x80,0xcd,
    0x91,0xdc,0x0b,0x46,0xe8,0xa5,0x72,0x3f,
    0xca,0x87,0x50,0x1d,0xb3,0xfe,0x29,0x64,
    0x38,0x75,0xa2,0xef,0x41,0x0c,0xdb,0x96,
    0x42,0x0f,0xd8,0x95,0x3b,0x76,0xa1,0xec,
    0xb0,0xfd,0x2a,0x67,0xc9,0x84,0x53,0x1e,
    0xeb,0xa6,0x71,0x3c,0x92,0xdf,0x08,0x45,
    0x19,0x54,0x83,0xce,0x60,0x2d,0xfa,0xb7,
    0x5d,0x10,0xc7,0x8a,0x24,0x69,0xbe,0xf3,
    0xaf,0xe2,0x35,0x78,0xd6,0x9b,0x4c,0x01,
    0xf4,0xb9,0x6e,0x23,0x8d,0xc0,0x17,0x5a,
    0x06,0x4b,0x9c,0xd1,0x7f,0x32,0xe5,0xa8
};

static uint8_t CalCRC8(const uint8_t *p, uint8_t len) {
    uint8_t crc = 0;
    while (len--) crc = crc8_table[(crc ^ *p++) & 0xff];
    return crc;
}

//----------------------------------------------------------
// 雷达数据包初始化判断，来判断是否成功连接串口
//----------------------------------------------------------
bool ld_usart_init(const char *device, int baudrate) {
    uart_fd = open(device, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (uart_fd < 0) {
        perror("open uart");
        return false;
    }

    struct termios options;
    tcgetattr(uart_fd, &options);

    cfsetispeed(&options, B230400);
    cfsetospeed(&options, B230400);

    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_cflag &= ~CRTSCTS;    // 关闭硬件流控

    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    options.c_iflag &= ~(INLCR | ICRNL | IGNCR);
    options.c_oflag &= ~OPOST;

    options.c_cc[VMIN]  = 0;        // 纯非阻塞
    options.c_cc[VTIME] = 0;

    tcsetattr(uart_fd, TCSANOW, &options);
    tcflush(uart_fd, TCIOFLUSH);

    printf("LD D300 serial port initialized: %s @ %d\n", device, baudrate);
    return true;
}

//----------------------------------------------------------
// 雷达数据包解析函数，解析360°所有方向的点云数据，并存入全局变量 g_lidar_frame 中
// 使用方法：在中断中调用 ld_usart_task();
//----------------------------------------------------------
void ld_usart_task(void) {
    if (uart_fd < 0) return;

    static uint8_t buf[PACKET_SIZE];
    static uint8_t state = 0;      // 0: 等待 0x54
    static uint8_t cnt = 0;        // 测量点计数（备用）
    uint8_t ch;

    // 将指针提到这里，避免 switch 中声明
    LiDARFrameTypeDef *frame = NULL;

    while (read(uart_fd, &ch, 1) > 0) {
        switch (state) {
            case 0:   // 起始符 0x54
                if (ch == 0x54) {
                    buf[0] = ch;
                    state = 1;
                }
                break;

            case 1:   // 帧信息 0x2C
                if (ch == 0x2C) {
                    buf[1] = ch;
                    state = 2;
                } else {
                    state = 0;   // 不是 0x2C，重新同步
                }
                break;

            case 2: case 3:   // 转速 2 字节 (LSB, MSB)
                buf[state] = ch;
                state++;
                break;

            case 4: case 5:   // 起始角度 2 字节
                buf[state] = ch;
                state++;
                break;

            // 12 个测量点，每个点 3 字节，共 36 字节
            case 6 ... 41:
                buf[state] = ch;
                state++;
                break;

            case 42: case 43:  // 结束角度 2 字节
                buf[state] = ch;
                state++;
                break;

            case 44: case 45:  // 时间戳 2 字节
                buf[state] = ch;
                state++;
                break;

            case 46:           // CRC8 最后一字节
                buf[46] = ch;
                state = 0;     // 准备好接收下一帧
                cnt = 0;

                // 解析校验
                frame = (LiDARFrameTypeDef *)buf;
                if ((frame->ver_len & 0x1F) == POINT_PER_PACK) {
                    uint8_t crc = CalCRC8(buf, PACKET_SIZE - 1);
                    if (crc == frame->crc8) {
                        memcpy(&g_lidar_frame, buf, sizeof(LiDARFrameTypeDef));
                        g_lidar_frame_valid = true;
                    }
                }
                break;

            default:
                state = 0;
                break;
        }
    }
}

//----------------------------------------------------------
// 数据处理函数，提取前方 ±50° 范围内的点，并计算平均距离
// 使用方法：在中断中调用 data_process();
//----------------------------------------------------------
void data_process(void)
{
    static uint8_t data_cnt = 0;   // 环形缓冲索引
    uint8_t i;
    uint32_t distance_sum = 0;
    uint16_t ave_distance = 0;
    float area_angle;
    float start_angle, end_angle;

    // 等待新的有效帧
    if (!g_lidar_frame_valid) {
        return;
    }

    // 角度转换（原始数据单位 0.01°，除以 100 得到度）
    start_angle = g_lidar_frame.start_angle / 100.0f;
    end_angle   = g_lidar_frame.end_angle   / 100.0f;

    // 处理跨越 0° 的情况，保证 end_angle > start_angle
    if (start_angle > end_angle) {
        end_angle += 360.0f;
    }
    area_angle = start_angle + (end_angle - start_angle) / 2.0f;
    if (area_angle >= 360.0f) {
        area_angle -= 360.0f;
    }

    // 只提取机器人前方 ±50° 范围内的点（对应 >220° 或 <320°）
    if (area_angle > 220.0f || area_angle < 320.0f)
    {
        // 累加所有 12 个测量点的距离
        for (i = 0; i < POINT_PER_PACK; i++) {
            distance_sum += g_lidar_frame.point[i].distance;
        }
        ave_distance = (uint16_t)(distance_sum / POINT_PER_PACK);

        // 存入环形数组，覆盖旧数据
        PointDataProcess[data_cnt].angle    = area_angle;
        PointDataProcess[data_cnt].distance = ave_distance;
        data_cnt = (data_cnt + 1) % 50;    // 循环递增
    }

    // 标记当前帧已处理完毕
    g_lidar_frame_valid = false;
}
