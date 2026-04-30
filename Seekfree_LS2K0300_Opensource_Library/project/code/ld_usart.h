#ifndef __ld_usart_h
#define __ld_usart_h

#include "zf_common_headfile.h"
#include <stdint.h>

#define POINT_PER_PACK   12
#define PACKET_SIZE      47        // 1+1+2+2+12*3+2+2+1

#pragma pack(1)
typedef struct {
    uint16_t distance;
    uint8_t  intensity;
} LidarPointStructDef;

typedef struct {
    uint8_t  header;
    uint8_t  ver_len;
    uint16_t speed;
    uint16_t start_angle;
    LidarPointStructDef point[POINT_PER_PACK];
    uint16_t end_angle;
    uint16_t timestamp;
    uint8_t  crc8;
} LiDARFrameTypeDef;
#pragma pack()

extern LiDARFrameTypeDef g_lidar_frame;
extern volatile bool g_lidar_frame_valid;

bool ld_usart_init(const char *device, int baudrate);
void ld_usart_task(void);

#endif