/*********************************************************************************************************************
* LS2K0300 Opensourec Library 即（LS2K0300 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件是LS2K0300 开源库的一部分
*
* LS2K0300 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
*
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
*
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
*
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
*
* 文件名称          uwb_usart
* 公司名称          成都逐飞科技有限公司
* 适用平台          LS2K0300
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者           备注
* 2025-02-27        大W            first version
********************************************************************************************************************/

#include "uwb_usart.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/select.h>
#include <sys/time.h>
#include <sys/ioctl.h>
#include <asm/termbits.h>

// <termios.h> conflicts with <asm/termbits.h> on LoongArch (both define struct termios).
// We only need tcflush / TCIOFLUSH from termios.h, so declare them manually.
#ifndef TCIOFLUSH
#define TCIOFLUSH 2
#endif
extern "C" int tcflush(int, int);

//==================================================内部数据定义====================================================

static int              g_uwb_fd = -1;
static UWB_RxCtrl       g_uwb_rx;
UWB_Data                g_uwb_data;
volatile uint32_t       g_uwb_frame_count = 0;

typedef union
{
    uint8          raw[sizeof(UWB_RawFrame)];
    UWB_RawFrame   item;
} UWB_RawBuf;

static UWB_RawBuf       g_uwb_buf;

// EMA 滤波状态
static float            g_ema_dist_m  = 0.0f;
static float            g_ema_azim    = 0.0f;
static float            g_ema_elev    = 0.0f;
static uint8            g_ema_init    = 0;

//==================================================内部函数声明====================================================

static void process_byte   (uint8 byte);
static void parse_frame    (void);
static void reset_rx_ctrl  (void);
static int  ser_available  (void);
static void dump_raw_frame (void);

//==================================================字节序转换======================================================
// UWB 模块数据为大端序，LS2K0300 为小端序

uint16 uwb_byte_swap16(uint16 val)
{
    return (val >> 8) | (val << 8);
}

uint32 uwb_byte_swap32(uint32 val)
{
    return ((val & 0x000000FF) << 24) |
           ((val & 0x0000FF00) <<  8) |
           ((val & 0x00FF0000) >>  8) |
           ((val & 0xFF000000) >> 24);
}

//==================================================内部函数========================================================

static void reset_rx_ctrl(void)
{
    g_uwb_rx.rx_status   = UWB_UART_IDLE;
    g_uwb_rx.rx_time     = 0;
    g_uwb_rx.rx_num      = 0;
    g_uwb_rx.get_check   = 0;
    g_uwb_rx.inc_num     = 0;
    g_uwb_rx.inc_limit   = 0;
    g_uwb_rx.command     = 0;
    g_uwb_rx.end_flag    = 0;
}

static int ser_available(void)
{
    if (g_uwb_fd < 0) return 0;

    fd_set rfds;
    struct timeval tv = {0, 0};

    FD_ZERO(&rfds);
    FD_SET(g_uwb_fd, &rfds);

    return select(g_uwb_fd + 1, &rfds, NULL, NULL, &tv) > 0;
}

static void process_byte(uint8 byte)
{
    if ((g_uwb_rx.rx_status != UWB_UART_IDLE) &&
        (g_uwb_rx.rx_status != UWB_UART_RECVING))
        return;

    g_uwb_buf.raw[g_uwb_rx.rx_num++] = byte;
    g_uwb_rx.rx_time = UWB_RX_DLY_MAX;
    g_uwb_rx.rx_status = UWB_UART_RECVING;

    // 帧头检测：收到 0xFF 时递增，否则清零
    if (byte == 0xFF)
        g_uwb_rx.end_flag++;
    else
        g_uwb_rx.end_flag = 0;

    // 连续 4 个 0xFF → 检测到帧头 (含重同步)
    if (g_uwb_rx.end_flag >= 4)
    {
        g_uwb_rx.end_flag  = 0;
        g_uwb_rx.get_check = 1;
        g_uwb_rx.inc_num   = 0;
        g_uwb_rx.inc_limit = 0;
        g_uwb_rx.command   = 0;
        g_uwb_rx.rx_num    = 4;
        return;
    }

    // 帧头已锁定，逐字节统计并识别命令
    if (g_uwb_rx.get_check)
    {
        g_uwb_rx.inc_num++;

        if (g_uwb_rx.inc_num == 5)
        {
            g_uwb_rx.command = byte;
        }

        if (g_uwb_rx.inc_num == 6)
        {
            g_uwb_rx.command = (g_uwb_rx.command << 8) | byte;

            if (g_uwb_rx.command == UWB_CMD_DIST_AZIMUTH)
            {
                g_uwb_rx.inc_limit = 33;
            }
        }

        if (g_uwb_rx.inc_num == g_uwb_rx.inc_limit)
        {
            g_uwb_rx.rx_status = UWB_UART_RECED;
        }
    }

    // 缓冲区溢出保护
    if (g_uwb_rx.rx_num >= UWB_RX_BUF_SIZE)
    {
        g_uwb_rx.rx_status = UWB_UART_RECED;
    }
}

static void dump_raw_frame(void)
{
    printf("[UWB RAW] %u bytes:", g_uwb_rx.rx_num);
    for (uint16 i = 0; i < g_uwb_rx.rx_num && i < 37; i++)
    {
        if (i % 16 == 0) printf("\r\n  ");
        printf("%02X ", g_uwb_buf.raw[i]);
    }
    printf("\r\n");
}

static void parse_frame(void)
{
    if (g_uwb_rx.command != UWB_CMD_DIST_AZIMUTH)
        return;

    if (g_uwb_rx.rx_num < 37)
        return;

    int32  dist   = (int32)uwb_byte_swap32(g_uwb_buf.item.distance);
    int16  azim   = (int16)uwb_byte_swap16((uint16)g_uwb_buf.item.azimuth);
    int16  elev   = (int16)uwb_byte_swap16((uint16)g_uwb_buf.item.elevation);
    uint32 tag_id = uwb_byte_swap32(g_uwb_buf.item.tag_id);
    uint32 anc_id = uwb_byte_swap32(g_uwb_buf.item.anchor_id);

    // 原始数据（不做校准）
    g_uwb_data.distance      = dist;
    // g_uwb_data.azimuth_deg   = (float)azim * 0.01f;
    g_uwb_data.azimuth_deg   = (float)azim;
    g_uwb_data.elevation_deg = (float)elev * 0.01f;
    g_uwb_data.tag_id        = tag_id;
    g_uwb_data.anchor_id     = anc_id;

    // 校准：corrected = raw * scale + offset
    float raw_dist_m = ((float)dist * UWB_DIST_SCALE + (float)UWB_DIST_OFFSET_MM) * 0.001f;
    float raw_azim   = g_uwb_data.azimuth_deg;
    float raw_elev   = g_uwb_data.elevation_deg;

    // EMA 低通滤波
    if (!g_ema_init)
    {
        g_ema_dist_m = raw_dist_m;
        g_ema_azim   = raw_azim;
        g_ema_elev   = raw_elev;
        g_ema_init   = 1;
    }
    else
    {
        g_ema_dist_m = UWB_FILTER_ALPHA * raw_dist_m + (1.0f - UWB_FILTER_ALPHA) * g_ema_dist_m;
        g_ema_azim   = UWB_FILTER_ALPHA * raw_azim   + (1.0f - UWB_FILTER_ALPHA) * g_ema_azim;
        g_ema_elev   = UWB_FILTER_ALPHA * raw_elev   + (1.0f - UWB_FILTER_ALPHA) * g_ema_elev;
    }

    g_uwb_data.distance_m  = g_ema_dist_m;
    g_uwb_data.azimuth_f   = g_ema_azim;
    g_uwb_data.elevation_f = g_ema_elev;
    g_uwb_data.data_valid  = 1;
    g_uwb_data.timestamp   = 0;
    g_uwb_frame_count++;  // 数据刷新，帧计数递增
}

//==================================================外部 API=========================================================

void uwb_usart_init(const char *device, uint32 baudrate)
{
    if (g_uwb_fd >= 0)
        uwb_usart_close();

    reset_rx_ctrl();
    memset(&g_uwb_data, 0, sizeof(g_uwb_data));
    memset(&g_uwb_buf,  0, sizeof(g_uwb_buf));
    g_ema_init = 0;

    g_uwb_fd = open(device, O_RDWR | O_NOCTTY | O_NDELAY);
    if (g_uwb_fd < 0)
    {
        printf("[UWB] open %s failed: %s\r\n", device, strerror(errno));
        return;
    }

    // 使用 termios2 + TCSETS2 设置波特率
    // LS2K0300 的 termios 不含 c_ispeed/c_ospeed，cfsetispeed(B115200) 会
    // 因 CBAUD 位宽不足返回 EINVAL。termios2 通过 BOTHER 模式支持任意整数波特率。
    struct termios2 tio;
    memset(&tio, 0, sizeof(tio));

    if (ioctl(g_uwb_fd, TCGETS2, &tio) != 0)
    {
        printf("[UWB] TCGETS2 get failed: %s\r\n", strerror(errno));
        close(g_uwb_fd);
        g_uwb_fd = -1;
        return;
    }

    // 原始模式
    tio.c_iflag = 0;
    tio.c_oflag = 0;
    tio.c_lflag = 0;
    tio.c_cflag = CS8 | CLOCAL | CREAD;
    tio.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS);

    // BOTHER 模式：c_ispeed/c_ospeed 为直接整数波特率
    tio.c_cflag &= ~CBAUD;
    tio.c_cflag |= BOTHER;
    tio.c_ispeed = baudrate;
    tio.c_ospeed = baudrate;

    tio.c_cc[VMIN]  = 0;
    tio.c_cc[VTIME] = 0;

    tcflush(g_uwb_fd, TCIOFLUSH);

    if (ioctl(g_uwb_fd, TCSETS2, &tio) != 0)
    {
        printf("[UWB] TCSETS2 set failed: %s\r\n", strerror(errno));
        close(g_uwb_fd);
        g_uwb_fd = -1;
        return;
    }

    // 回读验证
    struct termios2 verify;
    memset(&verify, 0, sizeof(verify));
    if (ioctl(g_uwb_fd, TCGETS2, &verify) == 0)
    {
        printf("[UWB] init ok, dev=%s baud=%u actual_in=%u actual_out=%u\r\n",
               device, baudrate, verify.c_ispeed, verify.c_ospeed);
    }
    else
    {
        printf("[UWB] init ok, dev=%s baud=%u (verify failed)\r\n", device, baudrate);
    }
}

void uwb_usart_task(void)
{
    if (g_uwb_fd < 0) return;

    // 超时检测
    if (g_uwb_rx.rx_time)
    {
        g_uwb_rx.rx_time--;
        if (g_uwb_rx.rx_time == 0)
            g_uwb_rx.rx_status = UWB_UART_RECED;
    }

    // 快速拉取所有可用字节
    while (ser_available())
    {
        uint8 byte;
        int n = read(g_uwb_fd, &byte, 1);
        if (n <= 0) break;
        process_byte(byte);
    }

    // 帧接收完毕 → 解算
    if (g_uwb_rx.rx_status == UWB_UART_RECED)
    {
        parse_frame();
        reset_rx_ctrl();

        if (g_uwb_data.data_valid)
        {
            //原始距离 (mm) 和校准后距离 (m)
            printf("[UWB] Tag=%u Anc=%u "
                   "raw=%dmm cal=%.2fm filt=%.2fm "
                   "azi=%.1f/%.1f elev=%.1f/%.1f\r\n",
                   g_uwb_data.tag_id,
                   g_uwb_data.anchor_id,
                   g_uwb_data.distance,
                   ((float)g_uwb_data.distance * UWB_DIST_SCALE + (float)UWB_DIST_OFFSET_MM) * 0.001f,
                   g_uwb_data.distance_m,
                   g_uwb_data.azimuth_deg,
                   g_uwb_data.azimuth_f,
                   g_uwb_data.elevation_deg,
                   g_uwb_data.elevation_f);
            g_uwb_data.data_valid = 0;
        }
    }
}

void uwb_usart_close(void)
{
    if (g_uwb_fd >= 0)
    {
        close(g_uwb_fd);
        g_uwb_fd = -1;
        g_ema_init = 0;
        printf("[UWB] closed\r\n");
    }
}