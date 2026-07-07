/********************************************************************************************************************
 * 上下板通信处理 — 接收上板数据，直行到 UWB 信标 0.4m 处停车
 *
 * 通信链路:
 *   上板: QR_process() -> g_qr_data -> qr_settlement() -> g_tx_packet = <QR原始数据>\r\n
 *         uart1_send() 发送完整帧
 *   下板: uart1_recv() 逐字节接收 -> 检测 \r\n 帧尾 -> 触发 MOVING 状态
 *         帧内若含 "01"/"02"/"03" 则更新 distance 变量
 *
 * 核心逻辑:
 *   收到任何完整帧（以 \r\n 结尾）→ 无条件触发直行到 0.4m
 *   distance 变量仅根据帧内是否出现 "01"/"02"/"03" 来更新，不影响触发条件
 *
 * 状态机:
 *   IDLE ──(收到完整帧)──> MOVING_TO_BEACON
 *   MOVING_TO_BEACON ──(UWB距离 ≤ 3.0m)──> DONE（停车）
 *   MOVING_TO_BEACON ──(UWB 超时)──> IDLE（安全停车）
 *   DONE ──(收到 "DONE" 指令)──> AGAIN
 *   AGAIN ──(UWB距离 ≤ distance)──> PROCESS_DONE（发送 [DONE]\n 给上板）
 *   AGAIN ──(UWB 超时)──> IDLE（安全停车）
 *   PROCESS_DONE ──> IDLE（重置）
 ********************************************************************************************************************/
#include "zf_common_headfile.h"

//==================================================运输状态枚举====================================================

enum TransportState
{
    TRANSPORT_IDLE = 0,             // 空闲：等待上板数据帧
    TRANSPORT_MOVING_TO_BEACON,     // 直行中：以 PWM 2500 驶向信标
    TRANSPORT_DONE,                 // 完成：已到达信标 3.0m 处
    TRANSPORT_AGAIN,                // 再次触发：收到新帧，重新直行
    PROCESS_DONE                    // 处理完成：已到达仓库，流程完成，发送完成信号给上板
};

//==================================================运输参数==========================================================

#define TRANSPORT_MOVE_PWM          2500        // 直行 PWM (0~10000)
#define TRANSPORT_STOP_DIST_M       0.3f        // 目标停车距离 (米)
#define TRANSPORT_UWB_TIMEOUT_LOOPS 2000        // UWB 超时（主循环迭代次数，约等效 2s）
#define TRANSPORT_RX_BUF_SIZE       128         // 接收缓冲区大小

//==================================================全局变量==========================================================

float distance = 0.0f;                          // 从帧内解析的目标距离(m)，保留供后续步骤使用

//==================================================内部静态变量======================================================

static int      g_transport_state   = TRANSPORT_IDLE;   // 当前状态
static uint32_t g_last_uwb_count    = 0;                // 上一帧 UWB 帧计数
static uint32_t g_uwb_stale_calls   = 0;                // 连续无新帧的调用次数

//==================================================帧内命令扫描=====================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     在帧数据中扫描 "01"/"02"/"03"，更新全局 distance
// 参数说明     frame: 帧数据（不含 \r\n）
//              len:   帧数据长度
// 备注信息     扫描到 "01"/"02"/"03" 后一字节不能是数字（防 "010" 误触发）
//              允许多次匹配，以最后一次为准
//-------------------------------------------------------------------------------------------------------------------
static void scan_and_update_distance(const uint8_t *frame, int len)
{
    for (int i = 0; i <= len - 2; i++)
    {
        if (frame[i] != '0')
            continue;

        // 匹配 "01" / "02" / "03"
        switch (frame[i + 1])
        {
            case '1': distance = 1.5f; break;
            case '2': distance = 1.0f; break;
            case '3': distance = 0.5f; break;
            default:  continue;
        }

        // 防止更长数字误触发（如 "010" / "0123"）
        if (i + 2 < len)
        {
            char next = (char)frame[i + 2];
            if (next >= '0' && next <= '9')
                continue;   // 跳过，不更新 distance
        }
    }
}

//==================================================运输任务主函数=====================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     运输任务主函数
// 使用示例     transport();  // 放在主循环 while(1) 中轮询
// 备注信息     两阶段设计:
//              阶段A: 接收 UART 数据，检测 \r\n 帧尾 → 触发 MOVING
//              阶段B: 状态机驱动电机 + UWB 距离监控（每次迭代都执行）
//-------------------------------------------------------------------------------------------------------------------
void transport(void)
{
    static uint8_t rx_buf[TRANSPORT_RX_BUF_SIZE];
    static int     rx_len = 0;

    // ==========================================================================
    // 阶段 A: UART 接收 + 帧检测（有数据才处理）
    // ==========================================================================

    if (uart1_available())
    {
        int space = (int)sizeof(rx_buf) - 1 - rx_len;
        if (space > 0)
        {
            int n = uart1_recv(rx_buf + rx_len, (uint32)space);
            if (n > 0)
            {
                rx_len += n;
                rx_buf[rx_len] = '\0';
            }
        }
    }

    // ---- 检测完整帧（以 \r\n 或 \n 结尾） ----
    int frame_end = -1;     // 帧尾后第一个字节的索引（即帧数据 + 分隔符的总长度）

    for (int i = 0; i < rx_len; i++)
    {
        if (rx_buf[i] == '\r' && i + 1 < rx_len && rx_buf[i + 1] == '\n')
        {
            frame_end = i + 2;      // 包括 \r\n
            break;
        }
        if (rx_buf[i] == '\n')
        {
            frame_end = i + 1;      // 仅有 \n
            break;
        }
    }

    if (frame_end > 0)
    {
        // ============================================================
        // 收到完整帧！无条件触发运输任务
        // ============================================================

        // 帧数据部分（不含 \r\n）
        int frame_data_len = frame_end;
        if (frame_data_len >= 2 &&
            rx_buf[frame_end - 2] == '\r' &&
            rx_buf[frame_end - 1] == '\n')
        {
            frame_data_len = frame_end - 2;     // 去掉 \r\n
        }
        else if (frame_data_len >= 1 &&
                 rx_buf[frame_end - 1] == '\n')
        {
            frame_data_len = frame_end - 1;     // 去掉 \n
        }

        // 尝试从帧内扫描 "01"/"02"/"03" 更新 distance（不影响触发）
        if (frame_data_len > 0)
        {
            scan_and_update_distance(rx_buf, frame_data_len);
        }

        // 触发运输任务
        g_transport_state = TRANSPORT_MOVING_TO_BEACON;
        g_last_uwb_count  = g_uwb_frame_count;
        g_uwb_stale_calls = 0;

        printf("[TRANSPORT] 收到帧 (%d字节数据), distance=%.1fm, 开始直行 PWM=%d\r\n",
               frame_data_len, distance, TRANSPORT_MOVE_PWM);

        // 清除已处理的帧
        if (frame_end < rx_len)
        {
            memmove(rx_buf, rx_buf + frame_end, rx_len - frame_end);
        }
        rx_len -= frame_end;
    }

    // ---- 缓冲区溢出保护 ----
    // 长时间无 \r\n 时，保留末尾字节（可能是不完整的帧头），丢弃旧数据
    if (rx_len > (int)sizeof(rx_buf) - 16)
    {
        // 保留最后 8 字节，防止帧数据被截断
        int keep = 8;
        if (keep > rx_len) keep = rx_len;
        memmove(rx_buf, rx_buf + rx_len - keep, keep);
        rx_len = keep;
    }

    // ==========================================================================
    // 阶段 B: 状态机 — 电机控制与距离监控（每次迭代都执行）
    // ==========================================================================

    switch (g_transport_state)
    {

    case TRANSPORT_IDLE:
        break;

    case TRANSPORT_MOVING_TO_BEACON:
    {
        // ---- UWB 数据新鲜度检测 ----
        if (g_uwb_frame_count != g_last_uwb_count)
        {
            g_last_uwb_count  = g_uwb_frame_count;
            g_uwb_stale_calls = 0;
        }
        else
        {
            g_uwb_stale_calls++;
        }

        // ---- 从未收到过 UWB 数据 → 原地等待 ----
        // if (g_uwb_frame_count == 0)
        // {
        //     Motor_Reset_ALL();
        //     return;
        // }

        // ---- UWB 超时保护 ----
        if (g_uwb_stale_calls >= TRANSPORT_UWB_TIMEOUT_LOOPS)
        {
            Motor_Reset_ALL();
            g_transport_state = TRANSPORT_IDLE;
            printf("[TRANSPORT] UWB 超时, 安全停车\r\n");
            return;
        }

        // ---- 距离检测：到达 3.0m → 停车 ----
        float dist = g_uwb_data.distance_m;

        if (dist > 0.0f && dist <= TRANSPORT_STOP_DIST_M)
        {
            Motor_Reset_ALL();
            g_transport_state = TRANSPORT_DONE;
            printf("[TRANSPORT] 到达信标 %.2fm, 停车 (distance=%.1fm)\r\n",
                   dist, distance);
            return;
        }

        // ---- 持续直行 ----
        Motor_Move_Straight(TRANSPORT_MOVE_PWM);
        break;
    }

    case TRANSPORT_DONE:
    {
        // 等待上板的完成信号
        /* UART1 接收 */
        char *cmd = uart1_recv_frame();
        if(cmd)
        {
            if(strcmp(cmd, "DONE") == 0)
            {
                g_transport_state = TRANSPORT_AGAIN;
                g_last_uwb_count  = g_uwb_frame_count;
                g_uwb_stale_calls = 0;
                printf("[TRANSPORT] 收到DONE, 开始二次直行 目标distance=%.1fm PWM=%d\r\n",
                       distance, TRANSPORT_MOVE_PWM);
            }
        }
        break;
    }

    case TRANSPORT_AGAIN:
    {
        // ---- UWB 数据新鲜度检测 ----
        if (g_uwb_frame_count != g_last_uwb_count)
        {
            g_last_uwb_count  = g_uwb_frame_count;
            g_uwb_stale_calls = 0;
        }
        else
        {
            g_uwb_stale_calls++;
        }

        // ---- 从未收到过 UWB 数据 → 原地等待 ----
        if (g_uwb_frame_count == 0)
        {
            Motor_Reset_ALL();
            return;
        }

        // ---- UWB 超时保护 ----
        if (g_uwb_stale_calls >= TRANSPORT_UWB_TIMEOUT_LOOPS)
        {
            Motor_Reset_ALL();
            g_transport_state = TRANSPORT_IDLE;
            printf("[TRANSPORT] AGAIN: UWB 超时, 安全停车\r\n");
            return;
        }

        // ---- 距离检测：到达 distance 目标距离 → 停车 ----
        float dist = g_uwb_data.distance_m;

        if (dist > 0.0f && dist <= distance)
        {
            Motor_Reset_ALL();
            g_transport_state = PROCESS_DONE;
            printf("[TRANSPORT] 到达仓库 %.2fm (目标=%.1fm), 完成运输\r\n",
                   dist, distance);
            return;
        }

        // ---- 持续直行 ----
        Motor_Move_Straight(TRANSPORT_MOVE_PWM);
        break;
    }

    case PROCESS_DONE:
    {
        uart1_printf("[DONE]\n");          // 发送完成信号给上板
        g_transport_state = TRANSPORT_IDLE; // 重置状态为 IDLE
        break;
    }

    default:
        g_transport_state = TRANSPORT_IDLE;
        break;

    } // end switch
}
