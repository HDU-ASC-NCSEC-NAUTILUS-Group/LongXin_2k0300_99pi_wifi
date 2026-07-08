/********************************************************************************************************************
 * 上下板通信处理 — 接收上板数据，直行到 UWB 信标处停车
 *
 * 通信链路:
 *   上板: QR_process() 识别二维码 → uart1_printf("[01]\n") 发送命令
 *         servo_move_sync() 抓取完成 → uart1_printf("[DONE]\n") 发送 DONE
 *   下板: uart1_recv_frame() 统一接收 [命令]\n 格式帧
 *         阶段A 根据命令内容分发，阶段B 运行状态机
 *
 * 命令说明:
 *   [01] → distance=0.5m, 触发 MOVING_TO_BEACON
 *   [02] → distance=1.0m, 触发 MOVING_TO_BEACON
 *   [03] → distance=0.5m, 触发 MOVING_TO_BEACON
 *   [DONE] → 置位 g_done_pending，在 TRANSPORT_DONE 状态下触发 TRANSPORT_AGAIN
 *
 * 状态机:
 *   IDLE ──(收到 01/02/03)──> MOVING_TO_BEACON
 *   MOVING_TO_BEACON ──(UWB距离 ≤ TRANSPORT_STOP_DIST_M)──> DONE
 *   MOVING_TO_BEACON ──(UWB 超时)──> IDLE（安全停车）
 *   DONE ──(g_done_pending 为真)──> AGAIN
 *   DONE ──(UWB 超时)──> IDLE（安全停车）
 *   AGAIN ──(UWB距离 ≤ distance)──> PROCESS_DONE
 *   AGAIN ──(UWB 超时 或 distance无效)──> IDLE（安全停车）
 *   PROCESS_DONE ──> 发送 [DONE]\n 给上板 ──> IDLE
 ********************************************************************************************************************/
#include "zf_common_headfile.h"

//==================================================运输状态枚举====================================================

enum TransportState
{
    TRANSPORT_IDLE = 0,             // 空闲：等待上板数据帧
    TRANSPORT_MOVING_TO_BEACON,     // 直行中：以 PWM 2500 驶向信标
    TRANSPORT_DONE,                 // 完成：已到达信标附近
    TRANSPORT_AGAIN,                // 再次触发：收到 DONE 指令，驶向仓库
    PROCESS_DONE                    // 处理完成：已到达仓库，发送完成信号给上板
};

//==================================================运输参数==========================================================

#define TRANSPORT_MOVE_PWM          5000        // 直行 PWM (0~10000)
#define TRANSPORT_STOP_DIST_M       1.5f        // 信标停车距离 (米)
#define TRANSPORT_UWB_TIMEOUT_LOOPS 2000        // UWB 超时（主循环迭代次数，约等效 2s）

//==================================================全局变量==========================================================

float distance = 0.0f;                          // 从命令解析的目标距离(m)，由 01/02/03 设置

//==================================================内部静态变量======================================================

static int      g_transport_state   = TRANSPORT_IDLE;   // 当前状态
static uint32_t g_last_uwb_count    = 0;                // 上一帧 UWB 帧计数
static uint32_t g_uwb_stale_calls   = 0;                // 连续无新 UWB 帧的调用次数
static bool     g_done_pending      = false;            // 收到过 DONE 但尚未处理

//==================================================运输任务主函数=====================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     运输任务主函数
// 使用示例     transport();  // 放在主循环 while(1) 中轮询
// 备注信息     两阶段设计:
//              阶段A: uart1_recv_frame() 统一接收 UART 命令 → 根据内容分发
//              阶段B: 状态机驱动电机 + UWB 距离监控（每次迭代都执行）
//-------------------------------------------------------------------------------------------------------------------
void transport(void)
{
    // ==========================================================================
    // 阶段 A: UART 命令接收（所有状态共用，每次最多处理一帧）
    // ==========================================================================

    char *cmd = uart1_recv_frame();

    if (cmd)
    {

        if (strcmp(cmd, "01") == 0)
        {
            g_done_pending     = false;      // 新任务清除旧的 DONE 等待
            g_transport_state  = TRANSPORT_MOVING_TO_BEACON;
            g_last_uwb_count   = g_uwb_frame_count;
            g_uwb_stale_calls  = 0;
            distance = 0.5f;
            printf("[TRANSPORT] 收到01, 开始直行 目标distance=%.1fm PWM=%d\r\n",
                   distance, TRANSPORT_MOVE_PWM);
        }
        else if (strcmp(cmd, "02") == 0)
        {
            g_done_pending     = false;
            g_transport_state  = TRANSPORT_MOVING_TO_BEACON;
            g_last_uwb_count   = g_uwb_frame_count;
            g_uwb_stale_calls  = 0;
            distance = 1.0f;
            printf("[TRANSPORT] 收到02, 开始直行 目标distance=%.1fm PWM=%d\r\n",
                   distance, TRANSPORT_MOVE_PWM);
        }
        else if (strcmp(cmd, "03") == 0)
        {
            g_done_pending     = false;
            g_transport_state  = TRANSPORT_MOVING_TO_BEACON;
            g_last_uwb_count   = g_uwb_frame_count;
            g_uwb_stale_calls  = 0;
            distance = 1.5f;
            printf("[TRANSPORT] 收到03, 开始直行 目标distance=%.1fm PWM=%d\r\n",
                   distance, TRANSPORT_MOVE_PWM);
        }

        // ---- DONE 命令：无条件置位 pending，等 TRANSPORT_DONE 时消费 ----
        else if (strcmp(cmd, "DONE") == 0)
        {
            g_done_pending = true;
            g_transport_state  = TRANSPORT_AGAIN;
            printf("[TRANSPORT] 收到DONE (pending, 当前状态=%d)\r\n", g_transport_state);
        }

        printf("[TRANSPORT] UART命令: %s\r\n", cmd);
    }

    // ==========================================================================
    // 阶段 B: 状态机 — 电机控制与距离监控（每次迭代都执行）
    // ==========================================================================

    switch (g_transport_state)
    {

    case TRANSPORT_IDLE:
        break;

    // =====================================================================
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
            printf("[TRANSPORT] UWB 超时, 安全停车\r\n");
            return;
        }

        // ---- 距离检测：到达信标附近 → 停车 ----
        float dist = g_uwb_data.distance_m;

        if (dist > 0.0f && dist <= TRANSPORT_STOP_DIST_M)
        {
            Motor_Reset_ALL();
            g_transport_state = TRANSPORT_DONE;
            printf("[TRANSPORT] 到达信标 %.2fm, 停车 (目标=%.1fm)\r\n",
                   dist, TRANSPORT_STOP_DIST_M);
            return;
        }

        // ---- 持续直行 ----
        printf("[TRANSPORT] MOTOR: Move_Straight(%d) dist=%.2f fcnt=%u stalls=%u\r\n",
               TRANSPORT_MOVE_PWM, g_uwb_data.distance_m,
               g_uwb_frame_count, g_uwb_stale_calls);
        Motor_Move_Straight(TRANSPORT_MOVE_PWM);
        break;
    }

    // =====================================================================
    case TRANSPORT_DONE:
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

        // ---- DONE 已到达（可能提前到达）→ 立即进入 AGAIN ----
        if (g_done_pending)
        {
            g_done_pending     = false;
            g_transport_state  = TRANSPORT_AGAIN;
            g_last_uwb_count   = g_uwb_frame_count;
            g_uwb_stale_calls  = 0;
            printf("[TRANSPORT] DONE生效, 开始二次直行 目标distance=%.1fm PWM=%d\r\n",
                   distance, TRANSPORT_MOVE_PWM);
            // 本次先 return，下个循环再跑 AGAIN 的电机逻辑（避免同一次调用中跳跃两个状态）
            return;
        }

        // ---- DONE 等待超时保护 ----
        if (g_uwb_stale_calls >= TRANSPORT_UWB_TIMEOUT_LOOPS)
        {
            Motor_Reset_ALL();
            g_transport_state = TRANSPORT_IDLE;
            printf("[TRANSPORT] DONE: 等待超时, 回IDLE\r\n");
            return;
        }

        break;
    }

    // =====================================================================
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

        // ---- distance 有效性保护 ----
        if (distance <= 0.0f)
        {
            Motor_Reset_ALL();
            g_transport_state = TRANSPORT_IDLE;
            printf("[TRANSPORT] AGAIN: distance=%.1f 无效, 回IDLE\r\n", distance);
            return;
        }

        // ---- 距离检测：到达仓库目标距离 → 停车 ----
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
        printf("[TRANSPORT] MOTOR: Move_Straight(%d) dist=%.2f fcnt=%u stalls=%u\r\n",
               TRANSPORT_MOVE_PWM, g_uwb_data.distance_m,
               g_uwb_frame_count, g_uwb_stale_calls);
        Motor_Move_Straight(TRANSPORT_MOVE_PWM);
        break;
    }

    // =====================================================================
    case PROCESS_DONE:
    {
        uart1_printf("[DONE]\n");          // 发送完成信号给上板
        g_transport_state = TRANSPORT_IDLE; // 重置状态为 IDLE
        break;
    }

    // =====================================================================
    default:
        g_transport_state = TRANSPORT_IDLE;
        break;

    } // end switch
}
