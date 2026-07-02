/********************************************************************************************************************
 * 上下板通信处理 — 接收上板结算指令并控制电机
 *
 * 通信链路:
 *   上板: UVC摄像头拍二维码 -> QR_process() -> qr_settlement() -> uart1_send()
 *         (正常产出 "01"/"02"/"03" 等指令; 兜底产出 "RAW:<qr>\r\n")
 *   下板: uart1_recv() 逐字节接收 -> 滑动匹配 "01"/"02"/"03" -> Motor_Move_Straight()
 *
 * 数据特点:
 *   - 数据是纯 ASCII 字符串流，可能带 \r\n 换行
 *   - 单次 read 可能读到完整指令、半包、粘包
 *   - 需用静态累加缓冲 + 滑动窗口匹配，兼容带帧头帧尾的场景
 ********************************************************************************************************************/
#include "zf_common_headfile.h"

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     接收上板指令并控制电机
// 使用示例     transport();  // 放在主循环 while(1) 中轮询
// 备注信息     通过 UART1 (/dev/ttyS1) 接收上板 qr_settlement 结算后的指令
//              用滑动窗口在字节流中匹配 "01"/"02"/"03"，兼容任意帧头帧尾
//              "01" -> 所有电机 PWM  3000 (前进)
//              "02" -> 所有电机 PWM  6000 (前进)
//              "03" -> 所有电机 PWM -4000 (后退)
//-------------------------------------------------------------------------------------------------------------------
void transport(void)
{
    static uint8_t  rx_buf[64];         // 累加缓冲区
    static int      rx_len = 0;         // 已缓存字节数
    static int      skip_next = 0;      // 匹配成功后跳过剩余数据直到换行

    // ---- 1. 非阻塞读取 ----
    if (!uart1_available())
        return;

    int n = uart1_recv(rx_buf + rx_len, (uint32)(sizeof(rx_buf) - 1 - rx_len));
    if (n <= 0)
        return;

    rx_len += n;
    rx_buf[rx_len] = '\0';

    // ---- 2. 滑动窗口匹配 "01"/"02"/"03" ----
    for (int i = 0; i <= rx_len - 2; i++)
    {
        // 遇到换行: 重置跳过标志, 意味着新的一行/包开始
        if (rx_buf[i] == '\r' || rx_buf[i] == '\n')
        {
            skip_next = 0;
            continue;
        }

        // 如果正在跳过当前行（上一条指令已处理），仅查找换行
        if (skip_next)
            continue;

        // 滑动匹配 "0x"
        if (rx_buf[i] == '0' && i + 1 < rx_len)
        {
            int matched = 0;        // 0=未匹配, 3000/6000/-4000=匹配到的 PWM

            switch (rx_buf[i + 1])
            {
                case '1': matched =  3000;  break;
                case '2': matched =  6000;  break;
                case '3': matched = -4000;  break;
                default:  break;
            }

            if (matched != 0)
            {
                // 确保 "01"/"02"/"03" 是独立字段，不被更长数字误触发
                // (如 "010" / "0123" 等不应匹配)
                if (i + 2 < rx_len)
                {
                    char next = (char)rx_buf[i + 2];
                    // next 是数字 → 说明是更长数字的一部分，跳过
                    if (next >= '0' && next <= '9')
                        continue;
                }

                Motor_Move_Straight(matched);
                skip_next = 1;  // 本行剩余数据跳过，直到换行
            }
        }
    }

    // ---- 3. 防止缓冲区无限增长 ----
    // 如果缓存超过 60 字节仍未匹配到有效指令，保留末尾 2 字节
    // (防止 "0x" 跨包被截断)，丢弃前面的垃圾数据
    if (rx_len > 60)
    {
        rx_buf[0] = rx_buf[rx_len - 2];
        rx_buf[1] = rx_buf[rx_len - 1];
        rx_len = 2;
    }

    // 如果已经匹配并跳过了当前行，且遇到换行后，清理缓冲区
    // 此处做惰性清理：当 skip_next 且 rx_len 较大时压缩
    if (skip_next && rx_len > 32)
    {
        // 找到最后一个 \r 或 \n
        int cut = rx_len;
        for (int i = rx_len - 1; i >= 0; i--)
        {
            if (rx_buf[i] == '\r' || rx_buf[i] == '\n')
            {
                cut = i + 1;
                break;
            }
        }
        if (cut < rx_len)
        {
            memmove(rx_buf, rx_buf + cut, rx_len - cut);
            rx_len -= cut;
            skip_next = 0;
        }
    }
}
