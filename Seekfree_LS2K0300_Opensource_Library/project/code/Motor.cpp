
#include "zf_common_headfile.h"
#include "defines.h"


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置电机速度
// 使用示例     Motor_Set(1, 500);
// 参数说明     num：指定电机编号，范围：1-4
// 参数说明     speed：指定电机PWM值，范围：0-10000
//-------------------------------------------------------------------------------------------------------------------
void Motor_Set(int num, int speed)
{
    // 速度设置为正向
    if (speed >= 0)
    {
        switch (num)
        {
            case 1:


                break;
            case 2:


                break;
            case 3:


                break;
            case 4:


                break;
            default:
                // 留一手，防止其他电机编号错误导致的错误
                break;
        }

    }
    // 速度设置为反向
    else
    {
        switch (num)
        {
            case 1:

            
                break;
            case 2:


                break;
            case 3:


                break;
            case 4:


                break;
            default:
                // 留一手，防止其他电机编号错误导致的错误
                break;
        }
    } 
}