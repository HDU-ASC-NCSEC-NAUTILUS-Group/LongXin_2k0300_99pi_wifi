/********************************************************************************************************************
* 文件名称     IMU_Analysis.cpp
* 功能描述     IMU姿态解算和校准模块
* 使用说明     1. 支持三轴/六轴/九轴模式，通过 IMU_ANALYSIS_MODE 宏选择
*              2. 陀螺仪校准：采集静止时的数据，计算零偏
*              3. 磁力计校准：支持Min-Max法和椭球拟合法，旋转设备采集数据
*              4. 磁力计校准方法通过 MAG_CALIB_METHOD 宏选择（1:Min-Max, 2:椭球拟合）
********************************************************************************************************************/
#include "zf_device_imu963ra.h"
#include "zf_driver_file.h"

#include "IMU_Analysis.h"
#include <math.h>
#include <chrono>






/*******************************************************************************************************************/
/*[S] 通用全局变量 [S]-----------------------------------------------------------------------------------------------*/
/*******************************************************************************************************************/

// 全局变量
volatile float Yaw_Result = 0.0f;    // 偏航角（Yaw）
volatile float Roll_Result = 0.0f;   // 横滚角（Roll）
volatile float Pitch_Result = 0.0f;  // 俯仰角（Pitch）
// IMU 数据采集和分析使能标志位
volatile uint8_t IMU_D_and_A_Enable = 0;
// IMU快速收敛相关
volatile uint8_t imu_quick_count = 0;
volatile uint8_t imu_stable = 0;
/*******************************************************************************************************************/
/*-----------------------------------------------------------------------------------------------[E] 通用全局变量 [E]*/
/*******************************************************************************************************************/






/*******************************************************************************************************************/
/*[S] 通用工具函数 [S]--------------------------------------------------------------------------------------------*/
/*******************************************************************************************************************/

#if DEFINE_IMU_ANALYSIS_MODE > 0
// 逆平方根函数
static float invSqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    union { float f; uint32_t i; } converter;
    converter.f = y;
    converter.i = 0x5f3759df - (converter.i >> 1);
    y = converter.f;
    y = y * (1.5f - (halfx * y * y));
    return y;
}
#endif

#if DEFINE_IMU_ANALYSIS_MODE > 0
// 角度范围限制函数
static float wrap_angle_deg(float angle)
{
    while (angle > 180.0f)
    {
        angle -= 360.0f;
    }
    while (angle < -180.0f)
    {
        angle += 360.0f;
    }
    return angle;
}
#endif

#if DEFINE_IMU_ANALYSIS_MODE == 4 || \
    DEFINE_IMU_ANALYSIS_MODE == 5 || \
    DEFINE_IMU_ANALYSIS_MODE == 6 || \
    DEFINE_IMU_ANALYSIS_MODE == 7
// 陀螺仪数据处理函数
// 输入：陀螺仪原始数据
// 输出：处理后的陀螺仪数据
static void gyro_data_process(float *gx, float *gy, float *gz)
{
    float gx_temp = (float)imu963ra_gyro_x - gyro_cal.offset_x;
    float gy_temp = (float)imu963ra_gyro_y - gyro_cal.offset_y;
    float gz_temp = (float)imu963ra_gyro_z - gyro_cal.offset_z;

    if (gx_temp < 7.0f && gx_temp > -7.0f) gx_temp = 0.0f;
    if (gy_temp < 7.0f && gy_temp > -7.0f) gy_temp = 0.0f;
    if (gz_temp < 7.0f && gz_temp > -7.0f) gz_temp = 0.0f;

    *gx = (float)(gx_temp / 10.0f * 10.0f) * PI / 180.0f;
    *gy = (float)(gy_temp / 10.0f * 10.0f) * PI / 180.0f;
    *gz = (float)(gz_temp / 10.0f * 10.0f) * PI / 180.0f;
}
#endif

/*******************************************************************************************************************/
/*--------------------------------------------------------------------------------------------[E] 通用工具函数 [E]*/
/*******************************************************************************************************************/






/*******************************************************************************************************************/
/*[S] 读取数据 [S]--------------------------------------------------------------------------------------------------*/
/*******************************************************************************************************************/

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     IMU 获取原始数据
// 使用示例     IMU_Update_Data();                                              // 执行该函数后，直接查看对应的变量即可
// 备注信息     定时器定时中断定时调用该函数，获取 IMU963RA 原始数据
//-------------------------------------------------------------------------------------------------------------------
// IMU963RA 原始数据变量
// 加速度计原始值
// imu963ra_acc_x           imu963ra_acc_y          imu963ra_acc_z
// 陀螺仪原始值
// imu963ra_gyro_x          imu963ra_gyro_y         imu963ra_gyro_z
// 磁力计原始值
// imu963ra_mag_x           imu963ra_mag_y          imu963ra_mag_z  
void IMU_Update_Data(void)
{
        imu963ra_get_acc();
        imu963ra_get_gyro();
        imu963ra_get_mag();
}
/*******************************************************************************************************************/
/*--------------------------------------------------------------------------------------------------[E] 读取数据 [E]*/
/*******************************************************************************************************************/







//-------------------------------------------------------------------------------------------------------------------
// 函数简介     应用加速度计校准参数
// 使用示例     IMU_Acc_Apply(&acc_cal, &ax, &ay, &az);
//-------------------------------------------------------------------------------------------------------------------
void IMU_Acc_Apply(float *ax, float *ay, float *az)
{
    *ax = (float)imu963ra_acc_x;
    *ay = (float)imu963ra_acc_y;
    *az = (float)imu963ra_acc_z;
}






/*******************************************************************************************************************/
/*[S] 陀螺仪校准 [S]------------------------------------------------------------------------------------------------*/
/*******************************************************************************************************************/

// 定义并初始化陀螺仪校准结构体
Gyro_Calib_StructDef gyro_cal = {
    .calib_state = GYRO_CALIB_STATE_IDLE, // 初始化为未校准

    .calib_count = 0,
    .offset_x = 0.0f,
    .offset_y = 0.0f,
    .offset_z = 0.0f,
    .sum_x = 0.0f,
    .sum_y = 0.0f,
    .sum_z = 0.0f,
};

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     陀螺仪开始校准
// 使用示例     Gyro_Calib_Start(&gyro_cal);
//-------------------------------------------------------------------------------------------------------------------
void IMU_Gyro_Calib_Start(Gyro_Calib_StructDef *cal)
{
    cal->calib_state = GYRO_CALIB_STATE_RUNNING; // 设置状态为校准中

    cal->calib_count = 0;
    cal->sum_x = 0.0f;
    cal->sum_y = 0.0f;
    cal->sum_z = 0.0f;
    cal->offset_x = 0.0f;
    cal->offset_y = 0.0f;
    cal->offset_z = 0.0f;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     陀螺仪校准状态机
// 返回参数     校准状态：0-未校准，1-校准中，2-已校准
// 使用示例     Gyro_Calib_Check(&gyro_cal);
// 使用要求：
//   - 在静止状态下进行校准，确保设备不受外部运动干扰
//-------------------------------------------------------------------------------------------------------------------
int IMU_Gyro_Calib_Check(Gyro_Calib_StructDef *cal)
{
    if(cal->calib_state == GYRO_CALIB_STATE_DONE)
    {
        return 2; // 校准完
    }

    if(cal->calib_state == GYRO_CALIB_STATE_IDLE)
    {
        return 0; // 未校准
    }
    
    // 那就是状态机还是运行子状态
    // 检查是否允许收集数据
    if(IMU_D_and_A_Enable)
    {
        IMU_Update_Data();
        // 收集数据
        cal->sum_x += (float)imu963ra_gyro_x;
        cal->sum_y += (float)imu963ra_gyro_y;
        cal->sum_z += (float)imu963ra_gyro_z;

        cal->calib_count++;
        IMU_D_and_A_Enable = 0;
        
        // 样本数量达成目标
        if(cal->calib_count >= GYRO_CALIB_TARGET_SAMPLES)
        {
            // 计算陀螺仪浮点数偏移量
            cal->offset_x = (float)cal->sum_x / GYRO_CALIB_TARGET_SAMPLES;
            cal->offset_y = (float)cal->sum_y / GYRO_CALIB_TARGET_SAMPLES;
            cal->offset_z = (float)cal->sum_z / GYRO_CALIB_TARGET_SAMPLES;
            
            // // 转化为整数偏移量
            // gyro_off_int[0] = (int16_t)gyro_off_x;    
            // gyro_off_int[1] = (int16_t)gyro_off_y;
            // gyro_off_int[2] = (int16_t)gyro_off_z;
            
            // 更新陀螺仪校准状态为完成
            cal->calib_state = GYRO_CALIB_STATE_DONE;
            
            printf("GYRO_CAL_OFFSET: X=%.2f, Y=%.2f, Z=%.2f\r\n", cal->offset_x, cal->offset_y, cal->offset_z);
        }
    }
    return 1; // 校准中
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     应用陀螺仪校准参数
// 参数说明     cal             陀螺仪校准结构体指针
// 参数说明     gx              用于存储校准后X轴数据的指针
// 参数说明     gy              用于存储校准后Y轴数据的指针
// 参数说明     gz              用于存储校准后Z轴数据的指针
// 使用示例     float gx, gy, gz;
// 使用示例     IMU_Gyro_Apply(&gyro_cal, &gx, &gy, &gz);
// 备注信息     函数内部直接使用全局变量 imu963ra_gyro_x/y/z 获取原始数据
//              校准完成后，将校准结果存储到传入的指针中
//              未校准时，直接将原始数据存储到传入的指针中
//-------------------------------------------------------------------------------------------------------------------
void IMU_Gyro_Apply(Gyro_Calib_StructDef *cal, float *gx, float *gy, float *gz)
{
    // 如果校准完成，应用校准参数
    if (cal->calib_state == GYRO_CALIB_STATE_DONE)
    {
        *gx = (float)imu963ra_gyro_x - cal->offset_x;
        *gy = (float)imu963ra_gyro_y - cal->offset_y;
        *gz = (float)imu963ra_gyro_z - cal->offset_z;

        if (-7.0f <= *gx && *gx <= 7.0f){*gx = 0.0f;}
        if (-7.0f <= *gy && *gy <= 7.0f){*gy = 0.0f;}
        if (-7.0f <= *gz && *gz <= 7.0f){*gz = 0.0f;}
    }
    // 未校准时，直接将原始数据存储到传入的指针中
    else
    {
        *gx = (float)imu963ra_gyro_x;
        *gy = (float)imu963ra_gyro_y;
        *gz = (float)imu963ra_gyro_z;
    }
}
/*******************************************************************************************************************/
/*------------------------------------------------------------------------------------------------[E] 陀螺仪校准 [E]*/
/*******************************************************************************************************************/






/*******************************************************************************************************************/
/*[S] 磁力计校准 [S]------------------------------------------------------------------------------------------------*/
/*******************************************************************************************************************/

// 磁力计校准结构体变量
Mag_Calib_StructDef mag_cal = {
    .calib_state = MAG_CALIB_STATE_IDLE,
    .calib_count = 0,
    .offset_x = 0.0f,    // X轴偏移
    .offset_y = 0.0f,     // Y轴偏移
    .offset_z = 0.0f,     // Z轴偏移
    .scale_x = 1.0f,   // X轴缩放
    .scale_y = 1.0f,   // Y轴缩放
    .scale_z = 1.0f,   // Z轴缩放
    .max_x = -32768,
    .min_x = 32767,
    .max_y = -32768,
    .min_y = 32767,
    .max_z = -32768,
    .min_z = 32767,
};

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     磁力计校准初始化
// 使用示例     Mag_Calib_Start(&mag_cal);
//-------------------------------------------------------------------------------------------------------------------
void IMU_Mag_Calib_Start(Mag_Calib_StructDef *cal)
{
    // 校准状态置为0，表示需要进行校准
    cal->calib_state = MAG_CALIB_STATE_RUNNING;
    cal->calib_count = 0;  
    // 硬铁偏移（中心点）
    cal->offset_x = 0.0f;     // X轴偏移
    cal->offset_y = 0.0f;     // Y轴偏移
    cal->offset_z = 0.0f;     // Z轴偏移    
    // 软铁缩放因子
    cal->scale_x = 1.0f;   // X轴缩放
    cal->scale_y = 1.0f;   // Y轴缩放
    cal->scale_z = 1.0f;   // Z轴缩放
    // 初始化最大最小值为当前测量范围
    cal->max_x = -32768;
    cal->min_x = 32767;
    cal->max_y = -32768;
    cal->min_y = 32767;
    cal->max_z = -32768;
    cal->min_z = 32767;
}

#if MAG_CALIB_METHOD == 1 // 应用Min-Max法
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     磁力计校准状态机(Min-Max法)
// 返回参数     校准状态：0-未校准，1-校准中，2-校准完
// 使用示例     Mag_Calib_Check(&mag_cal);
// 算法原理：
//   1. 采集各个方向的磁场数据
//   2. 找到每个轴的最大值和最小值
//   3. 硬铁偏移 = (最大值 + 最小值) / 2
//   4. 软铁缩放 = 最大半量程 / 当前轴半量程
// 
// 优点：
//   - 计算简单，速度快
//   - 内存占用小
// 缺点：
//   - 精度相对较低
//   - 对旋转的均匀性要求较高
// 
// 使用要求：
//   - 找一个远离强磁场干扰的环境（避开电机、磁铁、扬声器等）
//   - 保持设备稳定，避免剧烈晃动
//   - 缓慢旋转设备，确保覆盖以下所有方向：
//     1. 水平旋转：绕垂直轴360度旋转
//     2. 俯仰旋转：前端上下摆动，覆盖-90°到+90°
//     3. 横滚旋转：左右倾斜，覆盖-90°到+90°
//     4. 斜向旋转：进行一些对角线方向的旋转
//   - 旋转过程持续约20秒，确保采集足够的数据
//   - 校准完成后，会自动计算并输出校准参数
//-------------------------------------------------------------------------------------------------------------------
int IMU_Mag_Calib_Check(Mag_Calib_StructDef *cal)
{
    if(cal->calib_state == MAG_CALIB_STATE_DONE)
    {
        return 2; // 校准完
    }

    if(cal->calib_state == MAG_CALIB_STATE_IDLE)
    {
        return 0; // 未校准
    }

    // 那就是状态机还是运行子状态
    // 检查是否允许收集数据
    if (IMU_D_and_A_Enable)
    {
        IMU_Update_Data();

        // 更新最大最小值
        if (imu963ra_mag_x > cal->max_x) cal->max_x = imu963ra_mag_x;
        if (imu963ra_mag_x < cal->min_x) cal->min_x = imu963ra_mag_x;
        if (imu963ra_mag_y > cal->max_y) cal->max_y = imu963ra_mag_y;
        if (imu963ra_mag_y < cal->min_y) cal->min_y = imu963ra_mag_y;
        if (imu963ra_mag_z > cal->max_z) cal->max_z = imu963ra_mag_z;
        if (imu963ra_mag_z < cal->min_z) cal->min_z = imu963ra_mag_z;

        cal->calib_count++;
        IMU_D_and_A_Enable = 0;

        // 样本数量达成目标
        if (cal->calib_count >= MAG_CALIB_MINMAX_TARGET_SAMPLES)
        {
            // 计算硬铁偏移（椭球中心）
            cal->offset_x = (float)(cal->max_x + cal->min_x) / 2.0f;
            cal->offset_y = (float)(cal->max_y + cal->min_y) / 2.0f;
            cal->offset_z = (float)(cal->max_z + cal->min_z) / 2.0f;

            // 计算软铁缩放因子
            float half_range_x = (float)(cal->max_x - cal->min_x) / 2.0f;   
            float half_range_y = (float)(cal->max_y - cal->min_y) / 2.0f;
            float half_range_z = (float)(cal->max_z - cal->min_z) / 2.0f;
            
            float max_half_range = half_range_x;
            if (half_range_y > max_half_range) max_half_range = half_range_y;
            if (half_range_z > max_half_range) max_half_range = half_range_z;
            
            if (half_range_x > 1.0f) cal->scale_x = max_half_range / half_range_x;
            if (half_range_y > 1.0f) cal->scale_y = max_half_range / half_range_y;
            if (half_range_z > 1.0f) cal->scale_z = max_half_range / half_range_z;

            cal->calib_state = MAG_CALIB_STATE_DONE;
            printf("MAG_CAL_OFFSET: X=%.2f, Y=%.2f, Z=%.2f\r\n", cal->offset_x, cal->offset_y, cal->offset_z);
            printf("MAG_CAL_SCALE: X=%.2f, Y=%.2f, Z=%.2f\r\n", cal->scale_x, cal->scale_y, cal->scale_z);
        }
    }

    return 1; // 校准中
}
#endif

#if MAG_CALIB_METHOD == 2 // 应用椭球拟合法
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     磁力计校准状态机(椭球拟合法)
// 返回参数     校准状态：0-未校准，1-校准中，2-校准完
// 使用示例     Mag_Calib_Check(&mag_cal);
// 算法原理：
//   1. 采集3000个磁场数据点
//   2. 使用最小二乘法拟合椭球模型
//   3. 通过高斯消元法求解线性方程组
//   4. 计算椭球中心（硬铁偏移）和缩放因子（软铁校正）
// 
// 优点：
//   - 精度高，能处理复杂干扰
//   - 适应性强
// 缺点：
//   - 计算复杂
//   - 内存占用大（需要存储3000个数据点）
// 
// 容错机制：
//   - 当拟合失败时，自动切换到Min-Max法
// 使用要求：
//   - 找一个远离强磁场干扰的环境（避开电机、磁铁、扬声器等）
//   - 保持设备稳定，避免剧烈晃动
//   - 缓慢、均匀地旋转设备，确保覆盖所有空间方向：
//     - 水平方向：顺时针和逆时针各旋转360°
//     - 垂直方向：上下俯仰，从底部朝上到顶部朝上
//     - 倾斜方向：左右横滚，从左侧朝上到右侧朝上
//     - 斜向方向：进行一些复合角度的旋转
//   - 旋转过程持续约15-20秒，确保采集3000个数据点
//   - 如果拟合失败，会自动切换到Min-Max法作为备用
//   - 校准完成后，会自动计算并输出校准参数
//-------------------------------------------------------------------------------------------------------------------
int IMU_Mag_Calib_Check(Mag_Calib_StructDef *cal)
{
    if(cal->calib_state == MAG_CALIB_STATE_DONE)
    {
        return 2; // 校准完
    }

    if(cal->calib_state == MAG_CALIB_STATE_IDLE)
    {
        return 0; // 未校准
    }

    // 那就是状态机还是运行子状态
    // 检查是否允许收集数据
    if (IMU_D_and_A_Enable)
    {
        IMU_Update_Data();

        // 存储采集的磁力计数据
        static int16_t ellipsoid_mag_x_buf[3000];
        static int16_t ellipsoid_mag_y_buf[3000];
        static int16_t ellipsoid_mag_z_buf[3000];
        static int16_t max_x = -32768, min_x = 32767;
        static int16_t max_y = -32768, min_y = 32767;
        static int16_t max_z = -32768, min_z = 32767;

        // 记录最大值和最小值
        if (imu963ra_mag_x > max_x) max_x = imu963ra_mag_x;
        if (imu963ra_mag_x < min_x) min_x = imu963ra_mag_x;
        if (imu963ra_mag_y > max_y) max_y = imu963ra_mag_y;
        if (imu963ra_mag_y < min_y) min_y = imu963ra_mag_y;
        if (imu963ra_mag_z > max_z) max_z = imu963ra_mag_z;
        if (imu963ra_mag_z < min_z) min_z = imu963ra_mag_z;

        // 存储数据点
        if (cal->calib_count < 3000)
        {
            ellipsoid_mag_x_buf[cal->calib_count] = imu963ra_mag_x;
            ellipsoid_mag_y_buf[cal->calib_count] = imu963ra_mag_y;
            ellipsoid_mag_z_buf[cal->calib_count] = imu963ra_mag_z;
            cal->calib_count++;
        }

        IMU_D_and_A_Enable = 0;

        // 采满固定数量的点后执行椭球拟合
        if (cal->calib_count >= 3000)
        {
            // 更新结构体中的最大最小值
            cal->max_x = max_x;
            cal->min_x = min_x;
            cal->max_y = max_y;
            cal->min_y = min_y;
            cal->max_z = max_z;
            cal->min_z = min_z;

            // 执行椭球拟合
            double m_matrix[6][6 + 1];
            double solve[6];

            // 初始化矩阵
            memset(m_matrix, 0, sizeof(m_matrix));
            memset(solve, 0, sizeof(solve));

            // 构建矩阵
            for (uint16_t i = 0; i < cal->calib_count; i++)
            {
                double x = (double)ellipsoid_mag_x_buf[i];
                double y = (double)ellipsoid_mag_y_buf[i];
                double z = (double)ellipsoid_mag_z_buf[i];
                double V[7];

                V[0] = y * y;
                V[1] = z * z;
                V[2] = x;
                V[3] = y;
                V[4] = z;
                V[5] = 1.0;
                V[6] = -x * x;

                for (uint8_t row = 0; row < 6; row++)
                {
                    for (uint8_t col = 0; col < 6 + 1; col++)
                    {
                        m_matrix[row][col] += V[row] * V[col];
                    }
                }
            }

            // 归一化矩阵
            for (uint8_t row = 0; row < 6; row++)
            {
                for (uint8_t col = 0; col < 6 + 1; col++)
                {
                    m_matrix[row][col] /= (double)cal->calib_count;
                }
            }

            // 高斯消元法求解
            for (uint8_t k = 0; k < 6; k++)
            {
                // 寻找主元素
                uint8_t max_row = k;
                for (uint8_t i = k + 1; i < 6; i++)
                {
                    if (fabs(m_matrix[i][k]) > fabs(m_matrix[max_row][k]))
                    {
                        max_row = i;
                    }
                }

                // 交换行
                if (max_row != k)
                {
                    for (uint8_t j = 0; j <= 6; j++)
                    {
                        double tmp = m_matrix[k][j];
                        m_matrix[k][j] = m_matrix[max_row][j];
                        m_matrix[max_row][j] = tmp;
                    }
                }

                // 检查主元素是否为零
                if (fabs(m_matrix[k][k]) < 1e-10)
                {
                    // 拟合失败，使用Min-Max法作为备用
                    cal->offset_x = (float)(cal->max_x + cal->min_x) / 2.0f;
                    cal->offset_y = (float)(cal->max_y + cal->min_y) / 2.0f;
                    cal->offset_z = (float)(cal->max_z + cal->min_z) / 2.0f;

                    float half_range_x = (float)(cal->max_x - cal->min_x) / 2.0f;
                    float half_range_y = (float)(cal->max_y - cal->min_y) / 2.0f;
                    float half_range_z = (float)(cal->max_z - cal->min_z) / 2.0f;
                    
                    float max_half_range = half_range_x;
                    if (half_range_y > max_half_range) max_half_range = half_range_y;
                    if (half_range_z > max_half_range) max_half_range = half_range_z;
                    
                    if (half_range_x > 1.0f) cal->scale_x = max_half_range / half_range_x;
                    if (half_range_y > 1.0f) cal->scale_y = max_half_range / half_range_y;
                    if (half_range_z > 1.0f) cal->scale_z = max_half_range / half_range_z;
                    
                    cal->calib_state = MAG_CALIB_STATE_DONE;
                    printf("MAG_CAL_OFFSET: X=%.2f, Y=%.2f, Z=%.2f\r\n", cal->offset_x, cal->offset_y, cal->offset_z);
                    printf("MAG_CAL_SCALE: X=%.2f, Y=%.2f, Z=%.2f\r\n", cal->scale_x, cal->scale_y, cal->scale_z);
                    return 2;
                }

                // 消元
                for (uint8_t i = k + 1; i < 6; i++)
                {
                    double factor = m_matrix[i][k] / m_matrix[k][k];
                    for (uint8_t j = k; j <= 6; j++)
                    {
                        m_matrix[i][j] -= factor * m_matrix[k][j];
                    }
                }
            }

            // 回代求解
            for (int8_t i = 5; i >= 0; i--)
            {
                double sum = 0.0;
                for (uint8_t j = i + 1; j < 6; j++)
                {
                    sum += m_matrix[i][j] * solve[j];
                }
                solve[i] = (m_matrix[i][6] - sum) / m_matrix[i][i];
            }

            // 计算校准参数
            double a = solve[0];
            double b = solve[1];
            double c = solve[2];
            double d = solve[3];
            double e = solve[4];
            double f = solve[5];

            double X0 = -c / 2.0;
            double Y0 = -d / (2.0 * a);
            double Z0 = -e / (2.0 * b);
            double temp = X0 * X0 + a * Y0 * Y0 + b * Z0 * Z0 - f;

            if (temp > 0.0)
            {
                double A = sqrt(temp);
                double B = A / sqrt(a);
                double C = A / sqrt(b);

                cal->offset_x = (float)X0;
                cal->offset_y = (float)Y0;
                cal->offset_z = (float)Z0;

                cal->scale_x = (float)(1.0 / A);
                cal->scale_y = (float)(1.0 / B);
                cal->scale_z = (float)(1.0 / C);
            }
            else
            {
                // 拟合失败，使用Min-Max法作为备用
                cal->offset_x = (float)(cal->max_x + cal->min_x) / 2.0f;
                cal->offset_y = (float)(cal->max_y + cal->min_y) / 2.0f;
                cal->offset_z = (float)(cal->max_z + cal->min_z) / 2.0f;

                float half_range_x = (float)(cal->max_x - cal->min_x) / 2.0f;
                float half_range_y = (float)(cal->max_y - cal->min_y) / 2.0f;
                float half_range_z = (float)(cal->max_z - cal->min_z) / 2.0f;
                
                float max_half_range = half_range_x;
                if (half_range_y > max_half_range) max_half_range = half_range_y;
                if (half_range_z > max_half_range) max_half_range = half_range_z;
                
                if (half_range_x > 1.0f) cal->scale_x = max_half_range / half_range_x;
                if (half_range_y > 1.0f) cal->scale_y = max_half_range / half_range_y;
                if (half_range_z > 1.0f) cal->scale_z = max_half_range / half_range_z;
            }

            cal->calib_state = MAG_CALIB_STATE_DONE;
            printf("MAG_CAL_OFFSET: X=%.2f, Y=%.2f, Z=%.2f\r\n", cal->offset_x, cal->offset_y, cal->offset_z);
            printf("MAG_CAL_SCALE: X=%.6f, Y=%.6f, Z=%.6f\r\n", cal->scale_x, cal->scale_y, cal->scale_z);
            return 2;
        }
    }
    return 1; // 校准进行中
}
#endif

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     应用磁力计校准参数
// 参数说明     cal             磁力计校准结构体指针
// 参数说明     mx              用于存储校准后X轴数据的指针
// 参数说明     my              用于存储校准后Y轴数据的指针
// 参数说明     mz              用于存储校准后Z轴数据的指针
// 使用示例     int16_t mx, my, mz;
// 使用示例     IMU_Mag_Apply(&mag_cal, &mx, &my, &mz);
// 备注信息     函数内部直接使用全局变量 imu963ra_mag_x/y/z 获取原始数据
//              校准完成后，将校准结果存储到传入的指针中
//              未校准时，直接将原始数据存储到传入的指针中
//-------------------------------------------------------------------------------------------------------------------
void IMU_Mag_Apply(Mag_Calib_StructDef *cal, int16_t *mx, int16_t *my, int16_t *mz)
{
    // 如果校准完成，应用校准参数
    if (cal->calib_state == MAG_CALIB_STATE_DONE)
    {
        *mx = (int16_t)(((float)imu963ra_mag_x - cal->offset_x) * cal->scale_x);
        *my = (int16_t)(((float)imu963ra_mag_y - cal->offset_y) * cal->scale_y);
        *mz = (int16_t)(((float)imu963ra_mag_z - cal->offset_z) * cal->scale_z);
    }
    // 未校准时，直接将原始数据存储到传入的指针中
    else
    {
        *mx = (int16_t)imu963ra_mag_x;
        *my = (int16_t)imu963ra_mag_y;
        *mz = (int16_t)imu963ra_mag_z;
    }
}
/*******************************************************************************************************************/
/*------------------------------------------------------------------------------------------------[E] 磁力计校准 [E]*/
/*******************************************************************************************************************/







#if DEFINE_IMU_ANALYSIS_MODE == 1      
/*******************************************************************************************************************/
/*-[S] 三轴姿态解算 [S]----------------------------------------------------------------------------------------------*/
/*******************************************************************************************************************/

// 三轴姿态解算变量
ThreeAxis_StructDef three_axis = {
    .q = {1.0f, 0.0f, 0.0f, 0.0f}
};

static void ThreeAxis_Update(void)
{
    float gx, gy, gz;
    float q0, q1, q2, q3;
    float halfT;

    IMU_Gyro_Apply(&gyro_cal, &gx, &gy, &gz);

    gx = gx * PI / 180.0f;
    gy = gy * PI / 180.0f;
    gz = gz * PI / 180.0f;

    q0 = three_axis.q.q0;
    q1 = three_axis.q.q1;
    q2 = three_axis.q.q2;
    q3 = three_axis.q.q3;

    halfT = 0.5f * DELTA_T_3AXIS;

    q0 += (-q1 * gx - q2 * gy - q3 * gz) * halfT;
    q1 += ( q0 * gx + q2 * gz - q3 * gy) * halfT;
    q2 += ( q0 * gy - q1 * gz + q3 * gx) * halfT;
    q3 += ( q0 * gz + q1 * gy - q2 * gx) * halfT;

    float norm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    three_axis.q.q0 = q0 * norm;
    three_axis.q.q1 = q1 * norm;
    three_axis.q.q2 = q2 * norm;
    three_axis.q.q3 = q3 * norm;

    Yaw_Result = atan2f(2.0f * (q1 * q2 + q0 * q3), -2.0f * q2 * q2 - 2.0f * q3 * q3 + 1.0f) * 180.0f / PI;
    Roll_Result = 0.0f;
    Pitch_Result = 0.0f;
}

/*******************************************************************************************************************/
/*-----------------------------------------------------------------------------------------------[E] 三轴姿态解算 [E]*/
/*******************************************************************************************************************/
#endif

#if DEFINE_IMU_ANALYSIS_MODE == 2
/*******************************************************************************************************************/
/*-[S] 六轴姿态解算 [S]----------------------------------------------------------------------------------------------*/
/*******************************************************************************************************************/

// 六轴姿态解算变量
Mahony_AHRS_StructDef six_axis = {
    .q = {1.0f, 0.0f, 0.0f, 0.0f},
    .Kp = 0.0001f,
    .Ki = -0.0000324f,
    .quick_Kp = 10.0f,
    .quick_Ki = 0.005f,
    .exInt = 0.0f,
    .eyInt = 0.0f,
    .ezInt = 0.0f,
    .quick_mode = 0
};

// 加速度计滤波
static float ax_filtered = 0.0f, ay_filtered = 0.0f, az_filtered = 0.0f;
static bool acc_filter_first_run = true;
const float IMU_ALPHA = 0.3f;  // 滤波系数
// Yaw角滤波
static float last_raw_yaw = 0.0f;
static float accumulated_yaw_drift = 0.0f;
static bool yaw_filter_first_run = true;
const float YAW_DEADZONE_THRESHOLD = 2.0f;  // Yaw死区阈值

static void SixAxis_Update(void)
{
    float gx, gy, gz;
    float ax, ay, az;
    float q0, q1, q2, q3;
    float halfT;
    float norm;
    float vx, vy, vz;
    float ex, ey, ez;
    float Kp, Ki;

    IMU_Acc_Apply(&ax, &ay, &az);
    // 一阶低通滤波
    if (acc_filter_first_run) {
        ax_filtered = ax;
        ay_filtered = ay;
        az_filtered = az;
        acc_filter_first_run = false;
    } else {
        ax_filtered = ax * IMU_ALPHA + ax_filtered * (1.0f - IMU_ALPHA);
        ay_filtered = ay * IMU_ALPHA + ay_filtered * (1.0f - IMU_ALPHA);
        az_filtered = az * IMU_ALPHA + az_filtered * (1.0f - IMU_ALPHA);
    }

    // 使用滤波后的数据
    ax = ax_filtered;
    ay = ay_filtered;
    az = az_filtered;

    IMU_Gyro_Apply(&gyro_cal, &gx, &gy, &gz);

    gx = gx * PI / 180.0f;
    gy = gy * PI / 180.0f;
    gz = gz * PI / 180.0f;

    q0 = six_axis.q.q0;
    q1 = six_axis.q.q1;
    q2 = six_axis.q.q2;
    q3 = six_axis.q.q3;

    Kp = six_axis.Kp;
    Ki = six_axis.Ki;

    norm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= norm;
    ay *= norm;
    az *= norm;

    vx = 2.0f * (q1 * q3 - q0 * q2);
    vy = 2.0f * (q0 * q1 + q2 * q3);
    vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

    ex = (ay * vz - az * vy);
    ey = (az * vx - ax * vz);
    ez = (ax * vy - ay * vx);

    six_axis.exInt += ex * halfT;
    six_axis.eyInt += ey * halfT;
    six_axis.ezInt += ez * halfT;

    gx += Kp * ex + Ki * six_axis.exInt;
    gy += Kp * ey + Ki * six_axis.eyInt;
    gz += Kp * ez + Ki * six_axis.ezInt;

    halfT = 0.5f * DELTA_T_6AXIS;

    q0 += (-q1 * gx - q2 * gy - q3 * gz) * halfT;
    q1 += ( q0 * gx + q2 * gz - q3 * gy) * halfT;
    q2 += ( q0 * gy - q1 * gz + q3 * gx) * halfT;
    q3 += ( q0 * gz + q1 * gy - q2 * gx) * halfT;

    norm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    six_axis.q.q0 = q0 * norm;
    six_axis.q.q1 = q1 * norm;
    six_axis.q.q2 = q2 * norm;
    six_axis.q.q3 = q3 * norm;

    Roll_Result = atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2)) * 180.0f / PI;
    Pitch_Result = asinf(2.0f * (q0 * q2 - q3 * q1)) * 180.0f / PI;
//    Yaw_Result = atan2f(2.0f * (q1 * q2 + q0 * q3), -2.0f * q2 * q2 - 2.0f * q3 * q3 + 1.0f) * 180.0f / PI;
    // 计算原始Yaw角
    float raw_yaw = atan2f(2.0f * (q1 * q2 + q0 * q3), -2.0f * q2 * q2 - 2.0f * q3 * q3 + 1.0f) * 180.0f / PI;
    if (yaw_filter_first_run) {
    last_raw_yaw = raw_yaw;
    yaw_filter_first_run = false;   
    }
    // 计算两次Yaw角之间的变化值
    float diff = raw_yaw - last_raw_yaw;

    // 处理角度越界跳变
    if (diff > 180.0f)       diff -= 360.0f;
    else if (diff < -180.0f) diff += 360.0f;

    // 死区滤波处理
    if (fabsf(diff) < YAW_DEADZONE_THRESHOLD) {
        accumulated_yaw_drift += diff;
    }

    last_raw_yaw = raw_yaw;

    // 减去累积的死区误差
    float filtered_yaw = raw_yaw - accumulated_yaw_drift;

    // 归一化到 [-180, 180]
    while (filtered_yaw > 180.0f)  filtered_yaw -= 360.0f;
    while (filtered_yaw < -180.0f) filtered_yaw += 360.0f;

    Yaw_Result = filtered_yaw;
}

/*******************************************************************************************************************/
/*-----------------------------------------------------------------------------------------------[E] 六轴姿态解算 [E]*/
/*******************************************************************************************************************/
#endif

#if DEFINE_IMU_ANALYSIS_MODE == 3
/*******************************************************************************************************************/
/*-[S] 九轴姿态解算 [S]----------------------------------------------------------------------------------------------*/
/*******************************************************************************************************************/

// 九轴姿态解算变量
NineAxis_StructDef nine_axis = {
    .mahony = {
        .q = {1.0f, 0.0f, 0.0f, 0.0f},
        .Kp = 2.0f,
        .Ki = 0.005f,
        .quick_Kp = 10.0f,
        .quick_Ki = 0.005f,
        .exInt = 0.0f,
        .eyInt = 0.0f,
        .ezInt = 0.0f,
        .quick_mode = 1
    },
    .mag_declination = 0.0f
};

static void NineAxis_Update(void)
{
    float gx, gy, gz;
    float ax, ay, az;
    int16_t mx_raw, my_raw, mz_raw;
    float mx, my, mz;
    float q0, q1, q2, q3;
    float halfT;
    float norm;
    float vx, vy, vz;
    float wx, wy, wz;
    float hx, hy, bx, bz;
    float ex, ey, ez;
    float Kp, Ki;

    IMU_Acc_Apply(&ax, &ay, &az);
    IMU_Gyro_Apply(&gyro_cal, &gx, &gy, &gz);
    IMU_Mag_Apply(&mag_cal, &mx_raw, &my_raw, &mz_raw);

    gx = gx * PI / 180.0f;
    gy = gy * PI / 180.0f;
    gz = gz * PI / 180.0f;

    mx = (float)mx_raw;
    my = (float)my_raw;
    mz = (float)mz_raw;

    q0 = nine_axis.mahony.q.q0;
    q1 = nine_axis.mahony.q.q1;
    q2 = nine_axis.mahony.q.q2;
    q3 = nine_axis.mahony.q.q3;

    Kp = nine_axis.mahony.quick_mode ? nine_axis.mahony.quick_Kp : nine_axis.mahony.Kp;
    Ki = nine_axis.mahony.quick_mode ? nine_axis.mahony.quick_Ki : nine_axis.mahony.Ki;

    norm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= norm;
    ay *= norm;
    az *= norm;

    norm = invSqrt(mx * mx + my * my + mz * mz);
    mx *= norm;
    my *= norm;
    mz *= norm;

    float q0q0 = q0 * q0;
    float q0q1 = q0 * q1;
    float q0q2 = q0 * q2;
    float q0q3 = q0 * q3;
    float q1q1 = q1 * q1;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q3q3 = q3 * q3;

    vx = 2.0f * (q1q3 - q0q2);
    vy = 2.0f * (q0q1 + q2q3);
    vz = q0q0 - q1q1 - q2q2 + q3q3;

    hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
    hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
    bx = sqrtf(hx * hx + hy * hy);
    bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

    wx = 2.0f * bx * (0.5f - q2q2 - q3q3) + 2.0f * bz * (q1q3 - q0q2);
    wy = 2.0f * bx * (q1q2 - q0q3) + 2.0f * bz * (q0q1 + q2q3);
    wz = 2.0f * bx * (q0q2 + q1q3) + 2.0f * bz * (0.5f - q1q1 - q2q2);

    ex = (ay * vz - az * vy) + (my * wz - mz * wy);
    ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
    ez = (ax * vy - ay * vx) + (mx * wy - my * wx);

    if (Ki > 0.0f)
    {
        nine_axis.mahony.exInt += ex * Ki;
        nine_axis.mahony.eyInt += ey * Ki;
        nine_axis.mahony.ezInt += ez * Ki;
    }
    else
    {
        nine_axis.mahony.exInt = 0.0f;
        nine_axis.mahony.eyInt = 0.0f;
        nine_axis.mahony.ezInt = 0.0f;
    }

    gx += Kp * ex + nine_axis.mahony.exInt;
    gy += Kp * ey + nine_axis.mahony.eyInt;
    gz += Kp * ez + nine_axis.mahony.ezInt;

    halfT = 0.5f * DELTA_T_9AXIS;

    q0 += (-q1 * gx - q2 * gy - q3 * gz) * halfT;
    q1 += ( q0 * gx + q2 * gz - q3 * gy) * halfT;
    q2 += ( q0 * gy - q1 * gz + q3 * gx) * halfT;
    q3 += ( q0 * gz + q1 * gy - q2 * gx) * halfT;

    norm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    nine_axis.mahony.q.q0 = q0 * norm;
    nine_axis.mahony.q.q1 = q1 * norm;
    nine_axis.mahony.q.q2 = q2 * norm;
    nine_axis.mahony.q.q3 = q3 * norm;

    Roll_Result = atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2)) * 180.0f / PI;
    Pitch_Result = asinf(2.0f * (q0 * q2 - q3 * q1)) * 180.0f / PI;
    Yaw_Result = atan2f(2.0f * (q1 * q2 + q0 * q3), -2.0f * q2 * q2 - 2.0f * q3 * q3 + 1.0f) * 180.0f / PI;
}

/*******************************************************************************************************************/
/*-----------------------------------------------------------------------------------------------[E] 九轴姿态解算 [E]*/
/*******************************************************************************************************************/
#endif


/*******************************************************************************************************************/
/*[S] 仅Yaw输出姿态解算 [S]------------------------------------------------------------------------------------------*/
/*******************************************************************************************************************/

// Yaw-only算法变量
YawOnly_UnionDef yaw_only = {
    #if DEFINE_IMU_ANALYSIS_MODE == 4
    .mag_get_yaw = {
        .mag_declination = 0.0f,
        .yaw_filter_alpha = 0.3f,
        .yaw_filtered = 0.0f
    }
    #elif DEFINE_IMU_ANALYSIS_MODE == 5
    .mahony = {
        .q = {1.0f, 0.0f, 0.0f, 0.0f},
        .Kp = 2.0f,
        .Ki = 0.005f,
        .quick_Kp = 10.0f,
        .quick_Ki = 0.005f,
        .exInt = 0.0f,
        .eyInt = 0.0f,
        .ezInt = 0.0f,
        .quick_mode = 1
    }
    #elif DEFINE_IMU_ANALYSIS_MODE == 6
    .madgwick = {
        .q = {1.0f, 0.0f, 0.0f, 0.0f},
        .beta = 1.2f,
        .quick_beta = 10.0f,
        .invSampleFreq = 0.001f,
        .mag_declination = 0.0f
    }
    #elif DEFINE_IMU_ANALYSIS_MODE == 7
    .tilt_mag_yaw = {
        .yaw = 0.0f,
        .yaw_filtered = 0.0f,
        .yaw_error_int = 0.0f,
        .kp = 0.08f,
        .ki = 0.0025f,
        .quick_kp = 0.03f,
        .quick_ki = 0.0010f,
        .yaw_filter_alpha = 0.3f,
        .mag_declination = 0.0f
    }
    #endif
};

#if DEFINE_IMU_ANALYSIS_MODE == 4
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     Mag_Get_Yaw算法（仅磁力计+倾斜补偿）
//-------------------------------------------------------------------------------------------------------------------
static float Mag_Get_Yaw_Update(void)
{
    int16_t mag_x = 0, mag_y = 0, mag_z = 0;
    float ax, ay, az;

    IMU_Mag_Apply(&mag_cal, &mag_x, &mag_y, &mag_z);   
    IMU_Acc_Apply(&ax, &ay, &az);

    float mx = (float)mag_x;
    float my = (float)mag_y;
    float mz = (float)mag_z;

    float acc_norm = sqrtf(ax * ax + ay * ay + az * az);
    if (acc_norm < 1e-6f) return yaw_only.mag_get_yaw.yaw_filtered;
    ax /= acc_norm;
    ay /= acc_norm;
    az /= acc_norm;

    float mag_norm = sqrtf(mx * mx + my * my + mz * mz);
    if (mag_norm < 1e-6f) return yaw_only.mag_get_yaw.yaw_filtered;
    mx /= mag_norm;
    my /= mag_norm;
    mz /= mag_norm;

    float roll = atan2f(ay, az);
    float pitch = atan2f(-ax, sqrtf(ay * ay + az * az));

    float cr = cosf(0.5f * roll);
    float sr = sinf(0.5f * roll);
    float cp = cosf(0.5f * pitch);
    float sp = sinf(0.5f * pitch);

    float q0 = cr * cp;
    float q1 = sr * cp;
    float q2 = cr * sp;
    float q3 = -sr * sp;

    float r11 = 1.0f - 2.0f * (q2 * q2 + q3 * q3);
    float r12 = 2.0f * (q1 * q2 - q0 * q3);
    float r13 = 2.0f * (q1 * q3 + q0 * q2);
    float r21 = 2.0f * (q1 * q2 + q0 * q3);
    float r22 = 1.0f - 2.0f * (q1 * q1 + q3 * q3);
    float r23 = 2.0f * (q2 * q3 - q0 * q1);

    float mx_h = r11 * mx + r12 * my + r13 * mz;
    float my_h = r21 * mx + r22 * my + r23 * mz;

    float mag_h_norm = sqrtf(mx_h * mx_h + my_h * my_h);
    if (mag_h_norm < 1e-6f) return yaw_only.mag_get_yaw.yaw_filtered;

    mx_h /= mag_h_norm;
    my_h /= mag_h_norm;

    float yaw_rad = atan2f(my_h, mx_h);
    float yaw_deg = yaw_rad * 180.0f / PI;

    yaw_deg += yaw_only.mag_get_yaw.mag_declination;

    float yaw_mag = -yaw_deg;
    while (yaw_mag > 180.0f) yaw_mag -= 360.0f;
    while (yaw_mag < -180.0f) yaw_mag += 360.0f;

    return -yaw_mag;
}
#endif

#if DEFINE_IMU_ANALYSIS_MODE == 5
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     Mahony AHRS Yaw-only算法
//-------------------------------------------------------------------------------------------------------------------
static float Mahony_YawOnly_Update(float dt)
{
    int16_t mag_x = 0;
    int16_t mag_y = 0;
    int16_t mag_z = 0;

    float gx, gy, gz;
    float ax, ay, az;
    float recipNorm;

    float q0 = yaw_only.mahony.q.q0;
    float q1 = yaw_only.mahony.q.q1;
    float q2 = yaw_only.mahony.q.q2;
    float q3 = yaw_only.mahony.q.q3;

    float q0q0, q0q1, q0q2, q0q3;
    float q1q1, q1q2, q1q3;
    float q2q2, q2q3;
    float q3q3;

    float vx, vy, vz;
    float wx, wy, wz;
    float hx, hy, bx, bz;
    float ex, ey, ez;
    float halfT;
    float yaw_deg;
    float kp;
    float ki;

    if (dt <= 1e-6f)
    {
        dt = 0.01f;
    }

    kp = imu_stable ? yaw_only.mahony.Kp : yaw_only.mahony.quick_Kp;
    ki = imu_stable ? yaw_only.mahony.Ki : yaw_only.mahony.quick_Ki;

    IMU_Mag_Apply(&mag_cal, &mag_x, &mag_y, &mag_z);
    IMU_Acc_Apply(&ax, &ay, &az);  
    gyro_data_process(&gx, &gy, &gz);
    float mx = (float)mag_x;
    float my = (float)mag_y;
    float mz = (float)mag_z;

    float declination_rad = yaw_only.mag_get_yaw.mag_declination * PI / 180.0f;
    float mx_raw = mx;
    float my_raw = my;
    mx = mx_raw * cosf(declination_rad) - my_raw * sinf(declination_rad);
    my = mx_raw * sinf(declination_rad) + my_raw * cosf(declination_rad);

    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    if (recipNorm < 1e-6f)
    {
        return Yaw_Result;
    }
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    recipNorm = invSqrt(mx * mx + my * my + mz * mz);
    if (recipNorm < 1e-6f)
    {
        return Yaw_Result;
    }
    mx *= recipNorm;
    my *= recipNorm;
    mz *= recipNorm;

    q0q0 = q0 * q0;
    q0q1 = q0 * q1;
    q0q2 = q0 * q2;
    q0q3 = q0 * q3;
    q1q1 = q1 * q1;
    q1q2 = q1 * q2;
    q1q3 = q1 * q3;
    q2q2 = q2 * q2;
    q2q3 = q2 * q3;
    q3q3 = q3 * q3;

    vx = 2.0f * (q1q3 - q0q2);
    vy = 2.0f * (q0q1 + q2q3);
    vz = q0q0 - q1q1 - q2q2 + q3q3;

    hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
    hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
    bx = sqrtf(hx * hx + hy * hy);
    bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

    wx = 2.0f * bx * (0.5f - q2q2 - q3q3) + 2.0f * bz * (q1q3 - q0q2);
    wy = 2.0f * bx * (q1q2 - q0q3) + 2.0f * bz * (q0q1 + q2q3);
    wz = 2.0f * bx * (q0q2 + q1q3) + 2.0f * bz * (0.5f - q1q1 - q2q2);

    ex = (ay * vz - az * vy) + (my * wz - mz * wy);
    ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
    ez = (ax * vy - ay * vx) + (mx * wy - my * wx);

    if (ki > 0.0f)
    {
        yaw_only.mahony.exInt += ex * ki * dt;
        yaw_only.mahony.eyInt += ey * ki * dt;
        yaw_only.mahony.ezInt += ez * ki * dt;
    }
    else
    {
        yaw_only.mahony.exInt = 0.0f;
        yaw_only.mahony.eyInt = 0.0f;
        yaw_only.mahony.ezInt = 0.0f;
    }

    gx += kp * ex + yaw_only.mahony.exInt;
    gy += kp * ey + yaw_only.mahony.eyInt;
    gz += kp * ez + yaw_only.mahony.ezInt;

    halfT = 0.5f * dt;
    q0 += (-q1 * gx - q2 * gy - q3 * gz) * halfT;
    q1 += ( q0 * gx + q2 * gz - q3 * gy) * halfT;
    q2 += ( q0 * gy - q1 * gz + q3 * gx) * halfT;
    q3 += ( q0 * gz + q1 * gy - q2 * gx) * halfT;

    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;

    yaw_only.mahony.q.q0 = q0;
    yaw_only.mahony.q.q1 = q1;
    yaw_only.mahony.q.q2 = q2;
    yaw_only.mahony.q.q3 = q3;

    yaw_deg = atan2f(2.0f * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * 180.0f / PI;

    return yaw_deg;
}
#endif

#if DEFINE_IMU_ANALYSIS_MODE == 6
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     Madgwick AHRS Yaw-only算法
//-------------------------------------------------------------------------------------------------------------------
static float Madgwick_YawOnly_Update(float dt)
{
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float hx, hy;
    float _2bx, _2bz, _4bx, _4bz;
    float _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3;
    float q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
    float yaw_deg;
    float beta;

    int16_t mag_x = 0;
    int16_t mag_y = 0;
    int16_t mag_z = 0;

    float gx, gy, gz;
    float ax, ay, az;
    float q0 = yaw_only.madgwick.q.q0;
    float q1 = yaw_only.madgwick.q.q1;
    float q2 = yaw_only.madgwick.q.q2;
    float q3 = yaw_only.madgwick.q.q3;

    if (dt <= 1e-6f)
    {
        dt = 0.01f;
    }

    beta = imu_stable ? yaw_only.madgwick.beta : yaw_only.madgwick.quick_beta;

    IMU_Mag_Apply(&mag_cal, &mag_x, &mag_y, &mag_z);
    IMU_Acc_Apply(&ax, &ay, &az);
    gyro_data_process(&gx, &gy, &gz);
    float mx = (float)mag_x;
    float my = (float)mag_y;
    float mz = (float)mag_z;

    float declination_rad = yaw_only.mag_get_yaw.mag_declination * PI / 180.0f;
    float mx_raw = mx;
    float my_raw = my;
    mx = mx_raw * cosf(declination_rad) - my_raw * sinf(declination_rad);
    my = mx_raw * sinf(declination_rad) + my_raw * cosf(declination_rad);

    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    if (recipNorm < 1e-6f)
    {
        return atan2f(2.0f * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * 180.0f / PI;
    }
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    recipNorm = invSqrt(mx * mx + my * my + mz * mz);
    if (recipNorm < 1e-6f)
    {
        return atan2f(2.0f * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * 180.0f / PI;
    }
    mx *= recipNorm;
    my *= recipNorm;
    mz *= recipNorm;

    q0q1 = q0 * q1;
    q0q2 = q0 * q2;
    q0q3 = q0 * q3;
    q1q1 = q1 * q1;
    q1q2 = q1 * q2;
    q1q3 = q1 * q3;
    q2q2 = q2 * q2;
    q2q3 = q2 * q3;
    q3q3 = q3 * q3;

    hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
    hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
    _2bx = sqrtf(hx * hx + hy * hy);
    _2bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

    _2q0 = 2.0f * q0;
    _2q1 = 2.0f * q1;
    _2q2 = 2.0f * q2;
    _2q3 = 2.0f * q3;
    _2q0q2 = 2.0f * q0q2;
    _2q2q3 = 2.0f * q2q3;
    _4bx = 2.0f * _2bx;
    _4bz = 2.0f * _2bz;

    s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax)
       + _2q1 * (2.0f * q0q1 + _2q2q3 - ay)
       - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx)
       + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my)
       + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax)
       + _2q0 * (2.0f * q0q1 + _2q2q3 - ay)
       - 4.0f * q1 * (1.0f - 2.0f * q1q1 - 2.0f * q2q2 - az)
       + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx)
       + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my)
       + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax)
       + _2q3 * (2.0f * q0q1 + _2q2q3 - ay)
       - 4.0f * q2 * (1.0f - 2.0f * q1q1 - 2.0f * q2q2 - az)
       + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx)
       + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my)
       + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax)
       + _2q2 * (2.0f * q0q1 + _2q2q3 - ay)
       + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx)
       + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my)
       + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);

    recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
    if (recipNorm >= 1e-6f)
    {
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;
    }

    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz) - beta * s0;
    qDot2 = 0.5f * ( q0 * gx + q2 * gz - q3 * gy) - beta * s1;
    qDot3 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx) - beta * s2;
    qDot4 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx) - beta * s3;

    q0 += qDot1 * dt;
    q1 += qDot2 * dt;
    q2 += qDot3 * dt;
    q3 += qDot4 * dt;

    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;

    yaw_only.madgwick.q.q0 = q0;
    yaw_only.madgwick.q.q1 = q1;
    yaw_only.madgwick.q.q2 = q2;
    yaw_only.madgwick.q.q3 = q3;

    yaw_deg = atan2f(2.0f * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * 180.0f / PI;

    return yaw_deg;
}
#endif

#if DEFINE_IMU_ANALYSIS_MODE == 7
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     TiltMagYaw算法（重力投影磁修正陀螺积分）
//-------------------------------------------------------------------------------------------------------------------
static float TiltMagYaw_Update(float dt)
{
    int16_t mag_x = 0;
    int16_t mag_y = 0;
    int16_t mag_z = 0;

    float gx, gy, gz;
    float ax, ay, az;
    float yaw_deg;
    float kp, ki;

    if (dt <= 1e-6f)
    {
        dt = 0.01f;
    }

    kp = imu_stable ? yaw_only.tilt_mag_yaw.kp : yaw_only.tilt_mag_yaw.quick_kp;
    ki = imu_stable ? yaw_only.tilt_mag_yaw.ki : yaw_only.tilt_mag_yaw.quick_ki;

    IMU_Mag_Apply(&mag_cal, &mag_x, &mag_y, &mag_z);
    IMU_Acc_Apply(&ax, &ay, &az);
    gyro_data_process(&gx, &gy, &gz);
    float mx = (float)mag_x;
    float my = (float)mag_y;
    float mz = (float)mag_z;

    float acc_norm = sqrtf(ax * ax + ay * ay + az * az);
    float mag_norm = sqrtf(mx * mx + my * my + mz * mz);
    float cos_roll, sin_roll, cos_pitch, sin_pitch;
    float mx_level, my_level;
    float yaw_mag;
    float yaw_gyro_inc;
    float yaw_error;

    if (acc_norm > 1e-6f)
    {
        ax /= acc_norm;
        ay /= acc_norm;
        az /= acc_norm;
    }

    if (mag_norm > 1e-6f)
    {
        mx /= mag_norm;
        my /= mag_norm;
        mz /= mag_norm;
    }

    float pitch = -asinf(ax);
    float roll = atan2f(ay, az);

    cos_roll = cosf(roll);
    sin_roll = sinf(roll);
    cos_pitch = cosf(pitch);
    sin_pitch = sinf(pitch);

    mx_level = mx * cos_pitch + my * sin_roll * sin_pitch + mz * cos_roll * sin_pitch;
    my_level = my * cos_roll - mz * sin_roll;

    float declination_rad = yaw_only.tilt_mag_yaw.mag_declination * PI / 180.0f;
    float mx_raw = mx_level;
    float my_raw = my_level;
    mx_level = mx_raw * cosf(declination_rad) - my_raw * sinf(declination_rad);
    my_level = mx_raw * sinf(declination_rad) + my_raw * cosf(declination_rad);

    yaw_mag = atan2f(my_level, mx_level);
    yaw_mag = yaw_mag * 180.0f / PI;

    while (yaw_mag > 180.0f) yaw_mag -= 360.0f;
    while (yaw_mag < -180.0f) yaw_mag += 360.0f;

    yaw_deg = yaw_only.tilt_mag_yaw.yaw;
    yaw_gyro_inc = gz * 180.0f / PI * dt;
    yaw_deg += yaw_gyro_inc;

    while (yaw_deg > 180.0f) yaw_deg -= 360.0f;
    while (yaw_deg < -180.0f) yaw_deg += 360.0f;

    yaw_error = -yaw_mag - yaw_deg;
    while (yaw_error > 180.0f) yaw_error -= 360.0f;
    while (yaw_error < -180.0f) yaw_error += 360.0f;

    yaw_only.tilt_mag_yaw.yaw_error_int += yaw_error * ki * dt;
    yaw_deg += kp * yaw_error + yaw_only.tilt_mag_yaw.yaw_error_int;

    while (yaw_deg > 180.0f) yaw_deg -= 360.0f;
    while (yaw_deg < -180.0f) yaw_deg += 360.0f;

    yaw_only.tilt_mag_yaw.yaw = yaw_deg;
    yaw_only.tilt_mag_yaw.yaw_filtered = yaw_only.tilt_mag_yaw.yaw_filter_alpha * (-yaw_deg) +
                                            (1.0f - yaw_only.tilt_mag_yaw.yaw_filter_alpha) * yaw_only.tilt_mag_yaw.yaw_filtered;

    return yaw_only.tilt_mag_yaw.yaw_filtered;
}
#endif
/*******************************************************************************************************************/
/*------------------------------------------------------------------------------------------[E] 仅Yaw输出姿态解算 [E]*/
/*******************************************************************************************************************/


/*******************************************************************************************************************/
/*[S] 姿态更新主函数 [S]---------------------------------------------------------------------------------------------*/
/*******************************************************************************************************************/

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     IMU姿态解算主函数
// 备注信息     在定时器中断中调用，根据DEFINE_IMU_ANALYSIS_MODE选择解算模式
//-------------------------------------------------------------------------------------------------------------------
void IMU_Update_Analysis(void)
{
    // 0 关闭
    // 1 三轴
    // 2 六轴
    // 3 九轴
    // 4 [仅输出Yaw]Mag_Get_Yaw(仅磁力计+倾斜补偿)
    // 5 [仅输出Yaw]Mahony AHRS(九轴)
    // 6 [仅输出Yaw]Madgwick AHRS(九轴)
    // 7 [仅输出Yaw]TiltMagYaw(重力投影磁修正陀螺积分)
    
    #if   DEFINE_IMU_ANALYSIS_MODE == 0 
        Yaw_Result = 0.0f;
        Roll_Result = 0.0f;
        Pitch_Result = 0.0f;
    #elif DEFINE_IMU_ANALYSIS_MODE == 1
        ThreeAxis_Update();
    #elif DEFINE_IMU_ANALYSIS_MODE == 2
        SixAxis_Update();
    #elif DEFINE_IMU_ANALYSIS_MODE == 3
        NineAxis_Update();
    #elif DEFINE_IMU_ANALYSIS_MODE == 4
        Yaw_Result = Mag_Get_Yaw_Update();
        Roll_Result = 0.0f;
        Pitch_Result = 0.0f;
    #elif DEFINE_IMU_ANALYSIS_MODE == 5
        Yaw_Result = Mahony_YawOnly_Update(DELTA_T_9AXIS);
        Roll_Result = 0.0f;
        Pitch_Result = 0.0f;
    #elif DEFINE_IMU_ANALYSIS_MODE == 6
        Yaw_Result = Madgwick_YawOnly_Update(DELTA_T_9AXIS);
        Roll_Result = 0.0f;
        Pitch_Result = 0.0f;
    #elif DEFINE_IMU_ANALYSIS_MODE == 7
        Yaw_Result = TiltMagYaw_Update(DELTA_T_9AXIS);
        Roll_Result = 0.0f;
        Pitch_Result = 0.0f;
    #endif

}

/*******************************************************************************************************************/
/*---------------------------------------------------------------------------------------------[E] 姿态更新主函数 [E]*/
/*******************************************************************************************************************/


/*******************************************************************************************************************/
/*[S] 工具性函数 [S]------------------------------------------------------------------------------------------------*/
/*******************************************************************************************************************/

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     重置IMU姿态角数据
// 备注信息     调试性质
//-------------------------------------------------------------------------------------------------------------------
void IMU_Reset_Data (void)
{
    // 重置对应算法的中间数据
    // 0 关闭
    // 1 三轴
    // 2 六轴
    // 3 九轴
    // 4 [仅输出Yaw]Mag_Get_Yaw(仅磁力计+倾斜补偿)
    // 5 [仅输出Yaw]Mahony AHRS(九轴)
    // 6 [仅输出Yaw]Madgwick AHRS(九轴)
    // 7 [仅输出Yaw]TiltMagYaw(重力投影磁修正陀螺积分)


    #if   DEFINE_IMU_ANALYSIS_MODE == 0 

    #elif DEFINE_IMU_ANALYSIS_MODE == 1
        three_axis.q.q0 = 1.0f;
        three_axis.q.q1 = 0.0f;
        three_axis.q.q2 = 0.0f;
        three_axis.q.q3 = 0.0f;
    #elif DEFINE_IMU_ANALYSIS_MODE == 2
        six_axis.q.q0 = 1.0f;
        six_axis.q.q1 = 0.0f;
        six_axis.q.q2 = 0.0f;
        six_axis.q.q3 = 0.0f;
        six_axis.exInt = 0.0f;
        six_axis.eyInt = 0.0f;
        six_axis.ezInt = 0.0f;
        six_axis.quick_mode = 1;
    #elif DEFINE_IMU_ANALYSIS_MODE == 3
        nine_axis.mahony.q.q0 = 1.0f;
        nine_axis.mahony.q.q1 = 0.0f;
        nine_axis.mahony.q.q2 = 0.0f;
        nine_axis.mahony.q.q3 = 0.0f;
        nine_axis.mahony.exInt = 0.0f;
        nine_axis.mahony.eyInt = 0.0f;
        nine_axis.mahony.ezInt = 0.0f;
        nine_axis.mahony.quick_mode = 1;
    #elif DEFINE_IMU_ANALYSIS_MODE == 4
        yaw_only.mag_get_yaw.yaw_filtered = 0.0f;
    #elif DEFINE_IMU_ANALYSIS_MODE == 5
        yaw_only.mahony.q.q0 = 1.0f;
        yaw_only.mahony.q.q1 = 0.0f;
        yaw_only.mahony.q.q2 = 0.0f;
        yaw_only.mahony.q.q3 = 0.0f;
        yaw_only.mahony.exInt = 0.0f;
        yaw_only.mahony.eyInt = 0.0f;
        yaw_only.mahony.ezInt = 0.0f;
        yaw_only.mahony.quick_mode = 1;
    #elif DEFINE_IMU_ANALYSIS_MODE == 6
        yaw_only.madgwick.q.q0 = 1.0f;
        yaw_only.madgwick.q.q1 = 0.0f;
        yaw_only.madgwick.q.q2 = 0.0f;
        yaw_only.madgwick.q.q3 = 0.0f;
    #elif DEFINE_IMU_ANALYSIS_MODE == 7
        yaw_only.tilt_mag_yaw.yaw = 0.0f;
        yaw_only.tilt_mag_yaw.yaw_filtered = 0.0f;
        yaw_only.tilt_mag_yaw.yaw_error_int = 0.0f;
    #endif

    // 重置姿态结果
    Yaw_Result = 0.0f;    // 偏航角（Yaw）
    Roll_Result = 0.0f;   // 横滚角（Roll）
    Pitch_Result = 0.0f;  // 俯仰角（Pitch）
}
/*******************************************************************************************************************/
/*------------------------------------------------------------------------------------------------[E] 工具性函数 [E]*/
/*******************************************************************************************************************/





