#ifndef __image_process_h
#define __image_process_h

//===============================================================================================//
//二维码相关函数
void QR_process(void);
void object_tracking(void);
void coordinate_transformation(void);

extern float real_x, real_y;  // 从坐标转换函数获取的物理坐标

#endif
