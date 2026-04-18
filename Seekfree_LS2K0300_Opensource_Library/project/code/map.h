#ifndef MAP_H
#define MAP_H

#include <cstdint>
#include <vector>
#include "zf_common_headfile.h"

struct LaserPoint;

/**
 * @brief 初始化占据栅格地图
 * @param map_size_m         地图边长（米），例如2.0
 * @param grid_resolution_m  网格分辨率（米），例如0.05
 * @param car_screen_x       小车在屏幕上的X坐标（像素）
 * @param car_screen_y       小车在屏幕上的Y坐标（像素）
 * @param screen_radius_px   地图在屏幕上的显示半径（像素）
 * @param angle_offset_deg   雷达0度方向偏移（度），使0度对应屏幕上方（如90）
 */
void map_init(float map_size_m, float grid_resolution_m,
              uint16_t car_screen_x, uint16_t car_screen_y,
              uint8_t screen_radius_px, float angle_offset_deg);

/**
 * @brief 用一帧完整点云更新地图并重绘屏幕
 * @param points 点云数组指针
 * @param count  点数
 */
void map_update(const LaserPoint* points, size_t count);

/**
 * @brief 清空地图（重置所有栅格为未知）
 */
void map_clear();

/**
 * @brief 仅重绘屏幕（不更新栅格数据）
 */
void map_redraw();

#endif // MAP_H