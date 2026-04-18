#include "zf_common_headfile.h"
#include <cmath>
#include <cstring>
#include <vector>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// 屏幕尺寸
static constexpr uint16_t SCREEN_W = 240;
static constexpr uint16_t SCREEN_H = 320;

// 颜色
static constexpr uint16_t COLOR_BG      = RGB565_BLACK;
static constexpr uint16_t COLOR_OCC     = RGB565_WHITE;   // 障碍物格子
static constexpr uint16_t COLOR_FREE    = RGB565_GRAY;// 空闲格子（可设为背景色以隐藏）
static constexpr uint16_t COLOR_CAR     = RGB565_RED;

// 地图参数
static float map_size;           // 米
static float grid_res;           // 米
static int   grid_num;           // 每边网格数
static float scale_screen;       // 米到像素的缩放系数

static uint16_t car_x, car_y;    // 小车屏幕坐标
static float angle_offset_rad;   // 弧度

// 栅格概率存储
static std::vector<int8_t> grid; // 大小 grid_num * grid_num

// 概率更新参数
static constexpr int8_t OCC_INC = 6;
static constexpr int8_t FREE_DEC = -4;
static constexpr int8_t OCC_THRESH = 10;
static constexpr int8_t FREE_THRESH = -10;

// 世界坐标（米）转网格索引（小车位于地图中心）
static bool world_to_grid(float wx, float wy, int* gx, int* gy) {
    float half = map_size * 0.5f;
    int ix = static_cast<int>(( wx + half) / grid_res);
    int iy = static_cast<int>((-wy + half) / grid_res); // 注意Y方向
    if (ix < 0 || ix >= grid_num || iy < 0 || iy >= grid_num)
        return false;
    *gx = ix;
    *gy = iy;
    return true;
}

// 网格索引转屏幕像素（格子中心）
static void grid_to_screen(int gx, int gy, int* sx, int* sy) {
    float world_x = (gx + 0.5f) * grid_res - map_size * 0.5f;
    float world_y = -(gy + 0.5f) * grid_res + map_size * 0.5f;
    *sx = car_x + static_cast<int>( world_y * scale_screen);
    *sy = car_y + static_cast<int>(-world_x * scale_screen);
}

// Bresenham更新射线
static void update_ray(float angle_rad, float dist_m) {
    float end_x = dist_m * cosf(angle_rad);
    float end_y = dist_m * sinf(angle_rad);
    
    int start_gx = grid_num / 2;
    int start_gy = grid_num / 2;
    int end_gx, end_gy;
    if (!world_to_grid(end_x, end_y, &end_gx, &end_gy))
        return; // 终点超出地图，忽略
    
    int x0 = start_gx, y0 = start_gy;
    int x1 = end_gx, y1 = end_gy;
    int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = dx + dy, e2;
    
    while (true) {
        int idx = y0 * grid_num + x0;
        if (x0 == x1 && y0 == y1) {
            // 终点：占据
            if (dist_m < map_size * 0.5f) {
                grid[idx] = std::min<int8_t>(grid[idx] + OCC_INC, OCC_THRESH);
            }
            break;
        } else {
            // 中间：空闲
            grid[idx] = std::max<int8_t>(grid[idx] + FREE_DEC, FREE_THRESH);
        }
        e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}

// 初始化
void map_init(float map_size_m, float grid_res_m,
              uint16_t car_x_screen, uint16_t car_y_screen,
              uint8_t screen_radius_px, float angle_offset_deg)
{
    map_size = map_size_m;
    grid_res = grid_res_m;
    grid_num = static_cast<int>(map_size / grid_res + 0.5f);
    grid.assign(grid_num * grid_num, 0);
    
    car_x = car_x_screen;
    car_y = car_y_screen;
    scale_screen = screen_radius_px / (map_size * 0.5f);
    angle_offset_rad = angle_offset_deg * (M_PI / 180.0f);
}

void map_clear() {
    std::fill(grid.begin(), grid.end(), 0);
}

void map_update(const LaserPoint* points, size_t count) {
    if (points == nullptr || count == 0) return;
    
    // 1. 更新栅格
    for (size_t i = 0; i < count; ++i) {
        float ang = (points[i].angle_deg + angle_offset_rad) * (M_PI / 180.0f);
        float dist = points[i].distance_mm * 0.001f;
        if (dist > 0.05f && dist < map_size * 0.5f) {
            update_ray(ang, dist);
        }
    }
    
    // 2. 绘制
    ips200_full(COLOR_BG);
    
    int cell_px = static_cast<int>(grid_res * scale_screen);
    if (cell_px < 1) cell_px = 1;
    
    for (int gy = 0; gy < grid_num; ++gy) {
        for (int gx = 0; gx < grid_num; ++gx) {
            int8_t val = grid[gy * grid_num + gx];
            uint16_t color = COLOR_BG;
            if (val >= OCC_THRESH)
                color = COLOR_OCC;
            else if (val <= FREE_THRESH)
                color = COLOR_FREE;  // 若不想显示空闲格子，注释此行
            else
                continue;
            
            int sx, sy;
            grid_to_screen(gx, gy, &sx, &sy);
            // 绘制矩形
            for (int dy = 0; dy < cell_px; ++dy) {
                for (int dx = 0; dx < cell_px; ++dx) {
                    int px = sx + dx - cell_px/2;
                    int py = sy + dy - cell_px/2;
                    if (px >= 0 && px < SCREEN_W && py >= 0 && py < SCREEN_H)
                        ips200_draw_point(px, py, color);
                }
            }
        }
    }
    
    // 3. 画小车（5x5红块）
    for (int dy = -2; dy <= 2; ++dy) {
        for (int dx = -2; dx <= 2; ++dx) {
            int px = car_x + dx;
            int py = car_y + dy;
            if (px >= 0 && px < SCREEN_W && py >= 0 && py < SCREEN_H)
                ips200_draw_point(px, py, COLOR_CAR);
        }
    }
}

void map_redraw() {
    // 仅重绘，不更新数据
    map_update(nullptr, 0);
}