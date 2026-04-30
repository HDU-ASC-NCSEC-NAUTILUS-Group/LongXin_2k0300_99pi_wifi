#ifndef _zf_driver_uvc_h
#define _zf_driver_uvc_h

#include "zf_common_typedef.h"
#include <opencv2/opencv.hpp>

#define UVC_WIDTH   160
#define UVC_HEIGHT  120
#define UVC_FPS     30

int8_t uvc_camera_init(const char *path);
int8_t wait_image_refresh();
int8_t wait_image_refresh_rgb();

extern uint8_t *rgay_image;
extern uint8_t *rgb_image;

extern cv::Mat frame_rgb;      
extern cv::Mat frame_rgay; 

#endif
