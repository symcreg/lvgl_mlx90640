#ifndef __SYMC_MLX90640_H__
#define __SYMC_MLX90640_H__

#include "stm32f4xx.h"
#include "../../lvgl/lvgl.h"

#define uint8_t unsigned char
#define uint16_t unsigned short
#define uint32_t unsigned int

#define SRC_WIDTH 32
#define SRC_HEIGHT 24

#define TARGET_WIDTH 320
#define TARGET_HEIGHT 240


extern uint8_t mlx90640_buf[1544];
extern uint8_t state;
extern uint8_t is_update;

uint8_t Check(uint8_t *data);


// 双缓冲区

extern uint8_t disp_finish_flag;
extern uint8_t updating_buffer_flag;
extern uint8_t buffer_a[SRC_WIDTH * SRC_HEIGHT * 2];
extern uint8_t buffer_b[SRC_WIDTH * SRC_HEIGHT * 2];


void swap_buffers(lv_obj_t* img);
void update_buffer(uint8_t* src);
void create_dynamic_image(lv_obj_t* parent);
#endif