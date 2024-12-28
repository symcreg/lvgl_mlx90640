// �ȳ�����
// ģ��ѡ�ã�MLX90640 LCD ������ ���
// ͨ�ŷ�ʽ��I2C USART PWM SPI
// ͨ�����ʣ�400KHz
// �ȳ��񣺼�ⲻͬ����ĺ�����䣬ת�����¶�����
// ��Ҫ���ܣ���ȡ�ȳ����ǵ����ݣ���ʾ����Ļ�ϣ�ӳ�䵽����Ƕȣ���׼�¶����λ�ã�
// ͼ���ܻ���LVGL����ȫ���ƺ����ʹ�����������ʵ����ֲ
// ���ߣ�Sam Wang
// ��Ҫ����0���������ϲ��࣬��Ҫ�Լ��о�һ�£���Ҫ�������ֲᣬ�Լ�д����
// ��Ҫ����1��mlx90640�ķֱ�����32*24������Ļ�ķֱ�����320*240
// Ϊ����ʾ����Ļ�ϣ���Ҫ��32*24�����ݽ��в�ֵ�����320*240������
// ��Ҫ����2���ڴ����⣬320*240��������Ҫ150KB���ڴ棬��stm32f407���ڴ�ֻ��192KB����Ȼ����
// ���������ʹ�ö�̬�ڴ���䣬ÿ��ֻ����һ�е��ڴ棬Ȼ���ͷţ����߷ֿ���ʾ
// ��Ҫ����3��ˢ�������⣬���ÿ�ζ���ֵ���ᵼ��ˢ���ʹ��ͣ���Ҫ�Ż�
// ���������ʹ��˫���壬ÿ��ֻ��ֵһ�У�Ȼ����ʾ��Ȼ���ٲ�ֵһ�У�Ȼ����ʾ�����������ƣ���ֹ���ݳ�ͻ
// ˫���壺������������һ����ʾ��һ����ֵ��ÿ�β�ֵ��һ�У��ͽ�������������
// �����ƣ�ʹ���ź�����ÿ�β�ֵ��һ�У����ͷ��ź�������ʾ��һ�У��ͻ�ȡ�ź���

// ����ͼ��ϵͳ�ṹͼ

#include "main.h"
#include "delay.h"
#include "LED.h"
#include "KEY.h"
#include "lcd.h"
#include "my_pic.h"
#include "TIME.h"
#include "../BaseDrive/TOUCH/touch.h"
#include "linear_interpolation.h"
#include "../BaseDrive/servo/servo.h"
#include "../BaseDrive/USART.h"
#include "../BaseDrive/mlx90640/mlx90640.h"
#include <stdlib.h>
#include "lvgl.h"                // ��Ϊ����LVGL�ṩ�˸�������ͷ�ļ�����
#include "lv_port_disp.h"        // LVGL����ʾ֧��
#include "lv_port_indev.h"       // LVGL�Ĵ���֧��

/**********************************************************************************************************
�������ƣ�main����
�����������
�����������
�������أ���
**********************************************************************************************************/

#define UPDATE_TIME 500
#define IMG_WIDTH 320
#define IMG_HEIGHT 240
#define CHART_SIZE (32)
#define LOG_MAX_LINES 25
lv_obj_t* chart;
lv_chart_series_t* ser;
static float max_chart_cnt = 0;
static float min_chart_cnt = 500;
static uint8_t max_temp_x = 0;
static uint8_t max_temp_y = 0;
static uint8_t min_temp_x = 0;
static uint8_t min_temp_y = 0;
static float chart_cnt[CHART_SIZE] = {0};
static uint8_t chart_data[CHART_SIZE] = {0};
static uint16_t chart_max_x = 0;
static uint16_t chart_max_y = 0;
static uint8_t chart_max_value = 0;
static uint8_t chart_min_value = 0;
lv_obj_t* pointer_label;
lv_anim_t pointer_anim;
lv_obj_t* pointer_line_x;
static lv_point_t line_x_draw[2] = {{320+320/2,0},{320+320/2,240}};
lv_anim_t pointer_line_x_anim;
lv_obj_t* pointer_line_y;
static lv_point_t line_y_draw[2] = {{320,240/2},{320+320,240/2}};
lv_anim_t pointer_line_y_anim;
static lv_obj_t* log_area;

static uint8_t is_info_show = 1;
lv_obj_t *label_max_min_temp;
lv_obj_t *label_max_min_temp_xy;
lv_obj_t *label_servo_angle;
lv_obj_t *label_mean_variance;
lv_obj_t *label_time_stamp;
lv_obj_t *log_box;
char log_buf[LOG_MAX_LINES][32];
uint8_t log_line = 0;
// ��ť���¼��ص�����
static void snap_stream_event(lv_event_t *event)
{
    lv_obj_t *btn = lv_event_get_target(event);                    // ��õ�������ص������Ķ���
    if (event->code == LV_EVENT_CLICKED)
    {
        lv_obj_t *label = lv_obj_get_child(btn, NULL);             // ��ȡ��1���Ӷ���(���������ʱ���Ѱ��������ĵ�1���Ӷ�����һ��label����)
        is_update = !is_update;
		if(is_update){
			lv_label_set_text_fmt(label, "snapshot");
		}else{
			lv_label_set_text_fmt(label, "streaming");
		}
	}
}
static void hide_labels(void){
	lv_obj_add_flag(label_max_min_temp, LV_OBJ_FLAG_HIDDEN);
	lv_obj_add_flag(label_max_min_temp_xy, LV_OBJ_FLAG_HIDDEN);
	lv_obj_add_flag(label_servo_angle, LV_OBJ_FLAG_HIDDEN);
	lv_obj_add_flag(label_mean_variance, LV_OBJ_FLAG_HIDDEN);
}
static void show_labels(void){
	lv_obj_clear_flag(label_max_min_temp, LV_OBJ_FLAG_HIDDEN);
	lv_obj_clear_flag(label_max_min_temp_xy, LV_OBJ_FLAG_HIDDEN);
	lv_obj_clear_flag(label_servo_angle, LV_OBJ_FLAG_HIDDEN);
	lv_obj_clear_flag(label_mean_variance, LV_OBJ_FLAG_HIDDEN);
}
static void is_show_info_event(lv_event_t *event)
{
	lv_obj_t *btn = lv_event_get_target(event);                    // ��õ�������ص������Ķ���
	
	if (event->code == LV_EVENT_CLICKED)
	{
		if(!is_info_show){
			hide_labels();
			is_info_show = 1;
		}else{
			show_labels();
			is_info_show = 0;
		}
	}
}
lv_color_t map_temperature_to_color(float temperature, float min_t, float max_t){
	if (temperature < min_t) {
		temperature = min_t;
	}
	if (temperature > max_t) {
		temperature = max_t;
	}
	// ��ɫ -> ��ɫ -> ��ɫ
	// 0 0 255 -> 255 255 0 -> 255 0 0
	float ratio = (temperature - min_t) / (max_t - min_t);
	uint8_t r = 0, g = 0, b = 0;
	if(ratio < 0.5){
		r = 255 * ratio * 2;
		g = 255;
		b = 255 - 255 * ratio * 2;
	}else{
		r = 255;
		g = 255 - 255 * (ratio - 0.5) * 2;
		b = 0;
	}
	return lv_color_make(r, g, b);
}
static void anim_x_cb(void * var, int32_t v){
    lv_obj_set_x(var, v);
}
static void anim_line_x_cb(void * line, int32_t v){
    line_x_draw[0].x = v;
    line_x_draw[1].x = v;
    lv_line_set_points(line, line_x_draw, 2);
}static void anim_line_y_cb(void * line, int32_t v){
    line_y_draw[0].y = v;
    line_y_draw[1].y = v;
    lv_line_set_points(line, line_y_draw, 2);
}
//TODO: call it when calculate finished
static void update_pointer_anim(uint16_t x_d, uint16_t t_d){
    lv_anim_set_values(&pointer_anim, lv_obj_get_x(pointer_label), x_d);
    lv_anim_set_time(&pointer_anim, t_d);
    lv_anim_start(&pointer_anim);
}
static void update_line_x_anim(uint16_t x_d, uint16_t t_d){
    lv_anim_set_values(&pointer_line_x_anim, line_x_draw[0].x, x_d);
    lv_anim_set_time(&pointer_line_x_anim, t_d);
    lv_anim_start(&pointer_line_x_anim);
}
static void update_line_y_anim(uint16_t y_d, uint16_t t_d){
    lv_anim_set_values(&pointer_line_y_anim, line_y_draw[0].y, y_d);
    lv_anim_set_time(&pointer_line_y_anim, t_d);
    lv_anim_start(&pointer_line_y_anim);
}
static void create_pointer_anim(){
    pointer_label = lv_label_create(lv_scr_act());
    lv_label_set_text(pointer_label, "1");
    lv_obj_set_pos(pointer_label, 0, 440);
    lv_anim_init(&pointer_anim);
    lv_anim_set_var(&pointer_anim, pointer_label);
    lv_anim_set_values(&pointer_anim, lv_obj_get_x(pointer_label), 0);
    lv_anim_set_time(&pointer_anim, UPDATE_TIME);
    lv_anim_set_exec_cb(&pointer_anim, anim_x_cb);
    lv_anim_set_path_cb(&pointer_anim, lv_anim_path_overshoot);
    // lv_anim_start(&anim);
	pointer_line_x = lv_line_create(lv_scr_act());
    pointer_line_y = lv_line_create(lv_scr_act());
    lv_line_set_points(pointer_line_x, line_x_draw, 2);
    lv_line_set_points(pointer_line_y, line_y_draw, 2);

    lv_anim_init(&pointer_line_x_anim);
    lv_anim_init(&pointer_line_y_anim);
    lv_anim_set_var(&pointer_line_x_anim, pointer_line_x);
    lv_anim_set_var(&pointer_line_y_anim, pointer_line_y);
    lv_anim_set_values(&pointer_line_x_anim, line_x_draw[0].x, 320+320);
    lv_anim_set_values(&pointer_line_y_anim, line_y_draw[0].y, 240);
    lv_anim_set_time(&pointer_line_x_anim, 250);
    lv_anim_set_time(&pointer_line_y_anim, 250);
    lv_anim_set_exec_cb(&pointer_line_x_anim, anim_line_x_cb);
    lv_anim_set_exec_cb(&pointer_line_y_anim, anim_line_y_cb);
    // // lv_anim_set_path_cb(&pointer_line_x_anim, lv_anim_path_overshoot);
    // lv_anim_start(&pointer_line_x_anim);
    // lv_anim_start(&pointer_line_y_anim);

}
static void chart_draw_event_cb(lv_event_t * e){
    lv_obj_draw_part_dsc_t* chart_dsc = lv_event_get_draw_part_dsc(e);
    /* �����ƵĲ����Ƿ�����״�� */
    if (chart_dsc->part == LV_PART_ITEMS) {
		
        uint16_t index = chart_dsc->id; // ��ȡ��ǰ���ӵ�����
        lv_coord_t value = chart_dsc->value; // ��ȡ��ǰ���ӵ�ֵ
        /* ����ֵ���ò�ͬ��ɫ */
        // uint16_t ratio = value * 255 / 100;
        // chart_dsc->rect_dsc->bg_color = lv_color_make(ratio, 0, (255-ratio)/2);
		chart_dsc->rect_dsc->bg_color = map_temperature_to_color(value, chart_min_value, chart_max_value);
    }
}
static void update_chart_task(lv_timer_t *timer){
    // ��������
	//TODO: test
	// uint16_t max_v = 0;
	// uint16_t min_v = 100;
	// for (uint16_t i = 0; i < CHART_SIZE; i++) {
    //     chart_data[i] = lv_rand(0, 90);
	// 	if (chart_data[i] > max_v) {
	// 		max_v = chart_data[i];
	// 		chart_max_x = 10 + i * 310 / CHART_SIZE;
	// 	}
	// 	if (chart_data[i] < min_v) {
	// 		min_v = chart_data[i];
	// 	}
	// }
	// // TODO: update y
	// chart_max_y = lv_rand(0, 240);
	
	// chart_max_value = max_v;
	// chart_min_value = min_v;
	chart_max_x = max_temp_x * 10;
	chart_max_y = max_temp_y * 10;
    for (uint16_t i = 0; i < CHART_SIZE; i++) {
		lv_chart_set_next_value(chart, ser, chart_data[i]);
	}
    lv_chart_refresh(chart);
	
    update_pointer_anim(chart_max_x, UPDATE_TIME);
    
	//0 - 90
	SetServoAngle(0, 90 * (float)chart_max_x / 320);
	SetServoAngle(1, 45 * (float)chart_max_y / 240);
	
    update_line_x_anim(320+chart_max_x,UPDATE_TIME/2);
	update_line_y_anim(chart_max_y,UPDATE_TIME/2);
	// update_line_y_anim(lv_rand(0, 240),UPDATE_TIME/2);
    // update_line_y_anim(chart_max_y,250); //TODO: update y
}
void create_image_chart(){
    chart = lv_chart_create(lv_scr_act());
    lv_chart_set_type(chart, LV_CHART_TYPE_BAR);
    lv_chart_set_point_count(chart, CHART_SIZE);
    lv_obj_set_style_pad_column(chart, 2, 0);
    lv_obj_set_size(chart, 320, 200);
    lv_obj_set_style_bg_color(chart, lv_color_hex(0xE6E6FA), LV_PART_MAIN);
    //lv_obj_align(chart, LV_ALIGN_BOTTOM_LEFT, 0, 0);
    lv_obj_set_pos(chart, 0, 240);
	ser = lv_chart_add_series(chart, lv_color_hex(0xff0000), LV_CHART_AXIS_PRIMARY_Y);
    lv_obj_add_event_cb(chart, chart_draw_event_cb, LV_EVENT_DRAW_PART_BEGIN, ser);
    for(uint16_t i = 0; i < CHART_SIZE; i++) {
        lv_chart_set_next_value(chart, ser, 0);
    }
    lv_timer_create(update_chart_task, UPDATE_TIME, chart);
}
static void update_image_task(lv_timer_t* timer){
	
	// TODO: update the image
	// �ػ�
	// lv_img_set_src(img, &my_pic_dsc);
	lv_obj_invalidate(timer->user_data);
}
static void create_log_box(void){
	log_box = lv_textarea_create(lv_scr_act());
    
    // �����ı������С
    lv_obj_set_size(log_box, 450, 220); // �� 320���� 240
    // lv_obj_align(log_box, LV_ALIGN_CENTER, 0, 0); // ������ʾ
	lv_obj_set_pos(log_box, 320, 240);
    // �����ı�����
    lv_textarea_set_text(log_box, "");              // ��ʼΪ��
    lv_textarea_set_max_length(log_box, 1024);      // ��󳤶�
    lv_textarea_set_cursor_click_pos(log_box, false); // ��ֹ�û��ֶ����ù��
    // lv_textarea_set_scrollbar_mode(log_box, LV_SCROLLBAR_MODE_AUTO); // �Զ�������
    lv_obj_set_style_pad_all(log_box, 5, 0);        // �����ڱ߾�
}
void add_log(const char *log) {
    if (log_box) {
        lv_textarea_add_text(log_box, log);  // ׷����־����
        lv_textarea_add_text(log_box, "\n"); // ����
        // lv_textarea_scroll_to_bottom(log_box); // �������ײ�
    }
}
void clear_log(void) {
    if (log_box) {
        lv_textarea_set_text(log_box, ""); // ����ı�����
    }
}
void add_log_with_limit(const char* log){
	sprintf(log_buf[log_line], "%s", log);
	log_line = (log_line + 1) % LOG_MAX_LINES; // ѭ������

	// �����ı�����
	clear_log();
	for(uint8_t i = 0; i < LOG_MAX_LINES; i++){
		uint8_t idx = (log_line + i) % LOG_MAX_LINES;
		if(strlen(log_buf[idx]) > 0){
			add_log(log_buf[idx]);
		}
	}
}
void create_external_image(void){
	// lv_obj_t* img = lv_img_create(lv_scr_act());
	// static lv_img_dsc_t my_pic_dsc;
	// my_pic_dsc.header.cf = LV_IMG_CF_TRUE_COLOR;       // ͼ���ʽ�����ɫ
    // my_pic_dsc.header.w = IMG_WIDTH;                   // ���
    // my_pic_dsc.header.h = IMG_HEIGHT;                  // �߶�
    // my_pic_dsc.data = (const uint8_t *)my_pic_array;        // ͼ������
    // my_pic_dsc.data_size = IMG_WIDTH * IMG_HEIGHT * 2; // ���ݴ�С��2�ֽ�/���أ�

	// lv_img_set_src(img, &my_pic_dsc);
	// // ���Ͻ�
	// lv_obj_align(img, LV_ALIGN_TOP_LEFT, 0, 0);

	
	// lv_timer_create(update_image_task, UPDATE_TIME, img);
}

uint8_t data_buf[1544];

float myMap(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

extern unsigned int test;
int main(void)
{ 
    
	int16_t temp=0,rest_count=0;
    float Temperature=0;
    float min_temp=500,max_temp=0;
	float mean_temp = 0; // ƽ���¶�
	float variance_temp = 0; // ����
	float delta = 0;
	char output_max_min_temp[30] = {0};
	char output_max_min_temp_xy[35] = {0};
	char output_servo_angle[45] = {0};
	char output_mean_variance[45] = {0};
	char output_time_stamp[30] = {0};
	char log_buf[30] = {0};
	SysTick_Init();                     //  ϵͳ�δ�ʱ����ʼ��
	
	KEYGpio_Init();                     //  ����IO�ڳ�ʼ��
    
	LEDGpio_Init();                     //  ��ʼ��LED
	
	UART3_Configuration();              //  USART3����
    UART5_Configuration(115200);
	ServoInit();                        //  �����ʼ��

	LCD_Init();
	tp_dev.init();				//��������ʼ��

	lv_init();                             // LVGL ��ʼ��
    lv_port_disp_init();                   // ע��LVGL����ʾ����
    lv_port_indev_init();                  // ע��LVGL�Ĵ����������

	// 1ms��ʱ����ʼ��, ÿ1ms����һ���ж�
	TIM2_Configuration();               //  ��ʱ��2��ʼ��

	// // ͼ��

	// create_external_image();
    create_pointer_anim();
	create_image_chart();
	create_log_box();
	// image_init();
    // linear_interpolation();

	//TODO: test
	// lv_obj_t* img = lv_img_create(lv_scr_act());
	// static lv_img_dsc_t pic_dsc;
	// pic_dsc.header.cf = LV_IMG_CF_TRUE_COLOR;       // ͼ���ʽ�����ɫ
    // pic_dsc.header.w = SRC_WIDTH;                   // ���
    // pic_dsc.header.h = SRC_HEIGHT;                  // �߶�
    // pic_dsc.data = (const uint8_t *)&mlx90640_buf[4];        // ͼ������
    // pic_dsc.data_size = SRC_WIDTH * SRC_HEIGHT * 2; // ���ݴ�С��2�ֽ�/���أ�
	// lv_img_set_src(img, &pic_dsc);
	// lv_img_set_zoom(img, 256 * 8);
	// // ���Ͻ�
	// lv_obj_align(img, LV_ALIGN_TOP_LEFT, 144, 115);

	create_dynamic_image(lv_scr_act());


	
    // �����ı�ǩ
    label_max_min_temp = lv_label_create(lv_scr_act());                           // �����ı���ǩ; �����󣺵�ǰ���Ļ
    lv_label_set_text(label_max_min_temp, "initial");                                  // ���ñ�ǩ���ı�
    // lv_obj_align(label_max_min_temp, LV_ALIGN_CENTER, 0, 0);                                // �����ڣ�������
	lv_obj_set_pos(label_max_min_temp, 320+10, 240+10);
	
	label_max_min_temp_xy = lv_label_create(lv_scr_act());                           // �����ı���ǩ; �����󣺵�ǰ���Ļ
	lv_label_set_text(label_max_min_temp_xy, "initial");                                  // ���ñ�ǩ���ı�
	lv_obj_set_pos(label_max_min_temp_xy, 320+10, 240+40);
	
	label_servo_angle = lv_label_create(lv_scr_act());                           // �����ı���ǩ; �����󣺵�ǰ���Ļ
	lv_label_set_text(label_servo_angle, "initial");                                  // ���ñ�ǩ���ı�
	lv_obj_set_pos(label_servo_angle, 320+10, 240+70);

	label_mean_variance = lv_label_create(lv_scr_act());                           // �����ı���ǩ; �����󣺵�ǰ���Ļ
	lv_label_set_text(label_mean_variance, "initial");                                  // ���ñ�ǩ���ı�
	lv_obj_set_pos(label_mean_variance, 320+10, 240+110);

	label_time_stamp = lv_label_create(lv_scr_act());                           // �����ı���ǩ; �����󣺵�ǰ���Ļ
	lv_label_set_text(label_time_stamp, "initial");                                  // ���ñ�ǩ���ı�
	lv_obj_set_pos(label_time_stamp, 220+400, 240+10);
	

	hide_labels();
	// ��ť
	lv_obj_t* button_is_update = lv_btn_create(lv_scr_act());                               // ������ť; �����󣺵�ǰ���Ļ
	lv_obj_align(button_is_update, LV_ALIGN_TOP_RIGHT, 0, 0);                                  // �����ڣ�������
	// lv_obj_align(button_is_update, LV_ALIGN_TOP_LEFT, 0, 0);                                  // �����ڣ�������
	
	lv_obj_add_flag(button_is_update, LV_OBJ_FLAG_CHECKABLE);
	lv_obj_set_size(button_is_update, 100, 70);                                             // ���ð�ť�Ĵ�С
	lv_obj_t* label_is_update = lv_label_create(button_is_update);                          // ������ǩ; �����󣺰�ť
	lv_label_set_text(label_is_update, "snapshot");                                          // ���ñ�ǩ���ı�
	lv_obj_align(label_is_update, LV_ALIGN_CENTER, 0, 0);                                   // �����ڣ�������
	lv_obj_add_event_cb(button_is_update, snap_stream_event, LV_EVENT_CLICKED, NULL);                                     // ���ð�ť���¼��ص�����
	
	lv_obj_t* button_show_info = lv_btn_create(lv_scr_act());                               // ������ť; �����󣺵�ǰ���Ļ
	lv_obj_align(button_show_info, LV_ALIGN_TOP_RIGHT, 0, 90);                                  // �����ڣ�������
	// lv_obj_align(button_show_info, LV_ALIGN_TOP_LEFT, 0, 110);                                  // �����ڣ�������
	
	lv_obj_add_flag(button_show_info, LV_OBJ_FLAG_CHECKABLE);

	lv_obj_set_size(button_show_info, 100, 70);                                             // ���ð�ť�Ĵ�С
	lv_obj_t* label_show_info = lv_label_create(button_show_info);                          // ������ǩ; �����󣺰�ť
	lv_label_set_text(label_show_info, "info");                                          // ���ñ�ǩ���ı�
	lv_obj_align(label_show_info, LV_ALIGN_CENTER, 0, 0);                                   // �����ڣ�������
	lv_obj_add_event_cb(button_show_info, is_show_info_event, LV_EVENT_CLICKED, NULL);                                     // ���ð�ť���¼��ص�����

	uint16_t i = 0;
	static uint8_t msLVGL = 0;
	// �����Ļ�����startup_stm32f4xx.s�ļ��еĶ�ջ��С
	// printf("UART5 test\r\n");
	// add_log_with_limit("start");
	uint16_t time_ms = lv_tick_get();
	uint16_t time_s = time_ms / 1000; // ��
	while(1){
		// ÿ1ms����һ��
		if(lv_tick_get() / 1000 != time_s){
			time_ms = lv_tick_get();
			time_s = time_ms / 1000;
			// sprintf(log_buf, "time: %ds %dms", time_s, time_ms);
			// add_log_with_limit((const char*)log_buf);
			// add_log_with_limit("time");
			// printf("time: %d\r\n", time_s);
			// add_log_with_limit("time");
		}

		// add_log_with_limit("start");
		// add_log("start");
		// if (Uart5.ReceiveFinish)
        // {
        //     printf("Received data: ");
        //     for(i = 0; i < Uart5.RXlenth; i++){
        //         printf("%c", Uart5.Rxbuf[i]);
        //     }
		// 	lv_label_set_text(myLabel, "state 1");
        //     Uart5.ReceiveFinish = 0;
        //     Uart5.RXlenth = 0;
        //     printf("\r\n");
        // }
		// if(state == 1){
		// 	lv_label_set_text_fmt(myLabel, "size: %d", test);
		// 	// printf("state 1\n");
		// 	Uart5.ReceiveFinish = 0;
		// 	state = 0;
		// 	// Uart5.Time = 0;
		// }

		if(state == 1){
			state=0;
			if(Check(data_buf))
			{
				// max_temp_x = 0;
				// max_temp_y = 0;
				// min_temp_x = 0;
				// min_temp_y = 0;
				min_temp=500;
				max_temp=0;
				max_chart_cnt = 0;
				min_chart_cnt = 500;
				chart_max_value = 0;
				chart_min_value = 0;
				mean_temp = 0;
				variance_temp = 0;
				delta = 0;
				
				// �����¶�
				for(int i=0; i<768;)
				{
					// if(i%100==0){
					// 	sprintf(log_buf, "data: %d", i);
					// 	add_log_with_limit(log_buf);
					// }
					// �����������ֵ�ͷ���
					temp=((int16_t)data_buf[i*2+1+4]<<8|data_buf[i*2+4]);
					Temperature=(float)temp * 0.01f;
					delta = Temperature - mean_temp;
					mean_temp += delta / (i + 1);
					variance_temp += delta * (Temperature - mean_temp);
					
					if(Temperature>max_temp){
						max_temp_x = i % 32;
						max_temp_y = i / 32;
						max_temp=Temperature;
					}
					if(Temperature<min_temp){
						min_temp_x = i % 32;
						min_temp_y = i / 32;
						min_temp=Temperature;
					}
					// printf(" %.2f", Temperature);
					// float inv = myMap(Temperature,min_temp,max_temp,0,100);
					i++;
					// ÿ32�����ݻ���
					// if((i%32)==0&&i>0){
						// printf(" \r\n");
					// }
					// �����е����ݼ�����
					chart_cnt[i%32] += Temperature;
					
				}
				// ���㷽��
				variance_temp /= 768;

				for(uint8_t i = 0;i<32;i++){
					// chart_data[i] = chart_cnt[i]/24;
					// chart_data[i] = myMap(chart_cnt[i]/24,0,250,0,90);
					// printf(" %d", chart_data[i]);
					// printf(" %.2f", chart_cnt[i]/24);
					if(max_chart_cnt < chart_cnt[i]){
						max_chart_cnt = chart_cnt[i];
					}
					if(min_chart_cnt > chart_cnt[i]){
						min_chart_cnt = chart_cnt[i];
					}
				}
				// reset
				for(uint8_t i = 0;i<32;i++){
					chart_data[i] = myMap(chart_cnt[i], min_chart_cnt, max_chart_cnt, 0, 90);
					if(chart_max_value < chart_data[i]){
						chart_max_value = chart_data[i];
						// chart_max_x = 10 + i * 310 / CHART_SIZE;
					}
					if(chart_min_value > chart_data[i]){
						chart_min_value = chart_data[i];
					}
					chart_cnt[i] = 0;
					// printf(" %d", chart_data[i]);
				}
				// printf("max_x: %d max_y: %d min_x: %d min_y: %d\r\n", max_temp_x, max_temp_y, min_temp_x, min_temp_y);
				// printf("max_cnt: %f min_cnt: %f\r\n", max_chart_cnt, min_chart_cnt);
				// printf("\r\n");
				// ��ͼ
				// for(uint16_t y=0; y<24; y++)
				// {
				// 	for(uint16_t x=4; x<64+4; x=x+2)
				// 	{
				// 		temp=((int16_t)data_buf[x+1+64*y]<<8|data_buf[x+64*y]);
				// 		Temperature=(float)temp/100;
				// 		if(Temperature>max_temp)
				// 			max_temp=Temperature;
				// 		if(Temperature<min_temp)
				// 			min_temp=Temperature;
				// 		if(rest_count>20)
				// 		{
				// 			min_temp=500;
				// 			max_temp=0;
				// 			rest_count=0;
				// 		}
				// 		float inv= myMap(Temperature,min_temp,max_temp,0.5,1);
				// 		if(inv>0.75)
				// 			printf("**");
				// 		else
				// 			printf("--");

				// 	}
				// 	printf("\r\n");
				// }
							// rest_count++;
				// printf("\r\n");
				sprintf(output_max_min_temp, "max T: %.2f \tmin T: %.2f", max_temp, min_temp);
				sprintf(output_max_min_temp_xy, "max x: %d y: %d \tmin x: %d y: %d", max_temp_x, max_temp_y, min_temp_x, min_temp_y);
				sprintf(output_servo_angle, "servo 0: %d \tdegree\nservo 1: %d \tdegree", (int)(90 * (float)max_temp_x / 32), (int)(45 * (float)max_temp_y / 24));
				sprintf(output_mean_variance, "mean: %.2f \tvariance: %.2f", mean_temp, variance_temp);
				sprintf(output_time_stamp, "time: %d s %d ms", time_s, time_ms);
				lv_label_set_text(label_max_min_temp, output_max_min_temp);
				lv_label_set_text(label_max_min_temp_xy, output_max_min_temp_xy);
				lv_label_set_text(label_servo_angle, output_servo_angle);
				lv_label_set_text(label_mean_variance, output_mean_variance);
				lv_label_set_text(label_time_stamp, output_time_stamp);
				// lv_label_set_text_fmt(myLabel, "max: %f min: %f", max_temp, min_temp);
			}
		}
        

		// printf("USART3 test\r\n");
		// mlx90640_draw();
		// lv_obj_align(o_img, LV_ALIGN_TOP_LEFT, 0, i);

		// i++;
		// if(i >= 240){
		// 	i = 0;
		// }

		// SetServoAngle(1, 45);
		tp_dev.scan(0);
		// ����LED�����������������
		if(tp_dev.sta & TP_PRES_DOWN){
			LED8_ON;
		}else{
			LED8_OFF;
		}
		if(msLVGL++ >= 5){
			if(disp_finish_flag == 0){
				swap_buffers(lv_scr_act());
			}
			lv_timer_handler(); // 5ms����һ�Σ�����LVGL�Ķ�ʱ��
			msLVGL = 0;
			if(disp_finish_flag == 0){
				disp_finish_flag = 1;
			}
		}
		// delay_ms(1);
	}

	// image_free();
	return 0;
}

