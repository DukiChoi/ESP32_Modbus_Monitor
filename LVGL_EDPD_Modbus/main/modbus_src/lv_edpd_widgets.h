#ifndef LV_EDPD_WIDGETS_H
#define LV_EDPD_WIDGETS_H
#include "lvgl/lvgl.h"
#ifdef __cplusplus
extern "C" {
#endif
void lv_edpd_widgets_create(lv_obj_t *controls_page, lv_obj_t **graph_pages);
void lv_edpd_select_sensor(unsigned index);
#ifdef __cplusplus
}
#endif
#endif
