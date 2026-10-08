/** Home connection controls and per-sensor graph tabs. */
#include "lv_modbus.h"
#include "lv_modbus_connection.h"
#include "lv_edpd_widgets.h"
#include "lv_app_font.h"
#include "lv_sensor_profiles.h"

static void tab_changed(lv_obj_t *tabs, lv_event_t event)
{
    if (event != LV_EVENT_VALUE_CHANGED) return;
    unsigned tab = lv_tabview_get_tab_act(tabs);
    if (tab > 0) lv_edpd_select_sensor(tab - 1);
}

void lv_modbus(void)
{
    lv_theme_t *theme = LV_THEME_DEFAULT_INIT(lv_theme_get_color_primary(),
        lv_theme_get_color_secondary(), lv_theme_get_flags(),
        &lv_app_font, &lv_app_font, &lv_app_font, &lv_app_font);
    lv_theme_set_act(theme);
    modbus_connection_init();
    lv_obj_t *tv = lv_tabview_create(lv_scr_act(), NULL);
    lv_obj_set_style_local_pad_top(tv, LV_TABVIEW_PART_TAB_BG, LV_STATE_DEFAULT, 0);
    lv_obj_set_style_local_pad_bottom(tv, LV_TABVIEW_PART_TAB_BG, LV_STATE_DEFAULT, 0);
    lv_obj_set_style_local_pad_top(tv, LV_TABVIEW_PART_TAB_BTN, LV_STATE_DEFAULT, 8);
    lv_obj_set_style_local_pad_bottom(tv, LV_TABVIEW_PART_TAB_BTN, LV_STATE_DEFAULT, 8);
    lv_obj_set_style_local_text_font(tv, LV_TABVIEW_PART_TAB_BTN, LV_STATE_DEFAULT, &lv_font_montserrat_12);
    lv_obj_set_style_local_text_letter_space(tv, LV_TABVIEW_PART_TAB_BTN, LV_STATE_DEFAULT, -1);
    lv_obj_t *home = lv_tabview_add_tab(tv, "Home");
    lv_obj_t *pages[SENSOR_PROFILE_COUNT];
    for (unsigned i = 0; i < SENSOR_PROFILE_COUNT; i++)
        pages[i] = lv_tabview_add_tab(tv, sensor_profiles[i].name);
    lv_edpd_widgets_create(home, pages);
    lv_obj_set_event_cb(tv, tab_changed);
}
