/* Real LVGL host test; only UART/FreeRTOS are simulated. */
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "../main/modbus_src/lv_edpd_widgets.c"
#include "../main/modbus_src/lv_modbus.c"
#include "lvgl/src/lv_core/lv_debug.h"

static TickType_t tick;
static modbus_data_t queued[64];
static unsigned queue_read, queue_write;
static unsigned sends;
static modbus_batch_t sent;
static bool accept_batch = true;
void modbus_connection_init(void) {}
TickType_t xTaskGetTickCount(void) { return tick; }
QueueHandle_t xQueueCreate(unsigned length, unsigned size)
{ (void)length; assert(size == sizeof(modbus_data_t)); return (void *)1; }
int xQueueReceive(QueueHandle_t q, void *out, unsigned wait)
{
    (void)wait; assert(q == (void *)1);
    if (queue_read == queue_write) return 0;
    memcpy(out, &queued[queue_read++ % 64], sizeof(modbus_data_t)); return pdTRUE;
}
bool modbus_send_batch_at_baud(const modbus_request_t *requests, uint8_t count, uint32_t baud_rate)
{
    if (!accept_batch) return false;
    assert(count > 0 && count <= MODBUS_MAX_BATCH_REQUESTS);
    sent.count = count;
    sent.baud_rate = baud_rate;
    for (unsigned i = 0; i < count; i++) {
        assert(requests[i].function_code == 3 && requests[i].reg_count <= MODBUS_MAX_REGS);
        sent.requests[i] = requests[i];
    }
    sends++;
    return true;
}
static void respond(unsigned block, bool valid, uint16_t first)
{
    const modbus_request_t *req = &sent.requests[block];
    modbus_data_t *data = &queued[queue_write++ % 64];
    *data = (modbus_data_t){.request_id=req->request_id,.slave_id=req->slave_id,
        .function_code=req->function_code,.reg_addr=req->reg_addr,.reg_count=req->reg_count,
        .valid=valid,.batch_complete=block + 1 == sent.count};
    for (unsigned j = 0; j < req->reg_count; j++) data->values[j] = first + j;
}
static void finish_batch(void)
{
    for (unsigned b = 0; b < sent.count; b++) respond(b, true, 600);
    poll_task(NULL);
}
static unsigned count_type(lv_obj_t *parent, const char *type)
{
    unsigned count = lv_debug_check_obj_type(parent, type) ? 1 : 0;
    lv_obj_t *child = NULL;
    while ((child = lv_obj_get_child(parent, child))) count += count_type(child, type);
    return count;
}
static void set_running(bool running)
{
    lv_btn_set_state(s_start, running ? LV_BTN_STATE_CHECKED_RELEASED : LV_BTN_STATE_RELEASED);
    lv_event_send(s_start, LV_EVENT_VALUE_CHANGED, NULL);
    assert(s_running == running);
    assert(strcmp(lv_label_get_text(s_start_label), running ? "Modbus 중지" : "Modbus 시작") == 0);
}
static lv_obj_t *test_tabs;
static void choose(unsigned index)
{
    lv_tabview_set_tab_act(test_tabs, index + 1, LV_ANIM_OFF);
    lv_event_send(test_tabs, LV_EVENT_VALUE_CHANGED, NULL);
    assert(s_selected == index);
    assert(lv_obj_get_parent(s_graphs) == lv_page_get_scrl(s_graph_pages[index]));
    lv_chart_ext_t *ext = lv_obj_get_ext_attr(s_chart);
    assert(_lv_ll_get_len(&ext->series_ll) == selected_profile()->channel_count);
}
static void flush(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *colors)
{ (void)area; (void)colors; lv_disp_flush_ready(drv); }
void __wrap_lv_debug_log_error(const char *msg, uint64_t value)
{ fprintf(stderr, "LVGL assertion: %s (%llu)\n", msg, (unsigned long long)value); abort(); }

int main(void)
{
    const uint16_t raw_examples[] = {600, 560, 2798, 25, 60};
    const double expected_values[] = {60.0, 56.0, 2798.0, 0.25, 0.60};
    for (unsigned i = 0; i < SENSOR_PROFILE_COUNT; i++)
        assert(fabs(sensor_profile_convert_raw(&sensor_profiles[i], raw_examples[i]) - expected_values[i]) < 1e-9);
    /* Verify offset is applied after scaling, including negative results. */
    sensor_profile_t adjusted = sensor_profiles[1];
    adjusted.offset = 2.5;
    assert(fabs(sensor_profile_convert_raw(&adjusted, 560) - 58.5) < 1e-9);
    adjusted.offset = -60.0;
    assert(fabs(sensor_profile_convert_raw(&adjusted, 560) - (-4.0)) < 1e-9);
    lv_init();
    static lv_color_t pixels[320];
    static lv_disp_buf_t buf;
    lv_disp_buf_init(&buf, pixels, NULL, 320);
    lv_disp_drv_t drv; lv_disp_drv_init(&drv);
    drv.flush_cb = flush; drv.buffer = &buf; drv.hor_res = LV_HOR_RES_MAX; drv.ver_res = LV_VER_RES_MAX;
    assert(lv_disp_drv_register(&drv));
    lv_modbus();
    lv_obj_t *tabs = NULL, *child = NULL;
    while ((child = lv_obj_get_child(lv_scr_act(), child)))
        if (lv_debug_check_obj_type(child, "lv_tabview")) tabs = child;
    assert(tabs && lv_tabview_get_tab_count(tabs) == 6);
    test_tabs = tabs;
    lv_obj_t *edpd = lv_tabview_get_tab(tabs, 0), *graph = lv_tabview_get_tab(tabs, 1);
    assert(count_type(edpd, "lv_chart") == 0 && count_type(tabs, "lv_chart") == 1);
    assert(lv_font_get_glyph_bitmap(&lv_app_font, 'A') == lv_font_get_glyph_bitmap(&lv_font_montserrat_16, 'A'));
    assert(lv_font_get_glyph_bitmap(&lv_app_font, 0xD55C) == lv_font_get_glyph_bitmap(&lv_font_galmuri9, 0xD55C));
    lv_font_glyph_dsc_t glyph;
    assert(lv_font_get_glyph_dsc(&lv_app_font, &glyph, 0xB0, 0));
    assert(lv_font_get_glyph_dsc(&lv_app_font, &glyph, 0xB5, 0));
    assert(lv_theme_get_font_normal() == &lv_app_font);
    const double expected_min[] = {0, 0, 1000, 0, 0};
    const double expected_max[] = {300, 200, 7000, 1, 20};
    for (unsigned i = 0; i < SENSOR_PROFILE_COUNT; i++) {
        const sensor_profile_t *profile = &sensor_profiles[i];
        assert(profile->sensor_min == expected_min[i] && profile->sensor_max == expected_max[i]);
        assert(profile->decimals == 2);
    }
    assert(!s_running && s_block_count == 6);
    set_running(true); poll_task(NULL);
    assert(sent.count == 6 && sent.baud_rate == s_baud_rates[lv_dropdown_get_selected(s_baud_dropdown)]);
    const unsigned addresses[] = {11083,11003,11011,11321,20,11129};
    for (unsigned b = 0; b < 6; b++) assert(sent.requests[b].reg_addr == addresses[b]);
    respond(0, true, 723); poll_task(NULL);
    assert(s_received[0] && !s_received[1]);
    assert(lv_color_to32(lv_obj_get_style_bg_color(s_info_cards[0], LV_CONT_PART_MAIN)) == lv_color_to32(lv_color_hex(0x16A34A)));
    choose(2); assert(s_running && s_pending && sent.count == 6);
    choose(1);
    assert(!s_all_views[0][0].valid[CHART_POINTS - 1]);
    for (unsigned b = 1; b < 6; b++) respond(b, b != 4, b == 5 ? 576 : 500);
    poll_task(NULL);
    assert(!s_pending && s_all_views[0][0].history[CHART_POINTS - 1] == 723);
    assert(s_all_views[1][0].valid[CHART_POINTS - 1]);
    assert(!s_all_views[3][0].valid[CHART_POINTS - 1]);
    assert(s_all_views[4][0].history[CHART_POINTS - 1] == 576);
    choose(0); assert(strstr(lv_label_get_text(s_tooltip), "PS_Cyl1: 72.30"));
    choose(4); assert(strstr(lv_label_get_text(s_tooltip), "AVM: 5.76"));
    assert(strcmp(lv_label_get_text(s_views[0].value_label), "AVM: 5.76 mm") == 0);
    assert(lv_obj_get_parent(s_views[0].value_label) == s_graphs);
    assert(lv_obj_get_hidden(s_tooltip));
    lv_tabview_set_tab_act(tabs, 0, LV_ANIM_OFF);
    lv_event_send(tabs, LV_EVENT_VALUE_CHANGED, NULL); assert(s_running);
    tick=499; poll_task(NULL); assert(sends==1);
    lv_dropdown_set_selected(s_baud_dropdown,7);
    tick=500; poll_task(NULL); assert(sends==2 && sent.baud_rate==921600);
    tick=1100; poll_task(NULL); assert(sends==2); /* no overlap */
    choose(0); /* Rendering must not overwrite partial responses. */
    respond(0,true,800); poll_task(NULL); choose(1); choose(0);
    assert(s_all_raw[0][0]==800);
    for(unsigned b=1;b<6;b++) respond(b,false,0);
    poll_task(NULL); assert(sends==3 && s_pending);
    assert(s_all_views[0][0].history[CHART_POINTS-1]==800);
    tick=2600; poll_task(NULL); /* orange expires after 1500 ms without a good response */
    assert(lv_color_to32(lv_obj_get_style_bg_color(s_info_cards[0],LV_CONT_PART_MAIN))==lv_color_to32(lv_color_hex(0x6B7280)));
    set_running(false); finish_batch(); assert(!s_pending);
    assert(s_all_views[0][0].history[CHART_POINTS-1]==800);
    assert(lv_color_to32(lv_obj_get_style_bg_color(s_info_cards[1],LV_CONT_PART_MAIN))==lv_color_to32(lv_color_hex(0x6B7280)));
    set_running(true); accept_batch=false; unsigned before=sends;
    poll_task(NULL); assert(!s_pending && sends==before);
    accept_batch=true; poll_task(NULL); assert(s_pending);
    set_running(false); finish_batch();
    for(unsigned cycle=0;cycle<100;cycle++) {
        choose(cycle%5); lv_tick_inc(500); lv_task_handler();
        assert(count_type(tabs,"lv_chart")==1);
        assert(lv_mem_test()==LV_RES_OK);
        assert(s_all_views[0][0].history[CHART_POINTS-1]==800);
    }
    puts("PASS: six-block all-sensor sweeps, 500ms/no overlap, tab-independent histories, partial responses, green/gray reception/expiry/stop, baud selection, 100 switches");
    return 0;
}
