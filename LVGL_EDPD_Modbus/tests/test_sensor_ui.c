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
    assert(!s_running); poll_task(NULL); assert(sends == 0);
    set_running(true); poll_task(NULL);
    assert(sends == 1 && sent.count == 1 && sent.requests[0].reg_addr == 11083 && sent.requests[0].reg_count == 10);
    respond(0, true, 600); poll_task(NULL);
    assert(!s_pending && sends == 1);
    assert(strcmp(lv_label_get_text(s_views[0].value_label), "PS_Cyl1:\n60.00 °C") == 0);
    assert(strstr(lv_label_get_text(s_tooltip), "PS_Cyl1: 60.00"));
    assert(strstr(lv_label_get_text(s_tooltip), "Exh_Cyl1:"));
    assert(!strstr(lv_label_get_text(s_tooltip), "PS_Cyl2"));
    lv_tabview_set_tab_act(tabs, 0, LV_ANIM_OFF);
    lv_event_send(tabs, LV_EVENT_VALUE_CHANGED, NULL);
    assert(s_running && s_selected == 0); /* Home preserves acquisition. */
    assert(count_type(edpd, "lv_checkbox") == 0);
    assert(sent.baud_rate == 9600);
    lv_dropdown_set_selected(s_baud_dropdown, 7);
    tick = 499; poll_task(NULL); assert(sends == 1);
    tick = 500; poll_task(NULL); assert(sends == 2 && s_pending);
    assert(sent.baud_rate == 921600);
    tick = 1100; poll_task(NULL); assert(sends == 2); /* Do not overlap an overrun. */
    respond(0, true, 610); poll_task(NULL); assert(sends == 3 && s_last_start == 1100);
    set_running(false); finish_batch(); assert(!s_pending && sends == 3);

    choose(1); set_running(true); tick = 1600; poll_task(NULL);
    assert(sent.count == 3);
    assert(sent.requests[0].reg_addr == 11003 && sent.requests[0].reg_count == 6);
    assert(sent.requests[1].reg_addr == 11011 && sent.requests[1].reg_count == 5);
    assert(sent.requests[2].reg_addr == 11321 && sent.requests[2].reg_count == 1);
    lv_tabview_set_tab_act(tabs, 2, LV_ANIM_OFF);
    respond(0, true, 500); poll_task(NULL);
    assert(!s_views[5].valid[CHART_POINTS - 1]); /* Atomic display of the complete sweep. */
    respond(1, true, 723); poll_task(NULL);
    respond(2, true, 900); poll_task(NULL);
    assert(!s_pending);
    assert(strcmp(lv_label_get_text(s_views[0].value_label), "CrkP_Cyl1:\n72.30 °C") == 0);
    assert(strstr(lv_label_get_text(s_tooltip), "CrkP_Cyl1: 72.30"));
    assert(strstr(lv_label_get_text(s_tooltip), "MB_Fore: 50.00"));
    assert(s_views[0].history[CHART_POINTS - 1] == 723);
    assert(s_views[5].history[CHART_POINTS - 1] == 500);
    assert(s_views[11].history[CHART_POINTS - 1] == 900);
    assert(strcmp(s_axis_text, "200.00\n100.00\n0.00") == 0);
    assert(s_views[0].series->points[CHART_POINTS - 1] == 3615);
    tick = 2100; poll_task(NULL);
    respond(0, true, 999); queued[(queue_write - 1) % 64].request_id--; poll_task(NULL);
    assert(!s_block_done[0]);
    respond(0, false, 0); respond(1, true, 730); respond(2, true, 901); poll_task(NULL);
    assert(!s_views[5].valid[CHART_POINTS - 1] && s_views[0].valid[CHART_POINTS - 1]);
    assert(s_views[5].series->points[CHART_POINTS - 1] == LV_CHART_POINT_DEF);
    assert(strstr(lv_label_get_text(s_tooltip), "MB_Fore: --"));

    tick = 2600; poll_task(NULL); choose(4); finish_batch();
    assert(!s_pending && !s_views[0].valid[CHART_POINTS - 1]); /* Old group ignored. */
    tick = 3100; poll_task(NULL);
    assert(sent.count == 1 && sent.requests[0].reg_addr == 11129);
    respond(0, true, 576); poll_task(NULL);
    assert(strcmp(lv_label_get_text(s_views[0].value_label), "AVM:\n5.76 mm") == 0);
    assert(s_views[0].series->points[CHART_POINTS - 1] == 2880);
    choose(3); tick = 3600; poll_task(NULL);
    assert(sent.requests[0].reg_addr == 20 && sent.requests[0].slave_id == 1);
    respond(0, true, 19); poll_task(NULL);
    assert(strcmp(lv_label_get_text(s_views[0].value_label), "Water Activity:\n0.19 Aw") == 0);
    assert(s_views[0].series->points[CHART_POINTS - 1] == 1900);
    tick = 4100; poll_task(NULL); respond(0, true, 26); poll_task(NULL);
    assert(strcmp(lv_label_get_text(s_views[0].value_label), "Water Activity:\n0.26 Aw") == 0);
    choose(2); assert(!s_running && s_block_count == 1 && s_blocks[0].reg_count == 10);
    unsigned before = sends; tick += 1000; poll_task(NULL); assert(sends == before);
    assert(lv_btn_get_state(s_start) == LV_BTN_STATE_DISABLED);
    /* All sensor ranges clip only plot coordinates, never current values. */
    display_sample(0, true, 8000); commit_sweep();
    assert(s_views[0].series->points[CHART_POINTS - 1] == 10000);
    assert(strstr(lv_label_get_text(s_views[0].value_label), "8000.00 µm"));
    choose(0); set_running(true); accept_batch = false; poll_task(NULL);
    assert(!s_pending && sends == before);
    accept_batch = true; poll_task(NULL); assert(s_pending); set_running(false); finish_batch();
    size_t free_after_first = 0;
    for (unsigned cycle = 0; cycle < 20; cycle++) {
        for (unsigned profile = 0; profile < 5; profile++) {
            choose(profile); lv_tick_inc(500); lv_task_handler();
            unsigned representatives = 0;
            for (unsigned ch = 0; ch < sensor_profiles[profile].channel_count; ch++)
                representatives += sensor_profiles[profile].channels[ch].show_tooltip;
            assert(representatives == (profile < 3 ? 2 : 1));
            assert(lv_obj_get_parent(s_tooltip) == s_chart);
            assert(count_type(edpd, "lv_chart") == 0 && count_type(tabs, "lv_chart") == 1);
        }
        /* Finish theme transitions and deferred layout before comparing memory. */
        lv_tick_inc(500); lv_task_handler();
        lv_tick_inc(500); lv_task_handler();
        assert(lv_mem_test() == LV_RES_OK);
        lv_mem_monitor_t memory; lv_mem_monitor(&memory);
        if (!cycle) free_after_first = memory.free_size;
        /* The allocator may split/merge a few small headers as cards wrap. */
        else assert(memory.free_size + 128 >= free_after_first);
    }
    puts("PASS: 500ms sweep starts, no overlaps, contiguous FC03 blocks, atomic display, stale/failed replies, fixed ranges, scaled values, mixed fonts, 100 switches");
    return 0;
}
