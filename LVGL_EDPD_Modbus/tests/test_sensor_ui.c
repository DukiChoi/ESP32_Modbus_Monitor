/* Real LVGL host test; only UART/FreeRTOS are simulated. */
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include "../main/modbus_src/lv_edpd_widgets.c"
#include "../main/modbus_src/lv_modbus.c"
#include "lvgl/src/lv_core/lv_debug.h"

static TickType_t tick;
static modbus_data_t queued;
static bool have_response;
static unsigned sends;
static modbus_request_t sent;
void modbus_connection_init(void) {}
TickType_t xTaskGetTickCount(void) { return tick; }
QueueHandle_t xQueueCreate(unsigned length, unsigned size)
{ (void)length; assert(size == sizeof(modbus_data_t)); return (void *)1; }
int xQueueReceive(QueueHandle_t q, void *out, unsigned wait)
{
    (void)wait; assert(q == (void *)1);
    if (!have_response) return 0;
    memcpy(out, &queued, sizeof(queued)); have_response = false; return pdTRUE;
}
void modbus_send_read_request_to(QueueHandle_t q, uint8_t slave, uint8_t fc,
                                uint16_t addr, uint16_t count, uint32_t id)
{
    assert(q == (void *)1 && fc == 3 && count == 1);
    sent = (modbus_request_t){.response_queue=q,.slave_id=slave,.function_code=fc,
                             .reg_addr=addr,.reg_count=count,.request_id=id};
    sends++;
}
static void respond(uint16_t raw)
{
    queued = (modbus_data_t){.request_id=sent.request_id,.slave_id=sent.slave_id,
        .function_code=sent.function_code,.reg_addr=sent.reg_addr,.reg_count=1};
    queued.values[0] = (int16_t)raw;
    have_response = true;
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
static void choose(unsigned index)
{
    lv_checkbox_set_checked(s_choices[index], true);
    lv_event_send(s_choices[index], LV_EVENT_VALUE_CHANGED, NULL);
    assert(s_selected == index);
    lv_chart_ext_t *ext = lv_obj_get_ext_attr(s_chart);
    assert(_lv_ll_get_len(&ext->series_ll) == selected_profile()->channel_count);
    for (unsigned i = 0; i < SENSOR_PROFILE_COUNT; i++)
        assert(lv_checkbox_is_checked(s_choices[i]) == (i == index));
}
static void flush(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *colors)
{ (void)area; (void)colors; lv_disp_flush_ready(drv); }
void __wrap_lv_debug_log_error(const char *msg, uint64_t value)
{ fprintf(stderr, "LVGL assertion: %s (%llu)\n", msg, (unsigned long long)value); abort(); }

int main(void)
{
    lv_init();
    static lv_color_t pixels[320];
    static lv_disp_buf_t buf;
    lv_disp_buf_init(&buf, pixels, NULL, 320);
    lv_disp_drv_t drv; lv_disp_drv_init(&drv);
    drv.flush_cb = flush; drv.buffer = &buf; drv.hor_res = 320; drv.ver_res = 480;
    assert(lv_disp_drv_register(&drv));
    lv_modbus();
    lv_obj_t *tabs = NULL, *child = NULL;
    while ((child = lv_obj_get_child(lv_scr_act(), child)))
        if (lv_debug_check_obj_type(child, "lv_tabview")) tabs = child;
    assert(tabs && lv_tabview_get_tab_count(tabs) == 2);
    lv_obj_t *edpd = lv_tabview_get_tab(tabs, 0), *graph = lv_tabview_get_tab(tabs, 1);
    assert(count_type(edpd, "lv_chart") == 0 && count_type(graph, "lv_chart") == 1);
    assert(!s_running); poll_task(NULL); assert(sends == 0);
    set_running(true); poll_task(NULL);
    assert(sent.slave_id == 2 && sent.reg_addr == 11083);
    choose(1); respond(600); poll_task(NULL);
    assert(sent.reg_addr == 11011 && !s_cycle_valid[0]);
    /* Read BTM from the graph tab; all series advance only after the full sweep. */
    lv_tabview_set_tab_act(tabs, 1, LV_ANIM_OFF);
    for (unsigned i = 0; i < 12; i++) {
        assert(sent.reg_addr == sensor_profiles[1].channels[i].device_address);
        respond(560 + i * 10); poll_task(NULL);
        assert(s_views[0].valid[CHART_POINTS - 1] == (i == 11));
    }
    assert(strcmp(lv_label_get_text(s_views[0].value_label), "CrkP_Cyl1: 56.0 'C") == 0);
    for (unsigned i = 0; i < 12; i++) {
        assert(s_views[i].valid[CHART_POINTS - 1]);
        assert(s_views[i].history[CHART_POINTS - 1] == 560 + i * 10);
        if (i) assert(s_views[i].series->points[CHART_POINTS - 1] > s_views[i-1].series->points[CHART_POINTS - 1]);
    }
    set_running(false);
    unsigned before = sends; respond(999); poll_task(NULL);
    assert(sends == before && s_views[0].history[CHART_POINTS - 1] == 560);
    set_running(true); poll_task(NULL);
    respond(999); queued.request_id--; poll_task(NULL); assert(s_pending);
    tick += 601; poll_task(NULL);
    assert(strstr(lv_label_get_text(s_views[0].value_label), "응답 없음"));
    for (unsigned i = 1; i < 12; i++) { respond(600); poll_task(NULL); }
    assert(!s_views[0].valid[CHART_POINTS - 1]);
    assert(s_views[0].series->points[CHART_POINTS - 1] == LV_CHART_POINT_DEF);
    /* Group changes keep acquisition active and ignore its old in-flight reply. */
    choose(2); respond(600); poll_task(NULL);
    assert(sent.slave_id == 0 && sent.reg_addr == 10004);
    for (unsigned i = 0; i < 10; i++) { respond(2800); poll_task(NULL); }
    assert(strcmp(lv_label_get_text(s_views[0].value_label), "CYL1_Wear_FS: 2800 um") == 0);
    choose(3); respond(2800); poll_task(NULL);
    assert(sent.slave_id == 1 && sent.reg_addr == 20);
    respond(25); poll_task(NULL);
    assert(strcmp(lv_label_get_text(s_views[0].value_label), "Water Activity: 0.25 Aw") == 0);
    choose(4); respond(25); poll_task(NULL);
    assert(sent.slave_id == 2 && sent.reg_addr == 11129);
    respond(60); poll_task(NULL);
    assert(strcmp(lv_label_get_text(s_views[0].value_label), "AVM: 0.60 mm") == 0);
    respond(65535); poll_task(NULL);
    assert(s_views[0].history[CHART_POINTS - 1] == 65535);
    assert(s_views[0].series->points[CHART_POINTS - 1] <= 10000);
    set_running(false); respond(42); poll_task(NULL);
    size_t free_after_first = 0;
    for (unsigned cycle = 0; cycle < 20; cycle++) {
        for (unsigned profile = 0; profile < 5; profile++) {
            choose(profile); lv_tick_inc(500); lv_task_handler();
            assert(count_type(edpd, "lv_chart") == 0 && count_type(graph, "lv_chart") == 1);
        }
        assert(lv_mem_test() == LV_RES_OK);
        lv_mem_monitor_t memory; lv_mem_monitor(&memory);
        if (!cycle) free_after_first = memory.free_size;
        else assert(memory.free_size == free_after_first);
    }
    puts("PASS: two tabs, one overlaid chart, 12 aligned BTM channels, unit conversion, start/stop, graph-tab polling, stale replies, timeouts, 100 group switches");
    return 0;
}
