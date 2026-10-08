/** EDPD controls and a separate, overlaid sensor-group graph. */
#include "lv_edpd_widgets.h"
#include "lv_modbus_connection.h"
#include "lv_sensor_profiles.h"
#include <string.h>

#define RESPONSE_TIMEOUT pdMS_TO_TICKS(600)
#define CHART_POINTS 50

typedef struct {
    lv_obj_t *value_label;
    lv_chart_series_t *series;
    uint16_t history[CHART_POINTS];
    bool valid[CHART_POINTS];
} channel_view_t;

static lv_obj_t *s_chart;
static lv_obj_t *s_start_label;
static char s_axis_text[96];
static uint16_t s_cycle_raw[SENSOR_MAX_CHANNELS];
static bool s_cycle_valid[SENSOR_MAX_CHANNELS];
static const uint32_t s_colors[SENSOR_MAX_CHANNELS] = {
    0xE53935, 0x1E88E5, 0x43A047, 0xFB8C00, 0x8E24AA, 0x00ACC1,
    0xD81B60, 0x7CB342, 0x6D4C41, 0x3949AB, 0x00897B, 0xF9A825,
};

static lv_obj_t *s_graphs;
static lv_obj_t *s_status;
static lv_obj_t *s_start;
static lv_obj_t *s_choices[SENSOR_PROFILE_COUNT];
static lv_style_t s_radio_style;
static channel_view_t s_views[SENSOR_MAX_CHANNELS];
static QueueHandle_t s_responses;
static unsigned s_selected;
static unsigned s_next_channel;
static bool s_running;
static uint32_t s_generation;
static uint32_t s_serial;

/* Keep the in-flight request until response/timeout even across sensor changes.
 * A generation check prevents it from updating the new sensor's charts. */
static bool s_pending;
static uint32_t s_pending_generation;
static unsigned s_pending_channel;
static TickType_t s_sent_at;
static modbus_request_t s_request;

static const sensor_profile_t *selected_profile(void)
{
    return &sensor_profiles[s_selected];
}

static void update_status(void)
{
    const sensor_profile_t *profile = selected_profile();
    lv_label_set_text(s_start_label, s_running ? "Modbus 중지" : "Modbus 시작");
    lv_label_set_text_fmt(s_status, "%s | Slave ID %u | FC 0x03 | %u channels\n%s",
        profile->name, (unsigned)profile->slave_id, (unsigned)profile->channel_count,
        !s_responses ? "응답 큐 생성 실패" : s_running ? "통신 중" : "연결 대기");
}

static void format_value(char *out, size_t size, uint16_t raw)
{
    const sensor_profile_t *profile = selected_profile();
    if (profile->decimals == 0) lv_snprintf(out, size, "%u", (unsigned)raw);
    else lv_snprintf(out, size, "%u.%0*u", (unsigned)(raw / profile->value_divisor),
                     (int)profile->decimals, (unsigned)(raw % profile->value_divisor));
}

static void rebuild_graphs(void)
{
    lv_obj_clean(s_graphs);
    memset(s_views, 0, sizeof(s_views));
    memset(s_cycle_valid, 0, sizeof(s_cycle_valid));
    const sensor_profile_t *profile = selected_profile();
    lv_obj_t *title = lv_label_create(s_graphs, NULL);
    lv_label_set_text_fmt(title, "%s (%s)", profile->name, profile->unit);
    s_chart = lv_chart_create(s_graphs, NULL);
    lv_obj_set_size(s_chart, lv_obj_get_width_fit(s_graphs), LV_DPX(220));
    lv_obj_set_drag_parent(s_chart, true);
    lv_obj_set_style_local_pad_left(s_chart, LV_CHART_PART_BG, LV_STATE_DEFAULT, LV_DPX(55));
    lv_obj_set_style_local_pad_right(s_chart, LV_CHART_PART_BG, LV_STATE_DEFAULT, LV_DPX(12));
    lv_chart_set_type(s_chart, LV_CHART_TYPE_LINE);
    lv_chart_set_point_count(s_chart, CHART_POINTS);
    lv_chart_set_div_line_count(s_chart, 4, 5);
    lv_chart_set_range(s_chart, 0, 10000);
    /* All channels share one time axis and one physical-value scale. */
    for (unsigned i = 0; i < profile->channel_count; i++) {
        channel_view_t *view = &s_views[i];
        lv_color_t color = lv_color_hex(s_colors[i]);
        view->series = lv_chart_add_series(s_chart, color);
        lv_chart_clear_serie(s_chart, view->series);
        view->value_label = lv_label_create(s_graphs, NULL);
        lv_obj_set_width(view->value_label, lv_obj_get_width_fit(s_graphs));
        lv_obj_set_style_local_text_color(view->value_label, LV_LABEL_PART_MAIN, LV_STATE_DEFAULT, color);
        lv_label_set_text_fmt(view->value_label, "%s: -- %s",
                             profile->channels[i].name, profile->unit);
    }
}

static void select_sensor(unsigned index)
{
    if (index >= SENSOR_PROFILE_COUNT) return;
    s_selected = index;
    s_generation++;
    s_next_channel = 0;
    for (unsigned i = 0; i < SENSOR_PROFILE_COUNT; i++)
        lv_checkbox_set_checked(s_choices[i], i == index);
    rebuild_graphs();
    update_status();
}

static void sensor_event_cb(lv_obj_t *obj, lv_event_t event)
{
    if (event != LV_EVENT_VALUE_CHANGED) return;
    for (unsigned i = 0; i < SENSOR_PROFILE_COUNT; i++) {
        if (obj != s_choices[i]) continue;
        if (i == s_selected) lv_checkbox_set_checked(obj, true);
        else select_sensor(i);
        return;
    }
}

static void start_event_cb(lv_obj_t *obj, lv_event_t event)
{
    if (event != LV_EVENT_VALUE_CHANGED) return;
    if (!s_responses) {
        lv_btn_set_state(obj, LV_BTN_STATE_RELEASED);
        return;
    }
    s_running = lv_btn_get_state(obj) == LV_BTN_STATE_CHECKED_RELEASED ||
                lv_btn_get_state(obj) == LV_BTN_STATE_CHECKED_PRESSED;
    s_generation++;
    s_next_channel = 0;
    memset(s_cycle_valid, 0, sizeof(s_cycle_valid));
    update_status();
}

static void display_sample(unsigned channel, bool valid, uint16_t raw)
{
    const sensor_profile_t *profile = selected_profile();
    s_cycle_raw[channel] = raw;
    s_cycle_valid[channel] = valid;
    if (valid) {
        char value[24];
        format_value(value, sizeof(value), raw);
        lv_label_set_text_fmt(s_views[channel].value_label, "%s: %s %s",
                             profile->channels[channel].name, value, profile->unit);
    } else {
        lv_label_set_text_fmt(s_views[channel].value_label, "%s: 응답 없음",
                             profile->channels[channel].name);
    }
}

/* Commit once per full polling sweep so every series advances by the same step.
 * Missing channels get gaps. Re-map retained raw history onto a shared coordinate
 * range, preserving small values (e.g. 0.25 Aw) and avoiding int16 overflow. */
static void commit_sweep(void)
{
    unsigned count = selected_profile()->channel_count;
    uint32_t low = 65535, high = 0;
    bool any = false;
    for (unsigned i = 0; i < count; i++) {
        channel_view_t *view = &s_views[i];
        memmove(view->history, view->history + 1, (CHART_POINTS - 1) * sizeof(view->history[0]));
        memmove(view->valid, view->valid + 1, (CHART_POINTS - 1) * sizeof(view->valid[0]));
        view->history[CHART_POINTS - 1] = s_cycle_raw[i];
        view->valid[CHART_POINTS - 1] = s_cycle_valid[i];
        for (unsigned j = 0; j < CHART_POINTS; j++) {
            if (!view->valid[j]) continue;
            if (view->history[j] < low) low = view->history[j];
            if (view->history[j] > high) high = view->history[j];
            any = true;
        }
    }
    if (!any) { low = 0; high = 10; }
    uint32_t padding = (high - low) / 10 + 1;
    low = low > padding ? low - padding : 0;
    high = high + padding < 65535 ? high + padding : 65535;
    char lower[24], middle[24], upper[24];
    format_value(lower, sizeof(lower), (uint16_t)low);
    format_value(middle, sizeof(middle), (uint16_t)((low + high) / 2));
    format_value(upper, sizeof(upper), (uint16_t)high);
    lv_snprintf(s_axis_text, sizeof(s_axis_text), "%s\n%s\n%s", upper, middle, lower);
    lv_chart_set_y_tick_texts(s_chart, s_axis_text, 2, LV_CHART_AXIS_DRAW_LAST_TICK);
    for (unsigned i = 0; i < count; i++) {
        lv_coord_t points[CHART_POINTS];
        for (unsigned j = 0; j < CHART_POINTS; j++)
            points[j] = s_views[i].valid[j]
                ? (lv_coord_t)(((uint32_t)s_views[i].history[j] - low) * 10000 / (high - low))
                : LV_CHART_POINT_DEF;
        lv_chart_set_points(s_chart, s_views[i].series, points);
    }
    memset(s_cycle_valid, 0, sizeof(s_cycle_valid));
}

static bool matches_request(const modbus_data_t *data)
{
    return data->request_id == s_request.request_id &&
           data->slave_id == s_request.slave_id &&
           data->function_code == SENSOR_FUNCTION_CODE &&
           data->reg_addr == s_request.reg_addr && data->reg_count == 1;
}

static void poll_task(lv_task_t *task)
{
    (void)task;
    if (!s_responses) return;
    bool received = false;
    modbus_data_t data;
    while (xQueueReceive(s_responses, &data, 0) == pdTRUE) {
        if (s_pending && matches_request(&data)) {
            received = true;
            break;
        }
    }
    if (s_pending && (received || xTaskGetTickCount() - s_sent_at >= RESPONSE_TIMEOUT)) {
        if (s_running && s_pending_generation == s_generation) {
            display_sample(s_pending_channel, received, received ? (uint16_t)data.values[0] : 0);
            s_next_channel = (s_pending_channel + 1) % selected_profile()->channel_count;
            if (s_next_channel == 0) commit_sweep();
        }
        s_pending = false;
    }
    if (s_pending || !s_running) return;
    const sensor_profile_t *profile = selected_profile();
    s_pending_channel = s_next_channel;
    s_pending_generation = s_generation;
    s_request = (modbus_request_t) {
        .response_queue = s_responses,
        .request_id = ++s_serial,
        .slave_id = profile->slave_id,
        .function_code = SENSOR_FUNCTION_CODE,
        .reg_addr = profile->channels[s_pending_channel].device_address,
        .reg_count = 1,
    };
    modbus_send_read_request_to(s_responses, s_request.slave_id, SENSOR_FUNCTION_CODE,
                               s_request.reg_addr, 1, s_request.request_id);
    s_sent_at = xTaskGetTickCount();
    s_pending = true;
}

void lv_edpd_widgets_create(lv_obj_t *page, lv_obj_t *graph_page)
{
    s_responses = xQueueCreate(MODBUS_QUEUE_SIZE, sizeof(modbus_data_t));
    s_running = s_pending = false;
    lv_page_set_scrl_layout(page, LV_LAYOUT_COLUMN_MID);
    lv_coord_t width = lv_page_get_width_fit(page);

    lv_obj_t *controls = lv_cont_create(page, NULL);
    lv_cont_set_layout(controls, LV_LAYOUT_COLUMN_MID);
    lv_cont_set_fit2(controls, LV_FIT_NONE, LV_FIT_TIGHT);
    lv_obj_set_width(controls, width);
    lv_obj_t *label = lv_label_create(controls, NULL);
    lv_label_set_text(label, "센서 선택");
    lv_style_init(&s_radio_style);
    lv_style_set_radius(&s_radio_style, LV_STATE_DEFAULT, LV_RADIUS_CIRCLE);
    for (unsigned i = 0; i < SENSOR_PROFILE_COUNT; i++) {
        s_choices[i] = lv_checkbox_create(controls, NULL);
        lv_checkbox_set_text(s_choices[i], sensor_profiles[i].name);
        lv_obj_add_style(s_choices[i], LV_CHECKBOX_PART_BULLET, &s_radio_style);
        lv_obj_set_event_cb(s_choices[i], sensor_event_cb);
    }
    s_start = lv_btn_create(controls, NULL);
    lv_btn_set_checkable(s_start, true);
    lv_btn_set_state(s_start, LV_BTN_STATE_RELEASED);
    lv_obj_set_event_cb(s_start, start_event_cb);
    s_start_label = lv_label_create(s_start, NULL);
    s_status = lv_label_create(controls, NULL);
    lv_label_set_long_mode(s_status, LV_LABEL_LONG_BREAK);
    lv_obj_set_width(s_status, lv_obj_get_width_fit(controls));

    lv_page_set_scrl_layout(graph_page, LV_LAYOUT_COLUMN_MID);
    s_graphs = lv_cont_create(graph_page, NULL);
    lv_cont_set_layout(s_graphs, LV_LAYOUT_COLUMN_MID);
    lv_cont_set_fit2(s_graphs, LV_FIT_NONE, LV_FIT_TIGHT);
    lv_obj_set_width(s_graphs, lv_page_get_width_fit(graph_page));
    lv_obj_set_drag_parent(s_graphs, true);
    select_sensor(0);
    lv_task_create(poll_task, 100, LV_TASK_PRIO_LOW, NULL);
}
