/** All-sensor acquisition, Home reception status and per-group graph history. */
#include "lv_edpd_widgets.h"
#include "lv_app_font.h"
#include "lv_modbus_connection.h"
#include "lv_sensor_profiles.h"
#include <string.h>

#define SWEEP_INTERVAL pdMS_TO_TICKS(500)
#define CHART_POINTS 50
#define COMPONENT_RIGHT_SPACE 24

typedef struct {
    lv_obj_t *value_label;
    lv_chart_series_t *series;
    uint16_t history[CHART_POINTS];
    bool valid[CHART_POINTS];
} channel_view_t;

static lv_obj_t *s_chart;
static lv_obj_t *s_tooltip;
static lv_obj_t *s_start_label;
static lv_obj_t *s_baud_dropdown;
static const uint32_t s_baud_rates[] = {9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600};
static char s_axis_text[96];
static uint16_t s_all_raw[SENSOR_PROFILE_COUNT][SENSOR_MAX_CHANNELS];
static bool s_all_valid[SENSOR_PROFILE_COUNT][SENSOR_MAX_CHANNELS];
static bool s_received[SENSOR_PROFILE_COUNT];
static TickType_t s_received_at[SENSOR_PROFILE_COUNT];
static const uint32_t s_colors[SENSOR_MAX_CHANNELS] = {
    0xE53935, 0x1E88E5, 0x43A047, 0xFB8C00, 0x8E24AA, 0x00ACC1,
    0xD81B60, 0x7CB342, 0x6D4C41, 0x3949AB, 0x00897B, 0xF9A825,
};

static lv_obj_t *s_graphs;
static lv_obj_t *s_status;
static lv_obj_t *s_start;
static lv_obj_t *s_graph_pages[SENSOR_PROFILE_COUNT];
static lv_obj_t *s_info_cards[SENSOR_PROFILE_COUNT];
static channel_view_t s_all_views[SENSOR_PROFILE_COUNT][SENSOR_MAX_CHANNELS];
#define s_views s_all_views[s_selected]
static QueueHandle_t s_responses;
static unsigned s_selected;
static modbus_request_t s_blocks[SENSOR_MAX_CHANNELS];
static bool s_block_done[SENSOR_MAX_CHANNELS];
static unsigned s_block_count;
static unsigned s_block_profile[SENSOR_MAX_CHANNELS];
static unsigned s_batch_count;
static modbus_request_t s_batch_requests[SENSOR_MAX_CHANNELS];
static TickType_t s_last_start;
static bool s_has_started;
static bool s_running;
static uint32_t s_generation;
static uint32_t s_serial;

/* Keep in-flight sweeps until completion; generations reject replies from
 * before stop/restart. Tab changes never affect acquisition. */
static bool s_pending;
static uint32_t s_pending_generation;


/* Value labels use the mixed-language font, with a lowered w for A-subscript-w. */
static lv_font_t s_value_font;
static lv_font_t s_small_value_font;
static lv_font_t s_number_font;
static const lv_font_t *s_base_font;
static const uint8_t s_sub_w_bitmap[] = {0x83, 0x06, 0x4a, 0xa2, 0x80};
static bool value_glyph(const lv_font_t *font, lv_font_glyph_dsc_t *dsc,
                        uint32_t letter, uint32_t next)
{
    (void)font;
    if (letter == 'w') {
        memset(dsc, 0, sizeof(*dsc));
        dsc->bpp = 1;
        dsc->box_w = 7;
        dsc->box_h = 5;
        dsc->adv_w = 8;
        dsc->ofs_y = -1;
        return true;
    }
    const lv_font_t *base = font == &s_small_value_font && letter < 0x100
        ? &lv_font_montserrat_9 :
        font == &s_number_font && letter < 0x100 && letter != 0xB5 ? &lv_font_montserrat_12 : s_base_font;
    return lv_font_get_glyph_dsc(base, dsc, letter, next);
}
static const uint8_t *value_bitmap(const lv_font_t *font, uint32_t letter)
{
    (void)font;
    if (letter == 'w') return s_sub_w_bitmap;
    const lv_font_t *base = font == &s_small_value_font && letter < 0x100
        ? &lv_font_montserrat_9 :
        font == &s_number_font && letter < 0x100 && letter != 0xB5 ? &lv_font_montserrat_12 : s_base_font;
    return lv_font_get_glyph_bitmap(base, letter);
}

static const sensor_profile_t *selected_profile(void)
{
    return &sensor_profiles[s_selected];
}

static void update_status(void)
{
    lv_label_set_text(s_start_label, s_running ? "Modbus 중지" : "Modbus 시작");
    lv_label_set_text(s_status, !s_responses ? "Response queue unavailable" :
        s_running ? "FC03 | 500ms | RUNNING" : "FC03 | 500ms | STOPPED");
}

static void update_reception(void)
{
    TickType_t now = xTaskGetTickCount();
    for (unsigned i = 0; i < SENSOR_PROFILE_COUNT; i++) {
        bool active = s_running && s_received[i] && now - s_received_at[i] < pdMS_TO_TICKS(1500);
        lv_color_t color = lv_color_hex(active ? 0x16A34A : 0x6B7280);
        lv_obj_set_style_local_bg_color(s_info_cards[i], LV_CONT_PART_MAIN, LV_STATE_DEFAULT, color);
        lv_obj_set_style_local_bg_grad_color(s_info_cards[i], LV_CONT_PART_MAIN, LV_STATE_DEFAULT, color);
        lv_obj_set_style_local_bg_opa(s_info_cards[i], LV_CONT_PART_MAIN, LV_STATE_DEFAULT, LV_OPA_COVER);
        lv_obj_set_style_local_border_color(s_info_cards[i], LV_CONT_PART_MAIN, LV_STATE_DEFAULT, color);
        lv_obj_set_style_local_border_width(s_info_cards[i], LV_CONT_PART_MAIN, LV_STATE_DEFAULT, 1);
        lv_obj_set_style_local_border_opa(s_info_cards[i], LV_CONT_PART_MAIN, LV_STATE_DEFAULT, LV_OPA_COVER);
        lv_obj_set_style_local_text_color(s_info_cards[i], LV_CONT_PART_MAIN, LV_STATE_DEFAULT, LV_COLOR_WHITE);
    }
}

static void format_value(char *out, size_t size, uint16_t raw)
{
    const sensor_profile_t *profile = selected_profile();
    lv_snprintf(out, size, "%.*f", (int)profile->decimals,
                sensor_profile_convert_raw(profile, raw));
}

static void set_padding(lv_obj_t *obj, uint8_t part, lv_state_t state, lv_coord_t padding)
{
    lv_obj_set_style_local_pad_left(obj, part, state, padding);
    lv_obj_set_style_local_pad_right(obj, part, state, padding);
    lv_obj_set_style_local_pad_top(obj, part, state, padding);
    lv_obj_set_style_local_pad_bottom(obj, part, state, padding);
}

/* Keep the page padding symmetric; breathing room belongs inside the cards. */
static void page_layout(lv_obj_t *page)
{
    lv_page_set_scrl_layout(page, LV_LAYOUT_COLUMN_LEFT);
    lv_page_set_scrollbar_mode(page, LV_SCROLLBAR_MODE_AUTO);
    lv_obj_set_style_local_pad_left(page, LV_PAGE_PART_BG, LV_STATE_DEFAULT, 6);
    lv_obj_set_style_local_pad_right(page, LV_PAGE_PART_BG, LV_STATE_DEFAULT, 6);
    lv_obj_set_style_local_pad_top(page, LV_PAGE_PART_BG, LV_STATE_DEFAULT, 8);
    lv_obj_set_style_local_pad_bottom(page, LV_PAGE_PART_BG, LV_STATE_DEFAULT, 8);
    set_padding(page, LV_PAGE_PART_SCROLLABLE, LV_STATE_DEFAULT, 0);
    lv_obj_set_style_local_pad_inner(page, LV_PAGE_PART_SCROLLABLE, LV_STATE_DEFAULT, 10);
    lv_cont_set_fit2(lv_page_get_scrl(page), LV_FIT_PARENT, LV_FIT_TIGHT);
    lv_obj_set_drag_dir(lv_page_get_scrl(page), LV_DRAG_DIR_VER);
}

static void card_style(lv_obj_t *card)
{
    lv_obj_set_style_local_radius(card, LV_CONT_PART_MAIN, LV_STATE_DEFAULT, 10);
    lv_obj_set_style_local_border_width(card, LV_CONT_PART_MAIN, LV_STATE_DEFAULT, 1);
    lv_obj_set_style_local_border_color(card, LV_CONT_PART_MAIN, LV_STATE_DEFAULT, lv_color_hex(0x8193AC));
    lv_obj_set_style_local_border_opa(card, LV_CONT_PART_MAIN, LV_STATE_DEFAULT, LV_OPA_60);
    set_padding(card, LV_CONT_PART_MAIN, LV_STATE_DEFAULT, 8);
    lv_obj_set_style_local_pad_inner(card, LV_CONT_PART_MAIN, LV_STATE_DEFAULT, 10);
    lv_obj_set_drag_parent(card, true);
}

static lv_obj_t *row_create(lv_obj_t *parent, lv_coord_t width)
{
    lv_obj_t *row = lv_cont_create(parent, NULL);
    lv_cont_set_layout(row, LV_LAYOUT_ROW_MID);
    lv_cont_set_fit2(row, LV_FIT_NONE, LV_FIT_TIGHT);
    lv_obj_set_width(row, width);
    lv_obj_set_style_local_bg_opa(row, LV_CONT_PART_MAIN, LV_STATE_DEFAULT, LV_OPA_TRANSP);
    lv_obj_set_style_local_border_width(row, LV_CONT_PART_MAIN, LV_STATE_DEFAULT, 0);
    set_padding(row, LV_CONT_PART_MAIN, LV_STATE_DEFAULT, 0);
    lv_obj_set_style_local_pad_inner(row, LV_CONT_PART_MAIN, LV_STATE_DEFAULT, 8);
    lv_obj_set_drag_parent(row, true);
    return row;
}

/* Shorten only the presentation name; wire mapping and CSV names stay intact. */
static void channel_display_name(unsigned channel, char *out, size_t size)
{
    const sensor_profile_t *profile = selected_profile();
    const char *name = profile->channels[channel].name;
    const char *wear = strcmp(profile->name, "B-WACS") == 0 ? strstr(name, "Wear_") : NULL;
    if (wear) lv_snprintf(out, size, "%.*s%s", (int)(wear - name), name, wear + 5);
    else lv_snprintf(out, size, "%s", name);
}

/* One compact legend: units are shown in the graph title above it. */
static void update_tooltip(bool initial)
{
    const sensor_profile_t *profile = selected_profile();
    char text[256] = "";
    size_t used = 0;
    for (unsigned i = 0; i < profile->channel_count; i++) {
        if (!profile->channels[i].show_tooltip) continue;
        char name[40];
        channel_display_name(i, name, sizeof(name));
        char value[24] = "--";
        if (!initial && s_views[i].valid[CHART_POINTS - 1])
            format_value(value, sizeof(value), s_views[i].history[CHART_POINTS - 1]);
        used += lv_snprintf(text + used, sizeof(text) - used, "%s#%06X %s: %s#",
            used ? "\n" : "", (unsigned)s_colors[i], name, value);
    }
    lv_label_set_text(s_tooltip, text);
    lv_obj_align(s_tooltip, s_chart, LV_ALIGN_IN_TOP_RIGHT, -8, 8);
}

static void rebuild_graphs(void)
{
    lv_obj_clean(s_graphs);

    const sensor_profile_t *profile = selected_profile();
    lv_coord_t content_width = lv_obj_get_width_fit(s_graphs) - COMPONENT_RIGHT_SPACE;
    lv_obj_t *title = lv_label_create(s_graphs, NULL);
    lv_obj_set_style_local_text_font(title, LV_LABEL_PART_MAIN, LV_STATE_DEFAULT, &s_value_font);
    lv_label_set_text_fmt(title, profile->channel_count == 1 ? "%s: -- %s" : "%s (%s)",
        profile->name, profile->unit);
    s_chart = lv_chart_create(s_graphs, NULL);
    lv_obj_set_size(s_chart, content_width, LV_DPX(176));
    lv_obj_set_drag_parent(s_chart, true);
    lv_obj_set_style_local_pad_left(s_chart, LV_CHART_PART_BG, LV_STATE_DEFAULT, 80);
    lv_obj_set_style_local_pad_right(s_chart, LV_CHART_PART_BG, LV_STATE_DEFAULT, LV_DPX(12));
    lv_obj_set_style_local_pad_left(s_chart, LV_CHART_PART_SERIES_BG, LV_STATE_DEFAULT, 4);
    lv_chart_set_y_tick_length(s_chart, 4, 0);
    lv_chart_set_type(s_chart, LV_CHART_TYPE_LINE);
    lv_chart_set_point_count(s_chart, CHART_POINTS);
    lv_chart_set_div_line_count(s_chart, 4, 5);
    lv_chart_set_range(s_chart, 0, 10000);
    lv_snprintf(s_axis_text, sizeof(s_axis_text), "%.2f\n%.2f\n%.2f", profile->sensor_max,
                (profile->sensor_min + profile->sensor_max) / 2, profile->sensor_min);
    lv_chart_set_y_tick_texts(s_chart, s_axis_text, 2, LV_CHART_AXIS_DRAW_LAST_TICK);
    s_tooltip = lv_label_create(s_chart, NULL);
    lv_obj_set_style_local_text_font(s_tooltip, LV_LABEL_PART_MAIN, LV_STATE_DEFAULT, &lv_font_montserrat_12);
    lv_label_set_recolor(s_tooltip, true);
    set_padding(s_tooltip, LV_LABEL_PART_MAIN, LV_STATE_DEFAULT, 4);
    lv_obj_set_style_local_bg_color(s_tooltip, LV_LABEL_PART_MAIN, LV_STATE_DEFAULT,
        lv_obj_get_style_bg_color(s_chart, LV_CHART_PART_BG));
    lv_obj_set_style_local_bg_opa(s_tooltip, LV_LABEL_PART_MAIN, LV_STATE_DEFAULT, LV_OPA_COVER);
    lv_obj_set_style_local_border_width(s_tooltip, LV_LABEL_PART_MAIN, LV_STATE_DEFAULT, 1);
    lv_obj_set_style_local_border_color(s_tooltip, LV_LABEL_PART_MAIN, LV_STATE_DEFAULT, lv_color_hex(0x8193AC));
    lv_obj_set_style_local_radius(s_tooltip, LV_LABEL_PART_MAIN, LV_STATE_DEFAULT, 4);
    lv_obj_set_drag_parent(s_tooltip, true);
    update_tooltip(true);
    lv_obj_set_hidden(s_tooltip, profile->channel_count == 1);
    /* All channels share one time axis and one physical-value scale. */
    lv_obj_t *row = NULL;
    lv_coord_t column_width = (content_width - 6) / 4;
    for (unsigned i = 0; i < profile->channel_count; i++) {
        if (profile->channel_count > 1 && i % 4 == 0) {
            row = row_create(s_graphs, content_width);
            lv_obj_set_style_local_pad_inner(row, LV_CONT_PART_MAIN, LV_STATE_DEFAULT, 2);
        }
        channel_view_t *view = &s_views[i];
        lv_color_t color = lv_color_hex(s_colors[i]);
        view->series = lv_chart_add_series(s_chart, color);
        lv_chart_clear_serie(s_chart, view->series);
        if (profile->channel_count == 1) {
            view->value_label = title;
            continue;
        }
        lv_obj_t *card = lv_cont_create(row, NULL);
        lv_cont_set_layout(card, LV_LAYOUT_COLUMN_LEFT);
        lv_cont_set_fit2(card, LV_FIT_NONE, LV_FIT_TIGHT);
        lv_obj_set_width(card, column_width);
        card_style(card);
        set_padding(card, LV_CONT_PART_MAIN, LV_STATE_DEFAULT, 2);
        lv_obj_set_style_local_pad_left(card, LV_CONT_PART_MAIN, LV_STATE_DEFAULT, 0);
        lv_obj_set_style_local_pad_right(card, LV_CONT_PART_MAIN, LV_STATE_DEFAULT, 0);
        lv_obj_set_style_local_radius(card, LV_CONT_PART_MAIN, LV_STATE_DEFAULT, 6);
        lv_obj_set_style_local_border_color(card, LV_CONT_PART_MAIN, LV_STATE_DEFAULT, color);
        lv_obj_set_style_local_pad_inner(card, LV_CONT_PART_MAIN, LV_STATE_DEFAULT, 1);
        lv_obj_t *name_label = lv_label_create(card, NULL);
        lv_obj_set_style_local_text_font(name_label, LV_LABEL_PART_MAIN, LV_STATE_DEFAULT, &lv_font_montserrat_12);
        lv_obj_set_style_local_text_letter_space(name_label, LV_LABEL_PART_MAIN, LV_STATE_DEFAULT, -1);
        lv_obj_set_style_local_text_color(name_label, LV_LABEL_PART_MAIN, LV_STATE_DEFAULT, color);
        char name[40];
        channel_display_name(i, name, sizeof(name));
        lv_label_set_text(name_label, name);
        lv_obj_set_drag_parent(name_label, true);
        view->value_label = lv_label_create(card, NULL);
        lv_obj_set_style_local_text_font(view->value_label, LV_LABEL_PART_MAIN, LV_STATE_DEFAULT, &s_number_font);
        lv_obj_set_style_local_text_letter_space(view->value_label, LV_LABEL_PART_MAIN, LV_STATE_DEFAULT, -1);
        lv_label_set_long_mode(view->value_label, LV_LABEL_LONG_BREAK);
        lv_obj_set_width(view->value_label, lv_obj_get_width_fit(card));
        lv_obj_set_drag_parent(view->value_label, true);
        lv_obj_set_style_local_text_color(view->value_label, LV_LABEL_PART_MAIN, LV_STATE_DEFAULT, color);
        lv_label_set_text_fmt(view->value_label, "--%s", profile->unit);

    }
}

/* Sort by address and merge only explicitly mapped, consecutive registers.
 * In particular, do not assume BTM's gaps at 11009/11010 are readable. */
static void build_blocks(void)
{
    const sensor_profile_t *profile = selected_profile();
    uint32_t next = 0;
    while (1) {
        uint32_t start = 65536;
        for (unsigned i = 0; i < profile->channel_count; i++) {
            uint16_t addr = profile->channels[i].device_address;
            if (addr >= next && addr < start) start = addr;
        }
        if (start == 65536) break;
        unsigned count = 1;
        while (count < MODBUS_MAX_REGS && start + count < 65536) {
            bool found = false;
            for (unsigned i = 0; i < profile->channel_count; i++)
                if (profile->channels[i].device_address == start + count) found = true;
            if (!found) break;
            count++;
        }
        s_block_profile[s_block_count] = s_selected;
        s_blocks[s_block_count++] = (modbus_request_t) {
            .response_queue = s_responses, .slave_id = profile->slave_id,
            .function_code = SENSOR_FUNCTION_CODE, .reg_addr = start, .reg_count = count,
        };
        next = start + count;
    }
}

static void render_graph(void);
static void select_sensor(unsigned index)
{
    if (index >= SENSOR_PROFILE_COUNT) return;
    s_selected = index;
    lv_obj_set_parent(s_graphs, lv_page_get_scrl(s_graph_pages[index]));
    rebuild_graphs();
    render_graph();
}

void lv_edpd_select_sensor(unsigned index)
{
    if (index < SENSOR_PROFILE_COUNT && index != s_selected) select_sensor(index);
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
    memset(s_all_valid, 0, sizeof(s_all_valid));
    memset(s_received, 0, sizeof(s_received));
    update_reception();
    update_status();
}

static void display_sample(unsigned channel, bool valid, uint16_t raw)
{
    const sensor_profile_t *profile = selected_profile();
    if (valid) {
        char value[24];
        format_value(value, sizeof(value), raw);
        if (profile->channel_count == 1)
            lv_label_set_text_fmt(s_views[channel].value_label, "%s: %s %s", profile->name, value, profile->unit);
        else
            lv_label_set_text_fmt(s_views[channel].value_label, "%s%s", value, profile->unit);
    } else {
        if (profile->channel_count == 1)
            lv_label_set_text_fmt(s_views[channel].value_label, "%s: 응답 없음", profile->name);
        else
            lv_label_set_text(s_views[channel].value_label, "--");
    }

}

/* Commit the complete sweep at once. Sensor bounds define the fixed Y axis;
 * clipping the plot never clips the displayed current values. */
static void render_graph(void)
{
    const sensor_profile_t *profile = selected_profile();
    for (unsigned i = 0; i < profile->channel_count; i++) {
        channel_view_t *view = &s_views[i];
        if (s_has_started && profile->serial_enabled) display_sample(i, view->valid[CHART_POINTS - 1], view->history[CHART_POINTS - 1]);
        lv_coord_t points[CHART_POINTS];
        for (unsigned j = 0; j < CHART_POINTS; j++) {
            if (!view->valid[j]) { points[j] = LV_CHART_POINT_DEF; continue; }
            double value = sensor_profile_convert_raw(profile, view->history[j]);
            double position = (value - profile->sensor_min) / (profile->sensor_max - profile->sensor_min);
            if (position < 0) position = 0;
            if (position > 1) position = 1;
            points[j] = (lv_coord_t)(position * 10000 + 0.5);
        }
        lv_chart_set_points(s_chart, view->series, points);
    }
    update_tooltip(false);
}

static void commit_sweep(void)
{
    for (unsigned p = 0; p < SENSOR_PROFILE_COUNT; p++) {
        if (!sensor_profiles[p].serial_enabled) continue;
        for (unsigned i = 0; i < sensor_profiles[p].channel_count; i++) {
            channel_view_t *view = &s_all_views[p][i];
            memmove(view->history, view->history + 1, (CHART_POINTS - 1) * sizeof(view->history[0]));
            memmove(view->valid, view->valid + 1, (CHART_POINTS - 1) * sizeof(view->valid[0]));
            view->history[CHART_POINTS - 1] = s_all_raw[p][i];
            view->valid[CHART_POINTS - 1] = s_all_valid[p][i];
        }
    }
    render_graph();
}

static void poll_task(lv_task_t *task)
{
    (void)task;
    if (!s_responses) return;
    modbus_data_t data;
    while (xQueueReceive(s_responses, &data, 0) == pdTRUE) {
        if (!s_pending) continue;
        for (unsigned b = 0; b < s_batch_count; b++) {
            const modbus_request_t *req = &s_batch_requests[b];
            if (s_block_done[b] || data.request_id != req->request_id ||
                data.slave_id != req->slave_id || data.function_code != SENSOR_FUNCTION_CODE ||
                data.reg_addr != req->reg_addr || data.reg_count != req->reg_count) continue;
            s_block_done[b] = true;
            if (s_running && s_pending_generation == s_generation) {
                unsigned p = s_block_profile[b];
                const sensor_profile_t *profile = &sensor_profiles[p];
                if (data.valid) {
                    s_received[p] = true;
                    s_received_at[p] = xTaskGetTickCount();
                }
                for (unsigned ch = 0; ch < profile->channel_count; ch++) {
                    uint32_t addr = profile->channels[ch].device_address;
                    if (addr < req->reg_addr || addr >= (uint32_t)req->reg_addr + req->reg_count) continue;
                    s_all_valid[p][ch] = data.valid;
                    s_all_raw[p][ch] = data.valid ? (uint16_t)data.values[addr - req->reg_addr] : 0;
                }
            }
            bool all_done = true;
            for (unsigned i = 0; i < s_batch_count; i++) all_done = all_done && s_block_done[i];
            if (all_done && data.batch_complete) {
                s_pending = false;
                if (s_running && s_pending_generation == s_generation) commit_sweep();
            }
            break;
        }
    }
    update_reception();
    TickType_t now = xTaskGetTickCount();
    if (s_pending || !s_running ||
        (s_has_started && now - s_last_start < SWEEP_INTERVAL)) return;
    modbus_request_t requests[SENSOR_MAX_CHANNELS];
    for (unsigned i = 0; i < s_block_count; i++) {
        requests[i] = s_blocks[i];
        requests[i].request_id = ++s_serial;
    }
    if (!modbus_send_batch_at_baud(requests, s_block_count,
        s_baud_rates[lv_dropdown_get_selected(s_baud_dropdown)])) return;
    memcpy(s_batch_requests, requests, s_block_count * sizeof(requests[0]));
    s_batch_count = s_block_count;
    memset(s_block_done, 0, sizeof(s_block_done));
    memset(s_all_valid, 0, sizeof(s_all_valid));
    s_pending_generation = s_generation;
    s_last_start = now;
    s_has_started = true;
    s_pending = true;
}

void lv_edpd_widgets_create(lv_obj_t *page, lv_obj_t **graph_pages)
{
    s_base_font = lv_theme_get_font_normal();
    s_value_font = *s_base_font;
    s_value_font.get_glyph_dsc = value_glyph;
    s_value_font.get_glyph_bitmap = value_bitmap;
    s_small_value_font = lv_font_montserrat_9;
    s_small_value_font.get_glyph_dsc = value_glyph;
    s_small_value_font.get_glyph_bitmap = value_bitmap;
    s_number_font = lv_font_montserrat_12;
    s_number_font.get_glyph_dsc = value_glyph;
    s_number_font.get_glyph_bitmap = value_bitmap;
    s_responses = xQueueCreate(MODBUS_QUEUE_SIZE, sizeof(modbus_data_t));
    s_running = s_pending = s_has_started = false;
    page_layout(page);
    lv_obj_set_style_local_pad_inner(page, LV_PAGE_PART_SCROLLABLE, LV_STATE_DEFAULT, 4);
    lv_obj_set_style_local_pad_top(page, LV_PAGE_PART_BG, LV_STATE_DEFAULT, 6);
    lv_coord_t width = lv_page_get_width_fit(page);

    lv_obj_t *controls = lv_cont_create(page, NULL);
    lv_cont_set_layout(controls, LV_LAYOUT_COLUMN_LEFT);
    lv_cont_set_fit2(controls, LV_FIT_NONE, LV_FIT_TIGHT);
    lv_obj_set_width(controls, width);
    card_style(controls);
    lv_obj_set_style_local_pad_top(controls, LV_CONT_PART_MAIN, LV_STATE_DEFAULT, 3);
    lv_obj_set_style_local_pad_bottom(controls, LV_CONT_PART_MAIN, LV_STATE_DEFAULT, 3);
    lv_obj_set_style_local_pad_inner(controls, LV_CONT_PART_MAIN, LV_STATE_DEFAULT, 4);
    lv_coord_t inner_width = lv_obj_get_width_fit(controls) - COMPONENT_RIGHT_SPACE;
    lv_coord_t column_width = (inner_width - 8) / 2;
    lv_obj_t *speed_label = lv_label_create(controls, NULL);
    lv_label_set_text(speed_label, "보드레이트");
    lv_obj_t *speed_row = row_create(controls, inner_width);
    s_baud_dropdown = lv_dropdown_create(speed_row, NULL);
    lv_dropdown_set_options(s_baud_dropdown, "9600\n19200\n38400\n57600\n115200\n230400\n460800\n921600");
    lv_dropdown_set_selected(s_baud_dropdown, 4);
	//기본 드롭다운 115200으로ㅋ
    lv_dropdown_set_max_height(s_baud_dropdown, 150);
    lv_obj_set_width(s_baud_dropdown, column_width);
    lv_obj_set_style_local_pad_top(s_baud_dropdown, LV_DROPDOWN_PART_MAIN, LV_STATE_DEFAULT, 5);
    lv_obj_set_style_local_pad_bottom(s_baud_dropdown, LV_DROPDOWN_PART_MAIN, LV_STATE_DEFAULT, 5);
    lv_obj_set_style_local_text_font(s_baud_dropdown, LV_DROPDOWN_PART_MAIN, LV_STATE_DEFAULT, &lv_font_montserrat_16);
    lv_obj_set_style_local_text_font(s_baud_dropdown, LV_DROPDOWN_PART_LIST, LV_STATE_DEFAULT, &lv_font_montserrat_16);
    s_start = lv_btn_create(speed_row, NULL);
    lv_btn_set_fit(s_start, LV_FIT_NONE);
    lv_obj_set_size(s_start, column_width, 30);
    lv_obj_set_style_local_radius(s_start, LV_BTN_PART_MAIN, LV_STATE_DEFAULT, 8);
    lv_btn_set_checkable(s_start, true);
    lv_btn_set_state(s_start, LV_BTN_STATE_RELEASED);
    lv_obj_set_event_cb(s_start, start_event_cb);
    lv_obj_set_drag_parent(s_start, true);
    s_start_label = lv_label_create(s_start, NULL);
    s_status = lv_label_create(controls, NULL);
    lv_obj_set_style_local_text_font(s_status, LV_LABEL_PART_MAIN, LV_STATE_DEFAULT, &lv_font_montserrat_12);
    lv_label_set_long_mode(s_status, LV_LABEL_LONG_BREAK);
    lv_obj_set_width(s_status, inner_width);

    lv_obj_t *row = NULL;
    for (unsigned i = 0; i < SENSOR_PROFILE_COUNT; i++) {
        const sensor_profile_t *profile = &sensor_profiles[i];
        if (i % 2 == 0) row = row_create(page, width - COMPONENT_RIGHT_SPACE);
        lv_obj_t *card = lv_cont_create(row, NULL);
        s_info_cards[i] = card;
        lv_cont_set_layout(card, LV_LAYOUT_COLUMN_LEFT);
        lv_cont_set_fit2(card, LV_FIT_NONE, LV_FIT_TIGHT);
        lv_obj_set_width(card, (width - COMPONENT_RIGHT_SPACE - 8) / 2);
        card_style(card);
        lv_obj_set_click(card, false);
        lv_obj_set_style_local_pad_top(card, LV_CONT_PART_MAIN, LV_STATE_DEFAULT, 3);
        lv_obj_set_style_local_pad_bottom(card, LV_CONT_PART_MAIN, LV_STATE_DEFAULT, 3);
        lv_obj_set_style_local_pad_inner(card, LV_CONT_PART_MAIN, LV_STATE_DEFAULT, 0);
        lv_obj_t *name = lv_label_create(card, NULL);
        lv_obj_set_style_local_text_font(name, LV_LABEL_PART_MAIN, LV_STATE_DEFAULT, &lv_font_montserrat_12);
        lv_label_set_text(name, profile->name);
        lv_obj_t *details = lv_label_create(card, NULL);
        lv_obj_set_style_local_text_font(details, LV_LABEL_PART_MAIN, LV_STATE_DEFAULT, &lv_font_montserrat_12);
        lv_label_set_text_fmt(details, "%uch | ID %u%s",
            profile->channel_count, profile->slave_id, profile->serial_enabled ? "" : " | OFF");
        lv_obj_set_drag_parent(name, true);
        lv_obj_set_drag_parent(details, true);
    }
    for (unsigned i = 0; i < SENSOR_PROFILE_COUNT; i++) {
        s_graph_pages[i] = graph_pages[i];
        page_layout(graph_pages[i]);
    }
    lv_obj_t *graph_page = graph_pages[0];
    page_layout(graph_page);
    s_graphs = lv_cont_create(graph_page, NULL);
    lv_cont_set_layout(s_graphs, LV_LAYOUT_COLUMN_LEFT);
    lv_cont_set_fit2(s_graphs, LV_FIT_NONE, LV_FIT_TIGHT);
    lv_obj_set_width(s_graphs, lv_page_get_width_fit(graph_page));
    card_style(s_graphs);
    s_block_count = 0;
    for (s_selected = 0; s_selected < SENSOR_PROFILE_COUNT; s_selected++)
        if (selected_profile()->serial_enabled) build_blocks();
    select_sensor(0);
    update_status();
    update_reception();
    lv_task_create(poll_task, 10, LV_TASK_PRIO_LOW, NULL);
}
