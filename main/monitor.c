#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "bsp/esp-bsp.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "history.h"
#include "esp_intr_alloc.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "usb/cdc_acm_host.h"

enum { WAITING, METADATA, SAMPLES };
enum { SAMPLE_US = 60, GROUP_SIZE = 17 };
enum { CMD_START = 6, CMD_STOP = 7, CMD_OUTPUT = 12, CMD_VOLTAGE = 13, CMD_MODE = 17, CMD_METADATA = 25 };

enum {
    UI_BACKGROUND, UI_CHART, UI_SURFACE, UI_GRID, UI_CONTROL, UI_PRESSED,
    UI_TEXT, UI_MUTED, UI_CURRENT, UI_VOLTAGE, UI_POWER, UI_ACTION,
    UI_ON_BUTTON, UI_FAULT, UI_FILL_VOLTAGE, UI_FILL_BASELINE, UI_FILL_POWER,
    UI_FILL_UA, UI_FILL_MA, UI_FILL_A, UI_INK_A, UI_PEAK, UI_AVG,
    UI_BADGE_V, UI_BADGE_MAX, UI_BADGE_AVG, UI_BADGE_OUT, UI_BADGE_MIN,
    UI_BADGE_TEXT, UI_OUTPUT_ACTIVE, UI_OUTPUT_ACTIVE_TEXT,
    UI_BASELINE_INK, UI_CARD_CAPTION, UI_CHART_BORDER, UI_COLOR_COUNT
};
enum { THEME_DARK, THEME_LIGHT };

static const uint32_t theme_colors[2][UI_COLOR_COUNT] = {
    [THEME_DARK] = {
        0x151718, 0x1b1f20, 0x232727, 0x363e3e, 0x303737, 0x41494a,
        0xf1f1ed, 0xaeb8b6, 0x95e4ba, 0xffce8a, 0xd8c0fc, 0x344a46,
        0x285a4c, 0xe7988d,
        0x594021, 0x264953, 0x47365f, 0x28523d, 0x55451f, 0x5d302a,
        0xffaca0, 0xe6a16b, 0x78bbdb,
        0xe7b878, 0xe6a16b, 0x78bbdb, 0xed887e, 0x77d1aa,
        0x172322, 0xa03633, 0xfff8f5, 0xaee2e9, 0xdddcd5, 0x303737
    },
    [THEME_LIGHT] = {
        0xeeeae2, 0xfffdf8, 0xfffaf1, 0xd8d4c9, 0xe2ded6, 0xdad3c6,
        0x26312f, 0x52605a, 0x1e6747, 0x75470b, 0x603583, 0xd7e5da,
        0xbad9c9, 0x9c4035,
        0xe9ce9e, 0xbfdde0, 0xdcccee, 0xbfdec9, 0xead6a2, 0xecc8c0,
        0x91332d, 0x853b17, 0x205e77,
        0xe9c89b, 0xedb893, 0xa7cedf, 0xf0afa5, 0xa3d9be,
        0x172322, 0xa03633, 0xfff8f5, 0x225a65, 0x414944, 0xd8d4c9
    }
};
static unsigned active_theme = THEME_DARK;
static lv_style_t theme_bg[UI_COLOR_COUNT], theme_text[UI_COLOR_COUNT], theme_border[UI_COLOR_COUNT];

static lv_color_t ui_color(unsigned color)
{
    return lv_color_hex(theme_colors[active_theme][color]);
}

static void set_theme_colors(unsigned theme)
{
    active_theme = theme;
    for (unsigned i = 0; i < UI_COLOR_COUNT; ++i) {
        lv_color_t color = ui_color(i);
        lv_style_set_bg_color(&theme_bg[i], color);
        lv_style_set_text_color(&theme_text[i], color);
        lv_style_set_border_color(&theme_border[i], color);
    }
}

static void init_theme(void)
{
    for (unsigned i = 0; i < UI_COLOR_COUNT; ++i) {
        lv_style_init(&theme_bg[i]);
        lv_style_init(&theme_text[i]);
        lv_style_init(&theme_border[i]);
    }
    set_theme_colors(THEME_DARK);
}

typedef struct {
    int64_t stamp;
    float current;
} Reading;

static const char *TAG = "power_meter";
static SemaphoreHandle_t lock;
static QueueHandle_t output;
static int phase;
static bool disconnected;
enum { POWER_UNAVAILABLE, POWER_OFF, POWER_ON, POWER_UNKNOWN };
static int power_state;
static int power_request = -1;
static int output_mv;
static uint32_t power_first_tick;
static bool power_pending;
static char metadata[4096];
static size_t metadata_length;
static int64_t last_sample;
static uint32_t lost, output_drops;
static history_bucket_t *history_ring, *history_snapshot;
static history_bucket_t history_live, snapshot_head;
static int above_threshold;

static const float default_r[] = {1031.64, 101.65, 10.15, 0.94, 0.099, 0.009};
static float r[6], offset[6], average, slow_average, group_sum;
static int previous_range, expected, after_spike, consecutive, group_count;
static int64_t sample_time, group_start;
static bool filter_ready;
static uint32_t word;
static unsigned word_bytes;

static lv_obj_t *chart, *chart_state;
static lv_obj_t *voltage_value, *set_value, *out_value, *max_value, *min_value, *avg_1_value, *avg_10_value;
static lv_obj_t *main_power_button, *theme_button, *theme_picker;
static lv_obj_t *theme_options[2];
static int graph_voltage_mv;
static lv_obj_t *baseline_value, *average_power_value, *average_current_value, *average_current_title;
static float baseline_samples[1000];
static int snapshot_output_mv;
static uint32_t snapshot_first_tick;
static history_bucket_t plot[PLOT_COLUMNS], next_plot[PLOT_COLUMNS];
static float plot_min, plot_max;
static float log_top = 200000.0f;
static bool log_scale = true;
static unsigned span = 10;
static bool graph_dirty = true;
static const unsigned spans[] = {2, 10, 30, 60};
static unsigned span_index = 1;
enum { TAP_WINDOW, TAP_AXIS, TAP_PAUSE, TAP_CLEAR };
enum { CURRENT_UA, CURRENT_MA, CURRENT_A };
static unsigned average_unit = CURRENT_UA;
static const unsigned current_fills[] = {UI_FILL_UA, UI_FILL_MA, UI_FILL_A};
static const unsigned current_inks[] = {UI_CURRENT, UI_VOLTAGE, UI_INK_A};
static bool paused;
static lv_obj_t *power_panel, *power_target, *power_info;
static lv_obj_t *power_slider;
static int selected_mv;
static void reset_stats(void)
{
    memset(history_ring, 0, HISTORY_BUCKETS * sizeof(*history_ring));
    history_live = (history_bucket_t){0};
    above_threshold = 0;
}

static void reset_session(void)
{
    phase = WAITING;
    disconnected = false;
    power_state = POWER_UNAVAILABLE;
    power_request = -1;
    output_mv = 0;
    power_first_tick = 0;
    power_pending = false;
    metadata_length = 0;
    metadata[0] = 0;
    last_sample = 0;
    word = 0;
    word_bytes = 0;
    filter_ready = false;
    previous_range = expected = -1;
    after_spike = consecutive = group_count = 0;
    group_sum = 0;
    sample_time = 0;
    lost = output_drops = 0;
    reset_stats();
}

static void load_calibration(void)
{
    memcpy(r, default_r, sizeof(r));
    memset(offset, 0, sizeof(offset));
    for (char *line = metadata; line && *line; ) {
        char *next = strchr(line, '\n');
        if (next) {
            *next++ = 0;
        }
        unsigned i;
        float value;
        if (sscanf(line, "r%u: %f", &i, &value) == 2 && i < 6 &&
            isfinite(value) && value > 0) {
            r[i] = value;
        }
        if (sscanf(line, "o%u: %f", &i, &value) == 2 && i < 6 && isfinite(value)) {
            offset[i] = value;
        }
        line = next;
    }
}

static void decode_sample(uint32_t raw, int64_t received_at)
{
    int counter = (raw >> 18) & 63;
    int range = (raw >> 14) & 7;
    if (sample_time == 0) {
        sample_time = received_at;
    } else {
        unsigned missing = (counter - expected) & 63;
        lost += missing;
        sample_time += (missing + 1) * SAMPLE_US;
        if (missing) {
            group_count = 0;
            group_sum = 0;
        }
    }
    expected = (counter + 1) & 63;
    if (range >= 6) {
        return;
    }

    float amps = ((raw & 0x3fff) * 4.0f - offset[range]) *
                 ((1.8f / 163840.0f) / r[range]);
    float previous_fast = average;
    float previous_slow = slow_average;
    average = filter_ready ? 0.18f * amps + 0.82f * average : amps;
    slow_average = filter_ready ? 0.06f * amps + 0.94f * slow_average : amps;
    if (previous_range < 0) {
        previous_range = range;
    }
    if (previous_range != range || after_spike > 0) {
        if (previous_range != range) {
            consecutive = 0;
            after_spike = 3;
        } else {
            ++consecutive;
        }
        if (range == 5) {
            if (consecutive < 2) {
                average = previous_fast;
                slow_average = previous_slow;
            }
            amps = slow_average;
        } else {
            amps = average;
        }
        --after_spike;
    }
    previous_range = range;
    filter_ready = true;

    float current = amps * 1e6f;
    history_add(history_ring, &history_live, &above_threshold,
                (uint32_t)(sample_time / HISTORY_STEP_US), current);
    last_sample = received_at;
    if (group_count == 0) {
        group_start = sample_time;
    }
    group_sum += current;
    if (++group_count == GROUP_SIZE) {
        const Reading reading = {group_start, group_sum / GROUP_SIZE};
        if (xQueueSend(output, &reading, 0) != pdPASS) {
            ++output_drops;
        }
        group_count = 0;
        group_sum = 0;
    }
}

static bool on_data(const uint8_t *data, size_t size, void *arg)
{
    int64_t now = esp_timer_get_time();
    xSemaphoreTake(lock, portMAX_DELAY);
    if (phase == METADATA && size < sizeof(metadata) - metadata_length) {
        memcpy(metadata + metadata_length, data, size);
        metadata_length += size;
        metadata[metadata_length] = 0;
    } else if (phase == SAMPLES) {
        for (size_t i = 0; i < size; ++i) {
            word |= (uint32_t)data[i] << (8 * word_bytes++);
            if (word_bytes == 4) {
                decode_sample(word, now);
                word_bytes = 0;
                word = 0;
            }
        }
    }
    xSemaphoreGive(lock);
    return true;
}

static void on_event(const cdc_acm_host_dev_event_data_t *event, void *arg)
{
    if (event->type == CDC_ACM_HOST_DEVICE_DISCONNECTED ||
        event->type == CDC_ACM_HOST_ERROR) {
        xSemaphoreTake(lock, portMAX_DELAY);
        disconnected = true;
        power_state = POWER_UNAVAILABLE;
        power_request = -1;
        output_mv = 0;
        power_first_tick = 0;
        power_pending = false;
        phase = WAITING;
        last_sample = 0;
        xSemaphoreGive(lock);
    }
}

static void on_usb_device(usb_device_handle_t device)
{
    const usb_device_desc_t *description;
    if (usb_host_get_device_descriptor(device, &description) == ESP_OK) {
        ESP_LOGW(TAG, "USB device %04x:%04x, class %02x",
                 description->idVendor, description->idProduct, description->bDeviceClass);
    }
}

static void usb_events(void *arg)
{
    for (;;) {
        uint32_t flags;
        usb_host_lib_handle_events(portMAX_DELAY, &flags);
        if (flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
            usb_host_device_free_all();
        }
    }
}

static esp_err_t send_command(cdc_acm_dev_hdl_t device, uint8_t command)
{
    return cdc_acm_host_data_tx_blocking(device, &command, 1, 500);
}

static esp_err_t set_output_enabled(cdc_acm_dev_hdl_t device, bool enabled)
{
    const uint8_t command[] = {CMD_OUTPUT, enabled ? 1 : 0};
    return cdc_acm_host_data_tx_blocking(device, command, sizeof(command), 500);
}

static int apply_power_request(cdc_acm_dev_hdl_t device, int mv)
{
    if (mv >= 500 && mv <= 5000) {
        const uint8_t source_mode[] = {CMD_MODE, 2};
        const uint8_t voltage[] = {CMD_VOLTAGE, mv >> 8, mv & 255};
        if (cdc_acm_host_data_tx_blocking(device, source_mode, sizeof(source_mode), 500) == ESP_OK &&
            cdc_acm_host_data_tx_blocking(device, voltage, sizeof(voltage), 500) == ESP_OK &&
            set_output_enabled(device, true) == ESP_OK) {
            return POWER_ON;
        }
        ESP_LOGW(TAG, "Source output request failed; sending OFF");
    }
    return set_output_enabled(device, false) == ESP_OK ? POWER_OFF : POWER_UNKNOWN;
}

static bool configure_meter(cdc_acm_dev_hdl_t device)
{
    cdc_acm_line_coding_t settings = {
        .dwDTERate = 115200, .bCharFormat = 0, .bParityType = 0, .bDataBits = 8
    };
    esp_err_t result = cdc_acm_host_line_coding_set(device, &settings);
    if (result != ESP_OK) {
        ESP_LOGW(TAG, "USB line coding: %s", esp_err_to_name(result));
        return false;
    }
    result = cdc_acm_host_set_control_line_state(device, true, true);
    if (result != ESP_OK) {
        ESP_LOGW(TAG, "USB control line state: %s", esp_err_to_name(result));
        return false;
    }
    if (set_output_enabled(device, false) != ESP_OK) {
        ESP_LOGW(TAG, "Could not disable output on connect");
        return false;
    }
    if (send_command(device, CMD_STOP) != ESP_OK) {
        ESP_LOGW(TAG, "Stop command failed");
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(300));
    xSemaphoreTake(lock, portMAX_DELAY);
    phase = METADATA;
    xSemaphoreGive(lock);
    if (send_command(device, CMD_METADATA) != ESP_OK) {
        ESP_LOGW(TAG, "Metadata command failed");
        return false;
    }
    for (int i = 0; i < 60; ++i) {
        vTaskDelay(pdMS_TO_TICKS(50));
        xSemaphoreTake(lock, portMAX_DELAY);
        if (disconnected) {
            xSemaphoreGive(lock);
            return false;
        }
        char *end = strstr(metadata, "END");
        if (end) {
            *end = 0;
            load_calibration();
            phase = SAMPLES;
            xSemaphoreGive(lock);
            esp_err_t start = send_command(device, CMD_START);
            ESP_LOGW(TAG, "Calibration received, start command: %s", esp_err_to_name(start));
            return start == ESP_OK;
        }
        xSemaphoreGive(lock);
    }
    ESP_LOGW(TAG, "Timed out waiting for calibration metadata");
    return false;
}

static void meter_task(void *arg)
{
    const cdc_acm_host_device_config_t config = {
        .connection_timeout_ms = 1000, .out_buffer_size = 64,
        .in_buffer_size = 512, .event_cb = on_event, .data_cb = on_data
    };
    for (;;) {
        xSemaphoreTake(lock, portMAX_DELAY);
        reset_session();
        xSemaphoreGive(lock);
        cdc_acm_dev_hdl_t device;
        if (cdc_acm_host_open(0x15A2, 0x300A, 0, &config, &device) != ESP_OK) {
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }
        if (configure_meter(device)) {
            xSemaphoreTake(lock, portMAX_DELAY);
            if (!disconnected) {
                power_state = POWER_OFF;
            }
            xSemaphoreGive(lock);
            for (;;) {
                vTaskDelay(pdMS_TO_TICKS(100));
                xSemaphoreTake(lock, portMAX_DELAY);
                bool stop = disconnected;
                int request = power_request;
                power_request = -1;
                xSemaphoreGive(lock);
                if (stop) {
                    break;
                }
                if (request >= 0) {
                    int state = apply_power_request(device, request);
                    xSemaphoreTake(lock, portMAX_DELAY);
                    if (!disconnected) {
                        power_state = state;
                        output_mv = state == POWER_ON ? request : 0;
                        power_first_tick = history_live.n ? history_live.tick + 1 : 0;
                        power_pending = false;
                    }
                    xSemaphoreGive(lock);
                }
            }
        }
        cdc_acm_host_close(device);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

static void stream_task(void *arg)
{
    Reading reading;
    for (;;) {
        if (xQueueReceive(output, &reading, portMAX_DELAY) == pdPASS) {
            printf("PM,%" PRId64 ",%.3f\n", reading.stamp, reading.current);
        }
    }
}

static void format_current(char *text, size_t size, double current_ua)
{
    if (fabs(current_ua) < 1000.0) {
        snprintf(text, size, "%.3f uA", current_ua);
    } else {
        snprintf(text, size, "%.3f mA", current_ua / 1000.0);
    }
}

static void update_label(lv_obj_t *label, const char *text)
{
    if (strcmp(lv_label_get_text(label), text) != 0) {
        lv_label_set_text(label, text);
    }
}

static void update_tint(lv_obj_t *label, uint32_t color)
{
    lv_color_t tint = ui_color(color);
    if (!lv_color_eq(lv_obj_get_style_text_color(label, 0), tint)) {
        lv_obj_set_style_text_color(label, tint, 0);
    }
}

static uint32_t output_tint(int state, bool pending)
{
    if (pending) return UI_VOLTAGE;
    if (state == POWER_ON) return UI_INK_A;
    if (state == POWER_UNKNOWN) return UI_FAULT;
    return UI_MUTED;
}

static void refresh_power(int state, bool pending)
{
    if (lv_obj_is_hidden(power_panel)) {
        return;
    }
    char text[44];
    snprintf(text, sizeof(text), "Setpoint: %.2f V", selected_mv / 1000.0);
    update_label(power_target, text);
    const char *message = "USB not ready";
    if (pending) {
        message = "Sending command...";
    } else if (state == POWER_OFF) {
        message = "Output OFF";
    } else if (state == POWER_ON) {
        message = "Output ON requested";
    } else if (state == POWER_UNKNOWN) {
        message = "Output unknown. Send OFF again.";
    }
    update_label(power_info, message);
    update_tint(power_info, output_tint(state, pending));
    lv_obj_set_state(power_slider, LV_STATE_DISABLED, pending || state != POWER_OFF);
}

static void refresh_output_button(int state, bool pending)
{
    const char *icon = pending ? LV_SYMBOL_REFRESH :
                       state == POWER_UNKNOWN ? LV_SYMBOL_WARNING : LV_SYMBOL_POWER;
    update_label(lv_obj_get_child(main_power_button, 0), icon);
    lv_obj_set_state(main_power_button, LV_STATE_CHECKED, state == POWER_ON && !pending);
    lv_obj_set_state(main_power_button, LV_STATE_DISABLED,
                     pending || state == POWER_UNAVAILABLE || (state == POWER_OFF && !selected_mv));
}

static void draw_text(lv_layer_t *layer, int x, int y, int width, const char *text, uint32_t color)
{
    lv_draw_label_dsc_t label;
    lv_draw_label_dsc_init(&label);
    label.color = ui_color(color);
    label.text = text;
    label.text_local = true;
    const lv_area_t area = {x, y, x + width - 1, y + 19};
    lv_draw_label(layer, &label, &area);
}

static void draw_line(lv_layer_t *layer, lv_draw_line_dsc_t *line,
                      int x1, int y1, int x2, int y2)
{
    line->p1 = (lv_point_precise_t){x1, y1};
    line->p2 = (lv_point_precise_t){x2, y2};
    lv_draw_line(layer, line);
}

static int plot_y(float value, int top, int bottom)
{
    float fraction;
    if (log_scale) {
        fraction = log10f(fmaxf(value, 1.0f)) / log10f(log_top);
    } else {
        fraction = (value - plot_min) / (plot_max - plot_min);
    }
    fraction = fmaxf(0, fminf(1, fraction));
    return bottom - lroundf(fraction * (bottom - top));
}

static void draw_graph(lv_event_t *event)
{
    lv_layer_t *layer = lv_event_get_layer(event);
    lv_area_t area;
    lv_obj_get_coords(chart, &area);
    const int left = area.x1 + 31, right = area.x2 - 70;
    const int top = area.y1 + 15, bottom = area.y2 - 32;

    lv_draw_line_dsc_t line;
    lv_draw_line_dsc_init(&line);
    line.color = ui_color(UI_GRID);
    line.width = 1;
    for (unsigned i = 0; i < (log_scale ? 9U : 5U); ++i) {
        float tick = log_scale ? powf(10, i) : plot_min + i * (plot_max - plot_min) / 4;
        if (log_scale && tick > log_top) break;
        int y = plot_y(tick, top, bottom);
        char text[28];
        snprintf(text, sizeof(text), tick >= 1000 ? (log_scale ? "%.0f mA" : "%.1f mA") : "%.3g uA",
                 tick >= 1000 ? tick / 1000 : tick);
        draw_line(layer, &line, left, y, right, y);
        draw_text(layer, right + 5, y - 8, 64, text, UI_CURRENT);
    }
    for (int i = 0; i <= 4; ++i) {
        int x = left + (right - left) * i / 4;
        char text[24];
        snprintf(text, sizeof(text), "%.3g s", (float)span * (i - 4) / 4);
        draw_line(layer, &line, x, top, x, bottom);
        draw_text(layer, x - 13, bottom + 4, 52, text, UI_MUTED);
    }

    for (int volts = 0; volts <= 5; ++volts) {
        char text[8];
        snprintf(text, sizeof(text), "%d", volts);
        int y = bottom - volts * (bottom - top) / 5;
        draw_text(layer, area.x1 + 7, y - 8, 20, text, UI_VOLTAGE);
    }
    if (graph_voltage_mv > 0) {
        int y = bottom - (bottom - top) * graph_voltage_mv / 5000;
        line.color = ui_color(UI_VOLTAGE);
        line.width = 2;
        draw_line(layer, &line, left, y, right, y);
        draw_text(layer, left + 5, y > top + 20 ? y - 19 : y + 3, 57, "V-SET", UI_VOLTAGE);
    }
    line.width = 2;
    line.color = ui_color(UI_CURRENT);
    int previous_x = 0, previous_y = 0;
    bool previous_valid = false;
    const unsigned columns = span == 2 ? 200 : PLOT_COLUMNS;
    for (unsigned i = 0; i < columns; ++i) {
        const history_bucket_t *bucket = &plot[i];
        if (!bucket->n) {
            previous_valid = false;
            continue;
        }
        int x = left + (right - left) * i / (columns - 1);
        int high = plot_y(bucket->max, top, bottom);
        int low = plot_y(bucket->min, top, bottom);
        int mean = plot_y(bucket->sum / bucket->n, top, bottom);
        if (high != low) draw_line(layer, &line, x, high, x, low);
        if (previous_valid) draw_line(layer, &line, previous_x, previous_y, x, mean);
        else if (high == low) draw_line(layer, &line, x, mean, x + 1, mean);
        previous_x = x;
        previous_y = mean;
        previous_valid = true;
    }
}

static int compare_currents(const void *left, const void *right)
{
    float a = *(const float *)left, b = *(const float *)right;
    return (a > b) - (a < b);
}

static void format_power(char *text, size_t size, float power_uw)
{
    if (fabsf(power_uw) >= 1000000.0f) {
        snprintf(text, size, "%.3f W", power_uw / 1000000.0f);
    } else if (fabsf(power_uw) >= 1000.0f) {
        snprintf(text, size, "%.3f mW", power_uw / 1000.0f);
    } else {
        snprintf(text, size, "%.3f uW", power_uw);
    }
}

static void update_average_current(float current_ua, bool valid)
{
    unsigned unit = CURRENT_UA;
    if (valid && fabsf(current_ua) >= 1000000.0f) unit = CURRENT_A;
    else if (valid && fabsf(current_ua) >= 1000.0f) unit = CURRENT_MA;
    if (unit != average_unit) {
        lv_obj_t *tile = lv_obj_get_parent(average_current_value);
        lv_obj_remove_style(tile, &theme_bg[current_fills[average_unit]], 0);
        lv_obj_remove_style(average_current_value, &theme_text[current_inks[average_unit]], 0);
        lv_obj_add_style(tile, &theme_bg[current_fills[unit]], 0);
        lv_obj_add_style(average_current_value, &theme_text[current_inks[unit]], 0);
        average_unit = unit;
    }
    char text[32];
    if (!valid) strcpy(text, "--");
    else if (unit == CURRENT_A) snprintf(text, sizeof(text), "%.3f A", current_ua / 1000000.0f);
    else format_current(text, sizeof(text), current_ua);
    update_label(average_current_value, text);
}

static void refresh_plot(void)
{
    xSemaphoreTake(lock, portMAX_DELAY);
    bool power_available = power_state == POWER_ON && !power_pending &&
                           last_sample && esp_timer_get_time() - last_sample < 1000000;
    if (!paused) {
        snapshot_head = history_live;
        snapshot_output_mv = power_available ? output_mv : 0;
        snapshot_first_tick = power_first_tick;
        memcpy(history_snapshot, history_ring, HISTORY_BUCKETS * sizeof(*history_ring));
    }
    power_available = power_available && output_mv == snapshot_output_mv &&
                      power_first_tick == snapshot_first_tick;
    xSemaphoreGive(lock);
    const history_bucket_t newest = snapshot_head;
    history_bucket_t avg[2] = {0}, window = {0}, powered = {0};
    unsigned baseline_count = 0;
    const unsigned width = span * 100;
    const unsigned columns = width < PLOT_COLUMNS ? width : PLOT_COLUMNS;
    memset(next_plot, 0, sizeof(next_plot));
    for (unsigned age = 0; newest.n && age < HISTORY_BUCKETS && age <= newest.tick; ++age) {
        uint32_t tick = newest.tick - age;
        const history_bucket_t *b = age ? &history_snapshot[tick % HISTORY_BUCKETS] : &newest;
        if (!b->n || b->tick != tick) continue;
        if (age < 100) history_merge(&avg[0], b);
        if (age < 1000) {
            history_merge(&avg[1], b);
            baseline_samples[baseline_count++] = b->sum / b->n;
            if (power_available && tick >= snapshot_first_tick) history_merge(&powered, b);
        }
        if (age < width) {
            history_merge(&window, b);
            unsigned column = columns - 1 - age * columns / width;
            history_merge(&next_plot[column], b);
        }
    }

    char text[64];
    for (int i = 0; i < 2; ++i) {
        if (avg[i].n) format_current(text, sizeof(text), avg[i].sum / avg[i].n);
        else strcpy(text, "--");
        update_label(i == 0 ? avg_1_value : avg_10_value, text);
    }
    snprintf(text, sizeof(text), "Avg current / %u s", span);
    update_label(average_current_title, text);
    update_average_current(window.n ? window.sum / window.n : 0, window.n != 0);

    if (window.n) {
        format_current(text, sizeof(text), window.max);
        update_label(max_value, text);
        format_current(text, sizeof(text), window.min);
        update_label(min_value, text);
    } else {
        update_label(max_value, "--");
        update_label(min_value, "--");
    }

    if (baseline_count) {
        qsort(baseline_samples, baseline_count, sizeof(*baseline_samples), compare_currents);
        format_current(text, sizeof(text), baseline_samples[(baseline_count - 1) / 10]);
        update_label(baseline_value, text);
    } else {
        update_label(baseline_value, "--");
    }
    if (powered.n) {
        const float volts = snapshot_output_mv / 1000.0f;
        format_power(text, sizeof(text), volts * powered.sum / powered.n);
        update_label(average_power_value, text);
    } else {
        update_label(average_power_value, "--");
    }

    float min = fminf(0, window.min);
    float max = fmaxf(min + 1, window.max * 1.08f);
    float next_log_top = window.max > 200000.0f ?
                         powf(10, ceilf(log10f(window.max))) : 200000.0f;
    bool redraw = graph_dirty || (log_scale ? next_log_top != log_top :
                                  min != plot_min || max != plot_max);
    log_top = next_log_top;
    plot_min = min;
    plot_max = max;
    for (unsigned i = 0; !redraw && i < columns; ++i) {
        const history_bucket_t *before = &plot[i], *after = &next_plot[i];
        if (!before->n || !after->n) {
            redraw = (before->n != 0) != (after->n != 0);
        } else {
            // Skip visually unchanged frames at the graph's current pixel height.
            const int height = lv_obj_get_height(chart) - 48;
            redraw = plot_y(before->min, 0, height) != plot_y(after->min, 0, height) ||
                     plot_y(before->max, 0, height) != plot_y(after->max, 0, height) ||
                     plot_y(before->sum / before->n, 0, height) != plot_y(after->sum / after->n, 0, height);
        }
    }
    if (redraw) {
        memcpy(plot, next_plot, sizeof(plot));
        graph_dirty = false;
        lv_obj_invalidate(chart);
    }
}

static void refresh_ui(lv_timer_t *timer)
{
    xSemaphoreTake(lock, portMAX_DELAY);
    bool live = last_sample && esp_timer_get_time() - last_sample < 1000000;
    int power = power_state;
    bool pending = power_pending;
    int mv = output_mv;
    uint32_t first_tick = power_first_tick;
    xSemaphoreGive(lock);
    refresh_power(power, pending);
    lv_obj_set_hidden(theme_button, !lv_obj_is_hidden(power_panel));

    const char *output_status = "USB offline";
    if (pending) {
        output_status = "Sending";
    } else if (power == POWER_OFF) {
        output_status = "OFF";
    } else if (power == POWER_ON) {
        output_status = "ON";
    } else if (power == POWER_UNKNOWN) {
        output_status = "Unknown";
    }
    char text[44];
    update_label(chart_state, !live ? "NO LIVE DATA" : paused ? "PAUSED" : "LIVE");
    update_tint(chart_state, !live ? UI_FAULT : paused ? UI_VOLTAGE : UI_CURRENT);
    update_label(out_value, output_status);
    update_tint(out_value, output_tint(power, pending));
    snprintf(text, sizeof(text), "%.2f V", selected_mv / 1000.0);
    update_label(set_value, text);
    update_label(voltage_value, text);
    refresh_output_button(power, pending);
    int voltage_trace = power == POWER_ON && !pending ? mv : 0;
    if (voltage_trace != graph_voltage_mv) {
        graph_voltage_mv = voltage_trace;
        lv_obj_invalidate(chart);
    }
    if (!paused && lv_obj_is_hidden(power_panel)) {
        refresh_plot();
    }
    if (!live || pending || power != POWER_ON ||
        (paused && (mv != snapshot_output_mv || first_tick != snapshot_first_tick))) {
        update_label(average_power_value, "--");
    }
}

static void tap(lv_event_t *event)
{
    intptr_t action = (intptr_t)lv_event_get_user_data(event);
    lv_obj_t *label = lv_obj_get_child(lv_event_get_target(event), 0);
    if (action == TAP_WINDOW) {
        span_index = (span_index + 1) % (sizeof(spans) / sizeof(spans[0]));
        span = spans[span_index];
        lv_label_set_text_fmt(label, "%u s window", span);
    } else if (action == TAP_AXIS) {
        log_scale = !log_scale;
        lv_label_set_text(label, log_scale ? "Log Y" : "Linear Y");
    } else if (action == TAP_PAUSE) {
        paused = !paused;
        lv_label_set_text(label, paused ? LV_SYMBOL_PLAY : LV_SYMBOL_PAUSE);
    } else {
        xSemaphoreTake(lock, portMAX_DELAY);
        reset_stats();
        xSemaphoreGive(lock);
        snapshot_head = (history_bucket_t){0};
        memset(plot, 0, sizeof(plot));
    }
    graph_dirty = true;
    refresh_plot();
}

static void power_action(lv_event_t *event)
{
    intptr_t action = (intptr_t)lv_event_get_user_data(event);
    if (action == 0) {
        lv_obj_set_hidden(power_panel, false);
    } else if (action == 1) {
        lv_obj_set_hidden(power_panel, true);
        return;
    } else if (action == 2) {
        int step = lv_slider_get_value(power_slider);
        selected_mv = step ? 400 + step * 100 : 0;
    }

    xSemaphoreTake(lock, portMAX_DELAY);
    int state = power_state;
    if (action == 3 && !power_pending) {
        if (state == POWER_ON || state == POWER_UNKNOWN) {
            power_request = 0;
            power_pending = true;
        } else if (state == POWER_OFF && selected_mv >= 500 && selected_mv <= 5000) {
            power_request = selected_mv;
            power_pending = true;
        }
    }
    bool pending = power_pending;
    xSemaphoreGive(lock);
    refresh_power(state, pending);
    refresh_output_button(state, pending);
}

static lv_obj_t *power_button(lv_obj_t *parent, int x, int y, int width,
                              const char *title, intptr_t action)
{
    lv_obj_t *button = lv_button_create(parent);
    lv_obj_set_pos(button, x, y);
    lv_obj_set_size(button, width, 48);
    lv_obj_add_style(button, &theme_bg[UI_ACTION], 0);
    lv_obj_add_style(button, &theme_bg[UI_ON_BUTTON], LV_STATE_CHECKED);
    lv_obj_add_style(button, &theme_bg[UI_CONTROL], LV_STATE_DISABLED);
    lv_obj_add_style(button, &theme_text[UI_TEXT], 0);
    lv_obj_add_style(button, &theme_text[UI_MUTED], LV_STATE_DISABLED);
    lv_obj_t *label = lv_label_create(button);
    lv_label_set_text(label, title);
    lv_obj_center(label);
    lv_obj_add_event_cb(button, power_action, LV_EVENT_CLICKED, (void *)action);
    return button;
}

static void build_power_panel(lv_obj_t *screen)
{
    power_panel = lv_obj_create(screen);
    lv_obj_set_pos(power_panel, 0, 0);
    lv_obj_set_size(power_panel, 800, 480);
    lv_obj_set_style_radius(power_panel, 0, 0);
    lv_obj_set_style_border_width(power_panel, 0, 0);
    lv_obj_set_scrollable(power_panel, false);
    lv_obj_set_style_bg_opa(power_panel, LV_OPA_COVER, 0);
    lv_obj_add_style(power_panel, &theme_bg[UI_SURFACE], 0);
    lv_obj_add_style(power_panel, &theme_text[UI_TEXT], 0);

    power_button(power_panel, 12, 10, 54, LV_SYMBOL_LEFT, 1);
    lv_obj_t *title = lv_label_create(power_panel);
    lv_label_set_text(title, "Set output voltage");
    lv_obj_set_style_text_font(title, &lv_font_montserrat_22, 0);
    lv_obj_set_pos(title, 80, 20);
    power_target = lv_label_create(power_panel);
    lv_obj_add_style(power_target, &theme_text[UI_VOLTAGE], 0);
    lv_obj_set_style_text_font(power_target, &lv_font_montserrat_22, 0);
    lv_obj_set_pos(power_target, 32, 80);
    power_info = lv_label_create(power_panel);
    lv_obj_set_pos(power_info, 32, 275);
    power_slider = lv_slider_create(power_panel);
    const int slider_x = 45, slider_width = 625;
    lv_obj_set_pos(power_slider, slider_x, 155);
    lv_obj_set_size(power_slider, slider_width, 25);
    lv_slider_set_range(power_slider, 0, 46);
    lv_obj_add_style(power_slider, &theme_bg[UI_CONTROL], LV_PART_MAIN);
    lv_obj_add_style(power_slider, &theme_bg[UI_VOLTAGE], LV_PART_INDICATOR);
    lv_obj_add_style(power_slider, &theme_bg[UI_VOLTAGE], LV_PART_KNOB);
    lv_obj_add_style(power_slider, &theme_bg[UI_MUTED], LV_PART_KNOB | LV_STATE_DISABLED);
    lv_obj_add_event_cb(power_slider, power_action, LV_EVENT_VALUE_CHANGED, (void *)2);
    for (int volts = 0; volts <= 5; ++volts) {
        int step = volts ? volts * 10 - 4 : 0;
        int x = slider_x + slider_width * step / 46;
        lv_obj_t *tick = lv_obj_create(power_panel);
        lv_obj_set_size(tick, 1, 8);
        lv_obj_set_pos(tick, x, 188);
        lv_obj_set_style_border_width(tick, 0, 0);
        lv_obj_add_style(tick, &theme_bg[UI_MUTED], 0);
        lv_obj_set_clickable(tick, false);
        lv_obj_t *label = lv_label_create(power_panel);
        lv_label_set_text_fmt(label, "%d V", volts);
        lv_obj_add_style(label, &theme_text[UI_MUTED], 0);
        lv_obj_set_pos(label, x - 11, 201);
    }
    lv_obj_t *range = lv_label_create(power_panel);
    lv_label_set_text(range, "0 V = OFF   |   0.5 to 5.0 V in 0.1 V steps");
    lv_obj_set_pos(range, 32, 237);
    lv_obj_t *hint = lv_label_create(power_panel);
    lv_label_set_text(hint, "Return to the main screen to switch output on or off.");
    lv_obj_add_style(hint, &theme_text[UI_MUTED], 0);
    lv_obj_set_pos(hint, 32, 318);
    lv_obj_set_hidden(power_panel, true);
}

static void bind_voltage_settings(lv_obj_t *label)
{
    lv_obj_t *tile = lv_obj_get_parent(label);
    lv_obj_set_clickable(tile, true);
    lv_obj_add_style(tile, &theme_bg[UI_PRESSED], LV_STATE_PRESSED);
    lv_obj_add_event_cb(tile, power_action, LV_EVENT_CLICKED, (void *)0);
}

static lv_obj_t *readout(lv_obj_t *screen, int y, unsigned fill, unsigned accent,
                         const char *title, const char *initial)
{
    lv_obj_t *tile = lv_obj_create(screen);
    lv_obj_set_pos(tile, 590, y);
    lv_obj_set_size(tile, 194, 66);
    lv_obj_set_scrollable(tile, false);
    lv_obj_set_style_pad_all(tile, 0, 0);
    lv_obj_set_style_radius(tile, 7, 0);
    lv_obj_set_style_border_width(tile, 0, 0);
    lv_obj_add_style(tile, &theme_bg[fill], 0);
    lv_obj_t *label = lv_label_create(tile);
    lv_label_set_text(label, title);
    lv_obj_add_style(label, &theme_text[UI_CARD_CAPTION], 0);
    lv_obj_set_pos(label, 11, 8);
    lv_obj_t *value = lv_label_create(tile);
    lv_label_set_text(value, initial);
    lv_obj_set_style_text_font(value, &lv_font_montserrat_26, 0);
    lv_obj_add_style(value, &theme_text[accent], 0);
    lv_obj_set_pos(value, 11, 28);
    return value;
}

static lv_obj_t *stat_tile(lv_obj_t *screen, int x, int y, const char *key, unsigned fill, unsigned ink)
{
    lv_obj_t *tile = lv_obj_create(screen);
    lv_obj_set_pos(tile, x, y);
    lv_obj_set_size(tile, 250, 32);
    lv_obj_set_scrollable(tile, false);
    lv_obj_set_style_pad_all(tile, 0, 0);
    lv_obj_set_style_radius(tile, 5, 0);
    lv_obj_set_style_clip_corner(tile, true, 0);
    lv_obj_set_style_border_width(tile, 0, 0);
    lv_obj_add_style(tile, &theme_bg[UI_SURFACE], 0);
    lv_obj_t *label = lv_label_create(tile);
    lv_label_set_text(label, key);
    lv_obj_add_style(label, &theme_text[UI_BADGE_TEXT], 0);
    lv_obj_add_style(label, &theme_bg[fill], 0);
    lv_obj_set_style_bg_opa(label, LV_OPA_COVER, 0);
    lv_obj_set_style_pad_left(label, 9, 0);
    lv_obj_set_style_pad_top(label, 7, 0);
    lv_obj_set_size(label, 112, 32);
    lv_obj_set_pos(label, 0, 0);
    lv_obj_t *value = lv_label_create(tile);
    lv_label_set_text(value, "--");
    lv_obj_add_style(value, &theme_text[ink], 0);
    lv_obj_align(value, LV_ALIGN_RIGHT_MID, -10, 0);
    return value;
}

static lv_obj_t *view_button(lv_obj_t *parent, int x, int y, int width, int height,
                             const char *title, intptr_t action)
{
    lv_obj_t *button = lv_button_create(parent);
    lv_obj_set_pos(button, x, y);
    lv_obj_set_size(button, width, height);
    lv_obj_add_style(button, &theme_bg[UI_CONTROL], 0);
    lv_obj_add_style(button, &theme_bg[UI_PRESSED], LV_STATE_PRESSED);
    lv_obj_add_style(button, &theme_text[UI_TEXT], 0);
    lv_obj_t *label = lv_label_create(button);
    lv_label_set_text(label, title);
    lv_obj_center(label);
    lv_obj_add_event_cb(button, tap, LV_EVENT_CLICKED, (void *)action);
    return button;
}

static void theme_action(lv_event_t *event)
{
    intptr_t action = (intptr_t)lv_event_get_user_data(event);
    if (action < 0) {
        lv_obj_set_hidden(theme_picker, action == -2);
        return;
    }
    if ((unsigned)action != active_theme) {
        set_theme_colors((unsigned)action);
        lv_obj_set_state(theme_options[THEME_DARK], LV_STATE_CHECKED, active_theme == THEME_DARK);
        lv_obj_set_state(theme_options[THEME_LIGHT], LV_STATE_CHECKED, active_theme == THEME_LIGHT);
        lv_obj_report_style_change(NULL);
        refresh_ui(NULL);
        lv_obj_invalidate(chart);
    }
    lv_obj_set_hidden(theme_picker, true);
}

static lv_obj_t *theme_control(lv_obj_t *parent, int x, int y, int width,
                               const char *text, intptr_t action)
{
    lv_obj_t *button = lv_button_create(parent);
    lv_obj_set_pos(button, x, y);
    lv_obj_set_size(button, width, 35);
    lv_obj_add_style(button, &theme_bg[UI_CONTROL], 0);
    lv_obj_add_style(button, &theme_bg[UI_ON_BUTTON], LV_STATE_CHECKED);
    lv_obj_add_style(button, &theme_text[UI_TEXT], 0);
    lv_obj_t *label = lv_label_create(button);
    lv_label_set_text(label, text);
    lv_obj_center(label);
    lv_obj_add_event_cb(button, theme_action, LV_EVENT_CLICKED, (void *)action);
    return button;
}

static void draw_theme_icon(lv_event_t *event)
{
    lv_obj_t *button = lv_event_get_target(event);
    lv_layer_t *layer = lv_event_get_layer(event);
    lv_area_t area;
    lv_obj_get_coords(button, &area);
    const int cx = (area.x1 + area.x2) / 2;
    const int cy = (area.y1 + area.y2) / 2;
    static const int8_t outline[][2] = {
        {-4,-8}, {-8,-7}, {-12,-2}, {-8,1}, {-6,-1},
        {-6,8}, {6,8}, {6,-1}, {8,1}, {12,-2}, {8,-7},
        {4,-8}, {3,-6}, {0,-5}, {-3,-6}, {-4,-8}
    };
    lv_draw_line_dsc_t line;
    lv_draw_line_dsc_init(&line);
    line.color = ui_color(UI_TEXT);
    line.width = 2;
    for (unsigned i = 1; i < sizeof(outline) / sizeof(outline[0]); ++i) {
        draw_line(layer, &line, cx + outline[i - 1][0], cy + outline[i - 1][1],
                  cx + outline[i][0], cy + outline[i][1]);
    }
}

static void build_theme_picker(lv_obj_t *screen)
{
    theme_button = theme_control(screen, 742, 5, 42, "", -1);
    lv_obj_add_event_cb(theme_button, draw_theme_icon, LV_EVENT_DRAW_POST, NULL);
    theme_picker = lv_obj_create(screen);
    lv_obj_set_size(theme_picker, 800, 480);
    lv_obj_set_scrollable(theme_picker, false);
    lv_obj_set_style_pad_all(theme_picker, 0, 0);
    lv_obj_set_style_border_width(theme_picker, 0, 0);
    lv_obj_set_style_radius(theme_picker, 0, 0);
    lv_obj_set_style_bg_color(theme_picker, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(theme_picker, LV_OPA_30, 0);
    lv_obj_set_clickable(theme_picker, true);
    lv_obj_add_event_cb(theme_picker, theme_action, LV_EVENT_CLICKED, (void *)-2);
    lv_obj_t *panel = lv_obj_create(theme_picker);
    lv_obj_set_pos(panel, 556, 44);
    lv_obj_set_size(panel, 228, 149);
    lv_obj_set_scrollable(panel, false);
    lv_obj_set_style_pad_all(panel, 0, 0);
    lv_obj_set_style_border_width(panel, 1, 0);
    lv_obj_add_style(panel, &theme_border[UI_GRID], 0);
    lv_obj_add_style(panel, &theme_bg[UI_SURFACE], 0);
    lv_obj_add_style(panel, &theme_text[UI_TEXT], 0);
    lv_obj_t *title = lv_label_create(panel);
    lv_label_set_text(title, "Theme");
    lv_obj_set_pos(title, 14, 10);
    theme_options[THEME_LIGHT] = theme_control(panel, 14, 40, 198, "Light", THEME_LIGHT);
    theme_options[THEME_DARK] = theme_control(panel, 14, 85, 198, "Dark", THEME_DARK);
    lv_obj_set_state(theme_options[THEME_DARK], LV_STATE_CHECKED, true);
    lv_obj_set_hidden(theme_picker, true);
}

static void build_ui(void)
{
    init_theme();
    lv_obj_t *screen = lv_screen_active();
    lv_obj_set_scrollable(screen, false);
    lv_obj_add_style(screen, &theme_bg[UI_BACKGROUND], 0);
    lv_obj_add_style(screen, &theme_text[UI_TEXT], 0);

    lv_obj_t *title = lv_label_create(screen);
    lv_label_set_text(title, "Power Monitor");
    lv_obj_set_style_text_font(title, &lv_font_montserrat_22, 0);
    lv_obj_set_pos(title, 16, 10);
    view_button(screen, 220, 6, 121, 34, "10 s window", TAP_WINDOW);
    view_button(screen, 349, 6, 92, 34, "Log Y", TAP_AXIS);
    chart_state = lv_label_create(screen);
    lv_obj_set_pos(chart_state, 613, 16);
    lv_obj_add_style(chart_state, &theme_text[UI_CURRENT], 0);
    lv_label_set_text(chart_state, "NO LIVE DATA");

    chart = lv_obj_create(screen);
    lv_obj_set_scrollable(chart, false);
    lv_obj_set_pos(chart, 16, 46);
    lv_obj_set_size(chart, 565, 353);
    lv_obj_set_style_pad_all(chart, 0, 0);
    lv_obj_add_style(chart, &theme_bg[UI_CHART], 0);
    lv_obj_set_style_border_width(chart, 1, 0);
    lv_obj_add_style(chart, &theme_border[UI_CHART_BORDER], 0);
    lv_obj_set_style_radius(chart, 8, 0);
    lv_obj_add_event_cb(chart, draw_graph, LV_EVENT_DRAW_MAIN, NULL);
    plot_max = 1;

    voltage_value = readout(screen, 46, UI_FILL_VOLTAGE, UI_VOLTAGE, "V-SET  " LV_SYMBOL_RIGHT, "0.00 V");
    bind_voltage_settings(voltage_value);
    baseline_value = readout(screen, 118, UI_FILL_BASELINE, UI_BASELINE_INK, "Baseline current", "--");
    average_current_value = readout(screen, 190, UI_FILL_UA, UI_CURRENT, "Avg current  /  10 s", "--");
    average_current_title = lv_obj_get_child(lv_obj_get_parent(average_current_value), 0);
    average_power_value = readout(screen, 262, UI_FILL_POWER, UI_POWER, "Avg power  /  est.", "--");

    main_power_button = power_button(screen, 590, 351, 62, LV_SYMBOL_POWER, 3);
    lv_obj_set_style_radius(main_power_button, 5, 0);
    lv_obj_set_style_shadow_width(main_power_button, 0, 0);
    lv_obj_set_style_text_font(lv_obj_get_child(main_power_button, 0), &lv_font_montserrat_22, 0);
    lv_obj_add_style(main_power_button, &theme_bg[UI_ACTION], 0);
    lv_obj_add_style(main_power_button, &theme_bg[UI_OUTPUT_ACTIVE], LV_STATE_CHECKED);
    lv_obj_add_style(main_power_button, &theme_text[UI_OUTPUT_ACTIVE_TEXT], LV_STATE_CHECKED);
    lv_obj_t *pause = view_button(screen, 656, 351, 62, 48, LV_SYMBOL_PAUSE, TAP_PAUSE);
    lv_obj_t *clear = view_button(screen, 722, 351, 62, 48, LV_SYMBOL_TRASH, TAP_CLEAR);
    lv_obj_set_style_text_font(lv_obj_get_child(pause, 0), &lv_font_montserrat_22, 0);
    lv_obj_set_style_text_font(lv_obj_get_child(clear, 0), &lv_font_montserrat_22, 0);
    lv_obj_set_style_radius(pause, 5, 0);
    lv_obj_set_style_radius(clear, 5, 0);
    lv_obj_set_style_shadow_width(pause, 0, 0);
    lv_obj_set_style_shadow_width(clear, 0, 0);

    set_value = stat_tile(screen, 16, 409, "V-SET " LV_SYMBOL_RIGHT, UI_BADGE_V, UI_VOLTAGE);
    bind_voltage_settings(set_value);
    max_value = stat_tile(screen, 275, 409, "I-MAX", UI_BADGE_MAX, UI_PEAK);
    avg_1_value = stat_tile(screen, 534, 409, "AVG 1s", UI_BADGE_AVG, UI_AVG);
    out_value = stat_tile(screen, 16, 444, "OUT", UI_BADGE_OUT, UI_INK_A);
    min_value = stat_tile(screen, 275, 444, "I-MIN", UI_BADGE_MIN, UI_CURRENT);
    avg_10_value = stat_tile(screen, 534, 444, "AVG 10s", UI_BADGE_AVG, UI_AVG);

    build_power_panel(screen);
    build_theme_picker(screen);
    refresh_ui(NULL);
    lv_timer_create(refresh_ui, 500, NULL);
}
static void display_task(void *arg)
{
    bsp_display_cfg_t display = {
        .lvgl_port_cfg = ESP_LVGL_PORT_INIT_CONFIG(),
    };
    display.lvgl_port_cfg.task_affinity = 1;
    if (bsp_display_start_with_config(&display)) {
        bsp_display_lock(0);
        build_ui();
        bsp_display_unlock();
        ESP_LOGW("power_display", "SUB3 display initialized on core 1");
    }
    vTaskDelete(NULL);
}

void app_main(void)
{
    lock = xSemaphoreCreateMutex();
    output = xQueueCreate(256, sizeof(Reading));
    history_ring = heap_caps_calloc(HISTORY_BUCKETS, sizeof(*history_ring), MALLOC_CAP_SPIRAM);
    history_snapshot = heap_caps_calloc(HISTORY_BUCKETS, sizeof(*history_snapshot), MALLOC_CAP_SPIRAM);
    configASSERT(lock && output && history_ring && history_snapshot);
    setvbuf(stdout, NULL, _IOLBF, 256);

    usb_host_config_t usb = {.intr_flags = ESP_INTR_FLAG_LEVEL1};
    ESP_ERROR_CHECK(usb_host_install(&usb));
    BaseType_t started = xTaskCreatePinnedToCore(usb_events, "usb", 4096, NULL, 6, NULL, 0);
    configASSERT(started == pdPASS);
    ESP_ERROR_CHECK(cdc_acm_host_install(NULL));
    ESP_ERROR_CHECK(cdc_acm_host_register_new_dev_callback(on_usb_device));
    ESP_LOGW(TAG, "USB host ready, waiting for 15a2:300a");
    started = xTaskCreatePinnedToCore(stream_task, "pc_stream", 4096, NULL, 4, NULL, 0);
    configASSERT(started == pdPASS);
    started = xTaskCreatePinnedToCore(meter_task, "meter", 4096, NULL, 5, NULL, 0);
    configASSERT(started == pdPASS);

    // Keep RGB DMA interrupts and LVGL off the USB and serial streaming core.
    started = xTaskCreatePinnedToCore(display_task, "display_init", 6144, NULL, 5, NULL, 1);
    configASSERT(started == pdPASS);
}
