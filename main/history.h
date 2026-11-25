#pragma once

#include <stdint.h>

enum { HISTORY_STEP_US = 10000, HISTORY_BUCKETS = 6000, PLOT_COLUMNS = 240 };

typedef struct {
    uint32_t tick;
    uint32_t n;
    uint32_t pulses;
    float sum, min, max;
} history_bucket_t;

static inline void history_add(history_bucket_t *ring, history_bucket_t *live,
                               int *above, uint32_t tick, float value)
{
    if (live->n && live->tick != tick) {
        ring[live->tick % HISTORY_BUCKETS] = *live;
        live->n = 0;
    }
    if (!live->n) {
        *live = (history_bucket_t){.tick = tick, .min = value, .max = value};
    }
    if (value < live->min) live->min = value;
    if (value > live->max) live->max = value;
    ++live->n;
    live->sum += value;
    if (value >= 1000.0f && !*above) ++live->pulses;
    *above = value >= 1000.0f;
}

static inline void history_merge(history_bucket_t *dest, const history_bucket_t *src)
{
    if (!src->n) return;
    if (!dest->n || src->min < dest->min) dest->min = src->min;
    if (!dest->n || src->max > dest->max) dest->max = src->max;
    dest->n += src->n;
    dest->pulses += src->pulses;
    dest->sum += src->sum;
}
