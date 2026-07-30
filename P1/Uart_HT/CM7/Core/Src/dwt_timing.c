#include "dwt_timing.h"

/* ─── DWT register access (Cortex-M7, no HAL needed) ────────────────────── */
#define DWT_CYCCNT   (*(volatile uint32_t *)0xE0001004)
#define DWT_CTRL     (*(volatile uint32_t *)0xE0001000)
#define DEM_CR       (*(volatile uint32_t *)0xE000EDFC)

/* ─── Internal state ─────────────────────────────────────────────────────── */
static DWT_Tracker_t  s_trackers[DWT_MAX_TRACKERS];
static uint8_t        s_count = 0;

/* ─── API Implementation ─────────────────────────────────────────────────── */

void DWT_Init(void)
{
    DEM_CR     |= (1U << 24);   // enable trace
    DWT_CTRL   |= (1U << 0);    // enable CYCCNT
    DWT_CYCCNT  = 0U;
}

DWT_Tracker_t *DWT_Register(const char *name)
{
    if (s_count >= DWT_MAX_TRACKERS) {
        return NULL;
    }

    DWT_Tracker_t *t = &s_trackers[s_count++];
    t->name        = name;
    t->call_count  = 0U;
    t->cycles_last = 0U;
    t->cycles_max  = 0U;
    t->cycles_min  = UINT32_MAX;
    t->us_last     = 0U;
    t->us_max      = 0U;
    t->us_min      = UINT32_MAX;

    return t;
}

uint32_t DWT_Start(void)
{
    return DWT_CYCCNT;
}

void DWT_End(DWT_Tracker_t *tracker, uint32_t t_start)
{
    if (tracker == NULL) {
        return;
    }

    uint32_t elapsed = DWT_CYCCNT - t_start;

    tracker->call_count++;
    tracker->cycles_last = elapsed;
    tracker->us_last     = elapsed / DWT_CPU_FREQ_MHZ;

    if (elapsed > tracker->cycles_max) {
        tracker->cycles_max = elapsed;
        tracker->us_max     = tracker->us_last;
    }

    if (elapsed < tracker->cycles_min) {
        tracker->cycles_min = elapsed;
        tracker->us_min     = tracker->us_last;
    }
}

void DWT_Reset(DWT_Tracker_t *tracker)
{
    if (tracker == NULL) {
        return;
    }

    tracker->call_count  = 0U;
    tracker->cycles_last = 0U;
    tracker->cycles_max  = 0U;
    tracker->cycles_min  = UINT32_MAX;
    tracker->us_last     = 0U;
    tracker->us_max      = 0U;
    tracker->us_min      = UINT32_MAX;
}

const DWT_Tracker_t *DWT_GetAll(uint8_t *count)
{
    if (count != NULL) {
        *count = s_count;
    }
    return s_trackers;
}
