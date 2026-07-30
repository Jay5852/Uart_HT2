#ifndef DWT_TIMING_H
#define DWT_TIMING_H

#include <stdint.h>

/* ─── Configuration ──────────────────────────────────────────────────────── */
#define DWT_CPU_FREQ_MHZ   200U
#define DWT_MAX_TRACKERS   8U       // max number of functions you can track

/* ─── Per-function timing result ─────────────────────────────────────────── */
typedef struct {
    const char    *name;            // function name label
    uint32_t       call_count;      // total number of calls
    uint32_t       cycles_last;     // cycles of last execution
    uint32_t       cycles_max;      // max cycles ever recorded
    uint32_t       cycles_min;      // min cycles ever recorded
    uint32_t       us_last;         // last execution in microseconds
    uint32_t       us_max;          // max execution in microseconds
    uint32_t       us_min;          // min execution in microseconds
} DWT_Tracker_t;

/* ─── Public API ─────────────────────────────────────────────────────────── */

/**
 * @brief  Initialize DWT cycle counter. Call once in main() before use.
 */
void DWT_Init(void);

/**
 * @brief  Register a named tracker slot. Call once per function at startup.
 * @param  name   String label for the function (e.g. "Ring_write_handler")
 * @return Pointer to the tracker, or NULL if slots are full.
 */
DWT_Tracker_t *DWT_Register(const char *name);

/**
 * @brief  Capture start timestamp. Call at the top of the function to measure.
 * @return Current CYCCNT value (pass this to DWT_End).
 */
uint32_t DWT_Start(void);

/**
 * @brief  Capture end timestamp and update tracker stats.
 * @param  tracker   Pointer returned by DWT_Register.
 * @param  t_start   Value returned by DWT_Start.
 */
void DWT_End(DWT_Tracker_t *tracker, uint32_t t_start);

/**
 * @brief  Reset all stats for a tracker (call_count, max, min, last).
 * @param  tracker   Pointer returned by DWT_Register.
 */
void DWT_Reset(DWT_Tracker_t *tracker);

/**
 * @brief  Get pointer to all registered trackers (for debug display).
 * @param  count  Output: number of registered trackers.
 * @return Pointer to internal tracker array.
 */
const DWT_Tracker_t *DWT_GetAll(uint8_t *count);

#endif /* DWT_TIMING_H */
