# Function Timing Analysis — Uart_HT CM7

## Overview

This document captures the DWT (Data Watchpoint and Trace) cycle counter based timing
instrumentation added to the UART receive pipeline on STM32H755ZI (Cortex-M7 @ 200 MHz).

---

## Method: DWT Cycle Counter

The ARM Cortex-M7 has a built-in hardware cycle counter called **CYCCNT** inside the DWT unit.
It counts every CPU clock cycle with zero overhead — no timer peripheral needed.

### Registers Used

| Macro        | Address      | Purpose                        |
|--------------|--------------|--------------------------------|
| `DEM_CR`     | `0xE000EDFC` | Enable trace subsystem (bit 24)|
| `DWT_CTRL`   | `0xE0001000` | Enable CYCCNT (bit 0)          |
| `DWT_CYCCNT` | `0xE0001004` | 32-bit free-running cycle count|

### Initialization (in `main()` before while loop)

```c
DEM_CR   |= (1 << 24);  // enable trace
DWT_CTRL |= (1 << 0);   // enable CYCCNT
DWT_CYCCNT = 0;
```

### Measurement Pattern (used in every instrumented function)

```c
uint32_t _t_start = DWT_CYCCNT;       // capture start

/* ... function body ... */

cycles_last = DWT_CYCCNT - _t_start;  // elapsed cycles
us_last     = cycles_last / CPU_FREQ_MHZ;  // convert to microseconds
if (cycles_last > cycles_max) {
    cycles_max = cycles_last;
    us_max     = us_last;
}
```

`CPU_FREQ_MHZ = 200` → dividing cycles by 200 gives microseconds directly.

---

## Instrumented Functions

### 1. `Ring_write_handler`

**Purpose:** Called when DMA RX half/complete callback fires (`rx_event_flag = 1`).
Reads new bytes from the circular DMA buffer and writes them into the software ring buffer.

**Trigger:** `HAL_UART_RxHalfCpltCallback` and `HAL_UART_RxCpltCallback` both set `rx_event_flag = 1`.

**Variables:**

| Variable          | Description                        |
|-------------------|------------------------------------|
| `rwh_call_count`  | Total number of times called       |
| `rwh_cycles_last` | Cycle count of last execution      |
| `rwh_cycles_max`  | Maximum cycle count ever recorded  |
| `rwh_us_last`     | Last execution time in microseconds|
| `rwh_us_max`      | Maximum execution time in µs       |

**Measured Results:**

| Metric          | Value     |
|-----------------|-----------|
| `rwh_us_last`   | 139 µs    |
| `rwh_us_max`    | 139 µs    |
| `rwh_call_count`| 2         |

**Analysis:**
- 139 µs is high for a simple buffer copy. Root cause is STM32H7 D-Cache coherency —
  DMA writes to AXI SRAM (0x24xxxxxx) but CPU reads stale cached data causing cache miss
  penalties. Each 32-byte cache line miss costs ~10–20 extra cycles.
- `rwh_call_count = 2` means both the half-complete and full-complete DMA callbacks fired,
  which is the expected normal behavior for a 512-byte DMA buffer.

---

### 2. `Ring_read_handler`

**Purpose:** Called when `ring_read_flag = 1` (set by `Ring_write_handler`).
Scans the ring buffer for a valid `0xFA` start byte, validates frame length,
reads the full frame into `g_parser_buffer`, and verifies checksum.

**Variables:**

| Variable          | Description                        |
|-------------------|------------------------------------|
| `rrh_call_count`  | Total number of times called       |
| `rrh_cycles_last` | Cycle count of last execution      |
| `rrh_cycles_max`  | Maximum cycle count ever recorded  |
| `rrh_us_last`     | Last execution time in microseconds|
| `rrh_us_max`      | Maximum execution time in µs       |

**Measured Results:**

| Metric          | Value  |
|-----------------|--------|
| `rrh_us_last`   | 0 µs   |
| `rrh_us_max`    | 10 µs  |
| `rrh_call_count`| 32     |

**Analysis:**
- `rrh_call_count = 32` vs `rwh_call_count = 2` — Ring_read_handler is called far more
  often than Ring_write_handler. This is because `ring_read_flag` stays set across loop
  iterations when a frame is partially received (`available_data < frame_total_len` path),
  causing repeated calls until the full frame arrives.
- Max 10 µs is acceptable — includes ring buffer scan + checksum calculation.
- Last = 0 µs means the most recent call returned immediately (no data or no 0xFA found).

**Early return paths also timed:**
- Invalid frame length (`frame_total_len < 10 || > 28`) → timing captured before return
- Incomplete frame (`available_data < frame_total_len`) → timing captured before return

---

### 3. `packet_dispatcher`

**Purpose:** Called when `frame_ready_flag = 1`. Decodes the parameter type from
`g_parser_buffer` and routes to `ecg_handler`, `nibp_handler`, or `spo2_handler`.

**Variables:**

| Variable         | Description                        |
|------------------|------------------------------------|
| `pd_cycles_last` | Cycle count of last execution      |
| `pd_cycles_max`  | Maximum cycle count ever recorded  |
| `pd_us_last`     | Last execution time in microseconds|
| `pd_us_max`      | Maximum execution time in µs       |

**Measured Results:**

| Metric        | Value |
|---------------|-------|
| `pd_us_last`  | 1 µs  |
| `pd_us_max`   | 2 µs  |

**Analysis:**
- Very fast — 1–2 µs is expected since it's just a switch-case dispatch plus
  simple struct field assignments. No blocking calls in the measured path.

---

## Summary Table

| Function             | Last (µs) | Max (µs) | Call Count | Notes                          |
|----------------------|-----------|----------|------------|--------------------------------|
| `Ring_write_handler` | 139       | 139      | 2          | High — D-Cache miss suspected  |
| `Ring_read_handler`  | 0         | 10       | 32         | Called repeatedly until frame complete |
| `packet_dispatcher`  | 1         | 2        | —          | Fast dispatch, no blocking     |

---

## Variable Addresses (AXI SRAM — 0x24xxxxxx)

| Variable          | Address      |
|-------------------|--------------|
| `rx_event_flag`   | `0x24001086` |
| `ring_read_flag`  | `0x24001087` |
| `timer_flag`      | `0x24001089` |
| `rwh_call_count`  | `0x240012cc` |
| `rrh_call_count`  | `0x240012d0` |
| `rwh_us_last`     | `0x240012dc` |
| `rwh_us_max`      | `0x240012e0` |
| `rrh_us_last`     | `0x240012ec` |
| `rrh_us_max`      | `0x240012f0` |
| `pd_us_last`      | `0x240012fc` |
| `pd_us_max`       | `0x24001300` |

---

## Known Issue: D-Cache Coherency on STM32H7

The STM32H755 CM7 core has a 16KB D-Cache enabled by default. DMA transfers bypass
the cache and write directly to AXI SRAM. When the CPU then reads `g_dma_buffer`,
it may read stale cached data OR incur cache miss penalties on first access.

**Fix (not yet applied):** Add cache invalidation before reading DMA buffer:
```c
SCB_InvalidateDCache_by_Addr((uint32_t*)g_dma_buffer, DMA_BUFFER_SIZE);
```
Or declare `g_dma_buffer` in a non-cacheable MPU region.

---

*Branch: `feature/function-timing-analysis` | Repo: Uart_HT2*
