/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdio.h"
#define RING_BUFFER_SIZE 4096
#define DMA_BUFFER_SIZE 512

// DWT timing
#define DWT_CYCCNT   (*(volatile uint32_t *)0xE0001004)
#define DWT_CTRL     (*(volatile uint32_t *)0xE0001000)
#define DEM_CR       (*(volatile uint32_t *)0xE000EDFC)
#define CPU_FREQ_MHZ 200U
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
	uint8_t buffer[RING_BUFFER_SIZE];
	uint16_t head;
	uint16_t tail;

} ring_buffer_t;
ring_buffer_t uart_ring_buffer;

typedef struct {
	uint8_t parameter_type;
	uint8_t packet_type;
	uint8_t packet_id;

} protocol_frame_t;
protocol_frame_t frame;

static uint16_t g_dma_old_pos = 0;
static volatile uint8_t rx_event_flag = 0;
static volatile uint8_t ring_read_flag = 0;
static volatile uint8_t frame_ready_flag = 0;
static volatile uint8_t timer_flag = 0;

static uint8_t g_dma_buffer[DMA_BUFFER_SIZE];
static uint8_t g_parser_buffer[64];

// Ring_write_handler timing results
static volatile uint32_t rwh_call_count  = 0;
static volatile uint32_t rrh_call_count  = 0;
static volatile uint32_t rwh_cycles_last = 0;
static volatile uint32_t rwh_cycles_max  = 0;
static volatile uint32_t rwh_us_last     = 0;
static volatile uint32_t rwh_us_max      = 0;

// Ring_read_handler timing results
static volatile uint32_t rrh_cycles_last = 0;
static volatile uint32_t rrh_cycles_max  = 0;
static volatile uint32_t rrh_us_last     = 0;
static volatile uint32_t rrh_us_max      = 0;

// total dispatch timing (if-block in main to packet_dispatcher return)
static volatile uint32_t td_cycles_last  = 0;
static volatile uint32_t td_cycles_max   = 0;
static volatile uint32_t td_us_last      = 0;
static volatile uint32_t td_us_max       = 0;

// SPO2 - 10 Byte Command Database

static const uint8_t spo2_database[][10] = { { 0xFA, 0x0A, 0x03, 0x01, 0x01,
		0x00, 0x00, 0x00, 0x00, 0x09 }, //DC01-Handshake
		{ 0xFA, 0x0A, 0x03, 0x02, 0x03, 0x00, 0x00, 0x00, 0x00, 0x0C } //DR03-Self Test Request
};

// SPO2 - Patient Type Configuration(11 bytes)

static const uint8_t spo2_patient_type[][11] = { { 0xFA, 0x0B, 0x03, 0x01, 0x04,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x0D },  //Adult
		{ 0xFA, 0x0B, 0x03, 0x01, 0x04, 0x00, 0x01, 0x00, 0x00, 0x00, 0x0E }, // Children
		{ 0xFA, 0x0B, 0x03, 0x01, 0x04, 0x00, 0x02, 0x00, 0x00, 0x00, 0x0F } //Neonate
};

// SPO2 - Sensitivity Configuration (11 bytes)

static const uint8_t spo2_sensitivity[][11] = { { 0xFA, 0x0B, 0x03, 0x01, 0x05,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x0E },  // Low
		{ 0xFA, 0x0B, 0x03, 0x01, 0x05, 0x00, 0x00, 0x00, 0x00, 0x01, 0x0F }, //Middle
		{ 0xFA, 0x0B, 0x03, 0x01, 0x05, 0x00, 0x00, 0x00, 0x00, 0x02, 0x10 }, //High
		{ 0xFA, 0x0B, 0x03, 0x01, 0x05, 0x00, 0x00, 0x00, 0x00, 0x03, 0x11 } //Highest
};

// NIBP - 10 Byte Command Database

static const uint8_t nibp_database[][10] = { { 0xFA, 0x0A, 0x02, 0x01, 0x01,
		0x00, 0x00, 0x00, 0x00, 0x08 }, // DC01-Handshake
		{ 0xFA, 0x0A, 0x02, 0x01, 0x02, 0x00, 0x00, 0x00, 0x00, 0x09 }, // DC01-start measurement
		{ 0xFA, 0x0A, 0x02, 0x02, 0x03, 0x00, 0x00, 0x00, 0x00, 0x0B }, // DR03-Final result request
		{ 0xFA, 0x0A, 0x02, 0x01, 0x20, 0x00, 0x00, 0x00, 0x00, 0x27 } // DC20-stop measurement
};

// NIBP - Patient Type Configuration(11 bytes)

static const uint8_t nibp_patient_type[][11] = { { 0xFA, 0x0B, 0x02, 0x01, 0x10,
		0x00, 0x00, 0x00, 0x00, 0x01, 0x19 }, // DC00-Adult
		{ 0xFA, 0x0B, 0x02, 0x01, 0x10, 0x00, 0x00, 0x00, 0x00, 0x02, 0x1A }, // DC02-Child
		{ 0xFA, 0x0B, 0x02, 0x01, 0x10, 0x00, 0x00, 0x00, 0x00, 0x03, 0x1C } //DC01-Neonate
};

// ECG Command Database (10 Bytes)
static const uint8_t ecg_database[10] = { 0xFA, 0x0A, 0x01, 0x01, 0x01, 0x00,
		0x00, 0x00, 0x00, 0x07  // DC01 - Host Handshake Command
		};

// ECG Patient Type Configuration (11 bytes) — kept 2D since it has multiple entries
static const uint8_t ecg_patient_type[][11] = { { 0xFA, 0x0B, 0x01, 0x01, 0x10,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x17 }, // DC10 - Adult
		{ 0xFA, 0x0B, 0x01, 0x01, 0x10, 0x00, 0x00, 0x00, 0x00, 0x01, 0x18 }, // DC10 - Neonate
		{ 0xFA, 0x0B, 0x01, 0x01, 0x10, 0x00, 0x00, 0x00, 0x00, 0x02, 0x19 } // DC10 - Pediatric (Children)
};

// ECG Signal Configuration Commands
static const uint8_t ecg_leadmode[11] = { 0xFA, 0x0B, 0x01, 0x01, 0x20, 0x00,
		0x00, 0x00, 0x00, 0x01, 0x27  // DC20 - ECG Lead Mode
		};

static const uint8_t ecg_st_tempelate[14] = { 0xFA, 0x0E, 0x01, 0x01, 0x25,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2F // DC25 - ST Template Config
		};

//Updated by:  (SPO2 Real-Time Information)
typedef struct {
	uint16_t PR;      // BPM
	uint8_t spo2;     // SpO2 (%)
	uint16_t PI;      // PI
	uint8_t status1;
	uint8_t status2;
	uint8_t handshakestatus;    //box will be displayed on screen basedon timing
	uint8_t pulseWaveform;
	uint8_t pulseTone;
	uint8_t barGraph;
	uint8_t selfTestResult;
} SPO2_Result_t;
SPO2_Result_t spo2;
//Updated by: DA83 – NIBP Measurement Result
typedef struct {
	uint16_t SYS; // SYS (mmHg)
	uint16_t MAP; // MAP (mmHg)
	uint16_t DIA; // DIA (mmHg)
	uint16_t PI;  // BPM
	uint8_t patient_type;
	uint8_t errorCode;
	uint8_t measurement_Status;
	uint8_t measurement_result_Status;
	uint16_t cuffPressure; // Current cuff pressure (mmHg)
	uint8_t cuffTypeError;
	uint8_t systemStatus;
	uint8_t handshake;
	uint8_t operationtype;
	uint8_t start_end_Status;
} NIBP_Result_t;
NIBP_Result_t nibp;

typedef struct {
	uint8_t status;
	uint16_t ch1, ch2, v1, rr_wave;
	uint16_t hr, rr;
	uint8_t lead_active, ch_active;
	uint8_t ch_overload, cable_type;
	uint8_t hr_ar_ch_Analysis;
	uint8_t arr_code;
	uint16_t last_arr, cur_arr;
	uint8_t arr_status;
	uint8_t st_group;
	int16_t st1, st2, st3;
} ECG_Result_t;
ECG_Result_t ecg;

typedef struct {
	uint8_t temp_status;
	uint16_t t1, t2;
} TEMP_Result_t;
TEMP_Result_t temp;

typedef struct {

	// ── Measurement Values ──────────────────────────────────────

	// ── Error / Status Flags from Byte 6 (status1) ───────────────
	uint8_t hypoperfusion;    // 1 = Low blood perfusion (Bit 0)
	uint8_t motion;           // 1 = Motion interference (Bit 1)
	uint8_t excess_motion;    // 1 = Excessive motion    (Bit 2)
	uint8_t searching;        // 1 = Searching pulse     (Bit 3)
	uint8_t search_timeout;   // 1 = Search too long     (Bit 4)
	uint8_t sensor_off;       // 1 = Probe disconnected  (Bit 5)
	uint8_t finger_off;       // 1 = Finger out          (Bit 6)
	uint8_t probe_fault;      // 1 = Probe hardware fail (Bit 7)

	// ── Error / Status Flags from Byte 7 (status2) ───────────────
	uint8_t hw_error;         // 1 = Hardware error      (Bit 0)
	uint8_t light_high;       // 1 = Ambient light high  (Bit 1)
	uint8_t probe_mismatch;   // 1 = Probe mismatch      (Bit 2)

	// ── Master Status Flag for Display Banner Update ─────────────
	uint8_t status_flag;      // 1 = Any status/error changed

} SPO2_error_t;

// Single global instance
SPO2_error_t spo2_status;

typedef struct {
	uint8_t pulse_flag;
	uint8_t data_flag;
	uint8_t successflag;
	uint8_t errorflag;
} spo2_flag_t;
spo2_flag_t spo2_send;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* DUAL_CORE_BOOT_SYNC_SEQUENCE: Define for dual core boot synchronization    */
/*                             demonstration code based on hardware semaphore */
/* This define is present in both CM7/CM4 projects                            */
/* To comment when developping/debugging on a single core                     */
//#define DUAL_CORE_BOOT_SYNC_SEQUENCE

#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U) /* HW semaphore 0*/
#endif
#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void Ring_buffer_init(ring_buffer_t *l_ring_buffer);
static void Ring_buffer_write(ring_buffer_t *l_ring_buffer, uint8_t *data,
		uint16_t len);
static uint16_t Ring_buffer_count(ring_buffer_t *l_ring_buffer);
static void Ring_buffer_read(ring_buffer_t *l_ring_buffer,
		uint8_t *l_parser_buffer, uint16_t start_pos, uint16_t len);
static void Ring_write_handler(void);
static void Ring_read_handler(void);
static void packet_dispatcher(uint8_t *l_parser_buffer);
static void ecg_handler(void);
static void spo2_handler(void);
static void nibp_handler(void);
static uint8_t Checksum(uint8_t *l_parser_buffer);
static void timer_handler(void);
static void Display_spo2_success(void);
static void Display_spo2_error(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
HAL_StatusTypeDef HAL_TIM_Base_Start_IT(TIM_HandleTypeDef *htim);
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
/* USER CODE BEGIN Boot_Mode_Sequence_0 */
#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
  int32_t timeout;
#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */
/* USER CODE END Boot_Mode_Sequence_0 */

/* USER CODE BEGIN Boot_Mode_Sequence_1 */
#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
  /* Wait until CPU2 boots and enters in stop mode or timeout*/
  timeout = 0xFFFF;
  while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) != RESET) && (timeout-- > 0));
  if ( timeout < 0 )
  {
  Error_Handler();
 }
#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */
/* USER CODE END Boot_Mode_Sequence_1 */
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();
/* USER CODE BEGIN Boot_Mode_Sequence_2 */
#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
/* When system initialization is finished, Cortex-M7 will release Cortex-M4 by means of
HSEM notification */
/*HW semaphore Clock enable*/
__HAL_RCC_HSEM_CLK_ENABLE();
/*Take HSEM */
HAL_HSEM_FastTake(HSEM_ID_0);
/*Release HSEM in order to notify the CPU2(CM4)*/
HAL_HSEM_Release(HSEM_ID_0,0);
/* wait until CPU2 wakes up from stop mode */
timeout = 0xFFFF;
while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) == RESET) && (timeout-- > 0));
if ( timeout < 0 )
{
Error_Handler();
}
#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */
/* USER CODE END Boot_Mode_Sequence_2 */

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
  // Enable DWT cycle counter
  DEM_CR  |= (1 << 24);  // enable trace
  DWT_CTRL |= (1 << 0);  // enable CYCCNT
  DWT_CYCCNT = 0;

  Ring_buffer_init(&uart_ring_buffer);
  memset(g_dma_buffer, 0, sizeof(g_dma_buffer));
  //HAL_UARTEx_ReceiveToIdle_DMA(&huart1, g_dma_buffer, DMA_BUFFER_SIZE);
  HAL_UART_Receive_DMA(&huart1, g_dma_buffer, DMA_BUFFER_SIZE);
  HAL_TIM_Base_Start_IT(&htim7);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

		if (timer_flag == 1) {
			timer_handler();
		}

		if (rx_event_flag == 1) {
			//HAL_UART_Transmit(&huart2, "H", 1, 1);
			Ring_write_handler();
		}

		if (ring_read_flag == 1) {
			Ring_read_handler();
		}

		if (frame_ready_flag == 1) {
			uint32_t _td_start = DWT_CYCCNT;
			packet_dispatcher(g_parser_buffer);
			td_cycles_last = DWT_CYCCNT - _td_start;
			td_us_last     = td_cycles_last / CPU_FREQ_MHZ;
			if (td_cycles_last > td_cycles_max) {
				td_cycles_max = td_cycles_last;
				td_us_max     = td_us_last;
			}
		}

		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 25;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 5;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	timer_flag = 1;
}

void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
	rx_event_flag = 1;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	rx_event_flag = 1;
}

static void timer_handler(void) {
	timer_flag = 0;
	//check ecg_data available
	//check spo2 data available .
	if (spo2_send.data_flag == 1) {
		spo2_send.data_flag = 0;

		if (spo2_send.errorflag == 1) {
			spo2_send.errorflag = 0;
			Display_spo2_error();
			spo2_send.successflag =0;  //these is not the final way...
		}
		if (spo2_send.successflag == 1) {
			spo2_send.successflag = 0;
			Display_spo2_success();
		}
		//transmit
	}

	if (spo2_send.pulse_flag == 1) {
		spo2_send.pulse_flag = 0;
		//transmit
	}
}

void Display_spo2_success(void) {

	char buf[80];
	int len;

	// Always show the actual measured values first
	len = snprintf(buf, sizeof(buf), "SpO2: %d%%  PR: %d bpm  PI: %d\r\n",
			spo2.spo2, spo2.PR, spo2.PI);
	HAL_UART_Transmit(&huart2, (uint8_t*) buf, len, 50);

	// Then show only the highest-priority soft warning, if any
	if (spo2_status.hypoperfusion) {
		HAL_UART_Transmit(&huart2, (uint8_t*) "hypoperfusion error", 20, 50);
		return;
	}

	if (spo2_status.motion) {
		HAL_UART_Transmit(&huart2, (uint8_t*) "motion interference error", 26,
				50);
		return;
	}

	if (spo2_status.excess_motion) {
		HAL_UART_Transmit(&huart2, (uint8_t*) "excessive motion error", 23, 50);
		return;
	}

	if (spo2_status.light_high) {
		HAL_UART_Transmit(&huart2, (uint8_t*) "ambient light high error", 25,
				50);
		return;
	}

}

static void Display_spo2_error(void) {

	if (spo2_status.searching) {
		HAL_UART_Transmit(&huart2, (uint8_t*) "searching pulse error", 22, 50);
		return;
	}

	if (spo2_status.search_timeout) {
		HAL_UART_Transmit(&huart2, (uint8_t*) "search timeout error", 21, 50);
		return;
	}

	if (spo2_status.sensor_off) {
		HAL_UART_Transmit(&huart2, (uint8_t*) "sensor off error", 17, 50);
		return;
	}

	if (spo2_status.finger_off) {
		HAL_UART_Transmit(&huart2, (uint8_t*) "finger off error", 17, 50);
		return;
	}

	if (spo2_status.probe_fault) {
		HAL_UART_Transmit(&huart2, (uint8_t*) "probe fault error", 18, 50);
		return;
	}

	if (spo2_status.hw_error) {
		HAL_UART_Transmit(&huart2, (uint8_t*) "hardware error", 15, 50);
		return;
	}

	if (spo2_status.probe_mismatch) {
		HAL_UART_Transmit(&huart2, (uint8_t*) "probe mismatch error", 21, 50);
		return;
	}

}


static void Ring_buffer_init(ring_buffer_t *l_ring_buffer) {
	l_ring_buffer->head = 0;
	l_ring_buffer->tail = 0;
}

static uint16_t Ring_buffer_count(ring_buffer_t *l_ring_buffer) {
	if (l_ring_buffer->head >= l_ring_buffer->tail) {
		return l_ring_buffer->head - l_ring_buffer->tail;
	} else {
		return (RING_BUFFER_SIZE - l_ring_buffer->tail) + l_ring_buffer->head;
	}
}

static void Ring_buffer_write(ring_buffer_t *l_ring_buffer, uint8_t *data,
		uint16_t len) {
	for (uint16_t i = 0; i < len; i++) {
		uint16_t next_head = (l_ring_buffer->head + 1) % RING_BUFFER_SIZE;

		if (next_head == l_ring_buffer->tail) {
			// rx_overflow_count++;
			continue;
		}

		l_ring_buffer->buffer[l_ring_buffer->head] = data[i];
		l_ring_buffer->head = next_head;
	}
}

static void Ring_buffer_read(ring_buffer_t *l_ring_buffer,
		uint8_t *l_parser_buffer, uint16_t start_pos, uint16_t len) {
	uint16_t i;

	for (i = 0; i < len; i++) {
		l_parser_buffer[i] = l_ring_buffer->buffer[(start_pos + i)
				% RING_BUFFER_SIZE];
	}
	l_ring_buffer->tail = (start_pos + len) % RING_BUFFER_SIZE;

}

static uint8_t Checksum(uint8_t *l_parser_buffer) {
	uint8_t length, received_checksum, calculated_checksum, i;
	uint16_t sum = 0;

	length = l_parser_buffer[1];
	received_checksum = l_parser_buffer[length - 1];

	for (i = 0; i < (length - 1); i++) {
		sum += l_parser_buffer[i];
	}

	calculated_checksum = (uint8_t) sum;

	if (calculated_checksum == received_checksum) {
		return 1;
	}

	return 0;
}

static void Ring_write_handler(void) {
	uint32_t _t_start = DWT_CYCCNT;
	rwh_call_count++;
	uint16_t dma_new_pos, len, len1, len2;
	rx_event_flag = 0;

	/* Current DMA write position = buffer size - remaining DMA counter */
	dma_new_pos = DMA_BUFFER_SIZE - (uint16_t)__HAL_DMA_GET_COUNTER(huart1.hdmarx);

	if (dma_new_pos != g_dma_old_pos) {
		if (dma_new_pos > g_dma_old_pos) {
			len = dma_new_pos - g_dma_old_pos;
			Ring_buffer_write(&uart_ring_buffer, &g_dma_buffer[g_dma_old_pos], len);
		} else {
			len1 = DMA_BUFFER_SIZE - g_dma_old_pos;
			Ring_buffer_write(&uart_ring_buffer, &g_dma_buffer[g_dma_old_pos], len1);
			len2 = dma_new_pos;
			if (len2 > 0) {
				Ring_buffer_write(&uart_ring_buffer, &g_dma_buffer[0], len2);
			}
		}
		g_dma_old_pos = dma_new_pos;
	}

	ring_read_flag = 1;

	rwh_cycles_last = DWT_CYCCNT - _t_start;
	rwh_us_last     = rwh_cycles_last / CPU_FREQ_MHZ;
	if (rwh_cycles_last > rwh_cycles_max) {
		rwh_cycles_max = rwh_cycles_last;
		rwh_us_max     = rwh_us_last;
	}
}

static void Ring_read_handler(void) {
	uint32_t _t_start = DWT_CYCCNT;
	rrh_call_count++;
	uint16_t available_data, length_index;
	uint8_t fa_found_flag = 0;
	uint16_t frame_total_len = 0;
	uint16_t fa_pos = 0;
	uint8_t validate;
	ring_read_flag = 0;

	available_data = Ring_buffer_count(&uart_ring_buffer);

	for (; available_data >= 10; available_data--) {
		if (uart_ring_buffer.buffer[uart_ring_buffer.tail] == 0xFA) {
			fa_pos = uart_ring_buffer.tail;
			length_index = (uart_ring_buffer.tail + 1) % RING_BUFFER_SIZE;
			frame_total_len = uart_ring_buffer.buffer[length_index];
			fa_found_flag = 1;
			break;
		}
		uart_ring_buffer.tail = (uart_ring_buffer.tail + 1) % RING_BUFFER_SIZE;
	}

	if (fa_found_flag == 1) {
		fa_found_flag = 0;

		if (frame_total_len < 10 || frame_total_len > 28) {
			uart_ring_buffer.tail = (uart_ring_buffer.tail + 1)
					% RING_BUFFER_SIZE;
			rrh_cycles_last = DWT_CYCCNT - _t_start;
			rrh_us_last = rrh_cycles_last / CPU_FREQ_MHZ;
			if (rrh_cycles_last > rrh_cycles_max) { rrh_cycles_max = rrh_cycles_last; rrh_us_max = rrh_us_last; }
			return;
		}

		if (available_data < frame_total_len) {
			rrh_cycles_last = DWT_CYCCNT - _t_start;
			rrh_us_last = rrh_cycles_last / CPU_FREQ_MHZ;
			if (rrh_cycles_last > rrh_cycles_max) { rrh_cycles_max = rrh_cycles_last; rrh_us_max = rrh_us_last; }
			return;   // header found, but frame not fully arrived yet — wait
		}

		Ring_buffer_read(&uart_ring_buffer, g_parser_buffer, fa_pos,
				frame_total_len);
		ring_read_flag = 1;
		validate = Checksum(g_parser_buffer);
		if (validate) {
			frame_ready_flag = 1;
		}

	}

	rrh_cycles_last = DWT_CYCCNT - _t_start;
	rrh_us_last     = rrh_cycles_last / CPU_FREQ_MHZ;
	if (rrh_cycles_last > rrh_cycles_max) {
		rrh_cycles_max = rrh_cycles_last;
		rrh_us_max     = rrh_us_last;
	}

}

static void packet_dispatcher(uint8_t *l_parser_buffer) {
	frame_ready_flag = 0;
	frame.parameter_type = l_parser_buffer[2];
	frame.packet_type = l_parser_buffer[3];
	frame.packet_id = l_parser_buffer[4];

	switch (frame.parameter_type) {
	case 0x01:
		ecg_handler();
		break;

	case 0x02:
		nibp_handler();
		break;

	case 0x03:
		spo2_handler();
		break;

	default:
		/* unknown parameter type - ignored intentionally */
		break;

	}
}

static void ecg_handler(void) {
	switch (frame.packet_type) {
	case 0x04:
		switch (frame.packet_id) {

		case 0x81:
			HAL_UART_Transmit_DMA(&huart1, ecg_database, 10);
			break;

		case 0x90:
			ecg.status = g_parser_buffer[9];
			ecg.ch1 = ((g_parser_buffer[11] & 0x0F) << 8) | g_parser_buffer[10];
			ecg.ch2 = (g_parser_buffer[13] << 4)
					| ((g_parser_buffer[12] >> 4) & 0x0F);
			ecg.v1 = ((g_parser_buffer[14] & 0x0F) << 8) | g_parser_buffer[13];

			ecg.rr_wave = (g_parser_buffer[15] << 4)
					| ((g_parser_buffer[14] >> 4) & 0x0F);
			break;

		case 0x91:
			ecg.hr = (g_parser_buffer[10] << 8) | g_parser_buffer[9];
			ecg.rr = (g_parser_buffer[12] << 8) | g_parser_buffer[11];
			break;

		case 0x94:
			ecg.hr_ar_ch_Analysis = g_parser_buffer[9];   //
			break;

		case 0x98:
			ecg.st_group = g_parser_buffer[9];
			ecg.st1 = (g_parser_buffer[11] << 8) | g_parser_buffer[10];
			ecg.st2 = (g_parser_buffer[13] << 8) | g_parser_buffer[12];
			ecg.st3 = (g_parser_buffer[15] << 8) | g_parser_buffer[14];
			break;

		case 0xB0:
			temp.t1 = (g_parser_buffer[10] << 8) | g_parser_buffer[9];
			temp.t2 = (g_parser_buffer[12] << 8) | g_parser_buffer[11];
			temp.temp_status = g_parser_buffer[13];
			break;

		case 0x92:
			ecg.lead_active = g_parser_buffer[9];   //
			ecg.ch_active = g_parser_buffer[11];

			break;

		case 0x93:
			ecg.ch_overload = g_parser_buffer[9];   //
			ecg.cable_type = g_parser_buffer[10];
			break;

		case 0x96:
			ecg.arr_code = g_parser_buffer[9];
			ecg.last_arr = (g_parser_buffer[11] << 8) | g_parser_buffer[10];
			ecg.cur_arr = (g_parser_buffer[13] << 8) | g_parser_buffer[12];
			break;

		case 0x97:
			ecg.arr_status = g_parser_buffer[9];

			break;

		default:
			/* unknown parameter type - ignored intentionally */
			break;

		}
		break;

	case 0x03:
		switch (frame.packet_id) {
		case 0x80:

			break;

		default:
			/* unknown parameter type - ignored intentionally */
			break;

		}
		break;
	}
}

static void spo2_handler(void) {
	switch (frame.packet_type) {
	case 0x04:
		switch (frame.packet_id) {
		case 0x81:
			//HAL_Delay(1);
			HAL_UART_Transmit_DMA(&huart1, spo2_database[0], 10);
			break;

		case 0x84:
			spo2.pulseWaveform = g_parser_buffer[9];
			spo2.pulseTone = g_parser_buffer[9];
			spo2.barGraph = g_parser_buffer[9];
			spo2_send.pulse_flag = 1;
			break;

		case 0x85:
			spo2.PR = (g_parser_buffer[10] << 8) | g_parser_buffer[9];
			spo2.spo2 = g_parser_buffer[11];
			spo2.PI = (g_parser_buffer[13] << 8) | g_parser_buffer[12];
			spo2.status1 = g_parser_buffer[14];
			spo2.status2 = g_parser_buffer[15];
			memset(&spo2_status, 0, sizeof(spo2_status));
			spo2_send.data_flag = 1;
			if (spo2.status1 == 0x00 && spo2.status2 == 0x00) {
				spo2_send.successflag = 1;
			} else {
				if (spo2.status1 & (1 << 0)) {
					spo2_status.hypoperfusion = 1;
					spo2_send.successflag = 1;
				} else
					spo2_status.hypoperfusion = 0;

				if (spo2.status1 & (1 << 1)) {
					spo2_status.motion = 1;
					spo2_send.successflag = 1;
				} else
					spo2_status.motion = 0;

				if (spo2.status1 & (1 << 2)) {
					spo2_status.excess_motion = 1;
					spo2_send.successflag = 1;
				} else
					spo2_status.excess_motion = 0;

				if (spo2.status1 & (1 << 3)) {
					spo2_status.searching = 1;
					spo2_send.errorflag = 1;
				}

				else
					spo2_status.searching = 0;

				if (spo2.status1 & (1 << 4)) {
					spo2_status.search_timeout = 1;
					spo2_send.errorflag = 1;
				} else
					spo2_status.search_timeout = 0;

				if (spo2.status1 & (1 << 5)) {
					spo2_status.sensor_off = 1;
					spo2_send.errorflag = 1;
				} else
					spo2_status.sensor_off = 0;

				if (spo2.status1 & (1 << 6)) {
					spo2_status.finger_off = 1;
					spo2_send.errorflag = 1;
				} else
					spo2_status.finger_off = 0;

				if (spo2.status1 & (1 << 7)) {
					spo2_status.probe_fault = 1;
					spo2_send.errorflag = 1;
				} else
					spo2_status.probe_fault = 0;

				if (spo2.status2 & (1 << 0)) {
					spo2_status.hw_error = 1;
					spo2_send.errorflag = 1;
				} else
					spo2_status.hw_error = 0;

				if (spo2.status2 & (1 << 1)) {
					spo2_status.light_high = 1;
					spo2_send.successflag = 1;
				} else
					spo2_status.light_high = 0;

				if (spo2.status2 & (1 << 2)) {
					spo2_status.probe_mismatch = 1;
					spo2_send.errorflag = 1;
				} else
					spo2_status.probe_mismatch = 0;

			}

			break;

		default:
			/* unknown parameter type - ignored intentionally */
			break;

		}
		break;

	case 0x03:
		switch (frame.packet_id) {
		case 0x80:
			spo2.handshakestatus = g_parser_buffer[9];
			if (spo2.handshakestatus == 0x80) {
				//configuration can be done like self test etc..  and set the flag..
			} else {
				HAL_UART_Transmit_DMA(&huart1, spo2_database[0], 10);
			}
			break;

		case 0x83:
			//	rom ram cpu suceed based on first 4bits
			break;

		default:
			/* unknown parameter type - ignored intentionally */
			break;
		}
		break;
	}

}

static void nibp_handler(void) {
	switch (frame.packet_type) {
	case 0x04:
		switch (frame.packet_id) {
		case 0x81:
			HAL_UART_Transmit_DMA(&huart1, nibp_database[0], 10);
			break;

		case 0x84:
			nibp.cuffPressure = (g_parser_buffer[10] << 8) | g_parser_buffer[9];
			nibp.cuffTypeError = g_parser_buffer[11];
			nibp.systemStatus = g_parser_buffer[12]; //different system operation process

			break;

		case 0x86:
			nibp.operationtype = g_parser_buffer[9];
			nibp.start_end_Status = g_parser_buffer[10];
			if (nibp.start_end_Status == 0x00) {
				HAL_UART_Transmit_DMA(&huart1, nibp_database[2], 10);
			}
			break;

		default:
			/* unknown parameter type - ignored intentionally */
			break;

		}
		break;

	case 0x03:
		switch (frame.packet_id) {
		case 0x80:
			nibp.handshake = g_parser_buffer[9];
			if (nibp.handshake == 0x08) {
				//configuration can be done
			} else {
				HAL_UART_Transmit_DMA(&huart1, nibp_database[0], 10);
			}
			break;

		case 0x83:
			nibp.SYS = (g_parser_buffer[10] << 8) | g_parser_buffer[9];
			nibp.DIA = (g_parser_buffer[12] << 8) | g_parser_buffer[11];
			nibp.MAP = (g_parser_buffer[14] << 8) | g_parser_buffer[13];
			nibp.PI = (g_parser_buffer[16] << 8) | g_parser_buffer[15];
			nibp.patient_type = g_parser_buffer[17];
			nibp.errorCode = g_parser_buffer[18];
			nibp.measurement_Status = g_parser_buffer[19];
			nibp.measurement_result_Status = g_parser_buffer[20];

			break;

		default:
			/* unknown parameter type - ignored intentionally */
			break;
		}
		break;

	}
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
