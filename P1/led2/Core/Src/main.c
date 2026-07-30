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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define RING_BUFFER_SIZE 4096
#define DMA_BUFFER_SIZE 2048
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
    typedef struct
	{
    	uint8_t buffer[RING_BUFFER_SIZE];
    	uint16_t head;
    	uint16_t tail;

    }
    ring_buffer_t;
    ring_buffer_t uart_ring_buffer;

    typedef struct
   	{
       	uint8_t parameter_type;
       	uint8_t packet_type;
       	uint8_t packet_id;

       }
       protocol_frame_t;
       protocol_frame_t frame;

   static uint16_t g_dma_old_pos=0;
   static volatile uint16_t g_dma_size=0;
   static volatile uint8_t rx_event_flag=0;
   static volatile uint8_t frame_ready_flag=0;

   static uint8_t g_dma_buffer[DMA_BUFFER_SIZE];
   static uint8_t g_parser_buffer[64];

   // SPO2 - 10 Byte Command Database

   static const uint8_t spo2_database[][10] ={
   {0xFA, 0x0A, 0x03, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x09}, //DC01-Handshake
   {0xFA, 0x0A, 0x03, 0x02, 0x03, 0x00, 0x00, 0x00, 0x00, 0x0C} //DR03-Self Test Request
   };

   // SPO2 - Patient Type Configuration(11 bytes)

   static const uint8_t spo2_patient_type[][11] = {
   {0xFA, 0x0B, 0x03, 0x01, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0D},  //Adult
   {0xFA, 0x0B, 0x03, 0x01, 0x04, 0x00, 0x01, 0x00, 0x00, 0x00, 0x0E},  // Children
   {0xFA, 0x0B, 0x03, 0x01, 0x04, 0x00, 0x02, 0x00, 0x00, 0x00, 0x0F}   //Neonate
   };

   // SPO2 - Sensitivity Configuration (11 bytes)

   static const uint8_t spo2_sensitivity[][11] =
   {
       {0xFA, 0x0B, 0x03, 0x01, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0E},  // Low
       {0xFA, 0x0B, 0x03, 0x01, 0x05, 0x00, 0x00, 0x00, 0x00, 0x01, 0x0F},  //Middle
       {0xFA, 0x0B, 0x03, 0x01, 0x05, 0x00, 0x00, 0x00, 0x00, 0x02, 0x10},  //High
       {0xFA, 0x0B, 0x03, 0x01, 0x05, 0x00, 0x00, 0x00, 0x00, 0x03, 0x11}  //Highest
   };

   // NIBP - 10 Byte Command Database

   static const uint8_t nibp_database[][10] ={
   {0xFA, 0x0A, 0x02, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x08}, // DC01-Handshake
   {0xFA, 0x0A, 0x02, 0x01, 0x02, 0x00, 0x00, 0x00, 0x00, 0x09}, // DC01-start measurement
   {0xFA, 0x0A, 0x02, 0x02, 0x03, 0x00, 0x00, 0x00, 0x00, 0x0B}, // DR03-Final result request
   {0xFA, 0x0A, 0x02, 0x01, 0x20, 0x00, 0x00, 0x00, 0x00, 0x27} // DC20-stop measurement
   };

   // NIBP - Patient Type Configuration(11 bytes)

   static const uint8_t nibp_patient_type[][11]= {
   {0xFA, 0x0B, 0x02, 0x01, 0x10, 0x00, 0x00, 0x00, 0x00, 0x01, 0x19}, // DC00-Adult
   {0xFA, 0x0B, 0x02, 0x01, 0x10, 0x00, 0x00, 0x00, 0x00, 0x02, 0x1A}, // DC02-Child
   {0xFA, 0x0B, 0x02, 0x01, 0x10, 0x00, 0x00, 0x00, 0x00, 0x03, 0x1C} //DC01-Neonate
   };

   //Updated by:  (SPO2 Real-Time Information)
   typedef struct
   {
       uint16_t PR;      // BPM
       uint8_t spo2;     // SpO2 (%)
       uint16_t PI;      // PI
       uint8_t status1;
       uint8_t status2;
       uint8_t handshakestatus;      //box will be displayed on screen basedon timing
       uint8_t pulseWaveform;
       uint8_t pulseTone;
       uint8_t barGraph;
       uint8_t selfTestResult;
   }
       SPO2_Result_t;
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
   }
       NIBP_Result_t;
       NIBP_Result_t nibp;


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
static void Ring_buffer_init(ring_buffer_t *l_ring_buffer);
static void Ring_buffer_write(ring_buffer_t *l_ring_buffer, uint8_t *data, uint16_t len);
static uint16_t Ring_buffer_count(ring_buffer_t *l_ring_buffer );
static void Ring_buffer_read(ring_buffer_t *l_ring_buffer, uint8_t *l_parser_buffer, uint16_t start_pos, uint16_t len);
static void Ring_handler(void);
static void packet_dispatcher(uint8_t *l_parser_buffer);
static void ecg_handler(void);
static void spo2_handler(void);
static void nibp_handler(void);
static uint8_t Checksum(uint8_t *l_parser_buffer);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */


  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  Ring_buffer_init(&uart_ring_buffer);
  memset( g_dma_buffer, 0, sizeof( g_dma_buffer));
  HAL_UARTEx_ReceiveToIdle_DMA(&huart1, g_dma_buffer, DMA_BUFFER_SIZE );
  __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	 if(rx_event_flag==1)
	 {
		 Ring_handler();
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{

    if (huart->Instance == USART1) {
        rx_event_flag = 1;
        g_dma_size= Size;

    }
}

void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
	 rx_event_flag = 1;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	rx_event_flag = 1;
}

static void Ring_buffer_init(ring_buffer_t *l_ring_buffer)
{
    l_ring_buffer->head=0;
    l_ring_buffer->tail=0;
}

static uint16_t Ring_buffer_count(ring_buffer_t *l_ring_buffer) {
    if (l_ring_buffer->head >= l_ring_buffer->tail) {
        return l_ring_buffer->head - l_ring_buffer->tail;
    } else {
        return (RING_BUFFER_SIZE - l_ring_buffer->tail) + l_ring_buffer->head;
    }
}

static void Ring_buffer_write(ring_buffer_t *l_ring_buffer, uint8_t *data, uint16_t len)
{
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

static void Ring_buffer_read(ring_buffer_t *l_ring_buffer, uint8_t *l_parser_buffer, uint16_t start_pos, uint16_t len)
{
    uint16_t i;

    for (i = 0; i < len; i++)
    {
        l_parser_buffer[i] = l_ring_buffer->buffer[(start_pos + i) % RING_BUFFER_SIZE];
    }
    l_ring_buffer->tail = (start_pos + len) % RING_BUFFER_SIZE;

   // frame_ready_flag=1;

   // HAL_UART_Transmit_IT(&huart2, g_parser_buffer, len); //debug but turned off the


}

static uint8_t Checksum(uint8_t *l_parser_buffer)
{
    uint8_t length, received_checksum, calculated_checksum, i;
    uint16_t sum = 0;

    length = l_parser_buffer[1];
    received_checksum = l_parser_buffer[length - 1];

    for (i = 0; i < (length - 1); i++)
    {
        sum += l_parser_buffer[i];
    }

    calculated_checksum = (uint8_t)sum;

    if (calculated_checksum == received_checksum)
    {
        return 1;
    }

    return 0;
}


static void packet_dispatcher(uint8_t *l_parser_buffer)
{
	frame_ready_flag=0;
	frame.parameter_type = l_parser_buffer[2];
	frame.packet_type= l_parser_buffer[3];
	frame.packet_id=l_parser_buffer[4];

	switch(frame.parameter_type)
	{
	case 0x01:
		ecg_handler();
		break;

	case 0x02:
		spo2_handler();
		break;

	case 0x03:
		nibp_handler();
		break;

	default:
		 /* unknown parameter type - ignored intentionally */
        break;

	}
}

static void ecg_handler(void)
{
	if(frame.packet_type==0x04)
	{
		switch(frame.packet_id)
			{
			case 0x90:
				//ecg_handler();
				break;

			case 0x91:
				//spo2_handler();
				break;

			case 0x92:
				//nibp_handler();
				break;

			case 0x96:
				//ecg_handler();
				break;

			case 0x98:
				//spo2_handler();
				break;

			case 0x99:
				//nibp_handler();
				break;

			default:
			        /* unknown parameter type - ignored intentionally */
			    break;

			}
	}
	else if(frame.packet_type==0x03)
		{
		switch(frame.packet_id)
					{
					case 0x80:
					//	ecg_handler();
						break;

					default:
					        /* unknown parameter type - ignored intentionally */
					   break;

					}

		}


}

static void spo2_handler(void)
{
	if(frame.packet_type==0x04)
	{
		switch(frame.packet_id)
			{
			case 0x81:
				HAL_UART_Transmit_DMA(&huart1, spo2_database[0], 10);
				break;

			case 0x84:
			     spo2.pulseWaveform=g_parser_buffer[9];
			     spo2.pulseTone=g_parser_buffer[9];
			     spo2.barGraph=g_parser_buffer[9];
				break;

			case 0x85:
				spo2.PR=(g_parser_buffer[10]<<8) | g_parser_buffer[9];
				spo2.spo2=g_parser_buffer[11];
				spo2.PI=(g_parser_buffer[13]<<8) | g_parser_buffer[12];
				spo2.status1=g_parser_buffer[14];
				spo2.status2=g_parser_buffer[15];
				break;

			default:
			        /* unknown parameter type - ignored intentionally */
			    break;

			}
	}
	else if(frame.packet_type==0x03)
		{
		switch(frame.packet_id)
					{
					case 0x80:
					  spo2.handshakestatus=g_parser_buffer[9];
					  if(spo2.handshakestatus==0x80)
					  {
						  //configuration can be done like self test etc..  and set the flag..
					  }
					  else
					  {
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
		}



}

static void nibp_handler(void)
{
	if(frame.packet_type==0x04)
	{
		switch(frame.packet_id)
			{
			case 0x81:
				HAL_UART_Transmit_DMA(&huart1, nibp_database[0], 10);
				break;

			case 0x84:
				nibp.cuffPressure =(g_parser_buffer[10]<<8) | g_parser_buffer[9];
				nibp.cuffTypeError=g_parser_buffer[11];
				nibp.systemStatus=g_parser_buffer[12];//different system operation process
				break;

			case 0x86:
				nibp.operationtype=g_parser_buffer[9];
				nibp.start_end_Status=g_parser_buffer[10];
				if(nibp.start_end_Status==0x00)
				{
					HAL_UART_Transmit_DMA(&huart1, nibp_database[2], 10);
				}
				break;

			default:
			        /* unknown parameter type - ignored intentionally */
			    break;

			}
	}

	else if(frame.packet_type==0x03)
		{
		switch(frame.packet_id)
					{
					case 0x80:
						spo2.handshakestatus=g_parser_buffer[9];
	                     if(spo2.handshakestatus==0x08)
						  {
												  //configuration can be done
						  }
					    else
						  {
						  HAL_UART_Transmit_DMA(&huart1, spo2_database[0], 10);
						  }
						break;

					case 0x83:
					nibp.SYS=(g_parser_buffer[10]<<8) | g_parser_buffer[9];
					nibp.DIA=(g_parser_buffer[12]<<8) | g_parser_buffer[11];
					nibp.MAP=(g_parser_buffer[14]<<8) | g_parser_buffer[13];
					nibp.PI=(g_parser_buffer[16]<<8) | g_parser_buffer[15];
					nibp.patient_type=g_parser_buffer[17];
					nibp.errorCode=g_parser_buffer[18];
					nibp.measurement_Status=g_parser_buffer[19];
					nibp.measurement_result_Status=g_parser_buffer[20];
						break;

					default:
					        /* unknown parameter type - ignored intentionally */
					    break;
					}
		}



}



static void Ring_handler(void)
{
	uint16_t len,len1,len2;
	uint16_t available_data,length_index;
	uint8_t fa_found_flag=0;
	uint16_t frame_total_len=0;
	uint16_t fa_pos=0;
	uint8_t validate;

	rx_event_flag=0;
	if(g_dma_size>g_dma_old_pos)
	{
		len = g_dma_size - g_dma_old_pos;
	    Ring_buffer_write(&uart_ring_buffer , &g_dma_buffer[g_dma_old_pos],  len);
	}
	else
	{
		len1= DMA_BUFFER_SIZE - g_dma_old_pos;
		Ring_buffer_write(&uart_ring_buffer , &g_dma_buffer[g_dma_old_pos],  len1);

	    len2=g_dma_size;
	    if(len2>0)
	    {
	    	Ring_buffer_write(&uart_ring_buffer , &g_dma_buffer[0],  len2);
	    }

	}

	if(g_dma_size==DMA_BUFFER_SIZE)
	{
		g_dma_old_pos=0;
	}
	else
	{
		g_dma_old_pos=g_dma_size;
	}

	//

	//set the flag for read
//-----------------
	available_data = Ring_buffer_count(&uart_ring_buffer);

	for( ; available_data>=10; available_data--)
	{
		if(uart_ring_buffer.buffer[uart_ring_buffer.tail]==0xFA)
		{
			 fa_pos= uart_ring_buffer.tail ;
		     length_index = (uart_ring_buffer.tail + 1) % RING_BUFFER_SIZE;
		     frame_total_len = uart_ring_buffer.buffer[length_index];
		     fa_found_flag=1;
			break;
		}
		uart_ring_buffer.tail = (uart_ring_buffer.tail + 1) % RING_BUFFER_SIZE;
	}

	if(fa_found_flag==1)
	{
		fa_found_flag=0;

    if ( frame_total_len < 10 ||  frame_total_len > 28) {
        uart_ring_buffer.tail = (uart_ring_buffer.tail + 1) % RING_BUFFER_SIZE;
        return;
    }

    if (available_data < frame_total_len) {
        return;   // header found, but frame not fully arrived yet — wait
    }

    Ring_buffer_read(&uart_ring_buffer, g_parser_buffer, fa_pos, frame_total_len);
      validate = Checksum(g_parser_buffer);
		if(validate)
		{
			frame_ready_flag=1;
		}
   // ring_buffer_ready_flag=1;
	}



   //read

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

#ifdef  USE_FULL_ASSERT
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
