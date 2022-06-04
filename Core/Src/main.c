/* USER CODE BEGIN Header */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 UART_HandleTypeDef huart2;

osThreadId defaultTaskHandle;
uint32_t defaultTaskBuffer[ 128 ];
osStaticThreadDef_t defaultTaskControlBlock;
/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define ECHO_SEC (2)
#define ECHODELAY_MS ((uint32_t)ECHO_SEC*1000)
#define NBUFFECHO (9600/10*ECHO_SEC)

#define NQUEUE 	(9600/10*10/1000+1)

volatile uint8_t rxEchoISR[1+1];//в него Rx драйвер IRQ
volatile uint8_t bTxEchoISR[1+1];//буфер передачи

typedef struct{
	uint32_t time;
	uint8_t data;
}data_t;

data_t RxData[NBUFFECHO];
data_t RxQ[NQUEUE];	 //буфер для переноса из очереди в рабочий RxData[]

static size_t timeTail=0;
static size_t RxEchoTail=0;
static size_t nTxDataQ=0;
static size_t TxIndex=0;

volatile uint8_t EchoToTransfer=0;

xQueueHandle rxQueueEchoData;

#define phUartEcho &huart2

#define ECHOUART USART2


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	uint16_t TxXferCount=huart->TxXferCount;
	uint16_t TxXferSize=huart->TxXferSize;

	if (huart-> Instance == ECHOUART) // Echo
	{
		if(TxXferSize == 0 ) // какая-то ошибка
		{
			HAL_UART_Transmit_IT(huart, (uint8_t *)(bTxEchoISR),EchoToTransfer);
		}
		else if (TxXferSize > 0 & TxXferCount < TxXferSize)
		{
			if(TxXferCount>0)
				HAL_UART_Transmit_IT(huart, (uint8_t *)(bTxEchoISR+TxXferSize-TxXferCount),TxXferCount);
		}
	}

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart-> Instance == ECHOUART)
	{
		uint16_t RxXferCount=huart->RxXferCount;
		uint16_t RxXferSize=huart->RxXferSize;

		if(RxXferSize == 0)
		{
			printf("HAL_UART_RxCpltCallback Echo????");
		}
		else if(RxXferSize > 0 && RxXferCount < RxXferSize)
		{
			BaseType_t xHigherPriorityTaskWoken;
			xHigherPriorityTaskWoken = pdFALSE;

			for(int ii=0; ii<  (RxXferSize-RxXferCount); ii++)
			{
				data_t tmp;
					tmp.time = HAL_GetTick(); //захват времени, по соотношению к 2 сек, шкала времени избыточна
					tmp.data = rxEchoISR[ii];
				xQueueSendToBackFromISR(rxQueueEchoData, &tmp , &xHigherPriorityTaskWoken );
			}


			if( xHigherPriorityTaskWoken )
			{
				portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
			}

		}
		HAL_UART_Receive_IT(huart, (uint8_t *)(rxEchoISR),1);
	}
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	if (huart-> Instance == ECHOUART)
	{
		uint16_t RxXferCount=huart->RxXferCount;
		uint16_t RxXferSize=huart->RxXferSize;

		printf("< HAL_UART_ErrorCallback ECHOUART\n");
		printf("< RxXferCount=%d RxXferSize=%d\n",RxXferCount,RxXferSize);

		HAL_UART_Receive_IT(phUartEcho, (uint8_t *)(rxEchoISR),1);
	}

}
void HAL_UART_AbortReceiveCpltCallback(UART_HandleTypeDef *huart)
{
	printf("HAL_UART_AbortReceiveCpltCallback \n");
}
void HAL_UART_AbortTransmitCpltCallback(UART_HandleTypeDef *huart)
{
	printf("HAL_UART_AbortTransmitCpltCallback \n");
}
void HAL_UART_AbortCpltCallback(UART_HandleTypeDef *huart)
{
	printf("HAL_UART_AbortCpltCallback \n");
}











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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadStaticDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128, defaultTaskBuffer, &defaultTaskControlBlock);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
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
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */
  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BLINK_GPIO_Port, BLINK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BLINK_Pin */
  GPIO_InitStruct.Pin = BLINK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BLINK_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */

	  /* Infinite loop */
	  for(;;)
	  {

			HAL_StatusTypeDef res;

			rxQueueEchoData = xQueueCreate( NQUEUE , sizeof( data_t ) );

			if( rxQueueEchoData == NULL )
				printf("rxQueueEcho Queue was not created and must not be used.\n");

			// Обязательно сначала запускаем прием
			if(res=HAL_UART_Receive_IT(phUartEcho, (uint8_t *)rxEchoISR, 1) == HAL_OK)
				; //printf("HAL_OK\n");
			else if(res = HAL_ERROR)
				printf("Echo HAL_ERROR\n");
			else  if(res = HAL_BUSY)
				printf("Echo HAL_BUSY\n");


			const TickType_t xMaxExpectedBlockTime = pdMS_TO_TICKS( 1 );

			for(;;)
			{
				uint8_t numbsData = xQueueReceive(rxQueueEchoData, (data_t*)&RxQ,  xMaxExpectedBlockTime);
				if(numbsData > 0){
					nTxDataQ+=numbsData;
					if ( nTxDataQ >= NBUFFECHO ){
						printf("чего-то пошло не то, данные не ушли!");
					}
					for(int ii=0; ii< numbsData ; ii++)
					{
						RxData[timeTail].time=RxQ[ii].time;
						RxData[timeTail].data=RxQ[ii].data;
						if ( ++timeTail == NBUFFECHO ){
							timeTail = 0 ;
						}
					}

				}//забор из очередей
				//
				if (nTxDataQ){ //имеются данные для передачи
					volatile uint32_t time=HAL_GetTick();
					if (  ( time-RxData[TxIndex].time ) >= ECHODELAY_MS  ){


						//просто проверка Tx буфера
						if ( phUartEcho.TxXferCount > 0 ){
							vTaskDelay(1);//и опустошение, если биток, не блокируя, но можно while+timeout
						}

						if (HAL_UART_Transmit_IT(phUartEcho, (uint8_t *)&RxData[TxIndex].data,1 ) != HAL_OK);

						TxIndex++;
						if (TxIndex == NBUFFECHO){
							TxIndex = 0 ;
						}
						nTxDataQ--;
						if( nTxDataQ > NBUFFECHO ){
							printf("чего-то не то, nTxDataQ<0!");
						}
					}
				}
					#define fromMX(xx) xx##_GPIO_Port, xx##_Pin
					static char c;
				vTaskDelay( pdMS_TO_TICKS(1) );
					if ((++c & 63) == 63){
						HAL_GPIO_TogglePin(fromMX(BLINK));//работает! 63мс+блок очереди
					}
			}
	  }




  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
