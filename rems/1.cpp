

volatile uint8_t txSlave[200];
volatile uint8_t rxSlaveISR[200];
volatile uint8_t SlaveToTransfer=0;
volatile uint8_t rx1Slave[200];

xQueueHandle rxQueueSlave;

UART_HandleTypeDef *phuartSlave;

#define ECHOUART USART2

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
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	uint16_t TxXferCount=huart->TxXferCount;
	uint16_t TxXferSize=huart->TxXferSize;

	if (huart-> Instance == ECHOUART) // SLAVE
	{
		if(TxXferSize == 0 ) // какая-то ошибка
		{
			//printf(". HAL_UART_TxCpltCallback count=%d  size=%d \n",count,size);
			HAL_UART_Transmit_IT(huart, (uint8_t *)(txSlave),SlaveToTransfer);

		}
		else if (TxXferSize > 0 & TxXferCount < TxXferSize)
		{
			if(TxXferCount>0)
				HAL_UART_Transmit_IT(huart, (uint8_t *)(txSlave+TxXferSize-TxXferCount),TxXferCount);
			/*else
				txCmpl=1;*/

			//printf("USART2 HAL_UART_TxCpltCallback TxXferCount=%d  TxXferSize=%d \n",count,size);
		}
	}

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart-> Instance == ECHOUART)
	{
		uint16_t RxXferCount=huart->RxXferCount;
		uint16_t RxXferSize=huart->RxXferSize;

		if(RxXferSize == 0) // похоже на сбой
		{
			printf("HAL_UART_RxCpltCallback Slave????");
		}
		else if(RxXferSize > 0 && RxXferCount < RxXferSize)
		{
			BaseType_t xHigherPriorityTaskWoken;
			xHigherPriorityTaskWoken = pdFALSE; // We have not woken a task at the start of the ISR.

			for(int ii=0; ii<  (RxXferSize-RxXferCount); ii++)
			{
				xQueueSendToBackFromISR(rxQueueSlave, &rxSlaveISR[ii], &xHigherPriorityTaskWoken );
			}

			/*we can switch context if necessary. */
			if( xHigherPriorityTaskWoken )
			{
				// Actual macro used here is port specific.
				//taskYIELD_FROM_ISR ();
				portYIELD_FROM_ISR(xHigherPriorityTaskWoken); //для ARM7
				//taskYIELD() // для AVR
			}
			//printf("HAL_UART_RxCpltCallback Slave\n");
		}
		HAL_UART_Receive_IT(huart, (uint8_t *)(rxSlaveISR),1);
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

		HAL_UART_Receive_IT(phuartSlave, (uint8_t *)(rxSlaveISR),1);
		// проблема в приемном буфере, ПЕРЕПОЛНЕНИЕ, не все вытащили ДО прихода следующей пачки данных
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
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */


/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
		HAL_StatusTypeDef res;

		rxQueueSlave = xQueueCreate( 50, sizeof( uint8_t ) );
		if( rxQueueSlave == NULL )
			printf("rxQueueSlave Queue was not created and must not be used.\n");

		// Обязательно сначала запускаем прием
		if(res=HAL_UART_Receive_IT(phuartSlave, (uint8_t *)rxSlaveISR, 1) == HAL_OK)
			; //printf("HAL_OK\n");
		else if(res = HAL_ERROR)
			printf("Slave HAL_ERROR\n");
		else  if(res = HAL_BUSY)
			printf("Slave HAL_BUSY\n");


		const TickType_t xMaxExpectedBlockTime = pdMS_TO_TICKS( 2000 );

		for(;;)
		{
			uint8_t received=0;
			received = SlaveGetFromQueue(rxQueueSlave, (uint8_t*)rx1Slave,  xMaxExpectedBlockTime);
			if(received > 0)
			{
				printf("Slave %d -> ",received);

				for(int ii=0; ii< received ; ii++)
				{
					printf("%0.2X ",rx1Slave[ii]);
					txSlave[ii]=rx1Slave[ii];
				}

				printf("\n");

				if (HAL_UART_Transmit_IT(phuartSlave, (uint8_t *)txSlave,received ) != HAL_OK)
					;

			}

			vTaskDelay(1);
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
