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
#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "arm_math.h"
#include "usbd_cdc_if.h"
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

COM_InitTypeDef BspCOMInit;
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */
volatile uint32_t adc_value = 0;

#define ADC_BUFFER_SIZE 4096
__attribute__((section(".adc_buffer"), aligned(32)))
uint16_t adc_buffer[ADC_BUFFER_SIZE];

volatile uint32_t dma_half_count = 0;
volatile uint32_t dma_full_count = 0;
volatile uint16_t sample_seen_by_cpu = 0;
volatile uint32_t avg_seen_by_cpu = 0;

volatile uint16_t buf_min = 0;
volatile uint16_t buf_max = 0;
volatile uint16_t buf_p2p = 0;

#define FFT_SIZE 1024

// FFT buffers (can stay in cacheable memory as only DMA buffer needs MPU)
static float32_t fft_input[FFT_SIZE];
static float32_t fft_output[FFT_SIZE]; // [r0, i0, r1, i1 ...]
static float32_t fft_magnitudes[FFT_SIZE / 2]; // half-spectrum, real to conjugate symmetric

static arm_rfft_fast_instance_f32 fft_inst;

// debugger values for testing
volatile uint32_t peak_bin = 0;
volatile float32_t peak_magnitude = 0.0f;
volatile float32_t dc_magnitude = 0.0f;
volatile uint32_t measured_sample_rate = 0;

volatile uint32_t sys_clock_hz = 0;
volatile uint32_t hclk_hz = 0;

// Primitives for FreeRTOS
osSemaphoreId_t adcDataSemaphore;
osMessageQueueId_t adcQueue;

// Task handles
osThreadId_t acquisitionTaskHandle;
osThreadId_t processingTaskHandle;

// Shared state (processing to comms), raw peak bin/mag, protect later with mutex
typedef struct {
    uint32_t peak_bin;
    float32_t peak_magnitude;
    uint32_t timestamp_tick;
} ProcessingResults_t;

volatile ProcessingResults_t latest_results;

// message for queue, tell which half of buffer is ready
typedef enum {
	BUFFER_HALF_FIRST = 0,
	BUFFER_HALF_SECOND = 1,
} BufferHalf_t;

// Task health counters
volatile uint32_t acq_task_runs = 0;
volatile uint32_t proc_task_runs = 0;
volatile uint32_t coms_task_runs = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */
static void MPU_Config(void);
static void AcquisitionTask(void *argument);
static void ProcessingTask(void *argument);
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
	MPU_Config();
	SCB_EnableICache();
	SCB_EnableDCache();
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
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, ADC_BUFFER_SIZE);
  if (arm_rfft_fast_init_f32(&fft_inst, FFT_SIZE) != ARM_MATH_SUCCESS) {
	  Error_Handler();
  }
  sys_clock_hz = HAL_RCC_GetSysClockFreq();
  hclk_hz = HAL_RCC_GetHCLKFreq();
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  adcDataSemaphore = osSemaphoreNew(4, 0, NULL);
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  adcQueue = osMessageQueueNew(8, sizeof(BufferHalf_t), NULL);
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  const osThreadAttr_t acq_attr = {
		  .name = "AcqTask",
		  .stack_size = 1024,
		  .priority = (osPriority_t)osPriorityHigh,
  };
  acquisitionTaskHandle = osThreadNew(AcquisitionTask, NULL, &acq_attr);

  const osThreadAttr_t proc_attr = {
		  .name = "ProcTask",
		  .stack_size = 4096, // fft needs some stack
		  .priority = (osPriority_t)osPriorityNormal,
  };
  processingTaskHandle = osThreadNew(ProcessingTask, NULL, &proc_attr);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Initialize leds */
  BSP_LED_Init(LED_GREEN);
  BSP_LED_Init(LED_YELLOW);
  BSP_LED_Init(LED_RED);

  /* Initialize USER push-button, will be used to trigger an interrupt each time it's pressed.*/
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  /* Initialize COM1 port (115200, 8 bits (7-bit data + 1 stop bit), no parity */
  BspCOMInit.BaudRate   = 115200;
  BspCOMInit.WordLength = COM_WORDLENGTH_8B;
  BspCOMInit.StopBits   = COM_STOPBITS_1;
  BspCOMInit.Parity     = COM_PARITY_NONE;
  BspCOMInit.HwFlowCtl  = COM_HWCONTROL_NONE;
  if (BSP_COM_Init(COM1, &BspCOMInit) != BSP_ERROR_NONE)
  {
    Error_Handler();
  }

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // copy and convert adc samples (16 bit) to f32
	  // we can subtract 32768 to make 0 mean, so DC bin doesnt dominate
	  for (int i = 0; i < FFT_SIZE; ++i) {
		fft_input[i] = (float32_t)adc_buffer[i] - 32768.0f;
	  }

	  // foward Real FFT, i.e input to output (with complex interleaved)
	  arm_rfft_fast_f32(&fft_inst, fft_input, fft_output, 0);

	  // compute the magnitude of each complex bin
	  // output has FFT_SIZE/2 pairs, we skip Nyquist
	  arm_cmplx_mag_f32(fft_output, fft_magnitudes, FFT_SIZE / 2);

	  // now we find the peak, skip bin 0 and start at bin 2
	  // prevents leakage
	  dc_magnitude = fft_magnitudes[0];
	  uint32_t best_bin = 1;
	  float32_t best_mag = 0.0f;

	  for (uint32_t i = 2; i < FFT_SIZE/2; ++i) {
		  if (fft_magnitudes[i] > best_mag) {
			  best_mag = fft_magnitudes[i];
			  best_bin = i;
		  }
	  }

	  peak_bin = best_bin;
	  peak_magnitude = best_mag;

	  static uint32_t last_full_count = 0;
	  static uint32_t last_tick = 0;
	  uint32_t now = HAL_GetTick();
	  if (now - last_tick >= 1000) {
	      uint32_t delta = dma_full_count - last_full_count;
	      measured_sample_rate = delta * ADC_BUFFER_SIZE;
	      last_full_count = dma_full_count;
	      last_tick = now;
	  }

	  HAL_GPIO_TogglePin(LED1_GPIO_PORT, LED1_PIN);
	  HAL_Delay(200);
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
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 60;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_16B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_CIRCULAR;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.Oversampling.Ratio = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_8CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc) {
    dma_half_count++;
    /* First half of buffer is fresh. Give semaphore to wake acquisition task. */
    if (adcDataSemaphore != NULL) {
        osSemaphoreRelease(adcDataSemaphore);
    }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
    dma_full_count++;
    /* Second half of buffer is fresh. */
    if (adcDataSemaphore != NULL) {
        osSemaphoreRelease(adcDataSemaphore);
    }
}

static void AcquisitionTask(void *argument) {
    BufferHalf_t next_half = BUFFER_HALF_FIRST;

    for (;;) {
        // Block until DMA callback gives semaphore
        osSemaphoreAcquire(adcDataSemaphore, osWaitForever);

        // Post which half is now fresh to the processing queue
        osMessageQueuePut(adcQueue, &next_half, 0U, 0U);

        // Alternate for next iteration
        next_half = (next_half == BUFFER_HALF_FIRST) ? BUFFER_HALF_SECOND : BUFFER_HALF_FIRST;

        acq_task_runs++;
    }
}

static void ProcessingTask(void *argument) {
    BufferHalf_t half;

    for (;;) {
        // Block until acquisition task posts
        if (osMessageQueueGet(adcQueue, &half, NULL, osWaitForever) != osOK) {
            continue;
        }

        // Determine which 2048-sample slice to run FFT over. We still FFT only FFT_SIZE (1024)
        // samples from the start of that half for consistency with earlier behavior.
        uint16_t *src = (half == BUFFER_HALF_FIRST) ? &adc_buffer[0] : &adc_buffer[ADC_BUFFER_SIZE / 2];

        for (int i = 0; i < FFT_SIZE; i++) {
            fft_input[i] = (float32_t)src[i] - 32768.0f;
        }
        arm_rfft_fast_f32(&fft_inst, fft_input, fft_output, 0);
        arm_cmplx_mag_f32(fft_output, fft_magnitudes, FFT_SIZE / 2);

        dc_magnitude = fft_magnitudes[0];
        uint32_t best_bin = 1;
        float32_t best_mag = 0.0f;
        for (uint32_t i = 2; i < FFT_SIZE / 2; i++) {
            if (fft_magnitudes[i] > best_mag) {
                best_mag = fft_magnitudes[i];
                best_bin = i;
            }
        }

        peak_bin = best_bin;
        peak_magnitude = best_mag;

        // Publish to shared struct (will be mutex-protected later)
        latest_results.peak_bin = best_bin;
        latest_results.peak_magnitude = best_mag;
        latest_results.timestamp_tick = osKernelGetTickCount();

        proc_task_runs++;
    }
}

extern uint32_t _sadc_buffer;
static void MPU_Config(void) {
	MPU_Region_InitTypeDef mpu = {0};
	HAL_MPU_Disable();

	mpu.Enable           = MPU_REGION_ENABLE;
	mpu.Number           = MPU_REGION_NUMBER0;
	mpu.BaseAddress      = (uint32_t)&_sadc_buffer;
	mpu.Size             = MPU_REGION_SIZE_8KB;
	mpu.AccessPermission = MPU_REGION_FULL_ACCESS;
	mpu.IsBufferable     = MPU_ACCESS_BUFFERABLE;
	mpu.IsCacheable      = MPU_ACCESS_NOT_CACHEABLE;
	mpu.IsShareable      = MPU_ACCESS_SHAREABLE;
	mpu.TypeExtField     = MPU_TEX_LEVEL0;
	mpu.SubRegionDisable = 0x00;
	mpu.DisableExec      = MPU_INSTRUCTION_ACCESS_DISABLE;

	HAL_MPU_ConfigRegion(&mpu);
	HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 5 */
  static uint32_t last_full_count = 0;
  static uint32_t last_tick = 0;
  static uint32_t last_tx = 0;
  char tx_buf[128];
  /* Infinite loop */
  for(;;)
  {
	  // We will keep it at 30Hz for now, for sample rate
	  uint32_t now = osKernelGetTickCount();
	  if (now - last_tick >= 1000) {
		  uint32_t delta = dma_full_count - last_full_count;
		  measured_sample_rate = delta * ADC_BUFFER_SIZE;
		  last_full_count = dma_full_count;
		  last_tick = now;
	  }

	  // Heartbeat over USB CDC every 500ms
	  if (now - last_tx >= 500) {
	      int n = snprintf(tx_buf, sizeof(tx_buf),
	          "tick=%lu Fs=%lu peak_bin=%lu peak_mag=%.0f\r\n",
	          now,
	          measured_sample_rate,
	          latest_results.peak_bin,
	          latest_results.peak_magnitude);
	      if (n > 0) {
	          CDC_Transmit_FS((uint8_t *)tx_buf, (uint16_t)n);
	      }
	      last_tx = now;
	  }

	  HAL_GPIO_TogglePin(LED1_GPIO_PORT, LED1_PIN);
	  coms_task_runs++;

	  osDelay(33);
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
  if (htim->Instance == TIM6)
  {
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
