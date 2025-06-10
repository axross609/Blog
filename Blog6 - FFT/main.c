/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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

// Used mainly for calculations
#include "math.h"

// Used for printing strings to serial console
#include <stdio.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define PI	3.14159265358979f

// Number of FFT bits
#define FFT_BITS	9
// Number of samples (FFT_SIZE = 2^FFT_BITS)
#define FFT_SIZE	512

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

// Macro to convert 16bit ADC value to voltage (float)
#define adc16_to_voltage(value) (float) ((value * 3.3f)/((float)0xFFFF))

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

// Buffer to store the samples from the 16-bit ADC
// Make adcBuffer twice the size as the number of samples we want for FFT
// Once the first half of the buffer fills, we process the first half while the second half
// starts filling up. And vice-versa.
uint16_t adcBuffer_16b[2 * FFT_SIZE];

// Array to store the converted voltage values from the 16-bit ADC samples
// FFT is of FFT_SIZE but this variable is twice the size to include real and imaginary values
// Where: even index = real, odd index = imaginary
float fftAdc[2 * FFT_SIZE];

// Twiddle factors is FFT_SIZE/2 but is twice the size to include real and imaginary values
// Where: even index = real, odd index = imaginary
float twiddle[FFT_SIZE];

// Flag to signal that the data from ADC is ready for FFT
uint8_t fftADC_ready = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void fft_512samp_init() {

	// Zero out buffer (not needed but can be useful for debugging)
	for (int i = 0; i < 2 * FFT_SIZE; i++) {
		adcBuffer_16b[i] = 0;
		fftAdc[i] = 0;
	}

	// Pre-compute twiddle factors and interleave real and imaginary numbers in same array
	//		even indexes: real , odd indexes: imaginary
	for (uint16_t k = 0; k < FFT_SIZE / 2; k++) {
		float angle = 2.0f * PI * k / FFT_SIZE; // Calculate angle
		twiddle[2 * k] = cosf(angle); // Real
		twiddle[2 * k + 1] = -1 * sinf(angle); // Imaginary
	}

}

// Returns bit reversed index of the value that should go into position x
uint16_t bit_reverse16(uint16_t x, uint8_t bits) {
	x = ((x & 0x5555) << 1) | ((x >> 1) & 0x5555);  // swap odd and even bits
	x = ((x & 0x3333) << 2) | ((x >> 2) & 0x3333);  // swap 2-bit groups
	x = ((x & 0x0F0F) << 4) | ((x >> 4) & 0x0F0F);  // swap nibbles
	x = (x << 8) | (x >> 8);                         // swap bytes
	return x >> (16 - bits); // return adjusted for a bits <= 16
}

// Performs decimation in time FFT of 512 samples "in place" inside fftAdc buffer
// Bit reveral ordering and twiddle factor calculations must be completed beforehand
void fft_dit_512samp() {

	// For each FFT stage
	for (uint16_t stage = 1; stage <= FFT_BITS; stage++) {

		// Butterfly size is 2^stage
		uint16_t m = 1 << stage;
		uint16_t mHalf = m >> 1;

		// At each FFT stage with group size m, you use the twiddle spacing FFT_SIZE/m
		uint16_t stride = FFT_SIZE / m;

		// For each frequency bin
		for (uint16_t k = 0; k < FFT_SIZE; k += m) {
			// For each corresponding butterfy
			for (uint16_t j = 0; j < mHalf; j++) {

				// Determine the index of first value in butterfly
				uint32_t i0 = k + j;

				//Determine the index of second value in butterfly
				uint32_t i1 = i0 + mHalf;

				// Determine index of required twiddle value for this butterfly
				uint32_t t_idx = j * stride;

				// Get real and imaginary parts of twiddle value
				float wr = twiddle[2 * t_idx];
				float wi = twiddle[2 * t_idx + 1];

				// Get real and imaginary parts of first value
				float xr = fftAdc[2 * i0];
				float xi = fftAdc[2 * i0 + 1];

				// Get real and imaginary parts of second value
				float yr = fftAdc[2 * i1];
				float yi = fftAdc[2 * i1 + 1];

				// Complex multiply twiddle values with second value
				float tr = wr * yr - wi * yi;
				float ti = wr * yi + wi * yr;

				// Perform butterfly
				// Operate on first value
				fftAdc[2 * i0] = xr + tr; // Real
				fftAdc[2 * i0 + 1] = xi + ti; // Imaginary
				// Operate on second value
				fftAdc[2 * i1] = xr - tr; // Real
				fftAdc[2 * i1 + 1] = xi - ti; // Imaginary

			}

		}

	}

}

// Function serially prints all frequency bins and data as magnitude
void printFFT_Magnitude_All() {

	// Cycle through fftAdc array where even index is real, odd index is imaginary
	for (uint16_t i = 0; i < 2 * FFT_SIZE; i += 2) {

		// The k-th bin of the FFT
		uint16_t k = i / 2;

		// If k-th bin is positive frequency
		if (k < FFT_SIZE / 2) {

			// Calculate corresponding bin frequency
			float fk = ((float) k * 7500.0f) / (float) FFT_SIZE;

			// Calculate magnitude of bin data
			float magnitude = sqrtf(fftAdc[i] * fftAdc[i] + fftAdc[i + 1] * fftAdc[i + 1]);

			// Print frequency and magnitude to serial
			char buffer[32];
			snprintf(buffer, sizeof(buffer), "%.2f\t%.2f\n", fk, magnitude);
			HAL_UART_Transmit(&huart3, (uint8_t*) buffer, strlen(buffer), HAL_MAX_DELAY);

		} else { // If k-th bin is negative frequency

			// Calculate corresponding bin frequency (as negative value)
			float fk = (((float) k - FFT_SIZE) * 7500.0f) / (float) FFT_SIZE;

			// Calculate magnitude of bin data
			float magnitude = sqrtf(fftAdc[i] * fftAdc[i] + fftAdc[i + 1] * fftAdc[i + 1]);

			// Print frequency and magnitude to serial
			char buffer[32];
			snprintf(buffer, sizeof(buffer), "%.2f\t%.2f\n", fk, magnitude);
			HAL_UART_Transmit(&huart3, (uint8_t*) buffer, strlen(buffer), HAL_MAX_DELAY);
		}
	}

}

void printFFT_Magnitude_MaxPossitive() {

	float max = 0;
	uint16_t pos = 0;

	// Cycle through fftAdc array where even index is real, odd index is imaginary
	for (uint16_t i = 1; i < FFT_SIZE; i += 2) {

		// The k-th bin of the FFT
		uint16_t k = i / 2;

		// Calculate magnitude of bin data
		float magnitude = sqrtf(fftAdc[i] * fftAdc[i] + fftAdc[i + 1] * fftAdc[i + 1]);

		if (magnitude > max){
			max = magnitude;
			pos = k;
		}

	}

	// Calculate corresponding bin frequency
	float fk = ((float) pos * 7500.0f) / (float) FFT_SIZE;

	// Print frequency and magnitude to serial
	char buffer[32];
	snprintf(buffer, sizeof(buffer), "%.2f\t%.2f\n", fk, max);
	HAL_UART_Transmit(&huart3, (uint8_t*) buffer, strlen(buffer), HAL_MAX_DELAY);

}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc) {

	if (hadc->Instance == ADC1) {

		// Normally commented out, but it can be useful to stop DMA for debugging
		// HAL_ADC_Stop_DMA(&hadc1);

		// Define a pointer to point at the begining of the adcBufer array
		uint16_t *p_adcBuffer = &adcBuffer_16b[0];

		// Move data from adcBuffer to new fftAdc buffer for processing:
		// 		Convert ADC values to float voltages
		// 		Bit-reveral re-arrage of data
		for (uint16_t i = 0; i < FFT_SIZE; i++) {
			fftAdc[2 * i] = adc16_to_voltage(
					*(p_adcBuffer + bit_reverse16(i, FFT_BITS))); // Real part
			fftAdc[2 * i + 1] = 0; // Imaginary part
		}

		// Flag that data is ready for FFT
		fftADC_ready = 1;
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {

	if (hadc->Instance == ADC1) {

		// Normally commented out, but it can be useful to stop DMA for debugging
//		 HAL_ADC_Stop_DMA(&hadc1);

		// Define a pointer to point at the middle of the adcBufer array
		uint16_t *p_adcBuffer = &adcBuffer_16b[FFT_SIZE - 1];

		// Move data from adcBuffer to new fftAdc buffer for processing:
		// 		Convert ADC values to float voltages
		// 		Bit-reveral re-arrage of data
		for (uint16_t i = 0; i < FFT_SIZE; i++) {
			fftAdc[2 * i] = adc16_to_voltage(
					*(p_adcBuffer + bit_reverse16(i, FFT_BITS))); // Real part
			fftAdc[2 * i + 1] = 0; // Imaginary part
		}

		// Flag that data is ready for FFT
		fftADC_ready = 1;
	}
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MPU Configuration--------------------------------------------------------*/
	MPU_Config();

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
	MX_USART3_UART_Init();
	/* USER CODE BEGIN 2 */

	// Initialize FFT (mainly calculate twiddle factors)
	fft_512samp_init();

	// Start ADC with DMA
	if (HAL_ADC_Start_DMA(&hadc1, (uint32_t*) adcBuffer_16b, 2 * FFT_SIZE)
			!= HAL_OK) {
		Error_Handler();
	}

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		// If ADC has completed filling buffer + We moved data to fftAdc (which is bit reversed
		// and converted to a float voltage value)
		if (fftADC_ready) {

			// Perform decimation in time FFT
			fft_dit_512samp();

			//Print all the FFT data as magnitude
			printFFT_Magnitude_MaxPossitive();

			// Reset ready flag
			fftADC_ready = 0;
		}

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Supply configuration update enable
	 */
	HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

	/** Configure the main internal regulator output voltage
	 */
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {
	}

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
	RCC_OscInitStruct.HSICalibrationValue = 64;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 4;
	RCC_OscInitStruct.PLL.PLLN = 12;
	RCC_OscInitStruct.PLL.PLLP = 1;
	RCC_OscInitStruct.PLL.PLLQ = 4;
	RCC_OscInitStruct.PLL.PLLR = 2;
	RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
	RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
	RCC_OscInitStruct.PLL.PLLFRACN = 0;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 | RCC_CLOCKTYPE_D3PCLK1
			| RCC_CLOCKTYPE_D1PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
	RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_MultiModeTypeDef multimode = { 0 };
	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */

	/** Common config
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV128;
	hadc1.Init.Resolution = ADC_RESOLUTION_16B;
	hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
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
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler();
	}

	/** Configure the ADC multi-mode
	 */
	multimode.Mode = ADC_MODE_INDEPENDENT;
	if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_15;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_16CYCLES_5;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	sConfig.OffsetSignedSaturation = DISABLE;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART3_UART_Init(void) {

	/* USER CODE BEGIN USART3_Init 0 */

	/* USER CODE END USART3_Init 0 */

	/* USER CODE BEGIN USART3_Init 1 */

	/* USER CODE END USART3_Init 1 */
	huart3.Instance = USART3;
	huart3.Init.BaudRate = 115200;
	huart3.Init.WordLength = UART_WORDLENGTH_8B;
	huart3.Init.StopBits = UART_STOPBITS_1;
	huart3.Init.Parity = UART_PARITY_NONE;
	huart3.Init.Mode = UART_MODE_TX_RX;
	huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart3.Init.OverSampling = UART_OVERSAMPLING_16;
	huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
	huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart3) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART3_Init 2 */

	/* USER CODE END USART3_Init 2 */

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Stream0_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	/* USER CODE BEGIN MX_GPIO_Init_1 */

	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();

	/* USER CODE BEGIN MX_GPIO_Init_2 */

	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* MPU Configuration */

void MPU_Config(void) {
	MPU_Region_InitTypeDef MPU_InitStruct = { 0 };

	/* Disables the MPU */
	HAL_MPU_Disable();

	/** Initializes and configures the Region and the memory to be protected
	 */
	MPU_InitStruct.Enable = MPU_REGION_ENABLE;
	MPU_InitStruct.Number = MPU_REGION_NUMBER0;
	MPU_InitStruct.BaseAddress = 0x0;
	MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
	MPU_InitStruct.SubRegionDisable = 0x87;
	MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
	MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
	MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
	MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
	MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
	MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

	HAL_MPU_ConfigRegion(&MPU_InitStruct);
	/* Enables the MPU */
	HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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
