/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes
 * ------------------------------------------------------------------*/
#include "main.h"

/* Private includes
 * ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define ARM_MATH_CM4
// #include "arm_math.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "arm_const_structs.h"
/* USER CODE END Includes */

/* Private typedef
 * -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define
 * ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FFT_BUFFER_SIZE 2048
#define SAMPLE_RATE 2000.0f // Hz
#define DECIMATION 16
#define FIR_LEN 21
#define BLOCK_SIZE 64
#define A4 440.0f // Hz
#define NOTE_RATIO 1.05946309436f
/* USER CODE END PD */

/* Private macro
 * -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables
 * ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

arm_rfft_fast_instance_f32 fftHandler;
arm_cfft_instance_f32 zoomFftHandler;
uint16_t inputArr[2 * FFT_BUFFER_SIZE];

float32_t firArr[FIR_LEN] = {
    -0.0026012824532543164, -0.0033000391283683167, -0.004010081650667714,
    -0.0019592148837829122, 0.006662701906228264,   0.024831690243449004,
    0.052737841736489564,   0.0867532118751942,     0.11997490001564744,
    0.1442961800416668,     0.153228184594796,      0.1442961800416668,
    0.11997490001564744,    0.0867532118751942,     0.052737841736489564,
    0.024831690243449004,   0.006662701906228264,   -0.0019592148837829122,
    -0.004010081650667714,  -0.0033000391283683167, -0.0026012824532543164};
arm_fir_instance_f32 firHandler;
float32_t stateArr[FIR_LEN + BLOCK_SIZE - 1];

/* USER CODE END PV */

/* Private function prototypes
 * -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code
 * ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// used to set a delay that is more accurate than HAL_Delay
// the max delay is 819.2us using 80MHz clock
void delay(float32_t us) {
    uint16_t ticks = (uint16_t)(roundf(us * 80));
    uint16_t start_time = __HAL_TIM_GET_COUNTER(&htim1);
    while (__HAL_TIM_GET_COUNTER(&htim1) - start_time < ticks)
        ;
}

// prints string to UART pin
void print(char msg[]) {
    HAL_UART_Transmit(&huart2, msg, strlen(msg), HAL_MAX_DELAY);
}

/*
 * This function gets the general frequency of a signal
 * but keeps the input array untouched.
 */
float32_t get_rfft_freq(uint16_t *arr) {
    float32_t fftBufOut[FFT_BUFFER_SIZE];
    float32_t fftBufIn[FFT_BUFFER_SIZE];

    for (size_t i = 0; i < FFT_BUFFER_SIZE; ++i) {
        fftBufIn[i] = (float32_t)arr[i] - 2048; // set range to -2048 to +2047
    }

    arm_rfft_fast_f32(&fftHandler, fftBufIn, fftBufOut, 0);

    size_t largest_index = 0;
    float32_t largest_val = 0;

    // We'll ignore 0Hz and fs/2 for now which are stored
    // in fftBufOut[0] and fftBufOut[1] respectively
    for (size_t i = 1; i < FFT_BUFFER_SIZE / 2; ++i) {
        float32_t cur_val = fftBufOut[2 * i] * fftBufOut[2 * i] +
                            fftBufOut[2 * i + 1] * fftBufOut[2 * i + 1];
        if (cur_val > largest_val) {
            largest_index = i;
            largest_val = cur_val;
        }
    }

    return (float32_t)largest_index * SAMPLE_RATE / FFT_BUFFER_SIZE;
}

/*
 * translates `arr` in the frequency domain so `f_0` is
 * the new DC frequency (0Hz). Down samples by `DECIMATION`, and
 * filters the new array with a low pass filter with a cutofff
 * frequency of 150Hz
 */
void create_zoom_arr(float32_t *outArr, uint16_t *inArr, float32_t f_0) {
    float32_t input[FFT_BUFFER_SIZE] = {0};
    float32_t dest[FFT_BUFFER_SIZE] = {0};
    memset(outArr, 0, 2 * FFT_BUFFER_SIZE * sizeof(float32_t));

    // Real part
    // Shift in frequency domain
    for (size_t i = 0; i < FFT_BUFFER_SIZE; ++i) {
        input[i] = ((float32_t)inArr[i] - 2048) *
                   arm_cos_f32((float32_t)(2 * PI * f_0 * i / SAMPLE_RATE));
    }

    // Apply FIR low-pass filter
    for (size_t i = 0; i < FFT_BUFFER_SIZE; i += BLOCK_SIZE) {
        arm_fir_f32(&firHandler, &(input[i]), &(dest[i]), BLOCK_SIZE);
    }

    // Down sample by `DECIMATION/4`
    for (size_t i = 0; i < FFT_BUFFER_SIZE / DECIMATION * 4; ++i) {
        dest[i] = dest[i * DECIMATION / 4];
    }
    memset(&(dest[FFT_BUFFER_SIZE / DECIMATION * 4]), 0,
           (FFT_BUFFER_SIZE - FFT_BUFFER_SIZE / DECIMATION * 4) *
               sizeof(float32_t));

    // Apply FIR low-pass filter
    for (size_t i = 0; i < FFT_BUFFER_SIZE; i += BLOCK_SIZE) {
        arm_fir_f32(&firHandler, &(dest[i]), &(input[i]), BLOCK_SIZE);
    }

    // Save real part while decimating by another factor of `DECIMATION/4`
    for (size_t i = 0; i < FFT_BUFFER_SIZE / DECIMATION; ++i) {
        outArr[2 * i] = input[i * DECIMATION / 4];
    }

    // Imaginary part
    // Shift in frequency domain
    for (size_t i = 0; i < FFT_BUFFER_SIZE; ++i) {
        input[i] = -((float32_t)inArr[i] - 2048) *
                   arm_sin_f32((float32_t)(2 * PI * f_0 * i / SAMPLE_RATE));
    }

    // Apply FIR low-pass filter
    for (size_t i = 0; i < FFT_BUFFER_SIZE; i += BLOCK_SIZE) {
        arm_fir_f32(&firHandler, &(input[i]), &(dest[i]), BLOCK_SIZE);
    }

    // Down sample by `DECIMATION/4`
    for (size_t i = 0; i < FFT_BUFFER_SIZE / DECIMATION * 4; ++i) {
        dest[i] = dest[i * DECIMATION / 4];
    }
    memset(&(dest[FFT_BUFFER_SIZE / DECIMATION * 4]), 0,
           (FFT_BUFFER_SIZE - FFT_BUFFER_SIZE / DECIMATION * 4) *
               sizeof(float32_t));

    // Apply FIR low-pass filter
    for (size_t i = 0; i < FFT_BUFFER_SIZE; i += BLOCK_SIZE) {
        arm_fir_f32(&firHandler, &(dest[i]), &(input[i]), BLOCK_SIZE);
    }

    // Save imag part while decimating by another factor of `DECIMATION/4`
    for (size_t i = 0; i < FFT_BUFFER_SIZE / DECIMATION; ++i) {
        outArr[2 * i + 1] = input[i * DECIMATION / 4];
    }
}

float32_t get_zoom_freq(float32_t *zoomArr, float32_t f_0) {
    arm_cfft_f32(&zoomFftHandler, zoomArr, 0, 1);

    size_t largest_index = 0;
    float32_t largest_val = 0;

    for (size_t i = 0; i < FFT_BUFFER_SIZE; ++i) {
        float32_t cur_val = zoomArr[2 * i] * zoomArr[2 * i] +
                            zoomArr[2 * i + 1] * zoomArr[2 * i + 1];
        if (cur_val > largest_val) {
            largest_index = i;
            largest_val = cur_val;
        }
    }

    float32_t zoom_freq;
    if (largest_index <= FFT_BUFFER_SIZE / 2) {
        zoom_freq = (float32_t)largest_index * SAMPLE_RATE / FFT_BUFFER_SIZE /
                        DECIMATION +
                    f_0;
    } else {
        zoom_freq = ((float32_t)largest_index - FFT_BUFFER_SIZE) * SAMPLE_RATE /
                        FFT_BUFFER_SIZE / DECIMATION +
                    f_0;
    }

    return 0.987880793f * zoom_freq -
           0.00077004f; // Little term adjustment using LINEST correction

    /*
    Without term correction the below table shows the value discrepancy

    Actual	Calc
    40	    40.47
    50	    50.66
    60	    60.73
    70	    70.86
    80	    80.99
    90	    91.06
    100	    101.26
    150	    151.86
    200	    202.45
    250	    252.99
    300	    303.71
    400	    404.85
    500	    506.29
    600	    607.3
    650	    657.96

    Actual = (0.987880793 +/- 7.3131E-05) * Calc - 0.00077 +/- 0.022973443
    */
}

/*
 * Given a frequency `freq` the function will find the closest
 * note and return the note as well as the cents off in `buf`.
 *
 * Ensure that `buf` is able to store the value.
 */
void get_note(float32_t freq, char *buf) {
    float32_t noteExp = 12.0f * log2f(freq / A4);
    int16_t closestNote = (int16_t)(roundf(noteExp));
    float32_t cents = (noteExp - (float32_t)closestNote) * 100.0f;

    switch (closestNote % 12) {
    case 0:
        sprintf(buf, "A + %f", cents);
        break;
    case -11:
    case 1:
        sprintf(buf, "A# + %f", cents);
        break;
    case -10:
    case 2:
        sprintf(buf, "B + %f", cents);
        break;
    case -9:
    case 3:
        sprintf(buf, "C + %f", cents);
        break;
    case -8:
    case 4:
        sprintf(buf, "C# + %f", cents);
        break;
    case -7:
    case 5:
        sprintf(buf, "D + %f", cents);
        break;
    case -6:
    case 6:
        sprintf(buf, "D# + %f", cents);
        break;
    case -5:
    case 7:
        sprintf(buf, "E + %f", cents);
        break;
    case -4:
    case 8:
        sprintf(buf, "F + %f", cents);
        break;
    case -3:
    case 9:
        sprintf(buf, "F# + %f", cents);
        break;
    case -2:
    case 10:
        sprintf(buf, "G + %f", cents);
        break;
    case -1:
    case 11:
        sprintf(buf, "G# + %f", cents);
        break;
    default:
        sprintf(buf, "shouldnt be here");
        break;
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

    /* MCU
     * Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the
     * Systick.
     */
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
    MX_USART2_UART_Init();
    MX_TIM1_Init();
    /* USER CODE BEGIN 2 */

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    print("starting ADC reading...\r\n");
    HAL_TIM_Base_Start(&htim1);

    arm_rfft_fast_init_f32(&fftHandler, FFT_BUFFER_SIZE);
    zoomFftHandler = arm_cfft_sR_f32_len2048;
    arm_fir_init_f32(&firHandler, FIR_LEN, firArr, stateArr, BLOCK_SIZE);

    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);

    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)inputArr, 2 * FFT_BUFFER_SIZE);
    while (1) {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
        print("Reading in data...\r\n");
        HAL_Delay(1000);
    }
    /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Configure the main internal regulator output voltage
     */
    if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) !=
        HAL_OK) {
        Error_Handler();
    }

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
    RCC_OscInitStruct.MSIState = RCC_MSI_ON;
    RCC_OscInitStruct.MSICalibrationValue = 0;
    RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
    RCC_OscInitStruct.PLL.PLLM = 1;
    RCC_OscInitStruct.PLL.PLLN = 40;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
    RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
    RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
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

    ADC_ChannelConfTypeDef sConfig = {0};

    /* USER CODE BEGIN ADC1_Init 1 */

    /* USER CODE END ADC1_Init 1 */

    /** Common config
     */
    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV1;
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
    hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    hadc1.Init.LowPowerAutoWait = DISABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.NbrOfConversion = 1;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T1_TRGO;
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISINGFALLING;
    hadc1.Init.DMAContinuousRequests = ENABLE;
    hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
    hadc1.Init.OversamplingMode = DISABLE;
    if (HAL_ADC_Init(&hadc1) != HAL_OK) {
        Error_Handler();
    }

    /** Configure Regular Channel
     */
    sConfig.Channel = ADC_CHANNEL_15;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset = 0;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN ADC1_Init 2 */

    /* USER CODE END ADC1_Init 2 */
}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void) {
    /* USER CODE BEGIN TIM1_Init 0 */

    /* USER CODE END TIM1_Init 0 */

    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    /* USER CODE BEGIN TIM1_Init 1 */

    /* USER CODE END TIM1_Init 1 */
    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 1 - 1;
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1.Init.Period = 40000 - 1;
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
    sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) !=
        HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM1_Init 2 */

    /* USER CODE END TIM1_Init 2 */
}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {
    /* USER CODE BEGIN USART2_Init 0 */

    /* USER CODE END USART2_Init 0 */

    /* USER CODE BEGIN USART2_Init 1 */

    /* USER CODE END USART2_Init 1 */
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_CTS;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&huart2) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN USART2_Init 2 */

    /* USER CODE END USART2_Init 2 */
}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {
    /* DMA controller clock enable */
    __HAL_RCC_DMA1_CLK_ENABLE();

    /* DMA interrupt init */
    /* DMA1_Channel1_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
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
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* USER CODE BEGIN MX_GPIO_Init_2 */
    /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc) {
    float32_t f_0 = get_rfft_freq(inputArr);
    float32_t zoomArr[FFT_BUFFER_SIZE * 2] = {0};

    create_zoom_arr(zoomArr, inputArr, f_0);

    float32_t zoomFreq = get_zoom_freq(zoomArr, f_0);

    char output[32];
    sprintf(output, "0.5) %f\r\n", f_0);
    print(output);

    sprintf(output, "0.5) %f\r\n", zoomFreq);
    print(output);

    get_note(zoomFreq, output);
    print(output);
    print("\r\n");
}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
    float32_t f_0 = get_rfft_freq(&(inputArr[FFT_BUFFER_SIZE]));
    float32_t zoomArr[FFT_BUFFER_SIZE * 2] = {0};

    create_zoom_arr(zoomArr, &(inputArr[FFT_BUFFER_SIZE]), f_0);

    float32_t zoomFreq = get_zoom_freq(zoomArr, f_0);

    char output[32];
    sprintf(output, "1) %f\r\n", f_0);
    print(output);

    sprintf(output, "1) %f\r\n", zoomFreq);
    print(output);

    get_note(zoomFreq, output);
    print(output);
    print("\r\n");
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state
     */
    __disable_irq();
    while (1) {
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
void assert_failed(uint8_t *file, uint32_t line) {
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line
           number, ex: printf("Wrong parameters value: file %s on line %d\r\n",
       file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
