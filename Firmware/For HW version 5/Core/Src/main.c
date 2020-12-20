/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under Ultimate Liberty license
 * SLA0044, the "License"; You may not use this file except in compliance with
 * the License. You may obtain a copy of the License at:
 *                             www.st.com/SLA0044
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "stdbool.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define ADS1115_ADDRESS 0x48 // I2C addresses of ADC and DAC
#define MCP4725_ADDRESS 0x61
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
DAC_HandleTypeDef hdac;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
uint8_t UserRxBuffer[2048];
bool usbMessageReceived = false;

float shunt_resistance = 10; // Resistance of shunt resistor in ohms
float i_amp_gain = 3; // Gain setting of current-sense amplifier
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DAC_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint16_t adc_read_differential(uint8_t channel)
{
	uint8_t conversion_reg = 0x00;
	uint8_t config_reg = 0x01;
	uint8_t lothresh_reg = 0x02;
	uint8_t hithresh_reg = 0x03;

	unsigned char ADSwrite[6];
	int16_t reading;

	switch (channel)
	{
	case 0:
		ADSwrite[1] = 0b10000011;
		break;
	case 2:
		ADSwrite[1] = 0b10110011;
		break;
	}

	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

	ADSwrite[0] = config_reg; // Point to config register
	ADSwrite[2] = 0b10001000; // For Alert pin, set COMP_QUE to anything but 11. Active low

	HAL_I2C_Master_Transmit(&hi2c1, ADS1115_ADDRESS << 1, ADSwrite, 3, 100);

	while (HAL_GPIO_ReadPin(ADC_Alert_GPIO_Port, ADC_Alert_Pin) == 0)
	{
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
	}
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

	//HAL_Delay(200);

	ADSwrite[0] = conversion_reg; // Now point to conversion register

	HAL_I2C_Master_Transmit(&hi2c1, ADS1115_ADDRESS << 1, ADSwrite, 1, 100);

	HAL_I2C_Master_Receive(&hi2c1, ADS1115_ADDRESS << 1, ADSwrite, 2, 100);
	reading = (ADSwrite[0] << 8 | ADSwrite[1]);

	return reading;
}

void adc_init()
{
	uint8_t conversion_reg = 0x00;
	uint8_t config_reg = 0x01;
	uint8_t lothresh_reg = 0x02;
	uint8_t hithresh_reg = 0x03;

	unsigned char ADSwrite[6];

	ADSwrite[0] = hithresh_reg;
	ADSwrite[1] = 0x80;
	ADSwrite[2] = 0x00;
	HAL_I2C_Master_Transmit(&hi2c1, ADS1115_ADDRESS << 1, ADSwrite, 3, 100);

	ADSwrite[0] = lothresh_reg;
	ADSwrite[1] = 0x7F;
	ADSwrite[2] = 0xFF;
	HAL_I2C_Master_Transmit(&hi2c1, ADS1115_ADDRESS << 1, ADSwrite, 3, 100);
}

uint16_t adc_read_singleended(uint8_t channel)
{
	uint8_t conversion_reg = 0x00;
	uint8_t config_reg = 0x01;
	uint8_t lothresh_reg = 0x02;
	uint8_t hithresh_reg = 0x03;

	unsigned char ADSwrite[6];
	int16_t reading;
	float voltage;
	const float voltageConv = 4.096 / 32768.0;

	switch (channel)
	{
	case 0:
		ADSwrite[1] = 0xC3;
		break;
	case 1:
		ADSwrite[1] = 0xD3;
		break;
	case 2:
		ADSwrite[1] = 0xE3;
		break;
	case 3:
		ADSwrite[1] = 0xF3;
		break;
	}
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

	ADSwrite[0] = config_reg; // Point to config register
	//ADSwrite[2] = 0x03; // Working well
	ADSwrite[2] = 0b10001000; // For Alert pin, set COMP_QUE to anything but 11. Active low

	HAL_I2C_Master_Transmit(&hi2c1, ADS1115_ADDRESS << 1, ADSwrite, 3, 100);

	while (HAL_GPIO_ReadPin(ADC_Alert_GPIO_Port, ADC_Alert_Pin) == 0)
	{
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
	}
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

	//HAL_Delay(200);

	ADSwrite[0] = conversion_reg; // Now point to conversion register

	HAL_I2C_Master_Transmit(&hi2c1, ADS1115_ADDRESS << 1, ADSwrite, 1, 100);

	HAL_I2C_Master_Receive(&hi2c1, ADS1115_ADDRESS << 1, ADSwrite, 2, 100);
	reading = (ADSwrite[0] << 8 | ADSwrite[1]);
	if (reading < 0)
	{
		reading = 0;
	}
	//voltage = reading * voltageConv;
	return reading;
}

void set_current_limit(float current_ma)
{
	if (current_ma > 40)
	{
		float current_ma = 40;
	}

	uint16_t dac_level = (int) ((current_ma / 40) * 4095);

	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, dac_level);
	HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
}

void amp_dac_write(uint16_t level)
{
	unsigned char DACbuffer[3];

	DACbuffer[0] = 0x40; //Set write mode
	DACbuffer[1] = level >> 4;
	DACbuffer[2] = level << 4;

	HAL_I2C_Master_Transmit(&hi2c1, MCP4725_ADDRESS << 1, DACbuffer, 3, 100);
}

void dac_write_voltage(float voltage)
{
	uint16_t level = (int) (100.41 * voltage) + 1835;
	amp_dac_write(level);
}

void adc(uint8_t channel)
{
	uint16_t adc = adc_read_differential(channel);
	char adcmsg[16];
	memset(adcmsg, 0, sizeof(adcmsg));
	sprintf(adcmsg, "%d", adc);
	CDC_Transmit_FS(adcmsg, sizeof(adcmsg));
}

void single_shot()
{
	float voltage_running = 0;
	float current_running = 0;

	int num_oversamples = 25;

	for (int i = 0; i < num_oversamples; i++)
	{
		uint16_t v_measure = adc_read_singleended(0);
		int16_t i_measure = adc_read_differential(2);

		float voltage = (v_measure * 0.000625) - 8.190538;
		float current = (i_measure * 0.125 / 1000) / shunt_resistance
				/ i_amp_gain;

		current = current + ((3.44E-5 * voltage) - 7.35E-5);

		current_running += current;
		voltage_running += voltage;
	}

	current_running /= num_oversamples;
	voltage_running /= num_oversamples;

	char vmsg[32];
	char imsg[32];

	memset(vmsg, 0, sizeof(vmsg));
	memset(imsg, 0, sizeof(imsg));

	sprintf(vmsg, "%4.3f", voltage_running);
	sprintf(imsg, "%e", current_running);

	strcat(vmsg, ",");
	strcat(vmsg, imsg);

	strcat(vmsg, "\n");
	//RGB_Led_SetIntensity(1, 0, 0, 0);

	CDC_Transmit_FS(vmsg, sizeof(vmsg));

}

/*void RGB_LED_Init()
 {
 HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
 HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
 HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);

 HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
 HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
 HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
 }*/

/*void RGB_Led_SetIntensity(uint8_t channel, uint8_t red, uint8_t green, uint8_t blue)
 {
 switch (channel)
 {
 case 1:
 htim2.Instance->CCR1 = 100 - red;
 htim2.Instance->CCR2 = 100 - green;
 htim2.Instance->CCR3 = 100 - blue;
 break;
 case 2:
 htim3.Instance->CCR1 = 100 - red;
 htim3.Instance->CCR2 = 100 - green;
 htim3.Instance->CCR3 = 100 - blue;
 break;
 }
 }*/
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
  MX_DAC_Init();
  MX_I2C1_Init();
  MX_USB_DEVICE_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
	adc_init();
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

	//RGB_LED_Init();
	//RGB_Led_SetIntensity(1, 0, 0, 0);
	//RGB_Led_SetIntensity(2, 0, 0, 0);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{

		if (usbMessageReceived == true)
		{
			//RGB_Led_SetIntensity(2, 0, 0, 100);

			char chRxBuffer[2048];

			// Take the uint8_t buffer and turn it into a char array
			for (int i = 0; i <= 2048; i++)
			{
				chRxBuffer[i] = UserRxBuffer[i];
			}

			usbMessageReceived = false;

			char manualDACSearch[] = "DAC";
			char adcSearch[] = "ADC";

			char ampEnableSearch[] = "CH1:ENA";
			char ampDisableSearch[] = "CH1:DIS";
			char setVoltageSearch[] = "CH1:VOL";
			char setMeasVoltageSearch[] = "CH1:MEA:VOL";
			char setCurrLimitSearch[] = "CH1:CUR";

			char *manualDACPtr = strstr(chRxBuffer, manualDACSearch);
			char *manualADCPtr = strstr(chRxBuffer, adcSearch);

			char *ampEnablePtr = strstr(chRxBuffer, ampEnableSearch);
			char *ampDisablePtr = strstr(chRxBuffer, ampDisableSearch);
			char *setVoltagePtr = strstr(chRxBuffer, setVoltageSearch);
			char *setMeasVoltagePtr = strstr(chRxBuffer, setMeasVoltageSearch);
			char *setCurrLimitPtr = strstr(chRxBuffer, setCurrLimitSearch);

			if (ampEnablePtr != NULL)
			{
				HAL_GPIO_WritePin(GPIOA, AMP_EN_Pin, 1);
			}

			else if (manualDACPtr != NULL)
			{
				strtok(chRxBuffer, " ");
				int requested_level = atoi(strtok(NULL, " "));
				amp_dac_write(requested_level);
			}

			else if (manualADCPtr != NULL)
			{
				strtok(chRxBuffer, " ");
				int requested_adc_channel = atoi(strtok(NULL, " "));
				adc(requested_adc_channel);
			}

			else if (ampDisablePtr != NULL)
			{
				HAL_GPIO_WritePin(GPIOA, AMP_EN_Pin, 0);
			}
			else if (setVoltagePtr != NULL)
			{
				strtok(chRxBuffer, " ");
				float requested_voltage = atof(strtok(NULL, " "));
				dac_write_voltage(requested_voltage);
			}
			else if (setMeasVoltagePtr != NULL)
			{
				strtok(chRxBuffer, " ");
				float requested_voltage = atof(strtok(NULL, " "));
				dac_write_voltage(requested_voltage);
				HAL_Delay(1); // DAC and V buffer take ~70us to stabilise
				single_shot();
			}

			else if (setCurrLimitPtr != NULL)
			{
				strtok(chRxBuffer, " ");
				float requested_current_limit = atof(strtok(NULL, " "));
				set_current_limit(requested_current_limit);
			}

			else
			{
				char msg[1024] = "Test string not found";
				strcat(msg, "\n");
				CDC_Transmit_FS(msg, sizeof(msg));
			}
		}
		//RGB_Led_SetIntensity(2, 0, 0, 0);

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL3;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */
  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x2000090E;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 4800-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 100-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 4800-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 100-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(AMP_EN_GPIO_Port, AMP_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : AMP_EN_Pin */
  GPIO_InitStruct.Pin = AMP_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(AMP_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SHUNT_RANGE_Pin */
  GPIO_InitStruct.Pin = SHUNT_RANGE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SHUNT_RANGE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ADC_Alert_Pin */
  GPIO_InitStruct.Pin = ADC_Alert_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(ADC_Alert_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
