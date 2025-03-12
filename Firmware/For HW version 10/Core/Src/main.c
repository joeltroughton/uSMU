/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "stdlib.h"

#include "ee.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define ADS1115_ADDRESS 0x48 // I2C addresses of ADC and DAC
#define DAC8571_ADDRESS 0x4C
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

/* USER CODE BEGIN PV */
uint8_t UserRxBuffer[128];
volatile bool usbMessageReceived = false;

bool calibration_mode = false;
bool auto_gain_active = true;

float shunt_resistance = 49.9; // Resistance of shunt resistor in ohms

float i_amp_gain = 1.375; // Gain setting of current-sense amplifier

uint16_t num_oversamples = 25;

float x_ilim = 0;
float y_ilim = 0;

float dac_xadj = 0;
float dac_yadj = 0;

float zerocurrent_x = 0;
float zerocurrent_y = 0;

// 1.375x gain
float zerocurrent_x_1p375 = 0;
float zerocurrent_y_1p375 = 0;

// 8x gain
float zerocurrent_x_8 = 0;
float zerocurrent_y_8 = 0;

// 64x gain
float zerocurrent_x_64 = 0;
float zerocurrent_y_64 = 0;

// 176x gain
float zerocurrent_x_176 = 0;
float zerocurrent_y_176 = 0;

float v_xadj = 0;
float v_yadj = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DAC_Init(void);
static void MX_I2C1_Init(void);
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

	//HAL_GPIO_WritePin(GPIOA, LEDR_Pin, GPIO_PIN_SET);

	ADSwrite[0] = config_reg; // Point to config register
	ADSwrite[2] = 0b10001000; // 128SPS. For Alert pin, set COMP_QUE to anything but 11. Active low

	HAL_I2C_Master_Transmit(&hi2c1, ADS1115_ADDRESS << 1, ADSwrite, 3, 100);

	while (HAL_GPIO_ReadPin(ADC_Alert_GPIO_Port, ADC_Alert_Pin) == 0)
	{
		//HAL_GPIO_WritePin(GPIOA, LEDR_Pin, GPIO_PIN_RESET);
	}
	//HAL_GPIO_WritePin(GPIOA, LEDR_Pin, GPIO_PIN_SET);

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

	unsigned char ADSwrite[6];
	int16_t reading;

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

	ADSwrite[0] = config_reg; // Point to config register
	ADSwrite[2] = 0b10001000; // 128SPS. For Alert pin, set COMP_QUE to anything but 11. Active low

	HAL_I2C_Master_Transmit(&hi2c1, ADS1115_ADDRESS << 1, ADSwrite, 3, 100);

	while (HAL_GPIO_ReadPin(ADC_Alert_GPIO_Port, ADC_Alert_Pin) == 0)
	{
		//HAL_GPIO_WritePin(GPIOA, LEDR_Pin, GPIO_PIN_RESET);
	}
	//HAL_GPIO_WritePin(GPIOA, LEDR_Pin, GPIO_PIN_SET);

	ADSwrite[0] = conversion_reg; // Now point to conversion register

	HAL_I2C_Master_Transmit(&hi2c1, ADS1115_ADDRESS << 1, ADSwrite, 1, 100);

	HAL_I2C_Master_Receive(&hi2c1, ADS1115_ADDRESS << 1, ADSwrite, 2, 100);
	reading = (ADSwrite[0] << 8 | ADSwrite[1]);
	if (reading < 0)
	{
		reading = 0;
	}
	return reading;
}

void set_current_limit(float current_ma)
{
	if (current_ma > 40)
	{
		float current_ma = 40;
	}
	float dac_level = (current_ma * x_ilim) + y_ilim;

	uint16_t dac_level_int = (int) dac_level;

	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, dac_level_int);
	HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
}

void DAC8571_amp_dac_write(uint16_t level)
{
	unsigned char DACbuffer[3];

	DACbuffer[0] = 0b00010000; //Set write mode
	DACbuffer[1] = level >> 8;
	DACbuffer[2] = level;

	HAL_I2C_Master_Transmit(&hi2c1, DAC8571_ADDRESS << 1, DACbuffer, 3, 100);
}

void dac_write_voltage(float voltage)
{
	float flevel = (dac_xadj * voltage) + dac_yadj;
	uint16_t level = round(flevel);
	DAC8571_amp_dac_write(level);
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

	for (int i = 0; i < num_oversamples; i++)
	{
		uint16_t v_measure = adc_read_singleended(0);

		int16_t i_measure = adc_read_differential(2);
		float voltage = 0;
		if (calibration_mode == false)
		{
			voltage = (v_measure * v_xadj) + v_yadj;
		}
		else
		{
			voltage = v_measure;
		}

		float current = (i_measure * 0.125 / 1000) / shunt_resistance
				/ i_amp_gain;
		current = current + ((zerocurrent_x * voltage) + zerocurrent_y);
		float current_abs = fabs(current);

		// Set programmable gain amplifier based on the last measured current
		if (auto_gain_active == true)
		{
			if (current_abs > 1E-2)
			{
				// Purple
				set_pga_gain_1p375();
			}
			if (current_abs > 1E-3 && current_abs <= 1E-2)
			{
				// Red
				set_pga_gain_8();
			}
			if (current_abs > 1E-4 && current_abs <= 1E-3)
			{
				// Green
				set_pga_gain_64();
			}
			if (current_abs < 1E-5)
			{
				// Blue
				set_pga_gain_176();
			}
		}

		voltage_running += voltage;
		current_running += current;
	}

	voltage_running /= num_oversamples;
	current_running /= num_oversamples;

	char vmsg[32];
	char imsg[32];

	memset(vmsg, 0, sizeof(vmsg));
	memset(imsg, 0, sizeof(imsg));

	sprintf(vmsg, "%4.3f", voltage_running);
	sprintf(imsg, "%e", current_running);

	strcat(vmsg, ",");
	strcat(vmsg, imsg);

	strcat(vmsg, "\n");
	CDC_Transmit_FS(vmsg, sizeof(vmsg));

}

void set_pga_gain_1p375()
{
	HAL_GPIO_WritePin(GPIOA, LEDR_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, LEDG_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, LEDB_Pin, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(GPIOB, I_G0_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, I_G1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, I_G2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, I_G3_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, I_G4_Pin, GPIO_PIN_SET);
	// 01001

	i_amp_gain = 1.375;
	if (calibration_mode == false)
	{
		zerocurrent_x = zerocurrent_x_1p375;
		zerocurrent_y = zerocurrent_y_1p375;
	}
}

void set_pga_gain_8()
{
	HAL_GPIO_WritePin(GPIOA, LEDR_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, LEDG_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, LEDB_Pin, GPIO_PIN_SET);

	HAL_GPIO_WritePin(GPIOB, I_G0_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, I_G1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, I_G2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, I_G3_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, I_G4_Pin, GPIO_PIN_RESET);

	i_amp_gain = 8;
	if (calibration_mode == false)
	{
		zerocurrent_x = zerocurrent_x_8;
		zerocurrent_y = zerocurrent_y_8;
	}

}

void set_pga_gain_64()
{
	HAL_GPIO_WritePin(GPIOA, LEDR_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, LEDG_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, LEDB_Pin, GPIO_PIN_SET);

	HAL_GPIO_WritePin(GPIOB, I_G0_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, I_G1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, I_G2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, I_G3_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, I_G4_Pin, GPIO_PIN_RESET);

	i_amp_gain = 64;
	if (calibration_mode == false)
	{
		zerocurrent_x = zerocurrent_x_64;
		zerocurrent_y = zerocurrent_y_64;
	}
}

void set_pga_gain_176()
{
	HAL_GPIO_WritePin(GPIOA, LEDR_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, LEDG_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, LEDB_Pin, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(GPIOB, I_G0_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, I_G1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, I_G2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, I_G3_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, I_G4_Pin, GPIO_PIN_SET);

	i_amp_gain = 176;
	if (calibration_mode == false)
	{
		zerocurrent_x = zerocurrent_x_176;
		zerocurrent_y = zerocurrent_y_176;
	}
}

void read_eeprom()
{
	// Reads calibration data from emulated EEPROM

	uint8_t dataBuffer[4];

	memset(dataBuffer, 0, sizeof(dataBuffer));
	ee_read(0, 4, dataBuffer);
	memcpy(&dac_xadj, dataBuffer, sizeof dac_xadj);

	memset(dataBuffer, 0, sizeof(dataBuffer));
	ee_read(4, 4, dataBuffer);
	memcpy(&dac_yadj, dataBuffer, sizeof dac_yadj);

	memset(dataBuffer, 0, sizeof(dataBuffer));
	ee_read(8, 4, dataBuffer);
	memcpy(&v_xadj, dataBuffer, sizeof v_xadj);

	memset(dataBuffer, 0, sizeof(dataBuffer));
	ee_read(12, 4, dataBuffer);
	memcpy(&v_yadj, dataBuffer, sizeof v_yadj);

	memset(dataBuffer, 0, sizeof(dataBuffer));
	ee_read(16, 4, dataBuffer);
	memcpy(&zerocurrent_x_8, dataBuffer, sizeof zerocurrent_x_8);

	memset(dataBuffer, 0, sizeof(dataBuffer));
	ee_read(20, 4, dataBuffer);
	memcpy(&zerocurrent_y_8, dataBuffer, sizeof zerocurrent_y_8);

	memset(dataBuffer, 0, sizeof(dataBuffer));
	ee_read(24, 4, dataBuffer);
	memcpy(&zerocurrent_x_64, dataBuffer, sizeof zerocurrent_x_64);

	memset(dataBuffer, 0, sizeof(dataBuffer));
	ee_read(28, 4, dataBuffer);
	memcpy(&zerocurrent_y_64, dataBuffer, sizeof zerocurrent_y_64);

	memset(dataBuffer, 0, sizeof(dataBuffer));
	ee_read(32, 4, dataBuffer);
	memcpy(&zerocurrent_x_176, dataBuffer, sizeof zerocurrent_x_176);

	memset(dataBuffer, 0, sizeof(dataBuffer));
	ee_read(36, 4, dataBuffer);
	memcpy(&zerocurrent_y_176, dataBuffer, sizeof zerocurrent_y_176);

	memset(dataBuffer, 0, sizeof(dataBuffer));
	ee_read(40, 4, dataBuffer);
	memcpy(&x_ilim, dataBuffer, sizeof x_ilim);

	memset(dataBuffer, 0, sizeof(dataBuffer));
	ee_read(44, 4, dataBuffer);
	memcpy(&y_ilim, dataBuffer, sizeof y_ilim);

	memset(dataBuffer, 0, sizeof(dataBuffer));
	ee_read(48, 4, dataBuffer);
	memcpy(&zerocurrent_x_1p375, dataBuffer, sizeof zerocurrent_x_1p375);

	memset(dataBuffer, 0, sizeof(dataBuffer));
	ee_read(52, 4, dataBuffer);
	memcpy(&zerocurrent_y_1p375, dataBuffer, sizeof zerocurrent_y_1p375);
}

void dacReadBack()
{
	unsigned char DACbuffer[3];

	DACbuffer[0] = 00000000; //Set write mode
	DACbuffer[1] = 00000000;
	DACbuffer[2] = 00000000;

	HAL_I2C_Master_Receive(&hi2c1, DAC8571_ADDRESS << 1, DACbuffer, 3, 100);

	uint16_t reading = (DACbuffer[0] << 8 | DACbuffer[1]);

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
	MX_DAC_Init();
	MX_I2C1_Init();
	MX_USB_DEVICE_Init();
	/* USER CODE BEGIN 2 */

	adc_init();
	ee_init();
	read_eeprom();

	// RESET = LOW, SET = HIGH
	HAL_GPIO_WritePin(GPIOA, LEDR_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, LEDG_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, LEDB_Pin, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(GPIOB, I_G0_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, I_G1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, I_G2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, I_G3_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, I_G4_Pin, GPIO_PIN_RESET);

	//set_pga_gain_8();
	set_pga_gain_1p375();
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
			char manualCurDACSearch[] = "ILIM";

			char adcSearch[] = "ADC";

			char ampEnableSearch[] = "CH1:ENA";
			char ampDisableSearch[] = "CH1:DIS";
			char setVoltageSearch[] = "CH1:VOL";
			char setMeasVoltageSearch[] = "CH1:MEA:VOL";

			char setCurrLimitSearch[] = "CH1:CUR";
			char setOversamplesSearch[] = "CH1:OSR";
			char setVCalModeSearch[] = "CH1:VCAL";
			char setManualRangeSearch[] = "CH1:RANGE";
			char setResetSearch[] = "*RST";
			char idnSearch[] = "*IDN?";

			char writeSearch[] = "WRITE";
			char readSearch[] = "*READ";

			char dacCalSearch[] = "CAL:DAC";
			char vCalSearch[] = "CAL:VOL";
			char iCal1p375Search[] = "CAL:CUR:RANGE1";
			char iCal8Search[] = "CAL:CUR:RANGE2";
			char iCal64Search[] = "CAL:CUR:RANGE3";
			char iCal176Search[] = "CAL:CUR:RANGE4";
			char iLimCalSearch[] = "CAL:ILIM";

			char *manualDACPtr = strstr(chRxBuffer, manualDACSearch);
			char *manualCurDACPtr = strstr(chRxBuffer, manualCurDACSearch);

			char *manualADCPtr = strstr(chRxBuffer, adcSearch);

			char *writeEEPtr = strstr(chRxBuffer, writeSearch);
			char *readEEPtr = strstr(chRxBuffer, readSearch);

			char *dacCalPtr = strstr(chRxBuffer, dacCalSearch);
			char *vCalPtr = strstr(chRxBuffer, vCalSearch);
			char *iCal1p375Ptr = strstr(chRxBuffer, iCal1p375Search);
			char *iCal8Ptr = strstr(chRxBuffer, iCal8Search);
			char *iCal64Ptr = strstr(chRxBuffer, iCal64Search);
			char *iCal176Ptr = strstr(chRxBuffer, iCal176Search);
			char *iLimCalPtr = strstr(chRxBuffer, iLimCalSearch);

			char *ampEnablePtr = strstr(chRxBuffer, ampEnableSearch);
			char *ampDisablePtr = strstr(chRxBuffer, ampDisableSearch);
			char *setVoltagePtr = strstr(chRxBuffer, setVoltageSearch);
			char *setMeasVoltagePtr = strstr(chRxBuffer, setMeasVoltageSearch);

			char *setCurrLimitPtr = strstr(chRxBuffer, setCurrLimitSearch);
			char *setOSRPtr = strstr(chRxBuffer, setOversamplesSearch);
			char *setVCalModePtr = strstr(chRxBuffer, setVCalModeSearch);
			char *setManualRangePtr = strstr(chRxBuffer, setManualRangeSearch);
			char *resetPtr = strstr(chRxBuffer, setResetSearch);
			char *idnPtr = strstr(chRxBuffer, idnSearch);

			if (ampEnablePtr != NULL)
			{
				HAL_GPIO_WritePin(GPIOA, AMP_EN_Pin, 1);
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
				HAL_Delay(1);
				single_shot();
			}

			else if (setCurrLimitPtr != NULL)
			{
				strtok(chRxBuffer, " ");
				float requested_current_limit = atof(strtok(NULL, " "));
				set_current_limit(requested_current_limit);
			}

			else if (setOSRPtr != NULL)
			{
				strtok(chRxBuffer, " ");
				num_oversamples = atoi(strtok(NULL, " "));
			}

			else if (setVCalModePtr != NULL)
			{
				calibration_mode = true;
			}
			else if (setManualRangePtr != NULL)
			{
				auto_gain_active = false;
				strtok(chRxBuffer, " ");
				uint8_t range = atoi(strtok(NULL, " "));
				switch (range)
				{
				case 0:
					set_pga_gain_1p375();
					auto_gain_active = true;
					break;
				case 1:
					set_pga_gain_1p375();
					zerocurrent_x = 0;
					zerocurrent_y = 0;
					break;
				case 2:
					set_pga_gain_8();
					zerocurrent_x = 0;
					zerocurrent_y = 0;
					break;
				case 3:
					set_pga_gain_64();
					zerocurrent_x = 0;
					zerocurrent_y = 0;
					break;
				case 4:
					set_pga_gain_176();
					zerocurrent_x = 0;
					zerocurrent_y = 0;
					break;
				}

			}
			else if (resetPtr != NULL)
			{
				NVIC_SystemReset();
			}
			else if (idnPtr != NULL)
			{
				uint32_t revision =
						READ_REG(
								*((uint32_t *)UID_BASE)) + READ_REG(*((uint32_t *)(UID_BASE + 4U))) + READ_REG(*((uint32_t *)(UID_BASE + 8U)));
				char msg[64];
				memset(msg, 0, sizeof(msg));
				sprintf(msg, "uSMU version 1.0 ID:%d", revision);
				strcat(msg, "\n");
				CDC_Transmit_FS(msg, sizeof(msg));

			}

			else if (writeEEPtr != NULL)
			{
				strtok(chRxBuffer, " ");
				float num_to_write = atof(strtok(NULL, " "));

				char data[sizeof(float)];
				memcpy(data, &num_to_write, sizeof num_to_write); // send data

				ee_writeToRam(0, 4, data);
				ee_commit();
			}
			else if (dacCalPtr != NULL)
			{
				strtok(chRxBuffer, " ");
				float slope_to_write = atof(strtok(NULL, " "));
				float intercept_to_write = atof(strtok(NULL, " "));

				char x_data[sizeof(float)];
				char y_data[sizeof(float)];

				memcpy(x_data, &slope_to_write, sizeof slope_to_write); // send data
				memcpy(y_data, &intercept_to_write, sizeof intercept_to_write); // send data

				ee_writeToRam(0, 4, x_data);
				ee_writeToRam(4, 4, y_data);
				ee_commit();
			}

			else if (vCalPtr != NULL)
			{
				strtok(chRxBuffer, " ");
				float slope_to_write = atof(strtok(NULL, " "));
				float intercept_to_write = atof(strtok(NULL, " "));

				char x_data[sizeof(float)];
				char y_data[sizeof(float)];

				memcpy(x_data, &slope_to_write, sizeof slope_to_write); // send data
				memcpy(y_data, &intercept_to_write, sizeof intercept_to_write); // send data

				ee_writeToRam(8, 4, x_data);
				ee_writeToRam(12, 4, y_data);

				ee_commit();
			}
			else if (iCal1p375Ptr != NULL)
			{
				strtok(chRxBuffer, " ");
				float slope_to_write = atof(strtok(NULL, " "));
				float intercept_to_write = atof(strtok(NULL, " "));

				char x_data[sizeof(float)];
				char y_data[sizeof(float)];

				memcpy(x_data, &slope_to_write, sizeof slope_to_write); // send data
				memcpy(y_data, &intercept_to_write, sizeof intercept_to_write); // send data

				ee_writeToRam(48, 4, x_data);
				ee_writeToRam(52, 4, y_data);
				ee_commit();
			}
			else if (iCal8Ptr != NULL)
			{
				strtok(chRxBuffer, " ");
				float slope_to_write = atof(strtok(NULL, " "));
				float intercept_to_write = atof(strtok(NULL, " "));

				char x_data[sizeof(float)];
				char y_data[sizeof(float)];

				memcpy(x_data, &slope_to_write, sizeof slope_to_write); // send data
				memcpy(y_data, &intercept_to_write, sizeof intercept_to_write); // send data

				ee_writeToRam(16, 4, x_data);
				ee_writeToRam(20, 4, y_data);
				ee_commit();
			}
			else if (iCal64Ptr != NULL)
			{
				strtok(chRxBuffer, " ");
				float slope_to_write = atof(strtok(NULL, " "));
				float intercept_to_write = atof(strtok(NULL, " "));

				char x_data[sizeof(float)];
				char y_data[sizeof(float)];

				memcpy(x_data, &slope_to_write, sizeof slope_to_write); // send data
				memcpy(y_data, &intercept_to_write, sizeof intercept_to_write); // send data

				ee_writeToRam(24, 4, x_data);
				ee_writeToRam(28, 4, y_data);
				ee_commit();
			}
			else if (iCal176Ptr != NULL)
			{
				strtok(chRxBuffer, " ");
				float slope_to_write = atof(strtok(NULL, " "));
				float intercept_to_write = atof(strtok(NULL, " "));

				char x_data[sizeof(float)];
				char y_data[sizeof(float)];

				memcpy(x_data, &slope_to_write, sizeof slope_to_write); // send data
				memcpy(y_data, &intercept_to_write, sizeof intercept_to_write); // send data

				ee_writeToRam(32, 4, x_data);
				ee_writeToRam(36, 4, y_data);
				ee_commit();
			}
			else if (iLimCalPtr != NULL)
			{
				strtok(chRxBuffer, " ");
				float slope_to_write = atof(strtok(NULL, " "));
				float intercept_to_write = atof(strtok(NULL, " "));

				char x_data[sizeof(float)];
				char y_data[sizeof(float)];

				memcpy(x_data, &slope_to_write, sizeof slope_to_write); // send data
				memcpy(y_data, &intercept_to_write, sizeof intercept_to_write); // send data

				ee_writeToRam(40, 4, x_data);
				ee_writeToRam(44, 4, y_data);
				ee_commit();
			}

			else if (readEEPtr != NULL)
			{
				strtok(chRxBuffer, " ");
				int requested_address = atoi(strtok(NULL, " "));

				static uint8_t dataRead[4];
				ee_read(requested_address, 4, dataRead);
				float g;
				memcpy(&g, dataRead, sizeof g);    // receive data

				char msg[1024];
				memset(msg, 0, sizeof(msg));
				sprintf(msg, "%e", g);
				strcat(msg, "\n");
				CDC_Transmit_FS(msg, sizeof(msg));

			}
			else if (manualCurDACPtr != NULL)
			{
				strtok(chRxBuffer, " ");
				int requested_level = atoi(strtok(NULL, " "));
				HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R,
						requested_level);
				HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
			}
			else if (manualDACPtr != NULL)
			{
				strtok(chRxBuffer, " ");
				int requested_level = atoi(strtok(NULL, " "));
				DAC8571_amp_dac_write(requested_level);
			}

			else
			{
				char msg[1024] = "Test string not found";
				strcat(msg, "\n");
				CDC_Transmit_FS(msg, sizeof(msg));
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
	RCC_OscInitTypeDef RCC_OscInitStruct =
	{ 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct =
	{ 0 };
	RCC_PeriphCLKInitTypeDef PeriphClkInit =
	{ 0 };

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI
			| RCC_OSCILLATORTYPE_HSI48;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
	{
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB | RCC_PERIPHCLK_I2C1;
	PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
	PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;

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

	DAC_ChannelConfTypeDef sConfig =
	{ 0 };

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
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct =
	{ 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA,
			LEDR_Pin | LEDG_Pin | LEDB_Pin | AMP_EN_Pin | I_G4_Pin | I_G3_Pin
					| I_G2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, I_G0_Pin | I_G1_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pins : LEDR_Pin LEDG_Pin LEDB_Pin AMP_EN_Pin */
	GPIO_InitStruct.Pin = LEDR_Pin | LEDG_Pin | LEDB_Pin | AMP_EN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : I_G0_Pin I_G1_Pin */
	GPIO_InitStruct.Pin = I_G0_Pin | I_G1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : I_G4_Pin I_G3_Pin I_G2_Pin */
	GPIO_InitStruct.Pin = I_G4_Pin | I_G3_Pin | I_G2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : ADC_Alert_Pin */
	GPIO_InitStruct.Pin = ADC_Alert_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
