/* USER CODE BEGIN Header */
/**
  **************************
  * @file           : main.c
  * @brief          : Main program body
  **************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  **************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <math.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
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
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
#define MAX_SAMPLES 50
#define scale_factor 1.2f
#define SYSTEM_CLOCK_FREQUENCY 4000000 // 4 MHz
#define FULL_SCALE_CURRENT 75.0  // Assuming full-scale current range is 100 mA
#define LED_RANGE 0x1  // Corresponds to 100 mA full-scale current range (binary 10)
unsigned char uart_buf[100];
unsigned int uart_buf_len;
unsigned char spi_buf[20];

volatile uint8_t data_buffer[10];
volatile uint8_t data_full[100];
volatile uint8_t finished = 0;
volatile uint8_t indice = 0;
uint32_t dataSPI;
int IRdata;
int AIRdata;
int REDdata;
int AREDdata;
int RED_AREDdata;
int IR_AIRdata;
int IRdc[MAX_SAMPLES];
int Reddc[MAX_SAMPLES];
int i = 0;
float difIRheartsig_dc;
float difREDheartsig_dc;
float powdifIR;
float powdifRed;
float IRac;
float Redac;
float Ratio;
float SpO2 = 0;
float SpOpercentage = 0;
float voltage_REDdata;
float voltage_IRdata;

// Define AFE4490 register addresses
uint8_t CONTROL0 = 0x00;
uint8_t LED2STC = 0x01;
uint8_t LED2ENDC = 0x02;
uint8_t LED2LEDSTC = 0x03;
uint8_t LED2LEDENDC = 0x04;
uint8_t ALED2STC = 0x05;
uint8_t ALED2ENDC = 0x06;
uint8_t LED1STC = 0x07;
uint8_t LED1ENDC = 0x08;
uint8_t LED1LEDSTC = 0x09;
uint8_t LED1LEDENDC = 0x0A;
uint8_t ALED1STC = 0x0B;
uint8_t ALED1ENDC = 0x0C;
uint8_t LED2CONVST = 0x0D;
uint8_t LED2CONVEND = 0x0E;
uint8_t ALED2CONVST = 0x0F;
uint8_t ALED2CONVEND = 0x10;
uint8_t LED1CONVST = 0x11;
uint8_t LED1CONVEND = 0x12;
uint8_t ALED1CONVST = 0x13;
uint8_t ALED1CONVEND = 0x14;
uint8_t ADCRSTSTCT0 = 0x15;
uint8_t ADCRSTENDCT0 = 0x16;
uint8_t ADCRSTSTCT1 = 0x17;
uint8_t ADCRSTENDCT1 = 0x18;
uint8_t ADCRSTSTCT2 = 0x19;
uint8_t ADCRSTENDCT2 = 0x1A;
uint8_t ADCRSTSTCT3 = 0x1B;
uint8_t ADCRSTENDCT3 = 0x1C;
uint8_t PRPCOUNT = 0x1D;
uint8_t CONTROL1 = 0x1E;
uint8_t SPARE1 = 0x1F;
uint8_t TIAGAIN = 0x20;
uint8_t TIA_AMB_GAIN = 0x21;
uint8_t LEDCNTRL = 0x22;
uint8_t CONTROL2 = 0x23;
uint8_t SPARE2 = 0x24;
uint8_t SPARE3 = 0x25;
uint8_t SPARE4 = 0x26;
uint8_t RESERVED1 = 0x27;
uint8_t RESERVED2 = 0x28;
uint8_t ALARM = 0x29;
uint8_t LED2VAL = 0x2A;
uint8_t ALED2VAL = 0x2B;
uint8_t LED1VAL = 0x2C;
uint8_t ALED1VAL = 0x2D;
uint8_t LED2_ALED2VAL = 0x2E;
uint8_t LED1_ALED1VAL = 0x2F;
uint8_t DIAG = 0x30;

int a=0;
int b=0;
int c=0;
int e=0;
int f=0;
int hna=0;
int recu = 0;
unsigned char rx_data[10];
unsigned char start_data[10];
HAL_StatusTypeDef status;
int Red_current=0;
int IR_current=0;
float Frequency=0;
float Gain=0;
int Capacity=0;
int Resistance=0;
uint32_t register_value;
uint8_t PRP;
uint32_t value1;
uint32_t value2;
uint32_t ledcntrl_value;
uint16_t TIAGAIN_value;
volatile uint8_t stop_requested  = 0;
uint8_t uart_rx_buffer[10];
uint8_t stop_rx_data[16];
volatile uint8_t start_message_received = 0;
volatile uint8_t start_received = 0;
volatile uint8_t received = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
//static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */
void main_func(void);
void CheckForDebugMessage(void);
void set_PRPCOUNT_frequency(float desired_frequency);
void set_LED_currents(float led1_current_mA, float led2_current_mA);
void set_TIAGAIN_LED1(int RF_LED1, int CF_LED1, float STG2GAIN1);
void AFE4490_powerDown(uint8_t k);
void AFE4490_readEnable(uint8_t k);
void AFE4490_write(uint8_t addr, uint32_t data);
uint32_t AFE4490_read(uint8_t addr);
void AFE4490_registersInit();
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
float mean(int* data, int length);
float calculateSpO2();
//void sendSpO2Value(float spo2);
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
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  //MX_NVIC_Init();
  /* USER CODE BEGIN 2 */

  // Set AFE_PDNZ to 1
  //HAL_GPIO_WritePin(AFE_PDNZ_GPIO_Port, AFE_PDNZ_Pin, GPIO_PIN_SET);
  // Set SPI_CS to 1 (IDLE MODE)
  //HAL_GPIO_WritePin(SPI_CS0_GPIO_Port, SPI_CS0_Pin, GPIO_PIN_SET);

  // Send test UART message
  //uart_buf_len = sprintf(uart_buf, "AFE4490 Basic Test Acquisition program\r\n");
  //HAL_UART_Transmit(&huart2, uart_buf, uart_buf_len, HAL_MAX_DELAY);

  //-----------------------Some AFE4490 code tests-------------------------------
  // Power Down AFE4490
  // AFE4490_powerDown(1);
  // Power Up AFE4490
  // AFE4490_powerDown(0);

  // Write value to registers
  /*
  AFE4490_readEnable(0);

  spi_buf[0] = 0x00;
  spi_buf[1] = 0x4e;
  spi_buf[2] = 0xbf;

  HAL_GPIO_WritePin(SPI_CS0_GPIO_Port, SPI_CS0_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi1, (uint8_t *)&LED2STC, 1, HAL_MAX_DELAY);
  HAL_SPI_Transmit(&hspi1, (uint8_t *) spi_buf, 3, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(SPI_CS0_GPIO_Port, SPI_CS0_Pin, GPIO_PIN_SET);

  spi_buf[0] = 0x00;
  spi_buf[1] = 0xac;
  spi_buf[2] = 0xc8;

  HAL_GPIO_WritePin(SPI_CS0_GPIO_Port, SPI_CS0_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi1, (uint8_t *)&LED2ENDC, 1, HAL_MAX_DELAY);
  HAL_SPI_Transmit(&hspi1, (uint8_t *) spi_buf, 3, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(SPI_CS0_GPIO_Port, SPI_CS0_Pin, GPIO_PIN_SET);

  // Read back values from registers
  AFE4490_readEnable(1);

  spi_buf[0] = 0x00;
  spi_buf[1] = 0x00;
  spi_buf[2] = 0x00;

  HAL_GPIO_WritePin(SPI_CS0_GPIO_Port, SPI_CS0_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi1, (uint8_t *)&LED2STC, 1, HAL_MAX_DELAY);
  HAL_SPI_Receive(&hspi1, (uint8_t *)spi_buf, 3, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(SPI_CS0_GPIO_Port, SPI_CS0_Pin, GPIO_PIN_SET);

  uart_buf_len = sprintf(uart_buf, "\r\nBEGIN Reading\r\nLED2STC = 0x%02x%02x%02x\t", spi_buf[0], spi_buf[1], spi_buf[2]);
  HAL_UART_Transmit(&huart2, uart_buf, uart_buf_len, HAL_MAX_DELAY);

  spi_buf[0] = 0x00;
  spi_buf[1] = 0x00;
  spi_buf[2] = 0x00;

  HAL_GPIO_WritePin(SPI_CS0_GPIO_Port, SPI_CS0_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi1, (uint8_t *)&LED2ENDC, 1, HAL_MAX_DELAY);
  HAL_SPI_Receive(&hspi1, (uint8_t *)spi_buf, 3, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(SPI_CS0_GPIO_Port, SPI_CS0_Pin, GPIO_PIN_SET);

  uart_buf_len = sprintf(uart_buf, "LED2ENDC = 0x%02x%02x%02x\r\nEND Reading\r\n", spi_buf[0], spi_buf[1], spi_buf[2]);
  HAL_UART_Transmit(&huart2, uart_buf, uart_buf_len, HAL_MAX_DELAY);
  */
  //-----------------------AFE4490 END code tests-------------------------------

  // Test using Write and Read functions
  /*
  AFE4490_write(LED2STC, 0x004ebf);
  dataSPI = AFE4490_read(LED2STC);

  uart_buf_len = sprintf(uart_buf, "LED2STC = 0x%06x\r\n", dataSPI);
  HAL_UART_Transmit(&huart2, uart_buf, uart_buf_len, HAL_MAX_DELAY);
  */



  // AFE4490_write(CONTROL0, 0x000008);  //  Reset all registers to default values



  /*
  AFE4490_write(CONTROL0, 0x000008);
  AFE4490_write(CONTROL0, 0x000000);
  */

  //AFE4490_registersInit_62k5();




  // AFE4490_readEnable(0);
  /*
  AFE4490_write(0x01,0x0017C0);    ////
  AFE4490_write(0x02,0x001f3E);    ////
  AFE4490_write(0x03,0x001770);    ////
  AFE4490_write(0x04,0x001F3F);    ////
  AFE4490_write(0x05,0x000050);    ////
  AFE4490_write(0x06,0x0007CE);    ////
  AFE4490_write(0x07,0x000820);    ////
  AFE4490_write(0x08,0x000F9E);    ////
  AFE4490_write(0x09,0x0007D0);    ////
  AFE4490_write(0x0A,0x000F9F);    ////
  AFE4490_write(0x0B,0x000FF0);    ////
  AFE4490_write(0x0C,0x00176E);    ////
  AFE4490_write(0x0D,0x000006);    ////
  AFE4490_write(0x0E,0x0007cf);    ////
  AFE4490_write(0x0F,0x0007d6);    ////
  AFE4490_write(0x10,0x000f9f);    ////
  AFE4490_write(0x11,0x000fa6);    ////
  AFE4490_write(0x12,0x00176f);    ////
  AFE4490_write(0x13,0x001776);    ////
  AFE4490_write(0x14,0x001f3f);    ////
  AFE4490_write(0x15,0x000000);    ////
  AFE4490_write(0x16,0x000005);    ////
  AFE4490_write(0x17,0x0007d0);    ////
  AFE4490_write(0x18,0x0007d5);    ////
  AFE4490_write(0x19,0x000fa0);    ////
  AFE4490_write(0x1A,0x000fa5);    ////
  AFE4490_write(0x1B,0x001770);    ////
  AFE4490_write(0x1C,0x001775);    ////
  AFE4490_write(0x1D,0x001f3f);    ////
  AFE4490_write(0x1E,0x000101);    ////
  AFE4490_write(0x1F,0x000000);    ////
  AFE4490_write(0x20,0x000000);
  // AFE4490_write(0x21,0x000000);
  AFE4490_write(TIA_AMB_GAIN,0x000005);
  // AFE4490_write(0x22,0x02ffff);
  AFE4490_write(LEDCNTRL, 0x011414);
  AFE4490_write(0x23,0x020000);    ////
  AFE4490_write(0x24,0x000000);
  AFE4490_write(0x25,0x000000);
  AFE4490_write(0x26,0x000000);
  AFE4490_write(0x27,0x000000);
  AFE4490_write(0x28,0x000000);
  AFE4490_write(0x29,0x000080);
  AFE4490_write(0x2A,0x000000);
  AFE4490_write(0x2B,0x000000);
  AFE4490_write(0x2C,0x000000);
  AFE4490_write(0x2D,0x000000);
  AFE4490_write(0x2E,0x000000);
  AFE4490_write(0x2F,0x000000);
  AFE4490_write(0x30,0x000000);
  */


  // Basic configuration to observe ADC_RDY Pin
  /*
  AFE4490_write(CONTROL0, 0x000002);
  AFE4490_write(PRPCOUNT, 0x00FA0);
  AFE4490_write(CONTROL0, 0x000000);
  AFE4490_write(CONTROL1, 0x000100);
  */

  /*uint8_t addr;

  // Read registers from 0x01 to 0x23

  for (addr = 1; addr <= 35; addr++)
  {
	  dataSPI = AFE4490_read(addr);
	  uart_buf_len = sprintf(uart_buf, "addr 0x%02x = 0x%06x\r\n", addr, dataSPI);
	  HAL_UART_Transmit(&huart2, uart_buf, uart_buf_len, HAL_MAX_DELAY);
	  HAL_Delay(100);
  */
  //AFE4490_write(LEDCNTRL, 0x011414);

  //a=1;
  //set_LED_currents(5.859, 5.859);
  //set_TIAGAIN_LED1(500,5,0);
  //AFE4490_registersInit_62k5();
  //value  = AFE4490_read(TIAGAIN);

  /*
  AFE4490_write(LEDCNTRL, 0x010000);  //  I = 0 mA
  AFE4490_write(LEDCNTRL, 0x010303);  //  I = 1 mA
  AFE4490_write(LEDCNTRL, 0x010707);  //  I = 2 mA
  AFE4490_write(LEDCNTRL, 0x010A0A);  //  I = 3 mA
  AFE4490_write(LEDCNTRL, 0x010E0E);  //  I = 4 mA
  AFE4490_write(LEDCNTRL, 0x010E0E);  //  I = 5 mA
  AFE4490_write(LEDCNTRL, 0x011414);  //  I = 5.8835 mA
  */






  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	 // Toggle LED_STATUS pin every x seconds

	 //HAL_GPIO_TogglePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin);
	 //HAL_Delay(500);
	  a++;
	  main_func();
	  //AFE4490_write(LEDCNTRL, 0x011414);
	  //main_func();
	  //PRPCOUNT = 0x1D;
	  //set_PRPCOUNT_frequency(500);
	  // uart_buf_len = sprintf(uart_buf, "RED: %d\tIR: %d\r\n", REDdata, IRdata);
	  /*uart_buf_len = sprintf(uart_buf, "RED: %d\r\n", REDdata);
	  HAL_UART_Transmit(&huart2, uart_buf, uart_buf_len, HAL_MAX_DELAY);
	  */





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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/*static void MX_NVIC_Init(void)
{
	/* USART2_IRQn interrupt configuration
	HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(USART2_IRQn);
  /* EXTI1_IRQn interrupt configuration
	HAL_NVIC_SetPriority(EXTI1_IRQn, 1, 0);
	HAL_NVIC_EnableIRQ(EXTI1_IRQn);
}*/

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(AFE_PDNZ_GPIO_Port, AFE_PDNZ_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI_CS0_GPIO_Port, SPI_CS0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : AFE_PD_ALM_Pin AFE_DIAG_Pin */
  GPIO_InitStruct.Pin = AFE_PD_ALM_Pin|AFE_DIAG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_STATUS_Pin */
  GPIO_InitStruct.Pin = LED_STATUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_STATUS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ADC_RDY_Pin */
  GPIO_InitStruct.Pin = ADC_RDY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ADC_RDY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : AFE_PDNZ_Pin SPI_CS0_Pin */
  GPIO_InitStruct.Pin = AFE_PDNZ_Pin|SPI_CS0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  //HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  //HAL_NVIC_EnableIRQ(EXTI1_IRQn);

}

/* USER CODE BEGIN 4 */


// Enable/Disable AFE4490 Read/Write Mode
void AFE4490_readEnable(uint8_t k)
{
	unsigned char spi_buf[3];
	spi_buf[0] = 0x00;
	spi_buf[1] = 0x00;

	switch(k)
	{
	case 1:
	spi_buf[2] = 0x01;
	HAL_GPIO_WritePin(SPI_CS0_GPIO_Port, SPI_CS0_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, (uint8_t *)&CONTROL0, 1, HAL_MAX_DELAY);
	HAL_SPI_Transmit(&hspi1, (uint8_t *) spi_buf, 3, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(SPI_CS0_GPIO_Port, SPI_CS0_Pin, GPIO_PIN_SET);
	break;
	case 0:
	spi_buf[2] = 0x00;
	HAL_GPIO_WritePin(SPI_CS0_GPIO_Port, SPI_CS0_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, (uint8_t *)&CONTROL0, 1, HAL_MAX_DELAY);
	HAL_SPI_Transmit(&hspi1, (uint8_t *) spi_buf, 3, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(SPI_CS0_GPIO_Port, SPI_CS0_Pin, GPIO_PIN_SET);
	break;
	default:
	// Do nothing
	break;
	}
}

// AFE4490 Power down
void AFE4490_powerDown(uint8_t k)
{
	unsigned char spi_buf[3];
	spi_buf[0] = 0x00;
	spi_buf[1] = 0x00;

	switch(k)
	{
	case 1:
	spi_buf[2] = 0x01;
	HAL_GPIO_WritePin(SPI_CS0_GPIO_Port, SPI_CS0_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, (uint8_t *)&CONTROL2, 1, HAL_MAX_DELAY);
	HAL_SPI_Transmit(&hspi1, (uint8_t *) spi_buf, 3, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(SPI_CS0_GPIO_Port, SPI_CS0_Pin, GPIO_PIN_SET);
	break;
	case 0:
	spi_buf[2] = 0x00;
	HAL_GPIO_WritePin(SPI_CS0_GPIO_Port, SPI_CS0_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, (uint8_t *)&CONTROL2, 1, HAL_MAX_DELAY);
	HAL_SPI_Transmit(&hspi1, (uint8_t *) spi_buf, 3, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(SPI_CS0_GPIO_Port, SPI_CS0_Pin, GPIO_PIN_SET);
	break;
	default:
	// Do nothing
	break;
	}
}



// AFE4490 Write Function
void AFE4490_write(uint8_t addr, uint32_t data)
{
	unsigned char spi_buf[20];

	spi_buf[0] = (data>>16) & 0xFF;
	spi_buf[1] = (data>>8) & 0xFF;
	spi_buf[2] = data & 0xFF;

	AFE4490_readEnable(0);

	HAL_GPIO_WritePin(SPI_CS0_GPIO_Port, SPI_CS0_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, (uint8_t *)&addr, 1, HAL_MAX_DELAY);
	HAL_SPI_Transmit(&hspi1, (uint8_t *) spi_buf, 3, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(SPI_CS0_GPIO_Port, SPI_CS0_Pin, GPIO_PIN_SET);
}

uint32_t AFE4490_read(uint8_t addr)
{
	unsigned char spi_buf[20];
	uint32_t data;
	spi_buf[0] = 0x00;
	spi_buf[1] = 0x00;
	spi_buf[2] = 0x00;


	AFE4490_readEnable(1);

	HAL_GPIO_WritePin(SPI_CS0_GPIO_Port, SPI_CS0_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, (uint8_t *)&addr, 1, HAL_MAX_DELAY);
	HAL_SPI_Receive(&hspi1, (uint8_t *) spi_buf, 3, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(SPI_CS0_GPIO_Port, SPI_CS0_Pin, GPIO_PIN_SET);

	data = ((spi_buf[0] & 0xFF) << 16) + ((spi_buf[1] & 0xFF) << 8) + (spi_buf[2] & 0xFF);

	return data;
}

void CheckForDebugMessage(void)
{
  HAL_StatusTypeDef status1;

  // Vérifiez s'il y a des données disponibles en UART sans bloquer
  HAL_UART_Receive(&huart2, rx_data, 16, 10000);
    // Vérifiez si les données reçues correspondent au message de débogage attendu
    if (rx_data[0]=='p' && rx_data[1]=='r' && rx_data[2]=='e' && rx_data[3]=='t')
    {
      // Message de démarrage reçu, mettre à jour la variable globale
      start_received = 1;
    }
}

void main_func(void){
	enum ETAT {
		SET_IRED,
		RX_REDCURRENT,
		SET_IIR,
		RX_IRCURRENT,
		SET_GAIN,
		RX_GAIN,
		SET_CAPACITY,
		RX_CAPACITY,
		SET_FREQUENCY,
		RX_FREQUENCY,
		SET_RESISTANCE,
		RX_RESISTANCE,
		CONF_AFE,
		CHECK_CONF,
		TX_STARTAQUI,
		TX_DONNEE,
		S_FIN
	};
	static enum ETAT Etat = SET_IRED;

	switch(Etat){
	case SET_IRED: {
		uart_buf_len = sprintf(uart_buf, "Entrer IRED\r\n");

		// Transmettre le message via UART
		HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, uart_buf_len, HAL_MAX_DELAY);
		Etat = RX_REDCURRENT;
		break;
	}
	case RX_REDCURRENT: {
		status = HAL_UART_Receive(&huart2, rx_data, 16, 10000);
		if(status==HAL_TIMEOUT){
			sscanf(rx_data, "IRED: %d", &Red_current);
			Etat = SET_IIR;
		}
		break;
	}
	case SET_IIR: {
		uart_buf_len = sprintf(uart_buf, "Entrer IIR\r\n");

		// Transmettre le message via UART
		HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, uart_buf_len, HAL_MAX_DELAY);
		Etat = RX_IRCURRENT;
		break;
	}
	case RX_IRCURRENT: {
		status = HAL_UART_Receive(&huart2, rx_data, 16, 10000);
		if(status==HAL_TIMEOUT){
			sscanf(rx_data, "IIR: %d", &IR_current);
			Etat = SET_GAIN;
		}
		break;
	}
	case SET_GAIN: {
		uart_buf_len = sprintf(uart_buf, "Entrer G\r\n");

		// Transmettre le message via UART
		HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, uart_buf_len, HAL_MAX_DELAY);
		Etat = RX_GAIN;
		break;
	}
	case RX_GAIN: {
		status = HAL_UART_Receive(&huart2, rx_data, 16, 10000);
		if(status==HAL_TIMEOUT){
			sscanf(rx_data, "G: %f", &Gain);
			Etat = SET_CAPACITY;
		}
		break;
	}
	case SET_CAPACITY: {
		uart_buf_len = sprintf(uart_buf, "Entrer Cf\r\n");

		// Transmettre le message via UART
		HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, uart_buf_len, HAL_MAX_DELAY);
		Etat = RX_CAPACITY;
		break;
	}
	case RX_CAPACITY: {
		status = HAL_UART_Receive(&huart2, rx_data, 16, 10000);
		if(status==HAL_TIMEOUT){
			sscanf(rx_data, "Cf: %d", &Capacity);
			Etat = SET_FREQUENCY;
		}
		break;
	}
	case SET_FREQUENCY: {
		uart_buf_len = sprintf(uart_buf, "Entrer F\r\n");

		// Transmettre le message via UART
		HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, uart_buf_len, HAL_MAX_DELAY);
		Etat = RX_FREQUENCY;
		break;
	}
	case RX_FREQUENCY: {
		status = HAL_UART_Receive(&huart2, rx_data, 16, 10000);
		if(status==HAL_TIMEOUT){
			sscanf(rx_data, "F: %f", &Frequency);
			Etat = SET_RESISTANCE;
		}
		break;
	}
	case SET_RESISTANCE: {
		uart_buf_len = sprintf(uart_buf, "Entrer Rf\r\n");

		// Transmettre le message via UART
		HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, uart_buf_len, HAL_MAX_DELAY);
		Etat = RX_RESISTANCE;
		break;
	}
	case RX_RESISTANCE: {
		status = HAL_UART_Receive(&huart2, rx_data, 16, 10000);
		if(status==HAL_TIMEOUT){
			sscanf(rx_data, "Rf: %d", &Resistance);
			Etat = CONF_AFE;
		}
		break;
	}
	case CONF_AFE: {
		AFE4490_registersInit_62k5();
		Etat = CHECK_CONF;
		break;
	}
	case CHECK_CONF : {
		value1 = AFE4490_read(PRPCOUNT);
		value2 = AFE4490_read(LEDCNTRL);
		Etat = TX_STARTAQUI;
		break;
	}
	case TX_STARTAQUI: {
		uart_buf_len = sprintf(uart_buf, "Configuration terminée\r\n");

    	// Transmettre le message via UART
    	HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, uart_buf_len, HAL_MAX_DELAY);
    	b=1;
		Etat = TX_DONNEE;
		break;
	}
	case TX_DONNEE: {
		HAL_NVIC_EnableIRQ(EXTI1_IRQn);
		f=1;
		Etat = S_FIN;
   		break;
	}
	case S_FIN: {
		Etat = SET_IRED;
		break;
	}
	}
}

void set_PRPCOUNT_frequency(float desired_frequency) {
    // Calculate the Pulse Repetition Period in clock cycles
    float period_seconds = 1.0f / desired_frequency;
    uint32_t PRPCOUNT_value = (uint32_t)(period_seconds * SYSTEM_CLOCK_FREQUENCY) - 1;

    // Ensure the PRPCOUNT value is within the valid range (800 to 64000)
    if (PRPCOUNT_value < 800) {
        PRPCOUNT_value = 800;
    } else if (PRPCOUNT_value > 64000) {
        PRPCOUNT_value = 64000;
    }

    // Ensure the higher bits are set to zero and only the lower 16 bits are used
    register_value = (PRPCOUNT_value & 0xFFFF);

    // Write the value to the PRPCOUNT register
    AFE4490_write(PRPCOUNT, register_value);
}

void set_LED_currents(float led1_current_mA, float led2_current_mA) {
    // Ensure the LED currents are within the valid range
    if (led1_current_mA > FULL_SCALE_CURRENT) {
        led1_current_mA = FULL_SCALE_CURRENT;
    }
    if (led2_current_mA > FULL_SCALE_CURRENT) {
        led2_current_mA = FULL_SCALE_CURRENT;
    }

    // Calculate the register values
    uint8_t led1_register_value = (uint8_t)round((led1_current_mA / FULL_SCALE_CURRENT) * 256);
    uint8_t led2_register_value = (uint8_t)round((led2_current_mA / FULL_SCALE_CURRENT) * 256);

    // Combine the values into the LEDCNTRL register format
    // Bits D[23:18] must be '0'
    // Bits D[17:16] LED_RANGE[1:0]
    // Bits D[15:8] LED1[7:0]
    // Bits D[7:0] LED2[7:0]
    ledcntrl_value = (LED_RANGE << 16) | (led1_register_value << 8) | led2_register_value;

    // Write to the LEDCNTRL register
    AFE4490_write(LEDCNTRL, ledcntrl_value);
}

void set_TIAGAIN_LED1(int RF_LED1, int CF_LED1, float STG2GAIN1) {
    // Map RF_LED1 to its corresponding bit value
    uint8_t rf_led_bits;
    if (RF_LED1 == 500) {
        rf_led_bits = 0b000;
    } else if (RF_LED1 == 250) {
        rf_led_bits = 0b001;
    } else if (RF_LED1 == 100) {
        rf_led_bits = 0b010;
    } else if (RF_LED1 == 50) {
        rf_led_bits = 0b011;
    } else if (RF_LED1 == 25) {
        rf_led_bits = 0b100;
    } else if (RF_LED1 == 10) {
        rf_led_bits = 0b101;
    } else if (RF_LED1 == 1000) {
        rf_led_bits = 0b110;
    } else {
        rf_led_bits = 0b111; // None
    }

    // Map CF_LED1 to its corresponding bit value
    uint8_t cf_led_bits;
    switch (CF_LED1) {
        case 5:
            cf_led_bits = 0b00000;
            break;
        case 10:
            cf_led_bits = 0b00001;
            break;
        case 25:
            cf_led_bits = 0b00100;
            break;
        case 50:
            cf_led_bits = 0b01000;
            break;
        case 100:
            cf_led_bits = 0b10000;
            break;
        case 250:
            cf_led_bits = 0b11001;
            break;
        default:
            cf_led_bits = 0b00000; // Default to 5 pF
            break;
    }

    // Map STG2GAIN1 to its corresponding bit value
    uint8_t stg2gain_bits;
    if (STG2GAIN1 == 0) {
        stg2gain_bits = 0b000;
    } else if (STG2GAIN1 == 3.5) {
        stg2gain_bits = 0b001;
    } else if (STG2GAIN1 == 6) {
        stg2gain_bits = 0b010;
    } else if (STG2GAIN1 == 9.5) {
        stg2gain_bits = 0b011;
    } else if (STG2GAIN1 == 12) {
        stg2gain_bits = 0b100;
    } else {
        stg2gain_bits = 0b000; // Default to 0 dB
    }

    // Construct the value to be written to the TIAGAIN register for LED1
    TIAGAIN_value = (stg2gain_bits << 8) | (cf_led_bits << 3) | rf_led_bits;

    // Now, you can use this TIAGAIN value to initialize the AFE4490 TIAGAIN register for LED1
    AFE4490_write(TIAGAIN, TIAGAIN_value);
}


float mean(int* data, int length) {
    float sum = 0;
    for (int i = 0; i < length; i++) {
        sum += data[i];
    }
    return sum / length;
}

/*float calculateSpO2() {


    IRdc[i] = mean(IRdata, MAX_SAMPLES);
    Reddc[i] = mean(REDdata, MAX_SAMPLES);


    difIRheartsig_dc = IRdata[i] - IRdc[i];
    difREDheartsig_dc = REDdata[i] - Reddc[i];


    powdifIR = pow(difIRheartsig_dc, 2.0);
    powdifRed = pow(difREDheartsig_dc, 2.0);


    IRac = powdifIR / i;
    Redac = powdifRed / i;


    Ratio = (Redac / Reddc[i]) / (IRac / IRdc[i]);


    SpO2 = 100 - Ratio;
    return SpO2;
    // Imprimer le taux de SpO2
    //Serial.println("SpO2 percentage: " + String(SpO2));
}*/

/*void sendSpO2Value(float spo2) {
    char uart_buffer[50];
    int len = sprintf(uart_buffer, "SpO2: %.2f\r\n", spo2);
    HAL_UART_Transmit(&huart2, (uint8_t *)uart_buffer, len, HAL_MAX_DELAY);
}*/

// AFE4490 Registers Initialization
void AFE4490_registersInit_500k()
{
	char uart_buf[50];
	unsigned int uart_buf_len;

	uart_buf_len = sprintf(uart_buf, "AFE4490 registers initialization ...\r\n");
	HAL_UART_Transmit(&huart2, uart_buf, uart_buf_len, HAL_MAX_DELAY);

	// Enable Write Mode
	AFE4490_readEnable(0);

	// Timer Module Configuration
	// PRPCOUNT Configuration, Set PRF = 500 Hz --> PRPCOUNT = 7999 = 0x1F3F
	AFE4490_write(PRPCOUNT, 0x001F3F);

	// Conversions sequence timing configuration
	AFE4490_write(LED2CONVST, 0x000006);
	AFE4490_write(LED2CONVEND, 0x0007CF);
	AFE4490_write(ALED2CONVST, 0x0007D6);
	AFE4490_write(ALED2CONVEND, 0x000F9F);
	AFE4490_write(LED1CONVST, 0x000FA6);
	AFE4490_write(LED1CONVEND, 0x00176F);
	AFE4490_write(ALED1CONVST, 0x001776);
	AFE4490_write(ALED1CONVEND, 0x001F3F);
	AFE4490_write(ADCRSTSTCT0, 0x000000);
	AFE4490_write(ADCRSTENDCT0, 0x000005);
	AFE4490_write(ADCRSTSTCT1, 0x0007D0);
	AFE4490_write(ADCRSTENDCT1, 0x0007D5);
	AFE4490_write(ADCRSTSTCT2, 0x000FA0);
	AFE4490_write(ADCRSTENDCT2, 0x000FA5);
	AFE4490_write(ADCRSTSTCT3, 0x001770);
	AFE4490_write(ADCRSTENDCT3, 0x001775);

	// LED2 timing Pulses & Samples
	AFE4490_write(LED2STC, 0x0017C0);
	AFE4490_write(LED2ENDC, 0x001F3E);
	AFE4490_write(LED2LEDSTC, 0x001770);
	AFE4490_write(LED2LEDENDC, 0x001F3F);
	AFE4490_write(ALED2STC, 0x000050);
	AFE4490_write(ALED2ENDC, 0x0007CE);

	// LED1 timing Pulses & Samples
	AFE4490_write(LED1STC, 0x000820);
	AFE4490_write(LED1ENDC, 0x000F9E);
	AFE4490_write(LED1LEDSTC, 0x0007D0);
	AFE4490_write(LED1LEDENDC, 0x000F9F);
	AFE4490_write(ALED1STC, 0x000FF0);
	AFE4490_write(ALED1ENDC, 0x00176E);


	// CONTROL1 reg : Enable Timer, Average 1 ADC Sample
	AFE4490_write(CONTROL1, 0x000107);

	// Receiver Channel setting
	AFE4490_write(TIAGAIN, 0x000000); // RF, CF and Stage 2 gain settings are same for both LED1 and LED2
	AFE4490_write(TIA_AMB_GAIN, 0x000005); // Set CF = 5pF, RF = 10kOhm, By Stage 2 Gain

	// LEDCNTRL register, set max current to 75 mA
	// ILED1 = ILED2 = 5.859 mA
	// LED1 = LED2 = 20
	AFE4490_write(LEDCNTRL, 0x011414);

	// CONTROL2 register configuration
	AFE4490_write(CONTROL2, 0x000000);



}

void AFE4490_registersInit_62k5()
{
	char uart_buf[50];
	unsigned int uart_buf_len;

	uart_buf_len = sprintf(uart_buf, "AFE4490 registers initialization ...\r\n");
	HAL_UART_Transmit(&huart2, uart_buf, uart_buf_len, HAL_MAX_DELAY);

	// Enable Write Mode
	AFE4490_readEnable(0);

	// Timer Module Configuration
	// PRPCOUNT Configuration, Set PRF = 500 Hz --> PRPCOUNT = 7999 = 0x1F3F
	    AFE4490_write(CONTROL1, 0x000101);
	    AFE4490_write(CONTROL2, 0x000000);
	    set_PRPCOUNT_frequency(Frequency);
	    //AFE4490_write(PRPCOUNT, 0X001F3F);

	    AFE4490_write(LED2STC, 0X001770); //timer control
	    AFE4490_write(LED2ENDC,0X001F3E); //timer control
	    AFE4490_write(LED2LEDSTC,0X001770); //timer control
	    AFE4490_write(LED2LEDENDC,0X001F3F); //timer control
	    AFE4490_write(ALED2STC, 0X000000); //timer control
	    AFE4490_write(ALED2ENDC, 0X0007CE); //timer control
	    AFE4490_write(LED2CONVST,0X000002); //timer control
	    AFE4490_write(LED2CONVEND, 0X0007CF); //timer control
	    AFE4490_write(ALED2CONVST, 0X0007D2); //timer control
	    AFE4490_write(ALED2CONVEND,0X000F9F); //timer control

	    AFE4490_write(LED1STC, 0X0007D0); //timer control
	    AFE4490_write(LED1ENDC, 0X000F9E); //timer control
	    AFE4490_write(LED1LEDSTC, 0X0007D0); //timer control
	    AFE4490_write(LED1LEDENDC, 0X000F9F); //timer control
	    AFE4490_write(ALED1STC, 0X000FA0); //timer control
	    AFE4490_write(ALED1ENDC, 0X00176E); //timer control
	    AFE4490_write(LED1CONVST, 0X000FA2); //timer control
	    AFE4490_write(LED1CONVEND, 0X00176F); //timer control
	    AFE4490_write(ALED1CONVST, 0X001772); //timer control
	    AFE4490_write(ALED1CONVEND, 0X001F3F); //timer control

	    AFE4490_write(ADCRSTSTCT0, 0X000000); //timer control
	    AFE4490_write(ADCRSTENDCT0,0X000000); //timer control
	    AFE4490_write(ADCRSTSTCT1, 0X0007D0); //timer control
	    AFE4490_write(ADCRSTENDCT1, 0X0007D0); //timer control
	    AFE4490_write(ADCRSTSTCT2, 0X000FA0); //timer control
	    AFE4490_write(ADCRSTENDCT2, 0X000FA0); //timer control
	    AFE4490_write(ADCRSTSTCT3, 0X001770); //timer control
	    AFE4490_write(ADCRSTENDCT3, 0X001770);

	    set_LED_currents(IR_current,Red_current);

	    set_TIAGAIN_LED1(Resistance, Capacity, Gain);	// CF = 5pF, RF = 500kR
	    AFE4490_write(TIA_AMB_GAIN,0x000005);	// Timers ON, average 3 samples
	    //AFE4490_write(LEDCNTRL,0x0011414);	// Switch to READ mode
	    AFE4490_write(CONTROL2,0x000000);	// LED_RANGE=100mA, LED=50mA
	    AFE4490_write(CONTROL1,0x010707);	// Timers ON, average 3 samples
	    AFE4490_write(CONTROL0,0x000001);



}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (stop_requested) {
		HAL_NVIC_DisableIRQ(EXTI1_IRQn);
	    return;
	}
	e++;
	REDdata = AFE4490_read(LED2VAL);
	voltage_REDdata = (scale_factor * (float)REDdata) / pow(2,21);
	// AREDdata = AFE4490_read(ALED2VAL);
	IRdata = AFE4490_read(LED1VAL);
	voltage_IRdata = (scale_factor * (float)IRdata) / pow(2,21);


	// uart_buf_len = sprintf(uart_buf, "RED: %d\tIR: %d\r\n", REDdata, IRdata);
	uart_buf_len = sprintf(uart_buf, "IRdata: %f, REDdata: %f\r\n", voltage_IRdata, voltage_REDdata);
	HAL_UART_Transmit(&huart2, uart_buf, uart_buf_len, HAL_MAX_DELAY);
	c=1;
	HAL_StatusTypeDef status1 = HAL_UART_Receive(&huart2, rx_data, 16, 0); // Passer 0 pour un délai non bloquant
	if (rx_data[0]== 's'){
		stop_requested = 1;
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
#endif /* USE_FULL_ASSERT */
