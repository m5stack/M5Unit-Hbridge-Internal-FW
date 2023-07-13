/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "i2c_ex.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADC_CHANNEL_NUMS                2
#define ADC_SAMPLES_NUMS                ADC_CHANNEL_NUMS * 20

#define I2C1_ADDR_BASE 0x20
#define FIRMWARE_VERSION 2

enum {DIR_STOP = 0, DIR_FORWARD, DIR_REWERSE};
enum {SPEED_TYPE_8BIT = 0, SPEED_TYPE_16BIT};
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;

TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
volatile uint8_t direction = DIR_STOP;
volatile uint8_t speed_type_flag = SPEED_TYPE_8BIT;
volatile uint8_t speed_8bits = 0;
volatile uint16_t speed_16bits = 0;
volatile uint16_t pwm_freq = 1000;

volatile uint8_t g_i2c_addr = 0;
volatile uint8_t g_i2c_addr_last = 0;
volatile uint8_t g_i2c_addr_filter = 0;

float i_in_value = 0.0;

__IO uint32_t uiAdcValueBuf[ADC_SAMPLES_NUMS];
__IO uint16_t usAdcValue16[ADC_CHANNEL_NUMS];
__IO uint8_t usAdcValue8[ADC_CHANNEL_NUMS];

volatile uint8_t fm_version = FIRMWARE_VERSION;

#define APPLICATION_ADDRESS     ((uint32_t)0x08001000)
volatile uint8_t flag_jump_bootloader = 0;
volatile uint32_t jump_bootloader_timeout = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void IAP_Set()
{
	uint8_t i;
 
	uint32_t *pVecTab=(uint32_t *)(0x20000000);
	//????????SRAM???
	for(i = 0; i < 48; i++)
	{
		*(pVecTab++) = *(__IO uint32_t*)(APPLICATION_ADDRESS + (i<<2));
	}
  /* Enable the SYSCFG peripheral clock*/
#if 1 //STM32
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  //??? SRAM ??? 0x00000000
  __HAL_SYSCFG_REMAPMEMORY_SRAM();
#else //AMP32
    RCM_EnableAPB2PeriphClock(RCM_APB2_PERIPH_SYSCFG);
    /* Remap SRAM at 0x00000000 */
    SYSCFG->CFG1_B.MMSEL = SYSCFG_MemoryRemap_SRAM;
#endif
}

void i2c_port_set_to_input(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9 | GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = (GPIO_PIN_9 | GPIO_PIN_10);
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

long map(long x, long in_min, long in_max, long out_min, long out_max) {
  long result;

  result = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  if (result < out_min)
    result = out_min;
  else if (result > out_max)
    result = out_max;
  
  return result;
}

void set_pwm(uint8_t direction)
{
  uint32_t period;

  period = 1000000 / pwm_freq;
  if (!pwm_freq)
    __HAL_TIM_SET_AUTORELOAD(&htim3, 1000000 / 100);
  else
    __HAL_TIM_SET_AUTORELOAD(&htim3, period);  
  switch (direction)
  {
  case DIR_STOP:
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
    break;
  case DIR_FORWARD:
    if (speed_type_flag == SPEED_TYPE_8BIT) {
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (uint32_t)((float)speed_8bits/255.0f*period));
    } else {
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (uint32_t)((float)speed_16bits/65535.0f*period));
    }
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
    break;
  case DIR_REWERSE:
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
    if (speed_type_flag == SPEED_TYPE_8BIT) {
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, (uint32_t)((float)speed_8bits/255.0f*period));
    } else {
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, (uint32_t)((float)speed_16bits/65535.0f*period));
    }
    break;
  
  default:
    break;
  }   
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
  uint64_t adcTotal[ADC_CHANNEL_NUMS]={0};
  float vout = 0.0;
  
	HAL_ADC_Stop_DMA(hadc);
	for (uint8_t i = 0; i < ADC_SAMPLES_NUMS; i++) {
    adcTotal[i%ADC_CHANNEL_NUMS] += uiAdcValueBuf[i];
  }
	for (uint8_t i = 0; i < ADC_CHANNEL_NUMS; i++) {
    usAdcValue16[i] = (adcTotal[i] / 20);
  }

	usAdcValue8[0] = map(usAdcValue16[0],0,4095,0,255); 

  vout = usAdcValue16[1] * 3.3 / 4095;

  i_in_value = (vout - 1.24) / 0.5;  

	HAL_ADC_Start_DMA(hadc, (uint32_t *)uiAdcValueBuf, ADC_SAMPLES_NUMS);
}

void user_i2c1_init(void)
{


  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  LL_I2C_InitTypeDef I2C_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  /**I2C1 GPIO Configuration
  PA9   ------> I2C1_SCL
  PA10   ------> I2C1_SDA
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_9;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_4;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_10;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_4;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C1);

  /* I2C1 interrupt Init */
  NVIC_SetPriority(I2C1_IRQn, 0);
  NVIC_EnableIRQ(I2C1_IRQn);

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */

  /** I2C Initialization
  */
  LL_I2C_DisableOwnAddress2(I2C1);
  LL_I2C_DisableGeneralCall(I2C1);
  LL_I2C_EnableClockStretching(I2C1);
  I2C_InitStruct.PeripheralMode = LL_I2C_MODE_I2C;
  I2C_InitStruct.Timing = 0x0000020B;
  I2C_InitStruct.AnalogFilter = LL_I2C_ANALOGFILTER_ENABLE;
  I2C_InitStruct.DigitalFilter = 0;
  I2C_InitStruct.OwnAddress1 = (I2C1_ADDR_BASE+g_i2c_addr)<<1;
  I2C_InitStruct.TypeAcknowledge = LL_I2C_ACK;
  I2C_InitStruct.OwnAddrSize = LL_I2C_OWNADDRESS1_7BIT;
  LL_I2C_Init(I2C1, &I2C_InitStruct);
  LL_I2C_EnableAutoEndMode(I2C1);
  LL_I2C_SetOwnAddress2(I2C1, 0, LL_I2C_OWNADDRESS2_NOMASK);
  /* USER CODE BEGIN I2C1_Init 2 */
  set_i2c_slave_address(I2C1_ADDR_BASE+g_i2c_addr);
  /* USER CODE END I2C1_Init 2 */

}

uint8_t read_i2c1_addr(void)
{
  uint8_t i2c1_addr = 0;

  for (int i = 0; i < 4; i++) {
    i2c1_addr |= (!(HAL_GPIO_ReadPin(GPIOA, 1 << i))) << i;
  }
  
  return i2c1_addr;
}

void i2c1_addr_init(void)
{
  uint8_t i2c1_addr = 0;

  for (int i = 0; i < 10; i++) {
    i2c1_addr = read_i2c1_addr();
    if (i2c1_addr != g_i2c_addr_last) {
      g_i2c_addr_last = i2c1_addr;
      g_i2c_addr_filter = 0;

    } else {
      g_i2c_addr_filter++;
      if (g_i2c_addr_filter >= 3) {
        g_i2c_addr_filter = 0;
        g_i2c_addr = i2c1_addr;
        user_i2c1_init();
        return;
      }
    }
    HAL_Delay(100);
  }
  MX_I2C1_Init();
}

void Slave_Complete_Callback(uint8_t *rx_data, uint16_t len) 
{
  uint8_t rx_buf[16];
  uint8_t tx_buf[16];
  uint8_t rx_mark[16] = {0};

  if (len > 1 && (rx_data[0] <= 0x05))
  {
    if (len <= 7) {
      for (int i = 0; i < len - 1; i++) {
        rx_buf[rx_data[0]+i] = rx_data[1+i];
        rx_mark[rx_data[0]+i] = 1; 
      }
      if (rx_mark[0]) {
        direction = rx_buf[0];
      }
      if (rx_mark[1]) {
        speed_type_flag = SPEED_TYPE_8BIT;
        speed_8bits = rx_buf[1];
      }
      if (rx_mark[2] && rx_mark[3]) {
        speed_type_flag = SPEED_TYPE_16BIT;
        speed_16bits = (rx_buf[2] | (rx_buf[3] << 8));
      }
      if (rx_mark[4] && rx_mark[5]) {
        pwm_freq = (rx_buf[4] | (rx_buf[5] << 8));
        if (pwm_freq < 100)
          pwm_freq = 100;
      }
			set_pwm(direction);
				
    }
  }
  else if (len == 1 && (rx_data[0] <= 0x05))
  {
    if (rx_data[0] == 0x00) {
      i2c1_set_send_data((uint8_t *)&direction, 1);
    } else if (rx_data[0] == 0x01) {
      i2c1_set_send_data((uint8_t *)&speed_8bits, 1);
    } else if ((rx_data[0] == 0x02) || (rx_data[0] == 0x03)) {
      i2c1_set_send_data((uint8_t *)&speed_16bits, 2);
    } else if ((rx_data[0] == 0x04) || (rx_data[0] == 0x05)) {
      i2c1_set_send_data((uint8_t *)&pwm_freq, 2);
    }
  }  
  else if (len == 1 && (rx_data[0] == 0x10))
  {
    i2c1_set_send_data(usAdcValue8, 1);
  }  
  else if (len == 1 && ((rx_data[0] == 0x20) || (rx_data[0] == 0x21)))
  {
    i2c1_set_send_data((uint8_t *)&usAdcValue16[0], 2);
  }  
  else if (len == 1 && (rx_data[0] == 0xFE))
  {
    i2c1_set_send_data((uint8_t *)&fm_version, 1);
  }  
  else if (len == 1 && (rx_data[0] == 0xFF))
  {
    tx_buf[0] = I2C1_ADDR_BASE + g_i2c_addr;
    i2c1_set_send_data((uint8_t *)&tx_buf[0], 1);
  }  
	else if (len == 1 && (rx_data[0] >= 0x30 & rx_data[0] <= 0x33)) 
	{
    i2c1_set_send_data((uint8_t *)&i_in_value, 4);
  }     
  else if (len > 1 && (rx_data[0] == 0xFD)) 
  {
    if (rx_data[1] == 1) {
      flag_jump_bootloader = 1;
      if (flag_jump_bootloader) {
        LL_I2C_DeInit(I2C1);
        LL_I2C_DisableAutoEndMode(I2C1);
        LL_I2C_Disable(I2C1);
        LL_I2C_DisableIT_ADDR(I2C1);
        HAL_TIM_PWM_DeInit(&htim3);
        HAL_ADC_DeInit(&hadc);
        __HAL_RCC_DMA1_CLK_DISABLE();
        HAL_NVIC_DisableIRQ(DMA1_Channel1_IRQn);
        HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1); 
        HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2); 
        HAL_ADC_Stop_DMA(&hadc);
        i2c_port_set_to_input();
        while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9) || HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10))
        {
          jump_bootloader_timeout++;
          if (jump_bootloader_timeout >= 60000) {
            flag_jump_bootloader = 0;
            break;
          }
        }
        if (jump_bootloader_timeout < 60000) {
          NVIC_SystemReset();
        } else {
          // NVIC_SystemReset();
          user_i2c1_init();
          i2c1_it_enable();
          MX_DMA_Init();
          MX_ADC_Init();
          MX_TIM3_Init();          
          HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); 
          HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2); 
          HAL_ADC_Start_DMA(&hadc, (uint32_t *)uiAdcValueBuf, ADC_SAMPLES_NUMS);  
          jump_bootloader_timeout = 0;
        }
      }        
    }
  }   
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  IAP_Set();
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
  // MX_I2C1_Init();
  MX_ADC_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  i2c1_addr_init();
  HAL_ADCEx_Calibration_Start(&hadc);
  HAL_ADC_Start_DMA(&hadc, (uint32_t *)uiAdcValueBuf, ADC_SAMPLES_NUMS);  
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); 
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2); 
  i2c1_it_enable();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_0);
  while(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_0)
  {
  }
  LL_RCC_HSI_Enable();

   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {

  }
  LL_RCC_HSI_SetCalibTrimming(16);
  LL_RCC_HSI14_Enable();

   /* Wait till HSI14 is ready */
  while(LL_RCC_HSI14_IsReady() != 1)
  {

  }
  LL_RCC_HSI14_SetCalibTrimming(16);
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSI);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSI)
  {

  }
  LL_SetSystemCoreClock(8000000);

   /* Update the time base */
  if (HAL_InitTick (TICK_INT_PRIORITY) != HAL_OK)
  {
    Error_Handler();
  }
  LL_RCC_HSI14_EnableADCControl();
  LL_RCC_SetI2CClockSource(LL_RCC_I2C1_CLKSOURCE_HSI);
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = ENABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = ENABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

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

  LL_I2C_InitTypeDef I2C_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  /**I2C1 GPIO Configuration
  PA9   ------> I2C1_SCL
  PA10   ------> I2C1_SDA
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_9;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_4;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_10;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_4;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C1);

  /* I2C1 interrupt Init */
  NVIC_SetPriority(I2C1_IRQn, 0);
  NVIC_EnableIRQ(I2C1_IRQn);

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */

  /** I2C Initialization
  */
  LL_I2C_DisableOwnAddress2(I2C1);
  LL_I2C_DisableGeneralCall(I2C1);
  LL_I2C_EnableClockStretching(I2C1);
  I2C_InitStruct.PeripheralMode = LL_I2C_MODE_I2C;
  I2C_InitStruct.Timing = 0x0000020B;
  I2C_InitStruct.AnalogFilter = LL_I2C_ANALOGFILTER_ENABLE;
  I2C_InitStruct.DigitalFilter = 0;
  I2C_InitStruct.OwnAddress1 = 0x20<<1;
  I2C_InitStruct.TypeAcknowledge = LL_I2C_ACK;
  I2C_InitStruct.OwnAddrSize = LL_I2C_OWNADDRESS1_7BIT;
  LL_I2C_Init(I2C1, &I2C_InitStruct);
  LL_I2C_EnableAutoEndMode(I2C1);
  LL_I2C_SetOwnAddress2(I2C1, 0, LL_I2C_OWNADDRESS2_NOMASK);
  /* USER CODE BEGIN I2C1_Init 2 */
  set_i2c_slave_address(0x20);
  /* USER CODE END I2C1_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 8-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pins : PA0 PA1 PA2 PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
