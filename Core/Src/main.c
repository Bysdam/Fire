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
#include "stm32f4xx.h"
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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void gpio_init();
void USART1_init(void);
char USART1_read(void);
void motor_forward(void);
void motor_stop(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint32_t tick;
uint32_t delay = 300;

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
USART1_init();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */
gpio_init();
tick = HAL_GetTick();
//motor_forward();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  if ((HAL_GetTick()-tick) >= delay){
	  GPIOC->ODR ^= (0X01 << 13);  // Toggle PC13

	  tick = HAL_GetTick();}
	  char C = USART1_read();
	  if (C == 'U'){
		  motor_forward();
	  }
	  else if (C == 'S'){
	  		  motor_stop();
	  	  }

	  // Wait ~300 ms
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

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

/* USER CODE BEGIN 4 */
void gpio_init()
{
	RCC ->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
	RCC ->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
// enabling the PC13 to toggle to signal that the while loop is alive
	GPIOC -> MODER &= ~(0b11 << 26);
	GPIOC -> MODER |= 0x01 << 26;

// Configuring the motor direction pins
// CLEAR and set PA1 PA2 PA3 and PA4 pins to output
	// IN1 -> PA1
	GPIOA->MODER &= ~(3U << (1 * 2));
	GPIOA->MODER |=  (1U << (1 * 2));
	GPIOA->PUPDR &= ~(0x3U << (1 * 2));
	GPIOA->OTYPER &= ~(1U << 1);
	// IN2 -> PA2
	GPIOA->MODER &= ~(3U << (2 * 2));
	GPIOA->MODER |=  (1U << (2 * 2));
	GPIOA->PUPDR &= ~(0x3U << (2 * 2));
	GPIOA->OTYPER &= ~(1U << 2);
	// IN3 -> PA3
	GPIOA->MODER &= ~(3U << (3 * 2));
	GPIOA->MODER |=  (1U << (3 * 2));
	GPIOA->PUPDR &= ~(0x3U << (3 * 2));
	GPIOA->OTYPER &= ~(1U << 3);
	// IN4 -> PA4
	GPIOA->MODER &= ~(3U << (4 * 2));
	GPIOA->MODER |=  (1U << (4 * 2));
	GPIOA->PUPDR &= ~(0x3U << (4 * 2));
	GPIOA->OTYPER &= ~(1U << 4);
	}

void motor_forward(void)
{
    // IN1 = HIGH (PA1)
    GPIOA->BSRR = (1U << 1);
    // IN2 = LOW (PA2)
    GPIOA->BSRR = (1U << (2 + 16));
    // IN3 = HIGH (PA3)
    GPIOA->BSRR = (1U << 3);
    // IN4 = LOW (PA4)
    GPIOA->BSRR = (1U << (4 + 16));
}

void motor_stop(void)
{
    // IN1 = LOW (PA1)
    GPIOA->BSRR = (1U << (1 + 16));
    // IN2 = LOW (PA2)
    GPIOA->BSRR = (1U << (2 + 16));
    // IN3 = LOW (PA3)
    GPIOA->BSRR = (1U << (3 + 16));
    // IN4 = LOW (PA4)
    GPIOA->BSRR = (1U << (4 + 16));
}

void EN_USART1CLK(void){
	RCC ->AHB1ENR |= 0x1;
	RCC ->APB2ENR |= 0X10;
}

void USART1_ConfigPins(void){
	// Set MODER FOR PA9 AND PA10 to be alternative functions
	GPIOA -> MODER &= ~(0xF << (2*9));
	GPIOA -> MODER |= (0xA << (2*9));
	// set theier values to be AF7
	// PA9
	GPIOA -> AFR[1] &= ~(0xF << (4*1));
	GPIOA -> AFR[1] |= (0x7 << (4*1));

	// PA10
	GPIOA -> AFR[1] &= ~(0xF << (4*2));
	GPIOA -> AFR[1] |= (0x7 << (4*2));

}
void USART1_SetBaudRate(void){
	// to calcualte (16MHz/ (16 * 9600) = 104. 166 = 0X683)
	USART1 -> BRR = 0x683;
}

void USART1_ConfigSettings(void){
	// enable RX and TX and EU and oversampling 16
	USART1 -> CR1 = 0x200C;
}

void USART1_Send(int ch){
	while ( !(USART1 ->SR & 0x80) ){}
	USART1 -> DR = ch & 0xFF;
}

char USART1_read(void){
	while( !(USART1 ->SR & 0x20)){}
	return USART1 -> DR;
}

void USART1_init(void){
	EN_USART1CLK();
	USART1_ConfigPins();
	USART1_SetBaudRate();
	USART1_ConfigSettings();

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
