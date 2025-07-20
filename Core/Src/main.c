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

/* USER CODE BEGIN PFP */
void gpio_init();
void USART1_init(void);
char USART1_read(void);
void motor_forward(void);
void motor_stop(void);
void motor_backward(void);
void motor_left(void);
void motor_right(void);
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
USART1_init();
  /* USER CODE END Init */

  /* Configure the system clock */
  

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */
gpio_init();


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  char C = USART1_read();
	  if (C == 'U'){
		  motor_forward();
		  GPIOC->ODR = (0X01 << 13);  // Turn on PC13
	  }
	  else if (C == 'D') {
	          motor_backward();
			  GPIOC->ODR = (0X01 << 13);  // Turn on PC13

	      }
	  else if (C == 'L') {
		  motor_left();
		  GPIOC->ODR = (0X01 << 13);  // Turn on PC13

	  }
	  else if (C == 'R') {
		  motor_right();
		  GPIOC->ODR = (0X01 << 13);  // Turn on PC13

	  }
	  else if (C == 'S'){
	  		  motor_stop();
	  		  GPIOC->ODR = (0X0 << 13);  // Turn off PC13

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
void motor_backward(void)
{
    // Left motor backward: IN1 = LOW, IN2 = HIGH
    GPIOA->BSRR = (1U << (1 + 16));
    GPIOA->BSRR = (1U << 2);

    // Right motor backward: IN3 = LOW, IN4 = HIGH
    GPIOA->BSRR = (1U << (3 + 16));
    GPIOA->BSRR = (1U << 4);
}

void motor_left(void)
{
    // Left motor stop: IN1 = LOW, IN2 = LOW
    GPIOA->BSRR = (1U << (1 + 16));
    GPIOA->BSRR = (1U << (2 + 16));

    // Right motor forward: IN3 = HIGH, IN4 = LOW
    GPIOA->BSRR = (1U << 3);
    GPIOA->BSRR = (1U << (4 + 16));
}
void motor_right(void)
{
    // Left motor forward: IN1 = HIGH, IN2 = LOW
    GPIOA->BSRR = (1U << 1);
    GPIOA->BSRR = (1U << (2 + 16));

    // Right motor stop: IN3 = LOW, IN4 = LOW
    GPIOA->BSRR = (1U << (3 + 16));
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
