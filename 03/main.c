/* USER CODE BEGIN Header */
/**
    ******************************************************************************
    * @file                     : main.c
    * @brief                    : Main program body
    ******************************************************************************
    * @attention
    *
    * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
    * All rights reserved.</center></h2>
    *
    * This software component is licensed by ST under BSD 3-Clause license,
    * the "License"; You may not use this file except in compliance with the
    * License. You may obtain a copy of the License at:
    *                                                opensource.org/licenses/BSD-3-Clause
    *
    ******************************************************************************
    */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/

/* USER CODE BEGIN PTD */

// a struct that defines a GPIO device (maps GPIOs memory structure)
typedef struct
{
    uint32_t TODO
    uint32_t TODO
    uint32_t TODO
    uint32_t TODO
    uint32_t TODO
    uint32_t TODO
    uint16_t TODO
    uint16_t TODO
} GPIO_device; 

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/

/* USER CODE BEGIN PD */

// constants for storing addresses of GPIOs
#define GPIOAd TODO
#define GPIOBd TODO
#define GPIOCd TODO
#define GPIODd TODO
#define GPIOEd TODO
#define GPIOFd TODO
#define GPIOGd TODO
#define GPIOHd TODO
#define GPIOId TODO

// MODE constants
#define IN TODO
#define OUT TODO
#define AF TODO
#define ANALOG TODO

// PUPD constants
#define NO_PULL TODO
#define PULL_UP TODO
#define PULL_DOWN TODO

// OTYPE constants
#define PUSH_PULL TODO
#define OPEN_DRAIN TODO

// OSPEED constants
#define S2MHz TODO
#define S25MHz TODO
#define S50MHz TODO
#define S100MHz TODO

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

void ClockOn(GPIO_device * GPIO_addr);
void init_GPIO(GPIO_device * GPIO_addr, uint32_t Pin, uint32_t Mode, uint32_t PUPD, uint32_t OType, uint32_t OSpeed);
void GPIO_Output(GPIO_device * GPIO_addr, uint32_t Pin, uint32_t val);
uint32_t GPIO_Read(GPIO_device * GPIO_addr, uint32_t Pin);
void delay(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
    * @brief    The application entry point.
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
    /* USER CODE BEGIN 2 */

    // turn on button's GPIO (button is on PA0)
    TODO

    // turn on led's GPIO (leds are on PD12, PD13, PD14, PD15)
    TODO

    // init button
    TODO

    // init leds
    TODO

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        // read button and trigger led sequence
        TODO

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

    /** Configure the main internal regulator output voltage 
    */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
    /** Initializes the CPU, AHB and APB busses clocks 
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }
    /** Initializes the CPU, AHB and APB busses clocks 
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

// function for turning on a particular GPIO (enables the port clock)
void ClockOn(GPIO_device * GPIO_addr)
{
    TODO
}

// function for initializing GPIOs
void init_GPIO(GPIO_device * GPIO_addr, uint32_t Pin, uint32_t Mode, uint32_t PUPD, uint32_t OType, uint32_t OSpeed)
{
    TODO
}

// function for setting the value of an output GPIO
void GPIO_Output(GPIO_device * GPIO_addr, uint32_t Pin, uint32_t val) 
{
    TODO
}

// function for reading the value of an input GPIO
uint32_t GPIO_Read(GPIO_device * GPIO_addr, uint32_t Pin)
{
    TODO
}

// hardcoded delay
void delay(void) {
    volatile int d = 500000;
    while(d--);
}

/* USER CODE END 4 */

/**
    * @brief    This function is executed in case of error occurrence.
    * @retval None
    */
void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */

    /* USER CODE END Error_Handler_Debug */
}

#ifdef    USE_FULL_ASSERT
/**
    * @brief    Reports the name of the source file and the source line number
    *                 where the assert_param error has occurred.
    * @param    file: pointer to the source file name
    * @param    line: assert_param error line source number
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
