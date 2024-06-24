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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include "sys.h"

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
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define NVIC_VectTab_RAM             ((uint32_t)0x20000000)
#define NVIC_VectTab_FLASH           ((uint32_t)0x08000000)				//NVIC_VectTab_FLASH定义的是flash的起始地址；
#define BOOT_SIZE							0x5000						//正好对应着第一个扇区开始擦除。
#define ApplicationAddress	0x08005000

#define USER_FLASH_PAGES	2      //user app flash is 52 PAGE      两个扇区 


#define FLASH_USER_END_ADDR    ADDR_FLASH_SECTOR_3

#define	erasure_CMD	"era flash"

typedef  void (*iapfun)(void);
iapfun jump2app;

unsigned int count2 = 0;
unsigned char datatemp[256] = {0};
unsigned char boot_flag = 0;
unsigned char time_out_flag = 0;
extern uint8_t ch;


#define ADDR_FLASH_SECTOR_0     ((uint32_t)0x08000000) /* Base address of Sector 0, 16 Kbytes   */
#define ADDR_FLASH_SECTOR_1     ((uint32_t)0x08004000) /* Base address of Sector 1, 16 Kbytes   */
#define ADDR_FLASH_SECTOR_2     ((uint32_t)0x08008000) /* Base address of Sector 2, 16 Kbytes   */
#define ADDR_FLASH_SECTOR_3     ((uint32_t)0x0800C000) /* Base address of Sector 3, 16 Kbytes   */
#define ADDR_FLASH_SECTOR_4     ((uint32_t)0x08010000) /* Base address of Sector 4, 64 Kbytes   */
#define ADDR_FLASH_SECTOR_5     ((uint32_t)0x08020000) /* Base address of Sector 5, 128 Kbytes  */
#define ADDR_FLASH_SECTOR_6     ((uint32_t)0x08040000) /* Base address of Sector 6, 128 Kbytes  */
#define ADDR_FLASH_SECTOR_7     ((uint32_t)0x08060000) /* Base address of Sector 7, 128 Kbytes  */


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
    /* USER CODE BEGIN 1 */
	unsigned char i;

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
    MX_USART6_UART_Init();
    /* USER CODE BEGIN 2 */
    HAL_GPIO_WritePin(POWER_CTR_GPIO_Port,POWER_CTR_Pin,GPIO_PIN_SET);//上电把电源的控制角拉高。
    printf("boot start\r\n");
    printf("input \"era flash\" to erasure user flash, or wait 10s to start user app\r\n");
    HAL_Delay(500);//延迟个500Ms进入freertos系统中。
	
	for(i = 0; i<10; i++)
    {
        HAL_UART_Receive(&huart6, datatemp, 256, 1000);
//		WRITE_REG(huart6.Instance->DR,datatemp);
//		HAL_UART_Transmit(&huart6, datatemp, 256, 1000);

        // every second to find if receive erasure_CMD

        if(strstr((const char *)datatemp, erasure_CMD) != NULL)
        {
			printf("%s",datatemp);
			printf("准备擦除芯片的内部flash中。。。");
            // erase flash
            FLASH_EraseInitTypeDef EraseInitStruct;
            unsigned int PageError = 0;

            HAL_FLASH_Unlock();

            EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
            EraseInitStruct.Sector = 1;//开始的扇区
            EraseInitStruct.NbSectors = USER_FLASH_PAGES;//几个扇区
            EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
            if(HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) != HAL_OK)
            {
                HAL_FLASH_Lock();
                printf("Erase fail at:0x%x\n\r",PageError);
//                return 0;
            }
            HAL_FLASH_Lock();
            boot_flag = 1;
            printf("Erase OK\n\r");
            break;
        }
    }

    if(boot_flag == 1)
    {
        HAL_StatusTypeDef	temp;
        unsigned int Address;
        unsigned int data_32;
        unsigned char j = 0;
        printf("ready to receive bin, please send in 30s\n\r");
        Address = ApplicationAddress;
        temp = HAL_UART_Receive(&huart6, datatemp, 256, 30*1000);
        if(temp == HAL_TIMEOUT)
        {
            printf("time out, end wait to receive bin\n\r");
//            return 0;
        }
        else if(temp == HAL_OK)
        {
            while(1) // wait data and write, if time out tow times, break;
            {
                unsigned char i;
                HAL_FLASH_Unlock();
                for(i=0; i<64; i++)
                {
                    data_32 = *(unsigned int *)(&datatemp[i<<2]);
                    if(Address < FLASH_USER_END_ADDR)
                    {
                        if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, Address, data_32)==HAL_OK)
                        {
                            Address = Address + 4;
                        }
                        else
                        {
                            HAL_FLASH_Lock();
                            printf("write fail at: 0x%x\n\r", Address);
//                            return 0;
                        }
                    }
                }
                HAL_FLASH_Lock();
                printf("write 256 btye OK: 0x%x\t%d\n\r",Address,j++);

                temp = HAL_UART_Receive(&huart6, datatemp, 256, 2*1000);
                if(temp == HAL_TIMEOUT)
                {
                    time_out_flag++;
                    if(time_out_flag == 2)
                    {
                        printf("End write OK\n\r");
//						HAL_NVIC_SystemReset();
                        goto START_APP;
						
                        //return 0;
                    }
                }
            }
        }
    }

    else if(boot_flag == 0)  //start user app
    {
		START_APP:
        printf("start user app\n\r");
//        HAL_Delay(10);

        if(((*(unsigned int *)ApplicationAddress)&0x2FFE0000)==0x20000000)
        {
//            __set_PRIMASK(1);
//            __ASM("CPSID  I"); // disable irq, if use this, must enable irq at app
////            __disable_irq();
////			SysTick->CTRL = 0X00;//禁止SysTick
////			SysTick->LOAD = 0;
////			SysTick->VAL = 0;
//			__disable_irq();
			
//			__set_PRIMASK(1);    /*关闭全局中断，APP程序中带有RTOS时，这个很重要*/
 
			/* 关闭滴答时钟，复位*/
//			SysTick->CTRL = 0;
//			SysTick->LOAD = 0;
//			SysTick->VAL = 0;
//			HAL_SuspendTick();        // 挂起滴答定时器
			/* 设置所以时钟默认状态 */
//			HAL_RCC_DeInit();

			/* 关闭所有中断，清除中断挂起标志 */
//			for (i = 0; i < 8; i++)
//			{
//				NVIC->ICER[i]=0xFFFFFFFF;
//				NVIC->ICPR[i]=0xFFFFFFFF;
//			}    
//			HAL_UART_DeInit(&huart1);
//			HAL_UART_DeInit(&huart6);
//			HAL_SuspendTick();
			
			
//			HAL_RCC_DeInit();
//			__set_FAULTMASK(1);
//			INTX_DISABLE();
	   /* 在 RTOS 工程，这条语句很重要，设置为特权级模式，使用 MSP 指针 */
//			__set_CONTROL(0);


			
			
			
//            jump2app=(iapfun)*(unsigned int *)(ApplicationAddress+4);
            // user app addr's second word store the app start addr
//            __set_MSP(*(unsigned int *)ApplicationAddress);
            //user app addr's first word store the SP addr
//			printf("开始跳转APP\n\r");
//			__disable_irq();	

			

			
//            jump2app();									// run user app
//			printf("跳转完成\n\r");


//			SysTick->CTRL = 0X00;//禁止SysTick
//			SysTick->LOAD = 0;
//			SysTick->VAL = 0;
//			__disable_irq();


				uint32_t i=0;
				void (*SysMemBootJump)(void);        /* 声明一个函数指针 */
				__IO uint32_t BootAddr = 0x08005000;  /* APP 地址 */

//				__set_PRIMASK(1);	/* 禁止全局中断 */

				/* 关闭滴答定时器，复位到默认值 */
				SysTick->CTRL = 0;
				SysTick->LOAD = 0;
				SysTick->VAL = 0;
				/* 设置所有时钟到默认状态，使用HSI时钟 */
				HAL_RCC_DeInit();//关闭时钟这一步不能省掉。
				HAL_UART_DeInit(&huart6);//还需要关闭所有的外设。
//				__HAL_UART_DISABLE_IT(&huart6,UART_IT_RXNE);
//				HAL_GPIO_DeInit(LED0_GPIO_Port,LED0_Pin);//这些GOIO，可以不用关掉。
//				HAL_GPIO_DeInit(LED1_GPIO_Port,LED1_Pin);
//				HAL_GPIO_DeInit(LED2_GPIO_Port,LED2_Pin);
//				HAL_GPIO_DeInit(LED3_GPIO_Port,LED3_Pin);
//				HAL_GPIO_DeInit(POWER_CTR_GPIO_Port,POWER_CTR_Pin);
//				HAL_GPIO_DeInit(POWER_ON_OFF_GPIO_Port,POWER_ON_OFF_Pin);
//				HAL_TIM_Base_DeInit(&htim4);

				/* 关闭所有中断，清除所有中断挂起标志 */
				for (i = 0; i < 8; i++)
				{
					NVIC->ICER[i]=0xFFFFFFFF;
					NVIC->ICPR[i]=0xFFFFFFFF;
				}

//				__set_PRIMASK(0);	/* 使能全局中断 */
				
				SysMemBootJump = (void (*)(void)) (*((uint32_t *) (BootAddr + 4)));
				
				__set_MSP(*(uint32_t *)BootAddr);

			/* 在RTOS工程，这条语句很重要，设置为特权级模式，使用MSP指针 */
				__set_CONTROL(0);

//			jump2app=(iapfun)*(vu32*)(ApplicationAddress+4);		//用户代码区第二个字为程序开始地址(复位地址)		
//			MSR_MSP(*(vu32*)ApplicationAddress);					//初始化APP堆栈指针(用户代码区的第一个字用于存放栈顶地址)
			printf("准备跳转APP\r\n");
			SysMemBootJump();									//跳转到APP.
        }
        else
        {
            printf("no user app\r\n");
//            return 0;	
        }
    }
	
	

    /* USER CODE END 2 */

    /* Call init function for freertos objects (in freertos.c) */
    MX_FREERTOS_Init();
    /* Start scheduler */
    osKernelStart();

    /* We should never get here as control is now taken by the scheduler */
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
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Configure the main internal regulator output voltage
    */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
    /** Initializes the RCC Oscillators according to the specified parameters
    * in the RCC_OscInitTypeDef structure.
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 4;
    RCC_OscInitStruct.PLL.PLLN = 72;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 4;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }
    /** Initializes the CPU, AHB and APB buses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                  |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
        Error_Handler();
    }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM7 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    /* USER CODE BEGIN Callback 0 */

    /* USER CODE END Callback 0 */
    if (htim->Instance == TIM7) {
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

