### HC=SR04 초음파센서를 이용한 거리 측정

---

#### 1. TIM3를 이용한 1㎲ 타이머 인터럽트 구현



 **1.1 RCC 설정**

RCC 설정을 위해 다음 그림과 같이 Device Configuration 창에서 Pinout & Configuration 탭의 System Core 항목 중 RCC를 선택 후 우측의 RCC Mode and Configuration 의 Mode의 High Speed Clock(HSE) 및 Low Speed Clock(LSE) 모두를 Disable로 변경한다.

![](./img/system_core_rcc.png)





 **1.2 TIM3 설정**

  Pinout & Configuration탭의 Timers의 하위항목 중 Tim3를 선택한다.  Tim4 Mode and Configuration의 Mode에서 Internal Clock을 선택하면, 아래에 Parameter Setting 항목이 나타난다.

  ![](./img/tim3_mode_n_config1.png)

**1.3 타이머 클럭 확인**

 Tim4의 Congiguration의 arameter Settings 탭의 Prescaler,  Counter Period 설정에 앞서 Clock을 확인해야 한다. 

  STM32-F1xx 시리즈 MCU는 CPU와  다수의 peripheral 장치들로 이루어져 있으며, 이들은 ARMBA( Advanced Microcontroller Bus Architecture ) 버스로 연결되어 있다. 

  ARMBA( Advanced Microcontroller Bus Architecture ) 버스는 AHB(Advanced High Performance Bus), APB1(Advanced Peripheral Bus1 ), APB2( Advanced Peripheral Bus2 ) 의 3가지 BUS로 이루어지며, 다음은 각 Peripheral Device들이 어떤 버스에 연결되어 있는가를 보여주는  구성도이다. 

  ![](./img/stm32_f1xx_system_architecture.png)

  

1㎲ 주기의  TIM3 타이머 인터럽트 설정을 위해 우선 TIM3가 연결된 APB1 버스에 공급되는 클럭 주파수를 확인해보자.  Clock Configuration 탭을 선택한다.

  APB1 Timer Clock이 64(MHz)라는 것을 확인할 수 있다. ![](./img/tim4_clock_config.png)

  

TIM3에는 1초에 64,000,000 개의 클럭이 공급된다. 클럭 1개의 주기는 1/64,000,000 초 이므로 1㎲를 만드려면 64개의 클럭이 필요하다. ( 64 x 1/64,000,000 = 64/64,000,000 = 1/1,000,000 = 1㎲ )

이를 위해 Tim3 Mode and Configuration의 Parameter Settings에서 Prescaler와 Counter Period에 적절한 값을 설정하여 이를 맞춰줘야 한다. Prescaler = 1, Counter Period = 64 또는 Prescaler = 4, Counter Period = 16 과 같이 설정해야 하지만,  실제로 동작 시켰을 때 오류가 확인되어 일단 Prescaler = 4, Counter Period = 64로 설정 후, 거리 측정 코드에서 보정하여 주기로 하자. 다음 그림과 같이 Prescaler = 4-1, Counter Period = 64-1을 입력한다.

![](.\img\tim3_mode_n_config2.png)

TIM3 Mode and Configuration의 NVIC Setting 탭

 

  

  

  **HC-SR04 결선 및 GPIO 설정**

HC-SR04 초음파 센서는 VCC, Trig, Echo, GND 4개의 핀을 가지고 있다. 다음 그림과 같이 결선 한다.

  ![](./img/wiring_HC-SR04.png)

  센서의 Trig핀으로 10㎲ 펄스폭을 갖는 펄스를 공급하면 센서는 8개의 초음파를 발사한다. 이 때 센서의 Echo핀의 출력은 Low를 유지 하지만 발사한 초음파의 반사파가 수신되기 시작하면 Echo핀으로 High 신호를 출력한다. 이 High신호는 모든 반사파가 수신될 때까지 유지되다가 Low로 떨어진다. 즉 Echo 핀의 출력이 High를 유지한 시간이 초음파가 거리를 측정하고자 하는 대상까지 왕복하는데에 소요된 시간이라는 것이다. 음파의 속도는 340m/s이므로 속도 = 거리 / 시간 에서 속도와 시간을 알아낼 수 있으므로 거리를 구할 수 있다. 

이제 센서의 Trig 핀이 연결된 A0와 Echo핀이 연결된 A1에 대한 GPIO 설정을 해 주어야 한다.

- A0는 Trig핀에 Trig 펄스를 공급해 주어야 하므로 출력으로 설정하고,

  ![](.\img\config_GPIO_A0.png)

-  A1은 Echo핀의 출력이 언제 Low에서 High로 변하는 지, 또는 High에서 Low로 변하는 지 읽어내야 하므로 입력으로 설정해야 한다.

  ![](D:\Dropbox\myGit\stm32\ex12_HC-SR04\img\config_GPIO_A1.png)

  이제 GPIO Configuration을 확인하면 다음과 같다.

![](./img/GPIO_config.png)

**Code Generate**

`Project` 메뉴의 `Generate Code`를 클릭


![](D:/Dropbox/myGit/stm32/ex12_HC-SR04/img/generate_code.png)



`Project` 메뉴의 `Generate Code`를 클릭하여 생성된 `main.c`를 다음과 같이 수정한다. 



`main.c`의 다음코드를 `main.c`의 다음코드를 

```c
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */
```

아래와 같이 수정한다. 

```c
/* USER CODE BEGIN Includes */
#include<stdio.h>
#include<string.h>
/* USER CODE END Includes */
```



`main.c`의 다음코드를 `main.c`의 다음코드를 

```c
/* USER CODE BEGIN PV */

/* USER CODE END PV */
```

아래와 같이 수정한다. 

```c
/* USER CODE BEGIN PV */
uint8_t str[10];
uint8_t pos_pan = 75;
uint8_t pos_tilt = 75;
uint8_t digits;
/* USER CODE END PV */
```





`main.c`의 다음코드를 `main.c`의 다음코드를 

```c
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */
```

아래와 같이 수정한다. 

```c
/* USER CODE BEGIN PFP */
int check_digit(void);
void reset_str(void);
/* USER CODE END PFP */
```





`printf()` 사용을 위해 `main.c`의 다음코드를 `main.c`의 다음코드를 

```c
/* USER CODE BEGIN 0 */

}
/* USER CODE END 0 */
```

아래와 같이 수정한다. 

```c
/* USER CODE BEGIN 0 */
#ifdef __GNUC__
/* With GCC, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART1 and Loop until the end of transmission */
  if (ch == '\n')
    HAL_UART_Transmit (&huart2, (uint8_t*) "\r", 1, 0xFFFF);
  HAL_UART_Transmit (&huart2, (uint8_t*) &ch, 1, 0xFFFF);

  return ch;
}
/* USER CODE END 0 */
```





위에서 수정한 코드 마지막 줄 `/* USER CODE END 0 */` 바로 다음 줄에 다음코드를 삽입한다. 

```c
/* USER CODE BEGIN 1 */
	int check_digit() {
		int i, cnt;
		cnt = 0;
		for(i=0; i<=sizeof(str); i++) {
			if(str[i]>='0' && str[i]<='9') {
				cnt++;
			}
		}
		return cnt;
		}
void reset_str(void) {
	for(int i=0; i<=sizeof(str); i++)
	{
		str[i] = '\0';
	}
}

/* USER CODE END 1 */
```





`main.c`의 다음코드를 `main.c`의 다음코드를 

```c
 /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
```

아래와 같이 수정한다. 

```c
/* USER CODE BEGIN WHILE */
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_2);
  __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_2, pos_pan);
  __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, pos_tilt);

  while (1)
  {
    /* USER CODE END WHILE */
```



`main.c`의 다음코드를 `main.c`의 다음코드를 

```c
/* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */
```



```c
		/* USER CODE BEGIN 3 */

	  HAL_UART_Receive(&huart2,str,12,100);

	  printf("%s\r\n",str); // 125 75
	  char *pan = strtok(str, " ");
	  char *tilt = strtok(NULL, " ");
	  pos_pan = atoi(pan);
	  __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_2, pos_pan);
	  HAL_Delay(10);
	  pos_tilt = atoi(tilt);
	  __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, pos_tilt);
	  HAL_Delay(10);
	  printf("pan = %d, tilt = %d\r\n", pos_pan, pos_tilt);




	  char *ptr1_str = strstr(str, "pan");
	  char *ptr2_str = strstr(str, "tilt");

	  if(ptr1_str !=NULL )
	  {
		  digits = check_digit();
		  if     (digits==3) {
		  pos_pan = (str[3]-'0')*100 + (str[4]-'0')*10 +(str[5]-'0');
		  }
		  else if(digits==2) {
			  pos_pan = (str[3]-'0')*10 + (str[4]-'0');
		  }
		  else if(digits==1) {
			  pos_pan = str[3]-'0';
		  }
		  printf("pos_pan = %d\n",pos_pan);
		  __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_2, pos_pan);
		  reset_str();
	  }

	  else if(ptr2_str !=NULL )
	  {
		  digits = check_digit();
		  if     (digits==3){
		  pos_tilt = (str[4]-'0')*100 + (str[5]-'0')*10 +(str[6]-'0');
		  }
		  else if(digits==2) {
			  pos_tilt = (str[4]-'0')*10 + (str[5]-'0');
		  }
		  else if(digits==1) {
			  pos_tilt = str[4]-'0';
		  }
		  printf("pos_tilt = %d\n",pos_tilt);
		  __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, pos_tilt);
		  reset_str();
	  }
  }
  /* USER CODE END 3 */
```



다음은 편집이 완료된 `main.c`의 전체 코드이다.

```c
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
#include<stdio.h>
#include<string.h>
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
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t str[10];
uint8_t pos_pan;
uint8_t pos_tilt;
uint8_t digits;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
int check_digit(void);
void reset_str(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#ifdef __GNUC__
/* With GCC, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART1 and Loop until the end of transmission */
  if (ch == '\n')
    HAL_UART_Transmit (&huart2, (uint8_t*) "\r", 1, 0xFFFF);
  HAL_UART_Transmit (&huart2, (uint8_t*) &ch, 1, 0xFFFF);

  return ch;
}
/* USER CODE END 0 */

/* USER CODE BEGIN 1 */
	int check_digit() {
		int i, cnt;
		cnt = 0;
		for(i=0; i<=sizeof(str); i++) {
			if(str[i]>='0' && str[i]<='9') {
				cnt++;
			}
		}
		return cnt;
		}
void reset_str(void) {
	for(int i=0; i<=sizeof(str); i++)
	{
		str[i] = '\0';
	}
}

/* USER CODE END 1 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

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
  MX_TIM4_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_2);

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  HAL_UART_Receive(&huart2,str,10,100);
	  char *ptr1_str = strstr(str, "pan");
	  char *ptr2_str = strstr(str, "tilt");

	  if(ptr1_str !=NULL )
	  {
		  digits = check_digit();
		  if     (digits==3) {
		  pos_pan = (str[3]-'0')*100 + (str[4]-'0')*10 +(str[5]-'0');
		  }
		  else if(digits==2) {
			  pos_pan = (str[3]-'0')*10 + (str[4]-'0');
		  }
		  else if(digits==1) {
			  pos_pan = str[3]-'0';
		  }
		  printf("pos_pan = %d\n",pos_pan);
		  __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_2, pos_pan);
		  reset_str();
	  }

	  else if(ptr2_str !=NULL )
	  {
		  digits = check_digit();
		  if     (digits==3){
		  pos_tilt = (str[4]-'0')*100 + (str[5]-'0')*10 +(str[6]-'0');
		  }
		  else if(digits==2) {
			  pos_tilt = (str[4]-'0')*10 + (str[5]-'0');
		  }
		  else if(digits==1) {
			  pos_tilt = str[4]-'0';
		  }
		  printf("pos_tilt = %d\n",pos_tilt);
		  __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, pos_tilt);
		  reset_str();
	  }
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
  htim2.Init.Prescaler = 1280-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000-1;
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
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 1280-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 1000-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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

```

**Project** 메뉴의 **Build Project**를 선택하여 빌드한다.

![](D:/Dropbox/myGit/stm32/ex12_HC-SR04/img/build_result.png)



에러없이 빌드되었으면, 서보모터의 갈색 선을 GND에, 적색 선을 +5V에, 주황색 선을 NUCLEO 보드의 PB7 핀에 결선한다.

![](D:/Dropbox/myGit/stm32/ex12_HC-SR04/img/pb7.png)

 **RUN** 메뉴에서 **RUN** 항목을 선택하여 실행한다. 

`pan25` ~ `pan125` 를 시리얼로 전송하면 TIM2 Channel2에 연결된 서보모터가 0도 ~ 180도 회전하고, `tilt25` ~ `tilt125` 를 시리얼로 전송하면 TIM2 Channel2에 연결된 서보모터가 0도 ~ 180도 회전하는 것을 확인한다. 

![](D:/Dropbox/myGit/stm32/ex12_HC-SR04/putty_pt_ctrl.png)



![](D:/Dropbox/myGit/stm32/ex12_HC-SR04/img/putty_pt_ctrl.png)





















![](D:/Dropbox/myGit/stm32/ex12_HC-SR04/img/putty.png)



[**목차**](../README.md) 