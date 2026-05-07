/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include <stdint.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FRAME_LEN        11
#define SYNC1            0xAA
#define SYNC2            0x55
#define RINGBUF_SIZE     64
#define FAILSAFE_MS      200
#define FAILSAFE_CENTER  1024
#define HEARTBEAT_MS     250      /* 2 Hz toggle on lost link */

/* PWM brake servo — TIM2 CH1 (PA0), µs direct */
/* Physical inversion: 1900µs = rod extended = brake OFF, 1100µs = rod retracted = brake ON */
#define PWM_SERVO_MIN_US      1100     /* full brake ON */
#define PWM_SERVO_MAX_US      1900     /* brake OFF */
#define PWM_FAILSAFE_US       1150     /* failsafe → brake ON */

/* PWM gear servo — TIM2 CH2 (PA1), µs direct (mechanisch kalibriert) */
/*   NEUTRAL = 1375µs, DRIVE = 1450µs, REVERSE = 1300µs */
#define PWM_GEAR_MIN_US       1100
#define PWM_GEAR_MAX_US       1900
#define PWM_GEAR_FAILSAFE_US  1375     /* failsafe → NEUTRAL (kalibriert) */

#define PWM_PERIOD_US    20000    /* 50 Hz = 20ms period */
#define PWM_MAX_CCR      999      /* TIM2 counter period */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* ISR single-byte landing spot */
static uint8_t rx_byte;

/* SPSC ring buffer: ISR writes head, main reads tail — no lock needed */
static volatile uint8_t  rb_buf[RINGBUF_SIZE];
static volatile uint16_t rb_head = 0;
static volatile uint16_t rb_tail = 0;

typedef struct {
    uint16_t ch1, ch2, ch3, ch4;
    uint32_t last_frame_tick;
    uint8_t  link_ok;
} rc_state_t;

static rc_state_t rc = {
    .ch1 = PWM_FAILSAFE_US, .ch2 = FAILSAFE_CENTER,
    .ch3 = PWM_GEAR_FAILSAFE_US, .ch4 = FAILSAFE_CENTER,
    .last_frame_tick = 0, .link_ok = 0
};

/* Parser state */
typedef enum {
    PS_SYNC1 = 0,   /* hunting for 0xAA                       */
    PS_SYNC2,       /* got 0xAA, expect 0x55                  */
    PS_PAYLOAD      /* collecting 9 bytes: 4*u16 LE + CRC8    */
} parser_state_t;

static parser_state_t pstate = PS_SYNC1;
static uint8_t        payload[FRAME_LEN - 2];
static uint8_t        pidx = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
static uint8_t crc8_dallas(const uint8_t *data, uint8_t len);
static void    rb_push(uint8_t b);
static int     rb_pop(uint8_t *b);
static void    parser_feed(uint8_t b);
static void    update_servo_pwm(uint16_t pulse_us);
static void    update_gear_pwm(uint16_t pulse_us);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* CRC8 Dallas/Maxim — reflected poly 0x8C, init 0x00, no final xor. */
static uint8_t crc8_dallas(const uint8_t *data, uint8_t len) {
    uint8_t crc = 0x00;
    for (uint8_t i = 0; i < len; i++) {
        uint8_t b = data[i];
        for (uint8_t j = 0; j < 8; j++) {
            uint8_t mix = (crc ^ b) & 0x01;
            crc >>= 1;
            if (mix) crc ^= 0x8C;
            b >>= 1;
        }
    }
    return crc;
}

/* Writer — called only from ISR. Drops byte on overflow. */
static void rb_push(uint8_t b) {
    uint16_t next = (uint16_t)((rb_head + 1) % RINGBUF_SIZE);
    if (next != rb_tail) {
        rb_buf[rb_head] = b;
        rb_head = next;
    }
}

/* Reader — called only from main loop. */
static int rb_pop(uint8_t *b) {
    if (rb_head == rb_tail) return 0;
    *b = rb_buf[rb_tail];
    rb_tail = (uint16_t)((rb_tail + 1) % RINGBUF_SIZE);
    return 1;
}

/*
 * Resynchronizing frame parser.
 *
 *   PS_SYNC1   : hunt for 0xAA. Anything else is discarded.
 *   PS_SYNC2   : saw 0xAA, expect 0x55. If another 0xAA arrives, stay here
 *                (it might be the real start). Any other byte -> hunt again.
 *   PS_PAYLOAD : collect 9 bytes. When full, verify CRC over the first 8.
 *                On success, update rc and mark link_ok. On CRC failure,
 *                discard and resync — a spurious 0xAA 0x55 only costs one frame.
 */
static void parser_feed(uint8_t b) {
    switch (pstate) {
    case PS_SYNC1:
        if (b == SYNC1) pstate = PS_SYNC2;
        break;

    case PS_SYNC2:
        if (b == SYNC2)      { pstate = PS_PAYLOAD; pidx = 0; }
        else if (b != SYNC1) { pstate = PS_SYNC1; }
        break;

    case PS_PAYLOAD:
        payload[pidx++] = b;
        if (pidx == (FRAME_LEN - 2)) {
            uint8_t crc_calc = crc8_dallas(payload, 8);
            uint8_t crc_rx   = payload[8];
            if (crc_calc == crc_rx) {
                rc.ch1 = (uint16_t)payload[0] | ((uint16_t)payload[1] << 8);
                rc.ch2 = (uint16_t)payload[2] | ((uint16_t)payload[3] << 8);
                rc.ch3 = (uint16_t)payload[4] | ((uint16_t)payload[5] << 8);
                rc.ch4 = (uint16_t)payload[6] | ((uint16_t)payload[7] << 8);
                rc.last_frame_tick = HAL_GetTick();
                rc.link_ok = 1;
                update_servo_pwm(rc.ch1);
                update_gear_pwm(rc.ch3);
            }
            pstate = PS_SYNC1;
        }
        break;
    }
}

/* CH1 value is pulse width in µs, clamped to safe brake servo range. */
static void update_servo_pwm(uint16_t pulse_us) {
    if (pulse_us < PWM_SERVO_MIN_US) pulse_us = PWM_SERVO_MIN_US;
    if (pulse_us > PWM_SERVO_MAX_US) pulse_us = PWM_SERVO_MAX_US;
    uint32_t ccr = (pulse_us * (PWM_MAX_CCR + 1)) / PWM_PERIOD_US;
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, ccr);
}

/* CH3 value is pulse width in µs, clamped to safe gear servo range. */
static void update_gear_pwm(uint16_t pulse_us) {
    if (pulse_us < PWM_GEAR_MIN_US) pulse_us = PWM_GEAR_MIN_US;
    if (pulse_us > PWM_GEAR_MAX_US) pulse_us = PWM_GEAR_MAX_US;
    uint32_t ccr = (pulse_us * (PWM_MAX_CCR + 1)) / PWM_PERIOD_US;
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, ccr);
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
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart2, &rx_byte, 1);

  /* Start PWM on TIM2 CH1 (PA0) brake servo, CH2 (PA1) gear servo */
  extern TIM_HandleTypeDef htim2;
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

  /* Set initial failsafe positions */
  update_servo_pwm(PWM_FAILSAFE_US);
  update_gear_pwm(PWM_GEAR_FAILSAFE_US);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* Drain ISR bytes into parser */
    uint8_t b;
    while (rb_pop(&b)) parser_feed(b);

    /* Failsafe timeout */
    uint32_t now = HAL_GetTick();
    if (rc.link_ok && (now - rc.last_frame_tick) > FAILSAFE_MS) {
        rc.link_ok = 0;
        rc.ch1 = PWM_FAILSAFE_US;
        rc.ch3 = PWM_GEAR_FAILSAFE_US;
        rc.ch2 = rc.ch4 = FAILSAFE_CENTER;
        update_servo_pwm(PWM_FAILSAFE_US);
        update_gear_pwm(PWM_GEAR_FAILSAFE_US);
    }

    /* LED: solid on valid link, 2 Hz blink on failsafe */
    if (rc.link_ok) {
        HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
    } else {
        static uint32_t last_toggle = 0;
        if ((now - last_toggle) >= HEARTBEAT_MS) {
            HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
            last_toggle = now;
        }
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  htim2.Init.Prescaler = 1679;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  sConfigOC.Pulse = 75;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 0;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        rb_push(rx_byte);
        HAL_UART_Receive_IT(&huart2, &rx_byte, 1);   /* re-arm */
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        __HAL_UART_CLEAR_OREFLAG(huart);
        __HAL_UART_CLEAR_NEFLAG(huart);
        __HAL_UART_CLEAR_FEFLAG(huart);
        __HAL_UART_CLEAR_PEFLAG(huart);
        HAL_UART_Receive_IT(&huart2, &rx_byte, 1);   /* re-arm after any error */
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
#ifdef USE_FULL_ASSERT
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
