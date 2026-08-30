/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include "canbus.h"
#include <string.h>

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_CAN2_Init(void);

/* ============================================================
   MAIN
============================================================ */
int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_CAN1_Init();
    MX_CAN2_Init();

    /* USER CODE BEGIN 2 */
    HAL_CAN_Start(&hcan1);
    HAL_CAN_Start(&hcan2);

    last_4D0_time      = HAL_GetTick();   // <<< ważne
    last_can_activity  = HAL_GetTick();
    last_apim_time     = HAL_GetTick();



    while (1)
    {
        CAN2_TICK(&hcan2);      // okresowe ramki emulatora na CAN2
        CAN2_PROCESS(&hcan2);   // APIM, VIN i przyciski na CAN2
        CAN1_PROCESS(&hcan1);   // monitorowanie CAN1 (m.in. ramka 0x4D0)
    }
}

/* ============================================================
   CLOCK CONFIG
============================================================ */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV5;
    RCC_OscInitStruct.Prediv1Source   = RCC_PREDIV1_SOURCE_PLL2;

    RCC_OscInitStruct.PLL.PLLState    = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource   = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL      = RCC_PLL_MUL8;

    RCC_OscInitStruct.PLL2.PLL2State        = RCC_PLL2_ON;
    RCC_OscInitStruct.PLL2.PLL2MUL          = RCC_PLL2_MUL8;
    RCC_OscInitStruct.PLL2.HSEPrediv2Value  = RCC_HSE_PREDIV2_DIV5;

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
        Error_Handler();

    RCC_ClkInitStruct.ClockType =
        RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
        RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;

    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
        Error_Handler();
}

/* ============================================================
   GPIO INIT — przyciski + LED
============================================================ */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* PC10 / PC11 / PC12 — INPUT PULLUP */
    GPIO_InitStruct.Pin  = GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* PC15  — INPUT PULLUP */
    GPIO_InitStruct.Pin  = GPIO_PIN_15 | GPIO_PIN_14;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pins : PB3 PB4 */
    GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* LED — PD2 */
    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* LED — PA2 */
    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* ============================================================
   CAN1 INIT
============================================================ */
static void MX_CAN1_Init(void)
{
    hcan1.Instance = CAN1;
    hcan1.Init.Prescaler = 2;
    hcan1.Init.Mode = CAN_MODE_NORMAL;
    hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
    hcan1.Init.TimeSeg1 = CAN_BS1_13TQ;
    hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
    hcan1.Init.AutoRetransmission = ENABLE;
    if (HAL_CAN_Init(&hcan1) != HAL_OK)
     {
       Error_Handler();
     }
     /* USER CODE BEGIN CAN1_Init 2 */
     //=========================================================================================================================================
     CAN_FilterTypeDef FILTER;
     //
     FILTER.FilterActivation = CAN_FILTER_ENABLE;
     FILTER.FilterFIFOAssignment = CAN_RX_FIFO1;
     FILTER.FilterMode = CAN_FILTERMODE_IDMASK;
     FILTER.FilterScale = CAN_FILTERSCALE_16BIT;
     FILTER.SlaveStartFilterBank = 14;
     //
     FILTER.FilterBank = 0;
     FILTER.FilterIdHigh = 0x200<<5;
     FILTER.FilterIdLow = 0x080<<5;
     FILTER.FilterMaskIdHigh = 0x7FF<<5;
     FILTER.FilterMaskIdLow = 0x7FF<<5;
     HAL_CAN_ConfigFilter(&hcan1, &FILTER);
     //
     FILTER.FilterBank = 1;
     FILTER.FilterIdHigh = 0x7E8<<5;
     FILTER.FilterIdLow = 0x72E<<5;
     FILTER.FilterMaskIdHigh = 0x7FF<<5;
     FILTER.FilterMaskIdLow = 0x7FF<<5;
     HAL_CAN_ConfigFilter(&hcan1, &FILTER);
     //
     FILTER.FilterBank = 2;
     FILTER.FilterIdHigh = 0x7E9<<5;
     FILTER.FilterIdLow = 0x7EC<<5;
     FILTER.FilterMaskIdHigh = 0x7FF<<5;
     FILTER.FilterMaskIdLow = 0x7FF<<5;
     HAL_CAN_ConfigFilter(&hcan1, &FILTER);
     //
     FILTER.FilterBank = 3;
     FILTER.FilterIdHigh = 0x0F8<<5;
     FILTER.FilterIdLow = 0x74E<<5;
     FILTER.FilterMaskIdHigh = 0x7FF<<5;
     FILTER.FilterMaskIdLow = 0x7FF<<5;
     HAL_CAN_ConfigFilter(&hcan1, &FILTER);
     //
     FILTER.FilterBank = 4;
     FILTER.FilterIdHigh = 0x3B2<<5;
     FILTER.FilterIdLow = 0x3B3<<5;
     FILTER.FilterMaskIdHigh = 0x7FF<<5;
     FILTER.FilterMaskIdLow = 0x7FF<<5;
     HAL_CAN_ConfigFilter(&hcan1, &FILTER);
     //
     FILTER.FilterBank = 5;
     FILTER.FilterIdHigh = 0x048<<5;
     FILTER.FilterIdLow = 0x2A1<<5;
     FILTER.FilterMaskIdHigh = 0x7FF<<5;
     FILTER.FilterMaskIdLow = 0x7FF<<5;
     HAL_CAN_ConfigFilter(&hcan1, &FILTER);
     //
     FILTER.FilterBank = 6;
     FILTER.FilterIdHigh = 0x202<<5;
     FILTER.FilterIdLow = 0x777<<5;
     FILTER.FilterMaskIdHigh = 0x7FF<<5;
     FILTER.FilterMaskIdLow = 0x7FF<<5;
     HAL_CAN_ConfigFilter(&hcan1, &FILTER);
     //
     FILTER.FilterBank = 7;
     FILTER.FilterIdHigh = 0x4D0<<5;
     FILTER.FilterIdLow = 0x777<<5;
     FILTER.FilterMaskIdHigh = 0x7FF<<5;
     FILTER.FilterMaskIdLow = 0x7FF<<5;
     HAL_CAN_ConfigFilter(&hcan1, &FILTER);
   }
/* ============================================================
   CAN2 INIT
============================================================ */
static void MX_CAN2_Init(void)
{
    hcan2.Instance = CAN2;
    hcan2.Init.Prescaler = 2;
    hcan2.Init.Mode = CAN_MODE_NORMAL;
    hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
    hcan2.Init.TimeSeg1 = CAN_BS1_13TQ;
    hcan2.Init.TimeSeg2 = CAN_BS2_2TQ;
    hcan2.Init.AutoRetransmission = ENABLE;

    if (HAL_CAN_Init(&hcan2) != HAL_OK)
     {
       Error_Handler();
     }
     /* USER CODE BEGIN CAN2_Init 2 */
     //=========================================================================================================================================
     CAN_FilterTypeDef FILTER;
     //
     FILTER.FilterActivation = CAN_FILTER_ENABLE;
     FILTER.FilterFIFOAssignment = CAN_RX_FIFO0;
     FILTER.FilterMode = CAN_FILTERMODE_IDMASK;
     FILTER.FilterScale = CAN_FILTERSCALE_16BIT;
     FILTER.SlaveStartFilterBank = 14;
     //
     FILTER.FilterBank = 14;
     FILTER.FilterIdHigh = 0x3DA<<5;
     FILTER.FilterIdLow = 0x109<<5;
     FILTER.FilterMaskIdHigh = 0x7FF<<5;
     FILTER.FilterMaskIdLow = 0x7FF<<5;
     HAL_CAN_ConfigFilter(&hcan2, &FILTER);
     //
     FILTER.FilterBank = 15;
     FILTER.FilterIdHigh = 0x40A<<5;
     FILTER.FilterIdLow = 0x179<<5;
     FILTER.FilterMaskIdHigh = 0x7FF<<5;
     FILTER.FilterMaskIdLow = 0x7FF<<5;
     HAL_CAN_ConfigFilter(&hcan2, &FILTER);
     //
     FILTER.FilterBank = 16;
     FILTER.FilterIdHigh = 0x728<<5;
     FILTER.FilterIdLow = 0x081<<5;
     FILTER.FilterMaskIdHigh = 0x7FF<<5;
     FILTER.FilterMaskIdLow = 0x7FF<<5;
     HAL_CAN_ConfigFilter(&hcan2, &FILTER);
     //
     FILTER.FilterBank = 17;
     FILTER.FilterIdHigh = 0x3B2<<5;
     FILTER.FilterIdLow = 0x3B3<<5;
     FILTER.FilterMaskIdHigh = 0x7FF<<5;
     FILTER.FilterMaskIdLow = 0x7FF<<5;
     HAL_CAN_ConfigFilter(&hcan2, &FILTER);
     //
     FILTER.FilterBank = 18;
     FILTER.FilterIdHigh = 0x048<<5;
     FILTER.FilterIdLow = 0x2A1<<5;
     FILTER.FilterMaskIdHigh = 0x7FF<<5;
     FILTER.FilterMaskIdLow = 0x7FF<<5;
     HAL_CAN_ConfigFilter(&hcan2, &FILTER);
     //
     FILTER.FilterBank = 19;
     FILTER.FilterIdHigh = 0x202<<5;
     FILTER.FilterIdLow = 0x777<<5;
     FILTER.FilterMaskIdHigh = 0x7FF<<5;
     FILTER.FilterMaskIdLow = 0x7FF<<5;
     HAL_CAN_ConfigFilter(&hcan2, &FILTER);
     //
     FILTER.FilterBank = 20;
     FILTER.FilterIdHigh = 0x4D0<<5;
     FILTER.FilterIdLow = 0x777<<5;
     FILTER.FilterMaskIdHigh = 0x7FF<<5;
     FILTER.FilterMaskIdLow = 0x7FF<<5;
     HAL_CAN_ConfigFilter(&hcan2, &FILTER);
   }
/* ============================================================
   ERROR HANDLER
============================================================ */
void Error_Handler(void)
{
    __disable_irq();
    while (1) { }
}
