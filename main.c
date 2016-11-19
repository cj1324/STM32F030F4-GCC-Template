/* Includes ------------------------------------------------------------------*/

#include "main.h"
/** @addtogroup STM32F1xx_HAL_Examples
  * @{
  */

/** @addtogroup GPIO_IOToggle
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define PUTCHAR_PROTOTYPE void __io_putchar(void* p, char ch)
/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef UartHandle;
static GPIO_InitTypeDef  GPIO_InitStruct_LED,
                         GPIO_InitStruct_BTN,
                         GPIO_InitStruct_BEEP;
uint8_t aTxBuffer[] = " ****UART_TwoBoards_ComIT****  ****UART_TwoBoards_ComIT****  ****UART_TwoBoards_ComIT**** ";
uint8_t diffvalue = 0xA;

/* Buffer used for reception */
uint8_t aRxBuffer[RXBUFFERSIZE];

PUTCHAR_PROTOTYPE;

/* ADC handle declaration */
ADC_HandleTypeDef             AdcHandle;

/* ADC channel configuration structure declaration */
ADC_ChannelConfTypeDef        sConfig;

/* Converted value declaration */
uint32_t                      aResultDMA;
uint32_t                      DMACache;

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void Error_Handler(void);
static void UART_Init(void);
static void ADC_Init(void);


/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
    HAL_Init();
    SystemClock_Config();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitStruct_LED.Pin = LED_GPIO_PIN;
    GPIO_InitStruct_LED.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct_LED.Pull  = GPIO_PULLUP;
    GPIO_InitStruct_LED.Speed = GPIO_SPEED_LOW;
    HAL_GPIO_Init(LED_GPIO_PORT, &GPIO_InitStruct_LED);
    HAL_GPIO_WritePin(LED_GPIO_PORT, LED_GPIO_PIN, GPIO_PIN_RESET);

    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitStruct_BTN.Pin = BTN_GPIO_PIN;
    GPIO_InitStruct_BTN.Mode  = GPIO_MODE_IT_RISING;
    GPIO_InitStruct_BTN.Pull  = GPIO_PULLUP;
    GPIO_InitStruct_BTN.Speed = GPIO_SPEED_LOW;

    HAL_GPIO_Init(BTN_GPIO_PORT, &GPIO_InitStruct_BTN);

    HAL_NVIC_SetPriority(EXTI0_1_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

    GPIO_InitStruct_BEEP.Pin = BEEP_GPIO_PIN;
    GPIO_InitStruct_BEEP.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct_BEEP.Pull  = GPIO_PULLUP;
    GPIO_InitStruct_BEEP.Speed = GPIO_SPEED_LOW;
    HAL_GPIO_Init(BEEP_GPIO_PORT, &GPIO_InitStruct_BEEP);
    HAL_GPIO_WritePin(BEEP_GPIO_PORT, BEEP_GPIO_PIN, GPIO_PIN_RESET);

    UART_Init();
    ADC_Init();
    DMACache = aResultDMA;
    HAL_Delay(1000);
    HAL_GPIO_WritePin(LED_GPIO_PORT, LED_GPIO_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(BEEP_GPIO_PORT, BEEP_GPIO_PIN, GPIO_PIN_SET);

    while ( 1 ) {
        HAL_Delay(1500);
        HAL_GPIO_WritePin(BEEP_GPIO_PORT, BEEP_GPIO_PIN, GPIO_PIN_SET);
        HAL_Delay(100);
        if (HAL_ADC_Start_DMA(&AdcHandle, &aResultDMA, 1) != HAL_OK)
        {
            Error_Handler();
        }

        printf("MAIN: Voltage %d Current 0x0%x%x Prev 0x0%x%x \n",
                (aResultDMA * 8) / 100,
                (aResultDMA & 0xF00) >> 8, aResultDMA & 0xFF,
                (DMACache & 0xF00) >> 8, DMACache & 0xFF);

        if (DMACache < (aResultDMA - diffvalue) || DMACache > (aResultDMA + diffvalue)){
            HAL_GPIO_WritePin(BEEP_GPIO_PORT, BEEP_GPIO_PIN, GPIO_PIN_RESET);
        }

        HAL_Delay(100);
        DMACache = aResultDMA;
    }

    Error_Handler();
    return -1;
}

static void ADC_Init(void){
    /* ### - 1 - Initialize ADC peripheral #################################### */
    /*
     *  Instance                  = ADC1.
     *  ClockPrescaler            = PCLK divided by 4.
     *  LowPowerAutoWait          = Disabled
     *  LowPowerAutoPowerOff      = Disabled
     *  Resolution                = 12 bit (increased to 16 bit with oversampler)
     *  ScanConvMode              = ADC_SCAN_ENABLE 
     *  DataAlign                 = Right
     *  ContinuousConvMode        = Enabled
     *  DiscontinuousConvMode     = Enabled
     *  ExternalTrigConv          = ADC_SOFTWARE_START
     *  ExternalTrigConvEdge      = None (Software start)
     *  EOCSelection              = End Of Conversion event
     *  DMAContinuousRequests     = ENABLE
     */

    AdcHandle.Instance = ADC1;

    AdcHandle.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV4;
    AdcHandle.Init.LowPowerAutoWait      = DISABLE;
    AdcHandle.Init.LowPowerAutoPowerOff  = DISABLE;
    AdcHandle.Init.Resolution            = ADC_RESOLUTION_12B;
    AdcHandle.Init.ScanConvMode          = ADC_SCAN_ENABLE;
    AdcHandle.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    AdcHandle.Init.ContinuousConvMode    = DISABLE;
    AdcHandle.Init.DiscontinuousConvMode = DISABLE;
    AdcHandle.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
    AdcHandle.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
    AdcHandle.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;
    AdcHandle.Init.DMAContinuousRequests = ENABLE;
    AdcHandle.Init.Overrun               = ADC_OVR_DATA_OVERWRITTEN;

    /* Initialize ADC peripheral according to the passed parameters */
    if (HAL_ADC_Init(&AdcHandle) != HAL_OK)
    {
        Error_Handler();
    }


    /* ### - 2 - Start calibration ############################################ */
    if (HAL_ADCEx_Calibration_Start(&AdcHandle) != HAL_OK)
    {
        Error_Handler();
    }

    /* ### - 3 - Channel configuration ######################################## */
    /* Select Channel 0 to be converted */
    sConfig.Channel      = ADC_CHANNEL_6;
    sConfig.Rank         = ADC_RANK_CHANNEL_NUMBER;
    sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
    if (HAL_ADC_ConfigChannel(&AdcHandle, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }

    /* ### - 4 - Start conversion in DMA mode ################################# */
    if (HAL_ADC_Start_DMA(&AdcHandle, &aResultDMA, 1) != HAL_OK)
    {
        Error_Handler();
    }
}

static void UART_Init(void){
    /*##-1- Configure the UART peripheral ######################################*/
    /* Put the USART peripheral in the Asynchronous mode (UART Mode) */
    /* UART configured as follows:
      - Word Length = 8 Bits
      - Stop Bit = One Stop bit
      - Parity = None
      - BaudRate = 9600 baud
      - Hardware flow control disabled (RTS and CTS signals) */
    UartHandle.Instance        = USARTx;
    UartHandle.Init.BaudRate   = 9600;
    UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
    UartHandle.Init.StopBits   = UART_STOPBITS_1;
    UartHandle.Init.Parity     = UART_PARITY_NONE;
    UartHandle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
    UartHandle.Init.Mode       = UART_MODE_TX;
    UartHandle.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT; 
    if(HAL_UART_DeInit(&UartHandle) != HAL_OK)
    {
        Error_Handler();
    }
    if(HAL_UART_Init(&UartHandle) != HAL_OK)
    {
        Error_Handler();
    }

    init_printf(0, __io_putchar);
}

PUTCHAR_PROTOTYPE
{
    HAL_UART_Transmit(&UartHandle, (uint8_t *)&ch, 1, 0xFFFF);
}

/**
  * @brief  System Clock Configuration
  * @param  None
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef clkinitstruct = {0};
  RCC_OscInitTypeDef oscinitstruct = {0};

  /* Configure PLL ------------------------------------------------------*/
  oscinitstruct.OscillatorType  = RCC_OSCILLATORTYPE_HSI;
  oscinitstruct.HSIState        = RCC_HSI_ON;
  oscinitstruct.HSEState        = RCC_HSE_OFF;
  oscinitstruct.HSICalibrationValue = 16;
  oscinitstruct.PLL.PLLState    = RCC_PLL_ON;
  oscinitstruct.PLL.PLLSource   = RCC_PLLSOURCE_HSI;
  oscinitstruct.PLL.PREDIV      = RCC_PREDIV_DIV1;
  oscinitstruct.PLL.PLLMUL      = RCC_PLL_MUL4;
  if (HAL_RCC_OscConfig(&oscinitstruct)!= HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  clkinitstruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1);
  clkinitstruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  clkinitstruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  clkinitstruct.APB1CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&clkinitstruct, FLASH_LATENCY_1)!= HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
}

static void Error_Handler(void) {
    while (1) {
        HAL_GPIO_TogglePin(LED_GPIO_PORT, LED_GPIO_PIN);
        HAL_Delay(2000);
    }
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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif


