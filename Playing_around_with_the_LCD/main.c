/*************************************************************************
 *
 *    Used with ICCARM and AARM.
 *
 *    (c) Copyright IAR Systems 2006
 *
 *    File name   : glcd_ll.c
 *    Description : GLCD low level functions
 *
 *    History :
 *    1. Date        : December 2, 2006
 *       Author      : Stanimir Bonev
 *       Description : Create
 *
 *    $Revision: #2 $
 **************************************************************************/
#include "includes.h"
#include "glcd_ll.h"
#include "drv_glcd.h"
#include "drv_glcd_cnfg.h"


#include "stm32f4xx_tim.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"


int LCD_Test(void);

int main(void)
{

	LCD_Test();

    while(1)
    {
    }
}


/*-------------------------------------------------------------------------------------------------------------*/

int LCD_Test(void)
{
	int cntr = 0;
	char Backlight = 80;
	char BacklightStep = 1;
	uint8_t testData;

	printf("Press Esc for exit\n\r");


	/*We could use the buttons to control the screen*/

	uint8_t *pOutputData = (uint8_t)(testData);
	uint8_t *pInData;



	// initialize display
	GLCD_PowerUpInit(NULL);
	GLCD_Backlight(BACKLIGHT_ON);

	GLCD_SetFont(&Terminal_9_12_6,0x000F00,0x00FF0);
	GLCD_SetWindow(10,116,131,131);
	GLCD_TextSetPos(0,0);
	GLCD_print("\fNO CAMERA!!\r");


	/*Systick start*/
	SysTickStart(10);

	GLCD_UpdateMemory(pOutputData);


//	// adjust backlight level
//	if(Bit_SET == STM_EVAL_PBGetState(BUTTON_UP))
//	{
//		if(Backlight < 96) {
//			Backlight += BacklightStep;
//			GLCD_Backlight(Backlight);
//		}
//	}
//	else if(Bit_SET == STM_EVAL_PBGetState(BUTTON_DOWN))
//	{
//		if(Backlight > 64) {
//			Backlight -= BacklightStep;
//			GLCD_Backlight(Backlight);
//		}
//	}


	//SysTickStop();

	/**/
	printf("                                              \r");

	return 0;
}

/*-------------------------------------------------------------------------------------------------------------*/


static uint32_t Width;

/*************************************************************************
 * Function Name: GLCD_SetReset
 * Parameters: Boolean State
 * Return: none
 *
 * Description: Set reset pin state
 *
 *************************************************************************/
void GLCD_SetReset (Boolean State)
{
  GPIO_WriteBit(LCD_RST_PORT,LCD_RST_MASK,(State)?Bit_SET:Bit_RESET);
}

/*************************************************************************
 * Function Name: GLCD_SetBacklight
 * Parameters: uint8_t Light
 * Return: none
 *
 * Description: Set backlight pin state
 *
 *************************************************************************/
void GLCD_Backlight (uint8_t Light)
{
  TIM3->CCR3 = Light;
}

/*************************************************************************
 * Function Name: GLCD_LLInit
 * Parameters: none
 * Return: none
 *
 * Description: Init Reset and Backlight control outputs
 *
 *************************************************************************/
void GLCD_LLInit (void)
{
TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
GPIO_InitTypeDef GPIO_InitStructure;
TIM_OCInitTypeDef TIM_OCInitStructure;

  /* Enable GPIO clock and release reset*/
  RCC_AHB1PeriphClockCmd(LCD_RST_CLK | LCD_BL_CLK, ENABLE);
  RCC_AHB1PeriphResetCmd(LCD_RST_CLK | LCD_BL_CLK,DISABLE);

  /*LCD Reset pin init*/
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;

  GPIO_InitStructure.GPIO_Pin = LCD_RST_MASK;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(LCD_RST_PORT, &GPIO_InitStructure);

  GLCD_SetReset(0);

	/* LCD backlight Init*/
  // PWM DAC (TIM3/CH3)
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;

  GPIO_InitStructure.GPIO_Pin = LCD_BL_MASK;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(LCD_BL_PORT, &GPIO_InitStructure);

  GPIO_PinAFConfig(LCD_BL_PORT,LCD_BL_PIN_SOURCE,LCD_BL_PIN_AF);

  // Init PWM TIM3
  // Enable Timer3 clock and release reset
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
  RCC_APB1PeriphResetCmd(RCC_APB1Periph_TIM3,DISABLE);

  TIM_InternalClockConfig(TIM3);

  // Time base configuration
  TIM_TimeBaseStructure.TIM_Prescaler = 140;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_Period = 0xFF; // 8 bit resolution
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(TIM3,&TIM_TimeBaseStructure);

  // Channel 4 Configuration in PWM mode
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 0x00;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
  TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
  TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;
  TIM_OC3Init(TIM3,&TIM_OCInitStructure);
  // Double buffered
  TIM_ARRPreloadConfig(TIM3,ENABLE);
  // TIM3 counter enable
  TIM_Cmd(TIM3,ENABLE);

  GLCD_Backlight(0);
}

/*************************************************************************
 * Function Name: LcdSpiChipSelect
 * Parameters: Boolean Select
 * Return: none
 *
 * Description: SPI Chip select control
 * Select = true  - Chip is enable
 * Select = false - Chip is disable
 *
 *************************************************************************/
void GLCD_SPI_ChipSelect (Boolean Select)
{
  GPIO_WriteBit(LCD_CS_PORT,LCD_CS_MASK,(Select)?Bit_RESET:Bit_SET);
}

/*************************************************************************
 * Function Name: LcdSpiSetWordWidth
 * Parameters: uint32_t Data
 * Return: Boolean
 *
 * Description: Set SPI word width
 *
 *************************************************************************/
Boolean GLCD_SPI_SetWordWidth (uint32_t Data)
{
  if(   (8 != Data)
     && (9 != Data))
  {
    return(FALSE);
  }

  Width = Data;
  return(TRUE);
}

/*************************************************************************
 * Function Name: LcdSpiSetClockFreq
 * Parameters: uint32_t Frequency
 * Return: uint32_t
 *
 * Description: Set SPI clock
 *
 *************************************************************************/
/*
uint32_t GLCD_SPI_SetClockFreq (uint32_t Frequency)
{
uint32_t Fspi = SYS_GetFpclk(Ssp1_PCLK_OFFSET);
uint32_t Div = 2;
  while((Div * Frequency) < Fspi)
  {
    if((Div += 2) == 254)
    {
      break;
    }
  }
  Ssp1CPSR = Div;
  return(Fspi/Div);
}
*/
/*************************************************************************
 * Function Name: GLCD_SPI_Init
 * Parameters: uint32_t Clk, uint32_t Width
 * Return: none
 *
 * Description: Init SPI
 *
 *************************************************************************/
void GLCD_SPI_Init(uint32_t Clk, uint32_t Width)
{
GPIO_InitTypeDef GPIO_InitStructure;
  /* Enable GPIO clock and release reset*/
  RCC_AHB1PeriphClockCmd(LCD_CS_CLK | LCD_SPI_MISO_CLK |
                         LCD_SPI_MOSI_CLK | LCD_SPI_SCLK_CLK, ENABLE);
  RCC_AHB1PeriphResetCmd(LCD_CS_CLK | LCD_SPI_MISO_CLK |
                         LCD_SPI_MOSI_CLK | LCD_SPI_SCLK_CLK, DISABLE);

  /*LCD_CS*/
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;

  GPIO_InitStructure.GPIO_Pin = LCD_CS_MASK;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(LCD_CS_PORT, &GPIO_InitStructure);
  /*LCD_SPI_SCLK*/
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;

  GPIO_InitStructure.GPIO_Pin = LCD_SPI_SCLK_MASK;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(LCD_SPI_SCLK_PORT, &GPIO_InitStructure);

  /*LCD_SPI_MOSI*/
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;

  GPIO_InitStructure.GPIO_Pin = LCD_SPI_MOSI_MASK;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(LCD_SPI_MOSI_PORT, &GPIO_InitStructure);

  /*LCD_SPI_MISO*/
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;

  GPIO_InitStructure.GPIO_Pin = LCD_SPI_MISO_MASK;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(LCD_SPI_MISO_PORT, &GPIO_InitStructure);

  // Chip select
  GLCD_SPI_ChipSelect(0);
  GLCD_SPI_CLK_H();

  // Chip select
  GLCD_SPI_ChipSelect(FALSE);

  // Set data width
  GLCD_SPI_SetWordWidth(Width);

}

/*************************************************************************
 * Function Name: GLCD_SPI_TranserByte
 * Parameters: uint32_t Data
 * Return: uint32_t
 *
 * Description: Transfer byte from SPI
 *
 *************************************************************************/
uint32_t GLCD_SPI_TranserByte (uint32_t Data)
{
uint32_t InData = 0;
uint32_t Mask;
int i;
  for (Mask = 1UL << (Width-1); Mask; Mask>>= 1)
  {
    // Clock Low
    GLCD_SPI_CLK_L();
    // Set Data
    if (Mask & Data)
    {
      GLCD_SPI_MOSI_H();
    }
    else
    {
      GLCD_SPI_MOSI_L();
    }
    InData <<= 1;
    // Clock High
    for(i= 5; i; i-- );

    GLCD_SPI_CLK_H();

    // Get Data
    if (GLCD_SPI_MISO())
    {
      ++InData;
    }
    for(i= 5; i; i-- );
  }
  return(InData);
}

/*************************************************************************
 * Function Name: GLCD_SPI_SendBlock
 * Parameters: puint8_t pData, uint32_t Size
 *
 * Return: void
 *
 * Description: Write block of data to SPI
 *
 *************************************************************************/
/*instead of uint8_t, the example had pint8u*/
void GLCD_SPI_SendBlock (uint8_t *pData, uint32_t Size)
{
uint32_t OutCount = Size;
  while (OutCount--)
  {

	  uint32_t tmp=(int)(pData++)%1000;
	  if (tmp<100)
	  {
		  tmp = (uint32_t)(pData++)+0x100;
	  }
	  GLCD_SPI_TranserByte(tmp);
  }
}

/*************************************************************************
 * Function Name: GLCD_SPI_ReceiveBlock
 * Parameters: puint8_t pData, uint32_t Size
 *
 * Return: void
 *
 * Description: Read block of data from SPI
 *
 *************************************************************************/
/*instead of uint8_t, the example had pint8u*/
void GLCD_SPI_ReceiveBlock (uint8_t *pData, uint32_t Size)
{
  while (Size)
  {

	  *pData = GLCD_SPI_TranserByte(0xFFFF0);
	  --Size;
  }
}
