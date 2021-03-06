/*========================================================================================*/
/*==============================leds & delay==============================================*/
/*========================================================================================*/


#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_tim.h"

#include "Other.h"


void led_init(void)
{
	RCC_AHB1PeriphClockCmd (RCC_AHB1Periph_GPIOF,ENABLE);

	//LEDS
	GPIO_InitTypeDef a;
	GPIO_InitTypeDef * GPIO_InitStruct;
	GPIO_InitStruct = &a;

	GPIO_InitStruct->GPIO_Pin = GPIO_Pin_7;// | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStruct->GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct->GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStruct->GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct->GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOF,GPIO_InitStruct);

	GPIO_InitStruct->GPIO_Pin = GPIO_Pin_8;// | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStruct->GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct->GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStruct->GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct->GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOF,GPIO_InitStruct);


	GPIO_InitStruct->GPIO_Pin = GPIO_Pin_9;// | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStruct->GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct->GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStruct->GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct->GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOF,GPIO_InitStruct);

	GPIO_InitStruct->GPIO_Pin = GPIO_Pin_6;// | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStruct->GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct->GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStruct->GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct->GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOF,GPIO_InitStruct);
}

void led_on(Led_TypeDef led)
{
	if(led==red)
	{
		GPIO_SetBits(GPIOF,GPIO_Pin_8);
	}
	if(led==yellow)
	{
		GPIO_SetBits(GPIOF,GPIO_Pin_7);
	}
	if(led==green1)
	{
		GPIO_SetBits(GPIOF,GPIO_Pin_6);
	}
	if(led==green2)
	{
		GPIO_SetBits(GPIOF,GPIO_Pin_9);
	}
}



void led_off(Led_TypeDef led)
{
	if(led==red)
	{
		GPIO_ResetBits(GPIOF,GPIO_Pin_8);
	}
	if(led==yellow)
	{
		GPIO_ResetBits(GPIOF,GPIO_Pin_7);
	}
	if(led==green1)
	{
		GPIO_ResetBits(GPIOF,GPIO_Pin_6);
	}
	if(led==green2)
	{
		GPIO_ResetBits(GPIOF,GPIO_Pin_9);
	}
}

void toggle_led(Led_TypeDef led)
{
	if(led==red)
	{
		GPIO_ToggleBits(GPIOF,GPIO_Pin_8);
	}
	if(led==yellow)
	{
		GPIO_ToggleBits(GPIOF,GPIO_Pin_7);
	}
	if(led==green1)
		{
			GPIO_ToggleBits(GPIOF,GPIO_Pin_6);
		}
	if(led==green2)
		{
			GPIO_ToggleBits(GPIOF,GPIO_Pin_9);
		}
}

void Delay_us(unsigned int us )
{
	TIM_SetCounter(TIM6, 0);
	while(TIM_GetCounter(TIM6) < (us/2) ){};
}

