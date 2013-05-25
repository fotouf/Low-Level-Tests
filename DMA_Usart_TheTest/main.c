////////////////////////////////////////////////////////
///  This is a simple test code for the DMA-USART    ///
////////////////////////////////////////////////////////


#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_dma.h"
#include "stm32f4xx_usart.h"
#include "misc.h"


#include "main.h"



uint8_t TxMessageIMU[48] = {0};
#define IMU_TX_BUFFER           &TxMessageIMU[0]
char lol;
unsigned char temp;

int main(void)
{
	SystemInit();

	led_init();
	led_off(red);

	uint8_t source;
	source=RCC_GetSYSCLKSource();
	//if (source==0x08){led_on(green1);}

	init_usart3();
	Start_Continious_Mode();

    while(1)
    {

		//SendMsgIMU(3, &TxMessageIMU[0]);       // Set Continuous Mode

		Delay(1000000);
	    toggle_led(yellow);
		//if(DMA_GetFlagStatus(DMA1_Stream3,DMA_FLAG_TCIF3) == SET){led_off(green1);}
		if(DMA_GetCurrDataCounter(DMA1_Stream3)<8){toggle_led(green1);}

    }
}


void init_usart3(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef  USART_InitStruct;
	USART_ClockInitTypeDef  USART_ClockInitStruct;
	DMA_InitTypeDef DMA_InitStruct;
	NVIC_InitTypeDef NVIC_InitStruct;


	// Clocks
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);

	//GPIO
	GPIO_DeInit(GPIOD);

	GPIO_PinAFConfig(GPIOD,GPIO_PinSource8,GPIO_AF_USART3);			//Tx
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource9,GPIO_AF_USART3); 		//Rx

	GPIO_StructInit(&GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;						//Tx
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;					//No Pull
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;						//Rx
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;			    //No Pull
	GPIO_Init(GPIOD, &GPIO_InitStructure);


	// Usart
	USART_DeInit(USART3);

	USART3 -> CR3 |= 1<<4;			//NACK transmission in case of parity error is enabled
	USART_ClockStructInit(&USART_ClockInitStruct);			//clock disable, CPOL low,CPHA 1edge, Last-bit disable
	USART_ClockInit(USART3,&USART_ClockInitStruct);
	USART_InitStruct.USART_BaudRate = 9600;//Bps			//460800bps = requested baud rate in the previous board
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;
	USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_InitStruct.USART_StopBits = USART_StopBits_1 ;
	USART_InitStruct.USART_Parity = USART_Parity_No  ;
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;			//no CTS,RTS
	USART_Init(USART3,&USART_InitStruct);

	USART3-> CR1 &= ~(0x000001F0);			//disables interrupts of CR1
	USART3-> CR2 &= ~(0x00000040);			//disables all interrupts of CR2
	USART3->CR3  &= ~(0x00000401);			//disables all interrupts of CR3

	//// DMA

	//Clock

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1,ENABLE);

	//Only Transmit
	DMA_DeInit(DMA1_Stream3);						//deinitializes the DMA1 stream 1 registers to each default values

	DMA_StructInit(&DMA_InitStruct);
	DMA_InitStruct.DMA_Channel = DMA_Channel_4 ;
	DMA_InitStruct.DMA_PeripheralBaseAddr = (u32)(&(USART3->DR));//USART3_DATA_REGISTER; //Address of peripheral the DMA must map to
	DMA_InitStruct.DMA_Memory0BaseAddr = (u32)IMU_TX_BUFFER; //Array[8] Variable to which ADC values will be stored
	DMA_InitStruct.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	DMA_InitStruct.DMA_BufferSize = 0x08;
	DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte ; //type = unsigned char
	DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStruct.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStruct.DMA_Priority = DMA_Priority_VeryHigh;			//Same priority as receive-store, so hardware handles priority(the stream with the lower number takes priority over the stream with the higher number)
	DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Enable;
	DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_InitStruct.DMA_MemoryBurst = DMA_MemoryBurst_INC8;
	DMA_InitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
	DMA_Init(DMA1_Stream3,&DMA_InitStruct);
																//a stream may remain disabled if a configuration parameter is wrong
	DMA_FlowControllerConfig(DMA1_Stream3,DMA_FlowCtrl_Memory); //DMA controller is the flow controller//???do not know if usart supports this???
	DMA_SetCurrDataCounter(DMA1_Stream3,8);		//configures number data of data to be transferred

	// ! enable usart-dma interface
	USART_DMACmd(USART3,USART_DMAReq_Rx,DISABLE);	//disable the usart's transmit DMA interface


	//usart interrupts
	NVIC_InitStruct.NVIC_IRQChannel = USART3_IRQn;		//IMU interrupt handler
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 15;
	NVIC_Init(&NVIC_InitStruct);

	USART_ClearITPendingBit(USART3,USART_IT_TC);				//Clear possible pending interrupts
	//USART3->SR &= ~(0x80);

	// ! enable TC interrupt
	//USART_ITConfig(USART3,USART_IT_TC,ENABLE);


	USART_Cmd(USART3,ENABLE);					// Enables the usart 3 peripheral


}



void SendMsgIMU(unsigned short DataNumber,uint8_t *StartAdress)
{
	unsigned int data_number=0;

	// data packet
    *StartAdress++ = 0x01;
    *StartAdress++ = 0x02;
    *StartAdress++ = 0x03;
    *StartAdress++ = 0x04;
    *StartAdress++ = 0x05;
    *StartAdress++ = 0x06;
    *StartAdress++ = 0x07;
    *StartAdress++ = 0x08;
    *StartAdress   = 0x09;

    DMA_SetCurrDataCounter(DMA1_Stream3, DataNumber + 7);

    USART_ClearITPendingBit(USART3,USART_IT_TC);
    DMA_ClearFlag(DMA1_Stream3,DMA_FLAG_TCIF3);

    //Delay(500);		//Not necessary in normal running(not debug)	// ! wait to be sure that the flag is cleared

    // ! checking if flags are cleared=>cleared
    //if((USART_GetFlagStatus(USART3,USART_FLAG_TC) == RESET) &&(DMA_GetFlagStatus(DMA1_Stream3,DMA_FLAG_TCIF3) == RESET ) ){led_off(green1);}

    //	ideas
    if(USART_GetFlagStatus(USART3,USART_FLAG_TC) == RESET){led_on(yellow);}



	DMA_Cmd(DMA1_Stream3,ENABLE);
	while(DMA_GetCmdStatus(DMA1_Stream3) != ENABLE){toggle_led(yellow);}

	if(DMA_GetFIFOStatus(DMA1_Stream3)==DMA_FIFOStatus_HalfFull){led_on(red);}

	USART_DMACmd(USART3,USART_DMAReq_Tx,ENABLE);

	//if(((USART3->CR3) && (0x00000080)) == 0 ){led_on(green1);}

    // !!!no TC till this point!!!!

    // ! checking for DMA errors => NO ERRORS
    if((DMA_GetFlagStatus(DMA1_Stream3,DMA_FLAG_TEIF3)==SET)|(DMA_GetFlagStatus(DMA1_Stream3,DMA_FLAG_DMEIF3)==SET)|(DMA_GetFlagStatus(DMA1_Stream3,DMA_FLAG_FEIF3)==SET)){led_on(green2);}

    while(USART_GetFlagStatus(USART3,USART_FLAG_TC) == RESET){};

    // ! checking transmission
    //while(DMA_GetFlagStatus(DMA1_Stream3,DMA_FLAG_TCIF3) == RESET)
    while (DMA_GetCurrDataCounter(DMA1_Stream3)==8 )
	{

    	// !!!! There is a TC at this point!!!!
    	//if (USART_GetFlagStatus(USART3,USART_FLAG_TC) == SET){led_off(red);}
        if((DMA_GetFlagStatus(DMA1_Stream3,DMA_FLAG_TEIF3)==SET)|(DMA_GetFlagStatus(DMA1_Stream3,DMA_FLAG_DMEIF3)==SET)|(DMA_GetFlagStatus(DMA1_Stream3,DMA_FLAG_FEIF3)==SET)){led_on(green2);}

    	data_number = DMA1_Stream3->NDTR;
        toggle_led(green2);
		Delay(1000000);
		//if(DMA_GetFlagStatus(DMA1_Stream3,DMA_FLAG_TCIF3) == SET){led_on(green1);}

    }

//    temp=USART3->CR3;
//    if(((USART3->CR3) && (0x00000080)) == 0x00000080 ){led_on(green2);}

    // Ideas
    // checking how many times DMA completed
    //if (DMA_GetFlagStatus(DMA1_Stream3,DMA_FLAG_TCIF3) == SET)
    //{
    	//count_DMA_complete = count_DMA_complete + 1;
    	//toggle_led(red);
    //}

    //DMA_ClearFlag(DMA1_Stream3,DMA_FLAG_TCIF3);

}

void Start_Continious_Mode (void)
{
	SendMsgIMU(1, IMU_TX_BUFFER);
}

///////////////////////////////////////////////////////
//////////////////////////LEDS/////////////////////////

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

////////////////////////////////////////////////////////////////////////////////////////////////////



void Delay(__IO uint32_t nCount)
{
  while(nCount--)
  {
  }
}

void USART3_IRQHandler(void)
{
    USART_ClearFlag(USART3,USART_FLAG_TC);
	Delay(1000000);
	toggle_led(red);
	//USART3->SR &= ~(0x80);


}






