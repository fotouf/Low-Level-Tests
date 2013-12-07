/*
  ******************************************************************************
  * Configures the USART3 peripheral and the DMA1 controller, in order to build
  * the communication interface between the microcontroller board  and the IMU.
  ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_dma.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_gpio.h"
#include "misc.h"

#include "Serials.h"
#include "extern.h"
#include "Other.h"

/* Private typedef -----------------------------------------------------------*/
DMA_InitTypeDef  DMA_InitStructure;
NVIC_InitTypeDef  NVIC_InitStruct;

/*IMU stuff */
unsigned char TxMessageIMU[48]={0};
unsigned char RxMessageIMU[MESS_LENGTH_RX]={0};

/* High Level PC stuff */
int helios_stop;
char cmd;
unsigned char RxMessageHELIOS[20];
unsigned char TxMessageHELIOS[200];



/****************************************************************/
/*	Name:		USART_IMU_Config()									*/
/*	Function:	Configures the USART3 peripheral and the DMA1	*/
/* 				controller for the IMU serial interface			*/
/****************************************************************/

void USART_IMU_Config(void)
{
  USART_InitTypeDef USART_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;

  /*--------------Peripheral Clock-------------------------------*/
  /* Enable GPIO clock */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
  /* Enable USART clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
  /* Enable the DMA clock */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);


  /*---------------USART3 GPIO--------------------*/
  /* Connect USART pins */
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource8, GPIO_AF_USART3);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource9, GPIO_AF_USART3);

  /* Configure USART Tx and Rx as alternate functions */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;				//Tx
  GPIO_Init(GPIOD, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;				//Rx
  GPIO_Init(GPIOD, &GPIO_InitStructure);

  /*-------------- USART3 configuration ------------------------*/

  /* USART3 configured as follow:
        - BaudRate = 460800 baud
		- Word Length = 8 Bits
        - one Stop Bit
        - No parity
        - Hardware flow control disabled (RTS and CTS signals)
        - Receive and transmit enabled
  */
  USART_InitStructure.USART_BaudRate = 460800;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USART3, &USART_InitStructure);

  /* -----------DMA controller to manage USART TX and RX DMA request ----------*/

  /* Configure DMA Initialization Structure */
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable ;
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull ;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single ;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;

  DMA_InitStructure.DMA_PeripheralBaseAddr =(uint32_t) (&(USART3->DR)) ;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  /* Configure TX DMA */
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure.DMA_BufferSize = Command_Size ;
  DMA_InitStructure.DMA_Channel = DMA_Channel_4 ;
  DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral ;
  DMA_InitStructure.DMA_Memory0BaseAddr =(uint32_t)TxMessageIMU ;
  DMA_Init(DMA1_Stream3,&DMA_InitStructure);
  /* Configure RX DMA */
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;// DMA_Mode_Normal;
  DMA_InitStructure.DMA_BufferSize = 1;//MESS_LENGTH_RX ;
  DMA_InitStructure.DMA_Channel = DMA_Channel_4 ;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory ;
  DMA_InitStructure.DMA_Memory0BaseAddr =(uint32_t)RxMessageIMU ;
  DMA_Init(DMA1_Stream1,&DMA_InitStructure);


  //------------ USART interrupt handler settings (NOT USED ANYMORE)--------------------------//
  NVIC_InitStruct.NVIC_IRQChannel = USART3_IRQn;		//IMU interrupt handler
  NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
  NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 13;
  NVIC_Init(&NVIC_InitStruct);
  //----------------------------------------------------------------------------------------//

  USART_ClearITPendingBit(USART3,USART_IT_TC);				//Clear possible pending interrupts
  USART_ClearITPendingBit(USART3,USART_IT_RXNE);

  /* Enable USART */
  USART_Cmd(USART3, ENABLE);



  //-------------Receiving DMA interrupt settings------------------------------------------------//
  NVIC_InitStruct.NVIC_IRQChannel = DMA1_Stream1_IRQn;		//IMU receiving interrupt handler
  NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
  NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
  NVIC_Init(&NVIC_InitStruct);

  DMA_ClearITPendingBit(DMA1_Stream1,DMA_IT_TCIF1);

  DMA_ITConfig(DMA1_Stream1,DMA_IT_TC,ENABLE);
  //---------------------------------------------------------------------------------//

  //-------------Transmitting DMA interrupt settings------------------------------------------------//
  NVIC_InitStruct.NVIC_IRQChannel = DMA1_Stream3_IRQn;		//IMU transmitting interrupt handler
  NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
  NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
  NVIC_Init(&NVIC_InitStruct);

  DMA_ClearITPendingBit(DMA1_Stream3,DMA_IT_TCIF3);

  DMA_ITConfig(DMA1_Stream3,DMA_IT_TC,ENABLE);
  //---------------------------------------------------------------------------------//

}


/****************************************************************/
/*	Name:		SendMsgIMU()									*/
/*	Function:	Sends a command to the IMU						*/
/****************************************************************/

void SendMsgIMU(unsigned short DataNumber,uint8_t *StartAdress)
{
	/*------------------Build the command packet---------------------*/
	unsigned short Checksummpos;
	*StartAdress++ = SYNCX_IMU;                        // Sync Byte
	*StartAdress++ = STX_IMU;                          // Start of Tx Byte
	StartAdress++;                                     // jump over already set command Byte
	*StartAdress++ = (char)((DataNumber&0xFF00)>>8);  // MSB of Length of Data
	*StartAdress-- = (char)(DataNumber&0x00FF);       // LSB of Length of Data
	--StartAdress;                                     // Set address to command Byte
	Checksummpos = calcCRC(StartAdress,DataNumber+3); // Checksum built from CMD, LENGHT and DATA
	StartAdress += (DataNumber+3);
	*StartAdress++ = (char)((Checksummpos&0xFF00)>>8);
	*StartAdress++ = (char)(Checksummpos&0x00FF);
	*StartAdress = ETX;


	/*--------Before transfer the command to the IMU for continuous sending DO-----*/
	/*--------Enable the DMA for receiving & request to receive------------*/

	DMA_SetCurrDataCounter(DMA1_Stream1, 1);//MESS_LENGTH_RX);

	DMA_Cmd(DMA1_Stream1,ENABLE);
	while(DMA_GetCmdStatus(DMA1_Stream1) != ENABLE);

	USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);

	/*It is enabled in the synchronization step */


	/*-----------------------------------------------------------------------------*/


	/*-------Enable the DMA for transmission & request to transmit----------*/

    DMA_SetCurrDataCounter(DMA1_Stream3, DataNumber + 8);

	DMA_Cmd(DMA1_Stream3,ENABLE);
	while(DMA_GetCmdStatus(DMA1_Stream3) != ENABLE);

	USART_DMACmd(USART3,USART_DMAReq_Tx,ENABLE);

	/*----------------------Wait for completion-----------------------*/
    //while(DMA_GetFlagStatus(DMA1_Stream3,DMA_FLAG_TCIF3) == RESET){};

}


/****************************************************************/
/*	Name:		Start_Continious_Mode_IMU()							*/
/*	Function:	Sends start continuous mode command to the IMU	*/
/****************************************************************/

void Start_Continious_Mode_IMU (void)
{
	TxMessageIMU[2] = SBG_SET_CONTINUOUS_MODE;
	TxMessageIMU[5] = 0x00;     // Permanent (bool)
	TxMessageIMU[6] = 0x01;     // Enable (bool)
	TxMessageIMU[7] = 0x01;     // Divider (unsigned char)
	SendMsgIMU(3, TxMessageIMU);
}

/***************************************************************************/
/* Bezeichnung  :   Stop_Continious_Mode()                                 */
/* Funktion     :   Stoppt den Continuous Mode der IMU                     */
/* Übergabewert :   ---                                                    */
/* Rückgabewert :   ---                                                    */
/***************************************************************************/
void Stop_Continious_Mode(void)
{
    // Sets the IMU after initialization on Continuous Mode

    //volatile AT91PS_USART pUS0 = AT91C_BASE_US0;    // Base Adresse der USART
    TxMessageIMU[2] = SBG_SET_CONTINUOUS_MODE;
    TxMessageIMU[5] = 0x00;     // Permanent (bool)
    TxMessageIMU[6] = 0x00;     // Enable (bool)
    TxMessageIMU[7] = 0x01;     // Divider (unsigned char)
    //pUS0->US_RCR = 9; 								// Receive Counter Register (#receive transfers to be performed)

    SendMsgIMU(3, TxMessageIMU);       // Set Continuous Mode
}


/**************************************************************************** /
/ * Description: calcCRC () 												* /
/ * Function: Calculates the CRC from CMD Length and Data 					* /
/ * Transfer value: char * pBuffer -> value where the test is scheduled to begin
/ * Char buffer size -> Number of bytes to be tested for the checksum 		* /
/ * Return value: unsigned short -> CRC value 								* /
/ ****************************************************************************/
unsigned short calcCRC(unsigned char *pBuffer, unsigned short bufferSize)
{
    unsigned short  poly = 0x8408;
    unsigned short  crc = 0;
    unsigned char   carry = 0;
    unsigned char   i_bits = 0;
    unsigned short  j = 0;

    for (j=0; j<bufferSize; j++)
    {
        crc = crc ^ pBuffer[j];
        for (i_bits=0; i_bits<8; i_bits++)
        {
            carry = crc & 1;
            crc = crc>>1;
            if (carry)
            {
                crc = crc^poly;
            }
        }
    }
    return crc;
}




/************************************************* **************************/
/* Description: DMA1_Stream3_IRQHandler () 									*/
/* Function: Interrupt routine for DMA1_Stream3 / IMU command transmit handler			    */
/* Transfer value: --- 														*/
/* Returns: --- 															*/
/************************************************* **************************/
void DMA1_Stream3_IRQHandler(void)
{
	//led_on(red);

	USART_DMACmd(USART3,USART_DMAReq_Tx,DISABLE);

	/* It is already disabled but just to be sure*/
	DMA_Cmd(DMA1_Stream3,DISABLE);
	while(DMA_GetCmdStatus(DMA1_Stream3) != DISABLE);

	DMA_ClearFlag(DMA1_Stream3,DMA_FLAG_TCIF3);

}



/************************************************* **************************/
/* Description: DMA1_Stream1_IRQHandler () 									*/
/* Function: Interrupt routine for DMA1_Stream1 / IMU receive handler			    */
/* Transfer value: --- 														*/
/* Returns: --- 															*/
/************************************************* **************************/
void DMA1_Stream1_IRQHandler(void)
{


	//NOTE: When transmission complete, the DMA stream is disabled! In NORMAL mode: No DMA request is served unless the software re-programs the stream and re-enables it!!
	//			In Circular Mode: It is never disabled, so in order to change the data counter you should disable it amd re- enable it.

    /*---------------------------------------------------------------------------------------*/
	static unsigned int state1=0;
	static unsigned int counter=0;

	int shift;

	DMA_Cmd(DMA1_Stream1,DISABLE);
	while(DMA_GetCmdStatus(DMA1_Stream1) != DISABLE);



   switch (state1)
   {
   case 0:
	   if(RxMessageIMU[0]==0xff)
	   {
		   state1=1;
	   }
	   DMA_SetCurrDataCounter(DMA1_Stream1, 1);
	   break;

   case 1:
	   if(RxMessageIMU[0]==0x02)
	   {

		   state1=2;
		   DMA_SetCurrDataCounter(DMA1_Stream1, 58);
	   }
	   else
	   {
		   state1=0;
		   DMA_SetCurrDataCounter(DMA1_Stream1, 1);
	   }
	   break;

   case 2:

	   shift = 2;
	   Read_data_IMU(shift);
	   DMA_SetCurrDataCounter(DMA1_Stream1, 60);
	   state1=3;
	   break;

   case 3:
	   shift=0;
	   if ((RxMessageIMU[0]==0xff)&&(RxMessageIMU[1]==0x02))
	   {

		   Read_data_IMU(shift);
		   DMA_SetCurrDataCounter(DMA1_Stream1, 60);
	   }
	   else
	   {
		   //led_off(yellow);
		   state1=0;
		   DMA_SetCurrDataCounter(DMA1_Stream1, 1);
		   //counter=counter+1;
		   //if(counter>1){led_on(yellow);}
	   }
	   break;
   }

   DMA_Cmd(DMA1_Stream1,ENABLE);
   while(DMA_GetCmdStatus(DMA1_Stream1) != ENABLE);

   //------Reset Receiving DMA-----------------------------------------------//

   DMA_ClearFlag(DMA1_Stream1,DMA_FLAG_TCIF1);
   USART_DMACmd(USART3,USART_DMAReq_Rx,ENABLE);
   GPIO_ResetBits(GPIOE,GPIO_Pin_0);

}


/*------------------------------------------------------------*/
/* Reads the IMU data from the memory								*/
/* if the synchronization step in DMA1_Stream1_IRQHandler has found	*/
/* the Sync byte and the Start byte									*/
/*------------------------------------------------------------*/
void Read_data_IMU(int shift)
{

	unsigned int i;
	unsigned short lenght;
	unsigned short crc;
	char *qW_ptr = (char*)&Sensor_val.qW, *qX_ptr = (char*)&Sensor_val.qX, *qY_ptr = (char*)&Sensor_val.qY, *qZ_ptr = (char*)&Sensor_val.qZ;
	char *Gx_ptr = (char*)&Sensor_val.gyro_x, *Gy_ptr = (char*)&Sensor_val.gyro_y, *Gz_ptr = (char*)&Sensor_val.gyro_z;
	char *eulx_ptr = (char*)&Sensor_val.theta_x, *euly_ptr = (char*)&Sensor_val.theta_y, *eulz_ptr = (char*)&Sensor_val.theta_z;
	char *p_acc_x = (char*)&Sensor_val.acc_x;
	char *p_acc_y = (char*)&Sensor_val.acc_y;
	char *p_acc_z = (char*)&Sensor_val.acc_z;

	lenght = (((short)RxMessageIMU[3-shift]<<8) | (short)RxMessageIMU[4-shift]);            // Length extract
	crc = (((unsigned short)RxMessageIMU[lenght+5-shift]<<8) |
			(unsigned short)RxMessageIMU[lenght+1+5-shift]);                          // CRC extract

//	if (lenght>500){led_on(red);}
	if(crc == calcCRC(&RxMessageIMU[2-shift], lenght+3))                              // Crc test
	{

		switch(RxMessageIMU[2-shift])                                                 // Determine command
		{
		case SBG_CONTINIOUS_DEFAULT_OUTPUT:

			led_on(green2);
			for(i = 4; i > 0; i--)      // 4 byte load in float value
			{
				*qW_ptr++ = RxMessageIMU[4+i-shift];
				*qX_ptr++ = RxMessageIMU[8+i-shift];
				*qY_ptr++ = RxMessageIMU[12+i-shift];
				*qZ_ptr++ = RxMessageIMU[16+i-shift];
				*eulx_ptr++ = RxMessageIMU[20+i-shift];
				*euly_ptr++ = RxMessageIMU[24+i-shift];
				*eulz_ptr++ = RxMessageIMU[28+i-shift];
				*Gx_ptr++ = RxMessageIMU[32+i-shift];
				*Gy_ptr++ = RxMessageIMU[36+i-shift];
				*Gz_ptr++ = RxMessageIMU[40+i-shift];
				*p_acc_x++ = RxMessageIMU[44+i-shift];
				*p_acc_y++ = RxMessageIMU[48+i-shift];
				*p_acc_z++ = RxMessageIMU[52+i-shift];
			}
			Sensor_val.theta_x = -Sensor_val.theta_x;           // Due to orientation AS prototype
			Sensor_val.gyro_x = -Sensor_val.gyro_x;
			Sensor_val.theta_z = -Sensor_val.theta_z;           // Due to orientation AS prototype
			Sensor_val.gyro_z = -Sensor_val.gyro_z;

			GPIO_SetBits(GPIOE,GPIO_Pin_0);

			IMU_data_for_PC = 1;
//			if ((Sensor_val.theta_x <(0.2))&(Sensor_val.theta_x >(0)))
//			{
//				led_on(yellow);
//			}
//			else if((Sensor_val.theta_x >(0.2)))
//			{
//				led_off(yellow);
//			}


//			if ((Sensor_val.theta_y <(0.2))&(Sensor_val.theta_y >(0)))
//			{
//				led_on(yellow);
//			}
//			else if((Sensor_val.theta_y >(0.2)))
//			{
//				led_off(yellow);
//			}
//
//			if ((Sensor_val.theta_z >(0)))
//			{
//				led_on(yellow);
//			}
//			else if((Sensor_val.theta_z <(0)))
//			{
//				led_off(yellow);
//			}

			break;
		default: break;
		}
	}


}

/*-----------------------------------------------------------------------------------*/
/*                                       High Level PC                               */
/*-----------------------------------------------------------------------------------*/

/****************************************************************/
/*	Name:		USART_Config()									*/
/*	Function:	Configures the USART6 peripheral and the DMA2	*/
/* 				controller for the High Level serial interface	*/
/****************************************************************/

void USART_PC_Config(void)
{
  USART_InitTypeDef USART_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  NVIC_InitTypeDef NVIC_InitStruct;

  /*--------------Peripheral Clock-------------------------------*/
  /* Enable GPIO clock */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC , ENABLE);		//Tx
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG , ENABLE);		//Rx
  /* Enable USART clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);
  /* Enable the DMA clock */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);

  /*---------------USART6 GPIO--------------------*/
  /* Connect USART pins */
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_USART6);
  GPIO_PinAFConfig(GPIOG, GPIO_PinSource9, GPIO_AF_USART6);

  /* Configure USART Tx and Rx as alternate functions */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;					//Tx
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;					//Rx
  GPIO_Init(GPIOG, &GPIO_InitStructure);

  /*-------------- USART6 configuration ------------------------*/

  /* USART6 configured as follow:
        - BaudRate = 460800 baud
		- Word Length = 8 Bits
        - one Stop Bit
        - No parity
        - Hardware flow control disabled (RTS and CTS signals)
        - Receive and transmit enabled
  */
  USART_InitStructure.USART_BaudRate = 460800;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USART6, &USART_InitStructure);

  /* -----------DMA controller to manage USART TX and RX DMA request ----------*/

  /* Configure DMA Initialization Structure */
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable ;
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull ;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single ;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralBaseAddr =(uint32_t) (&(USART6->DR)) ;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  /* Configure TX DMA */
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure.DMA_BufferSize = TX_BUFFERSIZE;
  DMA_InitStructure.DMA_Channel = DMA_Channel_5 ;
  DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral ;
  DMA_InitStructure.DMA_Memory0BaseAddr =(uint32_t)HELIOS_TX_BUFFER ;
  DMA_Init(DMA2_Stream6,&DMA_InitStructure);
  /* Configure RX DMA */
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_BufferSize = 1;//RX_BUFFERSIZE ;
  DMA_InitStructure.DMA_Channel = DMA_Channel_5 ;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory ;
  DMA_InitStructure.DMA_Memory0BaseAddr =(uint32_t )HELIOS_RX_BUFFER ;
  DMA_Init(DMA2_Stream1,&DMA_InitStructure);

  //----------------USART interrupt handler settings (NOT USED ANYMORE)--------------//
  NVIC_InitStruct.NVIC_IRQChannel = USART6_IRQn;		//High Level PC interrupt handler
  NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
  NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 13;
  NVIC_Init(&NVIC_InitStruct);
  //---------------------------------------------------------------//

  USART_ClearITPendingBit(USART6,USART_IT_TC);				//Clear possible pending interrupts
  USART_ClearITPendingBit(USART6,USART_IT_RXNE);

  /* Enable USART */
  USART_Cmd(USART6, ENABLE);


  //-------------Receiving DMA interrupt settings------------------------------------------------//
  NVIC_InitStruct.NVIC_IRQChannel = DMA2_Stream1_IRQn;		//High Level PC interrupt handler
  NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
  NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
  NVIC_Init(&NVIC_InitStruct);

  DMA_ClearITPendingBit(DMA2_Stream1,DMA_IT_TCIF1);

  DMA_ITConfig(DMA2_Stream1,DMA_IT_TC,ENABLE);
  //---------------------------------------------------------------------------------//

  //-------------Transmitting DMA interrupt settings------------------------------------------------//
    NVIC_InitStruct.NVIC_IRQChannel = DMA2_Stream6_IRQn;		//High Level PC interrupt handler
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
    NVIC_Init(&NVIC_InitStruct);

    DMA_ClearITPendingBit(DMA2_Stream6,DMA_IT_TCIF6);

    DMA_ITConfig(DMA2_Stream6,DMA_IT_TC,ENABLE);
    //---------------------------------------------------------------------------------//



}

/****************************************************************/
/*	Name:		SendMsg()										*/
/*	Function:	Sends a command to the High Level PC			*/
/****************************************************************/

void SendMsgHELIOS(unsigned short DataNumber,uint8_t *StartAdress)
{

	unsigned short Checksummpos;

	// data packet
	*StartAdress++ = SYNCX_H;                          // Sync Byte
	StartAdress++;                                     // Command data frame send
	*StartAdress-- = DataNumber;                      // Length of Data
	Checksummpos = calcModulo256(StartAdress,DataNumber+2);   // Checksum built from CMD, LENGHT and DATA
	StartAdress += (DataNumber+2);
	*StartAdress++ = Checksummpos;
	*StartAdress = ETX;

	DMA_SetCurrDataCounter(DMA2_Stream6, DataNumber + 5);

	/*--------Before transfer the message to the High Level PC DO-----*/
	/*--------Enable the DMA for receiving & request to receive------------*/
	//Note: Just for checking. We send sth using Coolterm.
//
//    DMA_SetCurrDataCounter(DMA2_Stream1, 1);
//
//	DMA_Cmd(DMA2_Stream1,ENABLE);
//	while(DMA_GetCmdStatus(DMA2_Stream1) != ENABLE);
//
//	USART_DMACmd(USART6, USART_DMAReq_Rx, ENABLE);
	//------------------------------------------------------------------------//


	/*-------Enable the DMA for transmission & request to transmit----------*/
	//Note: Message set before main.

	DMA_Cmd(DMA2_Stream6,ENABLE);

	while(DMA_GetCmdStatus(DMA2_Stream6) != ENABLE){}

	USART_DMACmd(USART6,USART_DMAReq_Tx,ENABLE);

}


/****************************************************************/
/*	Name:		Start_Continious_Mode()							*/
/*	Function:	Sends start continuous mode command to the IMU	*/
/****************************************************************/

void Start_Continious_Mode_PC (void)
{
	SendMsgHELIOS(1, TxMessageHELIOS);
}

/***************************************************************************/
/* Description: DMA2_Stream6_IRQHandler () 									*/
/* Function: Interrupt routine for DMA2_Stream6 / PC transmit handler			    */
/* Transfer value: --- 														*/
/* Returns: --- 															*/
/***************************************************************************/
void DMA2_Stream6_IRQHandler(void)
{
	//toggle_led(yellow);
	//led_on(red);

	//USART_DMACmd(USART6,USART_DMAReq_Tx,DISABLE);

	/* It is already disabled but just to be sure*/
	DMA_Cmd(DMA2_Stream6,DISABLE);
	while(DMA_GetCmdStatus(DMA2_Stream6) != DISABLE);

	DMA_ClearFlag(DMA2_Stream6,DMA_FLAG_TCIF6);

}


void DMA2_Stream1_IRQHandler(void)
{
	static unsigned int state1=0;
	static unsigned int counter1=0;

	int shift;

	DMA_Cmd(DMA2_Stream1,DISABLE);
	while(DMA_GetCmdStatus(DMA2_Stream1) != DISABLE);


	switch (state1)
	{
	case 0:
		if(RxMessageHELIOS[0]==SYNCX_H)
		{
			state1=1;
		}
		DMA_SetCurrDataCounter(DMA2_Stream1, RX_BUFFERSIZE-1);
		break;

	case 1:
		shift = 1;
		Read_data_Helios(shift);
		DMA_SetCurrDataCounter(DMA2_Stream1, RX_BUFFERSIZE);
		state1=2;
		break;

	case 2:
		shift=0;
		if ((RxMessageHELIOS[0]==SYNCX_H))
		{
			Read_data_Helios(shift);
			DMA_SetCurrDataCounter(DMA2_Stream1, RX_BUFFERSIZE);
			//led_off(green1);
			//toggle_led(red);
		}
		else
		{
			//led_off(red);
			state1=0;
			DMA_SetCurrDataCounter(DMA2_Stream1, 1);
			//counter1=counter1+1;
			//if(counter1>1){led_off(yellow);}
		}
		break;
	}

	DMA_Cmd(DMA2_Stream1,ENABLE);
	while(DMA_GetCmdStatus(DMA2_Stream1) != ENABLE);

	//------Reset Receiving DMA-----------------------------------------------//

	DMA_ClearFlag(DMA2_Stream1,DMA_FLAG_TCIF1);
	USART_DMACmd(USART6,USART_DMAReq_Rx,ENABLE);
}

/*---------------------------------------------*/
/*  Reads the data sent by the High Level PC    */
/*----------------------------------------------*/

void Read_data_Helios(int shift)
{
	static unsigned int counter2 = 0;
			 char i;
			 unsigned char lenght;
			 unsigned short crc;
			 //unsigned int buffer;
		//	 unsigned int taktverhaeltnis = 0;
			 char *pos_x_ptr = (char*)&Helios_val.position_x_raw, *pos_y_ptr = (char*)&Helios_val.position_y_raw, *pos_z_ptr = (char*)&Helios_val.position_z_raw;
			 char *vel_x_ptr = (char*)&Helios_val.velocity_x_raw, *vel_y_ptr = (char*)&Helios_val.velocity_y_raw;
			 char *acc_x_ptr = (char*)&Helios_val.acceleration_x, *acc_y_ptr = (char*)&Helios_val.acceleration_y;



				 helios_stop = 0;                            //Retrieve Helios data

				 lenght = (RxMessageHELIOS[2-shift]);                              // Länge extrahieren
				 crc = ((((unsigned short)RxMessageHELIOS[17-shift])<<8) |
						 (unsigned short)RxMessageHELIOS[18-shift]);               // CRC extrahieren

				 if(crc == calcCRC(&RxMessageHELIOS[1-shift], lenght+2))                   // Crc testen
				 {
					 cmd = RxMessageHELIOS[1-shift];
					 switch(RxMessageHELIOS[1-shift] & 0x0F)                               // Control Mode
					 {
					 case 0x00:  for(i = 0; i < 4; i++)                          // position control
					 { //toggle_led(red);
						 *pos_x_ptr++ = RxMessageHELIOS[3+i-shift];
						 *pos_y_ptr++ = RxMessageHELIOS[7+i-shift];
						 *pos_z_ptr++ = RxMessageHELIOS[11+i-shift];
					 }
					 if(Control_Mode != C_POSITION)
					 {
						 phi_dot.phi_x = 0;
						 phi_dot.phi_y = 0;
						 old_phi_x_soll = 0;
						 old_phi_y_soll = 0;
					 }
					 Control_Mode = C_POSITION;
					 //		    	                              led_on(green2);
					 //		    	                              led_off(yellow);
					 break;

					 case 0x01:  for(i = 0; i < 4; i++)                          // velocity control
					 {
						 *vel_x_ptr++ = RxMessageHELIOS[3+i-shift];
						 *vel_y_ptr++ = RxMessageHELIOS[7+i-shift];
						 *pos_z_ptr++ = RxMessageHELIOS[11+i-shift];
					 }
					 Control_Mode = C_VELOCITY;
					 break;

					 case 0x02:  for(i = 0; i < 4; i++)                          // acceleration control
					 {
						 *acc_x_ptr++ = RxMessageHELIOS[3+i-shift];
						 *acc_y_ptr++ = RxMessageHELIOS[7+i-shift];
						 *pos_z_ptr++ = RxMessageHELIOS[11+i-shift];
					 }
					 Control_Mode = C_ACCELERATION_WZ;  //Hack weil die anderen Affen sind
					 break;

					 case 0x03:  Control_Mode = C_STOP;                          // stop


					 //		    	             	if(counter2 > 100){led_on(green2);}
					 //
					 //led_on(green2);
					 //counter2=counter2+1;
					 //if(counter2>1000){led_off(green1);}
					 //if(counter2>2000){led_on(green1);}

					 break;

					 case 0x04:  if(Control_Mode != C_FREEZE)
					 {
						 phi_dot.phi_x = 0;
						 phi_dot.phi_y = 0;
						 Helios_val.position_z_raw = Sensor_val.theta_z_abs;
						 Helios_val.position_z = Sensor_val.theta_z_abs;
						 old_theta_z_soll = Sensor_val.theta_z_abs;
					 }
					 Control_Mode = C_FREEZE;                        // perfect position control
					 break;

					 case 0x05: for(i = 0; i < 4; i++)
					 {
						 *vel_x_ptr++ = RxMessageHELIOS[3+i-shift];
						 *vel_y_ptr++ = RxMessageHELIOS[7+i-shift];
						 *pos_z_ptr++ = RxMessageHELIOS[11+i-shift];
					 }
					 Control_Mode = C_ROTONDO;
					 break;

					 case 0x06:  for(i = 0; i < 4; i++)                          // position control
					 {
						 *pos_x_ptr++ = RxMessageHELIOS[3+i-shift];
						 *pos_y_ptr++ = RxMessageHELIOS[7+i-shift];
						 *pos_z_ptr++ = RxMessageHELIOS[11+i-shift];
					 }
					 if(Control_Mode != C_HULL_POSITION)
					 {
						 phi_dot.phi_x = 0;
						 phi_dot.phi_y = 0;
						 old_phi_x_soll = 0;
						 old_phi_y_soll = 0;
					 }
					 Control_Mode = C_HULL_POSITION;
					 break;

					 case 0x07:  Control_Mode = C_ACCELERATION_WZ;                           // stop
					 break;

					 case 0x08:  Control_Mode = C_MODE_4;                            // stop
					 break;
					 case 0x09:  Control_Mode = C_MODE_5;                            // stop
					 break;
					 case 0x0A:  Control_Mode = C_MODE_6;                            // stop
					 break;
					 case 0x0B:  Control_Mode = C_MODE_7;                            // stop
					 break;
					 case 0x0C:  Control_Mode = C_MODE_8;                            // stop
					 break;
					 case 0x0D:  Control_Mode = C_MODE_9;                            // stop
					 break;
					 case 0x0E:  Control_Mode = C_MODE_10;                           // stop
					 break;
					 case 0x0F:  Control_Mode = C_MODE_11;                           // stop
					 break;
					 default:    break;
					 }

					 //	    	             switch(RxMessageHELIOS[1] & 0x30)                               // Hull Mode
					 //	    	             {
					 //	    	             	 case 0x00:  hull_position = RxMessageHELIOS[15];            // evt. oscillation
					 //	    	             	 	 	 	 Hull_Mode = H_OSCILLATION;
					 //	    	             	 	 	 	 timer_parking_down = 0;
					 //	    	             	 	 	 	 timer_parking_up = 0;
					 //	    	             	 	 	 	 break;
					 //
					 //	    	             	 case 0x04:  hull_position = RxMessageHELIOS[15];            // Position movement
					 //	    	             	 	 	 	 Hull_Mode = H_POSITION;
					 //	    	             	 	 	 	 mode_frequenzy = 1;
					 //	    	             	 	 	 	 timer_frequenzy = 0;
					 //	    	             	 	 	 	 timer_parking_down = 0;
					 //	    	             	 	 	 	 timer_parking_up = 0;
					 //
					 //	    	             	 	 	 	 //taktverhaeltnis = (hull_position * 6) + 27031;  // Umrechnung:  hull_position: Wert zwischen 0 - 255
					 //	    	                                                                                     //  -> duty_cycle: Wert zwischen 27031 - 28561
					 //	    	             	 	 	 	 //pPWM_CH0->PWMC_CDTYR = taktverhaeltnis;         // Value for the new duty cycle
					 //	    	             	 	 	 	 break;
					 //
					 //	    	             	 case 0x08:  hull_position = RxMessageHELIOS[15];            // parking
					 //	    	             	 	 	 	 Hull_Mode = H_PARKING;
					 //	    	             	 	 	 	 mode_frequenzy = 1;
					 //	    	             	 	 	 	 timer_frequenzy = 0;
					 //	    	             	 	 	 	 break;
					 //
					 //	    	             	 case 0x0C:  break;                                          // not used
					 //	    	             	 	 	 	 default:    break;
					 //	    	             }
					 //	    	             switch(RxMessageHELIOS[1] & 0xC0)                               // LED Mode
					 //	    	             {
					 //	    	             	 case 0x00:  Led_Mode = L_OFF;                               // off
					 //	    	             	 	 	 	 break;
					 //
					 //	    	             	 case 0x10:  led_value = RxMessageHELIOS[16];                // oscillation
					 //	    	             	 	 	 	 Led_Mode = L_OSCILLATION;
					 //	    	             	 	 	 	 break;
					 //
					 //	    	             	 case 0x20:  led_value = RxMessageHELIOS[16];                // constant
					 //	    	             	 	 	 	 Led_Mode = L_CONSTANT;
					 //	    	             	 	 	 	 break;
					 //
					 //	    	             	 case 0x30:  break;                                          // not used
					 //	    	             	 	 	 	 default:    break;
					 //	    	             }
				 }



}

/************************************************* ************************** /
/ * Description: Send_Sensor_Values_to_HELIOS () * /
/ * Function: Sends all sensor data to HELIOS Board * /
/ * Transfer value: unsigend char Anzhahl  -> Anzhahl to send data bytes * /
/ * Returns: --- * /
/ ************************************************* **************************/

void Send_Sensor_Values_to_HELIOS(unsigned char Anzahl)
{
	//toggle_led(yellow);
    unsigned char i;
    char *ptr_1 = (char*)&Sensor_val.theta_x;
    char *ptr_2 = (char*)&Sensor_val.theta_z_abs;
    char *ptr_3 = (char*)&Sensor_val.theta_dot_x;                                           //Sensor_val.theta_dot_x;
    char *ptr_4 = (char*)&phi_dot.phi_x;
    char *ptr_6 = (char*)&Sensor_val.gyro_x;
    char *ptr_7 = (char*)&Sensor_val.psi_dot_1;
    char *ptr_8 = (char*)&Motor_current_real.I_1;
    char *ptr_9 = (char*)&Helios_val.position_x;
    char *ptr_10 = (char*)&Sensor_val.qW;
    char *ptr_11 = (char*)&Sensor_val.acc_x;
    char *ptr_12 = (char*)&i;//data_check_counter;
 //   char *ptr_13 = ptr_6;

    TxMessageHELIOS[1] = CMD_SEND_DATA_HELIOS;      // Command data frame send

    for(i = 0; i<(2*4); i++)
    {
        TxMessageHELIOS[i+3] = *ptr_1++;            // theta_x, theta_y
    }

    for(i = 0; i<(1*4); i++)
    {
        TxMessageHELIOS[i+11] = *ptr_2++;           // theta_z_abs
    }
    for(i = 0; i<(3*4); i++)
    {
        TxMessageHELIOS[i+15] = *ptr_3++;           // theta_dot_x, theta_dot_y, theta_dot_z
    }
    for(i = 0; i<(10*4); i++)
    {
        TxMessageHELIOS[i+27] = *ptr_4++;           // phi_x, phi_y, phi_z, phi_dot_x, phi_dot_y, phi_dot_z
    }                                               // pos_x, pos_y, pos_dot_x, pos_dot_y
    for(i = 0; i<(3*4); i++)
    {
        TxMessageHELIOS[i+67] = *ptr_6++;           // gyro_x, gyro_y, gyro_z
    }
    for(i = 0; i<(3*4); i++)
    {
        TxMessageHELIOS[i+79] = *ptr_7++;           // psi_dot_1, psi_dot_2, psi_dot_3
    }
    for(i = 0; i<(3*4); i++)
    {
        TxMessageHELIOS[i+91] = *ptr_8++;           // I_1, I_2, I_3
    }
    for(i = 0; i<(7*4); i++)
    {
        TxMessageHELIOS[i+103] = *ptr_9++;          // position_x, position_y, position_z, velocity_x
    }                                               // velocity_y, acceleration_x, acceleration_y
    for(i = 0; i<(1*4); ++i)
    {
        TxMessageHELIOS[i+131] = *ptr_12++;         // data_check_counter
    }
    for(i = 0; i<(4*4); ++i)
    {
        TxMessageHELIOS[i+135] = *ptr_10++;         // qW, qX, qY, qZ
    }
    for(i = 0; i<(3*4); ++i)
    {
        TxMessageHELIOS[i+151] = *ptr_11++;         // acc_x, acc_y, acc_z
    }

    // total 40 floats = 160 byte
    SendMsgHELIOS(Anzahl, HELIOS_TX_BUFFER);
}
















/***************************************************************************/
/* Bezeichnung  :   calcModulo256()                                        */
/* Funktion     :   Berechnet Modulo 256 aus CMD, Lenght und Data          */
/* Übergabewert :   char *pBuffer   -> Wert wo der Test begonnen werden    */
/*                                     soll                                */
/*                  char buffersize -> Anzahl zu testender Byte für die    */
/*                                     checksumme                          */
/* Rückgabewert :   unsigned short  -> CRC Value                           */
/***************************************************************************/
unsigned char calcModulo256(char *pBuffer, unsigned short bufferSize)
{
    unsigned short  buffer = 0;
    unsigned char   modulo = 0;
    int i;

    for(i = 0; i < bufferSize; i++)
    {
        buffer += *pBuffer++;
    }

    modulo = buffer % 256;
    return modulo;
}

//void Delay(__IO uint32_t nCount)
//{
//  while(nCount--)
//  {
//  }
//}


