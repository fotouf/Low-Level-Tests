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

#include "main.h"

/* Private typedef -----------------------------------------------------------*/
DMA_InitTypeDef  DMA_InitStructure;


unsigned int counter=0;
unsigned int i;
unsigned int j=0;

unsigned char TxMessageIMU[48]={0};
#define IMU_TX_BUFFER	&TxMessageIMU;

unsigned char RxMessageIMU[MESS_LENGTH_RX]={0};
#define IMU_RX_BUFFER	&RxMessageIMU;

unsigned char SYNCX_IMU= 0xFF;
unsigned char STX_IMU=0x02;


NVIC_InitTypeDef  NVIC_InitStruct;


// Sensor Data
struct _Sensor_Values {
	float qW;
	float qX;
	float qY;
	float qZ;
	float gyro_x;
	float gyro_y;
	float gyro_z;
	float theta_x;
	float theta_y;
	float theta_z;
	float theta_dot_x;
	float theta_dot_y;
	float theta_dot_z;
	float psi_dot_1;
	float psi_dot_2;
	float psi_dot_3;
	int motor_vel_1;
	int motor_vel_2;
	int motor_vel_3;
	int motor_pos_1;
	int motor_pos_2;
	int motor_pos_3;
	short current_EPOS1;
	short current_EPOS2;
	short current_EPOS3;
	float theta_z_abs;
	float theta_dot_x_raw;
	float theta_dot_y_raw;
	float acc_x;
	float acc_y;
	float acc_z;
} Sensor_val;




/****************************************************************/
/*						Main program							*/
/****************************************************************/

int main(void)
{
	/*----------------------System Clock configuration-----------------------*/
	SystemInit();

	/*------Debug LED initialization--------*/
	led_init();
	led_off(red);

	/*------checking the clock-------*/
	uint8_t source;
	source=RCC_GetSYSCLKSource();
	if(source==0x08){led_on(green1);}

	Delay(50000000);

	/*---------Configures the USART3 peripheral and the DMA1 controller-------*/
	USART_Config();

	/*----------Sending start continuous mode command to the IMU--------*/
	//Comments:testing the sending part to see if the IMU responds
	Start_Continious_Mode();

	/*HERE goes the synchronization step*/
	//Synchronisation_step();


	led_on(green2);

	while (1)
	{

	}
}

/****************************************************************/
/*	Name:		USART_Config()									*/
/*	Function:	Configures the USART3 peripheral and the DMA1	*/
/* 				controller for the IMU serial interface			*/
/****************************************************************/

void USART_Config(void)
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


  NVIC_InitStruct.NVIC_IRQChannel = USART3_IRQn;		//IMU interrupt handler
  NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
  NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 13;
  NVIC_Init(&NVIC_InitStruct);

  USART_ClearITPendingBit(USART3,USART_IT_TC);				//Clear possible pending interrupts
  USART_ClearITPendingBit(USART3,USART_IT_RXNE);

  /* Enable USART */
  USART_Cmd(USART3, ENABLE);

  USART_ITConfig(USART3,USART_IT_RXNE,DISABLE);


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
	while(DMA_GetCmdStatus(DMA1_Stream3) != ENABLE){toggle_led(green1);}

	USART_DMACmd(USART3,USART_DMAReq_Tx,ENABLE);

	/*----------------------Wait for completion-----------------------*/
    //while(DMA_GetFlagStatus(DMA1_Stream3,DMA_FLAG_TCIF3) == RESET){};

}


/****************************************************************/
/*	Name:		Start_Continious_Mode()							*/
/*	Function:	Sends start continuous mode command to the IMU	*/
/****************************************************************/

void Start_Continious_Mode (void)
{
	TxMessageIMU[2] = SBG_SET_CONTINUOUS_MODE;
	TxMessageIMU[5] = 0x00;     // Permanent (bool)
	TxMessageIMU[6] = 0x01;     // Enable (bool)
	TxMessageIMU[7] = 0x01;     // Divider (unsigned char)
	SendMsgIMU(3, TxMessageIMU);
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



///****************************************************************************/
///* Description: USART3_IRQHandler() 									*/
///* Function: Interrupt routine for USART3 / IMU receive handler			    */
///*							Simple Version									*/
///****************************************************************************/
//
//void USART3_IRQHandler(void)
//{
//
//	/*Just checking if we get sth from the IMU*/
//	led_on(yellow);
//	USART_ClearFlag(USART3,USART_FLAG_RXNE);
//	while(DMA_GetFlagStatus(DMA1_Stream1,DMA_FLAG_TCIF1)==RESET){}
//	led_off(green2);
//	for (j = 0 ;j < 10 ; j++)
//	{
//		if (RxMessageIMU[j] == 0x0031){led_on(red);}
//
//	}
//
//	/*checking if we get messages more than one time*/
////	USART_ClearFlag(USART3,USART_FLAG_RXNE);
////	counter = counter + 1;
////	if (counter==7){led_on(green2);}
//	DMA_ClearFlag(DMA1_Stream1,DMA_FLAG_TCIF1);
//	USART_DMACmd(USART3, USART_DMAReq_Rx, DISABLE);
//	DMA_Cmd(DMA1_Stream1,DISABLE);
//	while(DMA_GetCmdStatus(DMA1_Stream1) != DISABLE);
//
//	DMA_SetCurrDataCounter(DMA1_Stream1, MESS_LENGTH_RX);
//
//	DMA_Cmd(DMA1_Stream1,ENABLE);
//	while(DMA_GetCmdStatus(DMA1_Stream1) != ENABLE);
//
//	USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);
//}


/*--------------------------------The interrupt handler we should use--------------------------*/

///************************************************* **************************/
///* Description: USART3_RX_Irq_Handler () 									*/
///* Function: Interrupt routine for USART3 / IMU receive handler			    */
///* Transfer value: --- 														*/
///* Returns: --- 															*/
///************************************************* **************************/
//void USART3_IRQHandler(void)
//{
//
//
//    char i;
//    unsigned short lenght;
//    unsigned short crc;
//    char *qW_ptr = (char*)&Sensor_val.qW, *qX_ptr = (char*)&Sensor_val.qX, *qY_ptr = (char*)&Sensor_val.qY, *qZ_ptr = (char*)&Sensor_val.qZ;
//    char *Gx_ptr = (char*)&Sensor_val.gyro_x, *Gy_ptr = (char*)&Sensor_val.gyro_y, *Gz_ptr = (char*)&Sensor_val.gyro_z;
//    char *eulx_ptr = (char*)&Sensor_val.theta_x, *euly_ptr = (char*)&Sensor_val.theta_y, *eulz_ptr = (char*)&Sensor_val.theta_z;
//    char *p_acc_x = (char*)&Sensor_val.acc_x;
//    char *p_acc_y = (char*)&Sensor_val.acc_y;
//    char *p_acc_z = (char*)&Sensor_val.acc_z;
//
//
//    //led_on(yellow);	//let's see if TC hits
//
//    if(USART_GetITStatus(USART3,USART_IT_RXNE) == SET )
//    {
//    led_on(yellow);
//
//    	if(!(USART_GetFlagStatus(USART3,USART_FLAG_ORE)==SET)|(USART_GetFlagStatus(USART3,USART_FLAG_NE)==SET)|(USART_GetFlagStatus(USART3,USART_FLAG_FE)==SET)|(USART_GetFlagStatus(USART3,USART_FLAG_PE)==SET) )
//    	{
//
//    		if((RxMessageIMU[0] == 0xff) && (RxMessageIMU[1] == 0x02))
//    		{
//    			lenght = (((short)RxMessageIMU[3]<<8) | (short)RxMessageIMU[4]);            // Length extract
//    			crc = (((unsigned short)RxMessageIMU[lenght+5]<<8) |
//    					(unsigned short)RxMessageIMU[lenght+1+5]);                          // CRC extract
//                if(crc == calcCRC(&RxMessageIMU[2], lenght+3))                              // Crc test
//                {
//                	switch(RxMessageIMU[2])                                                 // Determine command
//                	{
//                		case SBG_CONTINIOUS_DEFAULT_OUTPUT:   for(i = 4; i > 0; i--)      // 4 byte load in float value
//                												{
//                	                                            	*qW_ptr++ = RxMessageIMU[4+i];
//                	                                                *qX_ptr++ = RxMessageIMU[8+i];
//                	                                                *qY_ptr++ = RxMessageIMU[12+i];
//                	                                                *qZ_ptr++ = RxMessageIMU[16+i];
//                	                                                *eulx_ptr++ = RxMessageIMU[20+i];
//                	                                                *euly_ptr++ = RxMessageIMU[24+i];
//                	                                                *eulz_ptr++ = RxMessageIMU[28+i];
//                	                                                *Gx_ptr++ = RxMessageIMU[32+i];
//                	                                                *Gy_ptr++ = RxMessageIMU[36+i];
//                	                                                *Gz_ptr++ = RxMessageIMU[40+i];
//                	                                                *p_acc_x++ = RxMessageIMU[44+i];
//                	                                                *p_acc_y++ = RxMessageIMU[48+i];
//                	                                                *p_acc_z++ = RxMessageIMU[52+i];
//                	                                             }
//                	                                             Sensor_val.theta_x = -Sensor_val.theta_x;           // Due to orientation AS prototype
//                	                                             Sensor_val.gyro_x = -Sensor_val.gyro_x;
//                	                                             Sensor_val.theta_z = -Sensor_val.theta_z;           // Due to orientation AS prototype
//                	                                             Sensor_val.gyro_z = -Sensor_val.gyro_z;
//
//
//                	                                             if (Sensor_val.theta_x >(0.2)){led_on(red);led_off(yellow);}
//                	                                             if ((Sensor_val.theta_x <(0.2))&(Sensor_val.theta_x >(0))){led_on(yellow);led_off(red);}
//
//                	                                             //--------------Sending data to Helios----------------//
//                	                                             //pUS1->US_TPR = (AT91_REG)&RxMessageIMU[0];        // Starting address of send buffer
//                	                                             //pUS1->US_TCR = MESS_LENGTH_RX;                    // we'll transmit number of data chars via DMA
//                	                                             //pUS1->US_PTCR = AT91C_PDC_TXTEN;                  // enable transmit transfer
//                	                                             //Data_Available |= 0x01;
//                	                                             //-----------------------------------------------------//
//
//
//                	                                             counter = counter +1;
//
//
//                	                                             //debug_led2();
//                	                                             /////////////////////////
//                	                                             ////Request EPOS data////
//                	                                             /////////////////////////
//
////                	                                             EPOS_get_velocity_SDO(1);
////                	                                             EPOS_get_velocity_SDO(2);
////                	                                             EPOS_get_velocity_SDO(4);
////
////                	                                             //disable the usart3-IMU interrupts, while waiting for the EPOS data
////                	                                             USART_ITConfig(USART3,USART_IT_RXNE,DISABLE);
////                	                                             USART_ITConfig(USART3,USART_IT_TC,DISABLE);
////
////                	                                             //Enable Can interrupts
////                	                                             CAN_ITConfig(CANx,CAN_IT_FF0 | CAN_IT_FOV0 | CAN_IT_FMP0 | CAN_IT_FF1 | CAN_IT_FOV1 | CAN_IT_FMP1, ENABLE);
//
//                	                                             break;
//                		default: break;
//                	}
//                }
//    		}
//    	}
//    	else
//    	{
//    		USART3->SR &=~(0x00c0002f);		//clears the status bits for erros and RXNE
//    	}
//
//    	//------Reset Receiving DMA-----------------------------------------------//
//    	DMA_ClearFlag(DMA1_Stream1,DMA_FLAG_TCIF1);
//    	DMA_Cmd(DMA1_Stream1,DISABLE);							// if complete DMA is already disabled, but disable just in case..
//    	DMA_SetCurrDataCounter(DMA1_Stream1,MESS_LENGTH_RX);   // address of DMA input buffer already set at initialization
//    	DMA_Cmd(DMA1_Stream1,ENABLE);
//    	USART_DMACmd(USART3,USART_DMAReq_Rx,ENABLE);
//    	//------------------------------------------------------------------------//
//    }
//    else
//    {
//
//    	USART_ClearFlag(USART3,USART_FLAG_TC);
////
////    	DMA_SetCurrDataCounter(DMA1_Stream3,1);                              // restore the transmit count - clears ENDTX flag
////    	USART_DMACmd(USART3,USART_DMAReq_Tx,DISABLE);               // disable transmit transfer
//
//    	//USART3-> CR1 &=~( 0x40);
//    }
//
//}




/************************************************* **************************/
/* Description: DMA1_Stream3_IRQHandler () 									*/
/* Function: Interrupt routine for DMA1_Stream3 / IMU command transmit handler			    */
/* Transfer value: --- 														*/
/* Returns: --- 															*/
/************************************************* **************************/
void DMA1_Stream3_IRQHandler(void)
{
	led_on(red);

	USART_DMACmd(USART3,USART_DMAReq_Tx,DISABLE);

	/* It is already disabled but just to be sure*/
	DMA_Cmd(DMA1_Stream3,DISABLE);
	while(DMA_GetCmdStatus(DMA1_Stream3) != DISABLE){toggle_led(green1);}

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
    unsigned int i;
    unsigned short lenght;
    unsigned short crc;
    char *qW_ptr = (char*)&Sensor_val.qW, *qX_ptr = (char*)&Sensor_val.qX, *qY_ptr = (char*)&Sensor_val.qY, *qZ_ptr = (char*)&Sensor_val.qZ;
    char *Gx_ptr = (char*)&Sensor_val.gyro_x, *Gy_ptr = (char*)&Sensor_val.gyro_y, *Gz_ptr = (char*)&Sensor_val.gyro_z;
    char *eulx_ptr = (char*)&Sensor_val.theta_x, *euly_ptr = (char*)&Sensor_val.theta_y, *eulz_ptr = (char*)&Sensor_val.theta_z;
    char *p_acc_x = (char*)&Sensor_val.acc_x;
    char *p_acc_y = (char*)&Sensor_val.acc_y;
    char *p_acc_z = (char*)&Sensor_val.acc_z;


//    USART_DMACmd(USART3, USART_DMAReq_Rx, DISABLE);



//    USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);


//    i=0;
//    while(i<(120-1))
//    {
//    	if ((RxMessageIMU[i]==0xff)&&(RxMessageIMU[i+1]==0x02))
//    	{
//    		led_off(green1);
//    	}
//    	i++;
//    }
//
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
	   DMA_Cmd(DMA1_Stream1,ENABLE);
	   while(DMA_GetCmdStatus(DMA1_Stream1) != ENABLE);
	   break;

   case 1:
	   if(RxMessageIMU[0]==0x02)
	   {
		   state1=2;
		   DMA_SetCurrDataCounter(DMA1_Stream1, 58);
		   DMA_Cmd(DMA1_Stream1,ENABLE);
		          while(DMA_GetCmdStatus(DMA1_Stream1) != ENABLE);
	   }
	   else
	   {
		   state1=0;
		   DMA_SetCurrDataCounter(DMA1_Stream1, 1);
		   DMA_Cmd(DMA1_Stream1,ENABLE);
		          while(DMA_GetCmdStatus(DMA1_Stream1) != ENABLE);
	   }
	   break;
   case 2:
	   DMA_SetCurrDataCounter(DMA1_Stream1, 60);
	   DMA_Cmd(DMA1_Stream1,ENABLE);
	          while(DMA_GetCmdStatus(DMA1_Stream1) != ENABLE);
	   state1=3;

	   break;
   case 3:

	   if ((RxMessageIMU[0]==0xff)&&(RxMessageIMU[1]==0x02))
	   {
		   DMA_SetCurrDataCounter(DMA1_Stream1, 60);
		   DMA_Cmd(DMA1_Stream1,ENABLE);
		   while(DMA_GetCmdStatus(DMA1_Stream1) != ENABLE);
		   led_off(green1);
		   led_on(red);
	   }
	   else
	   {
		   led_off(red);
		   state1=0;
		   DMA_SetCurrDataCounter(DMA1_Stream1, 1);
		   DMA_Cmd(DMA1_Stream1,ENABLE);
		   while(DMA_GetCmdStatus(DMA1_Stream1) != ENABLE);
		   counter=counter+1;
		   if(counter>1){led_on(yellow);}
	   }
	   break;

   }


//    if(!(USART_GetFlagStatus(USART3,USART_FLAG_ORE)==SET)|(USART_GetFlagStatus(USART3,USART_FLAG_NE)==SET)|(USART_GetFlagStatus(USART3,USART_FLAG_FE)==SET)|(USART_GetFlagStatus(USART3,USART_FLAG_PE)==SET) )
//    {
//    	i=0;
//    	while(i<=MESS_LENGTH_RX)
//    	{
//    		if ((RxMessageIMU[i]==SYNCX_IMU)&&(RxMessageIMU[i+1]==STX_IMU)){led_off(green1);break;}
//    		i++;
//    	}
//
//    	if((RxMessageIMU[i] == SYNCX_IMU) && (RxMessageIMU[i+1] == STX_IMU))
//    	{
//
//    		led_off(green2);
////    		counter = counter +1;
////    		if (counter>1)
////    		{
////    			led_on(yellow);
////
////    		}
//
//    		lenght = (((short)RxMessageIMU[i+3]<<8) | (short)RxMessageIMU[i+4]);            // Length extract
//    		crc = (((unsigned short)RxMessageIMU[i+lenght+5]<<8) |
//    				(unsigned short)RxMessageIMU[i+lenght+1+5]);                          // CRC extract
//    		if(crc == calcCRC(&RxMessageIMU[i+2], lenght+3))                              // Crc test
//    		{
//
//    			switch(RxMessageIMU[i+2])                                                 // Determine command
//    			{
//    				case SBG_CONTINIOUS_DEFAULT_OUTPUT:
//    													//led_on(yellow);
//    													for(i = 4; i > 0; i--)      // 4 byte load in float value
//    														{
//    														*qW_ptr++ = RxMessageIMU[4+i];
//    														*qX_ptr++ = RxMessageIMU[8+i];
//    														*qY_ptr++ = RxMessageIMU[12+i];
//    														*qZ_ptr++ = RxMessageIMU[16+i];
//    														*eulx_ptr++ = RxMessageIMU[20+i];
//    														*euly_ptr++ = RxMessageIMU[24+i];
//    														*eulz_ptr++ = RxMessageIMU[28+i];
//    														*Gx_ptr++ = RxMessageIMU[32+i];
//    														*Gy_ptr++ = RxMessageIMU[36+i];
//    														*Gz_ptr++ = RxMessageIMU[40+i];
//    														*p_acc_x++ = RxMessageIMU[44+i];
//    														*p_acc_y++ = RxMessageIMU[48+i];
//    														*p_acc_z++ = RxMessageIMU[52+i];
//    														}
//    														Sensor_val.theta_x = -Sensor_val.theta_x;           // Due to orientation AS prototype
//    														Sensor_val.gyro_x = -Sensor_val.gyro_x;
//    														Sensor_val.theta_z = -Sensor_val.theta_z;           // Due to orientation AS prototype
//    														Sensor_val.gyro_z = -Sensor_val.gyro_z;
//
//
//    														//if (Sensor_val.theta_x >(0.2)){led_on(red);led_off(yellow);}
//    														//if ((Sensor_val.theta_x <(0.2))&(Sensor_val.theta_x >(0))){led_on(yellow);led_off(red);}
//
//    														//--------------Sending data to Helios----------------//
//    														//pUS1->US_TPR = (AT91_REG)&RxMessageIMU[0];        // Starting address of send buffer
//    														//pUS1->US_TCR = MESS_LENGTH_RX;                    // we'll transmit number of data chars via DMA
//    														//pUS1->US_PTCR = AT91C_PDC_TXTEN;                  // enable transmit transfer
//    														//Data_Available |= 0x01;
//    														//-----------------------------------------------------//
//
//
//
//
//    														//debug_led2();
//    														/////////////////////////
//    														////Request EPOS data////
//    														/////////////////////////
//
////                	                           			     EPOS_get_velocity_SDO(1);
////                	                                         EPOS_get_velocity_SDO(2);
////                	                                         EPOS_get_velocity_SDO(4);
////
////                	                                         //disable the usart3-IMU interrupts, while waiting for the EPOS data
////                	                                         USART_ITConfig(USART3,USART_IT_RXNE,DISABLE);
////                	                                         USART_ITConfig(USART3,USART_IT_TC,DISABLE);
////
////                	                                         //Enable Can interrupts
////                	                                         CAN_ITConfig(CANx,CAN_IT_FF0 | CAN_IT_FOV0 | CAN_IT_FMP0 | CAN_IT_FF1 | CAN_IT_FOV1 | CAN_IT_FMP1, ENABLE);
//
//    														break;
//    				default: break;
//    			}
//    		}
//    	}
//    }
//    else
//    {
//    	USART3->SR &=~(0x00c0002f);		//clears the status bits for erros and RXNE
//    }

    //------Reset Receiving DMA-----------------------------------------------//

//    DMA_Cmd(DMA1_Stream1,ENABLE);				//Not necessary if DMA circular mode works
   DMA_ClearFlag(DMA1_Stream1,DMA_FLAG_TCIF1);
    USART_DMACmd(USART3,USART_DMAReq_Rx,ENABLE);

}

/*========================================================================================*/
/*==============================leds & delay==============================================*/
/*========================================================================================*/
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

void Delay(__IO uint32_t nCount)
{
  while(nCount--)
  {
  }
}


