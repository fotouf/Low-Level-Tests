/*
  ******************************************************************************
  * Configures the USART6 peripheral and the DMA2 controller, in order to build
  * the communication interface between the microcontroller board  and the High
  * Level PC.
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
typedef enum
{
  green1 = 0,
  red = 1,
  yellow = 2,
  green2 = 3,
} Led_TypeDef;


struct _Helios_Values {
	float position_x;
	float position_y;
	float position_z;
	float velocity_x;
	float velocity_y;
	float acceleration_x;
	float acceleration_y;
	float position_x_raw;
	float position_y_raw;
	float position_z_raw;
	float velocity_x_raw;
	float velocity_y_raw;
} Helios_val;

int helios_stop;
char cmd;

struct _odometry_output {
	float phi_x;
	float phi_y;
	float phi_z;
	float phi_dot_x;
	float phi_dot_y;
	float phi_dot_z;
	float pos_x;
	float pos_y;
	float pos_x_dot;
	float pos_y_dot;
}phi_dot;

// Set point filter old values
float old_phi_dot_x_soll;
float old_phi_dot_y_soll;
float old_theta_z_soll;
float old_phi_x_soll;
float old_phi_y_soll;

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


unsigned char RxMessageHELIOS[20]={0};
unsigned int i;

unsigned char TxMessageIMU[BUFFERSIZE]="USART DMA Example: Communication between two USART using DMA";
#define IMU_TX_BUFFER	&TxMessageIMU;

/* Private function prototypes -----------------------------------------------*/
static void USART_Config(void);
void toggle_led(Led_TypeDef led);
void led_off(Led_TypeDef led);
void led_on(Led_TypeDef led);
void led_init(void);
void Start_Continious_Mode(void);
void SendMsg(unsigned short DataNumber,uint8_t *StartAdress);
void Delay(__IO uint32_t nCount);
unsigned short calcCRC(char *pBuffer, unsigned short bufferSize);



/****************************************************************/
/*						Main program							*/
/****************************************************************/

int main(void)
{
	/*-----init------*/
	helios_stop = 0;

	/*----------------------System Clock configuration-----------------------*/
	SystemInit();

	/*------Debug LED initialization--------*/
	led_init();
	led_off(red);

	/*------checking the clock-------*/
	uint8_t source;
	source=RCC_GetSYSCLKSource();
	if(source==0x08){led_on(green1);}

	/*---------Configures the USART6 peripheral and the DMA2 controller-------*/
	USART_Config();

	/*----------Sending start continuous mode command to the High Level PC--------*/
	//Comments:testing the sending part
	Start_Continious_Mode();

	while (1)
	{
		toggle_led(yellow);
	}
}

/****************************************************************/
/*	Name:		USART_Config()									*/
/*	Function:	Configures the USART6 peripheral and the DMA2	*/
/* 				controller for the High Level serial interface	*/
/****************************************************************/

static void USART_Config(void)
{
  USART_InitTypeDef USART_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  NVIC_InitTypeDef NVIC_InitStruct;

  /*--------------Peripheral Clock-------------------------------*/
  /* Enable GPIO clock */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC , ENABLE);		//Tx
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG , ENABLE);		//Rx
  /* Enable USART clock */
  USARTx_CLK_INIT(RCC_APB2Periph_USART6, ENABLE);
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
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure.DMA_PeripheralBaseAddr =(uint32_t) (&(USART6->DR)) ;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  /* Configure TX DMA */
  DMA_InitStructure.DMA_BufferSize = BUFFERSIZE;
  DMA_InitStructure.DMA_Channel = DMA_Channel_5 ;
  DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral ;
  DMA_InitStructure.DMA_Memory0BaseAddr =(uint32_t)IMU_TX_BUFFER ;
  DMA_Init(DMA2_Stream6,&DMA_InitStructure);
  /* Configure RX DMA */
  DMA_InitStructure.DMA_BufferSize = 0x03 ;
  DMA_InitStructure.DMA_Channel = DMA_Channel_5 ;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory ;
  DMA_InitStructure.DMA_Memory0BaseAddr =(uint32_t )RxMessageHELIOS ;
  DMA_Init(DMA2_Stream1,&DMA_InitStructure);


  NVIC_InitStruct.NVIC_IRQChannel = USART6_IRQn;		//High Level PC interrupt handler
  NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
  NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 13;
  NVIC_Init(&NVIC_InitStruct);

  USART_ClearITPendingBit(USART6,USART_IT_TC);				//Clear possible pending interrupts
  USART_ClearITPendingBit(USART6,USART_IT_RXNE);

  /* Enable USART */
  USART_Cmd(USARTx, ENABLE);

  USART_ITConfig(USART6,USART_IT_RXNE,ENABLE);

}

/****************************************************************/
/*	Name:		SendMsg()										*/
/*	Function:	Sends a command to the High Level PC			*/
/****************************************************************/

void SendMsg(unsigned short DataNumber,uint8_t *StartAdress)
{

	// data packet
//	*StartAdress++ = 0x01;
//	*StartAdress++ = 0x02;
//	*StartAdress++ = 0x03;
//	*StartAdress++ = 0x04;
//	*StartAdress++ = 0x05;
//	*StartAdress++ = 0x06;
//	*StartAdress++ = 0x07;
//	*StartAdress++ = 0x08;
//	*StartAdress   = 0x09;
//	DMA_SetCurrDataCounter(DMA2_Stream6, DataNumber + 8);

	/*--------Before transfer the message to the High Level PC DO-----*/
	/*--------Enable the DMA for receiving & request to receive------------*/
	//Note: Just for checking. We send sth using Coolterm.

    DMA_SetCurrDataCounter(DMA2_Stream1, 3);

	DMA_Cmd(DMA2_Stream1,ENABLE);
	while(DMA_GetCmdStatus(DMA2_Stream1) != ENABLE);

	USART_DMACmd(USART6, USART_DMAReq_Rx, ENABLE);
	//------------------------------------------------------------------------//


	/*-------Enable the DMA for transmission & request to transmit----------*/
	//Note: Message set before main.

	DMA_Cmd(DMA2_Stream6,ENABLE);

	while(DMA_GetCmdStatus(DMA2_Stream6) != ENABLE){}

	USART_DMACmd(USART6,USART_DMAReq_Tx,ENABLE);

	/*----------------------Wait for completion-----------------------*/
    while(USART_GetFlagStatus(USART6,USART_FLAG_TC) == RESET){};
    while(DMA_GetFlagStatus(DMA2_Stream6,DMA_FLAG_TCIF6) == RESET){}

    //checking
    led_on(red);

    /*------------Clear flags and disable DMA for TX to enable it later--------*/
    USART_ClearFlag(USART6,USART_FLAG_TC);
    DMA_ClearFlag(DMA2_Stream6,DMA_FLAG_TCIF6);
    USART_DMACmd(USART6, USART_DMAReq_Tx, DISABLE);

}


/****************************************************************/
/*	Name:		Start_Continious_Mode()							*/
/*	Function:	Sends start continuous mode command to the IMU	*/
/****************************************************************/

void Start_Continious_Mode (void)
{
	SendMsg(1, TxMessageIMU);
}


/****************************************************************************/
/* Description: USART6_IRQHandler() 										*/
/* Function: Interrupt routine for USART6 / High Level PC receive handler	*/
/*							Simple Version									*/
/****************************************************************************/

void USART6_IRQHandler(void)
{
	/*Just checking if we get sth from the High Level PC*/
	led_on(yellow);

	/*checking if we get messages more than one time*/
//	USART_ClearFlag(USART6,USART_FLAG_RXNE);
//	counter = counter + 1;
//	if (counter==7){led_on(green2);}


    /*------------Clear flags and disable DMA for RX to enable it later--------*/
	DMA_ClearFlag(DMA2_Stream1,DMA_FLAG_TCIF1);
	USART_ClearFlag(USART6,USART_FLAG_RXNE);
	USART_DMACmd(USART6, USART_DMAReq_Rx, DISABLE);

}

/*--------------------------------The interrupt handler we should use--------------------------*/

///**************************************************************************/
///* Description: USART6_IRQHandler () 										*/
///* Function: Interrupt routine for USART6 / High Level PC receive handler	*/
///* Transfer value: --- 													*/
///* Returns: --- 															*/
///**************************************************************************/

//void USART6_IRQHandler(void)
//{
//	 char i;
//	 unsigned char lenght;
//	 unsigned short crc;
//	 //unsigned int buffer;
////	 unsigned int taktverhaeltnis = 0;
//	 char *pos_x_ptr = (char*)&Helios_val.position_x_raw, *pos_y_ptr = (char*)&Helios_val.position_y_raw, *pos_z_ptr = (char*)&Helios_val.position_z_raw;
//	 char *vel_x_ptr = (char*)&Helios_val.velocity_x_raw, *vel_y_ptr = (char*)&Helios_val.velocity_y_raw;
//	 char *acc_x_ptr = (char*)&Helios_val.acceleration_x, *acc_y_ptr = (char*)&Helios_val.acceleration_y;
//
//	 if(USART_GetITStatus(USART6,USART_IT_RXNE) == SET )
//	 {
//		 //----//
//		 //debug_led2();		//check if i receive sth from the coolterm
//		 //----//
//
//		 DMA_SetCurrDataCounter(DMA2_Stream1,MESS_LENGTH_RX); // address of DMA input buffer already set in the initialization
//
//		 helios_stop = 0;                            //Retrieve Helios data
//	     if(!(USART_GetFlagStatus(USART6,USART_FLAG_ORE)==SET)|(USART_GetFlagStatus(USART6,USART_FLAG_NE)==SET)|(USART_GetFlagStatus(USART6,USART_FLAG_FE)==SET)|(USART_GetFlagStatus(USART6,USART_FLAG_PE)==SET) )
//	     {
//	    	 if(RxMessageHELIOS[0] == SYNCX_H)                               // Testen ob Richtiges Frame
//	    	 	 {
//	    		 	 lenght = (RxMessageHELIOS[2]);                              // Länge extrahieren
//	    	         crc = ((((unsigned short)RxMessageHELIOS[17])<<8) |
//	    	         (unsigned short)RxMessageHELIOS[18]);               // CRC extrahieren
//
//	    	         if(crc == calcCRC(&RxMessageHELIOS[1], lenght+2))                   // Crc testen
//	    	         {
//	    	        	 cmd = RxMessageHELIOS[1];
//	    	             switch(RxMessageHELIOS[1] & 0x0F)                               // Control Mode
//	    	             {
//	    	             	 case 0x00:  for(i = 0; i < 4; i++)                          // position control
//	    	                             {
//	    	             		 	 	 	 *pos_x_ptr++ = RxMessageHELIOS[3+i];
//	    	             		 	 	 	 *pos_y_ptr++ = RxMessageHELIOS[7+i];
//	    	             		 	 	 	 *pos_z_ptr++ = RxMessageHELIOS[11+i];
//	    	                              }
//	    	                              if(Control_Mode != C_POSITION)
//	    	                              {
//	    	                            	  phi_dot.phi_x = 0;
//	    	                            	  phi_dot.phi_y = 0;
//	    	                            	  old_phi_x_soll = 0;
//	    	                            	  old_phi_y_soll = 0;
//	    	                              }
//	    	                              Control_Mode = C_POSITION;
//	    	                              break;
//
//	    	             	 case 0x01:  for(i = 0; i < 4; i++)                          // velocity control
//	    	             	 	 	 	 {
//	    	             		 	 	 	 *vel_x_ptr++ = RxMessageHELIOS[3+i];
//	    	             		 	 	 	 *vel_y_ptr++ = RxMessageHELIOS[7+i];
//	    	             		 	 	 	 *pos_z_ptr++ = RxMessageHELIOS[11+i];
//	    	             	 	 	 	 }
//	    	             	 	 	 	 Control_Mode = C_VELOCITY;
//	    	             	 	 	 	 break;
//
//	    	             	 case 0x02:  for(i = 0; i < 4; i++)                          // acceleration control
//	    	             	 	 	 	 {
//	    	             		 	 	 	 *acc_x_ptr++ = RxMessageHELIOS[3+i];
//	    	             		 	 	 	 *acc_y_ptr++ = RxMessageHELIOS[7+i];
//	    	             		 	 	 	 *pos_z_ptr++ = RxMessageHELIOS[11+i];
//	    	             	 	 	 	 }
//	    	             	 	 	 	 Control_Mode = C_ACCELERATION_WZ;  //Hack weil die anderen Affen sind
//	    	             	 	 	 	 break;
//
//	    	             	 case 0x03:  Control_Mode = C_STOP;                          // stop
//	    	             	 	 	 	 break;
//
//	    	                 case 0x04:  if(Control_Mode != C_FREEZE)
//	    	                 	 	 	 {
//	    	                	 	 	 phi_dot.phi_x = 0;
//	    	                	 	 	 phi_dot.phi_y = 0;
//	    	                	 	 	 Helios_val.position_z_raw = Sensor_val.theta_z_abs;
//	    	                	 	 	 Helios_val.position_z = Sensor_val.theta_z_abs;
//	    	                	 	 	 old_theta_z_soll = Sensor_val.theta_z_abs;
//	    	                 	 	 	 }
//	    	                 	 	 	 Control_Mode = C_FREEZE;                        // perfect position control
//	    	                 	 	 	 break;
//
//	    	                 case 0x05:  for(i = 0; i < 4; i++)
//	    	                 	 	 	 {
//	    	                	 	 	 *vel_x_ptr++ = RxMessageHELIOS[3+i];
//	    	                	 	 	 *vel_y_ptr++ = RxMessageHELIOS[7+i];
//	    	                	 	 	 *pos_z_ptr++ = RxMessageHELIOS[11+i];
//	    	                 	 	 	 }
//	    	                 	 	 	 Control_Mode = C_ROTONDO;
//	    	                 	 	 	 break;
//
//	    	                 case 0x06:  for(i = 0; i < 4; i++)                          // position control
//	    	                 	 	 	 {
//	    	                	 	 	 *pos_x_ptr++ = RxMessageHELIOS[3+i];
//	    	                	 	 	 *pos_y_ptr++ = RxMessageHELIOS[7+i];
//	    	                	 	 	 *pos_z_ptr++ = RxMessageHELIOS[11+i];
//	    	                 	 	 	 }
//	    	                 	 	 	 if(Control_Mode != C_HULL_POSITION)
//	    	                 	 	 	 {
//	    	                 	 	 		 phi_dot.phi_x = 0;
//	    	                 	 	 		 phi_dot.phi_y = 0;
//	    	                 	 	 		 old_phi_x_soll = 0;
//	    	                 	 	 		 old_phi_y_soll = 0;
//	    	                 	 	 	 }
//	    	                 	 	 	 Control_Mode = C_HULL_POSITION;
//	    	                 	 	 	 break;
//
//	    	                 case 0x07:  Control_Mode = C_ACCELERATION_WZ;                           // stop
//	    	                 	 	 	 break;
//
//	    	                 case 0x08:  Control_Mode = C_MODE_4;                            // stop
//	    	                 	 	 	 break;
//	    	                 case 0x09:  Control_Mode = C_MODE_5;                            // stop
//	    	                 	 	 	 break;
//	    	                 case 0x0A:  Control_Mode = C_MODE_6;                            // stop
//	    	                 	 	 	 break;
//	    	                 case 0x0B:  Control_Mode = C_MODE_7;                            // stop
//	    	                 	 	 	 break;
//	    	                 case 0x0C:  Control_Mode = C_MODE_8;                            // stop
//	    	                 	 	 	 break;
//	    	                 case 0x0D:  Control_Mode = C_MODE_9;                            // stop
//	    	                 	 	 	 break;
//	    	                 case 0x0E:  Control_Mode = C_MODE_10;                           // stop
//	    	                 	 	 	 break;
//	    	                 case 0x0F:  Control_Mode = C_MODE_11;                           // stop
//	    	                 	 	 	 break;
//	    	                 default:    break;
//	    	             }
//
////	    	             switch(RxMessageHELIOS[1] & 0x30)                               // Hull Mode
////	    	             {
////	    	             	 case 0x00:  hull_position = RxMessageHELIOS[15];            // evt. oscillation
////	    	             	 	 	 	 Hull_Mode = H_OSCILLATION;
////	    	             	 	 	 	 timer_parking_down = 0;
////	    	             	 	 	 	 timer_parking_up = 0;
////	    	             	 	 	 	 break;
////
////	    	             	 case 0x04:  hull_position = RxMessageHELIOS[15];            // Position movement
////	    	             	 	 	 	 Hull_Mode = H_POSITION;
////	    	             	 	 	 	 mode_frequenzy = 1;
////	    	             	 	 	 	 timer_frequenzy = 0;
////	    	             	 	 	 	 timer_parking_down = 0;
////	    	             	 	 	 	 timer_parking_up = 0;
////
////	    	             	 	 	 	 //taktverhaeltnis = (hull_position * 6) + 27031;  // Umrechnung:  hull_position: Wert zwischen 0 - 255
////	    	                                                                                     //  -> duty_cycle: Wert zwischen 27031 - 28561
////	    	             	 	 	 	 //pPWM_CH0->PWMC_CDTYR = taktverhaeltnis;         // Value for the new duty cycle
////	    	             	 	 	 	 break;
////
////	    	             	 case 0x08:  hull_position = RxMessageHELIOS[15];            // parking
////	    	             	 	 	 	 Hull_Mode = H_PARKING;
////	    	             	 	 	 	 mode_frequenzy = 1;
////	    	             	 	 	 	 timer_frequenzy = 0;
////	    	             	 	 	 	 break;
////
////	    	             	 case 0x0C:  break;                                          // not used
////	    	             	 	 	 	 default:    break;
////	    	             }
////	    	             switch(RxMessageHELIOS[1] & 0xC0)                               // LED Mode
////	    	             {
////	    	             	 case 0x00:  Led_Mode = L_OFF;                               // off
////	    	             	 	 	 	 break;
////
////	    	             	 case 0x10:  led_value = RxMessageHELIOS[16];                // oscillation
////	    	             	 	 	 	 Led_Mode = L_OSCILLATION;
////	    	             	 	 	 	 break;
////
////	    	             	 case 0x20:  led_value = RxMessageHELIOS[16];                // constant
////	    	             	 	 	 	 Led_Mode = L_CONSTANT;
////	    	             	 	 	 	 break;
////
////	    	             	 case 0x30:  break;                                          // not used
////	    	             	 	 	 	 default:    break;
////	    	             }
//	    	         }
//	    	 	 }
//	     	 }
//	 }
//
//}

/*-------------------------------------------------------------------------------------------*/

/**************************************************************************** /
/ * Description: calcCRC () 												* /
/ * Function: Calculates the CRC from CMD Length and Data 					* /
/ * Transfer value: char * pBuffer -> value where the test is scheduled to begin
/ * Char buffer size -> Number of bytes to be tested for the checksum 		* /
/ * Return value: unsigned short -> CRC value 								* /
/ ****************************************************************************/
unsigned short calcCRC(char *pBuffer, unsigned short bufferSize)
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
