#include "stm32f4xx_can.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "misc.h"

#include "CAN.h"
#include "extern.h"
#include "Defines_ballbot.h"
#include "Other.h"




#define CANx CAN1

CanRxMsg RxMsg3;




/****************************************************************************/
/* Name	        :	CAN1_RX0_IRQHandler()									*/
/* Function		:	Handling the Interrupts of FIFO 0 of the CAN interface,	*/
/* 					FIFO 0 is configured to receive messages from EPOS 1 	*/
/* 					and 2													*/
/****************************************************************************/

void CAN1_RX0_IRQHandler(){

CanRxMsg RxMessage;
CanRxMsg* RxMsg;
RxMsg = &RxMessage;
volatile long DataLow;
volatile long DataHigh;

/*-----During initialization, just reading the FIFO to clear it-----*/
if (init==1)
{
	//toggle_led(green2);
	//Delay(10000);
	CAN_Receive(CAN1,CAN_FIFO0,&RxMsg3);
	//CAN_Receive(CAN1,CAN_FIFO1,&RxMsg3);
}

/*-------------------------After initialization--------------------*/

if ((CAN_GetLastErrorCode(CAN1)== CAN_ErrorCode_NoErr)&(init==0))		//If No error occurred
	{

	if (CAN_GetFlagStatus(CANx,CAN_FLAG_FMP0) == SET)		// check FIFO_0
	{
		//toggle_led(red);

		CAN_Receive(CANx,CAN_FIFO0,RxMsg);					//Read the message

		DataLow = RxMsg -> Data[3]<<24 |RxMsg -> Data[2]<<16 |RxMsg -> Data[1]<<8 | RxMsg -> Data[0];
		DataHigh = RxMsg -> Data[7]<<24 |RxMsg -> Data[6]<<16 |RxMsg -> Data[5]<<8 | RxMsg -> Data[4];

		/*---------------Check the Filter Match Index to see which filter is activated-----------*/
		/*----------------------*/
		/*	FIFO 0				*/
		/*	FMI = 0 -> EPOS 1	*/
		/*	FMI = 1 -> EPOS 2	*/
		/*----------------------*/
		if ((RxMessage.FMI == 0))// && (available_data.EPOS1 != 1))			//avoid taking a message already taken from the other FIFO mailbox
		{

			switch ( DataLow )
					{
						case RECEIVE_ACT_POS:
							Sensor_val.motor_pos_1 = DataHigh ;
							break;
						case RECEIVE_ACT_VEL:
							Sensor_val.motor_vel_1 = DataHigh;
							led_on(red);
							break;
						case RECEIVE_ACT_CUR:
							Sensor_val.current_EPOS1 = DataHigh;
							break;
						case ACK_POS_SEND:	 	; break;
						case ACK_VEL_SEND:		; break;
						case ACK_CUR_SEND:		; break;
						default: break;
					}
			available_data.EPOS1 = 1;
		}

		if (RxMessage.FMI == 1)// && (available_data.EPOS2 != 1))
		{
			//toggle_led(yellow);

			switch ( DataLow )
			{
				case RECEIVE_ACT_POS :
				Sensor_val.motor_pos_2 = DataHigh ;
					break;
				case RECEIVE_ACT_VEL:
				Sensor_val.motor_vel_2 = DataHigh;
					break;
				case RECEIVE_ACT_CUR:
				Sensor_val.current_EPOS2 = DataHigh;
					break;
				case ACK_POS_SEND:	 	; break;
				case ACK_VEL_SEND:		; break;
				case ACK_CUR_SEND:		; break;
				default: break;
			}
			available_data.EPOS2 = 1;
		}
	}
}
//-----NOTE: At the old board nothing happens if there are errors-----------------//



//if ((available_data.EPOS1 == 1 ) && (available_data.EPOS2 == 1) && (available_data.EPOS3 == 1))
//		{
//		// Odometry and control
//		Gyro_Values_to_Theta_dot();
//		odometry();
//		//kalman();
//
//		setpoint_filter();
//		state_feedback_control();
//		//non_linear_control();
//		//planar_control();
//
//
//		// Send the commands to the EPOS
//		Safety_first();
//		//Motor_current_real.I_1 = 0;
//		//Motor_current_real.I_2 = 0;
//		//Motor_current_real.I_3 = 0;
//		EPOS_set_current_SDO(1,(short)Motor_current_real.I_1);
//		EPOS_set_current_SDO(2,(short)Motor_current_real.I_2);
//		EPOS_set_current_SDO(4,(short)Motor_current_real.I_3);
//
//
//		// Reset the EPOS data available flag
//		available_data.EPOS1=0;
//		available_data.EPOS2=0;
//		available_data.EPOS3=0;
//
//		//Disable CAN interrupt
//		//CANx -> IER &= ~(0x0000007e);
//		CAN_ITConfig(CANx,CAN_IT_FF0 | CAN_IT_FOV0 | CAN_IT_FMP0 | CAN_IT_FF1 | CAN_IT_FOV1 | CAN_IT_FMP1, DISABLE);
//		//Clear pending interrupts
//		CAN_ClearITPendingBit(CANx,CAN_IT_FF0 | CAN_IT_FOV0 | CAN_IT_FF1 | CAN_IT_FOV1);
//
//		//Enable IMU interrupts
//		 USART_ITConfig(USART3,USART_IT_RXNE,ENABLE);
//		 USART_ITConfig(USART3,USART_IT_TC,ENABLE);
//
//		}

}


/****************************************************************************/
/* Name	        :	CAN1_RX1_IRQHandler()									*/
/* Function		:	Handling the Interrupts of FIFO 1 of the CAN interface,	*/
/* 					FIFO 1 is configured to receive messages from EPOS 4 	*/
/****************************************************************************/
void CAN1_RX1_IRQHandler(){

CanRxMsg RxMessage;
CanRxMsg* RxMsg;
RxMsg =&RxMessage;
volatile long DataLow;
volatile long DataHigh;

/*-----During initialization, just reading the FIFO to clear it-----*/
if (init==1)
{
	//toggle_led(green2);
	//Delay(10000);
	//CAN_Receive(CAN1,CAN_FIFO0,&RxMsg3);
	CAN_Receive(CAN1,CAN_FIFO1,&RxMsg3);
}

/*-------------------------After initialization--------------------*/

if ((CAN_GetLastErrorCode(CAN1)==CAN_ErrorCode_NoErr) & (init == 0))		//No error occurred
{

	if (CAN_GetFlagStatus(CANx,CAN_FLAG_FMP1) == SET)		// check FIFO_0
	{
		CAN_Receive(CANx,CAN_FIFO1,RxMsg);
		DataLow = RxMsg -> Data[3]<<24 |RxMsg -> Data[2]<<16 |RxMsg -> Data[1]<<8 | RxMsg -> Data[0];
		DataHigh = RxMsg -> Data[7]<<24 |RxMsg -> Data[6]<<16 |RxMsg -> Data[5]<<8 | RxMsg -> Data[4];

		/*---------------Check the Filter Match Index to see which filter is activated-----------*/
		/*----------------------*/
		/*	FIFO 1				*/
		/*	FMI = 0 -> EPOS 4	*/
		/*----------------------*/

		if (RxMessage.FMI == 0)// && (available_data.EPOS3 != 1))
		{
			//toggle_led(green2);
			switch ( DataLow )
			{
				case RECEIVE_ACT_POS :
					Sensor_val.motor_pos_3 = DataHigh ;
					break;
				case RECEIVE_ACT_VEL:
					Sensor_val.motor_vel_3 = DataHigh;
					break;
				case RECEIVE_ACT_CUR:
					Sensor_val.current_EPOS3 = DataHigh;
					break;
				case ACK_POS_SEND:	 	; break;
				case ACK_VEL_SEND:		; break;
				case ACK_CUR_SEND:		; break;
				default: break;
			}
			available_data.EPOS3 = 1;
		}
	}
}
//-----NOTE: At the old board nothing happens if there are errors-----------------//


//	if ((available_data.EPOS1 == 1 ) && (available_data.EPOS2 == 1) && (available_data.EPOS3 == 1))
//			{
//			// Odometry and control
//			Gyro_Values_to_Theta_dot();
//			odometry();
//			//kalman();
//
//			setpoint_filter();
//			state_feedback_control();
//			//non_linear_control();
//			//planar_control();
//
//
//			// Send the commands to the EPOS
//			Safety_first();
//			//Motor_current_real.I_1 = 0;
//			//Motor_current_real.I_2 = 0;
//			//Motor_current_real.I_3 = 0;
//			EPOS_set_current_SDO(1,(short)Motor_current_real.I_1);
//			EPOS_set_current_SDO(2,(short)Motor_current_real.I_2);
//			EPOS_set_current_SDO(4,(short)Motor_current_real.I_3);
//
//
//			// Reset the EPOS data available flag
//			available_data.EPOS1=0;
//			available_data.EPOS2=0;
//			available_data.EPOS3=0;
//
//			//Disable CAN interrupt
//			//CANx -> IER &= ~(0x0000007e);
//			CAN_ITConfig(CANx,CAN_IT_FF0 | CAN_IT_FOV0 | CAN_IT_FMP0 | CAN_IT_FF1 | CAN_IT_FOV1 | CAN_IT_FMP1, DISABLE);
//			//Clear pending interrupts
//			CAN_ClearITPendingBit(CANx,CAN_IT_FF0 | CAN_IT_FOV0 | CAN_IT_FF1 | CAN_IT_FOV1);
//
//
//			//Enable IMU interrupts
//			 USART_ITConfig(USART3,USART_IT_RXNE,ENABLE);
//			 USART_ITConfig(USART3,USART_IT_TC,ENABLE);
//
//			}

}

/***************************************************************************/
/* Name	        :	init_CAN()									           */
/* Function		:	Initialization of the CAN interface					   */
/***************************************************************************/

void init_CAN(void)
{
	/*--------------Definitions-----------------*/
	int i;

	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	CAN_InitTypeDef CAN_InitStruct;
	CAN_FilterInitTypeDef FilterInitTypeDef ;

	CanRxMsg RxMessage;
	CanRxMsg* RxMsg;
	RxMsg = &RxMessage;

	CanTxMsg TxMessage;
	CanTxMsg* TxMsg;
	TxMsg = &TxMessage;

	uint8_t TransmitMailbox;


	/*-------------Initialization--------------*/
	for(i=0;i<8;i++){RxMessage.Data[i] = 0x00;}

	/*------------Peripheral Clock---------------*/
	/*Enable GPIOB clock*/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
	/*Enable CAN1 clock*/
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1 , ENABLE);

	/*---------------CAN GPIO--------------------*/
	/* Connect CAN pins */
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_CAN1); //PB8-139 pin Rx
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_CAN1); //PB9-140 pin Tx
	GPIO_StructInit(&GPIO_InitStructure);

	/*Configure CAN Tx and Rx as alternate function*/
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;		//Rx
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;		//Tx
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/*------------Interrupt Configuration--------*/
	/*Disable all CAN interrupts*/
	CANx -> IER = 0x00000000;

	/*Set up CAN interrupts*/
	NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;				//CAN FIFO 0 interrupt handler
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 14;
	NVIC_Init(&NVIC_InitStructure);
	NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX1_IRQn;				//CAN FIFO 1 interrupt handler
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 14;
	NVIC_Init(&NVIC_InitStructure);

	/*------------CAN Peripheral Configuration------------*/

	/*Timing configuration*/
	CAN_InitStruct.CAN_SJW = CAN_SJW_3tq;		//Resynchronization Jump Width
	CAN_InitStruct.CAN_BS1 = CAN_BS1_8tq;		//Bit Timing Segment 1		(TS1 = BS1-1)
	CAN_InitStruct.CAN_BS2 = CAN_BS2_5tq;		//Bit Timing Segment 2		(TS2 = BS2-1)
	CAN_InitStruct.CAN_Prescaler =3;			//Prescaler					(BRP = Prescaler-1)

	//Priority by identifier,Automatically retransmit till successful,Normal mode,CAN working during debug
	CANx-> MCR = 0x00000000;

	/*Setting Normal Mode and Initializing*/
	CAN_InitStruct.CAN_Mode = CAN_Mode_Normal;
	CAN_Init(CAN1,&CAN_InitStruct);

	/*Clear Error Flag*/
	CAN_ClearFlag(CAN1,CAN_FLAG_LEC);


	/*------------------------------- Filters------------------------------------*/

	/*----------------------------FIFO0-----------------------*/

	/* --- Filter 0->EPOS1 & Filter 1->EPOS2------*/
	FilterInitTypeDef.CAN_FilterActivation=ENABLE;
	FilterInitTypeDef.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;
	FilterInitTypeDef.CAN_FilterIdHigh = 0xB040;	//0x582,rtr=data=0,ide=standard_id=0,exid=000	high->FMI=1
	FilterInitTypeDef.CAN_FilterIdLow = 0xB020;		//0x581,rtr=data=0,ide=standard_id=0,exid=000	low->FMI=0
	FilterInitTypeDef.CAN_FilterMaskIdHigh =0xFFFF;
	FilterInitTypeDef.CAN_FilterMaskIdLow = 0xFFFF;
	FilterInitTypeDef.CAN_FilterMode = CAN_FilterMode_IdMask;
	FilterInitTypeDef.CAN_FilterNumber = 0;
	FilterInitTypeDef.CAN_FilterScale = CAN_FilterScale_16bit;
	CAN_FilterInit(&FilterInitTypeDef);
	CAN_Receive(CANx,CAN_FIFO0,RxMsg);


	/*-----------------------------FIFO1---------------------*/

	/*----Filter 0 -> EPOS3----*/
	FilterInitTypeDef.CAN_FilterActivation=ENABLE;
	FilterInitTypeDef.CAN_FilterFIFOAssignment = CAN_Filter_FIFO1;
	FilterInitTypeDef.CAN_FilterIdHigh = 0x0000;	//no such ID in the CAN network,so this filter is never activated	high->FMI=3
	FilterInitTypeDef.CAN_FilterIdLow = 0xB080;		//0x584,rtr=data=0,ide=standard_id=0,exid=000	low->FMI=2
	FilterInitTypeDef.CAN_FilterMaskIdHigh =0xFFFF;
	FilterInitTypeDef.CAN_FilterMaskIdLow = 0xFFFF;
	FilterInitTypeDef.CAN_FilterMode = CAN_FilterMode_IdMask;
	FilterInitTypeDef.CAN_FilterNumber = 2;
	FilterInitTypeDef.CAN_FilterScale = CAN_FilterScale_16bit;
	CAN_FilterInit(&FilterInitTypeDef);
	CAN_Receive(CANx,CAN_FIFO1,RxMsg);


//	CAN_FIFORelease(CAN1,CAN_FIFO0);
//	CAN_FIFORelease(CAN1,CAN_FIFO1);


	/*Enable CAN interrupt*/
	CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);
	CAN_ITConfig(CAN1,CAN_IT_FMP1,ENABLE);

//	//Checking if CAN controller woke up
//	while( (CANx->MSR & (0x1 << 3) )!= (0x1 << 3)){;}

}


/***************************************************************************/
/* Name	        :	can_set_send_PDO_mb()								   */
/* Function		:	Sets the MB3 for a transmit command					   */
/* Transfer Value:															*/
/***************************************************************************/

void can_set_send_PDO_mb(unsigned char lenght, unsigned int id, unsigned char rtr)
{
	CanTxMsg TxMessage;

	TxMessage.StdId = id ;											// CAN Message Mode Register- it has the highest priority, as it has the lowest id value
	TxMessage.IDE = CAN_Id_Standard ;									//Specify standard type identifier
	if (rtr)
		{
		TxMessage.RTR = CAN_RTR_Remote ;										//Specify type remote frame
		}
	else{
		TxMessage.RTR = CAN_RTR_Data ;										//Specify type data frame
		TxMessage.DLC = lenght ;												//Specify the length of the frame
		}
}



/***************************************************************************/
/* Name	        :	can_send_PDO()								   */
/* Function		:	Loads the registers with the correct information and transmits the command*/
/* Transfer Value:														   */
/***************************************************************************/

void can_send_PDO(unsigned char *data)
{
	int i;
	uint8_t TransmitMailbox;


	CanTxMsg TxMessage;
	CanTxMsg* TxMsg;
	TxMsg = &TxMessage;

	for (i = 0; i<=7 ; i++){TxMessage.Data[i] = (int) data[i];}			//loads the data

	TransmitMailbox = CAN_Transmit(CANx,TxMsg);						//Sends the message if there is empty mailbox and returns the number of the mailbox

}


/***************************************************************************/
/* Name	        :	can_send_SDO()								   */
/* Function		:	Loads the registers with the correct data and transmits the command*/
/* Transfer Value:		* data -> pointer to data registers 				*/
/* 					Node -> ID number of EPOS 1,2,4 						*/
/* 					Rtr -> 1 = remote frame, 0 = normal frame				*/
/* 					Length -> length of the message					   		*/
/***************************************************************************/

void can_send_SDO(unsigned char *data, unsigned char node, unsigned char rtr, unsigned char lenght)
{
	CanTxMsg TxMessage;
	CanTxMsg* TxMsg;
	TxMsg =&TxMessage;

	uint8_t TransmitMailbox;
	int i;

	switch(node)												  			 // Switch EPOS
	{
		case 1:
				TxMessage.StdId = 0x601;										//EPOS 1 id
				break;
		case 2:
				TxMessage.StdId = 0x602;										//EPOS 2 id
				break;
		case 4:
				TxMessage.StdId = 0x604;										//EPOS 3 id
				break;
		default: break;
	}
	TxMessage.IDE = CAN_Id_Standard ;

	for (i = 0; i<=7 ; i++){TxMessage.Data[i] = (int) data[i];}			//loads the data

	if (rtr==1)
			{
			TxMessage.RTR = CAN_RTR_Remote ;										//Specify type remote frame
			TransmitMailbox = CAN_Transmit(CANx,&TxMessage);						//Sends the message if there is empty mailbox and returns the number of the mailbox
			}
		else{
			TxMessage.RTR = CAN_RTR_Data ;										//Specify type data frame
			TxMessage.DLC = lenght ;												//Specify the length of the frame
			TransmitMailbox = CAN_Transmit(CANx,&TxMessage);						//Sends the message if there is empty mailbox and returns the number of the mailbox
			}

}



