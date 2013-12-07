
#include "stm32f4xx_can.h"
#include "stm32f4xx_rcc.h"


#include "EPOS.h"
#include "extern.h"
#include "CAN.h"
#include "Other.h"

/***************************************************************************/
/* Name	        :	Set_Operational_NMT_State()							   */
/* Function		:	Sets EPOS in the pre-operational mode, in this mode are allowed PDOs   */
/*Transfer value: node -> EPOS*/
/***************************************************************************/

void Set_Operational_NMT_State(char node){
	unsigned char data[2];

	data[0]= 0x01;
	data[1]= node;

	can_send_SDO(&data[0], node, 0, 2);				// Sends the message to node

	Delay_us(1000);//_delay_ms(1);

}

/***************************************************************************/
/* Name	        :	Set_Pre_Operational_NMT_State()						   */
/* Function		:	Sets EPOS in the pre-operational mode 					*/
/* 				In this mode, no PDO's are allowed, only SDO    			*/
/*Transfer value: node -> EPOS												*/
/***************************************************************************/

void Set_Pre_Operational_NMT_State(char node){
	unsigned char data[2];

	data[0]= 0x80;
	data[1]= node;

	can_send_SDO(&data[0], node, 0, 2);				// Sends the message to node

	Delay_us(50000);//_delay_ms(10);									//Delay in order to let the EPOS respond before sending the next command
}


/***************************************************************************/
/* Name	        :	EPOS_reset_communicaton()					   */
/* Function		:	All EPOS will reset the Communication					*/
/*Transfer value: node -> EPOS												*/
/***************************************************************************/

void EPOS_reset_communicaton(char node){
	unsigned char data[2];

	data [0]= 0x82;
	data [1]= node;

	can_send_SDO(&data[0], node, 0, 2);				// Sends the message to node
	Delay_us(50000);//_delay_ms(10);									//Delay in order to let the EPOS respond before sending the next command
}


/***************************************************************************/
/* Name	        :	EPOS_set_operation_mode()					   */
/* Function		:	Sets the mode of EPOS					*/
/*Transfer value: node -> EPOS												*/
/*				mode ->  mode of operation see in EPOS.h					*/
/***************************************************************************/

void EPOS_set_operation_mode(char node,char mode){
	unsigned char data[8];

	data[0]= 0x22;
	data[1]= 0x60;
	data[2]= 0x60;
	data[3]= 0x00;
	data[4]= mode;
	data[5]= 0x00;
	data[6]= 0x00;
	data[7]= 0x00;

	can_send_SDO(&data[0], node, 0, 8);				// Sends the message to node

	Delay_us(50000);//_delay_ms(10);									//Delay in order to let the EPOS respond before sending the next command

}


/***************************************************************************/
/* Name	        :	EPOS_fault_reset()					   */
/* Function		:	Clears error flags on the EPOS					*/
/*Transfer value: node -> EPOS												*/
/***************************************************************************/

void EPOS_fault_reset(char node){
	unsigned char data[8];

	data [0]= 0x22;
	data [1]= 0x40;
	data [2]= 0x60;
	data [3]= 0x0;
	data [4]= 0x0F;
	data [5]= 0x00;
	data [6]= 0x00;
	data [7]= 0x00;

	can_send_SDO(&data[0], node, 0, 8);				//  Sends the message to node

	Delay_us(50000);//_delay_ms(10);									//Delay in order to let the EPOS respond before sending the next command

	data [0]= 0x22;
	data [1]= 0x40;
	data [2]= 0x60;
	data [3]= 0x0;
	data [4]= 0x8F;
	data [5]= 0x00;
	data [6]= 0x00;
	data [7]= 0x00;

	can_send_SDO(&data[0], node, 0, 8);				//  Sends the message to node

	Delay_us(50000);//_delay_ms(10);									//Delay in order to let the EPOS respond before sending the next command
}


/***************************************************************************/
/* Name	        :	EPOS_disable()					   */
/* Function		:	Disables the EPOS					*/
/*Transfer value: node -> EPOS												*/
/***************************************************************************/

void EPOS_disable(char node){
	unsigned char data[8];

	data [0]= 0x22;
	data [1]= 0x40;
	data [2]= 0x60;
	data [3]= 0x0;
	data [4]= DISABLE_OPERATION;
	data [5]= 0x00;
	data [6]= 0x00;
	data [7]= 0x00;

	can_send_SDO(&data[0], node, 0, 8);				//Sends the message to node

	Delay_us(1000);//_delay_ms(1);
}


/***************************************************************************/
/* Name	        :	EPOS_enable()					   */
/* Function		:	Enables the EPOS					*/
/*Transfer value: node -> EPOS												*/
/***************************************************************************/

void EPOS_enable(char node){
	unsigned char data[8];

	data [0]= 0x22;
	data [1]= 0x40;
	data [2]= 0x60;
	data [3]= 0x00;
	data [4]= ENABLE_OPERATION;
	data [5]= 0x00;
	data [6]= 0x00;
	data [7]= 0x00;

	can_send_SDO(&data[0], node, 0, 8);				// Sends the message to node
	Delay_us(1000);//_delay_ms(1);
}

/***************************************************************************/
/* Name	        :	EPOS_init()						   */
/* Function		:	EPOS initialization					*/
/*Transfer value: node -> EPOS												*/
/***************************************************************************/

void EPOS_init(char node){
	//EPOS Shut Down
	unsigned char data[8];

	data [0]= 0x22;
	data [1]= 0x40;
	data [2]= 0x60;
	data [3]= 0x00;
	data [4]= SHUTDOWN;
	data [5]= 0x00;
	data [6]= 0x00;
	data [7]= 0x00;

	can_send_SDO(&data[0], node, 0, 8);				// Sends the message to node

	//EPOS Enable operation mode

	Delay_us(50000);//_delay_ms(10);									//Delay in order to let the EPOS respond before sending the next command

	data [0]= 0x22;
	data [1]= 0x40;
	data [2]= 0x60;
	data [3]= 0x00;
	data [4]= ENABLE_OPERATION;
	data [5]= 0x00;
	data [6]= 0x00;
	data [7]= 0x00;

	can_send_SDO(&data[0], node, 0, 8);				// Sends the message to a node

	//EPOS halt mode
	Delay_us(50000);//_delay_ms(10);									//Delay in order to let the EPOS respond before sending the next command

	data [0]= 0x22;
	data [1]= 0x40;
	data [2]= 0x60;
	data [3]= 0x00;
	data [4]= 0x0F;
	data [5]= 0x01;
	data [6]= 0x00;
	data [7]= 0x00;

	can_send_SDO(&data[0], node, 0, 8);				// Sends the message to a node;

	Delay_us(50000);//_delay_ms(10);									//Delay in order to let the EPOS respond before sending the next command
}


/***************************************************************************/
/* Name	        :	EPOS_stop()						   */
/* Function		:	Stops the motors					*/
/*Transfer value: node -> EPOS												*/
/***************************************************************************/

void EPOS_stop(char node){
	unsigned char  data[8];

	//Quik_stop

	data [0]= 0x22;
	data [1]= 0x40;
	data [2]= 0x60;
	data [3]= 0x00;
	data [4]= QUICK_STOP;
	data [5]= 0x00;
	data [6]= 0x00;
	data [7]= 0x00;

	can_send_SDO(&data[0], node, 0, 8);				// Sendet die Message an node;

	Delay_us(1000);//_delay_ms(1);
}

/***************************************************************************/
/* Name	        :	_delay_ms()						   */
/* Function		:	Waiting x 1 ms					*/
/*Transfer value: ms -> number ms												*/
/***************************************************************************/

void _delay_ms(unsigned char ms){
	unsigned int delay = ms*2290;
	while(delay > 10){delay--;}
}


/***************************************************************************/
/* Name	:	EPOS_set_speed_SDO()								   */
/* Function		:	Set velocity, EPOS has to be in the velocity mode	   */
/* Transfer value	:	node -> EPOS										   */
/*					v	 -> Speed										   */
/***************************************************************************/
void EPOS_set_speed_SDO(int node,int v){
	unsigned char  data[8];

	data [0]= 0x22;
	data [1]= 0x6B;
	data [2]= 0x20;
	data [3]= 0x00;
	data [4]= (char)(v & 0x000000ff);				//the integer divide v into individual bytes
	data [5]= (char)((v & 0x0000ff00)>>8);
	data [6]= (char)((v & 0x00ff0000)>>16);
	data [7]= (char)((v & 0xff000000)>>24);

	can_send_SDO(&data[0], node, 0, 8);				// Sends the message to node

	Delay_us(50);//_delay_us(50);
}

/***************************************************************************/
/* Name	:	EPOS_set_speed_SDO()								   */
/* Function		:	Set velocity, EPOS has to be in the current mode	   */
/* Transfer value	:	node -> EPOS										   */
/*					i	 -> current										   */
/***************************************************************************/
void EPOS_set_current_SDO(int node, short i){
	unsigned char  data[8];

	data [0]= 0x22;
	data [1]= 0x30;
	data [2]= 0x20;
	data [3]= 0x00;
	data [4]= (char)(i & 0x00ff);						//the integer split i into individual bytes
	data [5]= (char)((i & 0xff00)>>8);

	can_send_SDO(&data[0], node, 0, 6);				// Sends the message to node

	Delay_us(100);//_delay_us(50);
	//led_on(yellow);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////





/***************************************************************************/
/* Name	:	EPOS_set_position_SDO()								   */
/* Function		:	Set position, EPOS has to be in the position mode	   */
/* Transfer value	:	node -> EPOS										   */
/*					pos	 -> position									   */
/***************************************************************************/
void EPOS_set_position_SDO(int node,int pos){
	unsigned char  data[8];

	data [0]= 0x22;
	data [1]= 0x62;
	data [2]= 0x20;
	data [3]= 0x00;
	data [4]= (char)(pos & 0x000000ff);			//the integer divide v into individual bytes
	data [5]= (char)((pos & 0x0000ff00)>>8);
	data [6]= (char)((pos & 0x00ff0000)>>16);
	data [7]= (char)((pos & 0xff000000)>>24);

	can_send_SDO(&data[0], node, 0, 8);				// Sends the message to node

	Delay_us(50);//_delay_us(50);
}

/***************************************************************************/
/* Name	:	EPOS_set_current_PDO()								   */
/* Function		:	Set current, EPOS has to be in the Operational Mode	   */
/* Transfer value	:	node -> EPOS		0,1,2,4		????					   */
/*					i	 -> current										   */
/***************************************************************************/
void EPOS_set_current_PDO(int node, short i){
	unsigned char  data[8];

	data [0]= (char)(i & 0x00FF);
	data [1]= (char)(i & 0xFF00)>>8;


	can_set_send_PDO_mb(2, 0x0200+node, 0);			// Sets the PDO message box
	can_send_PDO(&data[0]);							//Sends the message to node

	_delay_ms(1);
}

/***************************************************************************/
/* Name	:	EPOS_set_velocity_PDO()								   */
/* Function		:	Set velocity, EPOS has to be in the Operational Mode   */
/* Transfer value	:	node -> EPOS		0,1,2,4							   */
/*					v	 -> velocity									   */
/***************************************************************************/
void EPOS_set_velocity_PDO(int node, int v){
	unsigned char  data[8];

	data [0]= (char)(v & 0x000000ff);				//the integer divide v into individual bytes
	data [1]= (char)((v & 0x0000ff00)>>8);
	data [2]= (char)((v & 0x00ff0000)>>16);
	data [3]= (char)((v & 0xff000000)>>24);
	data [4]= 0x00;
	data [5]= 0x00;
	data [6]= 0x00;
	data [7]= 0x00;

	can_set_send_PDO_mb(8, 0x0400+node, 0);			// Sets the PDO message box
	can_send_PDO(&data[0]);							// Sends the message to node


	_delay_ms(1);
}

/***************************************************************************/
/* Name	:	EPOS_set_position_PDO()								   */
/* Function		:	Set position, EPOS has to be in the Operational Mode   */
/* Transfer value	:	node -> EPOS		0,1,2,4							   */
/*					i	 -> position      								   */
/***************************************************************************/
void EPOS_set_position_PDO(int node, int pos){
	unsigned char  data[8];

	data [0]= (char)(pos & 0x000000ff);				//the integer divide pos into individual bytes
	data [1]= (char)((pos & 0x0000ff00)>>8);
	data [2]= (char)((pos & 0x00ff0000)>>16);
	data [3]= (char)((pos & 0xff000000)>>24);


	can_set_send_PDO_mb(4, 0x0300+node, 0);			//  Sets the PDO message box
	can_send_PDO(&data[0]);							// Sends the message to node


	_delay_ms(1);
}

/***************************************************************************/
/* Name	:	EPOS_get_position_SDO()								   */
/* Function		:	Get position										   */
/* Transfer value	:	node -> EPOS										   */
/* Rückgabewert	:														   */
/***************************************************************************/
void EPOS_get_position_SDO(int node){
	//volatile AT91PS_CAN pCAN = AT91C_BASE_CAN;

	unsigned char  data[8];

	data [0]= 0x40;
	data [1]= 0x64;
	data [2]= 0x60;
	data [3]= 0x00;
	data [4]= 0x00;
	data [5]= 0x00;
	data [6]= 0x00;
	data [7]= 0x00;

	can_send_SDO(&data[0], node, 0, 4);			// Sends the message to node

}

/***************************************************************************/
/* Name	:	EPOS_get_velocity_SDO()								   */
/* Function		:	Get velocity										   */
/* Transfer value	:	node -> EPOS										   */
/* Rückgabewert	:														   */
/***************************************************************************/
void EPOS_get_velocity_SDO(int node){
	//volatile AT91PS_CAN pCAN = AT91C_BASE_CAN;

	unsigned char  data[8];

	data [0]= 0x40;
	data [1]= 0x28;
	data [2]= 0x20;
	data [3]= 0x00;
	data [4]= 0x00;
	data [5]= 0x00;
	data [6]= 0x00;
	data [7]= 0x00;

	can_send_SDO(&data[0], node, 0, 4);			// Sends the message to node

}

/***************************************************************************/
/* Name	:	EPOS_get_current_SDO()								   */
/* Function		:	Get current											   */
/* Transfer value	:	node -> EPOS										   */
/* Rückgabewert	:														   */
/***************************************************************************/
void EPOS_get_current_SDO(int node){
	//volatile AT91PS_CAN pCAN = AT91C_BASE_CAN;
	unsigned char  data[8];

	data [0]= 0x40;
	data [1]= 0x27;
	data [2]= 0x20;
	data [3]= 0x00;
	data [4]= 0x00;
	data [5]= 0x00;
	data [6]= 0x00;
	data [7]= 0x00;

	can_send_SDO(&data[0], node, 0, 4);			// Sends the message to node

}

/***************************************************************************/
/* Name	:	EPOS_stop()											   */
/* Function		:	Stop den Motor										   */
/* Transfer value	:	node -> EPOS										   */
/* Rückgabewert	:														   */
/***************************************************************************/
void EPOS_Reset_Home_Position(char node){
	unsigned char  data[8];

	// Reset Home Position

	data [0]= 0x22;
	data [1]= 0x81;
	data [2]= 0x20;
	data [3]= 0x00;
	data [4]= 0x00;
	data [5]= 0x00;
	data [6]= 0x00;
	data [7]= 0x00;

	can_send_SDO(&data[0], node, 0, 8);				// Sends the message to node


	_delay_ms(1);
}
