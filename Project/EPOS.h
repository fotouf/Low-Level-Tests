/*==============================================================================*/
/* 																				*/
/*	Autor 				: Samuel Schüller, Swen Wigert							*/
/*	File				: EPOS.h												*/
/*	Erstellt			: 11.03.2010											*/
/* 	Aktuelle Version	: v1.00													*/
/*																				*/
/*==============================================================================*/
/*																				*/
/*	Änderungshistory															*/
/*------------------------------------------------------------------------------*/
/*	v1.00	| Grundversion Initialisierung										*/
/* 	v1.01	|																	*/
/*==============================================================================*/

#ifndef EPOS_H_
#define EPOS_H_

//--------------------------------------------------------------------------------
// Prototyps
//--------------------------------------------------------------------------------
void Set_Operational_NMT_State(char node);
void Set_Pre_Operational_NMT_State(char node);
void EPOS_set_operation_mode(char node,char mode);
void EPOS_fault_reset(char node);
void EPOS_disable(char node);
void EPOS_enable(char node);
void EPOS_reset_communicaton(char node);
void EPOS_init(char node);
void EPOS_stop(char node);
void _delay_ms(unsigned char ms);
void _delay_us(unsigned short us);
void EPOS_set_speed_SDO(int node,int v);
void EPOS_set_current_SDO(int node, short i);
void EPOS_set_position_SDO(int node,int pos);
void EPOS_set_current_PDO(int node, short i);
void EPOS_set_velocity_PDO(int node, int v);
void EPOS_set_position_PDO(int node, int pos);
void EPOS_get_position_SDO(int node);
void EPOS_get_velocity_SDO(int node);
void EPOS_get_current_SDO(int node);
void EPOS_Reset_Home_Position(char node);


//--------------------------------------------------------------------------------
// Defines
//--------------------------------------------------------------------------------
// Modes of Operation
#define POSITION_MODE 			-1
#define VELOCITY_MODE 			-2
#define CURRENT_MODE 			-3
#define MASTER_ENCODER_MODE 	-5
#define STEP_DIRECTION_MODE 	-6
#define PROFILE_POSITION_MODE 	 1
#define HOMING_MODE 			 6
#define PROFILE_VELOCITY_MODE 	 3

// Control Word
#define SHUTDOWN 			0x06			// 0xxx x110
#define ENABLE_OPERATION 	0x0F			// 0xxx 1111
#define QUICK_STOP 			0x02			// 0xxx x01x
#define DISABLE_OPERATION 	0x07			// 0xxx 0111
#define FAULT_RESET			0x80			// 0xxx xxxx -> 1xxx xxxx

// Read Protokol
#define RECEIVE_ACT_POS 0x00606443
#define RECEIVE_ACT_VEL 0x00202843
#define RECEIVE_ACT_CUR 0x0020274B

#endif /* EPOS_H_ */
