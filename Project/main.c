
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_dma.h"
#include "stm32f4xx_usart.h"

#include "main.h"
#include "Serials.h"
#include "CAN.h"
#include "EPOS.h"
#include "Other.h"
#include "Defines_Ballbot.h"


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

// Torques in x, y and z
struct _Controll_Output {
	float T_x;
	float T_y;
	float T_z;
}Motor_torque_xyz;

// Motor Currents
struct _Motor_Values {
	float I_1;
	float I_2;
	float I_3;
}Motor_current_real;

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

// Odometry Data
struct _odometry_output {
	float phi_x;
	float phi_y;
	float phi_z;
	float phi_dot_x;				// Kugelumdrehungsgeschwindigkeit rad/s
	float phi_dot_y;
	float phi_dot_z;
	float pos_x;
	float pos_y;
	float pos_x_dot;
	float pos_y_dot;
}phi_dot;

// Setpointfilter old values
float old_phi_dot_x_soll;
float old_phi_dot_y_soll;
float old_theta_z_soll;
float old_phi_x_soll;
float old_phi_y_soll;

// Error detection
char error_detection;

//flags for the EPOS data
struct _EPOS_sent_data{
	unsigned int EPOS1;
	unsigned int EPOS2;
	unsigned int EPOS3;
}EPOS_sent_data, available_data;

unsigned int init = 1;






int main(void)
{
	/*-----init------*/
	//helios_stop = 0;

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
	USART_PC_Config();
	USART_IMU_Config();

	/*----------Sending start continuous mode command to the High Level PC--------*/
	//Comments:testing the sending part
	Start_Continious_Mode_IMU();

	/*--------Enable the DMA for receiving & request to receive------------*/

	DMA_SetCurrDataCounter(DMA2_Stream1, 1);

	DMA_Cmd(DMA2_Stream1,ENABLE);
	while(DMA_GetCmdStatus(DMA2_Stream1) != ENABLE);

	USART_DMACmd(USART6, USART_DMAReq_Rx, ENABLE);
	//------------------------------------------------------------------------//

	while (1)
	{


	}
}









/***************************************************************************/
/* Bezeichnung	:	Safety_first(void)()								   */
/* Funktion		:	Überprüft verschieden maximalwerte und schaltet stellt */
/*					wenn nötig den Regler ab							   */
/* Übergabewert	:	---													   */
/* Rückgabewert	:	---													   */
/***************************************************************************/
void Safety_first(void)
{
	// Winkelbegrenzung
	if((Sensor_val.theta_x > MAX_THETA) || (Sensor_val.theta_x < -MAX_THETA) ||
		(Sensor_val.theta_y > MAX_THETA) || (Sensor_val.theta_y < -MAX_THETA))
	{
		if(error_detection <= 15)
		{
			error_detection++;
		}
		else
		{
			Ballbot_stats = EMERCENCY_STOP;
			error_detection = 0;
		}
	}
	else
	{
		error_detection = 0;
	}
}




