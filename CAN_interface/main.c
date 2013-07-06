#include "main.h"
#include <math.h>
#include "CAN.h"
#include "EPOS.h"

#include "stm32f4xx_can.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"

void Delay(__IO uint32_t nCount);


/*-------------------------------GLOBAL VARIABLES---------------------------------------*/
int helios_stop;

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

// Motor torques in x, y and z axis
struct _Controll_Output {
	float T_x;
	float T_y;
	float T_z;
}Motor_torque_xyz;

// Motor currents
struct _Motor_Values {
	float I_1;
	float I_2;
	float I_3;
}Motor_current_real;

// Reference commands from Helios
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
	float phi_dot_x;
	float phi_dot_y;
	float phi_dot_z;
	float pos_x;
	float pos_y;
	float pos_x_dot;
	float pos_y_dot;
}phi_dot;

// Calculation of theta_z_abs
int count_jump; 					// globale Variable, die halbe Umdrehungen zählt
float theta_z_tmp; 					// globale Variable des alten Winkels theta_z
int first_time;

// Parameter for PID Control
float esum;							// I-Anteil des PID Reglers
float ealt;							// alter Fehler für D-Anteil des PID Regelers

// Roll-off filter Parameter
float xold1_1;
float xold2_1;
float xold3_1;
float xold4_1;
float xold1_2;
float xold2_2;
float xold3_2;
float xold4_2;
float xold1_3;
float xold2_3;
float xold3_3;
float xold4_3;

// Error detection
char error_detection;

// Trapezoidal Integration
float old_phi_dot_x;
float old_phi_dot_y;
float old_phi_dot_z;
float old_pos_x_dot;
float old_pos_y_dot;

// Set point filter old values
float old_phi_dot_x_soll;
float old_phi_dot_y_soll;
float old_theta_z_soll;
float old_phi_x_soll;
float old_phi_y_soll;

//--------------------------------------------------------------------------------
// Enums
//--------------------------------------------------------------------------------

enum Stats{INIT, IDLE, PARK, CONTROL, /*RESET,*/ STOP, EMERCENCY_STOP};
enum Stats Ballbot_stats;

enum control{C_POSITION, C_VELOCITY, C_ACCELERATION, C_STOP, C_FREEZE, C_ROTONDO, C_HULL_POSITION, C_ACCELERATION_WZ, C_MODE_4,
			 C_MODE_5, C_MODE_6, C_MODE_7, C_MODE_8, C_MODE_9, C_MODE_10, C_MODE_11};
enum control Control_Mode;

// Init z beim starten auf 0
float offset_z;

//flags for the EPOS data
struct _EPOS_sent_data{
	unsigned int EPOS1;
	unsigned int EPOS2;
	unsigned int EPOS3;
}EPOS_sent_data, available_data;

unsigned char epos;

CanRxMsg RxMsg2;
unsigned int init = 1;
/*--------------------------------------------------------------------------------------------*/


/****************************/
/*							*/
/* 		MAIN program		*/
/*							*/
/****************************/

int main(void)
{

/*------------- System Clock Configuration----------------- */
	SystemInit();

/*--------------------LED initialization--------------------*/
	led_init();
	led_off(red);

//-------------------Clock testing---------------------------//
//
//	RCC_ClocksTypeDef * RCC_Clocks;
//	RCC_ClocksTypeDef Clocks;
//
//	RCC_GetClocksFreq(&Clocks);					//not the real frequency in the chip. It is calculated based on the predefined in stm32f4xx.h constant and the selected clock source
//	Clocks.HCLK_Frequency = (RCC_Clocks->HCLK_Frequency);
//	Clocks.SYSCLK_Frequency = (RCC_Clocks->SYSCLK_Frequency);
//	Clocks.PCLK1_Frequency = (RCC_Clocks->PCLK1_Frequency);
//	Clocks.PCLK2_Frequency = (RCC_Clocks->PCLK2_Frequency);

//
//	//---testing with oscillator the board's clock frequency---//
//
//	//MCO1-PA8--PLL_frequency
//	RCC_MCO1Config(RCC_MCO1Source_PLLCLK,RCC_MCO1Div_1);
//
//	RCC_AHB1PeriphClockCmd (RCC_AHB1Periph_GPIOA,ENABLE);
//
//	GPIO_InitTypeDef a;
//	GPIO_InitTypeDef * GPIO_InitStruct;
//	GPIO_InitStruct = &a;
//
//	GPIO_PinAFConfig(GPIOA,GPIO_PinSource8,GPIO_AF_MCO);			//MCO1
//
//	GPIO_InitStruct->GPIO_Pin = GPIO_Pin_8;
//	GPIO_InitStruct->GPIO_Mode = GPIO_Mode_AF;
//	GPIO_Init(GPIOA,GPIO_InitStruct);

//	//MCO2-PC9--SYSCLK_frequency
//	RCC_MCO2Config(RCC_MCO2Source_SYSCLK,RCC_MCO2Div_1);
//
//	RCC_AHB1PeriphClockCmd (RCC_AHB1Periph_GPIOC,ENABLE);
//
//	GPIO_InitTypeDef a;
//	GPIO_InitTypeDef * GPIO_InitStruct;
//	GPIO_InitStruct = &a;
//
//	GPIO_PinAFConfig(GPIOC,GPIO_PinSource9,GPIO_AF_MCO);			//MCO2
//	GPIO_InitStruct->GPIO_Pin = GPIO_Pin_9;
//	GPIO_InitStruct->GPIO_Mode = GPIO_Mode_AF;
//	GPIO_Init(GPIOC,GPIO_InitStruct);

//// Check if the clock source is PLL
//	uint8_t source;
//	source=RCC_GetSYSCLKSource();
//	if (source==0x08){led_on(green1);}
//------------------------------------------------------------//

/*-------------------CAN initialization----------------------*/
	init_CAN();

/*-------------------EPOS initialization---------------------*/
	EPOS_fault_reset(1);
	EPOS_fault_reset(2);
	EPOS_fault_reset(4);
	EPOS_init(1);
	EPOS_init(2);
	EPOS_init(4);
	EPOS_fault_reset(1);
	EPOS_fault_reset(2);
	EPOS_fault_reset(4);
	EPOS_reset_communicaton(1);
	EPOS_reset_communicaton(2);
	EPOS_reset_communicaton(4);
	Set_Pre_Operational_NMT_State(1);
	Set_Pre_Operational_NMT_State(2);
	Set_Pre_Operational_NMT_State(4);
	EPOS_set_operation_mode(1, CURRENT_MODE);
	EPOS_set_operation_mode(2, CURRENT_MODE);
	EPOS_set_operation_mode(4, CURRENT_MODE);

	//Delay(1000); The pending messages are cleared even without delay

	while(1)
    {
		init = 0 ;		//Initialization has stopped

		//No pending messages
		//No errors
		toggle_led(green1);
		Delay(10000000);

/*-------------------------TESTING--------------------------*/

		/*getting the position of the motors*/
		EPOS_get_current_SDO(4);
		Delay(1000);
		EPOS_get_position_SDO(1);
		Delay(1000);
		EPOS_get_current_SDO(2);
		// NOTE: The board gets response from the EPOS we send the command

		/*setting the motors current*/
		EPOS_set_current_SDO(1,100);
		EPOS_set_current_SDO(2,100);
		EPOS_set_current_SDO(4,100);
		// NOTE: The motors are turning, sending to different number/EPOS different motor is turning
    }
}

/********************************/
/*Name:			Delay()			*/
/********************************/

void Delay(__IO uint32_t nCount)
{
  while(nCount--)
  {
  }
}

