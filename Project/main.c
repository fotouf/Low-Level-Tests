
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_dma.h"
#include "stm32f4xx_usart.h"

#include "main.h"
#include "Serials.h"
#include "CAN.h"
#include "EPOS.h"
#include "Other.h"
#include "Defines_Ballbot.h"
#include "odometry.h"



// Sensor Daten
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

// Momente in x,y und z Richtung
struct _Controll_Output {
	float T_x;
	float T_y;
	float T_z;
}Motor_torque_xyz;

// Motoren Str�me
struct _Motor_Values {
	float I_1;
	float I_2;
	float I_3;
}Motor_current_real;

// Helios Sollwerte
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

// Odometry Daten
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

// Calculation of theta_z_abs
int count_jump; 					// globale Variable, die halbe Umdrehungen z�hlt
float theta_z_tmp; 					// globale Variable des alten Winkels theta_z
int first_time;

// Parameter for PID Control
float esum;							// I-Anteil des PID Reglers
float ealt;							// alter Fehler f�r D-Anteil des PID Regelers

// LED Parameter
unsigned char led_value;

// Hull Parameter
unsigned char hull_position;
unsigned char change_way;
unsigned char mode_frequenzy;
unsigned int timer_parking_down;
unsigned int timer_parking_up;
unsigned int timer_frequenzy;

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

// Kalmanfilter Parameter
long int p11;
long int p12;
long int p13;
long int p14;
long int p21;
long int p22;
long int p23;
long int p24;
long int p31;
long int p32;
long int p33;
long int p34;
long int p41;
long int p42;
long int p43;
long int p44;

long int thetaxk;
long int phixk;
long int phidxk;
long int thetayk;
long int phiyk;
long int phidyk;


// Error detection
char error_detection;

// Trapez Integration
float old_phi_dot_x;
float old_phi_dot_y;
float old_phi_dot_z;
float old_pos_x_dot;
float old_pos_y_dot;

// Setpointfilter old values
float old_phi_dot_x_soll;
float old_phi_dot_y_soll;
float old_theta_z_soll;
float old_phi_x_soll;
float old_phi_y_soll;

// SetPoint
int temp_count;

// Not Aus wenn Helios stopt
int helios_stop;

// Hilfsvariable um EPOS zur rechten Zeit einzulesen
char Read_EPOS;

// Init z beim starten auf 0
float offset_z;

// constantly increasing counter for helios transmission check
float data_check_counter;


//flags for the EPOS data
struct _EPOS_sent_data{
	unsigned int EPOS1;
	unsigned int EPOS2;
	unsigned int EPOS3;
}EPOS_sent_data, available_data;

unsigned int init = 1;
unsigned int IMU_data_for_PC=0;


int main(void)
{
/*-------------------------------------------------------------------------*/
/*	Variables															   */
/*-------------------------------------------------------------------------*/

	int buffer;
	unsigned char freq_Data_HELIOS;

//------------------------------------------------------------------//
// 							INITIALIZATION 							//
//------------------------------------------------------------------//


	/*CLock and leds initialization*/
	System_Init();

	/*---------Configures the USART3 and USART6 peripheral and the DMA1 and DMA2 controllers-------*/
	USART_PC_Config();
	USART_IMU_Config();
	/*---------Configures CAN controller----------*/
	init_CAN();



//------------------------------------------------------------------//
// 							DEFAULT VALUE 							//
//------------------------------------------------------------------//

	// Absolutes theta_z Hilfsvariabeln
		count_jump = 0;
		theta_z_tmp = 0;
		first_time = 0;

		// PID Init
		esum = 0;
		ealt = 0;

		// SOLL Wert vorgaben
		Helios_val.position_x = 0;
		Helios_val.position_y = 0;
		Helios_val.position_z = 0;
		Helios_val.velocity_x = 0;
		Helios_val.velocity_y = 0;
		Helios_val.position_x_raw = 0;
		Helios_val.position_y_raw = 0;
		Helios_val.position_z_raw = 0;
		Helios_val.velocity_x_raw = 0;
		Helios_val.velocity_y_raw = 0;
		Helios_val.acceleration_x = 0;
		Helios_val.acceleration_y = 0;

		phi_dot.phi_x = 0;
		phi_dot.phi_y = 0;
		phi_dot.phi_z = 0;
		phi_dot.phi_dot_x = 0;
		phi_dot.phi_dot_y = 0;
		phi_dot.phi_dot_z = 0;
		phi_dot.pos_x_dot = 0;
		phi_dot.pos_y_dot = 0;
		phi_dot.pos_x = 0;
		phi_dot.pos_y = 0;
		Sensor_val.psi_dot_1 = 0;
		Sensor_val.psi_dot_2 = 0;
		Sensor_val.psi_dot_3 = 0;
		Sensor_val.gyro_x = 0;
		Sensor_val.gyro_y = 0;
		Sensor_val.gyro_z = 0;
		Sensor_val.theta_dot_x = 0;
		Sensor_val.theta_dot_y = 0;
		Sensor_val.theta_dot_z = 0;
		Sensor_val.theta_x = 0;
		Sensor_val.theta_y = 0;
		Sensor_val.theta_z = 0;
		Sensor_val.theta_z_abs = 0;
		Sensor_val.motor_vel_1 = 0;
		Sensor_val.motor_vel_2 = 0;
		Sensor_val.motor_vel_3 = 0;
		Sensor_val.motor_pos_1 = 0;
		Sensor_val.motor_pos_2 = 0;
		Sensor_val.motor_pos_3 = 0;
		Sensor_val.current_EPOS1 = 0;
		Sensor_val.current_EPOS2 = 0;
		Sensor_val.current_EPOS3 = 0;
		Sensor_val.theta_dot_x_raw = 0;
		Sensor_val.theta_dot_y_raw = 0;
		Sensor_val.qW = 0;
		Sensor_val.qX = 0;
		Sensor_val.qY = 0;
		Sensor_val.qZ = 0;
		Sensor_val.acc_x = 0;
		Sensor_val.acc_y = 0;
		Sensor_val.acc_z = 0;

		//Data_Available = (0x1<<2);					// Helios Data already available

		// Roll-off
		xold1_1 = 0;
		xold2_1 = 0;
		xold3_1 = 0;
		xold4_1 = 0;
		xold1_2 = 0;
		xold2_2 = 0;
		xold3_2 = 0;
		xold4_2 = 0;
		xold1_3 = 0;
		xold2_3 = 0;
		xold3_3 = 0;
		xold4_3 = 0;

		// Kalman
		p11 = 0;
		p12 = 0;
		p13 = 0;
		p14 = 0;
		p21 = 0;
		p22 = 0;
		p23 = 0;
		p24 = 0;
		p31 = 0;
		p32 = 0;
		p33 = 0;
		p34 = 0;
		p41 = 0;
		p42 = 0;
		p43 = 0;
		p44 = 0;

		thetaxk = 0;
		phixk   = 0;
		phidxk  = 0;
		thetayk = 0;
		phiyk   = 0;
		phidyk  = 0;

		// Sendefrequenz zu Helios Board einstellen
		freq_Data_HELIOS = 0;

		// Error detection
		error_detection = 0;

		// Trapez Integration
		old_phi_dot_x = 0;
		old_phi_dot_y = 0;
		old_phi_dot_z = 0;
		old_pos_x_dot = 0;
		old_pos_y_dot = 0;

		// Setpointfilter old values
		old_phi_dot_x_soll = 0;
		old_phi_dot_y_soll = 0;
		old_theta_z_soll = 0;
		old_phi_x_soll = 0;
		old_phi_y_soll = 0;

		temp_count = 0;

		// Helios Not Aus
		helios_stop = 0;

		Read_EPOS = 0;

		// Hull
		mode_frequenzy = 1;
		change_way = 1;
		timer_parking_down = 0;
		timer_parking_up = 0;
		timer_frequenzy = 0;

		// Init z beim starten auf 0
		offset_z = 0;

//		// Startet den Timer um die Helios Daten zu lesen
//		start_timer1(_6_3_MS);


		Control_Mode = C_STOP;
	  	Ballbot_stats = IDLE;						// First Status IDLE



	/*----------Sending start continuous mode command to the High Level PC--------*/
	//Comments:testing the sending part
	Start_Continious_Mode_IMU();

	/*--------Enable the DMA for receiving & request to receive------------*/

	DMA_SetCurrDataCounter(DMA2_Stream1, 1);

	DMA_Cmd(DMA2_Stream1,ENABLE);
	while(DMA_GetCmdStatus(DMA2_Stream1) != ENABLE);

	USART_DMACmd(USART6, USART_DMAReq_Rx, ENABLE);
	//------------------------------------------------------------------------//



//---------------------------------------------------------------------------
// Default Value
//---------------------------------------------------------------------------





	while (1)
	{


		if (IMU_data_for_PC == 1 )
		{
			Gyro_Values_to_Theta_dot();
			odometry();
			//toggle_led(yellow);
			Delay(1000);
			Send_Sensor_Values_to_HELIOS(160);
			IMU_data_for_PC = 0 ;
		}


	}
}









/***************************************************************************/
/* Bezeichnung	:	Safety_first(void)()								   */
/* Funktion		:	�berpr�ft verschieden maximalwerte und schaltet stellt */
/*					wenn n�tig den Regler ab							   */
/* �bergabewert	:	---													   */
/* R�ckgabewert	:	---													   */
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

/*	Initializes systems clock and leds	*/
void System_Init(void)
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
}

void Delay(__IO uint32_t nCount)
{
  while(nCount--)
  {
  }
}

/***************************************************************************/
/* Bezeichnung	:	init_controller()									   */
/* Funktion		:	Initialisiert alle Variabeln und die EPOS			   */
/* �bergabewert	:	---													   */
/* R�ckgabewert	:	---													   */
/***************************************************************************/
void init_controller(void)
{
	data_check_counter = 0;

	count_jump = 0;
	theta_z_tmp = 0;
	first_time = 0;
	temp_count = 0;
	error_detection = 0;
	helios_stop = 0;
	offset_z = 0;
	// PID Init
	esum = 0;
	ealt = 0;
	old_phi_dot_x = 0;
	old_phi_dot_y = 0;
	old_phi_dot_z = 0;
	old_pos_x_dot = 0;
	old_pos_y_dot = 0;
	old_phi_dot_x_soll = 0;	// Setpointfilter old values
	old_phi_dot_y_soll = 0;
	old_theta_z_soll = 0;
	old_phi_x_soll = 0;
	old_phi_y_soll = 0;
	p11 = 0; // Kalman
	p12 = 0;
	p13 = 0;
	p14 = 0;
	p21 = 0;
	p22 = 0;
	p23 = 0;
	p24 = 0;
	p31 = 0;
	p32 = 0;
	p33 = 0;
	p34 = 0;
	p41 = 0;
	p42 = 0;
	p43 = 0;
	p44 = 0;
	thetaxk = 0;
	phixk   = 0;
	phidxk  = 0;
	thetayk = 0;
	phiyk   = 0;
	phidyk  = 0;
	Control_Mode = C_VELOCITY;
	//stop_timer1();
	Sensor_val.theta_dot_x_raw = 0;
	Sensor_val.theta_dot_y_raw = 0;
	Read_EPOS = 0;
	phi_dot.phi_x = 0;
	phi_dot.phi_y = 0;
	phi_dot.phi_z = 0;
	phi_dot.phi_dot_x = 0;
	phi_dot.phi_dot_y = 0;
	phi_dot.phi_dot_z = 0;
	phi_dot.pos_x_dot = 0;
	phi_dot.pos_y_dot = 0;
	phi_dot.pos_x = 0;
	phi_dot.pos_y = 0;
	Helios_val.position_x = 0;
	Helios_val.position_y = 0;
	Helios_val.position_z = 0;
	Helios_val.velocity_x = 0;
	Helios_val.velocity_y = 0;
	Helios_val.position_x_raw = 0;
	Helios_val.position_y_raw = 0;
	Helios_val.position_z_raw = 0;
	Helios_val.velocity_x_raw = 0;
	Helios_val.velocity_y_raw = 0;
	Helios_val.acceleration_x = 0;
	Helios_val.acceleration_y = 0;
	Sensor_val.psi_dot_1 = 0;
	Sensor_val.psi_dot_2 = 0;
	Sensor_val.psi_dot_3 = 0;
	Sensor_val.gyro_x = 0;
	Sensor_val.gyro_y = 0;
	Sensor_val.gyro_z = 0;
	Sensor_val.theta_dot_x = 0;
	Sensor_val.theta_dot_y = 0;
	Sensor_val.theta_dot_z = 0;
	Sensor_val.theta_x = 0;
	Sensor_val.theta_y = 0;
	Sensor_val.theta_z = 0;
	Sensor_val.theta_z_abs = 0;
	Sensor_val.motor_vel_1 = 0;
	Sensor_val.motor_vel_2 = 0;
	Sensor_val.motor_vel_3 = 0;
	Sensor_val.motor_pos_1 = 0;
	Sensor_val.motor_pos_2 = 0;
	Sensor_val.motor_pos_3 = 0;
	Sensor_val.current_EPOS1 = 0;
	Sensor_val.current_EPOS2 = 0;
	Sensor_val.current_EPOS3 = 0;
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

	// At a test version of CAN there weren't any pending messages at this point

	Start_Continious_Mode_IMU();	// Initialize IMU
}

/***************************************************************************/
/* Bezeichnung	:	stop_controller()									   */
/* Funktion		:	Stopt den Regelungsprozes				 			   */
/* �bergabewert	:	---													   */
/* R�ckgabewert	:	---													   */
/***************************************************************************/
void stop_controller(void)
{
	Stop_Continious_Mode();
	Motor_current_real.I_1 = 0;
	Motor_current_real.I_2 = 0;
	Motor_current_real.I_3 = 0;
	EPOS_set_current_SDO(1,(short)Motor_current_real.I_1);
	EPOS_set_current_SDO(2,(short)Motor_current_real.I_2);
	EPOS_set_current_SDO(4,(short)Motor_current_real.I_3);
}
