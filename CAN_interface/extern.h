/*=========================================================================*/
/* 																		   */
/*	Autor 				: Samuel Schüller, Swen Wigert					   */
/*	File				: extern.h										   */
/*	Erstellt			: 12.02.2010									   */
/* 	Aktuelle Version	: v1.00										 	   */
/*																	       */
/*=========================================================================*/
/*																		   */
/*	Änderungshistory													   */
/*-------------------------------------------------------------------------*/
/*	v1.00	| Grundversion 												   */
/* 	v1.01	|															   */
/*=========================================================================*/

#ifndef EXTERN_H_
#define EXTERN_H_

extern unsigned char Data_Available;
extern int count_jump; 					// globale Variable, die halbe Umdrehungen zählt
extern float theta_z_tmp; 				// globale Variable des alten Winkels theta_z
extern int first_time;

extern unsigned char led_value;
extern unsigned char hull_position;
extern unsigned char mode_frequenzy;
extern unsigned int timer_parking_up;
extern unsigned int timer_parking_down;
extern unsigned int timer_frequenzy;

/////////////////////////////
extern unsigned int EPOS_data_available; //counts the EPOS that have sent data

//alternative
extern struct _EPOS_sent_data{
	unsigned int EPOS1;
	unsigned int EPOS2;
	unsigned int EPOS3;
}EPOS_sent_data, available_data;
////////////////////////////

extern const float lookup_table[];

extern struct _Sensor_Values {
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
}  Sensor_Values, Sensor_val;

extern struct _Motor_Values {
	float I_1;
	float I_2;
	float I_3;
}Motor_current_real;

// Momente in x,y und z Richtung
extern struct _Controll_Output {
	float T_x;
	float T_y;
	float T_z;
}Motor_torque_xyz;

extern struct _Helios_Values {
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

extern struct _odometry_output {
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


extern enum Stats{INIT, IDLE, PARK, CONTROL, /*RESET,*/ STOP, EMERCENCY_STOP} Ballbot_stats;

extern enum control{C_POSITION, C_VELOCITY, C_ACCELERATION, C_STOP, C_FREEZE, C_ROTONDO, C_HULL_POSITION, C_ACCELERATION_WZ, C_MODE_4,
			 C_MODE_5, C_MODE_6, C_MODE_7, C_MODE_8, C_MODE_9, C_MODE_10, C_MODE_11} Control_Mode;


// Roll-off
extern float xold1_1;
extern float xold2_1;
extern float xold3_1;
extern float xold4_1;
extern float xold1_2;
extern float xold2_2;
extern float xold3_2;
extern float xold4_2;
extern float xold1_3;
extern float xold2_3;
extern float xold3_3;
extern float xold4_3;

// Kalmanfilter Parameter
extern long int p11;
extern long int p12;
extern long int p13;
extern long int p14;
extern long int p21;
extern long int p22;
extern long int p23;
extern long int p24;
extern long int p31;
extern long int p32;
extern long int p33;
extern long int p34;
extern long int p41;
extern long int p42;
extern long int p43;
extern long int p44;


extern int thetaxk;
extern int phixk;
extern int phidxk;
extern int thetayk;
extern int phiyk;
extern int phidyk;

// Error detection
extern char error_detection;

// Trapez Integration
extern float old_phi_dot_x;
extern float old_phi_dot_y;
extern float old_phi_dot_z;
extern float old_pos_x_dot;
extern float old_pos_y_dot;

// Set point filter old values
extern float old_phi_dot_x_soll;
extern float old_phi_dot_y_soll;
extern float old_theta_z_soll;
extern float old_phi_x_soll;
extern float old_phi_y_soll;

extern int temp_count;

extern int helios_stop;

extern char Read_EPOS;

extern float offset_z;

// constantly increasing counter for helios transmission check
extern float data_check_counter;

// Parameter for PID Control
extern float esum;							// I-Anteil des PID Reglers
extern float ealt;							// alter Fehler für D-Anteil des PID Regelers

extern unsigned int init;

///////////////////
//Debugging stuff//
///////////////////
extern unsigned int counter;			//counts how many times we got data from the IMU








#endif /* EXTERN_H_ */
