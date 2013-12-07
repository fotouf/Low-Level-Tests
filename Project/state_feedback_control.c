/*=========================================================================*/
/* 																		   */
/*	Autor 				: Corsin Gwerder								   */
/*	File				: state_feedback_control.c						   */
/*	Erstellt			: 25.03.2010									   */
/* 	Aktuelle Version	: v1.00										 	   */
/*																	       */
/*=========================================================================*/
/*																		   */
/*	Änderungshistory													   */
/*-------------------------------------------------------------------------*/
/*	v1.00	| Grundversion Initialisierung								   */
/* 	v1.01	|															   */
/*=========================================================================*/

#include "state_feedback_control.h"
#include "Defines_Ballbot.h"
#include "extern.h"
#include <math.h>

/***************************************************************************/
/* Bezeichnung	:	state_feedback_control()							   */
/* Funktion		:	Berechnet die Stellgrössen -> Regler                   */
/* Übergabewert	:	---													   */
/* Rückgabewert	:	---													   */
/***************************************************************************/
void state_feedback_control(void)
{
	float T_1, T_2, T_3; //Real torques


	// multiply states by controller gains
	if (Control_Mode == C_FREEZE){
		T_1 = -K_PHI_FREEZE_18*PDP_PHI_8*phi_dot.phi_dot_x-K_PHI_FREEZE_110*PDP_PHI_10*phi_dot.phi_dot_y-K_PHI_FREEZE_17*PDP_PHI_7*phi_dot.phi_x-K_PHI_FREEZE_19*PDP_PHI_9*phi_dot.phi_y-K_PHI_FREEZE_12*PDP_PHI_2*Sensor_val.theta_dot_x-K_PHI_FREEZE_14*PDP_PHI_4*Sensor_val.theta_dot_y-K_PHI_FREEZE_16*PDP_PHI_6*Sensor_val.theta_dot_z-K_PHI_FREEZE_11*PDP_PHI_1*Sensor_val.theta_x-K_PHI_FREEZE_13*PDP_PHI_3*Sensor_val.theta_y-K_PHI_FREEZE_15*PDP_PHI_5*Sensor_val.theta_z_abs+K_PHI_FREEZE_15*Helios_val.position_z;
		T_2 = -K_PHI_FREEZE_28*PDP_PHI_8*phi_dot.phi_dot_x-K_PHI_FREEZE_210*PDP_PHI_10*phi_dot.phi_dot_y-K_PHI_FREEZE_27*PDP_PHI_7*phi_dot.phi_x-K_PHI_FREEZE_29*PDP_PHI_9*phi_dot.phi_y-K_PHI_FREEZE_22*PDP_PHI_2*Sensor_val.theta_dot_x-K_PHI_FREEZE_24*PDP_PHI_4*Sensor_val.theta_dot_y-K_PHI_FREEZE_26*PDP_PHI_6*Sensor_val.theta_dot_z-K_PHI_FREEZE_21*PDP_PHI_1*Sensor_val.theta_x-K_PHI_FREEZE_23*PDP_PHI_3*Sensor_val.theta_y-K_PHI_FREEZE_25*PDP_PHI_5*Sensor_val.theta_z_abs+K_PHI_FREEZE_25*Helios_val.position_z;
		T_3 = -K_PHI_FREEZE_38*PDP_PHI_8*phi_dot.phi_dot_x-K_PHI_FREEZE_310*PDP_PHI_10*phi_dot.phi_dot_y-K_PHI_FREEZE_37*PDP_PHI_7*phi_dot.phi_x-K_PHI_FREEZE_39*PDP_PHI_9*phi_dot.phi_y-K_PHI_FREEZE_32*PDP_PHI_2*Sensor_val.theta_dot_x-K_PHI_FREEZE_34*PDP_PHI_4*Sensor_val.theta_dot_y-K_PHI_FREEZE_36*PDP_PHI_6*Sensor_val.theta_dot_z-K_PHI_FREEZE_31*PDP_PHI_1*Sensor_val.theta_x-K_PHI_FREEZE_33*PDP_PHI_3*Sensor_val.theta_y-K_PHI_FREEZE_35*PDP_PHI_5*Sensor_val.theta_z_abs+K_PHI_FREEZE_35*Helios_val.position_z;
	}


	if (Control_Mode == C_POSITION){
		T_1 = -K_PHI_18*PDP_PHI_8*phi_dot.phi_dot_x-K_PHI_110*PDP_PHI_10*phi_dot.phi_dot_y-K_PHI_17*PDP_PHI_7*phi_dot.phi_x-K_PHI_19*PDP_PHI_9*phi_dot.phi_y+K_PHI_15*Helios_val.position_z+K_PHI_17*Helios_val.position_x+K_PHI_19*Helios_val.position_y-K_PHI_12*PDP_PHI_2*Sensor_val.theta_dot_x-K_PHI_14*PDP_PHI_4*Sensor_val.theta_dot_y-K_PHI_16*PDP_PHI_6*Sensor_val.theta_dot_z-K_PHI_11*PDP_PHI_1*Sensor_val.theta_x-K_PHI_13*PDP_PHI_3*Sensor_val.theta_y-K_PHI_15*PDP_PHI_5*Sensor_val.theta_z_abs;
		T_2 = -K_PHI_28*PDP_PHI_8*phi_dot.phi_dot_x-K_PHI_210*PDP_PHI_10*phi_dot.phi_dot_y-K_PHI_27*PDP_PHI_7*phi_dot.phi_x-K_PHI_29*PDP_PHI_9*phi_dot.phi_y+K_PHI_25*Helios_val.position_z+K_PHI_27*Helios_val.position_x+K_PHI_29*Helios_val.position_y-K_PHI_22*PDP_PHI_2*Sensor_val.theta_dot_x-K_PHI_24*PDP_PHI_4*Sensor_val.theta_dot_y-K_PHI_26*PDP_PHI_6*Sensor_val.theta_dot_z-K_PHI_21*PDP_PHI_1*Sensor_val.theta_x-K_PHI_23*PDP_PHI_3*Sensor_val.theta_y-K_PHI_25*PDP_PHI_5*Sensor_val.theta_z_abs;
		T_3 = -K_PHI_38*PDP_PHI_8*phi_dot.phi_dot_x-K_PHI_310*PDP_PHI_10*phi_dot.phi_dot_y-K_PHI_37*PDP_PHI_7*phi_dot.phi_x-K_PHI_39*PDP_PHI_9*phi_dot.phi_y+K_PHI_35*Helios_val.position_z+K_PHI_37*Helios_val.position_x+K_PHI_39*Helios_val.position_y-K_PHI_32*PDP_PHI_2*Sensor_val.theta_dot_x-K_PHI_34*PDP_PHI_4*Sensor_val.theta_dot_y-K_PHI_36*PDP_PHI_6*Sensor_val.theta_dot_z-K_PHI_31*PDP_PHI_1*Sensor_val.theta_x-K_PHI_33*PDP_PHI_3*Sensor_val.theta_y-K_PHI_35*PDP_PHI_5*Sensor_val.theta_z_abs;
	}


	if (Control_Mode == C_VELOCITY){
		T_1 = -K_PHI_DOT_17*PDP_PHI_7*phi_dot.phi_dot_x-K_PHI_DOT_18*PDP_PHI_8*phi_dot.phi_dot_y+K_PHI_DOT_17*Helios_val.velocity_x+K_PHI_DOT_18*Helios_val.velocity_y+K_PHI_DOT_15*Helios_val.position_z-K_PHI_DOT_12*PDP_PHI_2*Sensor_val.theta_dot_x-K_PHI_DOT_14*PDP_PHI_4*Sensor_val.theta_dot_y-K_PHI_DOT_16*PDP_PHI_6*Sensor_val.theta_dot_z-K_PHI_DOT_11*PDP_PHI_1*Sensor_val.theta_x-K_PHI_DOT_13*PDP_PHI_3*Sensor_val.theta_y-K_PHI_DOT_15*PDP_PHI_5*Sensor_val.theta_z_abs;
		T_2 = -K_PHI_DOT_27*PDP_PHI_7*phi_dot.phi_dot_x-K_PHI_DOT_28*PDP_PHI_8*phi_dot.phi_dot_y+K_PHI_DOT_27*Helios_val.velocity_x+K_PHI_DOT_28*Helios_val.velocity_y+K_PHI_DOT_25*Helios_val.position_z-K_PHI_DOT_22*PDP_PHI_2*Sensor_val.theta_dot_x-K_PHI_DOT_24*PDP_PHI_4*Sensor_val.theta_dot_y-K_PHI_DOT_26*PDP_PHI_6*Sensor_val.theta_dot_z-K_PHI_DOT_21*PDP_PHI_1*Sensor_val.theta_x-K_PHI_DOT_23*PDP_PHI_3*Sensor_val.theta_y-K_PHI_DOT_25*PDP_PHI_5*Sensor_val.theta_z_abs;
		T_3 = -K_PHI_DOT_37*PDP_PHI_7*phi_dot.phi_dot_x-K_PHI_DOT_38*PDP_PHI_8*phi_dot.phi_dot_y+K_PHI_DOT_37*Helios_val.velocity_x+K_PHI_DOT_38*Helios_val.velocity_y+K_PHI_DOT_35*Helios_val.position_z-K_PHI_DOT_32*PDP_PHI_2*Sensor_val.theta_dot_x-K_PHI_DOT_34*PDP_PHI_4*Sensor_val.theta_dot_y-K_PHI_DOT_36*PDP_PHI_6*Sensor_val.theta_dot_z-K_PHI_DOT_31*PDP_PHI_1*Sensor_val.theta_x-K_PHI_DOT_33*PDP_PHI_3*Sensor_val.theta_y-K_PHI_DOT_35*PDP_PHI_5*Sensor_val.theta_z_abs;
	}


	if(Control_Mode == C_ACCELERATION){
		T_1=K_PHI_15*Helios_val.position_z+K_PHI_17*Helios_val.acceleration_x+K_PHI_19*Helios_val.acceleration_y-K_PHI_12*PDP_PHI_2*Sensor_val.theta_dot_x-K_PHI_14*PDP_PHI_4*Sensor_val.theta_dot_y-K_PHI_16*PDP_PHI_6*Sensor_val.theta_dot_z-K_PHI_11*PDP_PHI_1*Sensor_val.theta_x-K_PHI_13*PDP_PHI_3*Sensor_val.theta_y-K_PHI_15*PDP_PHI_5*Sensor_val.theta_z_abs;
		T_2=K_PHI_25*Helios_val.position_z+K_PHI_27*Helios_val.acceleration_x+K_PHI_29*Helios_val.acceleration_y-K_PHI_22*PDP_PHI_2*Sensor_val.theta_dot_x-K_PHI_24*PDP_PHI_4*Sensor_val.theta_dot_y-K_PHI_26*PDP_PHI_6*Sensor_val.theta_dot_z-K_PHI_21*PDP_PHI_1*Sensor_val.theta_x-K_PHI_23*PDP_PHI_3*Sensor_val.theta_y-K_PHI_25*PDP_PHI_5*Sensor_val.theta_z_abs;
		T_3=K_PHI_35*Helios_val.position_z+K_PHI_37*Helios_val.acceleration_x+K_PHI_39*Helios_val.acceleration_y-K_PHI_32*PDP_PHI_2*Sensor_val.theta_dot_x-K_PHI_34*PDP_PHI_4*Sensor_val.theta_dot_y-K_PHI_36*PDP_PHI_6*Sensor_val.theta_dot_z-K_PHI_31*PDP_PHI_1*Sensor_val.theta_x-K_PHI_33*PDP_PHI_3*Sensor_val.theta_y-K_PHI_35*PDP_PHI_5*Sensor_val.theta_z_abs;
	}


	if(Control_Mode == C_ACCELERATION_WZ){
		T_1 = -K_PHI_12*PDP_PHI_2*Sensor_val.theta_dot_x-K_PHI_14*PDP_PHI_4*Sensor_val.theta_dot_y-K_PHI_11*PDP_PHI_1*Sensor_val.theta_x-K_PHI_13*PDP_PHI_3*Sensor_val.theta_y;
		T_2 = -K_PHI_22*PDP_PHI_2*Sensor_val.theta_dot_x-K_PHI_24*PDP_PHI_4*Sensor_val.theta_dot_y-K_PHI_21*PDP_PHI_1*Sensor_val.theta_x-K_PHI_23*PDP_PHI_3*Sensor_val.theta_y;
		T_3 = -K_PHI_32*PDP_PHI_2*Sensor_val.theta_dot_x-K_PHI_34*PDP_PHI_4*Sensor_val.theta_dot_y-K_PHI_31*PDP_PHI_1*Sensor_val.theta_x-K_PHI_33*PDP_PHI_3*Sensor_val.theta_y;
	}


	if (Control_Mode == C_ROTONDO){
		Helios_val.velocity_x = Helios_val.velocity_x*cosf(Sensor_val.theta_z_abs)+Helios_val.velocity_y*sinf(Sensor_val.theta_z_abs);
		Helios_val.velocity_y = -Helios_val.velocity_x*sinf(Sensor_val.theta_z_abs)+Helios_val.velocity_y*cosf(Sensor_val.theta_z_abs);

		T_1 = -K_PHI_DOT_17*PDP_PHI_7*phi_dot.phi_dot_x-K_PHI_DOT_18*PDP_PHI_8*phi_dot.phi_dot_y+K_PHI_DOT_17*Helios_val.velocity_x+K_PHI_DOT_18*Helios_val.velocity_y+K_PHI_DOT_15*Helios_val.position_z-K_PHI_DOT_12*PDP_PHI_2*Sensor_val.theta_dot_x-K_PHI_DOT_14*PDP_PHI_4*Sensor_val.theta_dot_y-K_PHI_DOT_16*PDP_PHI_6*Sensor_val.theta_dot_z-K_PHI_DOT_11*PDP_PHI_1*Sensor_val.theta_x-K_PHI_DOT_13*PDP_PHI_3*Sensor_val.theta_y-K_PHI_DOT_15*PDP_PHI_5*Sensor_val.theta_z_abs;
		T_2 = -K_PHI_DOT_27*PDP_PHI_7*phi_dot.phi_dot_x-K_PHI_DOT_28*PDP_PHI_8*phi_dot.phi_dot_y+K_PHI_DOT_27*Helios_val.velocity_x+K_PHI_DOT_28*Helios_val.velocity_y+K_PHI_DOT_25*Helios_val.position_z-K_PHI_DOT_22*PDP_PHI_2*Sensor_val.theta_dot_x-K_PHI_DOT_24*PDP_PHI_4*Sensor_val.theta_dot_y-K_PHI_DOT_26*PDP_PHI_6*Sensor_val.theta_dot_z-K_PHI_DOT_21*PDP_PHI_1*Sensor_val.theta_x-K_PHI_DOT_23*PDP_PHI_3*Sensor_val.theta_y-K_PHI_DOT_25*PDP_PHI_5*Sensor_val.theta_z_abs;
		T_3 = -K_PHI_DOT_37*PDP_PHI_7*phi_dot.phi_dot_x-K_PHI_DOT_38*PDP_PHI_8*phi_dot.phi_dot_y+K_PHI_DOT_37*Helios_val.velocity_x+K_PHI_DOT_38*Helios_val.velocity_y+K_PHI_DOT_35*Helios_val.position_z-K_PHI_DOT_32*PDP_PHI_2*Sensor_val.theta_dot_x-K_PHI_DOT_34*PDP_PHI_4*Sensor_val.theta_dot_y-K_PHI_DOT_36*PDP_PHI_6*Sensor_val.theta_dot_z-K_PHI_DOT_31*PDP_PHI_1*Sensor_val.theta_x-K_PHI_DOT_33*PDP_PHI_3*Sensor_val.theta_y-K_PHI_DOT_35*PDP_PHI_5*Sensor_val.theta_z_abs;
	}


	if (Control_Mode == C_HULL_POSITION){
		T_1 = -K_PHI_HULL_18*PDP_PHI_8*phi_dot.phi_dot_x-K_PHI_HULL_110*PDP_PHI_10*phi_dot.phi_dot_y-K_PHI_HULL_17*PDP_PHI_7*phi_dot.phi_x-K_PHI_HULL_19*PDP_PHI_9*phi_dot.phi_y+K_PHI_HULL_15*Helios_val.position_z+K_PHI_HULL_17*Helios_val.position_x+K_PHI_HULL_19*Helios_val.position_y-K_PHI_HULL_12*PDP_PHI_2*Sensor_val.theta_dot_x-K_PHI_HULL_14*PDP_PHI_4*Sensor_val.theta_dot_y-K_PHI_HULL_16*PDP_PHI_6*Sensor_val.theta_dot_z-K_PHI_HULL_11*PDP_PHI_1*Sensor_val.theta_x-K_PHI_HULL_13*PDP_PHI_3*Sensor_val.theta_y-K_PHI_HULL_15*PDP_PHI_5*Sensor_val.theta_z_abs;
		T_2 = -K_PHI_HULL_28*PDP_PHI_8*phi_dot.phi_dot_x-K_PHI_HULL_210*PDP_PHI_10*phi_dot.phi_dot_y-K_PHI_HULL_27*PDP_PHI_7*phi_dot.phi_x-K_PHI_HULL_29*PDP_PHI_9*phi_dot.phi_y+K_PHI_HULL_25*Helios_val.position_z+K_PHI_HULL_27*Helios_val.position_x+K_PHI_HULL_29*Helios_val.position_y-K_PHI_HULL_22*PDP_PHI_2*Sensor_val.theta_dot_x-K_PHI_HULL_24*PDP_PHI_4*Sensor_val.theta_dot_y-K_PHI_HULL_26*PDP_PHI_6*Sensor_val.theta_dot_z-K_PHI_HULL_21*PDP_PHI_1*Sensor_val.theta_x-K_PHI_HULL_23*PDP_PHI_3*Sensor_val.theta_y-K_PHI_HULL_25*PDP_PHI_5*Sensor_val.theta_z_abs;
		T_3 = -K_PHI_HULL_38*PDP_PHI_8*phi_dot.phi_dot_x-K_PHI_HULL_310*PDP_PHI_10*phi_dot.phi_dot_y-K_PHI_HULL_37*PDP_PHI_7*phi_dot.phi_x-K_PHI_HULL_39*PDP_PHI_9*phi_dot.phi_y+K_PHI_HULL_35*Helios_val.position_z+K_PHI_HULL_37*Helios_val.position_x+K_PHI_HULL_39*Helios_val.position_y-K_PHI_HULL_32*PDP_PHI_2*Sensor_val.theta_dot_x-K_PHI_HULL_34*PDP_PHI_4*Sensor_val.theta_dot_y-K_PHI_HULL_36*PDP_PHI_6*Sensor_val.theta_dot_z-K_PHI_HULL_31*PDP_PHI_1*Sensor_val.theta_x-K_PHI_HULL_33*PDP_PHI_3*Sensor_val.theta_y-K_PHI_HULL_35*PDP_PHI_5*Sensor_val.theta_z_abs;
	}

//	T_1 = T_1/2;
//	T_2 = T_2/2;
//	T_3 = T_3/2;

	// Low pass on controller output (roll-off)
	// FIR 4th order

	if (Control_Mode == C_FREEZE){
	T_1 = 0.15550658392382388*T_1+0.22134816409519509*xold1_1+0.24629050396196212*xold2_1+0.22134816409519509*xold3_1+0.15550658392382388*xold4_1;
	T_2 = 0.15550658392382388*T_2+0.22134816409519509*xold1_2+0.24629050396196212*xold2_2+0.22134816409519509*xold3_2+0.15550658392382388*xold4_2;
	T_3 = 0.15550658392382388*T_3+0.22134816409519509*xold1_3+0.24629050396196212*xold2_3+0.22134816409519509*xold3_3+0.15550658392382388*xold4_3;

	xold4_1 = xold3_1;
	xold3_1 = xold2_1;
	xold2_1 = xold1_1;
	xold1_1 = T_1;

	xold4_2 = xold3_2;
	xold3_2 = xold2_2;
	xold2_2 = xold1_2;
	xold1_2 = T_2;

	xold4_3 = xold3_3;
	xold3_3 = xold2_3;
	xold2_3 = xold1_3;
	xold1_3 = T_3;
	}


	// 20 Hz
	//T_1 = 0.15550658392382388*T_1+0.22134816409519509*xold1_1+0.24629050396196212*xold2_1+0.22134816409519509*xold3_1+0.15550658392382388*xold4_1;
	//T_2 = 0.15550658392382388*T_2+0.22134816409519509*xold1_2+0.24629050396196212*xold2_2+0.22134816409519509*xold3_2+0.15550658392382388*xold4_2;
	//T_3 = 0.15550658392382388*T_3+0.22134816409519509*xold1_3+0.24629050396196212*xold2_3+0.22134816409519509*xold3_3+0.15550658392382388*xold4_3;


	// 30 Hz
	//T_1 = 0.10878263918553246*T_1+0.29604095416667797*xold1_1+0.38022813688212925*xold2_1+0.29604095416667797*xold3_1+0.10878263918553246*xold4_1;
	//T_2 = 0.10878263918553246*T_2+0.29604095416667797*xold1_2+0.38022813688212925*xold2_2+0.29604095416667797*xold3_2+0.10878263918553246*xold4_2;
	//T_3 = 0.10878263918553246*T_3+0.29604095416667797*xold1_3+0.38022813688212925*xold2_3+0.29604095416667797*xold3_3+0.10878263918553246*xold4_3;

	// 40 Hz
	//T_1 = -0.006968620618444726*T_1+0.3182335599395813*xold1_1+0.5069708491761723*xold2_1+0.3182335599395813*xold3_1-0.006968620618444726*xold4_1;
	//T_2 = -0.006968620618444726*T_2+0.3182335599395813*xold1_2+0.5069708491761723*xold2_2+0.3182335599395813*xold3_2-0.006968620618444726*xold4_2;
	//T_3 = -0.006968620618444726*T_3+0.3182335599395813*xold1_3+0.5069708491761723*xold2_3+0.3182335599395813*xold3_3-0.006968620618444726*xold4_3;


	/*
	xold4_1 = xold3_1;
	xold3_1 = xold2_1;
	xold2_1 = xold1_1;
	xold1_1 = T_1;

	xold4_2 = xold3_2;
	xold3_2 = xold2_2;
	xold2_2 = xold1_2;
	xold1_2 = T_2;

	xold4_3 = xold3_3;
	xold3_3 = xold2_3;
	xold2_3 = xold1_3;
	xold1_3 = T_3;
	*/

	// saturation
	if (T_1 > MAX_TORQUE){
		T_1 = MAX_TORQUE;
	}

	if (T_1 < -MAX_TORQUE){
		T_1 = -MAX_TORQUE;
	}

	if (T_2 > MAX_TORQUE){
		T_2 = MAX_TORQUE;
	}

	if (T_2 < -MAX_TORQUE){
		T_2 = -MAX_TORQUE;
	}

	if (T_3 > MAX_TORQUE){
		T_3 = MAX_TORQUE;
	}

	if (T_3 < -MAX_TORQUE){
		T_3 = -MAX_TORQUE;
	}

	// Current to send to EPOS
	Motor_current_real.I_1 = T_1/(I_GEAR*K_M)*1000000;	//mA
	Motor_current_real.I_2 = T_2/(I_GEAR*K_M)*1000000;	//mA
	Motor_current_real.I_3 = T_3/(I_GEAR*K_M)*1000000;	//mA

}


/***************************************************************************/
/* Bezeichnung	:	non_linear_control()								   */
/* Funktion		:				                                           */
/* Übergabewert	:	---													   */
/* RÜckgabewert	:	---													   */
/***************************************************************************/

void non_linear_control(void)
{
	float T_1, T_2, T_3; //Real torques
	unsigned char i;
	int index_left, index_right;
	float rel_pos;

	if(Control_Mode == C_VELOCITY){

		// defining lists of linearization points
		float list[] = {-5/0.125, -2.5/0.125, -2/0.125, -1.5/0.125, -1/0.125, -0.5/0.125, 0, 0.5/0.125, 1/0.125, 1.5/0.125, 2/0.125, 2.5/0.125, 5/0.125};			// [rad/s]
		int length = 13;


		index_left = 0;
		index_right = 1;

		// Detect left and right boundary
		for (i=0;i<(length-1);i++){
			if (list[i] < phi_dot.phi_dot_x && list[i+1] > phi_dot.phi_dot_x){
				index_left = i;
				index_right = i+1;
				break;
			}
			else if (list[i] == phi_dot.phi_dot_x){
				index_left = i;
				index_right = i;
				break;
			}
			else if (phi_dot.phi_dot_x < list[0]){
				index_left = 0;
				index_right = 1;
				break;
			}
			else if (phi_dot.phi_dot_x > list[length-1]){
				index_left = length-2;
				index_right = length-1;
				break;
			}
		}

		// Relative distance of the state to the boundaries
		if (index_left==index_right){
			rel_pos = 0;
		}
		else {
			rel_pos = (phi_dot.phi_dot_x-list[index_left])/(list[index_right]-list[index_left]);
		}

		// Interpolation

		// Get all values for K_phi_dot
		float K_PHI_DOT_NLC_11 = lookup_table[index_left*24+0*8+0]+rel_pos*(lookup_table[index_right*24+0*8+0]-lookup_table[index_left*24+0*8+0]);
		float K_PHI_DOT_NLC_12 = lookup_table[index_left*24+0*8+1]+rel_pos*(lookup_table[index_right*24+0*8+1]-lookup_table[index_left*24+0*8+1]);
		float K_PHI_DOT_NLC_13 = lookup_table[index_left*24+0*8+2]+rel_pos*(lookup_table[index_right*24+0*8+2]-lookup_table[index_left*24+0*8+2]);
		float K_PHI_DOT_NLC_14 = lookup_table[index_left*24+0*8+3]+rel_pos*(lookup_table[index_right*24+0*8+3]-lookup_table[index_left*24+0*8+3]);
		float K_PHI_DOT_NLC_15 = lookup_table[index_left*24+0*8+4]+rel_pos*(lookup_table[index_right*24+0*8+4]-lookup_table[index_left*24+0*8+4]);
		float K_PHI_DOT_NLC_16 = lookup_table[index_left*24+0*8+5]+rel_pos*(lookup_table[index_right*24+0*8+5]-lookup_table[index_left*24+0*8+5]);
		float K_PHI_DOT_NLC_17 = lookup_table[index_left*24+0*8+6]+rel_pos*(lookup_table[index_right*24+0*8+6]-lookup_table[index_left*24+0*8+6]);
		float K_PHI_DOT_NLC_18 = lookup_table[index_left*24+0*8+7]+rel_pos*(lookup_table[index_right*24+0*8+7]-lookup_table[index_left*24+0*8+7]);
		float K_PHI_DOT_NLC_21 = lookup_table[index_left*24+1*8+0]+rel_pos*(lookup_table[index_right*24+1*8+0]-lookup_table[index_left*24+1*8+0]);
		float K_PHI_DOT_NLC_22 = lookup_table[index_left*24+1*8+1]+rel_pos*(lookup_table[index_right*24+1*8+1]-lookup_table[index_left*24+1*8+1]);
		float K_PHI_DOT_NLC_23 = lookup_table[index_left*24+1*8+2]+rel_pos*(lookup_table[index_right*24+1*8+2]-lookup_table[index_left*24+1*8+2]);
		float K_PHI_DOT_NLC_24 = lookup_table[index_left*24+1*8+3]+rel_pos*(lookup_table[index_right*24+1*8+3]-lookup_table[index_left*24+1*8+3]);
		float K_PHI_DOT_NLC_25 = lookup_table[index_left*24+1*8+4]+rel_pos*(lookup_table[index_right*24+1*8+4]-lookup_table[index_left*24+1*8+4]);
		float K_PHI_DOT_NLC_26 = lookup_table[index_left*24+1*8+5]+rel_pos*(lookup_table[index_right*24+1*8+5]-lookup_table[index_left*24+1*8+5]);
		float K_PHI_DOT_NLC_27 = lookup_table[index_left*24+1*8+6]+rel_pos*(lookup_table[index_right*24+1*8+6]-lookup_table[index_left*24+1*8+6]);
		float K_PHI_DOT_NLC_28 = lookup_table[index_left*24+1*8+7]+rel_pos*(lookup_table[index_right*24+1*8+7]-lookup_table[index_left*24+1*8+7]);
		float K_PHI_DOT_NLC_31 = lookup_table[index_left*24+2*8+0]+rel_pos*(lookup_table[index_right*24+2*8+0]-lookup_table[index_left*24+2*8+0]);
		float K_PHI_DOT_NLC_32 = lookup_table[index_left*24+2*8+1]+rel_pos*(lookup_table[index_right*24+2*8+1]-lookup_table[index_left*24+2*8+1]);
		float K_PHI_DOT_NLC_33 = lookup_table[index_left*24+2*8+2]+rel_pos*(lookup_table[index_right*24+2*8+2]-lookup_table[index_left*24+2*8+2]);
		float K_PHI_DOT_NLC_34 = lookup_table[index_left*24+2*8+3]+rel_pos*(lookup_table[index_right*24+2*8+3]-lookup_table[index_left*24+2*8+3]);
		float K_PHI_DOT_NLC_35 = lookup_table[index_left*24+2*8+4]+rel_pos*(lookup_table[index_right*24+2*8+4]-lookup_table[index_left*24+2*8+4]);
		float K_PHI_DOT_NLC_36 = lookup_table[index_left*24+2*8+5]+rel_pos*(lookup_table[index_right*24+2*8+5]-lookup_table[index_left*24+2*8+5]);
		float K_PHI_DOT_NLC_37 = lookup_table[index_left*24+2*8+6]+rel_pos*(lookup_table[index_right*24+2*8+6]-lookup_table[index_left*24+2*8+6]);
		float K_PHI_DOT_NLC_38 = lookup_table[index_left*24+2*8+7]+rel_pos*(lookup_table[index_right*24+2*8+7]-lookup_table[index_left*24+2*8+7]);

		T_1 = -K_PHI_DOT_NLC_17*PDP_PHI_7*phi_dot.phi_dot_x-K_PHI_DOT_NLC_18*PDP_PHI_8*phi_dot.phi_dot_y+K_PHI_DOT_NLC_17*Helios_val.velocity_x+K_PHI_DOT_NLC_18*Helios_val.velocity_y+K_PHI_DOT_NLC_15*Helios_val.position_z-K_PHI_DOT_NLC_12*PDP_PHI_2*Sensor_val.theta_dot_x-K_PHI_DOT_NLC_14*PDP_PHI_4*Sensor_val.theta_dot_y-K_PHI_DOT_NLC_16*PDP_PHI_6*Sensor_val.theta_dot_z-K_PHI_DOT_NLC_11*PDP_PHI_1*Sensor_val.theta_x-K_PHI_DOT_NLC_13*PDP_PHI_3*Sensor_val.theta_y-K_PHI_DOT_NLC_15*PDP_PHI_5*Sensor_val.theta_z_abs;
		T_2 = -K_PHI_DOT_NLC_27*PDP_PHI_7*phi_dot.phi_dot_x-K_PHI_DOT_NLC_28*PDP_PHI_8*phi_dot.phi_dot_y+K_PHI_DOT_NLC_27*Helios_val.velocity_x+K_PHI_DOT_NLC_28*Helios_val.velocity_y+K_PHI_DOT_NLC_25*Helios_val.position_z-K_PHI_DOT_NLC_22*PDP_PHI_2*Sensor_val.theta_dot_x-K_PHI_DOT_NLC_24*PDP_PHI_4*Sensor_val.theta_dot_y-K_PHI_DOT_NLC_26*PDP_PHI_6*Sensor_val.theta_dot_z-K_PHI_DOT_NLC_21*PDP_PHI_1*Sensor_val.theta_x-K_PHI_DOT_NLC_23*PDP_PHI_3*Sensor_val.theta_y-K_PHI_DOT_NLC_25*PDP_PHI_5*Sensor_val.theta_z_abs;
		T_3 = -K_PHI_DOT_NLC_37*PDP_PHI_7*phi_dot.phi_dot_x-K_PHI_DOT_NLC_38*PDP_PHI_8*phi_dot.phi_dot_y+K_PHI_DOT_NLC_37*Helios_val.velocity_x+K_PHI_DOT_NLC_38*Helios_val.velocity_y+K_PHI_DOT_NLC_35*Helios_val.position_z-K_PHI_DOT_NLC_32*PDP_PHI_2*Sensor_val.theta_dot_x-K_PHI_DOT_NLC_34*PDP_PHI_4*Sensor_val.theta_dot_y-K_PHI_DOT_NLC_36*PDP_PHI_6*Sensor_val.theta_dot_z-K_PHI_DOT_NLC_31*PDP_PHI_1*Sensor_val.theta_x-K_PHI_DOT_NLC_33*PDP_PHI_3*Sensor_val.theta_y-K_PHI_DOT_NLC_35*PDP_PHI_5*Sensor_val.theta_z_abs;
	}

	if (Control_Mode == C_FREEZE){
		T_1 = -K_PHI_FREEZE_18*PDP_PHI_8*phi_dot.phi_dot_x-K_PHI_FREEZE_110*PDP_PHI_10*phi_dot.phi_dot_y-K_PHI_FREEZE_17*PDP_PHI_7*phi_dot.phi_x-K_PHI_FREEZE_19*PDP_PHI_9*phi_dot.phi_y-K_PHI_FREEZE_12*PDP_PHI_2*Sensor_val.theta_dot_x-K_PHI_FREEZE_14*PDP_PHI_4*Sensor_val.theta_dot_y-K_PHI_FREEZE_16*PDP_PHI_6*Sensor_val.theta_dot_z-K_PHI_FREEZE_11*PDP_PHI_1*Sensor_val.theta_x-K_PHI_FREEZE_13*PDP_PHI_3*Sensor_val.theta_y-K_PHI_FREEZE_15*PDP_PHI_5*Sensor_val.theta_z_abs;
		T_2 = -K_PHI_FREEZE_28*PDP_PHI_8*phi_dot.phi_dot_x-K_PHI_FREEZE_210*PDP_PHI_10*phi_dot.phi_dot_y-K_PHI_FREEZE_27*PDP_PHI_7*phi_dot.phi_x-K_PHI_FREEZE_29*PDP_PHI_9*phi_dot.phi_y-K_PHI_FREEZE_22*PDP_PHI_2*Sensor_val.theta_dot_x-K_PHI_FREEZE_24*PDP_PHI_4*Sensor_val.theta_dot_y-K_PHI_FREEZE_26*PDP_PHI_6*Sensor_val.theta_dot_z-K_PHI_FREEZE_21*PDP_PHI_1*Sensor_val.theta_x-K_PHI_FREEZE_23*PDP_PHI_3*Sensor_val.theta_y-K_PHI_FREEZE_25*PDP_PHI_5*Sensor_val.theta_z_abs;
		T_3 = -K_PHI_FREEZE_38*PDP_PHI_8*phi_dot.phi_dot_x-K_PHI_FREEZE_310*PDP_PHI_10*phi_dot.phi_dot_y-K_PHI_FREEZE_37*PDP_PHI_7*phi_dot.phi_x-K_PHI_FREEZE_39*PDP_PHI_9*phi_dot.phi_y-K_PHI_FREEZE_32*PDP_PHI_2*Sensor_val.theta_dot_x-K_PHI_FREEZE_34*PDP_PHI_4*Sensor_val.theta_dot_y-K_PHI_FREEZE_36*PDP_PHI_6*Sensor_val.theta_dot_z-K_PHI_FREEZE_31*PDP_PHI_1*Sensor_val.theta_x-K_PHI_FREEZE_33*PDP_PHI_3*Sensor_val.theta_y-K_PHI_FREEZE_35*PDP_PHI_5*Sensor_val.theta_z_abs;
	}

	if (Control_Mode == C_POSITION){
		T_1 = -K_PHI_18*PDP_PHI_8*phi_dot.phi_dot_x-K_PHI_110*PDP_PHI_10*phi_dot.phi_dot_y-K_PHI_17*PDP_PHI_7*phi_dot.phi_x-K_PHI_19*PDP_PHI_9*phi_dot.phi_y+K_PHI_15*Helios_val.position_z+K_PHI_17*Helios_val.position_x+K_PHI_19*Helios_val.position_y-K_PHI_12*PDP_PHI_2*Sensor_val.theta_dot_x-K_PHI_14*PDP_PHI_4*Sensor_val.theta_dot_y-K_PHI_16*PDP_PHI_6*Sensor_val.theta_dot_z-K_PHI_11*PDP_PHI_1*Sensor_val.theta_x-K_PHI_13*PDP_PHI_3*Sensor_val.theta_y-K_PHI_15*PDP_PHI_5*Sensor_val.theta_z_abs;
		T_2 = -K_PHI_28*PDP_PHI_8*phi_dot.phi_dot_x-K_PHI_210*PDP_PHI_10*phi_dot.phi_dot_y-K_PHI_27*PDP_PHI_7*phi_dot.phi_x-K_PHI_29*PDP_PHI_9*phi_dot.phi_y+K_PHI_25*Helios_val.position_z+K_PHI_27*Helios_val.position_x+K_PHI_29*Helios_val.position_y-K_PHI_22*PDP_PHI_2*Sensor_val.theta_dot_x-K_PHI_24*PDP_PHI_4*Sensor_val.theta_dot_y-K_PHI_26*PDP_PHI_6*Sensor_val.theta_dot_z-K_PHI_21*PDP_PHI_1*Sensor_val.theta_x-K_PHI_23*PDP_PHI_3*Sensor_val.theta_y-K_PHI_25*PDP_PHI_5*Sensor_val.theta_z_abs;
		T_3 = -K_PHI_38*PDP_PHI_8*phi_dot.phi_dot_x-K_PHI_310*PDP_PHI_10*phi_dot.phi_dot_y-K_PHI_37*PDP_PHI_7*phi_dot.phi_x-K_PHI_39*PDP_PHI_9*phi_dot.phi_y+K_PHI_35*Helios_val.position_z+K_PHI_37*Helios_val.position_x+K_PHI_39*Helios_val.position_y-K_PHI_32*PDP_PHI_2*Sensor_val.theta_dot_x-K_PHI_34*PDP_PHI_4*Sensor_val.theta_dot_y-K_PHI_36*PDP_PHI_6*Sensor_val.theta_dot_z-K_PHI_31*PDP_PHI_1*Sensor_val.theta_x-K_PHI_33*PDP_PHI_3*Sensor_val.theta_y-K_PHI_35*PDP_PHI_5*Sensor_val.theta_z_abs;
	}

	if(Control_Mode == C_ACCELERATION){
		T_1 = K_PHI_15*Helios_val.position_z+K_PHI_17*Helios_val.acceleration_x+K_PHI_19*Helios_val.acceleration_y-K_PHI_12*PDP_PHI_2*Sensor_val.theta_dot_x-K_PHI_14*PDP_PHI_4*Sensor_val.theta_dot_y-K_PHI_16*PDP_PHI_6*Sensor_val.theta_dot_z-K_PHI_11*PDP_PHI_1*Sensor_val.theta_x-K_PHI_13*PDP_PHI_3*Sensor_val.theta_y-K_PHI_15*PDP_PHI_5*Sensor_val.theta_z_abs;
		T_2 = K_PHI_25*Helios_val.position_z+K_PHI_27*Helios_val.acceleration_x+K_PHI_29*Helios_val.acceleration_y-K_PHI_22*PDP_PHI_2*Sensor_val.theta_dot_x-K_PHI_24*PDP_PHI_4*Sensor_val.theta_dot_y-K_PHI_26*PDP_PHI_6*Sensor_val.theta_dot_z-K_PHI_21*PDP_PHI_1*Sensor_val.theta_x-K_PHI_23*PDP_PHI_3*Sensor_val.theta_y-K_PHI_25*PDP_PHI_5*Sensor_val.theta_z_abs;
		T_3 = K_PHI_35*Helios_val.position_z+K_PHI_37*Helios_val.acceleration_x+K_PHI_39*Helios_val.acceleration_y-K_PHI_32*PDP_PHI_2*Sensor_val.theta_dot_x-K_PHI_34*PDP_PHI_4*Sensor_val.theta_dot_y-K_PHI_36*PDP_PHI_6*Sensor_val.theta_dot_z-K_PHI_31*PDP_PHI_1*Sensor_val.theta_x-K_PHI_33*PDP_PHI_3*Sensor_val.theta_y-K_PHI_35*PDP_PHI_5*Sensor_val.theta_z_abs;
	}

	if(Control_Mode == C_ACCELERATION_WZ){
		T_1 = -K_PHI_12*PDP_PHI_2*Sensor_val.theta_dot_x-K_PHI_14*PDP_PHI_4*Sensor_val.theta_dot_y-K_PHI_11*PDP_PHI_1*Sensor_val.theta_x-K_PHI_13*PDP_PHI_3*Sensor_val.theta_y;
		T_2 = -K_PHI_22*PDP_PHI_2*Sensor_val.theta_dot_x-K_PHI_24*PDP_PHI_4*Sensor_val.theta_dot_y-K_PHI_21*PDP_PHI_1*Sensor_val.theta_x-K_PHI_23*PDP_PHI_3*Sensor_val.theta_y;
		T_3 = -K_PHI_32*PDP_PHI_2*Sensor_val.theta_dot_x-K_PHI_34*PDP_PHI_4*Sensor_val.theta_dot_y-K_PHI_31*PDP_PHI_1*Sensor_val.theta_x-K_PHI_33*PDP_PHI_3*Sensor_val.theta_y;
	}


	if (Control_Mode == C_ROTONDO){
		Helios_val.velocity_x = Helios_val.velocity_x*cosf(Sensor_val.theta_z_abs)-Helios_val.velocity_y*sinf(Sensor_val.theta_z_abs);
		Helios_val.velocity_y = Helios_val.velocity_x*sinf(Sensor_val.theta_z_abs)+Helios_val.velocity_y*cosf(Sensor_val.theta_z_abs);

		T_1 = -K_PHI_DOT_17*PDP_PHI_7*phi_dot.phi_dot_x-K_PHI_DOT_18*PDP_PHI_8*phi_dot.phi_dot_y+K_PHI_DOT_17*Helios_val.velocity_x+K_PHI_DOT_18*Helios_val.velocity_y+K_PHI_DOT_15*Helios_val.position_z-K_PHI_DOT_12*PDP_PHI_2*Sensor_val.theta_dot_x-K_PHI_DOT_14*PDP_PHI_4*Sensor_val.theta_dot_y-K_PHI_DOT_16*PDP_PHI_6*Sensor_val.theta_dot_z-K_PHI_DOT_11*PDP_PHI_1*Sensor_val.theta_x-K_PHI_DOT_13*PDP_PHI_3*Sensor_val.theta_y-K_PHI_DOT_15*PDP_PHI_5*Sensor_val.theta_z_abs;
		T_2 = -K_PHI_DOT_27*PDP_PHI_7*phi_dot.phi_dot_x-K_PHI_DOT_28*PDP_PHI_8*phi_dot.phi_dot_y+K_PHI_DOT_27*Helios_val.velocity_x+K_PHI_DOT_28*Helios_val.velocity_y+K_PHI_DOT_25*Helios_val.position_z-K_PHI_DOT_22*PDP_PHI_2*Sensor_val.theta_dot_x-K_PHI_DOT_24*PDP_PHI_4*Sensor_val.theta_dot_y-K_PHI_DOT_26*PDP_PHI_6*Sensor_val.theta_dot_z-K_PHI_DOT_21*PDP_PHI_1*Sensor_val.theta_x-K_PHI_DOT_23*PDP_PHI_3*Sensor_val.theta_y-K_PHI_DOT_25*PDP_PHI_5*Sensor_val.theta_z_abs;
		T_3 = -K_PHI_DOT_37*PDP_PHI_7*phi_dot.phi_dot_x-K_PHI_DOT_38*PDP_PHI_8*phi_dot.phi_dot_y+K_PHI_DOT_37*Helios_val.velocity_x+K_PHI_DOT_38*Helios_val.velocity_y+K_PHI_DOT_35*Helios_val.position_z-K_PHI_DOT_32*PDP_PHI_2*Sensor_val.theta_dot_x-K_PHI_DOT_34*PDP_PHI_4*Sensor_val.theta_dot_y-K_PHI_DOT_36*PDP_PHI_6*Sensor_val.theta_dot_z-K_PHI_DOT_31*PDP_PHI_1*Sensor_val.theta_x-K_PHI_DOT_33*PDP_PHI_3*Sensor_val.theta_y-K_PHI_DOT_35*PDP_PHI_5*Sensor_val.theta_z_abs;
	}


	if (Control_Mode == C_HULL_POSITION){
		T_1 = -K_PHI_HULL_18*PDP_PHI_8*phi_dot.phi_dot_x-K_PHI_HULL_110*PDP_PHI_10*phi_dot.phi_dot_y-K_PHI_HULL_17*PDP_PHI_7*phi_dot.phi_x-K_PHI_HULL_19*PDP_PHI_9*phi_dot.phi_y+K_PHI_HULL_15*Helios_val.position_z+K_PHI_HULL_17*Helios_val.position_x+K_PHI_HULL_19*Helios_val.position_y-K_PHI_HULL_12*PDP_PHI_2*Sensor_val.theta_dot_x-K_PHI_HULL_14*PDP_PHI_4*Sensor_val.theta_dot_y-K_PHI_HULL_16*PDP_PHI_6*Sensor_val.theta_dot_z-K_PHI_HULL_11*PDP_PHI_1*Sensor_val.theta_x-K_PHI_HULL_13*PDP_PHI_3*Sensor_val.theta_y-K_PHI_HULL_15*PDP_PHI_5*Sensor_val.theta_z_abs;
		T_2 = -K_PHI_HULL_28*PDP_PHI_8*phi_dot.phi_dot_x-K_PHI_HULL_210*PDP_PHI_10*phi_dot.phi_dot_y-K_PHI_HULL_27*PDP_PHI_7*phi_dot.phi_x-K_PHI_HULL_29*PDP_PHI_9*phi_dot.phi_y+K_PHI_HULL_25*Helios_val.position_z+K_PHI_HULL_27*Helios_val.position_x+K_PHI_HULL_29*Helios_val.position_y-K_PHI_HULL_22*PDP_PHI_2*Sensor_val.theta_dot_x-K_PHI_HULL_24*PDP_PHI_4*Sensor_val.theta_dot_y-K_PHI_HULL_26*PDP_PHI_6*Sensor_val.theta_dot_z-K_PHI_HULL_21*PDP_PHI_1*Sensor_val.theta_x-K_PHI_HULL_23*PDP_PHI_3*Sensor_val.theta_y-K_PHI_HULL_25*PDP_PHI_5*Sensor_val.theta_z_abs;
		T_3 = -K_PHI_HULL_38*PDP_PHI_8*phi_dot.phi_dot_x-K_PHI_HULL_310*PDP_PHI_10*phi_dot.phi_dot_y-K_PHI_HULL_37*PDP_PHI_7*phi_dot.phi_x-K_PHI_HULL_39*PDP_PHI_9*phi_dot.phi_y+K_PHI_HULL_35*Helios_val.position_z+K_PHI_HULL_37*Helios_val.position_x+K_PHI_HULL_39*Helios_val.position_y-K_PHI_HULL_32*PDP_PHI_2*Sensor_val.theta_dot_x-K_PHI_HULL_34*PDP_PHI_4*Sensor_val.theta_dot_y-K_PHI_HULL_36*PDP_PHI_6*Sensor_val.theta_dot_z-K_PHI_HULL_31*PDP_PHI_1*Sensor_val.theta_x-K_PHI_HULL_33*PDP_PHI_3*Sensor_val.theta_y-K_PHI_HULL_35*PDP_PHI_5*Sensor_val.theta_z_abs;
	}


	// Low pass on controller output (roll-off)
	// FIR 4th order
	if (Control_Mode == C_FREEZE){
	T_1 = 0.15550658392382388*T_1+0.22134816409519509*xold1_1+0.24629050396196212*xold2_1+0.22134816409519509*xold3_1+0.15550658392382388*xold4_1;
	T_2 = 0.15550658392382388*T_2+0.22134816409519509*xold1_2+0.24629050396196212*xold2_2+0.22134816409519509*xold3_2+0.15550658392382388*xold4_2;
	T_3 = 0.15550658392382388*T_3+0.22134816409519509*xold1_3+0.24629050396196212*xold2_3+0.22134816409519509*xold3_3+0.15550658392382388*xold4_3;

	xold4_1 = xold3_1;
	xold3_1 = xold2_1;
	xold2_1 = xold1_1;
	xold1_1 = T_1;

	xold4_2 = xold3_2;
	xold3_2 = xold2_2;
	xold2_2 = xold1_2;
	xold1_2 = T_2;

	xold4_3 = xold3_3;
	xold3_3 = xold2_3;
	xold2_3 = xold1_3;
	xold1_3 = T_3;
	}


	// saturation
	if (T_1 > MAX_TORQUE){
		T_1 = MAX_TORQUE;
	}

	if (T_1 < -MAX_TORQUE){
		T_1 = -MAX_TORQUE;
	}

	if (T_2 > MAX_TORQUE){
		T_2 = MAX_TORQUE;
	}

	if (T_2 < -MAX_TORQUE){
		T_2 = -MAX_TORQUE;
	}

	if (T_3 > MAX_TORQUE){
		T_3 = MAX_TORQUE;
	}

	if (T_3 < -MAX_TORQUE){
		T_3 = -MAX_TORQUE;
	}

	// Current to send to EPOS
	Motor_current_real.I_1 = T_1/(I_GEAR*K_M)*1000000;	//mA
	Motor_current_real.I_2 = T_2/(I_GEAR*K_M)*1000000;	//mA
	Motor_current_real.I_3 = T_3/(I_GEAR*K_M)*1000000;	//mA
}


/***************************************************************************/
/* Bezeichnung	:	state_feedback_control()							   */
/* Funktion		:	Regler mit planar Modell                               */
/* Übergabewert	:	---													   */
/* Rückgabewert	:	---													   */
/***************************************************************************/
void planar_control(void)
{
	float e = 0;

	// calculate torques

	float Control_output_x;
	float Control_output_y;
	float Control_output_x_filtered;
	float Control_output_y_filtered;


	// multiply states by controller gains
	if (Control_Mode == C_POSITION){
		Control_output_x = (phi_dot.phi_x*K_X_POS_1+phi_dot.phi_dot_x*K_X_POS_2+Sensor_val.theta_x*K_X_POS_3+Sensor_val.theta_dot_x*K_X_POS_4);
		Control_output_y = (phi_dot.phi_y*K_X_POS_1+phi_dot.phi_dot_y*K_X_POS_2+Sensor_val.theta_y*K_X_POS_3+Sensor_val.theta_dot_y*K_X_POS_4);
	}

	if(Control_Mode == C_VELOCITY){
		Control_output_x = (phi_dot.phi_dot_x*K_X_SPEED_2+Sensor_val.theta_x*K_X_SPEED_3+Sensor_val.theta_dot_x*K_X_SPEED_4);
		Control_output_y = (phi_dot.phi_dot_y*K_X_SPEED_2+Sensor_val.theta_y*K_X_SPEED_3+Sensor_val.theta_dot_y*K_X_SPEED_4);
	}

	if(Control_Mode == C_ACCELERATION){
		Control_output_x = (Sensor_val.theta_x*K_X_ACC_3+Sensor_val.theta_dot_x*K_X_ACC_4);
		Control_output_y = (Sensor_val.theta_y*K_X_ACC_3+Sensor_val.theta_dot_y*K_X_ACC_4);
	}

	/*
	// FIR 4th order
	Control_output_x_filtered=0.0515215858*Control_output_x+0.14400211607*xold1_1+0.1943124602979*xold2_1+0.14400211607*xold3_1+0.0515215858*xold4_1;
	Control_output_y_filtered=0.0515215858*Control_output_y+0.14400211607*xold1_2+0.1943124602979*xold2_2+0.14400211607*xold3_2+0.0515215858*xold4_2;

	xold4_1 = xold3_1;
	xold3_1 = xold2_1;
	xold2_1 = xold1_1;
	xold1_1 = Control_output_x;

	xold4_2 = xold3_2;
	xold3_2 = xold2_2;
	xold2_2 = xold1_2;
	xold1_2 = Control_output_y;
	*/

	Control_output_x_filtered = Control_output_x;
	Control_output_y_filtered = Control_output_y;

	// adding setpoint
	if (Control_Mode == C_POSITION){
		Motor_torque_xyz.T_x = N_X_POS*Helios_val.position_x-Control_output_x_filtered;
		Motor_torque_xyz.T_y = N_X_POS*Helios_val.position_y-Control_output_y_filtered;
	}

	if (Control_Mode == C_VELOCITY){
		Motor_torque_xyz.T_x = N_X_SPEED*Helios_val.velocity_x-Control_output_x_filtered;
		Motor_torque_xyz.T_y = N_X_SPEED*Helios_val.velocity_y-Control_output_y_filtered;
	}

	if (Control_Mode == C_ACCELERATION){
		Motor_torque_xyz.T_x = N_X_ACC*Helios_val.acceleration_x-Control_output_x_filtered;
		Motor_torque_xyz.T_y = N_X_ACC*Helios_val.acceleration_y-Control_output_y_filtered;
	}

	// PID Control z-axis
	e = Helios_val.position_z - Sensor_val.theta_z_abs;									//Vergleich
	esum = esum + e;																	//Integration I-Anteil
	Motor_torque_xyz.T_z = KP*e + KI*Ts*esum + KD/Ts*(e-ealt);							//Reglergleichung
	ealt = e;

	// sign change
	Motor_torque_xyz.T_x = -Motor_torque_xyz.T_x;

	// saturation
	if (Motor_torque_xyz.T_x > MAX_VIRTUAL_TORQUE){
		Motor_torque_xyz.T_x = MAX_VIRTUAL_TORQUE;
	}

	if (Motor_torque_xyz.T_x < -MAX_VIRTUAL_TORQUE){
		Motor_torque_xyz.T_x = -MAX_VIRTUAL_TORQUE;
	}

	if (Motor_torque_xyz.T_y > MAX_VIRTUAL_TORQUE){
		Motor_torque_xyz.T_y = MAX_VIRTUAL_TORQUE;
	}

	if (Motor_torque_xyz.T_y < -MAX_VIRTUAL_TORQUE){
		Motor_torque_xyz.T_y = -MAX_VIRTUAL_TORQUE;
	}

	if (Motor_torque_xyz.T_z > MAX_VIRTUAL_TORQUE){
		Motor_torque_xyz.T_z = MAX_VIRTUAL_TORQUE;
	}

	if (Motor_torque_xyz.T_z < -MAX_VIRTUAL_TORQUE){
		Motor_torque_xyz.T_z = -MAX_VIRTUAL_TORQUE;
	}

	float Pi_4 = ALPHA;
	float sin_alpha = sin(ALPHA);
	float sin_beta = sin(BETA);
	float cos_alpha = cos(ALPHA);
	float cos_beta = cos(BETA);
	float sqrt_3 = 1.7320508;

	float T_1, T_2, T_3; //Real torques

	//Conversion for real motor #1
	T_1 = 1./3*(Motor_torque_xyz.T_z + 2./cos_alpha*(Motor_torque_xyz.T_x*cos_beta - Motor_torque_xyz.T_y*sin_beta))/I_GEAR;
	//Conversion for real motor #2
	T_2 = 1./3*(Motor_torque_xyz.T_z + 1./cos_alpha*(sin_beta*((-sqrt_3)*Motor_torque_xyz.T_x + Motor_torque_xyz.T_y)-cos_beta*(Motor_torque_xyz.T_x + sqrt_3*Motor_torque_xyz.T_y)))/I_GEAR;
	//Conversion for real motor #3
	T_3 = 1./3*(Motor_torque_xyz.T_z + 1./cos_alpha*(sin_beta*(sqrt_3*Motor_torque_xyz.T_x + Motor_torque_xyz.T_y)+cos_beta*(-Motor_torque_xyz.T_x + sqrt_3*Motor_torque_xyz.T_y)))/I_GEAR;

	//Calculate current
	Motor_current_real.I_1 = T_1/K_M*1000000;	//mA
	Motor_current_real.I_2 = T_2/K_M*1000000;	//mA
	Motor_current_real.I_3 = T_3/K_M*1000000;	//mA
}


