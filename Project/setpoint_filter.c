/*=========================================================================*/
/* 																		   */
/*	Autor 				: Corsin Gwerder								   */
/*	File				: setpoint_filter.c							   	   */
/*	Erstellt			: 01.05.2010									   */
/* 	Aktuelle Version	: v1.00										 	   */
/*																	       */
/*=========================================================================*/
/*																		   */
/*	Änderungshistory													   */
/*-------------------------------------------------------------------------*/
/*	v1.00	| Grundversion Initialisierung								   */
/* 	v1.01	|															   */
/*=========================================================================*/

#include "extern.h"
#include "setpoint_filter.h"
#include "Defines_Ballbot.h"
#include <math.h>

/***************************************************************************/
/* Bezeichnung	:	setpoint_filter()									   */
/* Funktion		:	Geschwindigkeitssollwerte, filtern				       */
/* Übergabewert	:	---													   */
/* Rückgabewert	:	---													   */
/***************************************************************************/

void setpoint_filter(void)
{
//---------------------------------------------------------------------------------------------------------
	// Positionsrampe
	if(Control_Mode == C_POSITION || Control_Mode == C_HULL_POSITION){
		// Rampe
		if  (fabsf(Helios_val.position_x_raw-old_phi_x_soll) < PHI_DPOS_MAX*Ts){
			Helios_val.position_x = Helios_val.position_x_raw;
			old_phi_x_soll = Helios_val.position_x_raw;
		} else {
			Helios_val.position_x = old_phi_x_soll+PHI_DPOS_MAX*Ts*sign(Helios_val.position_x_raw-old_phi_x_soll);
			old_phi_x_soll=Helios_val.position_x;
		}

		if  (fabsf(Helios_val.position_y_raw-old_phi_y_soll) < PHI_DPOS_MAX*Ts){
			Helios_val.position_y = Helios_val.position_y_raw;
			old_phi_y_soll = Helios_val.position_y_raw;
		} else {
			Helios_val.position_y = old_phi_y_soll+PHI_DPOS_MAX*Ts*sign(Helios_val.position_y_raw-old_phi_y_soll);
			old_phi_y_soll = Helios_val.position_y;
		}
	}


//---------------------------------------------------------------------------------------------------------
	// Geschwindigkeitsbegrenzung und Rampe
	if(Control_Mode == C_VELOCITY || Control_Mode == C_ROTONDO){
		// Sollwert begrenzen
		if (Helios_val.velocity_x_raw > PHI_DOT_MAX){
			Helios_val.velocity_x_raw = PHI_DOT_MAX;
		}
		if (Helios_val.velocity_x_raw < -PHI_DOT_MAX){
			Helios_val.velocity_x_raw = -PHI_DOT_MAX;
		}

		if (Helios_val.velocity_y_raw > PHI_DOT_MAX){
			Helios_val.velocity_y_raw = PHI_DOT_MAX;
		}
		if (Helios_val.velocity_y_raw < -PHI_DOT_MAX){
			Helios_val.velocity_y_raw = -PHI_DOT_MAX;
		}

		// Rampe
		if  (fabsf(Helios_val.velocity_x_raw-old_phi_dot_x_soll) < PHI_DDOT_MAX*Ts){
			Helios_val.velocity_x = Helios_val.velocity_x_raw;
			old_phi_dot_x_soll = Helios_val.velocity_x_raw;
		} else {
			Helios_val.velocity_x = old_phi_dot_x_soll+PHI_DDOT_MAX*Ts*sign(Helios_val.velocity_x_raw-old_phi_dot_x_soll);
			old_phi_dot_x_soll=Helios_val.velocity_x;
		}

		if  (fabsf(Helios_val.velocity_y_raw-old_phi_dot_y_soll) < PHI_DDOT_MAX*Ts){
			Helios_val.velocity_y = Helios_val.velocity_y_raw;
			old_phi_dot_y_soll = Helios_val.velocity_y_raw;
		} else {
			Helios_val.velocity_y = old_phi_dot_y_soll+PHI_DDOT_MAX*Ts*sign(Helios_val.velocity_y_raw-old_phi_dot_y_soll);
			old_phi_dot_y_soll = Helios_val.velocity_y;
		}

		// Absicherung wenn soll am ist Wert davon läuft
		if (fabsf(Helios_val.velocity_x-phi_dot.phi_dot_x) > PHI_DOT_MAX_DIFF){
			Helios_val.velocity_x = old_phi_dot_x_soll;
		}
		if (fabsf(Helios_val.velocity_y-phi_dot.phi_dot_y) > PHI_DOT_MAX_DIFF){
			Helios_val.velocity_y = old_phi_dot_y_soll;
		}
	}

//---------------------------------------------------------------------------------------------------------
	// Im Beschleunigungsmodus sind Sollwerte immer NULL
	if(Control_Mode == C_ACCELERATION || Control_Mode == C_ACCELERATION_WZ){
		Helios_val.acceleration_x = 0;
		Helios_val.acceleration_y = 0;
	}

//---------------------------------------------------------------------------------------------------------
	// Rampe für Drehung um eigene Achse
	if  (fabsf(Helios_val.position_z_raw-old_theta_z_soll) < THETA_Z_DOT_MAX*Ts){
		Helios_val.position_z = Helios_val.position_z_raw;
		old_theta_z_soll = Helios_val.position_z_raw;
	} else {
		Helios_val.position_z = old_theta_z_soll+THETA_Z_DOT_MAX*Ts*sign(Helios_val.position_z_raw-old_theta_z_soll);
		old_theta_z_soll = Helios_val.position_z;
	}
	// Absicherung wenn soll am ist Wert davon läuft
	if (fabsf(Helios_val.position_z-Sensor_val.theta_z_abs) > THETA_Z_MAX_DIFF){
		Helios_val.position_z = old_theta_z_soll;
	}

//---------------------------------------------------------------------------------------------------------
	// ABS
	/*
	// Verhinderung der Kugelumdrehung um z
	if (fabsf(phi_dot.phi_dot_z) > PHI_DOT_Z_MAX){
		if (Helios_val.position_z_raw < Sensor_val.theta_z_abs){
			Helios_val.position_z = THETA_Z_SETPOINT_MAX+Sensor_val.theta_z_abs;
		} else if (Helios_val.position_z_raw > Sensor_val.theta_z_abs){
			Helios_val.position_z = -THETA_Z_SETPOINT_MAX*THETA_Z_SETPOINT_MAX+Sensor_val.theta_z_abs;
		}
	}

	// Rampe für Kugelumdrehung
	if  (fabsf(Helios_val.position_z-old_theta_z_soll) < THETA_Z_DOT_ABS*Ts){
		old_theta_z_soll = Helios_val.position_z;
	} else {
		Helios_val.position_z = old_theta_z_soll+THETA_Z_DOT_ABS*Ts*sign(Helios_val.position_z-old_theta_z_soll);
		old_theta_z_soll = Helios_val.position_z;
	}
	*/
//-------------------------------------------------------------------------------------------------------------

}




/***************************************************************************/
/* Bezeichnung	:	sign()												   */
/* Funktion		:	Signumfunktion, gibt Vorzeichen zurück (1,0 oder -1)   */
/* Übergabewert	:	---													   */
/* Rückgabewert	:	---													   */
/***************************************************************************/

int sign(float x){
	return -(x<0) + (x>0);
}
