#include "extern.h"
#include "odometry.h"
#include "Defines_Ballbot.h"
#include <math.h>


/***************************************************************************/
/* Bezeichnung	:	odometry_trivial()									   */
/* Funktion		:	Berechnet aus den Motorengeschwindigkeiten die 	       */
/*					Geschwindigketi in x und y Richtung					   */
/*					Berechnet theta_z_abs und absolute Position im Raum	   */
/* Übergabewert	:	---													   */
/* Rückgabewert	:	---													   */
/***************************************************************************/
void odometry(void)
{
	float diff_theta_z;

	//Sensor_val.psi_dot_1 = -Sensor_val.psi_dot_1;
	//Sensor_val.psi_dot_2 = -Sensor_val.psi_dot_2;
	//Sensor_val.psi_dot_3 = -Sensor_val.psi_dot_3;

	Sensor_val.psi_dot_1 = Sensor_val.motor_vel_1/(60.*I_GEAR)*2*PI;
	Sensor_val.psi_dot_2 = Sensor_val.motor_vel_2/(60.*I_GEAR)*2*PI;
	Sensor_val.psi_dot_3 = Sensor_val.motor_vel_3/(60.*I_GEAR)*2*PI;

	phi_dot.phi_dot_x=1/(3*RK)*(sqrtf(6)*RW*sinf(Sensor_val.theta_x)*sinf(Sensor_val.theta_y)*(-Sensor_val.psi_dot_2+Sensor_val.psi_dot_3)+sqrtf(2)*RW*cosf(Sensor_val.theta_x)*sinf(Sensor_val.theta_y)*(Sensor_val.psi_dot_1+Sensor_val.psi_dot_2+Sensor_val.psi_dot_3)+cosf(Sensor_val.theta_y)*(sqrtf(2)*RW*(-2*Sensor_val.psi_dot_1+Sensor_val.psi_dot_2+Sensor_val.psi_dot_3)+3*RK*Sensor_val.theta_dot_x));
	phi_dot.phi_dot_y=(sqrtf(6)*RW*cosf(Sensor_val.theta_x)*(-Sensor_val.psi_dot_2+Sensor_val.psi_dot_3)-sqrtf(2)*RW*sinf(Sensor_val.theta_x)*(Sensor_val.psi_dot_1+Sensor_val.psi_dot_2+Sensor_val.psi_dot_3)+3*RK*Sensor_val.theta_dot_y)/(3*RK);
	phi_dot.phi_dot_z=1/(3*RK)*(sqrtf(2)*RW*(cosf(Sensor_val.theta_x)*cosf(Sensor_val.theta_y)+2*sinf(Sensor_val.theta_y))*Sensor_val.psi_dot_1+sqrtf(2)*RW*(sqrtf(3)*cosf(Sensor_val.theta_y)*sinf(Sensor_val.theta_x)*(-Sensor_val.psi_dot_2+Sensor_val.psi_dot_3)+cosf(Sensor_val.theta_x)*cosf(Sensor_val.theta_y)*(Sensor_val.psi_dot_2+Sensor_val.psi_dot_3)-sinf(Sensor_val.theta_y)*(Sensor_val.psi_dot_2+Sensor_val.psi_dot_3))+3*RK*(-sinf(Sensor_val.theta_y)*Sensor_val.theta_dot_x+Sensor_val.theta_dot_z));

	phi_dot.phi_x += (old_phi_dot_x+phi_dot.phi_dot_x)/2*Ts;		// Sum(dx) = Sum(vx*Ts)
	phi_dot.phi_y += (old_phi_dot_y+phi_dot.phi_dot_y)/2*Ts;		// Sum(dy) = Sum(vy*Ts)
	phi_dot.phi_z += (old_phi_dot_z+phi_dot.phi_dot_z)/2*Ts;		// Sum(dz) = Sum(vz*Ts)

	old_phi_dot_x = phi_dot.phi_dot_x;
	old_phi_dot_y = phi_dot.phi_dot_y;
	old_phi_dot_z = phi_dot.phi_dot_z;



	if(first_time < 3)
	{
		offset_z = Sensor_val.theta_z;
		first_time++;

	} else {
		diff_theta_z = (Sensor_val.theta_z-offset_z) - theta_z_tmp;

		// Calculate the absolute Value of theta_z
		if(diff_theta_z < 0)								// abs
			diff_theta_z = -diff_theta_z;

		if(diff_theta_z > 0.7 && Sensor_val.theta_z > 0)
		{
			count_jump--;
		}
		else if(diff_theta_z > 0.7 && Sensor_val.theta_z < 0)
		{
			count_jump++;
		}

		Sensor_val.theta_z_abs = Sensor_val.theta_z + ((float)count_jump*2*PI) - offset_z;
		theta_z_tmp = (Sensor_val.theta_z-offset_z);					// alter Wert speichern
	}


	// Calculate absolute inertial position and speed
	phi_dot.pos_x_dot = RK*(sinf(Sensor_val.theta_z_abs)*phi_dot.phi_dot_x+cosf(Sensor_val.theta_z_abs)*phi_dot.phi_dot_y);
	phi_dot.pos_y_dot = RK*(sinf(Sensor_val.theta_z_abs)*phi_dot.phi_dot_y-cosf(Sensor_val.theta_z_abs)*phi_dot.phi_dot_x);

	phi_dot.pos_x += (old_pos_x_dot+phi_dot.pos_x_dot)/2*Ts;		// Sum(dx) = Sum(vx*Ts)
	phi_dot.pos_y += (old_pos_y_dot+phi_dot.pos_y_dot)/2*Ts;		// Sum(dy) = Sum(vy*Ts)

	old_pos_x_dot = phi_dot.phi_dot_x;
	old_pos_y_dot = phi_dot.phi_dot_y;


}


/***************************************************************************/
/* Bezeichnung	:	Gyro_Values_to_Theta_dot()                             */
/* Funktion		:	Berechnet die theta_dot Werte aus den Gyrowerte        */
/* Übergabewert	:	---													   */
/* Rückgabewert	:	---													   */
/***************************************************************************/

void Gyro_Values_to_Theta_dot(void)
{
     Sensor_val.theta_dot_x = Sensor_val.gyro_x + Sensor_val.gyro_y*tanf(Sensor_val.theta_y)*sinf(Sensor_val.theta_x) + Sensor_val.gyro_z*tanf(Sensor_val.theta_y)*cosf(Sensor_val.theta_x);
     Sensor_val.theta_dot_y = Sensor_val.gyro_y*cosf(Sensor_val.theta_x) - Sensor_val.gyro_z*sinf(Sensor_val.theta_x);
     Sensor_val.theta_dot_z = Sensor_val.gyro_y*sinf(Sensor_val.theta_x)/cosf(Sensor_val.theta_y) + Sensor_val.gyro_z*cosf(Sensor_val.theta_x)/cosf(Sensor_val.theta_y);
}
