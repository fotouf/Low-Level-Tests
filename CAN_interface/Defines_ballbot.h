/*=========================================================================*/
/* 																		   */
/*	Autor 				: Samuel Schüller, Swen Wigert					   */
/*	File				: Defines_Ballbot								   */
/*	Erstellt			: 25.03.2010									   */
/* 	Aktuelle Version	: v1.00										 	   */
/*																	       */
/*=========================================================================*/
/*																		   */
/*	Änderungshistory													   */
/*-------------------------------------------------------------------------*/
/*	v1.00	| Grundversion 												   */
/* 	v1.01	|															   */
/*=========================================================================*/

#ifndef DEFINES_BALLBOT_H_
#define DEFINES_BALLBOT_H_

//--------------------------------------------------------------------------------
// Defines
//--------------------------------------------------------------------------------

#define RK  	0.125					//Ball radius
#define RW  	0.06					//Omniwheel radius
#define ALPHA  	3.14159/(float)4		//Motor arrangement angle
#define BETA  	(float)0				//Angle between coordinate system and real motor configuration
#define Ts		1/158.7
#define I_GEAR	(float)26
#define K_M		20.2					//mN/A
#define PI		3.1415926


// Controller Gains
#define K_PHI_FREEZE_11   50.9089
#define K_PHI_FREEZE_12   12.2875
#define K_PHI_FREEZE_13   -4.1361e-014
#define K_PHI_FREEZE_14   -1.775e-014
#define K_PHI_FREEZE_15   -0.6455
#define K_PHI_FREEZE_16   -0.56015
#define K_PHI_FREEZE_17   1.291
#define K_PHI_FREEZE_18   1.7014
#define K_PHI_FREEZE_19   -9.2461e-015
#define K_PHI_FREEZE_110   -4.5005e-015
#define K_PHI_FREEZE_21   -25.4544
#define K_PHI_FREEZE_22   -6.1437
#define K_PHI_FREEZE_23   44.1024
#define K_PHI_FREEZE_24   10.6485
#define K_PHI_FREEZE_25   -0.6455
#define K_PHI_FREEZE_26   -0.56015
#define K_PHI_FREEZE_27   -0.6455
#define K_PHI_FREEZE_28   -0.8507
#define K_PHI_FREEZE_29   1.118
#define K_PHI_FREEZE_210   1.4736
#define K_PHI_FREEZE_31   -25.4544
#define K_PHI_FREEZE_32   -6.1437
#define K_PHI_FREEZE_33   -44.1024
#define K_PHI_FREEZE_34   -10.6485
#define K_PHI_FREEZE_35   -0.6455
#define K_PHI_FREEZE_36   -0.56015
#define K_PHI_FREEZE_37   -0.6455
#define K_PHI_FREEZE_38   -0.8507
#define K_PHI_FREEZE_39   -1.118
#define K_PHI_FREEZE_310   -1.4736

#define K_PHI_11   24.4239
#define K_PHI_12   5.1853
#define K_PHI_13   -8.1195e-014
#define K_PHI_14   -2.1169e-014
#define K_PHI_15   -0.32026
#define K_PHI_16   -0.32186
#define K_PHI_17   0.32026
#define K_PHI_18   0.5015
#define K_PHI_19   -2.4204e-015
#define K_PHI_110   -3.6059e-015
#define K_PHI_21   -12.2119
#define K_PHI_22   -2.5926
#define K_PHI_23   21.1568
#define K_PHI_24   4.4939
#define K_PHI_25   -0.32026
#define K_PHI_26   -0.32186
#define K_PHI_27   -0.16013
#define K_PHI_28   -0.25075
#define K_PHI_29   0.27735
#define K_PHI_210   0.43437
#define K_PHI_31   -12.2119
#define K_PHI_32   -2.5926
#define K_PHI_33   -21.1568
#define K_PHI_34   -4.4939
#define K_PHI_35   -0.32026
#define K_PHI_36   -0.32186
#define K_PHI_37   -0.16013
#define K_PHI_38   -0.25075
#define K_PHI_39   -0.27735
#define K_PHI_310   -0.43437

#define K_PHI_HULL_11   28.2817
#define K_PHI_HULL_12   6.0622
#define K_PHI_HULL_13   -4.1359e-014
#define K_PHI_HULL_14   -9.4128e-015
#define K_PHI_HULL_15   -0.36515
#define K_PHI_HULL_16   -0.33268
#define K_PHI_HULL_17   0.36515
#define K_PHI_HULL_18   0.57116
#define K_PHI_HULL_19   -7.4706e-016
#define K_PHI_HULL_110   -1.6702e-015
#define K_PHI_HULL_21   -14.1409
#define K_PHI_HULL_22   -3.0311
#define K_PHI_HULL_23   24.4878
#define K_PHI_HULL_24   5.2469
#define K_PHI_HULL_25   -0.36515
#define K_PHI_HULL_26   -0.33268
#define K_PHI_HULL_27   -0.18257
#define K_PHI_HULL_28   -0.28558
#define K_PHI_HULL_29   0.31623
#define K_PHI_HULL_210   0.49458
#define K_PHI_HULL_31   -14.1409
#define K_PHI_HULL_32   -3.0311
#define K_PHI_HULL_33   -24.4878
#define K_PHI_HULL_34   -5.2469
#define K_PHI_HULL_35   -0.36515
#define K_PHI_HULL_36   -0.33268
#define K_PHI_HULL_37   -0.18257
#define K_PHI_HULL_38   -0.28558
#define K_PHI_HULL_39   -0.31623
#define K_PHI_HULL_310   -0.49458

#define K_PHI_DOT_11   23.8047
#define K_PHI_DOT_12   5.0207
#define K_PHI_DOT_13   3.5398e-014
#define K_PHI_DOT_14   6.5849e-015
#define K_PHI_DOT_15   -0.42366
#define K_PHI_DOT_16   -0.39868
#define K_PHI_DOT_17   0.39223
#define K_PHI_DOT_18   7.2637e-016
#define K_PHI_DOT_21   -11.9024
#define K_PHI_DOT_22   -2.5104
#define K_PHI_DOT_23   20.6193
#define K_PHI_DOT_24   4.351
#define K_PHI_DOT_25   -0.42366
#define K_PHI_DOT_26   -0.39868
#define K_PHI_DOT_27   -0.19612
#define K_PHI_DOT_28   0.33968
#define K_PHI_DOT_31   -11.9024
#define K_PHI_DOT_32   -2.5104
#define K_PHI_DOT_33   -20.6193
#define K_PHI_DOT_34   -4.351
#define K_PHI_DOT_35   -0.42366
#define K_PHI_DOT_36   -0.39868
#define K_PHI_DOT_37   -0.19612
#define K_PHI_DOT_38   -0.33968

#define PDP_PHI_1	1.
#define PDP_PHI_2	1.
#define PDP_PHI_3	1.
#define PDP_PHI_4	1.
#define PDP_PHI_5	1.
#define PDP_PHI_6	1.
#define PDP_PHI_7	1.
#define PDP_PHI_8	1.
#define PDP_PHI_9	1.
#define PDP_PHI_10	1.

// Planar ===================================================================================
// Controller Gains

#define K_X_POS_1  	-0.3162			// phi
#define K_X_POS_2  	-0.5359			// phi_dot
#define K_X_POS_3  	-22.8608		// theta
#define K_X_POS_4  	-6.5726			// theta_dot

#define K_X_SPEED_2  -0.3162		// phi_dot
#define K_X_SPEED_3  -19.6585		// theta
#define K_X_SPEED_4  -5.5559		// theta_dot

#define K_X_ACC_3  -19.6585			// theta
#define K_X_ACC_4  -5.5559			// theta_dot


// Reference Set Point
#define N_X_POS     -0.3162
#define N_X_SPEED	-0.3162
#define N_X_ACC		-19.6585

// PID Controller Parameter z-axis
#define	KP	-0.15
#define KI	0
#define KD	-0.15
//=============================================================================================


// Security Parameters
#define MAX_THETA			(30/(float)180*3.1415926)	//rad - maximal pich/roll angle
#define MAX_TORQUE			4							//Nm - maximal torque output from controller
#define MAX_VIRTUAL_TORQUE	5							//Nm - maximal torque output from controller
#define MAX_MOTOR_SPEED		(10*2*PI)					//rad/s - maximal motor speed
#define THRESHOLD 			400  						//Umschaltgrenze für Hysterese

// Setpointfilter Parameters
#define PHI_DDOT_MAX			11.            			// Maximale Geschwindigkeitsanderung [rad/s^2]
#define PHI_DOT_MAX				12.           			// Maximale Geschwindigkeit [rad/s]
#define PHI_DPOS_MAX			8.						// Maximale Positionsaenderung
#define PHI_DOT_Z_MAX			PI/2.					// Maximal erlaubte Kugeldrehung um z [rad/s]
#define THETA_Z_SETPOINT_MAX	PI/4.					// Maximal erlaubte Sollwertdifferenz [rad]
#define THETA_Z_DOT_MAX			3.						// Maximale Drehgeschwindigkeit um z [rad/s]
#define THETA_Z_DOT_ABS			15.						// Maximale Drehgeschwindigkeit um z [rad/s]
#define PHI_DOT_MAX_DIFF		12.						// Maximale Differenz zwischen ist und soll Wert speed [rad/s]
#define THETA_Z_MAX_DIFF		5.						// Maximale Differenz zwischen ist und soll Wert z-Rotation [rad/s]

// Kalman Parameters
#define Q_1  	0.3
#define Q_2  	0.2
#define Q_3  	0.1
#define Q_4  	0.1

#define R_1  	0.3
#define R_2  	2
#define R_3  	0.2

#define P_11	1
#define	P_12	0
#define	P_13    0
#define	P_14    0
#define	P_21    0
#define	P_22    1
#define	P_23    0
#define	P_24    0
#define	P_31    0
#define	P_32    0
#define	P_33    1
#define	P_34    0
#define	P_41    0
#define	P_42    0
#define	P_43    0
#define	P_44    1





#endif /* DEFINES_BALLBOT_H_ */
