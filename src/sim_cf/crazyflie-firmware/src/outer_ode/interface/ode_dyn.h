#ifndef ODE_DYN_H
#define ODE_DYN_H

// Include Files
#include "config_outer.h"

#define G_QUAD 9.8f
#define M_QUAD 0.028f

#define Kp_Z   2.0f
#define Kp_VZ  25.0f
#define Ki_VZ  15.0f

#define Kp_rr  250.0f
#define Kp_pr  250.0f
#define Kp_yr  120.0f

#define Ki_rr  500.0f
#define Ki_pr  500.0f
#define Ki_yr  16.7f

#define Ip_qr	-0.760696995059653f
#define Iq_pr	0.761902331719982f
#define Ir_pq	-0.002866960484664f
#define Im_xx	6.034380278196999e4f
#define Im_yy	6.003985926176670e4f
#define Im_zz	3.417442050093412e4f

// #define C1 		0.04076521
// #define C2 		380.8359
// #define Ct 		1.285e-8
// #define Cd 		7.645e-11
// #define d  		0.046/sqrt(2.0)

// define here  your ODE system yp = \dot y = f(y)
class OdeFunc {
public:
	real params[N_PARAMS];

public:
	template <class C>
	void operator()(C yp[N_STATE], C y[N_STATE]) const {
		/* yp[0] = 1 - (params[1]+1)*y[0] + params[0]*y[0]*y[0]*y[1];
		yp[1] = params[1]*y[0] - params[0]*y[0]*y[0]*y[1];*/
		C cosRoll = cos(y[0]/**M_PI_VER/180*/);
		C sinRoll = sin(y[0]/**M_PI_VER/180*/);

		C cosPitch = cos(y[1]/**M_PI_VER/180*/);
		C sinPitch = sin(y[1]/**M_PI_VER/180*/);

		C velZ_sp = Kp_Z * (params[3] - y[9]);

		// Altitude Z
		yp[9] = y[8];
		// Integrale term in Vz
		yp[10] = velZ_sp - y[8];

		C thrust = 1000*(Kp_VZ * yp[10] + Ki_VZ * y[10]) + 36000;

		// integrale of error in p , q and r
		yp[5] = (params[0] - y[2]);//err_p;
		yp[6] = (params[1] - y[3]);//err_q;
		yp[7] = (params[2] - y[4]);//err_r;

		C cmd_r = y[5]*Ki_rr + yp[5]*Kp_rr;
		C cmd_p = y[6]*Ki_pr + yp[6]*Kp_pr;
		C cmd_y = y[7]*Ki_yr + yp[7]*Kp_yr;
		//std:cout << getAAF(cmd_p).convert_int() << std::endl;

		C temp1 = 2.7783e-12f*thrust + 2.5956e-08f;

		C Mx = temp1*cmd_r - 2.7783e-12f*cmd_p*cmd_y;
		C My = -2.7783e-12f * cmd_r*cmd_y + temp1*cmd_p;
		C Mz = -2.5409e-13f*cmd_r*cmd_p + 61.4875f*temp1*cmd_y;
		C F  = 2.1354e-11f *(cmd_p*cmd_p + cmd_r*cmd_r + 4*cmd_y*cmd_y + 4*thrust*thrust) + 1.5960e-06f*thrust + 0.0075f;

		// Roll , pitch , yaw derivatives
		yp[0] = y[2] + (y[4]*cosRoll + y[3]*sinRoll)*tan(y[1]/**M_PI_VER/180.0*/);
		yp[1] = y[3]*cosRoll - y[4]*sinRoll;

		// p , q and r derivatives
		yp[2] = Ip_qr * y[3] * y[4] + Im_xx * Mx ;
		yp[3] = Iq_pr * y[2] * y[4] + Im_yy * My ;
		yp[4] = Ir_pq * y[2] * y[3] + Im_zz * Mz ;

		// Speed Vz in world frame
		yp[8] = -G_QUAD + (F/M_QUAD) * (cosPitch * cosRoll);
	}
};

#endif // ODE_DYN_H
