#ifndef __ODE_INTEGR_H__
#define __ODE_INTEGR_H__

#include "config_outer.h"
#include "ode_dyn.h"

#include "fadbad_af.h"

// class used for computation of Taylor coefficients with FADBAD++
template <uint8_t ORDER>
class Ode
{
public:
	T<AF1,ORDER> x[N_STATE]; // Independent variables
	T<AF1,ORDER> xp[N_STATE]; // Dependent variables

	Ode(){}

	Ode(const OdeFunc &f)
	{
		f(xp , x);
	}

	void reset()
	{
		for(uint8_t i = 0 ; i<N_STATE ; i++)
			xp[i].reset();
	}
};

class TM_val
{

public:
	real tn;
	real curr_tau;
	AF1  *xn; // xn[N_STATE
	Ode<T_ORDER+1> ode_x;
	Ode<T_ORDER+2> ode_g;
	OdeFunc *m_f;

public:
	TM_val(AF1 *x0 , real t0 , OdeFunc *f);
	~TM_val();

	void buildAndEval();

	real fixpoint();

};

#endif // __ODE_INTEGR_H__