#ifndef __CONFIG_OUTER_H__
#define __CONFIG_OUTER_H__

#ifdef VERBOSE
	#include <iostream>
#endif

#ifdef USE_MAF1
	#include "aa.h"
	typedef MAF1 AF1;
#elif USE_MAF2
	#include "aa_mod.h"
	typedef MAF2 AF1;
#else
	#include "aa_mod2.h"
	typedef DAF AF1;
#endif

#ifndef N_STATE
	#define N_STATE 2
#endif

#ifndef T_ORDER
	#define T_ORDER 2
#endif

#ifndef N_PARAMS
	#define N_PARAMS 4
#endif

// #define RAD_2_DEG	M_PI/180.0f

#define H_MAX 0.015

#define NOISE_TOL 1e-1

#endif // __CONFIG_OUTER_H__