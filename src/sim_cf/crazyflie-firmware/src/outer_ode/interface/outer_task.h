#ifndef OUTER_TASK_H
#define OUTER_TASK_H

/* FreeRtos includes */
extern "C"
{
	#include "FreeRTOS.h"
	#include "task.h"

	#include "log.h"

	#include "console.h"
	#include "cfassert.h"
	#include "debug.h"

	#include "sensors.h"
}

#include "ode_integr.h"

//#define Z_LOG_FILE "zlog.txt"

#define INTEG_DURATION  0.4f
#define RPY_INCERTAINCY 0.0f // Interval(-1,1)
#define PQR_INCERTAINCY	Interval(-0.02 , 0.02)
#define UVW_INCERTAINCY	0.0f // Interval(-0.05 , 0.05)
#define Z_INCERTAINCY	Interval(-0.1 , 0.1)

void verifTaskInit();

bool verifTaskTest();

void verifTaskLaunch();

#endif //OUTER_TASK_H