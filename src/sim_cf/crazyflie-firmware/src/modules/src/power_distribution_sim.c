/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie simulation firmware
 *
 * Copyright (C) 2018 Eric Goubault, Sylvie Putot and Franck Djeumou
 *
 * Source file for Sensors simulation using external source of measurement
 * This is based on the work made in power_distribution_stock.c 

 * power_distribution_hitl.c - Crazyflie power distribution code for HITL
 */

/*FreeRtos includes*/
#include "FreeRTOS.h"
#include "task.h"

#include "power_distribution.h"

#include "log.h"
#include "param.h"
#include "num.h"

#include "crtp.h"

#include "console.h"
#include <string.h>

static struct {
  uint32_t m1;
  uint32_t m2;
  uint32_t m3;
  uint32_t m4;
} __attribute__((packed)) motorPower;

static uint32_t lastSentTime = 0;
static CRTPPacket p;

static void motorsSetRatio(const uint8_t *powerVal);

void powerDistributionInit(void)
{
  lastSentTime = xTaskGetTickCount();
  p.size = 4 * sizeof(uint32_t);
  p.header = CRTP_HEADER(CRTP_PORT_SETPOINT_SIM, 0);
  // Should already be initialized by the sensor task
  // crtpInitTaskQueue(CRTP_PORT_SETPOINT_SIM);
}

bool powerDistributionTest(void)
{
  return true;
}

#define limitThrust(VAL) limitUint16(VAL)

void powerStop()
{
  motorPower.m1 = 0;
  motorPower.m2 = 0;
  motorPower.m3 = 0;
  motorPower.m4 = 0;
  consolePrintf("STOP \n");
  motorsSetRatio((uint8_t *) &motorPower );
}

void powerDistribution(const control_t *control)
{
  #ifdef QUAD_FORMATION_X
  int16_t r = control->roll / 2.0f;
  int16_t p = control->pitch / 2.0f;
  motorPower.m1 = limitThrust(control->thrust - r + p + control->yaw);
  motorPower.m2 = limitThrust(control->thrust - r - p - control->yaw);
  motorPower.m3 =  limitThrust(control->thrust + r - p + control->yaw);
  motorPower.m4 =  limitThrust(control->thrust + r + p - control->yaw);
  #else // QUAD_FORMATION_NORMAL
  motorPower.m1 = limitThrust(control->thrust + control->pitch +
   control->yaw);
  motorPower.m2 = limitThrust(control->thrust - control->roll -
   control->yaw);
  motorPower.m3 =  limitThrust(control->thrust - control->pitch +
   control->yaw);
  motorPower.m4 =  limitThrust(control->thrust + control->roll -
   control->yaw);
  #endif
  
  if (xTaskGetTickCount() - lastSentTime >= M2T(2)){
    motorsSetRatio((uint8_t *) &motorPower);
    lastSentTime = xTaskGetTickCount();
  }
  // motorsSetRatio((uint8_t *) &motorPower);
}

static void motorsSetRatio(const uint8_t *powerVal){
  // uint8_t incr;
  // for (incr = 0 ; incr < p.size ; incr++){
  //   p.data[incr] = powerVal[incr];
  // }
  memcpy(p.data , powerVal , p.size);
  crtpSendPacket(&p);
  // consolePrintf("%d , %d , %d , %d  \n", (int) motorPower.m1 , (int) motorPower.m2 , (int) motorPower.m3 , (int) motorPower.m4 );
}

LOG_GROUP_START(motor)
LOG_ADD(LOG_INT32, m4, &motorPower.m4)
LOG_ADD(LOG_INT32, m1, &motorPower.m1)
LOG_ADD(LOG_INT32, m2, &motorPower.m2)
LOG_ADD(LOG_INT32, m3, &motorPower.m3)
LOG_GROUP_STOP(motor)
