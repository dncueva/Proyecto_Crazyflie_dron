/*
 *    ||          ____  _ __                           
 * +------+      / __ )(_) /_______________ _____  ___ 
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie simulation firmware
 *
 * Copyright (c) 2018  Eric Goubault, Sylvie Putot, Franck Djeumou
 *             Cosynux , LIX , France
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * main_sitl.c - Containing the main function.for the SITL simulation based on main.c
 */

#ifdef ENABLE_VERIF

extern "C"
{
  /* Personal configs */
  #include "FreeRTOSConfig.h"

  /* FreeRtos includes */
  #include "FreeRTOS.h"
  #include "task.h"

  #include <stdio.h>
  #include <stdlib.h>
  #include <stdbool.h>

  /* Project includes */
  #include "socketlink.h"
  #include "config.h"
  #include "system.h"
  #include "usec_time.h"
}

#include "outer_task.h"

#else

  /* Personal configs */
  #include "FreeRTOSConfig.h"

  /* FreeRtos includes */
  #include "FreeRTOS.h"
  #include "task.h"

  #include <stdio.h>
  #include <stdlib.h>
  #include <stdbool.h>

  /* Project includes */
  #include "socketlink.h"
  #include "config.h"
  #include "system.h"
  #include "usec_time.h"
#endif

#define CRTP_PORT 19950
char CRTP_SERVER_ADDRESS[] = "INADDR_ANY";

int main(int argc, char **argv) 
{

  if (argc == 4){
    cf_id = atoi(argv[1]);
    crtp_port = atoi(argv[2]);
    address_host = argv[3];
    printf("CF id : %d , address : %s , port : %d \n", cf_id , address_host , crtp_port );
  } else if (argc == 2) {
    // Initiaze socket parameters
    cf_id = atoi(argv[1]);
    crtp_port =  CRTP_PORT;
    address_host = CRTP_SERVER_ADDRESS;
    printf("CF id : %d , address : %s , port : %d \n", cf_id , address_host , crtp_port );
  }else {
    cf_id = 1;
    crtp_port =  CRTP_PORT;
    address_host = CRTP_SERVER_ADDRESS;
    printf("No port and ADDRESS selected ! 1-19950-INADDR_ANY selected as default\n");
  }

  // If the verification is enabled, launch the verif tool  
#ifdef ENABLE_VERIF
  verifTaskLaunch();
#endif

  //Launch the system task that will initialize and start everything
  systemLaunch();

  //Start the FreeRTOS scheduler
  vTaskStartScheduler();

  //Should never reach this point!
  while(1);

  return 0;
}

/*
* FreeRTOS assert name
*/
void vAssertCalled( unsigned long ulLine, const char * const pcFileName )
{
    printf("ASSERT: %s : %d\n", pcFileName, (int)ulLine);
    while(1);
}

unsigned long ulGetRunTimeCounterValue(void)
{
    return 0;
}

void vConfigureTimerForRunTimeStats(void)
{
    return;
}

/* For memory management */
void vApplicationMallocFailedHook(void)
{
    while(1);
}