#include "outer_task.h"

static int idPsp, idQsp, idRsp, idZsp;
// static int idRoll, idPitch, idYaw;
static int idRoll, idPitch;
static int idPcurr, idQcurr, idRcurr;
static int idPint, idQint, idRint;
// static int idUcurr, idVcurr, idWcurr;
static int idVZcurr;
static int idZcurr, idZint;

static AF1 x0[N_STATE];
static OdeFunc my_dyn;

static bool isInit = false;

//#$ TODO uncomment if using in sitl. For hitl, one should send the data via radio
//static FILE* zlogfile[N_STATE];

//static const char* Z_LOG_FILE[N_STATE] = {"roll.txt","pitch.txt","p.txt","q.txt","r.txt","int_p.txt","int_q.txt","int_r.txt","vz.txt","z.txt","int_vz.txt"};

static float currentTime = 0;

static void start_verif();


static void verifTask(void* params)
{
	// Leave some time to the other task to wake up
	vTaskDelay(M2T(1000));

	// Initialize components
	verifTaskInit();

	while(!sensorsAreCalibrated()) {
    	vTaskDelay(M2T(500));
  	}

  	vTaskDelay(M2T(10000));

	// Start the verification main routine
	while(1)
	{
		// INitialize state
		// x0[0] = logGetFloat(idRoll) + RPY_INCERTAINCY; // roll value
		// x0[1] = -logGetFloat(idPitch) + RPY_INCERTAINCY; // pitch value -> convention crazyflie
		// x0[2] = logGetFloat(idYaw) + RPY_INCERTAINCY; // yaw value

		// x0[3] = logGetFloat(idPcurr)  + PQR_INCERTAINCY; // roll rate value
		// x0[4] = -logGetFloat(idQcurr) + PQR_INCERTAINCY; // pitch rate value
		// x0[5] = logGetFloat(idRcurr); // yaw rate value

		// x0[6] = logGetFloat(idPint); // roll integrale value
		// x0[7] = -logGetFloat(idQint); // pitch integrale value
		// x0[8] = logGetFloat(idRint); // yaw integrale value

		// x0[9] = logGetFloat(idUcurr) + UVW_INCERTAINCY; // u value
		// x0[10] = logGetFloat(idVcurr) + UVW_INCERTAINCY; // v value
		// x0[11] = logGetFloat(idWcurr) + UVW_INCERTAINCY; // w value

		// x0[12] = logGetFloat(idZcurr) + Z_INCERTAINCY; // Z value
		// x0[13] = logGetFloat(idZint); // Z integrale value

		x0[0] = logGetFloat(idRoll) * (M_PI_VER/180) + RPY_INCERTAINCY; // roll value
		x0[1] = -logGetFloat(idPitch) * (M_PI_VER/180) + RPY_INCERTAINCY; // pitch value -> convention crazyflie

		x0[2] = logGetFloat(idPcurr) * (M_PI_VER/180)  + PQR_INCERTAINCY; // roll rate value
		x0[3] = -logGetFloat(idQcurr)* (M_PI_VER/180) + PQR_INCERTAINCY; // pitch rate value
		x0[4] = logGetFloat(idRcurr) * (M_PI_VER/180); // yaw rate value

		x0[5] = logGetFloat(idPint) * (M_PI_VER/180) / Ki_rr; // roll integrale value
		x0[6] = -logGetFloat(idQint)* (M_PI_VER/180) / Ki_pr; // pitch integrale value
		x0[7] = logGetFloat(idRint) * (M_PI_VER/180) / Ki_yr; // yaw integrale value

		x0[8] = logGetFloat(idVZcurr) + UVW_INCERTAINCY; // u value

		x0[9] = logGetFloat(idZcurr) + Z_INCERTAINCY; // Z value
		x0[10] = logGetFloat(idZint) / Ki_VZ; // Z integrale value

		// INitialize setpoint
		my_dyn.params[0] = logGetFloat(idPsp) * (M_PI_VER/180);
		my_dyn.params[1] = -logGetFloat(idQsp) * (M_PI_VER/180);
		my_dyn.params[2] = logGetFloat(idRsp) * (M_PI_VER/180);
		my_dyn.params[3] = logGetFloat(idZsp);

		currentTime = xTaskGetTickCount()/1000.0f;
		// commented block below is related to the commented lines marked with #$ TODO
		//for(uint8_t i= 0 ; i<N_STATE ; i++){
		//	Interval curr_Z = x0[i].getInterval();
		//	fprintf(zlogfile[i], "%f\t%f\t%f\n", (double) currentTime, (double) curr_Z.getMin(), (double) curr_Z.getMax());
		//}

		// DEBUG_PRINT("Psp = %f", (double) my_dyn.params[0]);
		// DEBUG_PRINT("Qsp = %f", (double) my_dyn.params[1]);
		// DEBUG_PRINT("Rsp = %f", (double) my_dyn.params[2]);
		// DEBUG_PRINT("Zsp = %f", (double) my_dyn.params[3]);

		// for (uint8_t i=0 ; i<N_STATE ; i++){
		// 	Interval temp = x0[i].getInterval();
		// 	DEBUG_PRINT("x[%d] : [%f , %f ] \n", (int) i , (double) temp.getMin(), (double) temp.getMax());
		// }

		// Now calculate the outer approximation
		start_verif();
		vTaskDelay(M2T(200));
		// Apply a correction in case of problem founded

	}
}

void verifTaskInit()
{
	if(isInit)
		return;
	idPsp = logGetVarId("controller","rollRate");
	idQsp = logGetVarId("controller","pitchRate");
	idRsp = logGetVarId("controller","yawRate");
	idZsp = logGetVarId("posCtl","targetZ");

	idRoll = logGetVarId("stabilizer","roll");
	idPitch = logGetVarId("stabilizer","pitch");
	// idYaw = logGetVarId("stabilizer","yaw");

	idPcurr = logGetVarId("gyro","x");
	idQcurr = logGetVarId("gyro","y");
	idRcurr = logGetVarId("gyro","z");

	//idUcurr = logGetVarId("kalman","statePX");
	//idVcurr = logGetVarId("kalman","statePY");
	//idWcurr = logGetVarId("kalman","statePZ");
	idVZcurr = logGetVarId("stateEstimateV","Vz");

	idZcurr = logGetVarId("stateEstimate","z");

	idPint = logGetVarId("pid_rate","roll_outI");
	idQint = logGetVarId("pid_rate","pitch_outI");
	idRint = logGetVarId("pid_rate","yaw_outI");
	idZint = logGetVarId("posCtl","VZi");

	DEBUG_PRINT("Got Variable ID succeed \n");

	// commented block below is related to the commented lines marked with #$ TODO
	//for(uint8_t i= 0 ; i<N_STATE ; i++){
	//	zlogfile[i] = fopen(Z_LOG_FILE[i] , "w+");
	//}

	isInit = true;
}

void verifTaskLaunch(void)
{
	xTaskCreate(verifTask, VERIF_TASK_NAME,
              VERIF_TASK_STACKSIZE, NULL, VERIF_TASK_PRI, NULL);
}

bool verifTaskTest(void)
{
	return true;
}

static void start_verif()
{
	TM_val tm_val(x0, 0, &my_dyn);
	Interval curr_Z;
	while (tm_val.tn <= INTEG_DURATION){
		tm_val.buildAndEval();
		// commented block below is related to the commented lines marked with #$ TODO
		//for(uint8_t i= 0 ; i<N_STATE ; i++){
		//	Interval curr_Z = x0[i].getInterval();
		//	fprintf(zlogfile[i], "%f\t%f\t%f\n", (double) currentTime, (double) curr_Z.getMin(), (double) curr_Z.getMax());
		//}

		//curr_Z = x0[9].getInterval();
		//fprintf(zlogfile, "%f\t%f\t%f\n", (double) (currentTime+tm_val.tn) , (double) curr_Z.getMin(), (double) curr_Z.getMax());
/*		DEBUG_PRINT(" t = %f" , (double) tm_val.tn);
		for (uint8_t i=0 ; i<N_STATE ; i++){
			Interval temp = x0[i].getInterval();
			DEBUG_PRINT("x[%d] : [%f , %f ] \n", (int) i , (double) temp.getMin(), (double) temp.getMax());
		}
*/
	}
		// Interval temp = x0[9].getInterval();
		// DEBUG_PRINT("x[%d] : [%f , %f ] \n", (int) 9 , (double) temp.getMin(), (double) temp.getMax());
}