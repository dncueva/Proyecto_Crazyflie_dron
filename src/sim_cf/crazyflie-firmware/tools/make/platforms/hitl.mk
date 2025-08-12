# Make configuration for the Crazyflie 2 platform

PLATFORM_HELP_hitl = Crazyflie2 Sim platform, includes Crazyflie 2.0 and Crazyflie 2.1
PLATFORM_NAME_hitl = CF2 Simulation platform

CPU=stm32f4

######### Sensors configuration ##########
CFLAGS += -DSENSOR_INCLUDED_SIM
PROJ_OBJ += sensors_sim.o

######### Stabilizer configuration ##########
ESTIMATOR          ?= kalman
CONTROLLER         ?= Any # one of Any, PID, Mellinger
POWER_DISTRIBUTION ?= sim
