cmake_minimum_required(VERSION 2.8.3 FATAL_ERROR)
cmake_policy(SET CMP0042 NEW)
cmake_policy(SET CMP0048 NEW)
cmake_policy(SET CMP0054 NEW)

if (NOT CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr" CACHE STRING "install prefix" FORCE)
endif()

project(cf2_sitl VERSION 1.0.0)
include(GNUInstallDirs)

#######################
## Find Dependencies ##
#######################

# Add search directory for CMake on OS X
list(APPEND CMAKE_MODULE_PATH /usr/local/share/cmake/Modules)
# list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

option(BUILD_VERIF "Build the verification tool" OFF)

## System dependencies are found with CMake's conventions
find_package(PkgConfig REQUIRED)

# find thread package
find_package (Threads REQUIRED)

set (CF2_SITL_SRCS_DIR
  ${CMAKE_CURRENT_SOURCE_DIR}/../src/
)

set(FREERTOS_SRC_DIR 
  ${CF2_SITL_SRCS_DIR}/lib/FreeRTOS
)

# Code variables
set(ESTIMATOR
  kalman
)
set(CONTROLLER
  Any #PID, Mellinger
)

if(BUILD_VERIF)

  set(N_STATE 11)
  set(N_NOISE 14)
  set(T_ORDER 3) 

  set(EXTRA_ARGS_VER
    "-DTIME_STEP_VAR -DUSE_MCU -DUSE_MAF2 -DN_STATE=${N_STATE} -DN_NOISE=${N_NOISE} -DT_ORDER=${T_ORDER}"
  )

  set(INCL_VERIF
    ${CF2_SITL_SRCS_DIR}/outer_ode/interface
  )

  set(SRC_VERIF
    ${CF2_SITL_SRCS_DIR}/outer_ode/src/interval.cpp
    ${CF2_SITL_SRCS_DIR}/outer_ode/src/aa_mod.cpp
    ${CF2_SITL_SRCS_DIR}/outer_ode/src/ode_integr.cpp
    ${CF2_SITL_SRCS_DIR}/outer_ode/src/outer_task.cpp
  )
endif(BUILD_VERIF)

#Generate version.c dependencies
add_custom_command(OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/../version.c
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../
  COMMAND python ${CMAKE_CURRENT_SOURCE_DIR}/../tools/make/versionTemplate.py ${CMAKE_CURRENT_SOURCE_DIR}/../src/utils/src/version.vtpl ${CMAKE_CURRENT_SOURCE_DIR}/../version.c
  )
add_custom_target(version_cf ALL DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/../version.c)

if(BUILD_VERIF)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 ${EXTRA_ARGS_VER} -Os -g3 -Wall -Wno-maybe-uninitialized -Werror -Werror -Wno-error=maybe-uninitialized -Wmissing-braces -fno-strict-aliasing -ffunction-sections -fdata-sections -Wdouble-promotion -Wno-deprecated-declarations")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DENABLE_VERIF -DESTIMATOR_NAME=${ESTIMATOR}Estimator -DCONTROLLER_NAME=ControllerType${CONTROLLER} -DSENSOR_INCLUDED_SIM -DSENSORS_FORCE=SensorImplementation_sim")
endif(BUILD_VERIF)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=gnu11 -Os -g3 -Wall -Wno-maybe-uninitialized -Wno-error=maybe-uninitialized -Wmissing-braces -fno-strict-aliasing -ffunction-sections -fdata-sections -Wdouble-promotion -Wno-deprecated-declarations")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DESTIMATOR_NAME=${ESTIMATOR}Estimator -DCONTROLLER_NAME=ControllerType${CONTROLLER} -DSENSOR_INCLUDED_SIM -DSENSORS_FORCE=SensorImplementation_sim")

#set (CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} --specs=nano.specs -T ${CF2_SITL_SRCS_DIR}/../tools/make/F405/linker/FLASH.ld")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wno-deprecated-declarations")
set (CMAKE_BUILD_TYPE Release)

set(${PROJECT_NAME}_INCLUDE_DIRS
    ${PROJECT_SOURCE_DIR}/include
    CACHE INTERNAL "${PROJECT_NAME}: Include Directories" FORCE)

set(FREERTOS_SRCS
   ${FREERTOS_SRC_DIR}/event_groups.c
   ${FREERTOS_SRC_DIR}/list.c
   ${FREERTOS_SRC_DIR}/queue.c
   ${FREERTOS_SRC_DIR}/tasks.c
   ${FREERTOS_SRC_DIR}/timers.c
   ${FREERTOS_SRC_DIR}/croutine.c
   ${FREERTOS_SRC_DIR}/portable/GCC/Linux/port.c
   ${FREERTOS_SRC_DIR}/portable/MemMang/heap_3.c
)

set (CF2_SITL_INCLUDE_DIRS
  ${FREERTOS_SRC_DIR}/include
  ${FREERTOS_SRC_DIR}/portable/GCC/Linux
  ${CF2_SITL_SRCS_DIR}/config
  ${CF2_SITL_SRCS_DIR}/hal/interface
  ${CF2_SITL_SRCS_DIR}/modules/interface
  ${CF2_SITL_SRCS_DIR}/platform
  ${CF2_SITL_SRCS_DIR}/utils/interface
  #Verification
  ${INCL_VERIF}
  #${CMAKE_CURRENT_SOURCE_DIR}/include
)

include_directories(${CF2_SITL_INCLUDE_DIRS})

set(SRCS
  # main
  ${CF2_SITL_SRCS_DIR}/init/main_sitl.c

  #Hal
  ${CF2_SITL_SRCS_DIR}/hal/src/usec_time_sim.c
  ${CF2_SITL_SRCS_DIR}/hal/src/buzzer.c

  #Communication
  ${CF2_SITL_SRCS_DIR}/hal/src/socketlink.c
  ${CF2_SITL_SRCS_DIR}/modules/src/crtp.c

  #sensors
  ${CF2_SITL_SRCS_DIR}/hal/src/sensors_sim.c
  ${CF2_SITL_SRCS_DIR}/hal/src/sensors.c

  #Modules
  ${CF2_SITL_SRCS_DIR}/modules/src/system_sitl.c
  ${CF2_SITL_SRCS_DIR}/modules/src/comm_sitl.c
  ${CF2_SITL_SRCS_DIR}/modules/src/console.c
  ${CF2_SITL_SRCS_DIR}/modules/src/pid.c
  ${CF2_SITL_SRCS_DIR}/modules/src/crtpservice.c
  ${CF2_SITL_SRCS_DIR}/modules/src/param.c
  ${CF2_SITL_SRCS_DIR}/modules/src/log.c
  ${CF2_SITL_SRCS_DIR}/modules/src/worker.c
  ${CF2_SITL_SRCS_DIR}/modules/src/trigger.c
  ${CF2_SITL_SRCS_DIR}/modules/src/sitaw.c
  ${CF2_SITL_SRCS_DIR}/modules/src/queuemonitor.c
  ${CF2_SITL_SRCS_DIR}/modules/src/msp.c
  ${CF2_SITL_SRCS_DIR}/modules/src/platformservice.c
  ${CF2_SITL_SRCS_DIR}/modules/src/sysload.c
  ${CF2_SITL_SRCS_DIR}/modules/src/mem_sitl.c

  #stabilizer modules
  ${CF2_SITL_SRCS_DIR}/modules/src/commander.c
  ${CF2_SITL_SRCS_DIR}/modules/src/controller.c
  ${CF2_SITL_SRCS_DIR}/modules/src/crtp_commander.c
  ${CF2_SITL_SRCS_DIR}/modules/src/crtp_commander_rpyt.c
  ${CF2_SITL_SRCS_DIR}/modules/src/crtp_commander_generic.c
  ${CF2_SITL_SRCS_DIR}/modules/src/crtp_localization_service.c

  ${CF2_SITL_SRCS_DIR}/modules/src/attitude_pid_controller.c
  ${CF2_SITL_SRCS_DIR}/modules/src/sensfusion6.c
  ${CF2_SITL_SRCS_DIR}/modules/src/stabilizer.c
  ${CF2_SITL_SRCS_DIR}/modules/src/position_estimator_altitude.c
  ${CF2_SITL_SRCS_DIR}/modules/src/position_controller_pid.c

  ${CF2_SITL_SRCS_DIR}/modules/src/estimator.c
  ${CF2_SITL_SRCS_DIR}/modules/src/estimator_complementary.c
  ${CF2_SITL_SRCS_DIR}/modules/src/estimator_kalman.c
  ${CF2_SITL_SRCS_DIR}/modules/src/outlierFilter.c

  #Controller and power distribution
  ${CF2_SITL_SRCS_DIR}/modules/src/controller.c
  ${CF2_SITL_SRCS_DIR}/modules/src/controller_pid.c
  ${CF2_SITL_SRCS_DIR}/modules/src/controller_mellinger.c
  ${CF2_SITL_SRCS_DIR}/modules/src/power_distribution_sim.c

  #High level commander
  ${CF2_SITL_SRCS_DIR}/modules/src/crtp_commander_high_level.c
  ${CF2_SITL_SRCS_DIR}/modules/src/planner.c
  ${CF2_SITL_SRCS_DIR}/modules/src/pptraj.c

  #Utilities
  ${CF2_SITL_SRCS_DIR}/utils/src/filter.c
  ${CF2_SITL_SRCS_DIR}/utils/src/eprintf.c
  ${CF2_SITL_SRCS_DIR}/utils/src/crc.c
  ${CF2_SITL_SRCS_DIR}/utils/src/num.c
  ${CF2_SITL_SRCS_DIR}/utils/src/debug.c
  ${CF2_SITL_SRCS_DIR}/utils/src/configblocksitl.c

  ${CF2_SITL_SRCS_DIR}/../version.c
  ${FREERTOS_SRCS}

  #Verification include
  ${SRC_VERIF}
)

if(BUILD_VERIF)
  SET_SOURCE_FILES_PROPERTIES( ${CF2_SITL_SRCS_DIR}/init/main_sitl.c PROPERTIES LANGUAGE CXX )
endif(BUILD_VERIF)

# Create an executable
add_executable(cf2 ${SRCS})

target_link_libraries(cf2 ${CMAKE_THREAD_LIBS_INIT} m)

if(BUILD_VERIF)
  target_compile_options(cf2 PUBLIC -DSITL_CF2 -DENABLE_VERIF -DARCH_64)
else(BUILD_VERIF)
  target_compile_options(cf2 PUBLIC -DSITL_CF2 -DARCH_64)
endif(BUILD_VERIF)

set_target_properties(
  cf2 PROPERTIES LINK_FLAGS "-Wl,--gc-sections,-T${CMAKE_CURRENT_SOURCE_DIR}/log_param_linker.ld"
)
#set_target_properties(
#  cf2 PROPERTIES LINK_FLAGS "-Wl,-T${CF2_SITL_SRCS_DIR}/../tools/make/F405/linker/COMMON.ld,-T${CF2_SITL_SRCS_DIR}/../tools/make/F405/linker/DEF.ld,-T${CMAKE_CURRENT_SOURCE_DIR}/sections_FLASH.ld")
#set_target_properties(cf2 PROPERTIES LINK_DEPENDS ${CF2_SITL_SRCS_DIR}/../tools/make/F405/linker/FLASH.ld)
