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
 * mem_sitl.c: Memory module. simplified version of mem_cf2 for simulation.
 * It is useless but an implementation free from driver dep was needed to be
 * able to handle trajectory upload
 */

#include <string.h>
#include <errno.h>
#include <stdint.h>
#include <stdbool.h>

/* FreeRtos includes */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"

#include "config.h"
#include "crtp.h"
#include "mem.h"

#include "crtp_commander_high_level.h"

#include "console.h"
#include "assert.h"
#include "debug.h"

#if 0
#define MEM_DEBUG(fmt, ...) DEBUG_PRINT("D/log " fmt, ## __VA_ARGS__)
#define MEM_ERROR(fmt, ...) DEBUG_PRINT("E/log " fmt, ## __VA_ARGS__)
#else
#define MEM_DEBUG(...)
#define MEM_ERROR(...)
#endif


// Maximum log payload length
#define MEM_MAX_LEN 30

// The first part of the memory ids are static followed by a dynamic part
// of one wire ids that depends on the decks that are attached
#define EEPROM_ID       0x00
#define LEDMEM_ID       0x01
#define LOCO_ID         0x02
#define TRAJ_ID         0x03
#define OW_FIRST_ID     0x04

#define STATUS_OK 0

#define MEM_TYPE_EEPROM 0x00
#define MEM_TYPE_OW     0x01
#define MEM_TYPE_LED12  0x10
#define MEM_TYPE_LOCO   0x11
#define MEM_TYPE_TRAJ   0x12

#define MEM_LOCO_INFO             0x0000
#define MEM_LOCO_ANCHOR_BASE      0x1000
#define MEM_LOCO_ANCHOR_PAGE_SIZE 0x0100
#define MEM_LOCO_PAGE_LEN         (3 * sizeof(float) + 1)

//Private functions
static void memTask(void * prm);
static void memSettingsProcess(int command);
static void memWriteProcess(void);
static void memReadProcess(void);
static void createNbrResponse(CRTPPacket* p);
static void createInfoResponse(CRTPPacket* p, uint8_t memId);
static void createInfoResponseBody(CRTPPacket* p, uint8_t type, uint32_t memSize, const uint8_t data[8]);

static bool isInit = false;

static uint8_t nbrOwMems = 0;

static const uint8_t noData[8] = {0, 0, 0, 0, 0, 0, 0, 0};
static CRTPPacket p;

void memInit(void)
{
  if(isInit)
    return;

  isInit = true;
  
  //Start the mem task
  xTaskCreate(memTask, MEM_TASK_NAME,
              MEM_TASK_STACKSIZE, NULL, MEM_TASK_PRI, NULL);
}

bool memTest(void)
{
  return isInit;
}

void memTask(void * param)
{
	crtpInitTaskQueue(CRTP_PORT_MEM);
	
	while(1)
	{
		crtpReceivePacketBlock(CRTP_PORT_MEM, &p);

		switch (p.channel)
		{
      case MEM_SETTINGS_CH:
        memSettingsProcess(p.data[0]);
        break;
      case MEM_READ_CH:
        memReadProcess();
        break;
      case MEM_WRITE_CH:
        memWriteProcess();
        break;
      default:
        break;
		}
	}
}

void memSettingsProcess(int command)
{
  switch (command)
  {
    case MEM_CMD_GET_NBR:
      createNbrResponse(&p);
      crtpSendPacket(&p);
      break;

    case MEM_CMD_GET_INFO:
      {
        uint8_t memId = p.data[1];
        createInfoResponse(&p, memId);
        crtpSendPacket(&p);
      }
      break;
  }
}

void createNbrResponse(CRTPPacket* p)
{
  p->header = CRTP_HEADER(CRTP_PORT_MEM, MEM_SETTINGS_CH);
  p->size = 2;
  p->data[0] = MEM_CMD_GET_NBR;
  p->data[1] = nbrOwMems + OW_FIRST_ID;
}

void createInfoResponse(CRTPPacket* p, uint8_t memId)
{
  p->header = CRTP_HEADER(CRTP_PORT_MEM, MEM_SETTINGS_CH);
  p->size = 2;
  p->data[0] = MEM_CMD_GET_INFO;
  p->data[1] = memId;

  // No error code if we fail, just send an empty packet back
  switch(memId)
  {
    case TRAJ_ID:
      createInfoResponseBody(p, MEM_TYPE_TRAJ, sizeof(trajectories_memory), noData);
      break;
    default:
      break;
  }
}

void createInfoResponseBody(CRTPPacket* p, uint8_t type, uint32_t memSize, const uint8_t data[8])
{
  p->data[2] = type;
  p->size += 1;

  memcpy(&p->data[3], &memSize, 4);
  p->size += 4;

  memcpy(&p->data[7], data, 8);
  p->size += 8;
}


void memReadProcess()
{
  uint8_t memId = p.data[0];
  uint8_t readLen = p.data[5];
  uint32_t memAddr;
  uint8_t status = STATUS_OK;

  memcpy(&memAddr, &p.data[1], 4);

  MEM_DEBUG("Packet is MEM READ\n");
  p.header = CRTP_HEADER(CRTP_PORT_MEM, MEM_READ_CH);
  // Dont' touch the first 5 bytes, they will be the same.

  switch(memId)
  {
    case EEPROM_ID:
      status = EIO;
      break;

    case LEDMEM_ID:
      status = EIO;
      break;

    case LOCO_ID:
      status = EIO;
      break;

    case TRAJ_ID:
      {
        if (memAddr + readLen <= sizeof(trajectories_memory) &&
            memcpy(&p.data[6], &(trajectories_memory[memAddr]), readLen)) {
          status = STATUS_OK;
        } else {
          status = EIO;
        }
      }
      break;

    default:
      status = EIO;
      break;
  }

#if 0
  {
    int i;
    for (i = 0; i < readLen; i++)
      consolePrintf("%X ", p.data[i+6]);

    consolePrintf("\nStatus %i\n", status);
  }
#endif

  p.data[5] = status;
  if (status == STATUS_OK)
    p.size = 6 + readLen;
  else
    p.size = 6;


  crtpSendPacket(&p);
}

void memWriteProcess()
{
  uint8_t memId = p.data[0];
  uint8_t writeLen;
  uint32_t memAddr;
  uint8_t status = STATUS_OK;

  memcpy(&memAddr, &p.data[1], 4);
  writeLen = p.size - 5;

  MEM_DEBUG("Packet is MEM WRITE\n");
  p.header = CRTP_HEADER(CRTP_PORT_MEM, MEM_WRITE_CH);
  // Dont' touch the first 5 bytes, they will be the same.

  switch(memId)
  {
    case EEPROM_ID:
      status = EIO;
      break;

    case LEDMEM_ID:
      status = EIO;
      break;

    case LOCO_ID:
      // Not supported
      status = EIO;
      break;

    case TRAJ_ID:
      {
        if ((memAddr + writeLen) <= sizeof(trajectories_memory)) {
          memcpy(&(trajectories_memory[memAddr]), &p.data[5], writeLen);
          status = STATUS_OK;
        } else {
          status = EIO;
        }
      }
      break;

    default:
      status = EIO;
      break;
  }

  p.data[5] = status;
  p.size = 6;

  crtpSendPacket(&p);
}
