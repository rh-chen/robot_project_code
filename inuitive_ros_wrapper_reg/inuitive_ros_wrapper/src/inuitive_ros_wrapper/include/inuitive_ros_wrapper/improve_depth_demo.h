/****************************************************************************
 *
 *   FileName: improve_depth_demo.h
 *
 *   Author:
 *
 *   Date: 
 *
 *   Description: OS abstraction layer API 
 *   
 ****************************************************************************/

#ifndef IMPROVE_DEPTH_DEMO_H
#define IMPROVE_DEPTH_DEMO_H

#include "inu_types.h"
#include "err_defs.h"


typedef enum
{
   IMPROVE_DEPTH_DEMOG_ALG_ID_NEGATE_E    = 0,
   IMPROVE_DEPTH_DEMOG_NUM_OF_DEMO_ALGS,
   IMPROVE_DEPTH_DEMOG_NO_ALG_ID_E       = 0xFFFFFFFF
} IMPROVE_DEPTH_DEMOG_algIdE;

typedef enum
{
   IMPROVE_DEPTH_DEMOG_START_ALG_E = 0,
   IMPROVE_DEPTH_DEMOG_STOP_ALG_E  = 1
} IMPROVE_DEPTH_DEMOG_opcodeE;


typedef struct
{
   UINT32                  inPhysicalAddr;
   UINT32                  inVirtualAddr;
   UINT32                  outPhysicalAddr;
   UINT32                  bufferWidth;
   UINT32                  bufferHeight;
   UINT32                  frameWidth;
   UINT32                  frameHeight;
   UINT32                  numOfImagesInFrame;  // 1 image for none-interleave frame. 2 imafes for interleave frame
} IMPROVE_DEPTH_DEMOG_FrameParamsT;

typedef struct
{
   UINT8 *ddrP;
   UINT32 ddrSize;
   IMPROVE_DEPTH_DEMOG_opcodeE  opcode;
   IMPROVE_DEPTH_DEMOG_algIdE   algId;
   IMPROVE_DEPTH_DEMOG_FrameParamsT FrameParams;
} IMPROVE_DEPTH_DEMOG_gpCevaMsgProtocolT;


#endif   // IMPROVE_DEPTH_DEMO_H

