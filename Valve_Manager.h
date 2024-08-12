#ifndef __VALVE_MNG_H
#define __VALVE_MNG_H

#include "define.h"

#define IRG_MSG_BUILD_OK    0x2
#define IRG_MSG_BUILD_FAIL  0x3
#define IRG_MSG_DROP_OK     0x4
#define IRG_MSG_DROP_FAIL   0x5
#define IRG_MSG_INIT        0x7
#define IRG_MSG_OFF         0x08
#define IRG_VCU_ERROR       0x0A
#define IRG_MSG_ACK         0x10
#define IRG_MSG_HW_ERROR    0x20
#define IRG_MSG_PING        0x40
#define IRG_MSG_HW_ERROR_2  0x80

//typedef struct _NextIrrigation
//{
//    unsigned int    iDuration;    
//    Time            startTime;
//}NextIrrigation;

typedef struct _CommandData
{
    unsigned int    iDuration;   
    BYTE            offTime;    
    BYTE            cycles;    
    unsigned int    index;    
    Time            startTime;  
}CommandData;

typedef struct _ValveUnit
{
    unsigned long   VCU_ID; 
    CommandData     cmdData; 
    BYTE            cmdStatus;    
    BYTE            vlvStatus;    
    unsigned long   nSec2Start; 
    BYTE            iExtPrm;
    DateTime        stopTimeStamp;     
    CommandData    nextIrg;
}ValveUnit;

//extern BYTE     g_bMainPumpOpen ;
//extern unsigned int gOpenPumpDelay;
//extern unsigned int gClosePumpDelay;
//extern BYTE     g_bHighPrio;
//extern BYTE     g_cmdSndCnt;
//extern DateTime g_PumpCloseTime;
extern BYTE     g_bVlvListUpdated;

extern unsigned int g_lGlobalCmd;

extern  ValveUnit vlvCmdArr[];

void CloseMainPump(BYTE pmpIdx, int bForceClose);

void CloseAllValve();

//void OpenMainPump(BYTE pmpIdx,int bForceOpen);

void InitValvesArrays();

void InitVlvCmd(BYTE i);

//BYTE InsertNewCmd(unsigned long id, unsigned int dur, BYTE startHour, BYTE startMin);
BYTE InsertExtNewCmd(unsigned long id,  unsigned int dur, BYTE offTime, BYTE cycles, BYTE startHour, BYTE startMin, unsigned int cmdIndex);

void UpdateVCUStatus(unsigned long id, BYTE situation, BYTE extMsg);

ValveCmd2EZR GetNextValveCmd();

//void CalcEndTime(BYTE index);

BYTE GetLaterTimestamp(DateTime dt1, DateTime dt2);  //(BYTE index);

BYTE GetCmdsCnt();

BYTE SendVlvCmdToEzr(/*DateTime curDT*/);

void DecreaseTime2End();

BYTE IsPumpBusy();

//void PrintVlvArrStatus();

//void RemoveVCU(unsigned long Id);

void CheckVCUStatus();

BYTE InsertVlv(unsigned long id);

//void InitValvesUpdateStatus();

BYTE DeleteNotinListVlv(BYTE*);

void InitCmd(CommandData* cmd);

unsigned long CalcScndsToStart(Time strtTime);

BYTE IsTimeStartIrg(Time strtTime);

BYTE GetVlvCnt();

int CalcDuration(Time endTime);

#endif __VALVE_MNG_H