#ifndef PUMP_MANAGER_H
#define PUMP_MANAGER_H
#include "Valve_Manager.h"

typedef struct _PumpUnit
{
    CommandData     cmdData; 
    BYTE            IsCmd4Pump;    
    BYTE            IsPumpOpen;    
    DateTime        stopTimeStamp;     
    CommandData*    nextIrg;
}PumpUnit;

extern BYTE g_bMngPumpNow;

BYTE GetPumpStat();

BYTE SetPumpCmd(unsigned int dur, BYTE offTime, BYTE cycles, BYTE startHour, BYTE startMin, unsigned int cmdIndex, BYTE pumpIdx);

void CheckPumpStatus();

DateTime GetTimeAfterDelay(int delay);

void InitPumpCmdUnit();

//BYTE IsPumpCmd();

unsigned int GetPumpCmdIdx(BYTE pmpIdx);

void CheckPumpFitStatus();

void TestMainPump(BYTE nDur);

#endif PUMP_MANAGER_H
