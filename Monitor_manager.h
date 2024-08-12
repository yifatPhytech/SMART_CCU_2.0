#ifndef __MONITOR_MANAGER_H
#define __MONITOR_MANAGER_H


extern bit bWaitForMonitorCmd;
extern bit bEndOfMonitorTask;
extern BYTE monitorCurTask;
extern BYTE bMonitorConnected;

/////////////////////////////////////////////
// Monitor_manager functions
////////////////////////////////////////////

void MonitorMain();

void MeasureBatt();

bool MeasureMain();

#endif __MONITOR_MANAGER_H
