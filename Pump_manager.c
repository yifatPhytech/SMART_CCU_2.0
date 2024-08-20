#include <stdio.h>
#include <stdlib.h>

#include "Pump_manager.h"
#include "Rtc_Manager.h"
#include "data_manager.h"

unsigned short DAYS_EACH_MONTH[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

PumpUnit pumpAsVlv[2];

BYTE g_bMngPumpNow;
static BYTE g_cmdPumFailed = 0;


BYTE GetPumpStat()
{
    if ((pumpAsVlv[0].IsPumpOpen == FALSE) && (pumpAsVlv[1].IsPumpOpen == FALSE))
        return FALSE; 
    return TRUE;
}

BYTE GetCurPumpStat(BYTE idx)
{
    return pumpAsVlv[idx].IsPumpOpen;
}

void CheckPumpFitStatus()
{
    BYTE i;
    for (i = 0; i < 2; i++)
    //if hasnt changd pump state currntly
    if (g_bMngPumpNow == FALSE)            //toddo - make double
     // if pump logic state at CBU is open, and at CCU - it closed      //TODO  add check for 2nd pump
        if ((g_bCBUPumpState[i] == 1) &&  (pumpAsVlv[i].IsPumpOpen == FALSE))   //(g_bMainPumpOpen == FALSE))            
            CloseMainPump(i, TRUE);
}

void InitPumpCmdUnit()
{
    BYTE i;
    for (i = 0; i < 2; i++)
    {
        InitCmd(&pumpAsVlv[i].cmdData);
        pumpAsVlv[i].IsCmd4Pump = 0;   
        pumpAsVlv[i].IsPumpOpen = 0;  
        InitCmd(&pumpAsVlv[i].nextIrg);        
//        pumpAsVlv[i].nextIrg = NULL;    //.iDuration = 0;//NULL;      
    }
}

//BYTE IsPumpCmd()
//{
//    return pumpAsVlv.IsCmd4Pump;
//}

BYTE OpenCloseMainPump(BYTE pmpIdx, int actionID, BYTE bOpen)
{
    BYTE n = 0, res;
    unsigned int cmdIdx;  
    ERS485Command cmd;
           
    if (pmpIdx == 0)
        cmd = CMD_PUMP1_MNG;
    if (pmpIdx == 1)
        cmd = CMD_PUMP2_MNG;
    do
    {
        res = SendRecRS485(cmd, bOpen);
        n++;                 
    }       
    while ((n < 3) && (res != 1));      
     
    cmdIdx = GetPumpCmdIdx(pmpIdx);    
                    
    SavePumpActionData(actionID, res, cmdIdx, pmpIdx);
    if (res == 1)             
    {
        g_cmdPumFailed = 0;
    }
    else
        g_cmdPumFailed++;
    if (g_cmdPumFailed > 3)
    {
        g_cmdPumFailed = 0;   
        res = 1;
    }        
    g_bMngPumpNow = TRUE;
    return res;   
}

void CloseMainPump(BYTE pmpIdx, int bForceClose)
{
    if ((pumpAsVlv[pmpIdx].IsPumpOpen == FALSE) && (!bForceClose))
        return; 
    
    #ifdef DebugMode
    SendDebugMsg("\r\nClose Main Pump \0");
    #endif DebugMode   
//    if (pmpIdx == 0) 
    if (OpenCloseMainPump(pmpIdx, 4, PUMP_CMD_OFF) == 1)
        pumpAsVlv[pmpIdx].IsPumpOpen = FALSE;   
}

void OpenMainPump(BYTE pmpIdx, int bForceOpen)
{
    #ifdef DebugMode
    SendDebugMsg("\r\nOpen Pump\0");
    #endif DebugMode
    if ((pumpAsVlv[pmpIdx].IsPumpOpen == TRUE) && (bForceOpen == 0))
        return ;               
        
    if (OpenCloseMainPump(pmpIdx, 2, PUMP_CMD_ON) == 1)             
        pumpAsVlv[pmpIdx].IsPumpOpen = TRUE;    
    #ifdef DebugMode
    SendDebugMsg("\r\nPump Opened\0");
    #endif DebugMode    
}

void TestMainPump(BYTE nDur)
{
    OpenMainPump(0, TRUE);
    delay_ms(nDur * 1000);
    CloseMainPump(0, 1);
}

DateTime GetTimeAfterDelay(int delay, DateTime dt)
{
    int tmp; 
//    DateTime dt;      
//    dt = g_curTime; 
                  
//    tmp = g_curTime.minute + delay;  
    tmp = dt.minute + delay;  
    #ifdef DebugMode   
    SendDebugMsg("\r\ndelay \0");     
    PrintNum(delay);
    SendDebugMsg("\r\nminutes+duration: \0");     
    PrintNum(tmp);
    #endif DebugMode       
    if (tmp < 60)
        dt.minute = tmp;    
    else
        {              
            dt.minute = tmp % 60;//dt.minute % 60; 
            tmp = tmp / 60;
            tmp += dt.hour;
            if (tmp < 24)
                dt.hour = tmp;
            else
            {
                dt.day += tmp / 24;
                dt.hour = tmp % 24;
                if (dt.day > DAYS_EACH_MONTH[dt.month-1])
                {
                    dt.day -= DAYS_EACH_MONTH[dt.month-1];
                    dt.month++;
                    if (dt.month > 12)
                    {
                        dt.month = 1;
                        dt.year++;    
                    }
                } 
            }
        }         
    return dt;
}

void OpenPumpCmd(BYTE pmpIdx)
{
    OpenMainPump(pmpIdx, TRUE);
    pumpAsVlv[pmpIdx].stopTimeStamp = GetTimeAfterDelay(pumpAsVlv[pmpIdx].cmdData.iDuration, g_curTime);   
//    CalcPumpCmdEndTime();     
    pumpAsVlv[pmpIdx].cmdData.cycles--; 
//     #ifdef DebugMode   
//    SendDebugMsg("\r\nCalc pump EndTime: \0");   
//    #endif DebugMode   
}

/*void ClosePumpCmd(BYTE pmpIdx)
{
    DateTime dt;
    
    CloseMainPump(pmpIdx, TRUE);
    if (pumpAsVlv[pmpIdx].cmdData.cycles == 0)  
    {               
        if (pumpAsVlv[pmpIdx].nextIrg.iDuration == 0)
            pumpAsVlv[pmpIdx].IsCmd4Pump = FALSE;      
        else                                       
        {
            pumpAsVlv[pmpIdx].cmdData = pumpAsVlv[pmpIdx].nextIrg;
//            free(pumpAsVlv[pmpIdx].nextIrg);
            InitCmd(&pumpAsVlv[pmpIdx].nextIrg);// = NULL; 
        }
    }            
    else         
    {
        dt = GetTimeAfterDelay(pumpAsVlv[pmpIdx].cmdData.offTime, g_curTime);  
        pumpAsVlv[pmpIdx].cmdData.startTime.hour = dt.hour;   
        pumpAsVlv[pmpIdx].cmdData.startTime.minute = dt.minute;   
    }
} */

void ClosePumpCmd(BYTE pmpIdx)
{
    DateTime dt;
    
    CloseMainPump(pmpIdx, TRUE);     
    
    if (pumpAsVlv[pmpIdx].nextIrg.iDuration > 0)         // if got next irrigation - it cancelled the next cycles if were
    {
        pumpAsVlv[pmpIdx].cmdData = pumpAsVlv[pmpIdx].nextIrg;
        InitCmd(&pumpAsVlv[pmpIdx].nextIrg);// = NULL; 
    }    
    else     
    {
        if (pumpAsVlv[pmpIdx].cmdData.cycles == 0)        // if no next and no more cycles - pump is free
        {               
            pumpAsVlv[pmpIdx].IsCmd4Pump = FALSE;      
        }                                                   
        else                                                // if has more cycles - calc next opening time
        {
            dt = GetTimeAfterDelay(pumpAsVlv[pmpIdx].cmdData.offTime, g_curTime);  
            pumpAsVlv[pmpIdx].cmdData.startTime.hour = dt.hour;   
            pumpAsVlv[pmpIdx].cmdData.startTime.minute = dt.minute;   
        }
    }
}

void CheckPumpStatus()
{
    int  tmp;
    BYTE  t, i;

//    if (pumpAsVlv.IsCmd4Pump == FALSE)
//        return 0; 
    for (i = 0; i < 2; i++)
    {                   
    #ifdef DebugMode        
    SendDebugMsg("\r\nCheck pump status ");     
    PrintNum(i);
    SendDebugMsg("\r\nIsPump Open ");     
    PrintNum(pumpAsVlv[i].IsPumpOpen);
    #endif          
        if (pumpAsVlv[i].IsPumpOpen == FALSE)   
        {
            if (pumpAsVlv[i].IsCmd4Pump == TRUE)
            {                     
                tmp = IsTimeStartIrg(pumpAsVlv[i].cmdData.startTime);   

                #ifdef DebugMode 
                SendDebugMsg("\r\nSec 2 start pump \0");     
                PrintNum(tmp);  
                #endif DebugMode
                if (tmp == 1) //(tmp < 60)               
                {                            
                    OpenPumpCmd(i);    
                }     
            }           
        }     
        else
        {        
            t = GetLaterTimestamp(g_curTime, pumpAsVlv[i].stopTimeStamp);  
            #ifdef DebugMode   
            SendDebugMsg("\r\nGetLaterTimestamp g_curTime stopTimeStamp \0");     
            PrintNum(t);  
            #endif DebugMode

            if ((t == 1) || (t == 0))  
            {
                ClosePumpCmd(i);   
            }      
        }   
    }    
//    return pumpAsVlv.IsCmd4Pump;
}

unsigned int GetPumpCmdIdx(BYTE pmpIdx)
{
    if (pumpAsVlv[pmpIdx].IsCmd4Pump == TRUE) 
        return pumpAsVlv[pmpIdx].cmdData.index;  
    return 0;
}

BYTE SetPumpCmd(unsigned int dur, BYTE offTime, BYTE cycles, BYTE startHour, BYTE startMin, unsigned int cmdIndex, BYTE pumpIdx)
{
    Time t, t1, t2;
    unsigned long n1, n2;    
    BYTE bIsNextIrg = FALSE;
         
    if (dur > MAX_IRRIGATION_MNT)      
        return 0;  
    if ((pumpIdx != 0) && (pumpIdx != 1))  
        return 0;
    GetRealTime();   
    SavePumpActionData(6, 1, cmdIndex, pumpIdx); 
    // if server send start time is cycles = -99 it means start now, end time 0is in start time
    if (cycles == 0x9D) //((startHour == 99) && (startMin == 99)) 
    {      
        t2.hour = startHour;
        t2.minute = startMin; 
        startHour = g_curTime.hour;                            
        startMin = g_curTime.minute;      
    }   
    t.hour = startHour;
    t.minute = startMin;                                       

    // if valve is already working - check if this command is during its work
    if ((pumpAsVlv[pumpIdx].IsPumpOpen) &&  (dur != 0))//  ((g_bMainPumpOpen == TRUE) &&)  
    {                
        n1 = CalcScndsToStart(t);
        t1.hour = pumpAsVlv[pumpIdx].stopTimeStamp.hour;
        t1.minute = pumpAsVlv[pumpIdx].stopTimeStamp.minute;        
        n2 = CalcScndsToStart(t1); 
        // if it later -        
        if (n1 > n2)
            bIsNextIrg = TRUE;  
    }
    if (bIsNextIrg == FALSE)    
    {
        pumpAsVlv[pumpIdx].cmdData.startTime = t; 
        pumpAsVlv[pumpIdx].cmdData.offTime = offTime;    
        if (cycles == 0)
            pumpAsVlv[pumpIdx].cmdData.cycles = 1;   
        if (cycles != 0x9D)
        {
            pumpAsVlv[pumpIdx].cmdData.iDuration = dur;     
            pumpAsVlv[pumpIdx].cmdData.cycles = cycles;
        }  
        else
        {       
            if (dur != 0)                                                 
                pumpAsVlv[pumpIdx].cmdData.iDuration = CalcDuration(t2);
            pumpAsVlv[pumpIdx].cmdData.cycles = 1;
        } 
        pumpAsVlv[pumpIdx].cmdData.index = cmdIndex;    
        pumpAsVlv[pumpIdx].IsCmd4Pump = TRUE;
                  
        if (dur == 0)                                
        {                         
            pumpAsVlv[pumpIdx].IsCmd4Pump = FALSE;   
            CloseMainPump(pumpIdx, TRUE);   
        }
        else
            if (cycles == 0x9D)//((startHour == g_curTime.hour) && (startMin == g_curTime.minute))   
            {
                //OpenMainPump(TRUE);  
                OpenPumpCmd(pumpIdx);  
            }
    }   
    else
    {           
        #ifdef DebugMode  
        SendDebugMsg("\r\nnext Irg \0");
        #endif DebugMode 
//        if (pumpAsVlv[pumpIdx].nextIrg == NULL)   
//        {
//            pumpAsVlv[pumpIdx].nextIrg = (CommandData*)malloc(sizeof(CommandData));       
//        }
//        if (pumpAsVlv[pumpIdx].nextIrg != NULL)
        { 
            pumpAsVlv[pumpIdx].nextIrg.iDuration = dur;
            pumpAsVlv[pumpIdx].nextIrg.startTime = t;   
            pumpAsVlv[pumpIdx].nextIrg.offTime = offTime;    
            pumpAsVlv[pumpIdx].nextIrg.cycles = cycles;    
            pumpAsVlv[pumpIdx].nextIrg.index = cmdIndex;              
//            pumpAsVlv[pumpIdx].cmdData.cycles = 0;           // current irrigation should end anyway after this cycle, never mind if there was more cycles
            #ifdef DebugMode  
            SendDebugMsg("\r\nsetup next pump work\0");
            #endif DebugMode     
        }  
//        else
//        {
//            #ifdef DebugMode  
//            SendDebugMsg("\r\nallocate mem fail1\0");
//            #endif DebugMode     
//        }
    }
}


