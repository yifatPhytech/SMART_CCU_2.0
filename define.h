#ifndef DEFINE_H
#define DEFINE_H

#include <mega644p.h>
#include <delay.h>
#include <stdbool.h>
#include "RS485_Manager.h"

//optional:
#define LiteDebug
//#define ValveDebug
//#define LitePlusDebug
#define DebugMode
//#define Satellite
//#define TestMonitor
//#define OldDataMsg

#define BRD1000_V3

 
typedef unsigned char BYTE;


typedef enum _POST
{
	POST_NONE,
	POST_ALERT,
	POST_CBU_DATA,     
    POST_VCU_DATA,
	POST_PUMP_ACT,
    POST_GET_CMD,
    POST_CCU_PRM, 
    POST_GET_UPDATE,
    POST_CNFRM_UPDATE,  
    POST_CBU_META_DATA,  
    POST_VCU_LIST,
    POST_CNT
} POST;

typedef enum _PUMP_CMD
{
    PUMP_NONE,
    PUMP_OPEN,
    PUMP_CLOSE,
}PUMP_CMD ;

typedef struct _DateTime
{
    unsigned char year;   // 0-99 (representing 2000-2099)
    unsigned char month;  // 1-12
    unsigned char day;    // 1-31
    unsigned char hour;   // 0-23
    unsigned char minute; // 0-59
    unsigned char second; // 0-59
}DateTime;

typedef struct _Time
{
    unsigned char hour;   // 0-23
    unsigned char minute; // 0-59
}Time;

typedef struct _ValveCmd2EZR
{
    unsigned long   m_ID;
    unsigned int    m_duration;  
    unsigned int    m_StartAfter;
    BYTE            m_OffTime; 
    BYTE            m_nCycles;
    unsigned int    m_nCmdIdx;
}ValveCmd2EZR;


typedef struct 
{
    char eLoggerID[4];
    unsigned char eStartConnectionH;
    unsigned char eConnectionInDay;
    unsigned char eConnectIntervalH;
    int     eOpenPumpDelay;             // use this for main pump  
    unsigned char eOpenPumpWithDealy;    // use this for main pump  
    unsigned short nMaxSenNum;
    unsigned short eTimeZoneOffset;
    BYTE    eRoamingDelay;
    BYTE    eUseCntrCode;
    char    eMobileNetCode[5];
    char    eMobileCntryCode[5];
    char    eIPorURLval1[33];
    char    ePORTval1[5];
    char    eAPN[33];   
//    float   eDelta;   
    char    eCbuGlblData[CBU_GET_GLBL_CNFG_LEN];   
//    BYTE    eCbuDryContact;
} _tagAPPEEPROM;


typedef struct 
{
    BYTE            Id; 
    char            Name[10];         
    BYTE            SensorType;
    float           Conversion;
    float           Range;
    float           HighTreshold;
    float           LowTreshold;    
    unsigned int    HighFillingTime;
    unsigned int    LowFillingTime;
    unsigned int    HighStablingTime;
    unsigned int    LowStablingTime;
} _tagPortDefEEPROM;

typedef struct 
{
    char            Reset;
    unsigned int    HighTreshold;
    unsigned int    LowTreshold;    
    unsigned int    HighFillingTime;
    unsigned int    LowFillingTime;
    unsigned int    HighStablingTime;
    unsigned int    LowStablingTime;
} _tagFlowDefEEPROM;

#define DATA_REGISTER_EMPTY (1<<UDRE0)
#define RX_COMPLETE (1<<RXC0)
#define FRAMING_ERROR (1<<FE0)
#define PARITY_ERROR (1<<UPE0)
#define DATA_OVERRUN (1<<DOR0)

//#define MINUTE_BEFORE_IRG   3
//
//#define MAX_INT         32767
//#define MIN_INT         0x8000
//#define BTR_FULL_LIMIT     3600
#define BTR_EMPTY_LIMIT     3700
#define BTR_STATUS_FULL    0
#define BTR_STATUS_EMPTY    1

#define POINTERS_SIZE               sizeof(_ExtEpromPointers)//18  //8
#define VCU_PACKET_SIZE             40
#define PUMP_ACTION_PACKET_SIZE     VCU_PACKET_SIZE  //8
#define CBU_MNT_DATA_SIZE       50
#define ALERTS_MEMORY_PACKET_SIZE   15
#define MAX_DATA_2_EPRM_SIZE        CBU_MNT_DATA_SIZE

#define MAX_CMD 50

#define VCU1        0//1

#define MAX_EMPTY_SEC           600

//#define SENSOR_RSSI_EEPROM_INDEX    3   //11
//#define SENSOR_BTR_EEPROM_INDEX     0   //8

#define INT_VREF 		2560	//reference voltage [mV] internal
#define ADC_VREF_TYPE ((1<<REFS1) | (1<<REFS0) | (0<<ADLAR))  //reference voltage [mV] internal

#define MAX_RX_BUF_LEN      240 //was 64. change to 100 3/2014 for alerts
#define MAX_RX1_BUF_LEN      240 //should be coordinated with the buffer in EZR
#define MAX_WAIT_MNTR_SEC   2 //10

#ifdef UseGPS
#define MINMEA_MAX_LENGTH   MAX_RX1_BUF_LEN
#endif UseGPS

#define SUCCESS 1
#define FAILURE 0

#define CONTINUE    1
#define WAIT        2

#define TRUE 1
#define FALSE 0
#define ERROR 2


#define UART_RADIO_UHF  0x0
#define UART_RS485      0x10
#define UART_DBG        0x20
#define UART_NONE       0x30

#define NO_ANSWER       0
#define TASK_COMPLETE   1
#define TASK_FAILED     2
#define TASK_FAILED_NO_ANSWER       3

#define TASK_NONE       0
#define TASK_EZR_COM    1
#define TASK_MODEM      2
#define TASK_SLEEP      4
#define TASK_WAKEUP     5
#define TASK_MONITOR    6
//#define TASK_GPS        7
#define TASK_BRIDGE     9
//#define TASK_EZR        10
//#define TASK_LISTEN       10

#define RATE9600    1
#define RATE38400   2
#define RATE19200   3

#define TASK_MONITOR_CONNECT    1
#define TASK_MONITOR_WAIT   2

//#define MUX_CHARGE  0


#define MAX_IRRIGATION_MNT  4320    //1080    //
//#define VCU_CLS_MSG_MNT     3
#define CODE_PING           9999
#define CODE_RST            9998
#define CODE_VLV_TST        9997
#define CODE_PUMP_TST       9996
#define CODE_PUMP_START     9995
#define CODE_STOP_ALL       9994



#ifdef BRD1000_V3

#define CUT_OFF() (PORTB.0 = 0);
#endif BRD1000_V3  

#define WIRELESS_PWR_ENABLE() (PORTA.6 = 1);        //##
#define WIRELESS_PWR_DISABLE() (PORTA.6 = 0);

#define WIRELESS_CTS_ENABLE()   (DDRA.7 = 1);      //##
#define WIRELESS_CTS_DISABLE()  (DDRA.7 = 0);

#define WIRELESS_CTS_ON()   (PORTA.7 = 0);      //##
#define WIRELESS_CTS_OFF()  (PORTA.7 = 1);

#define ENABLE_CLOCK_INT()   (EIMSK |= (1<<INT2));    // enable  external interrupt (clock int)
#define DISABLE_CLOCK_INT()  (EIMSK &= ~(1<<INT2));   // disable  external interrupt (clock int)

#define RESET_CLOCK_INT()   (EIFR &= ~(1<<INTF2)) ;

#define ENABLE_TIMER1_COMPA()   ( TIMSK1 |= (1<<OCIE1A));
#define DISABLE_TIMER1_COMPA()  ( TIMSK1 &= ~(1<<OCIE1A));   //

#define ENABLE_TIMER0()    (TCCR0B=(0<<WGM02) | (1<<CS02) | (0<<CS01) | (1<<CS00));
#define DISABLE_TIMER0()   (TCCR0B=(0<<WGM02) | (0<<CS02) | (0<<CS01) | (0<<CS00));

#define ENABLE_TIMER1()     (TCCR1B=(0<<ICNC1) | (0<<ICES1) | (0<<WGM13) | (1<<WGM12) | (0<<CS12) | (1<<CS11) | (0<<CS10));
#define DISABLE_TIMER1()   (TCCR1B=(0<<ICNC1) | (0<<ICES1) | (0<<WGM13) | (0<<WGM12) | (0<<CS12) | (0<<CS11) | (0<<CS10));

#define ENABLE_UART0()      (UCSR0B=(1<<RXCIE0) | (0<<TXCIE0) | (0<<UDRIE0) | (1<<RXEN0) | (1<<TXEN0) | (0<<UCSZ02) | (0<<RXB80) | (0<<TXB80));
#define DISABLE_UART0()     (UCSR0B=(0<<RXCIE0) | (0<<TXCIE0) | (0<<UDRIE0) | (0<<RXEN0) | (0<<TXEN0) | (0<<UCSZ02) | (0<<RXB80) | (0<<TXB80));

#define ENABLE_UART1()      (UCSR1B=(1<<RXCIE1) | (0<<TXCIE1) | (0<<UDRIE1) | (1<<RXEN1) | (1<<TXEN1) | (0<<UCSZ12) | (0<<RXB81) | (0<<TXB81));
#define DISABLE_UART1()     (UCSR1B=(0<<RXCIE1) | (0<<TXCIE1) | (0<<UDRIE1) | (0<<RXEN1) | (0<<TXEN1) | (0<<UCSZ12) | (0<<RXB81) | (0<<TXB81));

#define ENABLE_RX_INT_UART0()  (UCSR1B=(1<<RXCIE0) | (0<<TXCIE0) | (0<<UDRIE0) | (1<<RXEN0) | (1<<TXEN0) | (0<<UCSZ02) | (0<<RXB80) | (0<<TXB80));
#define DISABLE_RX_INT_UART0() (UCSR1B=(0<<RXCIE0) | (0<<TXCIE0) | (0<<UDRIE0) | (1<<RXEN0) | (1<<TXEN0) | (0<<UCSZ02) | (0<<RXB80) | (0<<TXB80));

#define ENABLE_RX_INT_UART1()  (UCSR1B=(1<<RXCIE1) | (0<<TXCIE1) | (0<<UDRIE1) | (1<<RXEN1) | (1<<TXEN1) | (0<<UCSZ12) | (0<<RXB81) | (0<<TXB81));
#define DISABLE_RX_INT_UART1() (UCSR1B=(0<<RXCIE1) | (0<<TXCIE1) | (0<<UDRIE1) | (1<<RXEN1) | (1<<TXEN1) | (0<<UCSZ12) | (0<<RXB81) | (0<<TXB81));

#define ENABLE_ADC()  (ADCSRA=(1<<ADEN) | (0<<ADSC) | (0<<ADATE) | (0<<ADIF) | (0<<ADIE) | (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0));
#define DISABLE_ADC() (ADCSRA=(0<<ADEN) | (0<<ADSC) | (0<<ADATE) | (0<<ADIF) | (0<<ADIE) | (0<<ADPS2) | (0<<ADPS1) | (0<<ADPS0));

#define WATCHDOG_ENABLE_STEP1() (WDTCSR=(0<<WDIE) | (1<<WDP3) | (1<<WDCE) | (1<<WDE) | (0<<WDP2) | (0<<WDP1) | (1<<WDP0));
#define WATCHDOG_ENABLE_STEP2() (WDTCSR=(0<<WDIE) | (1<<WDP3) | (0<<WDCE) | (1<<WDE) | (0<<WDP2) | (0<<WDP1) | (1<<WDP0));

/////////////////////////////////////////////
// GPS_manager functions
////////////////////////////////////////////
#ifdef UseGPS
void GPSMain();
#endif UseGPS


/////////////////////////////////////////////
// Debug functions
////////////////////////////////////////////

void PrintNum(long val);
#ifdef DebugMode

void SendDebugMsg(flash unsigned char *bufToSend);


void PrintOnlyNum(long val);

#endif DebugMode

#endif DEFINE_H