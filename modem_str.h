#ifndef __MODEM_STR_H
#define __MODEM_STR_H

flash unsigned char PHYTECH_FILE_END_MARK[] = "phy111\r\n#";  //8  MUST BE AT LEAST 6 CHARS!!!!!!!!!

flash unsigned char  AT_DELL_ECHO[] = "ATE0\r\n\0";
flash unsigned char  AT_SAVE[] = "AT&W\r\n\0"; 
flash unsigned char  AT_SET_BAUDRATE[] = "AT+IPR=9600;&W\r\n\0";   //"AT+IPR=19200;&W\r\n\0";
flash unsigned char  AT_IsModemOK[] = "AT\r\n\0";             //5 bytes
flash unsigned char  AT_IsModemReg[] = "AT+CREG?\r\n\0";     //10 bytes
flash unsigned char  AT_REG_UMTS_STATUS[] = "AT+CGREG?\r\n\0";
flash unsigned char  AT_REG_LTE_STATUS[] = "AT+CEREG?\r\n\0";
flash unsigned char  AT_COPS_AUTO_0[] = "AT+COPS=0\r\n\0";      //11 bytes
flash unsigned char  AT_COPS_LST[] = "AT+COPS=?\r\n\0";
flash unsigned char  AT_COPS_MAN[] = "AT+COPS=1,2,@";       //12 bytes. must have # at the end
flash unsigned char  AT_COPS_MAN_MONITOR[] = "AT+COPS=1,2,\"42503\"\r\n\0";       //only for monitor connecting
flash unsigned char  AT_COPS_ASK[] = "AT+COPS?\r\n\0";
flash unsigned char  AT_CELL_MONITOR[] = "AT+QNWINFO\r\n\0";   //"AT#MONI\r\n\0";
flash unsigned char  AT_CSQ[] = "AT+CSQ\r\n\0";               //8 bytes  AT+GMR-returns the software revision identification
flash unsigned char  AT_EOD[] = "+++\0";               //8 bytes  AT+GMR-returns the software revision identification
flash unsigned char  AT_QSS[] = "AT+QSIMSTAT?\r\n\0"; //"AT#QSS?\r\n\0";   
flash unsigned char AT_CCID[] = "AT+QCCID\r\n\0";       //"AT#CCID\r\n\0";
flash unsigned char AT_PWROFF[] = "AT+QPOWD\r\n\0";

//GPRS connecting commands:
flash unsigned char GPRS_ATTACH[] = "AT+CGATT=1\r\n\0";                               //12
flash unsigned char DEF_PDP_CNTXT[] = "AT+CGDCONT=1,\"IP\",@";    //39
flash unsigned char DEF_PDP_CNTXT_VZN[] = "AT+CGDCONT=1,\"IPV4V6\",@";    //39
flash unsigned char ACTIVATE_CNTXT[] = "AT+CGACT=1,1\r\n\0";    //"AT#SGACT=1,1\r\n\0";                                //14
flash unsigned char DEACTIVATE_CNTXT[] = "AT+CGACT=0,1\r\n\0";  //"AT#SGACT=1,0\r\n\0";                                //14
flash unsigned char ISACTIVATE_CNTXT[] = "AT+CGACT?\r\n\0";
flash unsigned char AT_TCP_OPN[] = "AT+QIOPEN=1,0,\"TCP\",@";           //"AT#SD=1,0,1020,\"phytech1.dyndns.org\"\r\n";   //40
flash unsigned char AT_TCP_CLS[] = "AT+QICLOSE=0\r\n\0";    //"AT#SH=1\r\n\0";                               //9
// post commands
//flash unsigned char AT_POST_TITLE_PRM[] = "POST /api/sensor/loggerparamsezr HTTP/1.1\r\n#";  //                 api/file/sensorparams
flash unsigned char AT_POST_TITLE_PRM[] = "POST /api/sensor/loggerparamscs HTTP/1.1\r\n#";  // 
flash unsigned char AT_POST_TITLE_VLV[] = "POST /api/sensor/getCoolingTask HTTP/1.1\r\n#";  //    
//flash unsigned char AT_POST_TITLE_VLV[] = "POST /api/sensor/getBulkCoolingTask HTTP/1.1\r\n#";  //       
#ifdef SMART_SERVER 
flash unsigned char AT_POST_TITLE_CBU_DATA[] = "POST /data HTTP/1.1\r\n#";  //
#else
flash unsigned char AT_POST_TITLE_CBU_DATA[] = "POST /api/sensor/cbudatav3 HTTP/1.1\r\n#";  //
#endif   
flash unsigned char AT_POST_TITLE_ALERT[] = "POST /api/sensor/cbualerts HTTP/1.1\r\n#";  //   
flash unsigned char AT_POST_TITLE_VCU_LST[] = "POST /api/sensor/ccumetadata HTTP/1.1\r\n#";  //  
flash unsigned char AT_POST_TITLE_CBU_META_DATA[] = "POST /api/sensor/cbumetadatav3 HTTP/1.1\r\n#";  //   
flash unsigned char AT_POST_TITLE_VCU_DATA[] = "POST /api/sensor/vcudata HTTP/1.1\r\n#";              // ValvSensorData
flash unsigned char AT_POST_TITLE_GETPRM[] = "POST /api/sensor/sensorupdate HTTP/1.1\r\n#";  //  41          api/file/postparamupdate
flash unsigned char AT_POST_TITLE_CNFRMPRM[] = "POST /api/sensor/sensorupdateconfirmation HTTP/1.1\r\n#";  //api/file/postparamupdateconfirmation
flash unsigned char AT_POST_TITLE_CNFRMVLV[] = "POST /api/sensor/sensorvlvcmdconfirmation HTTP/1.1\r\n#";  //api/file/postparamupdateconfirmation

flash unsigned char AT_GET_SW_STATUS_3[] = "GET http://@";  
flash unsigned char AT_GET_SW_STATUS_ATM[] = "/status/@";  //  
flash unsigned char AT_GET_SW_STATUS_EZR[] = "/rfstatus/@";  //  
flash unsigned char AT_GET_SW_STATUS_CBU[] = "/cbustatus/@";  //  
flash unsigned char AT_GET_SW_STATUS_2[] = " HTTP/1.1\r\n@"; 
flash unsigned char AT_POST_CONN[] = "Connection: keep-alive\r\n#";                     //24
flash unsigned char AT_POST_TYPE[] = "Content-Type: multipart/form-data; boundary=#";   //52
flash unsigned char AT_POST_HOST[] = "Host: @";         //Host: phytech1.dyndns.org:1011\r\n#";             //32
flash unsigned char AT_POST_LENGTH1[] = "Content-Length: 257\r\n\r\n#";                     //BuildExtParamBuff  253
//flash unsigned char AT_POST_LENGTH2[] = "Content-Length: 11\r\n\r\n#";                     //BuildExtParamBuff  253
flash unsigned char AT_POST_LENGTH4[] = "Content-Length: @";                     //23
//flash unsigned char AT_POST_LENGTH3[] = "Content-Length: 130\r\n\r\n#";                     //23
//flash unsigned char AT_POST_LENGTH6[] = "Content-Length: 123\r\n\r\n#";                     //23
flash unsigned char AT_POST_FILE_HDR1[] = "Content-Disposition: form-data; name=\"file\"; #";             //45
flash unsigned char AT_POST_FILE_HDR_PRM[] = "filename=\"PARAMS.txt\"\r\n#";    //23
flash unsigned char AT_POST_FILE_HDR_DATA[] = "filename=\"VLVDATA.txt\"\r\n#";    //21
flash unsigned char AT_POST_FILE_HDR_UPDTPRM[] = "filename=\"UPDTPRM.txt\"\r\n#";    //29
//flash unsigned char AT_POST_FILE_HDR_GETDATA[] = "filename=\"GETPARAMPOST.txt\"\r\n#";    //29
flash unsigned char AT_POST_FILE_HDR_VALVE[] = "filename=\"VALVCMD.txt\"\r\n#";    //24
flash unsigned char AT_POST_FILE_HDR_ALERT[] = "filename=\"ALERTSS.txt\"\r\n#";    //24
flash unsigned char AT_POST_FILE_HDR_PUMP[] = "filename=\"PUMPACT.txt\"\r\n#";    //24
flash unsigned char AT_POST_FILE_HDR_CBU_DATA[] = "filename=\"CBUDATA.txt\"\r\n#";    //24
flash unsigned char AT_POST_FILE_HDR_CBU_META[] = "filename=\"CBUMETA.txt\"\r\n#";    //24
flash unsigned char AT_POST_FILE_HDR_VCU_LIST[] = "filename=\"VCULIST.txt\"\r\n#";    //24
flash unsigned char AT_POST_FILE_HDR2[] = "Content-Type: text/plain\r\n\r\n#";          //28


#endif __MODEM_STR_H
