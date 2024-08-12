#ifndef __EEPROM_H
#define __EEPROM_H


extern char e2_writeFlag;
extern volatile unsigned char eepromReadBuf[];    //buffer for eeprom read operation

/////////////////////////////////////////////
// eeprom functions
////////////////////////////////////////////

char e2_readSeqBytes(unsigned int address, char read_length);

char e2_writePage(unsigned int address, char write_length, char* string_1);

#endif __EEPROM_H