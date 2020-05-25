#ifndef __Mousejack_H
#define __Mousejack_H

#include "main.h"
#include "MY_NRF24.h"
//#include "MY_NRF24.h"
//#include "main.c"

//Function list
void print_payload_details(void);
uint16_t crc_update(uint16_t crc, uint8_t byte, uint8_t bits);
bool transmit(void);
void scan(void);
void start_transmit(void);
void ms_crypt(void);
void ms_checksum(void);
void fingerprint(void);
void ms_transmit(uint8_t meta, uint8_t hid);
void log_checksum(void);
void log_transmit(uint8_t meta, uint8_t keys2send[], uint8_t keysLen);
void launch_attack(void);
void reset();
void loop(void);


#endif
