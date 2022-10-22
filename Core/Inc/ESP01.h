/*
 * ESP01.h
 *
 *  Created on: Oct 17, 2022
 *      Author: German
 */

#ifndef INC_ESP01_H_
#define INC_ESP01_H_

#include <stdint.h>

typedef void *(ESP01GpioWriteCH_EN)(uint32_t value);

typedef struct{
	uint8_t *OnDataBuf;
	uint16_t SizeOnDataBuf;
	uint8_t *WriteTxBuf;
	uint16_t SizeWriteTxBuf;
	ESP01GpioWriteCH_EN *aESP01GpioWriteCH_EN;
}_sESP01CONFIG;

void ESP01Init(_sESP01CONFIG *aESP01CONFIG);
uint8_t ESP01SetWIFI(char *aSSID, char *aPASSWORD);
uint8_t ESP01ConnectUDP(char *aRemoteIP, uint16_t aRemotePort, uint16_t aLocalPort);
void ESP01OnRX(uint8_t *buf, uint16_t length);
uint8_t ESP01SendUDPData(uint8_t *buf, uint16_t length);
void ESP01Task();
void ESP01TimeOut10ms();


#endif /* INC_ESP01_H_ */
