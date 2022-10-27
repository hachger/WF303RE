/*
 * ESP01.h
 *
 *  Created on: Oct 17, 2022
 *      Author: German
 */

#ifndef INC_ESP01_H_
#define INC_ESP01_H_

#include <stdint.h>

typedef enum{
	ESP01WIFIDISCONNECTED,
	ESP01WIFICONNECTING,
	ESP01WIFICONNECTED,
	ESP01UDPREADY,
	ESP01UDPBUSY,
}ESP01STATE;

#define SIZEBUFRX		512
#define SIZEBUFTX		512
#define SIZEBUFDATAGRAM 128

typedef void *(ESP01GpioWriteCH_EN)(uint32_t value);
typedef void *(ESP01OnUDPData)(uint8_t * buf, uint16_t lenght);
typedef void *(ESP01OnWIFIConnected)();
typedef void *(ESP01OnWIFIDisconnected)();
typedef void *(ESP01OnUDPReady)();

void ESP01Init(ESP01GpioWriteCH_EN *aESP01GpioWriteCH_EN, ESP01OnUDPData *aESP01OnUDPData);
uint8_t ESP01SetWIFI(char *aSSID, char *aPASSWORD);
uint8_t ESP01ConnectUDP(char *aRemoteIP, uint16_t aRemotePort, uint16_t aLocalPort);
void ESP01SetRxByte(uint8_t value);
uint8_t ESP01GetTxByte(uint8_t *value);
uint8_t ESP01SendUDPData(uint8_t *buf, uint16_t dataLength, uint16_t bufSize);
void ESP01Task();
void ESP01TimeOut10ms();
void ESP01AttachOnWIFIConnected(ESP01OnWIFIConnected *aESP01OnWIFIConnected);
void ESP01AttachOnWIFIDisconnected(ESP01OnWIFIDisconnected *aESP01OnWIFIDisconnected);
void ESP01AttachOnUDPReady(ESP01OnUDPReady *aESP01UDPReady);
ESP01STATE ESP01GetLastSTATE();


#endif /* INC_ESP01_H_ */
