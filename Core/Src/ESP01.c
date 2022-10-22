/*
 * ESP01.c
 *
 *  Created on: Oct 17, 2022
 *      Author: German
 */

#include "ESP01.h"

typedef enum{
	STESP01IDLE,
	STESP01ATAT,
	STESP01ATCIPMUX,
	STESP01ATCWJAP,
	STESP01ATCIFSR,
	STESP01ATCLOSE,
	STESP01ATCIPSTART,
	STESP01ATCONNECTED,
	STESP01HARDRESET,
	STESP01ATCWMODE,
}_eESP01STATUS;


#define SIZEBUFRX		512
#define SIZEBUFTX		512


//static void ESP01DecodeAT();
//static void ESP01DoWifi();
//static uint8_t CmpResponse(const char *str, uint16_t index, uint8_t n);

//static uint8_t bufRX[SIZEBUFRX], header, timeOut;
//static uint16_t iRXw, iRXr, nRXbytes, iResponse, ipdBytes;
//
//static uint8_t bufTX[SIZEBUFTX];
//static uint16_t iTXw, iTXr, nTXbytes;
//
//static uint32_t timeOutResponse = 0;
//static uint8_t triesAT = 0;
//
//static ESP01GpioWrite *doESP01GpioWrite = 0;
//
//static char SSID[48] = "";
//static char PASSWORD[32] = "";
//static char RemoteIP[16] = "";
//static uint16_t RemotePort = 0;
//
//static uint8_t esp01Status = STESP01IDLE;
//static uint8_t WAITINGSYMBOL = 0;
//static uint8_t STARTWAITINGSYMBOL = 0;
//
//static const char WIFIConnected[] = "WIFI CONNECTED\r";
//static const char WIFIGotIP[] = "WIFI GOT IP\r";
//static const char WIFIDisconnected[] = "WIFI DISCONNECTED\r";
//const char WIFIDisconnect[] = "WIFI DISCONNECT\r";
//const char _OK[] = "OK\r";
//const char _ERROR[] = "ERROR\r";
//const char _CONNECT[] = "CONNECT\r";
//const char _CLOSED[] = "CLOSED\r";
//const char _DISCONNECTED[] = "DISCONNECTED\r";
//const char _DNSFAIL[] = "DNS FAIL\r";
//const char _SENDOK[] = "SEND OK\r";
//const char _BUSY[] = "busy p...\r";
//const char _AT[] = "AT\r\n";
//const char _ATCIPMUX[] = "AT+CIPMUX=0\r\n";
//const char _ATRST[] = "AT+RST\r\n";
//const char _ATCIPSEND[] = "AT+CIPSEND=";
//const char _ATCIPCLOSE[] = "AT+CIPCLOSE\r\n";
//const char _ATCWMODE[] = "AT+CWMODE_CUR=1\r\n";
//const char _ATCIPSTART[] = "AT+CIPSTART=";
//const char _ATCIPSTARTUDP[] = "AT+CIPSTART=\"UDP\",";
//const char _ATCIPSTARTTCP[] = "AT+CIPSTART=\"TCP\",";
//const char _ATCWJAP[] = "AT+CWJAP=";
//const char _ATCIPDNS[] = "AT+CIPDNS_CUR=1,\"208.67.220.220\",\"8.8.8.8\"\r\n";
//const char _ATCIFSR[] = "AT+CIFSR\r\n";
//const char CIFSRAPIP[] = "+CIFSR:APIP\r";
//const char CIFSRAPMAC[] = "+CIFSR:APMAC\r";
//const char CIFSRSTAIP[] = "+CIFSR:STAIP\r";
//const char CIFSRSTAMAC[] = "+CIFSR:STAMAC\r";
//
//
//
//void ESP01AttachGpioWriteCH_EN(ESP01GpioWrite *aESP01GpioWrite){
//	doESP01GpioWrite = aESP01GpioWrite;
//}
//
//void ESP01OnRX(uint8_t *buf, uint16_t length){
//	for(uint16_t i=0; i<length; i++){
//		bufRX[iRXw++] = buf[i];
//		iRXw &= (SIZEBUFRX-1);
//	}
//}
//
//void ESP01Init(char *aSSID, char *aPASSWORD, uint8_t *aBufOnData, uint16_t aSizeBufOnData){
//
//}
//
//uint8_t ESP01ConnectUDP(char *aRemoteIP, uint16_t aRemotePort, uint16_t aLocalPort){
//
//}
//
//void ESP01Task(){
//
//	if(SSID[0]=="\0" && esp01Status!=STESP01IDLE)
//		esp01Status = STESP01IDLE;
//
//	if(iRWw != iRXr)
//		ESP01DecodeAT();
//
//	ESP01DoWifi();
//
//}
//
//void ESP01TimeOut(){
//	timeOutESP01++;
//}
//
//
//static void ESP01DoWifi(){
//	if(timeOutResponse || STESP01IDLE==ATIDLE)
//		return;
//
//	switch(esp01Status){
//	case STESP01ATAT:
//		WAITINGSYMBOL = 0;
//		STARTWAITINGSYMBOL = 0;
//		if(triesAT){
//			triesAT--;
//			if(!triesAT){
//				esp01Status = STESP01HARDRESET;
////				HAL_GPIO_WritePin(HARDRST_GPIO_Port, HARDRST_Pin, GPIO_PIN_RESET);
//				timeOutResponse = 100;
//			}
//			else{
//				PutAT(_AT);
//				timeOutResponse = 10;
//			}
//		}
//		else{
//			PutAT(_AT);
//			timeOutResponse = 10;
//			triesAT = 5;
//		}
//		break;
//	case STESP01ATCLOSE:
//		PutAT(_ATCIPCLOSE);
//		timeOutResponse = 10;
//		break;
//	case STESP01ATCWMODE:
//		PutAT(_ATCWMODE);
//		timeOutResponse = 10;
//		break;
//	case STESP01ATCIPMUX:
//		PutAT(_ATCIPMUX);
//		timeOutResponse = 5;
//		break;
//	case STESP01ATCWJAP://"SSID","SSIDPass"
//		PutAT(_ATCWJAP);
////		PutValueOnTx(&TXUSART3, '"');
////		for(aux=0; aux<33; aux++){
////			if(configData.SSID[aux]=='\0')
////				break;
////			PutValueOnTx(&TXUSART3, configData.SSID[aux]);
////		}
////		PutValueOnTx(&TXUSART3, '"');
////		PutValueOnTx(&TXUSART3, ',');
////		PutValueOnTx(&TXUSART3, '"');
////		for(aux=0; aux<33; aux++){
////			if(configData.SSIDPassword[aux]=='\0')
////				break;
////			PutValueOnTx(&TXUSART3, configData.SSIDPassword[aux]);
////		}
////		PutValueOnTx(&TXUSART3, '"');
////		PutValueOnTx(&TXUSART3, '\r');
////		PutValueOnTx(&TXUSART3, '\n');
//		timeOutResponse = 100;
//		break;
//	case STESP01ATCIFSR:
//		PutAT(_ATCIFSR);
//		timeOutResponse = 10;
//		break;
//	case STESP01ATCLOSE:
//		PutAT(_ATCIPCLOSE);
//		timeOutResponse = 10;
//		break;
//	case STESP01ATCIPSTART:
//		PutAT(_ATCIPSTART);
////		PutValueOnTx(&TXUSART3, '"');
////		PutBufOnTx(&TXUSART3, (uint8_t *)configData.PROTO, 3);
////		PutValueOnTx(&TXUSART3, '"');
////		PutValueOnTx(&TXUSART3, ',');
////		PutValueOnTx(&TXUSART3, '"');
////		for(aux=0; aux<65; aux++){
////			if(configData.RemoteIPURL[aux] == '\0'){
////				break;
////			}
////			PutValueOnTx(&TXUSART3, configData.RemoteIPURL[aux]);
////		}
////		PutValueOnTx(&TXUSART3, '"');
////		PutValueOnTx(&TXUSART3, ',');
////		for(aux=0; aux<6; aux++){
////			if(configData.RemotePort[aux] == '\0'){
////				break;
////			}
////			PutValueOnTx(&TXUSART3, configData.RemotePort[aux]);
////		}
////		PutValueOnTx(&TXUSART3, ',');
////		for(aux=0; aux<6; aux++){
////			if(configData.LocalPort[aux] == '\0'){
////				break;
////			}
////			PutValueOnTx(&TXUSART3, configData.LocalPort[aux]);
////		}
////		PutValueOnTx(&TXUSART3, '\r');
////		PutValueOnTx(&TXUSART3, '\n');
////		timeOutResponse = 50;
////		SENDPOST = 0;
////		timeOutReceiveOk = 40;
//		break;
//	case STESP01ATCONNECTED:
//		break;
//	default:
//		esp01Status = STESP01ATAT;
//	}
//
//}
//
//
//static uint8_t CmpResponse(const char *str, uint16_t index, uint8_t n){
//	uint8_t j;
//
//	for(j=0; j<n; j++){
//		if(str[j] != bufRX[index])
//			return 0;
//		index++;
//		index &= (SIZEBUFRX - 1);
//	}
//	return 1;
//}
//
//
//static void ESP01DecodeAT(){
//	uint16_t index;
//
//	index = iRXw;
//	index &= (SIZEBUFRX - 1);
//
//	while(iRXr != index){
//		switch(header){
//			case 0:
//				if(bufRX[iRXr] == 'A')
//					header = 1;
//				if(bufRX[iRXr] == 'W'){
//					header = 10;
//					iResponse = RX->ir;
//				}
//				if(bufRX[iRXr] == 'l')
//					header = 20;
//				if(bufRX[iRXr] == '+'){
//					header = 30;
//					iResponse = RX->ir;
//				}
//				if(bufRX[iRXr] == '\r')
//					header = 40;
//				if(bufRX[iRXr]=='C' || bufRX[iRXr]=='D'){
//					header = 50;
//					iResponse = RX->ir;
//				}
//				if(bufRX[iRXr] == '>'){
//					timeOutSymbol = 0;
//					WAITINGSYMBOL = 0;
//				}
//				timeOut = 100;
//				break;
//			case 1:
//				if(bufRX[iRXr] == 'T')
//					header = 2;
//				else
//					header = 0;
//				break;
//			case 2:
//				if(bufRX[iRXr] == '\n')
//					header = 0;
//				break;
//			case 10:
//				if(bufRX[iRXr] == 'I')
//					header = 11;
//				else
//					header = 0;
//				break;
//			case 11:
//				if(bufRX[iRXr] == 'F')
//					header = 12;
//				else
//					header = 0;
//				break;
//			case 12:
//				if(bufRX[iRXr] == 'I')
//					header = 13;
//				else
//					header = 21;
//				break;
//			case 13:
//				if(bufRX[iRXr] == '\n'){
//					header = 0;
//					if(CmpResponse(WIFIGotIP, iResponse, 11)==1){
//						if(esp01Status == STESP01ATCWJAP){
//							esp01Status = STESP01ATCIFSR;
//							timeOutResponse = 20;
//						}
//					}
//					if(CmpResponse(WIFIDisconnected, iResponse, 17)==1){
//						esp01Status = STESP01ATAT;
//						WIFICONNECTED = 0;
//					}
//					if(CmpResponse(WIFIDisconnect, iResponse, 15)==1){
//						if(esp01Status != STESP01ATCWJAP){
//							esp01Status = STESP01ATAT;
//							WIFICONNECTED = 0;
//						}
//					}
//				}
//				break;
//			case 20:
//				if(bufRX[iRXr] == 'i')
//					header = 21;
//				else
//					header = 0;
//				break;
//			case 21:
//				if(bufRX[iRXr] == '\n')
//					header = 0;
//				break;
//			case 30:
//				if(bufRX[iRXr] == '\n'){
//					header = 0;
//					if(esp01Status == STESP01ATCIFSR){
//						if(CmpResponse(CIFSRSTAIP, iResponse, 12) == 1){
//						}
//						if(CmpResponse(CIFSRSTAMAC, iResponse, 13) == 1){
//							esp01Status = STESP01ATCLOSE;
//							timeOutResponse = 0;
//							WIFICONNECTED = 1;
//							triesAT = 0;
//						}
//					}
//				}
//				break;
//			case 40:
//				if(bufRX[iRXr] == '\n')
//					header = 41;
//				else
//					header = 0;
//				break;
//			case 41:
//				if(bufRX[iRXr] == '+')
//					header = 43;
//				else{
//					header = 42;
//					iResponse = iRXr;
//				}
//				break;
//			case 42:
//				if(bufRX[iRXr] == '\n'){
//					header = 0;
//					if(CmpResponse(_OK, iResponse, 2) == 1){
//						if(ATSTATUS == ATCIPMUX){
//							ATSTATUS = ATCIPDNS;
//							timeOutResponse = 0;
//						}
//						if(esp01Status == STESP01ATAT){
//							esp01Status = STESP01ATCLOSE;
//							timeOutResponse = 0;
//						}
//						if(esp01Status == STESP01ATCWMODE){
//							timeOutResponse = 0;
//							esp01Status = STESP01ATCIPMUX;
//						}
//					}
//					if(CmpResponse(_BUSY, iResponse, 4) == 1){
//						esp01Status = STESP01ATAT;
//						triesAT = 1;
//						WIFICONNECTED = 0;
//						timeOutResponse = 0;
//					}
//					if(CmpResponse(_SENDOK, iResponse, 7) == 1){
//						timeOutResponse = 0;
//					}
//				}
//				break;
//			case 43:
//				if(bufRX[iRXr] == 'I')
//					header = 44;
//				else
//					header = 42;
//				break;
//			case 44:
//				if(bufRX[iRXr] == 'P')
//					header = 45;
//				else
//					header = 42;
//				break;
//			case 45:
//				if(bufRX[iRXr] == 'D')
//					header = 46;
//				else
//					header = 42;
//				break;
//			case 46:
//				if(bufRX[iRXr] == ','){
//					header = 47;
//					ipdBytes = 0;
//				}
//				else
//					header = 42;
//				break;
//			case 47:
//				if(bufRX[iRXr] == ':')
//					header = 48;
//				else{
//					ipdBytes *= 10;
//					ipdBytes += (bufRX[iRXr] - 0x30);
//				}
//				break;
//			case 48:
////				if(DecodeHeaderAT(RX->buf[RX->ir])){
////					RX->iData = atidata;
////					DecodeCMD(RX, &TXUSART3);
////				}
////				ipdBytes--;
////				if(ipdBytes == 0)
////					atheader = 0;
//				break;
//			case 50:
//				if(bufRX[iRXr] == '\n'){
//					header = 0;
//					if(CmpResponse(_CONNECT, iResponse, 7) == 1){
//						if(esp01Status == STESP01ATCIPSTART){
//							esp01Status = STESP01ATCONNECTED;
//							timeOutResponse = 0;
//						}
//					}
//					if(CmpResponse(_DISCONNECTED, iResponse, 12) == 1){
//						esp01Status = STESP01ATAT;
//						timeOutResponse = 0;
//						WIFICONNECTED = 0;
//					}
//					if(CmpResponse(_CLOSED, iResponse, 6) == 1){
//						if(esp01Status == STESP01ATCLOSE){
//							esp01Status = STESP01ATCIPSTART;
//							timeOutResponse = 0;
//						}
//					}
//				}
//				break;
//			default:
//				header = 0;
//		}
//	}
//}



