/*
 * ESP01.c
 *
 *  Created on: Oct 17, 2022
 *      Author: German
 */

#include "ESP01.h"
#include "string.h"

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


static void ESP01DecodeAT();
static void ESP01DoWifi();
static uint8_t CmpResponse(const char *str, uint16_t index, uint8_t n);
static void ESP01PutAT(const char *atCMD);
static void ESP01PutByteOnTx(uint8_t value);

static uint8_t bufRX[SIZEBUFRX], header, timeOut;
static uint16_t iRXw, iRXr, iResponse, ipdBytes, nBytesDatagram, iBufDatagram;

static uint8_t bufTX[SIZEBUFTX];
static uint16_t iTXw, iTXr;

static uint8_t bufDatagram[SIZEBUFDATAGRAM];

static uint32_t timeOutResponse = 0;
static uint8_t triesAT = 0;
static uint8_t NEEDWAITSYMBOL = 0;
static uint8_t WAITINGSYMBOL = 0;
static uint8_t timeOutSymbol = 0;

static ESP01GpioWriteCH_EN *doESP01GpioWriteCH_EN = 0;
static ESP01OnUDPData *doESP01OnUDPData = 0;
static ESP01OnWIFIConnected *doESP01WIFIConnected = 0;
static ESP01OnWIFIDisconnected *doESP01WIFIDisconnected = 0;
static ESP01OnUDPReady *doESP01UDPReady = 0;


static char SSID[48] = "";
static char PASSWORD[32] = "";
static char RemoteIP[16] = "";
static char RemotePort[6] = "";
static char LocalPort[6] = "";

const char _UDP[] = "UDP";
const char _WIFIConnected[] = "WIFI CONNECTED\r";
const char _WIFIGotIP[] = "WIFI GOT IP\r";
const char _WIFIDisconnected[] = "WIFI DISCONNECTED\r";
const char _WIFIDisconnect[] = "WIFI DISCONNECT\r";
const char _OK[] = "OK\r";
const char _ERROR[] = "ERROR\r";
const char _CONNECT[] = "CONNECT\r";
const char _CLOSED[] = "CLOSED\r";
const char _DISCONNECTED[] = "DISCONNECTED\r";
const char _DNSFAIL[] = "DNS FAIL\r";
const char _SENDOK[] = "SEND OK\r";
const char _BUSY[] = "busy p...\r";
const char _AT[] = "AT\r\n";
const char _ATCIPMUX[] = "AT+CIPMUX=0\r\n";
const char _ATRST[] = "AT+RST\r\n";
const char _ATCIPSEND[] = "AT+CIPSEND=";
const char _ATCIPCLOSE[] = "AT+CIPCLOSE\r\n";
const char _ATCWMODE[] = "AT+CWMODE_CUR=1\r\n";
const char _ATCIPSTART[] = "AT+CIPSTART=";
const char _ATCIPSTARTUDP[] = "AT+CIPSTART=\"UDP\",";
const char _ATCIPSTARTTCP[] = "AT+CIPSTART=\"TCP\",";
const char _ATCWJAP[] = "AT+CWJAP=";
const char _ATCIPDNS[] = "AT+CIPDNS_CUR=1,\"208.67.220.220\",\"8.8.8.8\"\r\n";
const char _ATCIFSR[] = "AT+CIFSR\r\n";
const char _CIFSRAPIP[] = "+CIFSR:APIP\r";
const char _CIFSRAPMAC[] = "+CIFSR:APMAC\r";
const char _CIFSRSTAIP[] = "+CIFSR:STAIP\r";
const char _CIFSRSTAMAC[] = "+CIFSR:STAMAC\r";


_eESP01STATUS ESP01TASKSTATE;

static ESP01STATE lastESP01STATE = ESP01WIFIDISCONNECTED;

static int aux = 0;


void ESP01Init(ESP01GpioWriteCH_EN *aESP01GpioWriteCH_EN, ESP01OnUDPData *aESP01OnUDPData){
	doESP01GpioWriteCH_EN = aESP01GpioWriteCH_EN;
	doESP01OnUDPData = aESP01OnUDPData;
}

uint8_t ESP01SetWIFI(char *aSSID, char *aPASSWORD){
	strncpy(SSID, aSSID, 48);
	strncpy(PASSWORD, aPASSWORD, 32);

	ESP01TASKSTATE = STESP01ATAT;
	lastESP01STATE = ESP01WIFICONNECTING;

	return 1;
}

uint8_t ESP01ConnectUDP(char *aRemoteIP, uint16_t aRemotePort, uint16_t aLocalPort){
	char str[5];

	strncpy(RemoteIP, aRemoteIP, 16);
    RemoteIP[15] = '\0';

	str[4] = aRemotePort%10 + 0x30;
	aRemotePort /= 10;
	str[3] = aRemotePort%10 + 0x30;
	aRemotePort /= 10;
	str[2] = aRemotePort%10 + 0x30;
	aRemotePort /= 10;
	str[1] = aRemotePort%10 + 0x30;
	aRemotePort /= 10;
	str[0] = aRemotePort + 0x30;

	int i = 0;
	int j = 0;
	while(str[i] == '0')
		i++;
	while(i < 5)
		RemotePort[j++] = str[i++];
	RemotePort[j] = '\0';

	str[4] = aLocalPort%10 + 0x30;
	aLocalPort /= 10;
	str[3] = aLocalPort%10 + 0x30;
	aLocalPort /= 10;
	str[2] = aLocalPort%10 + 0x30;
	aLocalPort /= 10;
	str[1] = aLocalPort%10 + 0x30;
	aLocalPort /= 10;
	str[0] = aLocalPort + 0x30;

	while(str[i] == '0')
		i++;
	while(i < 5)
		LocalPort[j++] = str[i++];
	LocalPort[j] = '\0';


    ESP01TASKSTATE = STESP01ATCLOSE;
    lastESP01STATE = ESP01UDPBUSY;

    return 1;
}

void ESP01SetRxByte(uint8_t value){
	bufRX[iRXw++] = value;
	if(iRXw == SIZEBUFRX)
		iRXw = 0;
}

uint8_t ESP01HasByteToTx(){
	if(WAITINGSYMBOL)
		return 0;
	if(iTXr != iTXw)
		return 1;
	return 0;
}


uint8_t ESP01GetTxByte(uint8_t *value){
	*value = bufTX[iTXr++];
	if(iTXr == SIZEBUFTX)
		iTXr = 0;

	if(iTXr == iTXw)
		lastESP01STATE = ESP01UDPREADY;

	return 1;
}

uint8_t ESP01SendUDPData(uint8_t *buf, uint16_t dataLength, uint16_t bufSize){
	char str[3];

	if(dataLength > (SIZEBUFTX - 16))
		return 0;

	if(lastESP01STATE != ESP01UDPREADY)
		return 0;

	ESP01PutAT(_ATCIPSEND);

	aux = dataLength;
	str[2] = aux%10 + 0x30;
	aux /= 10;
	str[1] = aux%10 + 0x30;
	aux /= 10;
	str[0] = aux%10 + 0x30;

	ESP01PutByteOnTx(str[0]);
	ESP01PutByteOnTx(str[1]);
	ESP01PutByteOnTx(str[2]);
	ESP01PutByteOnTx('\r');
	ESP01PutByteOnTx('\n');
	ESP01PutByteOnTx('>');

	for(uint16_t i=0, j=0; i<dataLength; i++){
		bufTX[iTXw++] = buf[j++];
		if(iTXw == SIZEBUFTX)
			iTXw = 0;
		if((buf+j) == (buf+bufSize)){
			buf = buf-bufSize;
			j = 0;
		}
	}

	NEEDWAITSYMBOL = 1;
	WAITINGSYMBOL = 0;

	lastESP01STATE = ESP01UDPBUSY;

	return 1;
}


void ESP01Task(){

	if(SSID[0]=='\0' && ESP01TASKSTATE!=STESP01IDLE)
		ESP01TASKSTATE = STESP01IDLE;

	if(iRXw != iRXr)
		ESP01DecodeAT();

	ESP01DoWifi();

}

void ESP01TimeOut10ms(){
	if(timeOutResponse)
		timeOutResponse--;

	if(timeOutSymbol){
		timeOutSymbol--;
	}

	if(timeOut){
		timeOut--;
		if(!timeOut)
			header = 0;
	}
}

void ESP01AttachOnWIFIConnected(ESP01OnWIFIConnected *aESP01OnWIFIConnected){
	doESP01WIFIConnected = aESP01OnWIFIConnected;
}

void ESP01AttachOnWIFIDisconnected(ESP01OnWIFIDisconnected *aESP01OnWIFIDisconnected){
	doESP01WIFIDisconnected = aESP01OnWIFIDisconnected;
}

void ESP01AttachOnUDPReady(ESP01OnUDPReady *aESP01UDPReady){
	doESP01UDPReady = aESP01UDPReady;
}

ESP01STATE ESP01GetLastSTATE(){
	return lastESP01STATE;
}


static void ESP01PutAT(const char *atCMD){
	int i=0;

	while(atCMD[i]){
		bufTX[iTXw++] = atCMD[i];
		if(iTXw == SIZEBUFTX)
			iTXw = 0;
	}
}

static void ESP01PutByteOnTx(uint8_t value){
	bufTX[iTXw++] = value;
	if(iTXw == SIZEBUFTX)
		iTXw = 0;
}

static void ESP01DoWifi(){
	if(timeOutResponse || ESP01TASKSTATE==STESP01IDLE)
		return;

	switch(ESP01TASKSTATE){
	case STESP01ATAT:
		WAITINGSYMBOL = 0;
		NEEDWAITSYMBOL = 0;
		if(triesAT){
			triesAT--;
			if(!triesAT){
				ESP01TASKSTATE = STESP01HARDRESET;
				doESP01GpioWriteCH_EN(0);
				timeOutResponse = 100;
			}
			else{
				ESP01PutAT(_AT);
				timeOutResponse = 10;
			}
		}
		else{
			ESP01PutAT(_AT);
			timeOutResponse = 10;
			triesAT = 5;
		}
		break;
	case STESP01HARDRESET:
		doESP01GpioWriteCH_EN(1);
		ESP01TASKSTATE = STESP01ATAT;
		timeOutResponse = 100;
		break;
	case STESP01ATCWMODE:
		ESP01PutAT(_ATCWMODE);
		timeOutResponse = 10;
		break;
	case STESP01ATCIPMUX:
		ESP01PutAT(_ATCIPMUX);
		timeOutResponse = 5;
		break;
	case STESP01ATCLOSE:
		lastESP01STATE = ESP01WIFIDISCONNECTED;
		ESP01PutAT(_ATCIPCLOSE);
		timeOutResponse = 10;
		break;
	case STESP01ATCWJAP://"SSID","SSIDPass"
		ESP01PutAT(_ATCWJAP);
		ESP01PutByteOnTx('"');
		for(aux=0; aux<48; aux++){
			if(SSID[aux]=='\0')
				break;
			ESP01PutByteOnTx(SSID[aux]);
		}
		ESP01PutByteOnTx('"');
		ESP01PutByteOnTx(',');
		ESP01PutByteOnTx('"');
		for(aux=0; aux<32; aux++){
			if(PASSWORD[aux]=='\0')
				break;
			ESP01PutByteOnTx(PASSWORD[aux]);
		}
		ESP01PutByteOnTx('"');
		ESP01PutByteOnTx('\r');
		ESP01PutByteOnTx('\n');
		timeOutResponse = 100;
		break;
	case STESP01ATCIFSR:
		ESP01PutAT(_ATCIFSR);
		timeOutResponse = 10;
		break;
	case STESP01ATCIPSTART:
		lastESP01STATE = ESP01UDPBUSY;
		ESP01PutAT(_ATCIPSTART);
		ESP01PutByteOnTx('"');
		ESP01PutAT(_UDP);
		ESP01PutByteOnTx('"');
		ESP01PutByteOnTx(',');
		ESP01PutByteOnTx('"');
		for(aux=0; aux<16; aux++){
			if(RemoteIP[aux] == '\0')
				break;
			ESP01PutByteOnTx(RemoteIP[aux]);
		}
		ESP01PutByteOnTx('"');
		ESP01PutByteOnTx(',');
		for(aux=0; aux<6; aux++){
			if(RemotePort[aux] == '\0')
				break;
			ESP01PutByteOnTx(RemotePort[aux]);
		}
		ESP01PutByteOnTx(',');
		for(aux=0; aux<6; aux++){
			if(LocalPort[aux] == '\0')
				break;
			ESP01PutByteOnTx(LocalPort[aux]);
		}
		ESP01PutByteOnTx('\r');
		ESP01PutByteOnTx('\n');
		timeOutResponse = 50;
		break;
	case STESP01ATCONNECTED:
		break;
	default:
		ESP01TASKSTATE = STESP01ATAT;
	}

}


static uint8_t CmpResponse(const char *str, uint16_t index, uint8_t n){
	uint8_t j;

	for(j=0; j<n; j++){
		if(str[j] != bufRX[index])
			return 0;
		index++;
		index &= (SIZEBUFRX - 1);
	}
	return 1;
}


static void ESP01DecodeAT(){
	uint16_t index;

	index = iRXw;
	index &= (SIZEBUFRX - 1);

	while(iRXr != index){
		switch(header){
			case 0:
				if(bufRX[iRXr] == 'A')
					header = 1;
				if(bufRX[iRXr] == 'W'){
					header = 10;
					iResponse = iRXr;
				}
				if(bufRX[iRXr] == 'l')
					header = 20;
				if(bufRX[iRXr] == '+'){
					header = 30;
					iResponse = iRXr;
				}
				if(bufRX[iRXr] == '\r')
					header = 40;
				if(bufRX[iRXr]=='C' || bufRX[iRXr]=='D'){
					header = 50;
					iResponse = iRXr;
				}
				if(bufRX[iRXr] == '>'){
					timeOutSymbol = 0;
					WAITINGSYMBOL = 0;
				}
				timeOut = 100;
				break;
			case 1:
				if(bufRX[iRXr] == 'T')
					header = 2;
				else
					header = 0;
				break;
			case 2:
				if(bufRX[iRXr] == '\n')
					header = 0;
				break;
			case 10:
				if(bufRX[iRXr] == 'I')
					header = 11;
				else
					header = 0;
				break;
			case 11:
				if(bufRX[iRXr] == 'F')
					header = 12;
				else
					header = 0;
				break;
			case 12:
				if(bufRX[iRXr] == 'I')
					header = 13;
				else
					header = 21;
				break;
			case 13:
				if(bufRX[iRXr] == '\n'){
					header = 0;
					if(CmpResponse(_WIFIGotIP, iResponse, 11)==1){
						if(ESP01TASKSTATE == STESP01ATCWJAP){
							ESP01TASKSTATE = STESP01ATCIFSR;
							timeOutResponse = 20;
						}
					}
					if(CmpResponse(_WIFIDisconnected, iResponse, 17)==1){
						ESP01TASKSTATE = STESP01ATAT;
						lastESP01STATE =  ESP01WIFIDISCONNECTED;
						if(doESP01WIFIDisconnected)
							doESP01WIFIDisconnected();
					}
					if(CmpResponse(_WIFIDisconnect, iResponse, 15)==1){
						if(ESP01TASKSTATE != STESP01ATCWJAP){
							ESP01TASKSTATE = STESP01ATAT;
							lastESP01STATE = ESP01WIFIDISCONNECTED;
							if(doESP01WIFIDisconnected)
								doESP01WIFIDisconnected();
						}
					}
				}
				break;
			case 20:
				if(bufRX[iRXr] == 'i')
					header = 21;
				else
					header = 0;
				break;
			case 21:
				if(bufRX[iRXr] == '\n')
					header = 0;
				break;
			case 30:
				if(bufRX[iRXr] == '\n'){
					header = 0;
					if(ESP01TASKSTATE  == STESP01ATCIFSR){
						if(CmpResponse(_CIFSRSTAIP, iResponse, 12) == 1){
						}
						if(CmpResponse(_CIFSRSTAMAC, iResponse, 13) == 1){
							ESP01TASKSTATE = STESP01ATCLOSE;
							timeOutResponse = 0;
							lastESP01STATE = ESP01WIFICONNECTED;
							triesAT = 0;
							if(doESP01WIFIConnected)
								doESP01WIFIConnected();
						}
					}
				}
				break;
			case 40:
				if(bufRX[iRXr] == '\n')
					header = 41;
				else
					header = 0;
				break;
			case 41:
				if(bufRX[iRXr] == '+')
					header = 43;
				else{
					header = 42;
					iResponse = iRXr;
				}
				break;
			case 42:
				if(bufRX[iRXr] == '\n'){
					header = 0;
					if(CmpResponse(_OK, iResponse, 2) == 1){
						if(ESP01TASKSTATE == STESP01ATAT){
							ESP01TASKSTATE = STESP01ATCWMODE;
							timeOutResponse = 0;
						}
						if(ESP01TASKSTATE == STESP01ATCWMODE){
							timeOutResponse = 0;
							ESP01TASKSTATE = STESP01ATCIPMUX;
						}
						if(ESP01TASKSTATE == STESP01ATCIPMUX){
							ESP01TASKSTATE = STESP01ATCLOSE;
							timeOutResponse = 0;
						}
					}
					if(CmpResponse(_BUSY, iResponse, 4) == 1){
						ESP01TASKSTATE = STESP01ATAT;
						triesAT = 1;
						lastESP01STATE = ESP01WIFIDISCONNECTED;
						timeOutResponse = 0;
					}
					if(CmpResponse(_SENDOK, iResponse, 7) == 1){
						timeOutResponse = 0;
					}
				}
				break;
			case 43:
				if(bufRX[iRXr] == 'I')
					header = 44;
				else
					header = 42;
				break;
			case 44:
				if(bufRX[iRXr] == 'P')
					header = 45;
				else
					header = 42;
				break;
			case 45:
				if(bufRX[iRXr] == 'D')
					header = 46;
				else
					header = 42;
				break;
			case 46:
				if(bufRX[iRXr] == ','){
					header = 47;
					ipdBytes = 0;
				}
				else
					header = 42;
				break;
			case 47:
				if(bufRX[iRXr] == ':')
					header = 48;
				else{
					ipdBytes *= 10;
					ipdBytes += (bufRX[iRXr] - 0x30);
					nBytesDatagram = ipdBytes;
					iBufDatagram = 0;
				}
				break;
			case 48:
				ipdBytes--;
				if(ipdBytes == 0){
					header = 0;
					if(doESP01OnUDPData != NULL)
						doESP01OnUDPData(bufDatagram, nBytesDatagram);
				}
				else{
					bufDatagram[iBufDatagram++] = bufRX[iRXr];
					if(iBufDatagram == SIZEBUFDATAGRAM)
						iBufDatagram = 0;
				}
				break;
			case 50:
				if(bufRX[iRXr] == '\n'){
					header = 0;
					if(CmpResponse(_CONNECT, iResponse, 7) == 1){
						if(ESP01TASKSTATE == STESP01ATCIPSTART){
							ESP01TASKSTATE = STESP01ATCONNECTED;
							lastESP01STATE = ESP01UDPREADY;
							if(doESP01UDPReady)
								doESP01UDPReady();
							timeOutResponse = 0;
						}
					}
					if(CmpResponse(_DISCONNECTED, iResponse, 12) == 1){
						ESP01TASKSTATE = STESP01ATAT;
						timeOutResponse = 0;
						lastESP01STATE =  ESP01WIFIDISCONNECTED;
						if(doESP01WIFIDisconnected)
							doESP01WIFIDisconnected();
					}
					if(CmpResponse(_CLOSED, iResponse, 6) == 1){
						if(ESP01TASKSTATE == STESP01ATCLOSE){
							ESP01TASKSTATE = STESP01ATCIPSTART;
							timeOutResponse = 0;
						}
					}
				}
				break;
			default:
				header = 0;
		}
	}
}



