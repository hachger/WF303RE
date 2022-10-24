/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "utils.h"
#include <string.h>
#include "ESP01.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define IS100MS			flag1.bit.b0
#define ANALOGREADY		flag1.bit.b1
#define ESP01DEBUG		flag1.bit.b2
#define CONTAVERAGE		flag1.bit.b3

#define LEDIDLE				0xE8000000
#define LEDTXANALOG			0xE8A00000
#define LEDESP01DBG			0xEAEAEAEA
#define LEDWIFIDISCONNECTED	0x0000EE00
#define LEDWIFICONNECTED	0x0000EEA0

#define ADCDATASIZE		256

#define RXBUFSIZE		256
#define TXBUFSIZE		256
#define RXESP01BUFSIZE	128
#define TXESP01BUFSIZE	128

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
_uFlag flag1;
_uWork w, w1;
_sRx RX, RXESP01;
_sTx TX, TXESP01;

char strAux[32], ssid[48], password[32], remoteIP[16];
uint16_t remotePort, localPort;

uint8_t bufRX[RXBUFSIZE], bufRXESP01[RXESP01BUFSIZE], auxDataRXESP01;
uint8_t bufTX[TXBUFSIZE], bufTXESP01[TXESP01BUFSIZE];
uint16_t timeOutSendAlive;


uint16_t ADCData[ADCDATASIZE][8];
uint8_t indexADCData, indexADCData10ms, iADCDataReady;
uint16_t ADC3DataAux;
uint32_t sumADCData10ms[8], sumADCDataLock[8];
uint16_t timeOutAnalog, timeOutAnalogAux;
uint8_t n10msData, stateSendSamples, numSendSamples, cksSendSamples;

uint32_t maskLedStatus, ledStatus;
uint32_t lastTickValue;
uint8_t timeOut100ms;

uint16_t timeOutTestMotors;

uint8_t countPlus, timeOutPlus;


const char FIRMWARE[] = "20221017_v01b01";
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_ADC3_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
void DecodeHeader(_sRx *RX);
uint8_t PutBufOnTx(_sTx *TX, uint8_t *buf, uint8_t length);
uint8_t PutByteOnTx(_sTx *TX, uint8_t value);
uint8_t PutHeaderOnTx(_sTx *TX, uint8_t id, uint8_t lCmd);
//void CalcAndPutCksOnTx(_sTx *TX);
uint8_t PutStrOntx(_sTx *TX, const char *str);
uint8_t GetByteFromRx(_sRx *RX, int8_t pre, int8_t pos);
void GetBufFromRx(_sRx *RX, uint8_t *buf, int length);
void DecodeCmd(_sRx *RX, _sTx *TX);
void Do100ms();

void ESP01ChEN(uint32_t value);
void ESP01DatagramReady(uint8_t *buf, uint16_t length);
void ESP01WIFIConnected();
void ESP01WIFIDisconnected();
void ESP01UDPReady();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	if(hadc->Instance == ADC1){
		ADCData[indexADCData][7] = ADC3DataAux;
		sumADCData10ms[0] += ADCData[indexADCData][0];
		sumADCData10ms[1] += ADCData[indexADCData][1];
		sumADCData10ms[2] += ADCData[indexADCData][2];
		sumADCData10ms[3] += ADCData[indexADCData][3];
		sumADCData10ms[4] += ADCData[indexADCData][4];
		sumADCData10ms[5] += ADCData[indexADCData][5];
		sumADCData10ms[6] += ADCData[indexADCData][6];
		sumADCData10ms[7] += ADCData[indexADCData][7];
		indexADCData++;
		indexADCData &= (ADCDATASIZE-1);
		if(timeOutAnalogAux){
			timeOutAnalogAux--;
			if(!timeOutAnalogAux){
				timeOutAnalogAux = timeOutAnalog;
				iADCDataReady = indexADCData;
				memcpy(sumADCDataLock, sumADCData10ms, sizeof(sumADCDataLock));
				ANALOGREADY = 1;
			}
		}
	}
	if(hadc->Instance == ADC3){
		ADC3DataAux = HAL_ADC_GetValue(&hadc3);
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
//	if(timeOutAnalogAux){
//		timeOutAnalogAux--;
//		if(!timeOutAnalogAux){
//			timeOutAnalogAux = timeOutAnalog;
//			iADCDataReady = indexADCData;
//			memcpy(sumADCDataLock, sumADCData10ms, sizeof(sumADCDataLock));
//			ANALOGREADY = 1;
//		}
//	}

	sumADCData10ms[0] -= ADCData[indexADCData10ms][0];
	sumADCData10ms[1] -= ADCData[indexADCData10ms][1];
	sumADCData10ms[2] -= ADCData[indexADCData10ms][2];
	sumADCData10ms[3] -= ADCData[indexADCData10ms][3];
	sumADCData10ms[4] -= ADCData[indexADCData10ms][4];
	sumADCData10ms[5] -= ADCData[indexADCData10ms][5];
	sumADCData10ms[6] -= ADCData[indexADCData10ms][6];
	sumADCData10ms[7] -= ADCData[indexADCData10ms][7];
	indexADCData10ms++;
	indexADCData10ms &= (ADCDATASIZE - 1);
	HAL_ADC_Start_IT(&hadc3);
	HAL_ADC_Start(&hadc2);
	HAL_ADCEx_MultiModeStart_DMA(&hadc1, (uint32_t *)&ADCData[indexADCData], 4);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart->Instance == USART2){
		if(ESP01DEBUG){
			if(RX.buf[RX.iw] == '+'){
				if(countPlus == 0)
					timeOutPlus = 3;
				countPlus++;
				if(countPlus==3 && timeOutPlus!=0){
					ESP01DEBUG = 0;
					ledStatus = LEDIDLE;
				}
			}
			else
				countPlus = 0;

			PutByteOnTx(&TXESP01, RX.buf[RX.iw]);
		}

		RX.iw++;
		RX.iw &= RX.maskSize;
		HAL_UART_Receive_IT(&huart2, &RX.buf[RX.iw], 1);
	}
	if(huart->Instance == USART3){
		if(ESP01DEBUG){
			PutByteOnTx(&TX, auxDataRXESP01);
		}
		ESP01SetRxByte(auxDataRXESP01);
		HAL_UART_Receive_IT(&huart3, &auxDataRXESP01, 1);
//		RXESP01.iw++;
//		RXESP01.iw &= RXESP01.maskSize;
//		HAL_UART_Receive_IT(&huart3, &RXESP01.buf[RXESP01.iw], 1);
	}
}


void DecodeHeader(_sRx *RX){
    uint8_t i;

    i = RX->iw;

    while(RX->ir != i){
       switch(RX->header){
        case 0:
            if(RX->buf[RX->ir] == 'U'){
                RX->header = 1;
                RX->timeout = 5;
            }
            break;
        case 1:
            if(RX->buf[RX->ir] == 'N')
                RX->header = 2;
            else{
                RX->header = 0;
                RX->ir--;
            }
            break;
        case 2:
            if(RX->buf[RX->ir] == 'E')
                RX->header = 3;
            else{
                RX->header = 0;
                RX->ir--;
            }
            break;
        case 3:
            if(RX->buf[RX->ir] == 'R')
                RX->header = 4;
            else{
                RX->header = 0;
                RX->ir--;
            }
            break;
        case 4:
            RX->nbytes = RX->buf[RX->ir];
            RX->header = 5;
            break;
        case 5:
            if(RX->buf[RX->ir] == ':'){
                RX->header = 6;
                RX->iData = RX->ir + 1;
                RX->iData &= RX->maskSize;
                RX->cks = 'U' ^ 'N' ^ 'E' ^ 'R' ^ ':' ^ RX->nbytes;
            }
            else{
                RX->header = 0;
                RX->ir--;
            }
            break;
        case 6:
            RX->nbytes--;
            if(RX->nbytes > 0){
                RX->cks ^= RX->buf[RX->ir];
            }
            else{
                RX->header = 0;
                if(RX->cks == RX->buf[RX->ir])
                    RX->ISCMD = 1;
            }
            break;
        default:
            RX->header = 0;
        }

        RX->ir &= RX->maskSize;
        RX->ir++;
        RX->ir &= RX->maskSize;
    }

}

uint8_t PutBufOnTx(_sTx *TX, uint8_t *buf, uint8_t length){
	uint8_t i;

    for(i=0; i<length; i++){
        TX->cks ^= buf[i];
        TX->buf[TX->iw++] = buf[i];
        TX->iw &= TX->maskSize;
    }

    return TX->cks;
}

uint8_t PutByteOnTx(_sTx *TX, uint8_t value){
	TX->cks ^= value;
	TX->buf[TX->iw++] = value;
    TX->iw &= TX->maskSize;

    return TX->cks;
}

uint8_t PutHeaderOnTx(_sTx *TX, uint8_t id, uint8_t lCmd){
    TX->buf[TX->iw++] = 'U';
    TX->iw &= TX->maskSize;
    TX->buf[TX->iw++] = 'N';
    TX->iw &= TX->maskSize;
    TX->buf[TX->iw++] = 'E';
    TX->iw &= TX->maskSize;
    TX->buf[TX->iw++] = 'R';
    TX->iw &= TX->maskSize;
    TX->buf[TX->iw++] = lCmd + 1;//id + payload + cks
    TX->iw &= TX->maskSize;
    TX->buf[TX->iw++] = ':';
    TX->iw &= TX->maskSize;
    TX->buf[TX->iw++] = id;
    TX->iw &= TX->maskSize;

    TX->cks ^= (lCmd + 1);
    TX->cks ^= ('U' ^ 'N' ^ 'E' ^ 'R' ^ ':' ^ id);

    return TX->cks;
}

//void CalcAndPutCksOnTx(_sTx *TX){
//    uint8_t cks, i;
//
//    cks = 0;
//    i = TX->length + 6;
//    i = TX->iw-i;
//    i &= TX->maskSize;
//    while(i != TX->iw){
//        cks ^= TX->buf[i++];
//        i &= TX->maskSize;
//    }
//
//    TX->buf[TX->iw++] = cks;
//    TX->iw &= TX->maskSize;
//}

uint8_t PutStrOntx(_sTx *TX, const char *str){
    uint8_t i = 0;

    while(str[i]){
        TX->cks |= str[i];
    	TX->buf[TX->iw++] = str[i++];
        TX->iw &= TX->maskSize;

    }

    return TX->cks;
}

uint8_t GetByteFromRx(_sRx *RX, int8_t pre, int8_t pos){
    uint8_t aux;

    RX->iData += pre;
    RX->iData &= RX->maskSize;
    aux = RX->buf[RX->iData];
    RX->iData += pos;
    RX->iData &= RX->maskSize;

    return aux;
}

void GetBufFromRx(_sRx *RX, uint8_t *buf, int length){
    for(int i=0; i<length; i++){
        buf[i] = RX->buf[RX->iData];
        RX->iData++;
        RX->iData &= RX->maskSize;
    }
}

void DecodeCmd(_sRx *RX, _sTx *TX){
	RX->ISCMD = 0;

    TX->cks = 0;
    switch (RX->buf[RX->iData])
    {
    case 0xA0://READ LAST ANALOG INPUTS
		ledStatus = LEDIDLE;
		w.u8[0] = indexADCData;
		w.u8[0]--;
		w.u8[0] &= (ADCDATASIZE - 1);
		PutHeaderOnTx(TX, 0xA0, 2*16+1);
		PutBufOnTx(TX, (uint8_t *)&ADCData[w.u8[0]], 16);
		for(w.u8[2] = 0; w.u8[2]<8; w.u8[2]++){
			w.u16[0] = sumADCData10ms[w.u8[2]]/40;
			PutByteOnTx(TX, w.u8[0]);
			PutByteOnTx(TX, w.u8[1]);
		}
		PutByteOnTx(TX, TX->cks);
        break;
    case 0xA1://READ Continuous every Nx10ms Nx5 Samples and Average
    	w.u8[0] =  GetByteFromRx(RX, 1, 0);
    	if(w.u8[0] != 0){
    		w.u8[0] /= 10;
    		if(w.u8[0] == 0)
    			w.u8[0] = 1;
    		if(w.u8[0] > 6)
    			w.u8[0] = 6;
    		n10msData = w.u8[0];
    		timeOutAnalog = n10msData*40;
    		timeOutAnalogAux = timeOutAnalog;
    		ledStatus = LEDTXANALOG;
    		stateSendSamples = 0;
    		CONTAVERAGE = 0;
    	}
    	else{
    		timeOutAnalog = 0;
    		timeOutAnalogAux = 0;
    		ledStatus = LEDIDLE;
    	}
    	PutHeaderOnTx(TX, 0xA1, 2);
    	PutByteOnTx(TX, 0x0D);
    	PutByteOnTx(TX, TX->cks);
    	break;
    case 0xA2://Read Continuous Average every N ms
    	w.u8[0] =  GetByteFromRx(RX, 1, 0);
    	if(w.u8[0] != 0){
    		if(w.u8[0] < 10)
    			w.u8[0] = 10;
    		timeOutAnalog = w.u8[0]*4;
    		timeOutAnalogAux = timeOutAnalog;
    		ledStatus = LEDTXANALOG;
    		CONTAVERAGE = 1;
    	}
    	else{
    		timeOutAnalog = 0;
    		timeOutAnalogAux = 0;
    		ledStatus = LEDIDLE;
    	}
    	PutHeaderOnTx(TX, 0xA2, 2);
    	PutByteOnTx(TX, 0x0D);
    	PutByteOnTx(TX, TX->cks);
    	break;
    case 0xA3://SET Motor Period
    	w.u8[0] = GetByteFromRx(RX, 1, 0);
    	w.u8[1] = GetByteFromRx(RX, 1, 0);
    	if(w.u16[0] < 100){
    		w.u16[0] = 100;
    	}
    	if(w.u16[0] > 9000){
    		w.u16[0] = 9000;
    	}

		HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
		HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
		HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);
		HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_4);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);

		__HAL_TIM_SET_AUTORELOAD(&htim3, w.u16[0]);

		HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
		HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
		HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
		HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

		PutHeaderOnTx(TX, 0xA3, 3);
    	PutByteOnTx(TX, w.u8[0]);
    	PutByteOnTx(TX, w.u8[1]);
    	PutByteOnTx(TX, TX->cks);
    	break;
    case 0xA4://Test Motors
    	w.i8[0] = GetByteFromRx(RX, 1, 0);
    	w.i8[1] = GetByteFromRx(RX, 1, 0);
    	w.i8[2] = GetByteFromRx(RX, 1, 0);
    	w.i8[3] = GetByteFromRx(RX, 1, 0);
    	if(w.i8[0] > 100)
    		w.i8[0] = 90;
    	if(w.i8[0] < -100)
    		w.i8[0] = -90;
    	if(w.i8[1] > 100)
    		w.i8[1] = 90;
    	if(w.i8[1] < -100)
    		w.i8[1] = -90;
    	w1.i16[0] = (w.i8[0]*100)/__HAL_TIM_GET_AUTORELOAD(&htim3);
    	w1.i16[1] = (w.i8[1]*100)/__HAL_TIM_GET_AUTORELOAD(&htim3);
    	if(w.u16[1] > 100)
    		w.u16[1] = 100;
    	timeOutTestMotors = w.u16[1];
    	if(w1.i16[0] < 0){
    		w1.i16[0] *= -1;
    		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, w1.i16[0]);
    		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
    	}
    	else{
    		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
    		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, w1.i16[0]);
    	}
    	if(w1.i16[1] < 0){
    		w1.i16[1] *= -1;
    		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, w1.i16[1]);
    		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
    	}
    	else{
    		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
    		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, w1.i16[1]);
    	}
    	PutHeaderOnTx(TX, 0xA4, 5);
    	PutByteOnTx(TX, w.u8[0]);
    	PutByteOnTx(TX, w.u8[1]);
    	PutByteOnTx(TX, w.u8[2]);
    	PutByteOnTx(TX, w.u8[3]);
    	PutByteOnTx(TX, TX->cks);
    	break;
    case 0xB0://SET transparent ESP01
   		ESP01DEBUG  = 1;
   		ledStatus = LEDESP01DBG;
   		countPlus = 0;
    	break;
    case 0xB1://SET WIFI SSID and PASSWORD
    	for(w.u8[0] = 0; w.u8[0] < 48; w.u8[0]++){
    		w.u8[1] = GetByteFromRx(RX, 1, 0);
    		ssid[w.u8[0]] = w.u8[1];
    		if(w.u8[1] == '\0')
    			break;
    	}
		ssid[47] = '\0';
    	for(w.u8[0] = 0; w.u8[0] < 32; w.u8[0]++){
    		w.u8[1] = GetByteFromRx(RX, 1, 0);
    		password[w.u8[0]] = w.u8[1];
    		if(w.u8[1] == '\0')
    			break;
    	}
		password[31] = '\0';
    	ESP01SetWIFI(ssid, password);
    	PutHeaderOnTx(TX, 0xB1, 48+32+1);
    	PutBufOnTx(TX, (uint8_t *)ssid, 48);
    	PutBufOnTx(TX, (uint8_t *)password, 32);
    	PutByteOnTx(TX, TX->cks);
    	break;
    case 0xB2://SET RemoteIP, RemotePort and LocalPort
    	if(ESP01GetLastSTATE() == ESP01WIFIDISCONNECTED){
    		PutHeaderOnTx(TX, 0xB2, 2);
    		PutByteOnTx(TX, 0xFF);
    		PutByteOnTx(TX, TX->cks);
    		break;
    	}
    	for(w.u8[0] = 0; w.u8[0] < 16; w.u8[0]++){
    		w.u8[1] = GetByteFromRx(RX, 1, 0);
    		remoteIP[w.u8[0]] = w.u8[1];
    		if(w.u8[1] == '\0')
    			break;
    	}
		remoteIP[15] = '\0';
		w.u8[0] = GetByteFromRx(RX, 1, 0);
		w.u8[1] = GetByteFromRx(RX, 1, 0);
		w.u8[2] = GetByteFromRx(RX, 1, 0);
		w.u8[3] = GetByteFromRx(RX, 1, 0);
		remotePort = w.u16[0];
		localPort = w.u16[1];
		ESP01ConnectUDP(remoteIP, remotePort, localPort);
		PutHeaderOnTx(TX, 0xB2, 22);
		PutByteOnTx(TX, 0x0D);
		PutBufOnTx(TX, (uint8_t *)remoteIP, 16);
		PutBufOnTx(TX, (uint8_t *)&remotePort, 2);
		PutBufOnTx(TX, (uint8_t *)&localPort, 2);
		PutByteOnTx(TX, TX->cks);
    	break;
    case 0xF0://ALIVE
    	PutHeaderOnTx(TX, 0xF0, 2);
        PutByteOnTx(TX, 0x0D);
        PutByteOnTx(TX, TX->cks);
        break;
    case 0xF1://FIRMWARE
        PutStrOntx(TX, "+&DBG");
        PutStrOntx(TX, FIRMWARE);
        PutByteOnTx(TX, '\n');
        break;
    default:
        PutHeaderOnTx(TX, RX->buf[RX->iData], 2);
        PutByteOnTx(TX, 0xFF);
        PutByteOnTx(TX, TX->cks);
        break;
    }

}

void Do100ms(){
	timeOut100ms = 10;

	if(maskLedStatus & ledStatus)
		HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_RESET);
	maskLedStatus >>= 1;
	if(!maskLedStatus)
		maskLedStatus = 0x80000000;

	if(timeOutTestMotors){
		timeOutTestMotors--;
		if(!timeOutTestMotors){
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
		}
	}

	if(timeOutSendAlive){
		timeOutSendAlive--;
		if(!timeOutSendAlive){
			timeOutSendAlive = 200;
			TX.cks = 0;
			PutHeaderOnTx(&TX, 0xF0, 2);
			PutByteOnTx(&TX, 0x0D);
			PutByteOnTx(&TX, TX.cks);
			TXESP01.cks = 0;
			PutHeaderOnTx(&TXESP01, 0xF0, 2);
			PutByteOnTx(&TXESP01, 0x0D);
			PutByteOnTx(&TXESP01, TXESP01.cks);
		}
	}
}


void ESP01ChEN(uint32_t value){
	HAL_GPIO_WritePin(CH_ENA_GPIO_Port, CH_ENA_Pin, value);
}

void ESP01DatagramReady(uint8_t *buf, uint16_t length){
	for(uint16_t i=0; i<length; i++)
		RXESP01.buf[RXESP01.iw++] = buf[i];
}

void ESP01WIFIConnected(){
	PutStrOntx(&TX, "+&DBGWIFI Connected\n");
}

void ESP01WIFIDisconnected(){
	PutStrOntx(&TX, "+&DBGWIFI Disconnected\n");
}

void ESP01UDPReady(){
	PutStrOntx(&TX, "+&DBGUDP Connected\n");
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	flag1.byte = 0;
	RX.buf = bufRX;
	RX.header = 0;
	RX.ir = 0;
	RX.iw = 0;
	RX.maskSize = RXBUFSIZE - 1;

	TX.buf = bufTX;
	TX.ir = 0;
	TX.iw = 0;
	TX.maskSize = TXBUFSIZE - 1;

	RXESP01.buf = bufRXESP01;
	RXESP01.header = 0;
	RXESP01.ir = 0;
	RXESP01.iw = 0;
	RXESP01.maskSize = RXESP01BUFSIZE - 1;

	TXESP01.buf = bufTXESP01;
	TXESP01.ir = 0;
	TXESP01.iw = 0;
	TXESP01.maskSize = TXESP01BUFSIZE - 1;

	timeOut100ms = 10;
	ledStatus = LEDIDLE;

	indexADCData = 0;
	indexADCData10ms -= 40;
	indexADCData10ms &= (ADCDATASIZE - 1);

	timeOutTestMotors = 0;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  indexADCData = 0;
  HAL_TIM_Base_Start_IT(&htim1);
  HAL_UART_Receive_IT(&huart2, &RX.buf[RX.iw], 1);
  HAL_UART_Receive_IT(&huart3, &auxDataRXESP01, 1);

  HAL_TIM_Base_Start(&htim3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  HAL_GPIO_WritePin(CH_ENA_GPIO_Port, CH_ENA_Pin, GPIO_PIN_SET);
  ESP01Init((ESP01GpioWriteCH_EN *)&ESP01ChEN, (ESP01OnUDPData *)&ESP01DatagramReady);
  ESP01AttachOnWIFIConnected((ESP01OnWIFIConnected *)&ESP01WIFIConnected);
  ESP01AttachOnWIFIDisconnected((ESP01OnWIFIDisconnected *)&ESP01WIFIDisconnected);
  ESP01AttachOnUDPReady((ESP01OnUDPReady *)&ESP01UDPReady);


  lastTickValue = HAL_GetTick();

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if(!timeOut100ms)
		  Do100ms();

	  if((HAL_GetTick()-lastTickValue) >= 10){
		  lastTickValue = HAL_GetTick();

		  if(RX.header){
			  RX.timeout--;
			  if(!RX.timeout)
				  RX.header = 0;
		  }

		  if(timeOut100ms)
			  timeOut100ms--;

		  if(timeOutPlus){
			  timeOutPlus--;
			  if(!timeOutPlus)
				  countPlus = 0;
		  }


	  }

	  if(RX.ISCMD)
		  DecodeCmd(&RX, &TX);

	  if(RX.ir != RX.iw)
		  DecodeHeader(&RX);

	  if(RXESP01.ISCMD)
		  DecodeCmd(&RXESP01, &TXESP01);

	  if(RXESP01.ir != RXESP01.iw)
		  DecodeHeader(&RXESP01);

	  if(ANALOGREADY){
		  ANALOGREADY = 0;
		  if(CONTAVERAGE){
			  TX.cks = 0;
			  PutHeaderOnTx(&TX, 0xA2, 16+2);
			  PutByteOnTx(&TX, 0x0A);
			  for(w.u8[2] = 0; w.u8[2]<8; w.u8[2]++){
				  w.u16[0] = sumADCDataLock[w.u8[2]]/40;
				  PutByteOnTx(&TX, w.u8[0]);
				  PutByteOnTx(&TX, w.u8[1]);
			  }
			  PutByteOnTx(&TX, TX.cks);
		  }
		  else{
			  TX.cks = 0;
			  w.u8[3] = iADCDataReady;
			  w.u8[3] -= (n10msData*40);
			  w.u8[3] &= (ADCDATASIZE - 1);
			  w.u8[2] = n10msData*5 + 1;
			  w.u8[2] *= 16;
			  PutHeaderOnTx(&TX, 0xA1, w.u8[2]+3);
			  PutByteOnTx(&TX, 0x0A);
			  w.u8[2] /= 16;
			  w.u8[2]--;
			  PutByteOnTx(&TX, w.u8[2]);
			  while(w.u8[2]){
				  PutBufOnTx(&TX, (uint8_t *)&ADCData[w.u8[3]], 16);
				  w.u8[3] += 8;
				  w.u8[3] &= (ADCDATASIZE-1);
				  w.u8[2]--;
			  }
			  for(w.u8[2] = 0; w.u8[2]<8; w.u8[2]++){
				  w.u16[0] = sumADCDataLock[w.u8[2]]/40;
				  PutByteOnTx(&TX, w.u8[0]);
				  PutByteOnTx(&TX, w.u8[1]);
			  }
			  PutByteOnTx(&TX, TX.cks);
		  }
	  }

	  if(TX.ir != TX.iw){
		  if(__HAL_UART_GET_FLAG(&huart2, UART_FLAG_TXE)){
			  huart2.Instance->TDR = TX.buf[TX.ir++];
			  TX.ir &= TX.maskSize;
		  }
	  }

	  if(ESP01DEBUG){
		  if(TXESP01.ir != TXESP01.iw){
			  if(__HAL_UART_GET_FLAG(&huart3, UART_FLAG_TXE)){
				  huart3.Instance->TDR = TXESP01.buf[TXESP01.ir++];
				  TXESP01.ir &= TXESP01.maskSize;
			  }
		  }
	  }
	  else{
		  if(TXESP01.iw != TXESP01.ir){
			  w.u16[0] = TXESP01.iw - TXESP01.ir;
			  if(ESP01SendUDPData(&TXESP01.buf[TXESP01.ir], w.u16[0], TXESP01BUFSIZE))
				  TXESP01.iw = TXESP01.ir;
		  }

		  if(ESP01HasByteToTx()){
			  if(__HAL_UART_GET_FLAG(&huart3, UART_FLAG_TXE)){
				  if(ESP01GetTxByte(&w.u8[0]))
					  huart3.Instance->TDR = w.u8[0];
			  }
		  }
	  }


	  ESP01Task();

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_USART3
                              |RCC_PERIPHCLK_TIM1|RCC_PERIPHCLK_ADC12
                              |RCC_PERIPHCLK_ADC34|RCC_PERIPHCLK_TIM34;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV6;
  PeriphClkInit.Adc34ClockSelection = RCC_ADC34PLLCLK_DIV6;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  PeriphClkInit.Tim34ClockSelection = RCC_TIM34CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 4;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_DUALMODE_REGSIMULT;
  multimode.DMAAccessMode = ADC_DMAACCESSMODE_12_10_BITS;
  multimode.TwoSamplingDelay = ADC_TWOSAMPLINGDELAY_1CYCLE;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 4;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_VREFINT;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */

  /** Common config
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DMAContinuousRequests = DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc3.Init.LowPowerAutoWait = DISABLE;
  hadc3.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc3, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 18000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 99;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 9000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CH_ENA_GPIO_Port, CH_ENA_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD1_Pin */
  GPIO_InitStruct.Pin = LD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CH_ENA_Pin */
  GPIO_InitStruct.Pin = CH_ENA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CH_ENA_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
