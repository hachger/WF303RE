/*
 * utils.h
 *
 *  Created on: Oct 17, 2022
 *      Author: German
 */

#ifndef UTILS_H_
#define UTILS_H_

/*
 * PA0 = A0
 * PA1 = A1
 * PA4 = A2
 * PB0 = A3
 * PC1 = A4
 * PC0 = A5
 * PC3 = A6
 * PC2 = A7
 *
 * PC6 = TIM3_CH1
 * PC7 = TIM3_CH2
 * PC8 = TIM3_CH3
 * PC9 = TIM3_CH4
 *
 * PC11 = USART3_RX
 * PC10 = USART3_TX
 * PC12 = CH_ENA
 */

typedef struct {
    uint8_t 	*buf;
    uint8_t 	header;
    uint16_t 	iw;
    uint16_t 	ir;
    uint16_t 	iData;
    uint8_t 	timeout;
    uint8_t 	cks;
    uint16_t 	nbytes;
    uint16_t 	maskSize;
    uint8_t 	ISCMD;
}_sRx;

typedef struct {
    uint8_t		*buf;
    uint16_t 	iw;
    uint16_t 	ir;
    uint8_t 	cks;
    uint16_t 	length;
    uint16_t 	maskSize;
}_sTx;

typedef union{
    struct{
        uint8_t b0:1;
        uint8_t b1:1;
        uint8_t b2:1;
        uint8_t b3:1;
        uint8_t b4:1;
        uint8_t b5:1;
        uint8_t b6:1;
        uint8_t b7:1;
    }bit;
    uint8_t byte;
}_uFlag;

typedef union{
    uint8_t     u8[4];
    int8_t      i8[4];
    uint16_t    u16[2];
    int16_t     i16[2];
    uint32_t    u32;
    int32_t     i32;
    float       f;
}_uWork;


static void IntToStr(int32_t value, char *str, int digits, char fillChar){
    uint8_t i, j, k, s;

    if(value == 0){
        for(i=0; i<digits-1 && digits; i++)
            str[i] = fillChar;
        str[i++] = '0';
        str[i] = '\0';
        return;
    }

    s = 0;
    i = 0;
    if(value < 0){
        s = 1;
        value *= -1;
        if(digits)
            digits--;
    }
    i = 0;
    while(value || digits){
        if(value)
            str[i] = value%10 + 0x30;
        else
            str[i] = fillChar;
        value /= 10;
        if(digits)
            digits--;
        i++;
    }
    if(s == 1)
        str[i++] = '-';
    for(j=i-1, k=0; k<j; j--, k++){
        s = str[j];
        str[j] = str[k];
        str[k] = s;
    }
    str[i]='\0';
}



#endif /* UTILS_H_ */
