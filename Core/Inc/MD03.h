#ifndef MD03_INC
#define MD03_INC
#include "stm32f4xx.h"


void MD03SetMotor(USART_TypeDef*,uint8_t,int);
void MD03SetMotorDeg(USART_TypeDef*,uint8_t,int,int);
void MD03SetMotorDegSpd(USART_TypeDef*,uint8_t,int,int);
void MD03SetMotorSpd(USART_TypeDef*,uint8_t,int);

void MD03SetPIDDeg(USART_TypeDef*,uint8_t,float,float,float);
void MD03SetPIDSpd(USART_TypeDef*,uint8_t,float,float,float);
void MD03SetEncInv(USART_TypeDef*,uint8_t,uint8_t);

#endif
