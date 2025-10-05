#include"servoForSTM.h"

void servo22SetPulse(USART_TypeDef* USARTx_2,uint8_t id,uint8_t port,int pulse){//500<pulse<2500

    uint8_t send;
    uint8_t sum=6;

    if(pulse> 4095)pulse=4095;
    if(pulse< 0   )pulse=0;

    send=0x80 | (0b000<<4) | (id & 0xF);
    sum+=send;
    while ( (USARTx_2->SR & 0x0080) != 0x0080);//0x0080=USART_TXE
    USARTx_2->DR = (uint8_t)send;

    send=pulse&0x7F;
    sum+=send;
    while ( (USARTx_2->SR & 0x0080) != 0x0080);
    USARTx_2->DR = (uint8_t)send;

    send=((pulse>>7)&0b11111) | ( (port&0b11) << 5);
    sum+=send;
    while ( (USARTx_2->SR & 0x0080) != 0x0080);
    USARTx_2->DR = (uint8_t)send;

    send=sum&0x7F;
    while ( (USARTx_2->SR & 0x0080) != 0x0080);
    USARTx_2->DR = (uint8_t)send;
}
