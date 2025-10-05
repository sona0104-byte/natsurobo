#include "main.h"

void AirSet(USART_TypeDef* USARTx_2,int AS1,int AS2,int AS3,int AS4);
/**
  * @brief  電磁弁をUSART/UARTで制御します
  * @param  USARTx_2: 使用するUSART/UART
  * @param  AS1: 基板上のAS1で1ならON,0ならOFFになる(AS他も同様)
  * @retval 無し
  */
void AirSet(USART_TypeDef* USARTx_2,int AS1,int AS2,int AS3,int AS4){
	uint8_t send = 240;	//0x240 = 0b10010000
	send |= AS1 << 3;   //
	send |= AS2 << 2;
	send |= AS3 << 1;
	send |= AS4;
	while ( (USARTx_2->SR & 0x0080) != 0x0080);
		USARTx_2->DR = (uint8_t)send;
}
