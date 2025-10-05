#include <MD03.h>



/**
  * @brief  MDを電圧指令で駆動します
  * @param  USARTx_2: 使用するUSART/UART
  * @param  id : 駆動するMD(0～15)
  * @param  pow: MDの出力(±950)
  *
  * @retval 無し
  */
void MD03SetMotor(USART_TypeDef* USARTx_2,uint8_t id,int pow){
	uint8_t res;
	uint8_t sum=6;
	if(pow>950)pow=950;
	if(pow<-950)pow=-950;

	pow+=1000;

	res= 0b10000000 | (0b1111 & id);
	sum+=res;
	while ( (USARTx_2->SR & 0x0080) != 0x0080);//0x0080=USART_TXE
	USARTx_2->DR = (uint8_t)res;

	res=pow & 0x7F;
	sum+=res;
	while ( (USARTx_2->SR & 0x0080) != 0x0080);
	USARTx_2->DR = (uint8_t)res;

	res=(pow >> 7) & 0x7F;
	sum+=res;
	while ( (USARTx_2->SR & 0x0080) != 0x0080);
	USARTx_2->DR = (uint8_t)res;

	res = sum & 0x7F;
	while ( (USARTx_2->SR & 0x0080) != 0x0080);
	USARTx_2->DR = (uint8_t)res;

}


/**
  * @brief  MDを角度指令で駆動します(定電圧)
  * @param  USARTx_2: 使用するUSART/UART
  * @param  id : 駆動するMD(0～15)
  * @param  deg: 目標角度(-134,217,728～134,217,727)
  * @param  pow: MDの最大出力(0～950)
  *
  * @retval 無し
  */
void MD03SetMotorDeg(USART_TypeDef* USARTx_2,uint8_t id,int deg,int pow){
	uint8_t res;
	uint8_t sum=6;
	if(pow>950)pow=950;
	if(pow<0)pow=0;
	if(deg> 0x7FFFFFF)deg= 0x7FFFFFF;
	if(deg<-0x8000000)deg=-0x8000000;

	deg+=0x8000000;

	res= 0b10010000 | (0b1111 & id);
	sum+=res;
	while ( (USARTx_2->SR & 0x0080) != 0x0080);//0x0080=USART_TXE
	USARTx_2->DR = (uint8_t)res;

	res=deg & 0x7F;
	sum+=res;
	while ( (USARTx_2->SR & 0x0080) != 0x0080);
	USARTx_2->DR = (uint8_t)res;

	res=(deg >> 7) & 0x7F;
	sum+=res;
	while ( (USARTx_2->SR & 0x0080) != 0x0080);
	USARTx_2->DR = (uint8_t)res;

	res=(deg >> 14) & 0x7F;
	sum+=res;
	while ( (USARTx_2->SR & 0x0080) != 0x0080);
	USARTx_2->DR = (uint8_t)res;

	res=(deg >> 21) & 0x7F;
	sum+=res;
	while ( (USARTx_2->SR & 0x0080) != 0x0080);
	USARTx_2->DR = (uint8_t)res;

	res=pow & 0x7F;
	sum+=res;
	while ( (USARTx_2->SR & 0x0080) != 0x0080);
	USARTx_2->DR = (uint8_t)res;

	res=(pow >> 7) & 0x7F;
	sum+=res;
	while ( (USARTx_2->SR & 0x0080) != 0x0080);
	USARTx_2->DR = (uint8_t)res;

	res = sum & 0x7F;
	while ( (USARTx_2->SR & 0x0080) != 0x0080);
	USARTx_2->DR = (uint8_t)res;

}



/**
  * @brief  MDを角度指令で駆動します(定速)
  * @param  USARTx_2: 使用するUSART/UART
  * @param  id : 駆動するMD(0～15)
  * @param  deg: 目標角度(-134,217,728～134,217,727)
  * @param  spd: MDの最大速度(0～16383)
  *
  * @retval 無し
  */
void MD03SetMotorDegSpd(USART_TypeDef* USARTx_2,uint8_t id,int deg,int spd){
	uint8_t res;
	uint8_t sum=6;
	if(spd>16383)spd=16383;
	if(spd<0    )spd=0;
	if(deg> 0x7FFFFFF)deg= 0x7FFFFFF;
	if(deg<-0x8000000)deg=-0x8000000;

	deg+=0x8000000;

	res= 0b10100000 | (0b1111 & id);
	sum+=res;
	while ( (USARTx_2->SR & 0x0080) != 0x0080);//0x0080=USART_TXE
	USARTx_2->DR = (uint8_t)res;

	res=deg & 0x7F;
	sum+=res;
	while ( (USARTx_2->SR & 0x0080) != 0x0080);
	USARTx_2->DR = (uint8_t)res;

	res=(deg >> 7) & 0x7F;
	sum+=res;
	while ( (USARTx_2->SR & 0x0080) != 0x0080);
	USARTx_2->DR = (uint8_t)res;

	res=(deg >> 14) & 0x7F;
	sum+=res;
	while ( (USARTx_2->SR & 0x0080) != 0x0080);
	USARTx_2->DR = (uint8_t)res;

	res=(deg >> 21) & 0x7F;
	sum+=res;
	while ( (USARTx_2->SR & 0x0080) != 0x0080);
	USARTx_2->DR = (uint8_t)res;

	res=spd & 0x7F;
	sum+=res;
	while ( (USARTx_2->SR & 0x0080) != 0x0080);
	USARTx_2->DR = (uint8_t)res;

	res=(spd >> 7) & 0x7F;
	sum+=res;
	while ( (USARTx_2->SR & 0x0080) != 0x0080);
	USARTx_2->DR = (uint8_t)res;

	res = sum & 0x7F;
	while ( (USARTx_2->SR & 0x0080) != 0x0080);
	USARTx_2->DR = (uint8_t)res;

}


/**
  * @brief  MDを速度指令で駆動します
  * @param  USARTx_2: 使用するUSART/UART
  * @param  id : 駆動するMD(0～15)
  * @param  spd: 指令速度(-134,217,728～134,217,727)
  *
  * @retval 無し
  */
void MD03SetMotorSpd(USART_TypeDef* USARTx_2,uint8_t id,int spd){
	uint8_t res;
	uint8_t sum=6;

    if(spd> 0xFFFFF )spd= 0xFFFFF;
    if(spd<-0x100000)spd=-0x100000;

	spd+=0x100000;

	res= 0b10110000 | (0b1111 & id);
	sum+=res;
	while ( (USARTx_2->SR & 0x0080) != 0x0080);//0x0080=USART_TXE
	USARTx_2->DR = (uint8_t)res;

	res=spd & 0x7F;
	sum+=res;
	while ( (USARTx_2->SR & 0x0080) != 0x0080);
	USARTx_2->DR = (uint8_t)res;

	res=(spd >> 7) & 0x7F;
	sum+=res;
	while ( (USARTx_2->SR & 0x0080) != 0x0080);
	USARTx_2->DR = (uint8_t)res;

	res=(spd >> 14) & 0x7F;
	sum+=res;
	while ( (USARTx_2->SR & 0x0080) != 0x0080);
	USARTx_2->DR = (uint8_t)res;

	res = sum & 0x7F;
	while ( (USARTx_2->SR & 0x0080) != 0x0080);
	USARTx_2->DR = (uint8_t)res;
}


/**
  * @brief  MDの角度制御器のPIDゲインを設定します
  * @param  USARTx_2: 使用するUSART/UART
  * @param  id : 設定するMD(0～15)
  * @param  p  : Pゲイン
  * @param  i  : Iゲイン
  * @param  d  : Dゲイン
  *
  * @retval 無し
  */
void MD03SetPIDDeg(USART_TypeDef* USARTx_2,uint8_t id,float p,float i,float d){
    uint8_t send;
    uint8_t sum=6;

    int pi,ii,di;
    pi=p*1024;
    ii=i*65536/1000;
    di=d*1024*1000;
    if(pi<0)pi=0;
    if(pi>2097151)pi=2097151;
    if(ii<0)ii=0;
    if(ii>2097151)ii=2097151;
    if(di<0)di=0;
    if(di>2097151)di=2097151;

    send=0x80 | (0b100<<4) | (id & 0xF);
    sum+=send;
	while ( (USARTx_2->SR & 0x0080) != 0x0080);
	USARTx_2->DR = (uint8_t)send;


    send=pi&0x7F;
    sum+=send;
	while ( (USARTx_2->SR & 0x0080) != 0x0080);
	USARTx_2->DR = (uint8_t)send;

    send=(pi>>7)&0x7F;
    sum+=send;
	while ( (USARTx_2->SR & 0x0080) != 0x0080);
	USARTx_2->DR = (uint8_t)send;

    send=(pi>>14)&0x7F;
    sum+=send;
	while ( (USARTx_2->SR & 0x0080) != 0x0080);
	USARTx_2->DR = (uint8_t)send;


    send=ii&0x7F;
    sum+=send;
	while ( (USARTx_2->SR & 0x0080) != 0x0080);
	USARTx_2->DR = (uint8_t)send;

    send=(ii>>7)&0x7F;
    sum+=send;
	while ( (USARTx_2->SR & 0x0080) != 0x0080);
	USARTx_2->DR = (uint8_t)send;

    send=(ii>>14)&0x7F;
    sum+=send;
	while ( (USARTx_2->SR & 0x0080) != 0x0080);
	USARTx_2->DR = (uint8_t)send;



    send=di&0x7F;
    sum+=send;
	while ( (USARTx_2->SR & 0x0080) != 0x0080);
	USARTx_2->DR = (uint8_t)send;

    send=(di>>7)&0x7F;
    sum+=send;
	while ( (USARTx_2->SR & 0x0080) != 0x0080);
	USARTx_2->DR = (uint8_t)send;

    send=(di>>14)&0x7F;
    sum+=send;
	while ( (USARTx_2->SR & 0x0080) != 0x0080);
	USARTx_2->DR = (uint8_t)send;



    send=sum&0x7F;
	while ( (USARTx_2->SR & 0x0080) != 0x0080);
	USARTx_2->DR = (uint8_t)send;

}

/**
  * @brief  MDの速度制御器のPIDゲインを設定します
  * @param  USARTx_2: 使用するUSART/UART
  * @param  id : 設定するMD(0～15)
  * @param  p  : Pゲイン
  * @param  i  : Iゲイン
  * @param  d  : Dゲイン
  *
  * @retval 無し
  */
void MD03SetPIDSpd(USART_TypeDef* USARTx_2,uint8_t id,float p,float i,float d){
    uint8_t send;
    uint8_t sum=6;

    int pi,ii,di;
    pi=p*1024;
    ii=i*65536/250;
    di=d*1024*250;
    if(pi<0)pi=0;
    if(pi>2097151)pi=2097151;
    if(ii<0)ii=0;
    if(ii>2097151)ii=2097151;
    if(di<0)di=0;
    if(di>2097151)di=2097151;

    send=0x80 | (0b101<<4) | (id & 0xF);
    sum+=send;
	while ( (USARTx_2->SR & 0x0080) != 0x0080);
	USARTx_2->DR = (uint8_t)send;



    send=pi&0x7F;
    sum+=send;
	while ( (USARTx_2->SR & 0x0080) != 0x0080);
	USARTx_2->DR = (uint8_t)send;

    send=(pi>>7)&0x7F;
    sum+=send;
	while ( (USARTx_2->SR & 0x0080) != 0x0080);
	USARTx_2->DR = (uint8_t)send;

    send=(pi>>14)&0x7F;
    sum+=send;
	while ( (USARTx_2->SR & 0x0080) != 0x0080);
	USARTx_2->DR = (uint8_t)send;


    send=ii&0x7F;
    sum+=send;
	while ( (USARTx_2->SR & 0x0080) != 0x0080);
	USARTx_2->DR = (uint8_t)send;

    send=(ii>>7)&0x7F;
    sum+=send;
	while ( (USARTx_2->SR & 0x0080) != 0x0080);
	USARTx_2->DR = (uint8_t)send;

    send=(ii>>14)&0x7F;
    sum+=send;
	while ( (USARTx_2->SR & 0x0080) != 0x0080);
	USARTx_2->DR = (uint8_t)send;



    send=di&0x7F;
    sum+=send;
	while ( (USARTx_2->SR & 0x0080) != 0x0080);
	USARTx_2->DR = (uint8_t)send;

    send=(di>>7)&0x7F;
    sum+=send;
	while ( (USARTx_2->SR & 0x0080) != 0x0080);
	USARTx_2->DR = (uint8_t)send;

    send=(di>>14)&0x7F;
    sum+=send;
	while ( (USARTx_2->SR & 0x0080) != 0x0080);
	USARTx_2->DR = (uint8_t)send;




    send=sum&0x7F;
	while ( (USARTx_2->SR & 0x0080) != 0x0080);
	USARTx_2->DR = (uint8_t)send;
}


/**
  * @brief  MDのエンコーダ方向を設定します
  * @param  USARTx_2: 使用するUSART/UART
  * @param  id : 設定するMD(0～15)
  * @param  en : 反転有効フラグ
  *
  * @retval 無し
  */
void MD03SetEncInv(USART_TypeDef* USARTx_2,uint8_t id,uint8_t en){
	uint8_t send;
	uint8_t sum=6;

    send=0x80 | (0b110<<4) | (id & 0xF);
    sum+=send;
	while ( (USARTx_2->SR & 0x0080) != 0x0080);
	USARTx_2->DR = (uint8_t)send;

    send=en&0x01;
    sum+=send;
	while ( (USARTx_2->SR & 0x0080) != 0x0080);
	USARTx_2->DR = (uint8_t)send;

    send=sum&0x7F;
	while ( (USARTx_2->SR & 0x0080) != 0x0080);
	USARTx_2->DR = (uint8_t)send;
}

