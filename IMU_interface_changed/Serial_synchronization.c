
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_dma.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_gpio.h"
#include "misc.h"

#include "main.h"




unsigned int i=0;
unsigned char FirstMessage[560]={0};

void Synchronisation_step(void)
{
	USART_ITConfig(USART3,USART_IT_RXNE,ENABLE);
	USART_ITConfig(USART3,USART_IT_TC,DISABLE);


	USART_DMACmd(USART3, USART_DMAReq_Rx, DISABLE);

	DMA_Cmd(DMA1_Stream1,DISABLE);
	while(DMA_GetCmdStatus(DMA1_Stream1) != DISABLE);

}


void USART3_IRQHandler(void)
{
//	char i;

	unsigned short lenght;
	unsigned short crc;
//	    char *qW_ptr = (char*)&Sensor_val.qW, *qX_ptr = (char*)&Sensor_val.qX, *qY_ptr = (char*)&Sensor_val.qY, *qZ_ptr = (char*)&Sensor_val.qZ;
//	    char *Gx_ptr = (char*)&Sensor_val.gyro_x, *Gy_ptr = (char*)&Sensor_val.gyro_y, *Gz_ptr = (char*)&Sensor_val.gyro_z;
//	    char *eulx_ptr = (char*)&Sensor_val.theta_x, *euly_ptr = (char*)&Sensor_val.theta_y, *eulz_ptr = (char*)&Sensor_val.theta_z;
//	    char *p_acc_x = (char*)&Sensor_val.acc_x;
//	    char *p_acc_y = (char*)&Sensor_val.acc_y;
//	    char *p_acc_z = (char*)&Sensor_val.acc_z;
//
//
//	    //led_on(yellow);	//let's see if TC hits

	FirstMessage[i] = USART_ReceiveData(USART3);

	/*First we look for the start byte and we find it we read and store the first message */

	if (FirstMessage[0]== 0xff)
	{


//		if ((FirstMessage[i-1]==0xff)&&(i!=2)&&(i!=62)&&(FirstMessage[i]== 0x02))
//		{
//			int test=5;
//		}
		i=i+1;
		/*Then we check if the message is complete and correct*/
		if ((FirstMessage[1]== 0x02)&&(i==MESS_LENGTH_RX))
			{

			USART_ITConfig(USART3,USART_IT_RXNE,DISABLE);

			DMA_SetCurrDataCounter(DMA1_Stream1,MESS_LENGTH_RX);

			DMA_Cmd(DMA1_Stream1,ENABLE);
			while(DMA_GetCmdStatus(DMA1_Stream1) != ENABLE);

			USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);

				lenght = (((short)FirstMessage[3]<<8) | (short)FirstMessage[4]);            // Length extract
				crc = (((unsigned short)FirstMessage[lenght+5]<<8) |
						(unsigned short)FirstMessage[lenght+1+5]);                          // CRC extract
				if(crc == calcCRC(&FirstMessage[2], lenght+3))                              // Crc test
				{

					switch(FirstMessage[2])                                                 // Determine command
					{
						case SBG_CONTINIOUS_DEFAULT_OUTPUT:break;
						default: led_on(yellow);break;
					}

					/* When the board has received a correct message, we stop the receiving byte-by-byte by
					 * the usart interrupt, we enable the DMA to read a data packet at a time and use the DMA
					 * interrupt from now one */


				}
				else
				{
					i=0;
				}
			}

	}


	USART_ClearFlag(USART3,USART_FLAG_RXNE);


	}
