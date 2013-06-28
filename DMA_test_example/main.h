/**
  ******************************************************************************
  * @file    USART/USART_TwoBoards/USART_DataExchangeDMA/main.h
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    18-January-2013
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2013 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software
  * distributed under the License is distributed on an "AS IS" BASIS,
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H


/* Exported typedef ----------------------------------------------------------*/
typedef enum {FAILED = 0, PASSED = !FAILED} TestStatus;

/* Exported define -----------------------------------------------------------*/
/* Uncomment the line below if you will use the USART in Transmitter Mode */
 //#define USART_TRANSMITTER
/* Uncomment the line below if you will use the USART in Receiver Mode */
//#define USART_RECEIVER

  /* Definition for USARTx resources ********************************************/
  #define USARTx                           USART3
  #define USARTx_CLK                       RCC_APB1Periph_USART3
  #define USARTx_CLK_INIT                  RCC_APB1PeriphClockCmd
  #define USARTx_IRQn                      USART3_IRQn
  #define USARTx_IRQHandler                USART3_IRQHandler

  #define USARTx_TX_PIN                    GPIO_Pin_8
  #define USARTx_TX_GPIO_PORT              GPIOD
  #define USARTx_TX_GPIO_CLK               RCC_AHB1Periph_GPIOD
  #define USARTx_TX_SOURCE                 GPIO_PinSource8
  #define USARTx_TX_AF                     GPIO_AF_USART3

  #define USARTx_RX_PIN                    GPIO_Pin_9
  #define USARTx_RX_GPIO_PORT              GPIOD
  #define USARTx_RX_GPIO_CLK               RCC_AHB1Periph_GPIOD
  #define USARTx_RX_SOURCE                 GPIO_PinSource9
  #define USARTx_RX_AF                     GPIO_AF_USART3

  /* Definition for DMAx resources **********************************************/
  #define USARTx_DR_ADDRESS                ((uint32_t)USART3 + 0x04)

  #define USARTx_DMA                       DMA1
  #define USARTx_DMAx_CLK                  RCC_AHB1Periph_DMA1

  #define USARTx_TX_DMA_CHANNEL            DMA_Channel_4
  #define USARTx_TX_DMA_STREAM             DMA1_Stream3
  #define USARTx_TX_DMA_FLAG_FEIF          DMA_FLAG_FEIF3
  #define USARTx_TX_DMA_FLAG_DMEIF         DMA_FLAG_DMEIF3
  #define USARTx_TX_DMA_FLAG_TEIF          DMA_FLAG_TEIF3
  #define USARTx_TX_DMA_FLAG_HTIF          DMA_FLAG_HTIF3
  #define USARTx_TX_DMA_FLAG_TCIF          DMA_FLAG_TCIF3

  #define USARTx_RX_DMA_CHANNEL            DMA_Channel_4
  #define USARTx_RX_DMA_STREAM             DMA1_Stream1
  #define USARTx_RX_DMA_FLAG_FEIF          DMA_FLAG_FEIF1
  #define USARTx_RX_DMA_FLAG_DMEIF         DMA_FLAG_DMEIF1
  #define USARTx_RX_DMA_FLAG_TEIF          DMA_FLAG_TEIF1
  #define USARTx_RX_DMA_FLAG_HTIF          DMA_FLAG_HTIF1
  #define USARTx_RX_DMA_FLAG_TCIF          DMA_FLAG_TCIF1

  #define USARTx_DMA_TX_IRQn               DMA1_Stream3_IRQn
  #define USARTx_DMA_RX_IRQn               DMA1_Stream1_IRQn
  #define USARTx_DMA_TX_IRQHandler         DMA1_Stream3_IRQHandler
  #define USARTx_DMA_RX_IRQHandler         DMA1_Stream1_IRQHandler

  //-----------Serial protocol--------//
  // startbit, stopbit, end of fram bit
  #define SYNCX_IMU   0xFF
  #define STX_IMU     0x02
  #define ETX         0x03

  //-----------IMU stuff-------------//
#define SBG_SET_CONTINUOUS_MODE 			0x53
#define SBG_CONTINIOUS_DEFAULT_OUTPUT       0x90


/* Misc definition ************************************************************/
/* Transmit buffer size */
#define Command_Size                       11
#define MESS_LENGTH_RX          60

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
