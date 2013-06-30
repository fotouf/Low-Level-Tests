

/* Exported typedef ----------------------------------------------------------*/
typedef enum {FAILED = 0, PASSED = !FAILED} TestStatus;


  /* Definition for USARTx resources ********************************************/
  #define USARTx                           USART6
  #define USARTx_CLK                       RCC_APB2Periph_USART6
  #define USARTx_CLK_INIT                  RCC_APB2PeriphClockCmd
  #define USARTx_IRQn                      USART6_IRQn
  #define USARTx_IRQHandler                USART6_IRQHandler

  #define USARTx_TX_PIN                    GPIO_Pin_6
  #define USARTx_TX_GPIO_PORT              GPIOC
  #define USARTx_TX_GPIO_CLK               RCC_AHB1Periph_GPIOC
  #define USARTx_TX_SOURCE                 GPIO_PinSource6
  #define USARTx_TX_AF                     GPIO_AF_USART6

  #define USARTx_RX_PIN                    GPIO_Pin_9
  #define USARTx_RX_GPIO_PORT              GPIOG
  #define USARTx_RX_GPIO_CLK               RCC_AHB1Periph_GPIOG
  #define USARTx_RX_SOURCE                 GPIO_PinSource9
  #define USARTx_RX_AF                     GPIO_AF_USART6

  /* Definition for DMAx resources **********************************************/
  #define USARTx_DR_ADDRESS                ((uint32_t)USART6 + 0x04)

  #define USARTx_DMA                       DMA2
  #define USARTx_DMAx_CLK                  RCC_AHB1Periph_DMA2

  #define USARTx_TX_DMA_CHANNEL            DMA_Channel_5
  #define USARTx_TX_DMA_STREAM             DMA2_Stream6
  #define USARTx_TX_DMA_FLAG_FEIF          DMA_FLAG_FEIF6
  #define USARTx_TX_DMA_FLAG_DMEIF         DMA_FLAG_DMEIF6
  #define USARTx_TX_DMA_FLAG_TEIF          DMA_FLAG_TEIF6
  #define USARTx_TX_DMA_FLAG_HTIF          DMA_FLAG_HTIF6
  #define USARTx_TX_DMA_FLAG_TCIF          DMA_FLAG_TCIF6

  #define USARTx_RX_DMA_CHANNEL            DMA_Channel_5
  #define USARTx_RX_DMA_STREAM             DMA2_Stream1
  #define USARTx_RX_DMA_FLAG_FEIF          DMA_FLAG_FEIF1
  #define USARTx_RX_DMA_FLAG_DMEIF         DMA_FLAG_DMEIF1
  #define USARTx_RX_DMA_FLAG_TEIF          DMA_FLAG_TEIF1
  #define USARTx_RX_DMA_FLAG_HTIF          DMA_FLAG_HTIF1
  #define USARTx_RX_DMA_FLAG_TCIF          DMA_FLAG_TCIF1

  #define USARTx_DMA_TX_IRQn               DMA2_Stream6_IRQn
  #define USARTx_DMA_RX_IRQn               DMA2_Stream1_IRQn
  #define USARTx_DMA_TX_IRQHandler         DMA2_Stream6_IRQHandler
  #define USARTx_DMA_RX_IRQHandler         DMA2_Stream1_IRQHandler


//Additional definitions---------------------------------------------------------//

/* Transmit buffer size */
#define BUFFERSIZE                       60
#define MESS_LENGTH_RX          60
#define SYNCX_H                     'H'                 // 'H' = 72

// Mode control
enum control{C_POSITION, C_VELOCITY, C_ACCELERATION, C_STOP, C_FREEZE, C_ROTONDO, C_HULL_POSITION, C_ACCELERATION_WZ, C_MODE_4,
			 C_MODE_5, C_MODE_6, C_MODE_7, C_MODE_8, C_MODE_9, C_MODE_10, C_MODE_11};
enum control Control_Mode;


