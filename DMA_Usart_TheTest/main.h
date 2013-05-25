typedef enum
{
  green1 = 0,
  red = 1,
  yellow = 2,
  green2 = 3,
} Led_TypeDef;

void led_on(Led_TypeDef led);
void led_off(Led_TypeDef led);
void led_init(void);
void toggle_led(Led_TypeDef led);


void SendMsgIMU(unsigned short AnzahlDaten,uint8_t *StartAdresse);
void Start_Continious_Mode(void);
void init_usart3(void);
void USART3_IRQHandler(void);


void Delay(__IO uint32_t nCount);















