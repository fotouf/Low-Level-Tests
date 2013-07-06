#ifndef MAIN_H_
#define MAIN_H_

typedef enum
{
  green1 = 0,
  red = 1,
  yellow = 2,
  green2 = 3,
} Led_TypeDef;

void Safety_first(void);
void debug_led(void);
void debug_led2(void);
void led_on(Led_TypeDef led);
void led_off(Led_TypeDef led);
void led_init(void);
void toggle_led(Led_TypeDef led);


void system_clock_init(void);

extern int helios_stop;


#endif /* MAIN_H_ */
