

#ifndef ESP_H
#define ESP_H

#include "STM32F401RE.h"

void initESP8266(USART_TypeDef * ESP_USART, USART_TypeDef * TERM_USART);
void serveWebpage(uint8_t str []);

#endif