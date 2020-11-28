/**
    Main Header: Contains general defines and selected portions of CMSIS files
    @file main.h
    @author Josh Brake
    @version 1.0 10/7/2020
*/

#ifndef MAIN_H
#define MAIN_H

#include "setup.h"
#include <string.h>

///////////////////////////////////////////////////////////////////////////////
// Custom defines
///////////////////////////////////////////////////////////////////////////////

// for playing
#define NORMAL 0
#define ALIEN 1


#define NVIC_ISER0 ((uint32_t *) 0xE000E100UL)
#define NVIC_ISER1 ((uint32_t *) 0xE000E104UL)
#define SYSCFG_EXTICR4 ((uint32_t *) (0x40013800UL + 0x14UL))
#define DMA_STREAM DMA2_Stream0

#define NSS_PIN GPIO_PA1
#define SPI1CLK_PIN GPIO_PA5
#define MISO_PIN GPIO_PA6
#define MOSI_PIN GPIO_PA7
#define BUTTON_PIN 13 // PC13
#define LED_PIN GPIO_PA8

// Request defines
#define REQ_UNKNOWN 0
#define REQ_PLAY 1
#define REQ_SLOW 2
#define REQ_FAST 3
#define REQ_ALIEN 4


#define ESP_USART_ID USART1_ID
#define TERM_USART_ID USART2_ID
#define DELAY_TIM TIM5
#define ADC_TIM TIM2
#define PLAY_TIM TIM4

#define __NVIC_PRIO_BITS          4U       /*!< STM32F4XX uses 4 Bits for the Priority Levels */

#include "cmsis_gcc.h"
#include "core_cm4.h"


#endif // MAIN_H