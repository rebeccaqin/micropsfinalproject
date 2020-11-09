/**
    Main Header: Contains general defines and selected portions of CMSIS files
    @file main.h
    @author Josh Brake
    @version 1.0 10/7/2020
*/

#ifndef MAIN_H
#define MAIN_H

#include "STM32F401RE.h"
// Vendor-provided device header file.
#include "stm32f4xx.h"
///////////////////////////////////////////////////////////////////////////////
// Custom defines
///////////////////////////////////////////////////////////////////////////////

// Wifi Network Info (make sure to keep surrounding double quotes)
#define SSID "Structure"
#define PASSWORD "structure0770"

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
#define REQ_RECORDING_START 1
#define REQ_RECORDING_STOP 2
#define REQ_PLAY 3

// LED pin
#define LED_PIN 5

#define ESP_USART_ID USART1_ID
#define TERM_USART_ID USART2_ID
#define DELAY_TIM TIM5
#define ADC_TIM TIM2
#define CMD_DELAY_MS 40
#define BUFFER_SIZE 2048


#define __NVIC_PRIO_BITS          4U       /*!< STM32F4XX uses 4 Bits for the Priority Levels */

#include "cmsis_gcc.h"
#include "core_cm4.h"


#endif // MAIN_H