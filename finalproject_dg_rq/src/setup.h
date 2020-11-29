#ifndef STM32F4_SETUP_H
#define STM32F4_SETUP_H

// Vendor-provided device header file.
#include "stm32f4xx.h"
#include "STM32F401RE.h"

// Wifi Network Info (make sure to keep surrounding double quotes)
#define SSID "Fios-QS9zf"
#define PASSWORD "yes753code39jab"

#define SYSTEMCORECLOCK 84000000
#define CMD_DELAY_MS 60
#define BUFFER_SIZE 2048
#define DELAY_TIM TIM5
#define ESP_USART_ID 1
#define TERM_USART_ID 2

// FLASH 
void initFLASH(void);

// TIMER FUNCTIONS
#define fast 0
#define slow 1
#define normal 2
void initADCTIM(void);
void initPlayTIM(int speed);
void initDelayTIM(TIM_TypeDef * TIMx);
void delay_millis(TIM_TypeDef * TIMx, uint32_t ms);
void delay_micros(TIM_TypeDef * TIMx, uint32_t us);

// ADC FUNCTIONS
void configureADC(); 


// SPI FUNCTIONS
/* Enables the SPI peripheral and intializes its clock speed (baud rate), polarity, and phase.
 *    -- clkdivide: (0x01 to 0xFF). The SPI clk will be the master clock / clkdivide.
 *    -- cpol: clock polarity (0: inactive state is logical 0, 1: inactive state is logical 1).
 *    -- cpha: clock phase (1: data changed on leading edge of clk and captured on next edge, 
 *          0: data captured on leading edge of clk and changed on next edge)
 * Note: the SPI mode register is set with the following unadjustable settings:
 *    -- Master mode
 *    -- Fixed peripheral select
 *    -- Chip select lines directly connected to peripheral device
 *    -- Mode fault detection enabled
 *    -- WDRBT disabled
 *    -- LLB disabled
 *    -- PCS = 0000 (Peripheral 0 selected), means NPCS[3:0] = 1110
 * Refer to the datasheet for more low-level details. */ 
void spiInit(uint32_t clkdivide, uint32_t cpol, uint32_t ncpha);

/* Transmits a short (2 bytes) over SPI and returns the received short.
 *    -- send: the short to send over SPI
 *    -- return: the short received over SPI */
uint16_t spiSendReceive12(uint16_t send);

// ESP FUNCTIONS
void initESP8266(USART_TypeDef * ESP_USART, USART_TypeDef * TERM_USART);
void serveWebpage(uint8_t str []);
#endif