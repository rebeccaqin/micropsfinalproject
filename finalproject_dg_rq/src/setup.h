#ifndef STM32F4_SETUP_H
#define STM32F4_SETUP_H

// Vendor-provided device header file.
#include "stm32f4xx.h"

#define SYSTEMCORECLOCK 8000000
#define CMD_DELAY_MS 40
#define BUFFER_SIZE 2048
#define DELAY_TIM TIM5
#define ESP_USART_ID 1
#define TERM_USART_ID 2
// Wifi Network Info (make sure to keep surrounding double quotes)
#define SSID "Structure"
#define PASSWORD "structure0770"

// RCC

// Clock configuration
#define SW_HSI  0
#define SW_HSE  1
#define SW_PLL  2

void configureClock();

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

// GPIO
// Values for GPIO pins ("val" arguments)
#define GPIO_LOW    0
#define GPIO_HIGH   1

// Arbitrary GPIO functions for pinMode()
#define GPIO_INPUT  0
#define GPIO_OUTPUT 1
#define GPIO_ALT    2
#define GPIO_ANALOG 3

// Pin definitions for every GPIO pin
#define GPIO_PA0    0
#define GPIO_PA1    1
#define GPIO_PA2    2
#define GPIO_PA3    3
#define GPIO_PA4    4
#define GPIO_PA5    5
#define GPIO_PA6    6
#define GPIO_PA7    7
#define GPIO_PA8    8
#define GPIO_PA9    9
#define GPIO_PA10   10
#define GPIO_PA11   11
#define GPIO_PA12   12
#define GPIO_PA13   13
#define GPIO_PA14   14
#define GPIO_PA15   15

void pinMode(GPIO_TypeDef *, int pin, int function);
int digitalRead(GPIO_TypeDef *, int pin);
void digitalWrite(GPIO_TypeDef *, int pin, int val);
void togglePin(GPIO_TypeDef *, int pin);

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

// USART

#define USART1_ID   1
#define USART2_ID   2
USART_TypeDef * initUSART(uint8_t USART_ID, uint32_t baud_rate);
void sendChar(USART_TypeDef * USART, uint8_t data);
void sendString(USART_TypeDef * USART, uint8_t * charArray);
void receiveString(USART_TypeDef * USART, uint8_t * charArray);

// Ring buffer
#define UART_BUFFER_SIZE 512

/** Structure for ring_buffer.
    Contains a uint8_t buffer of length UART_BUFFER_SIZE to hold a string as
    well as two integers to hold the value of the head and tail of the buffer
    to indicate what characters have or have not been read out.
 */
typedef struct {
    uint8_t buffer[UART_BUFFER_SIZE];
    volatile uint32_t head;
    volatile uint32_t tail;
} ring_buffer;

/** TODO
    Initialize the ring buffer.
    Currently this function only initializes a receive buffer. You should add
    the functionality for a transmit buffer and initialize it in this function.
 */
void init_ring_buffer(void);

/** Read a character from the buffer
    @return character if it there is a character to read; otherwise return -1.
 */
uint8_t read_char_buffer(void);

/** TODO
   Declaration for function to write charater to the transmit buffer.
   You need to implement this.
   It should take a single character, add it to the transmit buffer, and
   increment head appropriately.
 */
void write_char_buffer(uint8_t c);

/** Check if there is data that has not yet been read out of the buffer.
    i.e., check if head is equal to tail.
    @return 1 if head != tail or 0 if head == tail
 */
uint8_t is_data_available(void);

/** Flush the buffer
    (i.e., set all its contents to 0 and reset the head and tail).
 */
void flush_buffer(void);

/** Define our interrupt service routine here. Make sure to put this function
    call into the correct IRQHandler function to map it to the vector table.
 */
void usart_ISR(USART_TypeDef * USART);

/** Look for a particular string in the given buffer
    @return 1 if the string is found and 0 if not found
 */
uint8_t look_for_substring(uint8_t *str, uint8_t *buffertolookinto);

// ESP FUNCTIONS
void initESP8266(USART_TypeDef * ESP_USART, USART_TypeDef * TERM_USART);
void serveWebpage(uint8_t str []);
#endif