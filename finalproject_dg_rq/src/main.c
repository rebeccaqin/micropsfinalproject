/**
    Main: Contains main function
    @file main.c
    @author Josh Brake
    @version 1.0 10/7/2020
*/

#include "STM32F401RE.h"
#include "main.h"
#include <string.h>
#include "UARTRingBuffer.h"
#include "esp.h"

size_t VOLTAGE_ARRAY_SIZE = 5000;
uint16_t VOLTAGE_ARRAY[5000]; 

void init_DMA(){
    // DMA2 configuration (stream 6 / channel 4).
    // SxCR register:
    // - Memory-to-peripheral
    // - Circular mode enabled.
    // - Increment memory ptr, don't increment periph ptr.
    // - 8-bit data size for both source and destination.
    // - High priority (2/3).

    // Reset DMA2 Stream 0
    DMA_STREAM->CR &= ~(DMA_SxCR_CHSEL |
                            DMA_SxCR_PL |
                            DMA_SxCR_MSIZE |
                            DMA_SxCR_PSIZE |
                            DMA_SxCR_PINC |
                            DMA_SxCR_EN );
    // Set up DMA2 Stream 0
    DMA_STREAM->CR |=  ( (0x2 << DMA_SxCR_PL_Pos) |
                            (0x0 << DMA_SxCR_MSIZE_Pos) |
                            (0x0 << DMA_SxCR_PSIZE_Pos) |
                            (0x0 << DMA_SxCR_CHSEL_Pos) |
                            (DMA_SxCR_MINC)             );
    
    // Set DMA source and destination addresses.
    // Dest: Address of the character array buffer in memory.
    DMA_STREAM->M0AR  = (uint32_t) &VOLTAGE_ARRAY;
    // Source: ADC data register
    DMA_STREAM->PAR   = (uint32_t) &(ADC->ADCDR);
    // Set DMA data transfer length (# of samples).
    DMA_STREAM->NDTR  = (uint16_t) 1;

    // Enable DMA stream.
    DMA_STREAM->CR   |= DMA_SxCR_EN;
}

/** Map TIM2 IRQ handler to our custom ISR
 * TODO: fix this with updated DMA stream
 */
int count = 0;
void TIM2_IRQHandler(void) {
    // Clear update interrupt flag
    TIM2->SR &= ~(TIM_SR_UIF);
    
    if (count < VOLTAGE_ARRAY_SIZE) {
         DMA1->HIFCR = (DMA_LIFCR_CTCIF0 | DMA_LIFCR_CHTIF0 | DMA_LIFCR_CTEIF0 | DMA_LIFCR_CDMEIF0 | DMA_LIFCR_CFEIF0);
        // Dest: Address of the character array buffer incremented in memory.
        DMA_STREAM->M0AR  = (uint32_t) &(VOLTAGE_ARRAY[count]);
        // Reset number of bytes to transmit
        DMA_STREAM->NDTR  = (uint16_t) 1;
        // Re-enable DMA stream.
        DMA_STREAM->CR   |= DMA_SxCR_EN;
    }
    else {
        TIM2->CR1 &= ~(0b1);
        DMA_STREAM->CR   &= ~(DMA_SxCR_EN);
        ADC->ADCCR2.ADON = 0;
        digitalWrite(GPIOA, LED_PIN, GPIO_LOW);
        recording = 0;
    }
}

/** Map USART1 IRQ handler to our custom ISR
 */
void USART1_IRQHandler(){
    USART_TypeDef * ESP_USART = id2Port(ESP_USART_ID);
    usart_ISR(ESP_USART);
}

/** Map Button IRQ handler to our custom ISR
 * TODO: fix this so it handles recording 
 */
int recording = 0;
void EXTI15_10_IRQHandler(void){
    // Check that the button EXTI_13 was what triggered our interrupt
    if (EXTI->PR & (1 << BUTTON_PIN)){
        // If so, clear the interrupt
        EXTI->PR |= (1 << BUTTON_PIN);
        if (recording) {
            TIM2->CR1 &= ~(0b1);
            DMA_STREAM->CR   &= ~(DMA_SxCR_EN);
            ADC->ADCCR2.ADON = 0;
            digitalWrite(GPIOA, LED_PIN, GPIO_LOW);
            recording = 0;
        }
        else {
            initTIM2();
            // Clear Stream 0 DMA flags
            DMA1->HIFCR = (DMA_LIFCR_CTCIF0 | DMA_LIFCR_CHTIF0 | DMA_LIFCR_CTEIF0 | DMA_LIFCR_CDMEIF0 | DMA_LIFCR_CFEIF0);
            ADC->ADCCR2.ADON = 1;
            init_DMA();
            digitalWrite(GPIOA, LED_PIN, GPIO_HIGH);
            recording = 1;
        }
    }
}   

int main(void) {

    configureClock();

    // Enable GPIOA and GPIOC clock
    RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOCEN);

    // Enable and configure ADC1 and SYSCFG and SPI1
    RCC->APB2ENR |= (RCC_APB2ENR_ADC1EN | RCC_APB2ENR_SYSCFGEN | RCC_APB2ENR_SPI1EN);
    configureADC();
    spiInit(0, 1, 0);

    // Initialize timers
    RCC->APB1ENR |= (RCC_APB1ENR_TIM2EN | RCC_APB1ENR_TIM5EN); // TIM2_EN
    initTIM(DELAY_TIM);

    // Enable interrupts globally
    __enable_irq();

    // set GPIO PA 8 to LED pin
    pinMode(GPIOA, LED_PIN, GPIO_OUTPUT);
    pinMode(GPIOC, BUTTON_PIN, GPIO_INPUT);
    *SYSCFG_EXTICR4 |= 0b00100000; // Set EXTICR4 for PC13 to 2
    // Configure interrupt for falling edge of GPIO PC13
    // 1. Configure mask bit
    // 2. Disable rising edge trigger
    // 3. Enable falling edge trigger
    // 4. Turn on EXTI interrupt in NVIC_ISER1
    EXTI->IMR |= 1 << 13; // PC13 is EXTI13
    EXTI->RTSR &= ~(1 << 13); // PC13 is EXTI13
    EXTI->FTSR |= 1 << 13; // PC13 is EXTI13
    __NVIC_EnableIRQ(EXTI15_10_IRQn); 
    
    while(1){
        
    }
    /*
    // Configure ESP and Terminal UARTs
    USART_TypeDef * ESP_USART = initUSART(ESP_USART_ID, 115200);
    USART_TypeDef * TERM_USART = initUSART(TERM_USART_ID, 115200);
    */
    // Configure USART1 interrupt
    /*
    // Configure interrupt for USART1
    *NVIC_ISER1 |= (1 << 5);
    ESP_USART->CR1.RXNEIE = 1;
    
    // Initialize ring buffer
    init_ring_buffer();
    flush_buffer();

    /*
    // Initialize ESP
    delay_millis(DELAY_TIM, 1000);
    initESP8266(ESP_USART, TERM_USART);
    delay_millis(DELAY_TIM, 500);
    
    // Set up temporary buffers for requests
    uint8_t volatile http_request[BUFFER_SIZE] = "";
    uint8_t volatile temp_str[BUFFER_SIZE] = "";
    
    while(1) {
        // Clear temp_str buffer
        memset(http_request, 0, BUFFER_SIZE);
        volatile uint32_t http_req_len = 0;

        // Loop through and read any data available in the buffer
        if(is_data_available()) {
            do{
                memset(temp_str, 0, BUFFER_SIZE);
                readString(ESP_USART, temp_str); // Read in available bytes
                strcat(http_request, temp_str); // Append to current http_request string
                http_req_len = strlen(http_request); // Store length of request
                delay_millis(DELAY_TIM, 20); // Delay
            } while(is_data_available()); // Check for end of transaction

            // Echo received string to the terminal
            sendString(TERM_USART, http_request);

            // Search to see if there was a GET request
            volatile uint8_t get_request = look_for_substring("GET", http_request);

            // If a GET request, process the request
            if(get_request == 1){
                // Look for "REQ" in http_request
                volatile uint8_t button_req = look_for_substring("REQ", http_request);
                volatile uint8_t favicon_req = look_for_substring("favicon", http_request);

                if(!favicon_req){
                    
                    /* Look for request data and process it
                        If REQ=ON, then turn LED on.
                        If REQ=OFF, then turn LED off.
                        If we don't recognize the REQ, then send message to terminal and don't do anything.
                    */
                   /*
                    if(button_req == 1){
                        volatile uint8_t button_req_type;
                        if(look_for_substring("=START", http_request)) button_req_type = REQ_RECORDING_START;
                        else if(look_for_substring("=STOP", http_request)) button_req_type = REQ_RECORDING_STOP;
                        else if(look_for_substring("=PLAYVOICE", http_request)) button_req_type = REQ_PLAY;
                        else button_req_type = REQ_UNKNOWN;

                        switch(button_req_type){
                            case REQ_RECORDING_START:
                                digitalWrite(GPIOA, LED_PIN, 1);
                                sendString(TERM_USART, "Turning LED on.\n");
                                break;
                            case REQ_RECORDING_STOP:
                                digitalWrite(GPIOA, LED_PIN, 0);
                                sendString(TERM_USART, "Turning LED off.\n");
                                break;
                            case REQ_PLAY:
                                break;
                            case REQ_UNKNOWN:
                                sendString(TERM_USART, "Unknown request.\n");
                        }
                    }

                    // Serve the individual HTML commands for the webpage
                    serveWebpage("<!DOCTYPE html>");
                    serveWebpage("<meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">");
                    serveWebpage("<title>Alien Voice Filter</title>");
                    serveWebpage("<h3>~~ALiEn vOicE FiLteR~~</h3>");
                    serveWebpage("<p>Welcome to the spooky alien voice filter!</p>");
                    serveWebpage("<p>Press record to start recording your voice. Press stop recording to stop. The recording will automatically stop after 30 seconds.</p>");
                    serveWebpage("<form action=\"REQ=START\"><input type=\"submit\" value = \"Record\"></form>");
                    serveWebpage("<form action=\"REQ=STOP\"><input type=\"submit\" value = \"Stop Recording\"></form>");
                    serveWebpage("<form action=\"REQ=PLAYVOICE\"><input type=\"submit\" value = \"Play voice\"></form>");

                    // Read if LED is on or off and display to webpage.
                    if(digitalRead(GPIOA, LED_PIN)){
                        serveWebpage("<p>LED is ON.</p>");
                    }
                    else {
                        serveWebpage("<p>LED is OFF.</p>");
                    }
                }

                // Close connection
                memset(temp_str, 0, BUFFER_SIZE);
                sendString(ESP_USART, "AT+CIPCLOSE=0\r\n");
                readString(ESP_USART, temp_str);
                sendString(TERM_USART,temp_str);
            }
        }
    }
    */
}