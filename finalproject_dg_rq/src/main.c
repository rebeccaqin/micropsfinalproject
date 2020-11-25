/**
    Main: Contains main function
    @file main.c
    @author Rebecca Qin and Dithi Ganjam
*/

#include "main.h"

size_t VOLTAGE_ARRAY_SIZE = 45000;
uint16_t VOLTAGE_ARRAY[45000]; 
int count = 0; // count up to 248,000 for full FLASH memory (sector 1-7)
size_t NUM_SAMPLES = 248000;
int recording = 0;
int play_index = 0;
uint16_t *FLASH_SECTOR_1_ADDRESS = (uint16_t *) 0x08004000;

void play() {
    initPlayTIM(normal);
}

void TIM4_IRQHandler(void) {
    // Clear update interrupt flag
    TIM4->SR &= ~(TIM_SR_UIF);
    if (play_index == NUM_SAMPLES) play_index = 0;
    //spiSendReceive12(VOLTAGE_ARRAY[play_index]);
    spiSendReceive12( *(FLASH_SECTOR_1_ADDRESS + play_index));
    ++play_index;
}
/** Map Button IRQ handler to our custom ISR
 * Button turns on the TIM2 for interrupts to sample at 10kHz
 * If recording, it turns off TIM2, DMA, and ADC to stop recording
 */

void EXTI15_10_IRQHandler(void){
    // Check that the button EXTI_13 was what triggered our interrupt
    if (EXTI->PR & (1 << BUTTON_PIN)){
        // If so, clear the interrupt
        EXTI->PR |= (1 << BUTTON_PIN);
        if (recording) {
            TIM2->CR1 &= ~(0b1);
            ADC1->CR2 &= ~(ADC_CR2_ADON);
            digitalWrite(GPIOA, LED_PIN, GPIO_LOW);
            recording = 0;
        }
        else {
            initFLASH();
            count = 0;
            initADCTIM();
            digitalWrite(GPIOA, LED_PIN, GPIO_HIGH);
            recording = 1;
        }
    }
}   

/** Map TIM2 IRQ handler to our custom ISR
 * TIM2 handler turns on the ADC to start conversions
 * if the buffer is full, turn off recording (timer, ADC, DMA)
 */

void TIM2_IRQHandler(void) {
    // Clear update interrupt flag
    TIM2->SR &= ~(TIM_SR_UIF);
    if (count < NUM_SAMPLES) {
        configureADC();
    }
    else {
        TIM2->CR1 &= ~(0b1);
        ADC1->CR2 &= ~(ADC_CR2_ADON);
        digitalWrite(GPIOA, LED_PIN, GPIO_LOW);
        recording = 0;
    }
}

/** Map ADC IRQ handler to our custom ISR
 * ADC handler turns on the DMA when the the ADC indicates that it's finished converting
 */
void ADC_IRQHandler(void){
    ADC1->SR &= ~(0b10);// clear interrupt
    *(FLASH_SECTOR_1_ADDRESS + count) = (uint16_t) ADC1->DR; 
    while(((FLASH->SR >> FLASH_SR_BSY_Pos) & 0b1) == 1); 
    ++count;
}

/** Map USART1 IRQ handler to our custom ISR
 */
void USART1_IRQHandler(){
    USART_TypeDef * ESP_USART = id2Port(ESP_USART_ID);
    usart_ISR(ESP_USART);
}


int main(void) {

    configureClock();

    // Enable GPIOA and GPIOC clock and DMA2
    RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_DMA2EN);

    // Enable and configure ADC1 and SYSCFG and SPI1
    RCC->APB2ENR |= (RCC_APB2ENR_ADC1EN | RCC_APB2ENR_SYSCFGEN | RCC_APB2ENR_SPI1EN);
    spiInit(0, 1, 1);
    
    // Initialize timers
    RCC->APB1ENR |= (RCC_APB1ENR_TIM2EN | RCC_APB1ENR_TIM5EN | RCC_APB1ENR_TIM3EN | RCC_APB1ENR_TIM4EN); // TIM2_EN
    initDelayTIM(DELAY_TIM);
    initDelayTIM(TIM3);
    // Enable interrupts globally
    __enable_irq();

    // set GPIO PA 8 to LED pin
    pinMode(GPIOA, LED_PIN, GPIO_OUTPUT);
    pinMode(GPIOC, BUTTON_PIN, GPIO_INPUT);
    pinMode(GPIOA, GPIO_PA0, GPIO_ANALOG);
    *SYSCFG_EXTICR4 |= 0b00100000; // Set EXTICR4 for PC13 to 2
    // Configure interrupt for falling edge of GPIO PC13
    // 1. Configure mask bit
    // 2. Disable rising edge trigger
    // 3. Enable falling edge trigger
    // 4. Turn on EXTI interrupt in NVIC_ISER1
    EXTI->IMR |= 1 << 13; // PC13 is EXTI13
    EXTI->RTSR &= ~(1 << 13); // PC13 is EXTI13
    EXTI->FTSR |= 1 << 13; // PC13 is EXTI13
    __NVIC_EnableIRQ(EXTI15_10_IRQn); // enable button interrupt
    __NVIC_EnableIRQ(ADC_IRQn); // enable ADC interrupt
    int playing = 0;
    while(1){
       // delay_millis(TIM3, 20);
        if (count >= NUM_SAMPLES && !playing) {
            play();
            playing = 1;
        }
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