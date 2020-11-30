/**
    Main: Contains main function
    @file main.c
    @author Josh Brake
    @version 1.0 10/7/2020
*/

#include "STM32F401RE.h"
#include "main.h"
#include <string.h>
#include <math.h>
#include "UARTRingBuffer.h"

size_t VOLTAGE_ARRAY_SIZE = 45000; // RAM
uint16_t VOLTAGE_ARRAY[45000]; // RAM
int count = 0; // count up to 248,000 for full FLASH memory (sector 1-7)
size_t NUM_SAMPLES = 130000; // sector 6-7
int recording = 0;
int play_index = 0;
uint16_t *FLASH_SECTOR_6_ADDRESS = (uint16_t *) 0x08040000;
int TYPE = NORMAL;

// TIMER
const uint16_t arr_for_sampling = 2100;

void initADCTIM(void) {
    //enable all registers
    TIM2->EGR |= 1;
    //set PWM frequency to 40kHz
    TIM2->ARR = arr_for_sampling;
    // Configure interrupt enable on update event
    TIM2->DIER |= 0b1; // TIM_DIER_UIE
    NVIC_EnableIRQ(TIM2_IRQn);
    //enable counter
    TIM2->CR1 |= 1;
}

void initPlayTIM(uint16_t arr) {
    TIM4->EGR |= 1;
    TIM4->DIER |= 0b1; // TIM_DIER_UIE
    TIM4->ARR = arr;
    NVIC_EnableIRQ(TIM4_IRQn);
    //enable counter
    TIM4->CR1 |= 1;
}

/** Initialize the ESP and print out IP address to terminal
 */
void initESP8266(USART_TypeDef * ESP_USART, USART_TypeDef * TERM_USART){
    uint8_t volatile str[BUFFER_SIZE] = "";

    // Disable echo
    sendString(ESP_USART, "ATE1\r\n");
    delay_millis(DELAY_TIM, CMD_DELAY_MS);
    readString(ESP_USART, str);
    delay_millis(DELAY_TIM, CMD_DELAY_MS);
    sendString(TERM_USART, str);
    delay_millis(DELAY_TIM, CMD_DELAY_MS);

    // Set CWMODE to AP + station mode
    sendString(ESP_USART, "AT+CWMODE=3\r\n");
    delay_millis(DELAY_TIM, CMD_DELAY_MS);
    readString(ESP_USART, str);
    delay_millis(DELAY_TIM, CMD_DELAY_MS);
    sendString(TERM_USART, str);
    delay_millis(DELAY_TIM, CMD_DELAY_MS);

    // Enable multiple connections
    sendString(ESP_USART, "AT+CIPMUX=1\r\n");
    delay_millis(DELAY_TIM, CMD_DELAY_MS);
    readString(ESP_USART, str);
    delay_millis(DELAY_TIM, CMD_DELAY_MS);
    sendString(TERM_USART, str);
    delay_millis(DELAY_TIM, CMD_DELAY_MS);

    // Create TCP server on port 80
    sendString(ESP_USART, "AT+CIPSERVER=1,80\r\n");
    delay_millis(DELAY_TIM, CMD_DELAY_MS);
    readString(ESP_USART, str);
    delay_millis(DELAY_TIM, CMD_DELAY_MS);
    sendString(TERM_USART, str);
    delay_millis(DELAY_TIM, CMD_DELAY_MS);

    // Connect to WiFi network
    uint8_t connect_cmd[128] = "";
    sprintf(connect_cmd,"AT+CWJAP=\"%s\",\"%s\"\r\n", SSID, PASSWORD);

    sendString(ESP_USART, connect_cmd);
    delay_millis(DELAY_TIM, CMD_DELAY_MS);
    readString(ESP_USART, str);
    delay_millis(DELAY_TIM, CMD_DELAY_MS);
    sendString(TERM_USART, str);
    delay_millis(DELAY_TIM, CMD_DELAY_MS);

    // Wait for connection
    delay_millis(DELAY_TIM, 30000);

    // Print out status
    sendString(ESP_USART, "AT+CIFSR\r\n");
    delay_millis(DELAY_TIM, CMD_DELAY_MS);
    readString(ESP_USART, str);
    delay_millis(DELAY_TIM, CMD_DELAY_MS);
    sendString(TERM_USART, str);

}

/** Send command to ESP and echo to the terminal.
    @param C-string (i.e., pointer to start of a null-terminated array
        of characters.
*/
void serveWebpage(uint8_t str []) {
    USART_TypeDef * ESP_USART = id2Port(ESP_USART_ID);
    USART_TypeDef * TERM_USART = id2Port(TERM_USART_ID);
    uint8_t cmd_response[BUFFER_SIZE] = "";

    uint32_t str_length = strlen(str)+2;
    

    memset(cmd_response, 0, BUFFER_SIZE);
    // Send to terminal what we're sending
    sendString(TERM_USART, "Serving: ");
    sendString(TERM_USART, str);
    sendString(TERM_USART, "\r\n");

    // Send HTML
    memset(cmd_response, 0, BUFFER_SIZE);
    uint8_t cmd[BUFFER_SIZE] = "";
    sprintf(cmd, "AT+CIPSEND=0,%d\r\n",str_length);
    sendString(ESP_USART, cmd);
    delay_millis(DELAY_TIM, CMD_DELAY_MS);
    readString(ESP_USART, cmd_response);
    sendString(TERM_USART, cmd_response);

    memset(cmd_response, 0, BUFFER_SIZE);
    sendString(ESP_USART, str);
    sendString(ESP_USART, "\r\n");
    delay_millis(DELAY_TIM, CMD_DELAY_MS);
    readString(ESP_USART, cmd_response);
    sendString(TERM_USART, cmd_response);

}

void play(int type, int arr) {
    TYPE = type;
    initPlayTIM(arr);
}

void TIM4_IRQHandler(void) {
    // Clear update interrupt flag
    TIM4->SR &= ~(0b1);
    if (play_index == NUM_SAMPLES) play_index = 0; //TIM4->CR1 &= ~(0b1); // RAM
    uint16_t note = *(FLASH_SECTOR_6_ADDRESS + play_index); //VOLTAGE_ARRAY[play_index]; // RAM
    if (TYPE == ALIEN) {
        note = (uint16_t) ((double) (note * (((double) sin(2*3.14*(900/40000)*play_index)+1))));
    }
    spiSendReceive12(note);
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
            TIM2->CR1 &= ~(0b1); // disable timer
            ADC1->CR2.ADON = 0; // turn off ADC
            digitalWrite(GPIOA, LED_PIN, GPIO_LOW);
            recording = 0;
        }
        else {
            initFLASH(); // RAM
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
    TIM2->SR &= ~(0b1);
    if (count < NUM_SAMPLES) {//VOLTAGE_ARRAY_SIZE) { // RAM
        configureADC();
    }
    else {
        TIM2->CR1 &= ~(0b1); // disable timer
        ADC1->CR2.ADON = 0;  // turn off ADC
        digitalWrite(GPIOA, LED_PIN, GPIO_LOW);
        recording = 0;
    }
}

/** Map ADC IRQ handler to our custom ISR
 * ADC handler turns on the DMA when the the ADC indicates that it's finished converting
 */
void ADC_IRQHandler(void){
    ADC1->SR &= ~(0b10);// clear interrupt
    // VOLTAGE_ARRAY[count] = ADC1->DR.DR; // RAM
    *(FLASH_SECTOR_6_ADDRESS + count) = (uint16_t) ADC1->DR.DR; 
    while(FLASH->SR.BSY == 1); 
    ++count;
}

/** Map USART1 IRQ handler to our custom ISR
 */
void USART1_IRQHandler(){
    USART_TypeDef * ESP_USART = id2Port(ESP_USART_ID);
    usart_ISR(ESP_USART);
}

int main(void) {
    // Configure flash latency and set clock to run at 84 MHz
    configureFlash();
    configureClock();

    // Enable GPIOA clock
    RCC->AHB1ENR.GPIOAEN = 1;
    RCC->AHB1ENR.GPIOCEN = 1;

    RCC->APB2ENR.ADC1EN = 1;
    RCC->APB2ENR.SYSCFGEN = 1;
    RCC->APB2ENR.SPI1EN = 1;
    // Initialize timers
    RCC->APB1ENR.TIM2EN = 1;
    RCC->APB1ENR.TIM3EN = 1;
    RCC->APB1ENR.TIM4EN = 1;
    RCC->APB1ENR.TIM5EN = 1;
    initDelayTIM(DELAY_TIM);
    initDelayTIM(TIM3);

    // Set up LED pin as output
    pinMode(GPIOA, LED_PIN, GPIO_OUTPUT);
    pinMode(GPIOC, BUTTON_PIN, GPIO_INPUT); // button press
    pinMode(GPIOA, GPIO_PA0, GPIO_ANALOG); // ADC pin

    // Enable interrupts globally
    __enable_irq();
    // button interrupt
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
        delay_millis(TIM3, 300);
        if (count >= NUM_SAMPLES && !playing) {
            play(NORMAL, arr_for_sampling);
            playing = 1;
        }
    }
/*
    // Configure ESP and Terminal UARTs
    USART_TypeDef * ESP_USART = initUSART(ESP_USART_ID, 115200);
    USART_TypeDef * TERM_USART = initUSART(TERM_USART_ID, 115200);

    // Configure interrupt for USART1
    *NVIC_ISER1 |= (1 << 5);
    ESP_USART->CR1.RXNEIE = 1;
    
    // Initialize ring buffer
    init_ring_buffer();
    flush_buffer();

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
                        if(look_for_substring("=PLAY", http_request)) button_req_type = REQ_PLAY;
                        else if(look_for_substring("=SLOW", http_request)) button_req_type = REQ_SLOW;
                        else if(look_for_substring("=FAST", http_request)) button_req_type = REQ_FAST;
                        else if(look_for_substring("=ALIEN", http_request)) button_req_type = REQ_ALIEN;

                        switch(button_req_type){
                            case REQ_PLAY:
                                play(NORMAL, normal);
                                sendString(TERM_USART, "Playing back regular voice.\n");
                                break;
                            case REQ_SLOW:
                                play(NORMAL, slow);
                                sendString(TERM_USART, "Playing voice slowed down.\n");
                                break;
                            case REQ_FAST:
                                play(NORMAL, fast);
                                sendString(TERM_USART, "Playing voice sped up.\n");
                                break;
                            case REQ_ALIEN:
                                play(ALIEN, normal);
                                sendString(TERM_USART, "Playing alien voice.\n");
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
                    serveWebpage("<form action=\"REQ=PLAY\"><input type=\"submit\" value = \"Playback\"></form>");
                    serveWebpage("<form action=\"REQ=SLOW\"><input type=\"submit\" value = \"Slow Down\"></form>");
                    serveWebpage("<form action=\"REQ=FAST\"><input type=\"submit\" value = \"Speed Up\"></form>");
                    serveWebpage("<form action=\"REQ=ALIEN\"><input type=\"submit\" value = \"Alien Voice\"></form>");

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
