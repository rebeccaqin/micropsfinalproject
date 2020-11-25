#include "setup.h"
const uint16_t arr_for_sampling = 200;

/**
 * RCC FUNCTIONS
 */

void configureClock(){
    /* Configure APB prescalers
    1. Set APB2 (high-speed bus) prescaler to no division
    2. Set APB1 (low-speed bus) to divide by 2.
    */

    RCC->CFGR &= (0b000 << RCC_CFGR_PPRE2_Pos);
    RCC->CFGR &= (0b100 << RCC_CFGR_PPRE1_Pos);
    RCC->CFGR |= (0b100 << RCC_CFGR_PPRE1_Pos);
    
    // Turn on and bypass for HSE from ST-LINK
    RCC->CR |= RCC_CR_HSEBYP;
    RCC->CR |= RCC_CR_HSEON;
    while(!((RCC->CR >> RCC_CR_HSERDY_Pos) & 0b1));
    
    // Configure and turn on PLL for 84 MHz
    // configurePLL();

    // Select HSE as clock source
    RCC->CFGR |= RCC_CFGR_SW_HSE;
    while(((RCC->CFGR >> RCC_CFGR_SWS_Pos) & 0b1) != 0b01);
}

/**
 * FLASH FUNCTIONS
 */

void initFLASH(void) {
    FLASH->KEYR = 0x45670123;
    FLASH->KEYR = 0xCDEF89AB; // enable write to CR
    FLASH->OPTKEYR = 0x08192A3B;
    FLASH->OPTKEYR = 0x4C5D6E7F; // enable write to OPTCR
    FLASH->CR |= (0b01 << FLASH_CR_PSIZE_Pos); // half-word (16-bits)
    FLASH->OPTCR |= (0xaa << FLASH_OPTCR_RDP_Pos);
    FLASH->OPTCR |= (FLASH_OPTCR_nWRP_1 | FLASH_OPTCR_nWRP_2 | FLASH_OPTCR_nWRP_3 | FLASH_OPTCR_nWRP_4 | FLASH_OPTCR_nWRP_5 | FLASH_OPTCR_nWRP_6 | FLASH_OPTCR_nWRP_7);
    FLASH->OPTCR &= ~(0b1 << 31);
    clearFLASH();
    FLASH->CR |= FLASH_CR_PG;
}

void clearFLASH(){
    // Clear flash memory
    while(((FLASH->SR >> FLASH_SR_BSY_Pos) & 0b1) == 1);
    FLASH->CR |= (0b0001 << FLASH_CR_SNB_Pos);
    FLASH->CR |= FLASH_CR_SER;
    FLASH->CR |= FLASH_CR_STRT;
    while(((FLASH->SR >> FLASH_SR_BSY_Pos) & 0b1) == 1);
    FLASH->CR |= (0b0010 << FLASH_CR_SNB_Pos);
    FLASH->CR |= FLASH_CR_SER;
    FLASH->CR |= FLASH_CR_STRT;
    while(((FLASH->SR >> FLASH_SR_BSY_Pos) & 0b1) == 1);
    FLASH->CR |= (0b0011 << FLASH_CR_SNB_Pos);
    FLASH->CR |= FLASH_CR_SER;
    FLASH->CR |= FLASH_CR_STRT;
    while(((FLASH->SR >> FLASH_SR_BSY_Pos) & 0b1) == 1);
    FLASH->CR |= (0b0100 << FLASH_CR_SNB_Pos);
    FLASH->CR |= FLASH_CR_SER;
    FLASH->CR |= FLASH_CR_STRT;
    while(((FLASH->SR >> FLASH_SR_BSY_Pos) & 0b1) == 1);
    FLASH->CR |= (0b0101 << FLASH_CR_SNB_Pos);
    FLASH->CR |= FLASH_CR_SER;
    FLASH->CR |= FLASH_CR_STRT;
    while(((FLASH->SR >> FLASH_SR_BSY_Pos) & 0b1) == 1);
    FLASH->CR |= (0b0110 << FLASH_CR_SNB_Pos);
    FLASH->CR |= FLASH_CR_SER;
    FLASH->CR |= FLASH_CR_STRT;
    while(((FLASH->SR >> FLASH_SR_BSY_Pos) & 0b1) == 1);
    FLASH->CR |= (0b0111 << FLASH_CR_SNB_Pos);
    FLASH->CR |= FLASH_CR_SER;
    FLASH->CR |= FLASH_CR_STRT;
    while(((FLASH->SR >> FLASH_SR_BSY_Pos) & 0b1) == 1);
}
/**
 * TIMER FUNCTIONS
 */

void initADCTIM(void) {
    //enable all registers
    TIM2->EGR |= 1;
    //set PWM frequency to 40kHz
    TIM2->ARR = arr_for_sampling;
    // Configure interrupt enable on update event
    TIM2->DIER |= (TIM_DIER_UIE);
    NVIC_EnableIRQ(TIM2_IRQn);
    //enable counter
    TIM2->CR1 |= 1;
}

void initPlayTIM(int speed) {
    TIM4->EGR |= 1;
    TIM4->DIER |= (TIM_DIER_UIE);
    TIM4->ARR = arr_for_sampling;
    /*
    if (speed == fast) TIM4->ARR = arr_for_sampling/3; 
    else if (speed == slow) TIM4->ARR = arr_for_sampling*2; 
    else TIM4->ARR = arr_for_sampling; 
    */
    NVIC_EnableIRQ(TIM4_IRQn);
    //enable counter
    TIM4->CR1 |= 1;
}

void initDelayTIM(TIM_TypeDef * TIMx){
    // Set prescaler to give 1 µs time base
    uint32_t psc_div = (uint32_t) ((SYSTEMCORECLOCK/1e6)-1);

    // Set prescaler division factor
    TIMx->PSC = (psc_div - 1);
    // Generate an update event to update prescaler value
    TIMx->EGR |= 1;
    // Enable counter
    TIMx->CR1 |= 1; // Set CEN = 1
}

void delay_millis(TIM_TypeDef * TIMx, uint32_t ms){
  TIMx->ARR = ms*1000;// Set timer max count
  TIMx->EGR |= 1;     // Force update
  TIMx->SR &= ~(0x1); // Clear UIF
  TIMx->CNT = 0;      // Reset count

  while(!(TIMx->SR & 1)); // Wait for UIF to go high
}

void delay_micros(TIM_TypeDef * TIMx, uint32_t us){
  TIMx->ARR = us;// Set timer max count
  TIMx->EGR |= 1;     // Force update
  TIMx->SR &= ~(0x1); // Clear UIF
  TIMx->CNT = 0;      // Reset count

  while(!(TIMx->SR & 1)); // Wait for UIF to go high
}

/**
 * ADC FUNCTIONS
 */ 

void configureADC(){
    // turn on ADC
    ADC1->CR2 |= ADC_CR2_ADON;
    // set the number of conversions in the sequence to 1
    ADC1->SQR1 |= (0b0000 << 20);
    // set the channel of the ADC selected as the first conversion
    // TODO: DMA2 channel 0 stream 0 
    ADC1->SQR3 |= 0b00000;
    // set single conversion mode
    ADC1->CR2 |= (0 << ADC_CR2_CONT_Pos); 
    //enable interupts after each ADC conversion to allow transfer from DR to DMA
    ADC1->CR1 |= ADC_CR1_EOCIE;
    // turn on end of conversion selection
    ADC1->CR1 |= ADC_CR2_EOCS;
    // turn on DMA mode
    // ADC1->CR2 |= ADC_CR2_DMA;
    //sampling time for ADC TRY THIS LATER
    // ADC->ADCSMPR2 |= 0b111;
    // enable SWSTART to start 
    ADC1->CR2 |= ADC_CR2_SWSTART;
}

/**
 * GPIO FUNCTIONS
 */

void pinMode(GPIO_TypeDef* GPIO_PORT_PTR, int pin, int function) {
    switch(function) {
        case GPIO_INPUT:
            GPIO_PORT_PTR->MODER &= ~(0b11 << 2*pin);
            break;
        case GPIO_OUTPUT:
            GPIO_PORT_PTR->MODER |= (0b1 << 2*pin);
            GPIO_PORT_PTR->MODER &= ~(0b1 << (2*pin+1));
            break;
        case GPIO_ALT:
            GPIO_PORT_PTR->MODER &= ~(0b1 << 2*pin);
            GPIO_PORT_PTR->MODER |= (0b1 << (2*pin+1));
            break;
        case GPIO_ANALOG:
            GPIO_PORT_PTR->MODER |= (0b11 << 2*pin);
            break;
    }
}

int digitalRead(GPIO_TypeDef* GPIO_PORT_PTR, int pin) {
    return ((GPIO_PORT_PTR->IDR) >> pin) & 1;
}

void digitalWrite(GPIO_TypeDef* GPIO_PORT_PTR, int pin, int val) {
    if(val == 1){
        GPIO_PORT_PTR->ODR |= (1 << pin);
    }
    else if(val == 0){
        GPIO_PORT_PTR->ODR &= ~(1 << pin);
    }
    
}

void togglePin(GPIO_TypeDef* GPIO_PORT_PTR,int pin) {
    // Use XOR to toggle
    GPIO_PORT_PTR->ODR ^= (1 << pin);
}

/**
 * SPI FUNCTIONS
 */
/* Enables the SPI peripheral and intializes its clock speed (baud rate), polarity, and phase.
 *    -- br: (0b000 - 0b111). The SPI clk will be the master clock / 2^(BR+1).
 *    -- cpol: clock polarity (0: inactive state is logical 0, 1: inactive state is logical 1).
 *    -- ncpha: clock phase (0: data changed on leading edge of clk and captured on next edge, 
 *          1: data captured on leading edge of clk and changed on next edge)
 * Note: the SPI mode register is set with the following unadjustable settings:
 *    -- Master mode
 *    -- Fixed peripheral select
 *    -- Chip select lines directly connected to peripheral device
 *    -- Mode fault detection enabled
 *    -- WDRBT disabled
 *    -- LLB disabled
 *    -- PCS = 0000 (Peripheral 0 selected), means NPCS[3:0] = 1110
 * Refer to the datasheet for more low-level details. */ 
void spiInit(uint32_t br, uint32_t cpol, uint32_t cpha) {
    // Turn on GPIOA and GPIOB clock domains (GPIOAEN and GPIOBEN bits in AHB1ENR)
    RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN);

    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN; // Turn on SPI1 clock domain (SPI1EN bit in APB2ENR)

    // Initially assigning SPI pins
    pinMode(GPIOA, 5, GPIO_ALT); // PA5, Arduino D13, SPI1_SCK
    pinMode(GPIOA, 7, GPIO_ALT); // PA7, Arduino D11, SPI1_MOSI
    pinMode(GPIOA, 1, GPIO_OUTPUT); // PA1, Arduino D10, Manual CS

    // Set to AF05 for SPI alternate functions
    GPIOA->AFR[0] |= ((5 << (4*5)) | (5 << (4*6)) | (5 << (4*7)));

    SPI1->CR1 &= ~(SPI_CR1_BR);
    SPI1->CR1 |= (br << SPI_CR1_BR_Pos); // Set the clock divisor
    SPI1->CR1 &= ~(SPI_CR1_CPOL);
    SPI1->CR1 |= (cpol << SPI_CR1_CPOL_Pos); // Set the polarity
    SPI1->CR1 &= ~(SPI_CR1_CPHA);
    SPI1->CR1 |= (cpha << SPI_CR1_CPHA_Pos); // Set the phase
    SPI1->CR1 &= ~(SPI_CR1_LSBFIRST); // Set least significant bit first
    SPI1->CR1 |= SPI_CR1_DFF;      // Set data format to 16 bits
    SPI1->CR1 &= ~(SPI_CR1_SSM); // Turn off software slave management
    SPI1->CR2 |= SPI_CR2_SSOE;  // Set the NSS pin to output mode
    SPI1->CR1 |= SPI_CR1_MSTR;  // Put SPI in master mode
    SPI1->CR1 |= SPI_CR1_SPE;   // Enable SPI
}



/* Transmits a short (2 bytes) over SPI and returns the received short.
 *    -- send: the short to send over SPI
 *    -- return: the short received over SPI */
uint16_t spiSendReceive12(uint16_t send) {
    while(((SPI1->SR >> SPI_SR_TXE_Pos) & 0b1) != 0b1){} // wait until the transmission buffer is empty

    digitalWrite(GPIOA, 1, 0);
    SPI1->CR1 |= SPI_CR1_SPE;
    // apply a mask so only the first 12 
    send = (0b0001 << 12) | (send & 0xFFF);
    SPI1->DR = send;
    
    while(!((SPI1->SR >> SPI_SR_RXNE_Pos) & 0b1));
    uint16_t rec = SPI1->DR;
    
    SPI1->CR1 &= ~(SPI_CR1_SPE);
    digitalWrite(GPIOA, 1, 1);

    return rec;
}

/**
 * USART FUNCTIONS
 */


USART_TypeDef * id2Port(uint32_t USART_ID){
    USART_TypeDef * USART;
    switch(USART_ID){
        case(USART1_ID) :
            USART = USART1;
            break;
        case(USART2_ID) :
            USART = USART2;
            break;
        default :
            USART = 0;
    }
    return USART;
}

USART_TypeDef * initUSART(uint8_t USART_ID, uint32_t baud_rate){
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; // Enable GPIO port A

    USART_TypeDef * USART = id2Port(USART_ID);

    switch(USART_ID){
        case USART1_ID :
            RCC->APB2ENR |= RCC_APB2ENR_USART1EN; // Set USART1EN

            // Configure pin modes as ALT function
            pinMode(GPIOA, GPIO_PA9, GPIO_ALT); // TX
            pinMode(GPIOA, GPIO_PA10, GPIO_ALT); // RX

            GPIOA->AFR[1] |= (0b111 << 4*2) | (0b111 << 4*1);
            break;
        case USART2_ID :
            RCC->APB1ENR |= (1 << 17); // Set USART1EN

            // Configure pin modes as ALT function
            pinMode(GPIOA, GPIO_PA2, GPIO_ALT); // TX
            pinMode(GPIOA, GPIO_PA3, GPIO_ALT); // RX

            // Configure correct alternate functions (AF07)
            GPIOA->AFR[0] |= ((0b111 << (4*2)) | (0b111 << (4*3)));
            break;
    }

    USART->CR1 |= USART_CR1_UE; // Enable USART
    USART->CR1 &= ~(USART_CR1_M);// M=0 corresponds to 8 data bits
    USART->CR2 &= (0b00 << USART_CR2_STOP_Pos); // 0b00 corresponds to 1 stop bit
    USART->CR1 &= ~(USART_CR1_OVER8);  // Set to 16 times sampling freq

    // Set baud rate to 9.6 kbps
    // Tx/Rx baud = (f_CK)/(8*(2-OVER8)*USARTDIV) = Tx/Rx baud = (f_CK)/(16*USARTDIV)
    // f_CK = 84e6 Hz on APB2 (USART1) or 42e6 on APB1 (USART2)
    // USARTDIV = 546.875 should be in BRR
    // 546 = 0x0222
    // 0.875 = 7/8 = 0b1110
    // DIV_Mantissa = 0x222
    // DIV_Fraction = 0b111

    if(USART_ID == USART1_ID){
        if (baud_rate == 9600){
            USART->BRR |= (546 << USART_BRR_DIV_Mantissa_Pos);
            USART->BRR |= (0b1110 << USART_BRR_DIV_Fraction_Pos);
        } else {
            // Default to 115200
            USART->BRR |= (45 << USART_BRR_DIV_Mantissa_Pos);
            USART->BRR |= (0b1001 << USART_BRR_DIV_Fraction_Pos);
        }

    }
    else if (USART_ID == USART2_ID){
        if (baud_rate == 9600){
            USART->BRR |= (273 << USART_BRR_DIV_Mantissa_Pos);
            USART->BRR |= (0b0111 << USART_BRR_DIV_Fraction_Pos);
        } else {
            // Default to 115200
            USART->BRR |= (22 << USART_BRR_DIV_Mantissa_Pos);
            USART->BRR |= (0b1101 << USART_BRR_DIV_Fraction_Pos);
        }
    }

    USART->CR1 |= USART_CR1_TE; // Enable transmission
    USART->CR1 |= USART_CR1_RE; // Enable reception

    return USART;
}

void sendChar(USART_TypeDef * USART, uint8_t data){
    while(!((USART->SR >> USART_SR_TXE_Pos) & 0b1));
    USART->DR = data;
    while(!((USART->SR >> USART_SR_TC_Pos) & 0b1));
}

void sendString(USART_TypeDef * USART, uint8_t * charArray){

    uint32_t i = 0;
    do{
        sendChar(USART, charArray[i]);
        i++;
    }
    while(charArray[i] != 0);
}

uint8_t readChar(USART_TypeDef * USART){
    if(is_data_available()){
        uint8_t data = read_char_buffer();
        return data;
    } else {
        return 0;
    }

}

void readString(USART_TypeDef * USART, uint8_t * charArray){
    uint32_t i = 0;
    do{
        charArray[i] = readChar(USART);
        i++;
    }
    while(is_data_available());
}

/**
 * Ring Buffer
 */

#include <string.h>

ring_buffer rx_buffer = {{0}, 0, 0};

ring_buffer * _rx_buffer;

void init_ring_buffer(void){
    _rx_buffer = &rx_buffer;
}

void store_char(int8_t c, ring_buffer * buffer){
    uint32_t i = (uint32_t)(buffer->head + 1) % UART_BUFFER_SIZE;

    if(i != buffer->tail) {
        buffer->buffer[buffer->head] = c;
        buffer-> head = i;
    }
}

uint8_t read_char_buffer(void){
    if(_rx_buffer->head == _rx_buffer->tail) return -1;
    else {
        int8_t c = _rx_buffer->buffer[_rx_buffer->tail];
        _rx_buffer->tail = (uint32_t)(_rx_buffer->tail + 1) % UART_BUFFER_SIZE;
        return c;
    }
}

uint8_t is_data_available(void){
    return (uint8_t)((_rx_buffer->head - _rx_buffer->tail) % UART_BUFFER_SIZE);
}

void flush_buffer(void){
    memset(_rx_buffer->buffer, '\0', UART_BUFFER_SIZE);
    _rx_buffer->head = 0;
    _rx_buffer->tail = 0;
}

void usart_ISR(USART_TypeDef * USART){
    if(((USART->SR >> USART_SR_RXNE_Pos) & 0b1) && ((USART->CR1 >> USART_CR1_RXNEIE_Pos) & 0b1)){
        uint8_t c = USART->DR;
        store_char(c, _rx_buffer);
        return;
    }
}

uint8_t look_for_substring (uint8_t *str, uint8_t *buffertolookinto){
    uint32_t stringlength = strlen(str);
	volatile int bufferlength = strlen(buffertolookinto);
	uint32_t so_far = 0;
	uint32_t indx = 0;
repeat:
	while (str[so_far] != buffertolookinto[indx]) {
        if(indx >= bufferlength) return 0;
        indx++;
    }

	if (str[so_far] == buffertolookinto[indx])
	{
		while (str[so_far] == buffertolookinto[indx])
		{
			so_far++;
			indx++;
		}
	}

	else
	{
		so_far = 0;
		if (indx >= bufferlength) return 0;
		goto repeat;
	}

	if (so_far == stringlength) return 1;
	else return 0;
}
/**
 * ESP FUNCTIONS
 */

/** Initialize the ESP and print out IP address to terminal
 */
void initESP8266(USART_TypeDef * ESP_USART, USART_TypeDef * TERM_USART){
    uint8_t volatile str[BUFFER_SIZE] = "";

    // Disable echo
    sendString(ESP_USART, "ATE0\r\n");
    delay_millis(DELAY_TIM, CMD_DELAY_MS);
    readString(ESP_USART, str);
    sendString(TERM_USART, str);
    delay_millis(DELAY_TIM, CMD_DELAY_MS);

    // Enable multiple connections
    sendString(ESP_USART, "AT+CIPMUX=1\r\n");
    delay_millis(DELAY_TIM, CMD_DELAY_MS);
    readString(ESP_USART, str);
    sendString(TERM_USART, str);
    delay_millis(DELAY_TIM, CMD_DELAY_MS);

    // Create TCP server on port 80
    sendString(ESP_USART, "AT+CIPSERVER=1,80\r\n");
    delay_millis(DELAY_TIM, CMD_DELAY_MS);
    readString(ESP_USART, str);
    sendString(TERM_USART, str);
    delay_millis(DELAY_TIM, CMD_DELAY_MS);

    // Connect to WiFi network
    uint8_t connect_cmd[128] = "";
    sprintf(connect_cmd,"AT+CWJAP=\"%s\",\"%s\"\r\n", SSID, PASSWORD);

    sendString(ESP_USART, connect_cmd);
    delay_millis(DELAY_TIM, CMD_DELAY_MS);
    readString(ESP_USART, str);
    sendString(TERM_USART, str);
    delay_millis(DELAY_TIM, CMD_DELAY_MS);

    // Wait for connection
    delay_millis(DELAY_TIM, 10000);

    // Print out status
    sendString(ESP_USART, "AT+CIFSR\r\n");
    delay_millis(DELAY_TIM, CMD_DELAY_MS);
    readString(ESP_USART, str);
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