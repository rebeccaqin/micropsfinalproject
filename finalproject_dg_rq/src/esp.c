#include "esp.h"
#include "main.h"

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