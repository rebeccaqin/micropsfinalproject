// STM32F401RE_RCC.c
// Source code for RCC functions

#include "STM32F401RE_RCC.h"

void configurePLL() {
    // Set clock to 84 MHz
    // Output freq = (src_clk) * (N/M) / P
    // (8 MHz) * (336/8) / 4 = 84 MHz
    // M:8, N:336, P:4, Q:7
    // Use HSE as PLLSRC

    RCC1->CR.PLLON = 0; // Turn off PLL
    while (RCC1->CR.PLLRDY != 0); // Wait till PLL is unlocked (e.g., off)

    // Load configuration
    RCC1->PLLCFGR.PLLSRC = PLLSRC_HSE;
    RCC1->PLLCFGR.PLLM = 8;
    RCC1->PLLCFGR.PLLN = 336;
    RCC1->PLLCFGR.PLLP = 0b01; // divide by 4 
    RCC1->PLLCFGR.PLLQ = 4;

    // Enable PLL and wait until it's locked
    RCC1->CR.PLLON = 1;
    while(RCC1->CR.PLLRDY == 0);
}

void configureClock(){
    /* Configure APB prescalers
    1. Set APB2 (high-speed bus) prescaler to no division
    2. Set APB1 (low-speed bus) to divide by 2.
    */

    RCC1->CFGR.PPRE2 = 0b000;
    RCC1->CFGR.PPRE1 = 0b100;
    
    // Turn on and bypass for HSE from ST-LINK
    RCC1->CR.HSEBYP = 1;
    RCC1->CR.HSEON = 1;
    while(!RCC1->CR.HSERDY);
    
    // Configure and turn on PLL for 84 MHz
    configurePLL();

    // Select PLL as clock source
    RCC1->CFGR.SW = SW_PLL;
    while(RCC1->CFGR.SWS != 0b10);

    SystemCoreClock = 84000000;
}