// STM32F401RE_ADC.c
// C file for ADC functions

#include "STM32F401RE_ADC.h"

void configureADC(){
    // set clocks
    // set the number of conversions in the sequence to 1
    ADC->ADCSQR1 |= (0b0000 << 20);
    // set the channel of the ADC selected as the first conversion
    // TODO: DMA2 channel 0 stream 0 
    ADC->ADCSQR3 |= 0b00000;
    // set prescalar to divide by 8 to set freq of clock to ADC
    ADC->ADCCCR |= (0b11 << 16);
    // set single conversion mode
    ADC->ADCCR2.CONT = 0;
    //enable interupts after each ADC conversion to allow transfer from DR to DMA
    ADC->CR1 |= (1 << 5);
    // enable SWSTART to start 
    ADC->ADCCR2.SWSTART = 1;
}
