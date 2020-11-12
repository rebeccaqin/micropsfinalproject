// STM32F401RE_ADC.c
// C file for ADC functions

#include "STM32F401RE_ADC.h"

void configureADC(){
    // turn on ADC
    ADC->ADCCR2.ADON = 1;
    // set the number of conversions in the sequence to 1
    ADC->ADCSQR1 |= (0b0000 << 20);
    // set the channel of the ADC selected as the first conversion
    ADC->ADCSQR3 |= 0b00000;
    // set single conversion mode
    ADC->ADCCR2.CONT = 0;
    //enable interupts after each ADC conversion to allow transfer from DR to DMA
    // ADC->ADCCR1 |= (1 << 5);
    // turn on end of conversion selection
    ADC->ADCCR2.EOCS = 1;
    // enable DMA requests from ADC
    ADC->ADCCR2.DMA = 1;
    //sampling time for ADC TRY THIS LATER
    // ADC->ADCSMPR2 |= 0b111;
    // enable SWSTART to start 
    ADC->ADCCR2.SWSTART = 1;
}
