#include "STM32F401RE_ADC.h"

void configureADC(){
    // turn on ADC
    ADC1->CR2.ADON = 1;
    // set the number of conversions in the sequence to 1
    ADC1->SQR1 |= (0b0000 << 20);
    // set the channel of the ADC selected as the first conversion
    // TODO: DMA2 channel 0 stream 0 
    ADC1->SQR3 |= 0b00000;
    // set single conversion mode
    ADC1->CR2.CONT = 0;
    //enable interupts after each ADC conversion to allow transfer from DR to DMA
    ADC1->CR1.EOCIE = 1;
    // turn on end of conversion selection
    ADC1->CR2.EOCS = 1;
    // turn on DMA mode
    // ADC1->CR2 |= ADC_CR2_DMA;
    //sampling time for ADC TRY THIS LATER
    // ADC->ADCSMPR2 |= 0b111;
    // enable SWSTART to start 
    ADC1->CR2.SWSTART = 1;
}
