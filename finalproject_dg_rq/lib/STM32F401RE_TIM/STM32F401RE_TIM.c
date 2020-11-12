// STM32F401RE_TIM.c
// TIM functions

#include "STM32F401RE_TIM.h"
#include "STM32F401RE_RCC.h"

void initTIM(TIM_2_TypeDef * TIMx){
  // Set prescaler to give 1 µs time base
  uint32_t psc_div = (uint32_t) ((SystemCoreClock/1e6)-1);

  // Set prescaler division factor
  TIMx->PSC = (psc_div - 1);
  // Generate an update event to update prescaler value
  TIMx->EGR |= 1;
  // Enable counter
  TIMx->CR1 |= 1; // Set CEN = 1
}

void delay_millis(TIM_2_TypeDef * TIMx, uint32_t ms){
  TIMx->ARR = ms*1000;// Set timer max count
  TIMx->EGR |= 1;     // Force update
  TIMx->SR &= ~(0x1); // Clear UIF
  TIMx->CNT = 0;      // Reset count

  while(!(TIMx->SR & 1)); // Wait for UIF to go high
}

void delay_micros(TIM_2_TypeDef * TIMx, uint32_t us){
  TIMx->ARR = us;// Set timer max count
  TIMx->EGR |= 1;     // Force update
  TIMx->SR &= ~(0x1); // Clear UIF
  TIMx->CNT = 0;      // Reset count

  while(!(TIMx->SR & 1)); // Wait for UIF to go high
}

void initTIM2() {
    initTIM(TIM2);
    //enable all registers
    TIM2->EGR |= 1;
  
    //set one pulse mode by setting OPM bit
    // TIM2->CR1 |= (1 << 3);
    
    //set PWM frequency to 75000 clock cycles
    // if we need duty cycle: set TIM2->CCR1
    // 15 cycles per ADC conversion and buffer size 5000
    TIM2->ARR = 75000;
    
    // Configure interrupt enable on update event
    TIM2->DIER |= (TIM_DIER_UIE);
    NVIC_EnableIRQ(TIM2_IRQn);
    //enable counter
    TIM2->CR1 |= 1;
}