// STM32F401RE_ADC.h
// Header for ADC functions

#ifndef STM32F4_ADC_H
#define STM32F4_ADC_H

#include <stdint.h>

///////////////////////////////////////////////////////////////////////////////
// Definitions
///////////////////////////////////////////////////////////////////////////////

#define __IO volatile

// Base addresses for ADC
#define ADC_BASE (0x40012000UL) // base address of ADC

///////////////////////////////////////////////////////////////////////////////
// Bitfield structs
///////////////////////////////////////////////////////////////////////////////

typedef struct {
  __IO uint32_t LATENCY :4;
  __IO uint32_t         :4;
  __IO uint32_t PRFTEN  :1;
  __IO uint32_t ICEN    :1;
  __IO uint32_t DCEN    :1;
  __IO uint32_t ICRST   :1;
  __IO uint32_t DCRST   :1;
  __IO uint32_t         :19;
} ADCSR_bits;

typedef struct {
  __IO uint32_t ADON    :1;
  __IO uint32_t CONT    :1;
  __IO uint32_t         :6;
  __IO uint32_t DMA     :1;
  __IO uint32_t DDS     :1;
  __IO uint32_t EOCS    :1;
  __IO uint32_t ALIGN   :1;
  __IO uint32_t         :4; 
  __IO uint32_t JEXTSEL :4;
  __IO uint32_t JEXTEN  :2;
  __IO uint32_t JSWSTART:1;
  __IO uint32_t         :1;
  __IO uint32_t EXTSEL  :4;
  __IO uint32_t EXTEN   :2;
  __IO uint32_t SWSTART :1;
  __IO uint32_t         :1;
} ADCCR2_bits;

typedef struct {
  __IO ADCSR_bits ADCSR;      /*!< ADC status register,   Address offset: 0x00 */
  __IO uint32_t ADCCR1;     /*!< ADC control register 1,              Address offset: 0x04 */
  __IO ADCCR2_bits ADCCR2;  /*!< ADC control register 2,       Address offset: 0x08 */
  __IO uint32_t ADCSMPR1;       /*!< ADC sample time register 1,           Address offset: 0x0C */
  __IO uint32_t ADCSMPR2;       /*!< ADC sample time register 2,          Address offset: 0x10 */
  __IO uint32_t ADCJOFR1;    /*!< ADC injected channel data offset register 1,  Address offset: 0x14 */
  __IO uint32_t ADCJOFR2;    /*!< ADC injected channel data offset register 2,  Address offset: 0x18 */
  __IO uint32_t ADCJOFR3;    /*!< ADC injected channel data offset register 3,  Address offset: 0x1C */
  __IO uint32_t ADCJOFR4;    /*!< ADC injected channel data offset register 4,  Address offset: 0x20 */
  __IO uint32_t ADCHTR;   /*!< ADC watchdog higher threshold register Address offset: 0x24 */
  __IO uint32_t ADCLTR;   /*!< ADC watchdog lower threshold register Address offset: 0x28 */
  __IO uint32_t ADCSQR1;   /*!< ADC regular sequence register Address offset: 0x28 */
  __IO uint32_t ADCSQR2;   /*!< ADC regular sequence register Address offset: 0x30 */
  __IO uint32_t ADCSQR3;   /*!< ADC regular sequence register Address offset: 0x34 */
  __IO uint32_t ADCJSQR;   /*!< ADC injected sequence register Address offset: 0x38 */
  __IO uint32_t ADCJDR1;    /*!< ADC injected data register 1,  Address offset: 0x3C */
  __IO uint32_t ADCJDR2;    /*!< ADC injected data register 2,  Address offset: 0x40 */
  __IO uint32_t ADCJDR3;    /*!< ADC injected data register 3,  Address offset: 0x44 */
  __IO uint32_t ADCJDR4;    /*!< ADC injected data register 4,  Address offset: 0x48 */
  __IO uint32_t ADCDR;    /*!< ADC regular data register,  Address offset: 0x4C */
  __IO uint32_t ADCCCR;    /*!< ADC common control register,  Address offset: 0x04 */
} ADC_2_TypeDef;

#define ADC ((ADC_2_TypeDef *) ADC_BASE)

///////////////////////////////////////////////////////////////////////////////
// Function prototypes
///////////////////////////////////////////////////////////////////////////////

void configureADC();

#endif