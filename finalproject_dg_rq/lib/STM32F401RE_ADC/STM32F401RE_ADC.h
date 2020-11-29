// STM32F401RE_ADC.h
// Header for ADC functions

#ifndef STM32F4_ADC_H
#define STM32F4_ADC_H

#include <stdint.h> // Include stdint header

///////////////////////////////////////////////////////////////////////////////
// Definitions
///////////////////////////////////////////////////////////////////////////////

#define ADC1_BASE (0x40012000UL)
#define __IO volatile

///////////////////////////////////////////////////////////////////////////////
// Bitfield structs
///////////////////////////////////////////////////////////////////////////////

typedef struct {
  __IO uint32_t AWDCH       : 5;
  __IO uint32_t EOCIE       : 1;
  __IO uint32_t AWDIE       : 1;
  __IO uint32_t JEOCIE      : 1;
  __IO uint32_t SCAN        : 1;
  __IO uint32_t AWDSGL      : 1;
  __IO uint32_t JAUTO       : 1;
  __IO uint32_t DISCEN      : 1;
  __IO uint32_t JDISCEN     : 1;
  __IO uint32_t DISC_NUM    : 3;
  __IO uint32_t             : 6;
  __IO uint32_t JAWDEN      : 1;
  __IO uint32_t AWDEN       : 1;
  __IO uint32_t RES         : 2;
  __IO uint32_t OVRIE       : 1;
  __IO uint32_t             : 5;
} ADC_CR1_bits;

typedef struct {
  __IO uint32_t ADON        : 1;
  __IO uint32_t CONT        : 1;
  __IO uint32_t             : 6;
  __IO uint32_t DMA         : 1;
  __IO uint32_t DDS         : 1;
  __IO uint32_t EOCS        : 1;
  __IO uint32_t ALIGN       : 1;
  __IO uint32_t             : 4;
  __IO uint32_t JEXTSEL     : 4;
  __IO uint32_t JEXTEN      : 2;
  __IO uint32_t JSWSTART    : 1;
  __IO uint32_t             : 1;
  __IO uint32_t EXTSEL      : 4;
  __IO uint32_t EXTEN       : 2;
  __IO uint32_t SWSTART     : 1;
  __IO uint32_t             : 1;
} ADC_CR2_bits;

typedef struct {
  __IO uint32_t DR  : 16;
  __IO uint32_t     : 16;
} ADC_DR_bits;


typedef struct {
  __IO uint32_t SR;        /*!< ADC status register      Address offset: 0x00 */
  __IO ADC_CR1_bits CR1;   /*!< ADC control register 1   Address offset: 0x04 */
  __IO ADC_CR2_bits CR2;   /*!< ADC control register 2   Address offset: 0x08 */
  __IO uint32_t SMPR1;     /*!< ADC SMPR1                Address offset: 0x0C */
  __IO uint32_t SMPR2;     /*!< ADC SMPR2                Address offset: 0x10 */
  __IO uint32_t JOFR1;     /*!< ADC JOFR1                Address offset: 0x14 */
  __IO uint32_t JOFR2;     /*!< ADC JOFR2                Address offset: 0x18 */
  __IO uint32_t JOFR3;     /*!< ADC JOFR3                Address offset: 0x1C */
  __IO uint32_t JOFR4;     /*!< ADC JOFR4                Address offset: 0x20 */
  __IO uint32_t HTR;       /*!< ADC HTR                  Address offset: 0x24 */
  __IO uint32_t LTR;       /*!< ADC LTR                  Address offset: 0x28 */
  __IO uint32_t SQR1;      /*!< ADC SQR1                 Address offset: 0x2C */
  __IO uint32_t SQR2;      /*!< ADC SQR2                 Address offset: 0x30 */
  __IO uint32_t SQR3;      /*!< ADC SQR3                 Address offset: 0x34 */
  __IO uint32_t JSQR;      /*!< ADC JSQR                 Address offset: 0x38 */
  __IO uint32_t JDR1;      /*!< ADC JDR1                 Address offset: 0x3C */
  __IO uint32_t JDR2;      /*!< ADC JDR2                 Address offset: 0x40 */
  __IO uint32_t JDR3;      /*!< ADC JDR3                 Address offset: 0x44 */
  __IO uint32_t JDR4;      /*!< ADC JDR4                 Address offset: 0x48 */
  __IO ADC_DR_bits DR;     /*!< ADC data register        Address offset: 0x4C */
} ADC_TypeDef;

// Pointers to ADC-sized chunks of memory for each peripheral
#define ADC1 ((ADC_TypeDef *) ADC1_BASE)

void configureADC(); 

#endif
