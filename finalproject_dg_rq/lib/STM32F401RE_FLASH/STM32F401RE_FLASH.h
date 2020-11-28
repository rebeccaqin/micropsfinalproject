// STM32F401RE_FLASH.h
// Header for FLASH functions

#ifndef STM32F4_FLASH_H
#define STM32F4_FLASH_H

#include <stdint.h>

///////////////////////////////////////////////////////////////////////////////
// Definitions
///////////////////////////////////////////////////////////////////////////////

#define __IO volatile

// Base addresses for GPIO ports
#define FLASH_BASE (0x40023C00UL) // base address of RCC

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
} ACR_bits;

typedef struct {
  __IO ACR_bits ACR;      /*!< FLASH access control register,   Address offset: 0x00 */
  __IO uint32_t KEYR;     /*!< FLASH key register,              Address offset: 0x04 */
  __IO uint32_t OPTKEYR;  /*!< FLASH option key register,       Address offset: 0x08 */
  __IO uint32_t SR;       /*!< FLASH status register,           Address offset: 0x0C */
  __IO uint32_t CR;       /*!< FLASH control register,          Address offset: 0x10 */
  __IO uint32_t OPTCR;    /*!< FLASH option control register ,  Address offset: 0x14 */
  __IO uint32_t OPTCR1;   /*!< FLASH option control register 1, Address offset: 0x18 */
} FLASH1_TypeDef;

#define FLASH1 ((FLASH1_TypeDef *) FLASH_BASE)

///////////////////////////////////////////////////////////////////////////////
// Function prototypes
///////////////////////////////////////////////////////////////////////////////

void configureFlash();

#endif