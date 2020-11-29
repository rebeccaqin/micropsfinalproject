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
  __IO uint32_t EOP     :1;
  __IO uint32_t OPERR   :1;
  __IO uint32_t         :2;
  __IO uint32_t WRPERR  :1;
  __IO uint32_t PGAERR  :1;
  __IO uint32_t PGPERR  :1;
  __IO uint32_t PGSERR  :1;
  __IO uint32_t RDERR   :1;
  __IO uint32_t         :7;
  __IO uint32_t BSY     :1;
  __IO uint32_t         :15;
} FLASH_SR_bits;

typedef struct {
  __IO uint32_t PG      :1;
  __IO uint32_t SER     :1;
  __IO uint32_t MER     :1;
  __IO uint32_t SNB     :4;
  __IO uint32_t         :1;
  __IO uint32_t PSIZE   :2;
  __IO uint32_t         :6;
  __IO uint32_t STRT    :1;
  __IO uint32_t         :7;
  __IO uint32_t EOPIE   :1;
  __IO uint32_t ERRIE   :1;
  __IO uint32_t         :5;
  __IO uint32_t LOCK    :1;
} FLASH_CR_bits;

typedef struct {
  __IO uint32_t OPTLOCK :1;
  __IO uint32_t OPTSTRT :1;
  __IO uint32_t BOR_LEV :2;
  __IO uint32_t         :1;
  __IO uint32_t WDG_SW  :1;
  __IO uint32_t nRSTSTOP  :1;
  __IO uint32_t nRSTSTDBY :1;
  __IO uint32_t RDP     :8;
  __IO uint32_t nWRP    :8;
  __IO uint32_t         :7;
  __IO uint32_t SPRMOD  :1;
} FLASH_OPTCR_bits;

typedef struct {
  __IO ACR_bits ACR;      /*!< FLASH access control register,   Address offset: 0x00 */
  __IO uint32_t KEYR;     /*!< FLASH key register,              Address offset: 0x04 */
  __IO uint32_t OPTKEYR;  /*!< FLASH option key register,       Address offset: 0x08 */
  __IO FLASH_SR_bits SR;       /*!< FLASH status register,           Address offset: 0x0C */
  __IO FLASH_CR_bits CR;       /*!< FLASH control register,          Address offset: 0x10 */
  __IO FLASH_OPTCR_bits OPTCR;    /*!< FLASH option control register ,  Address offset: 0x14 */
  __IO uint32_t OPTCR1;   /*!< FLASH option control register 1, Address offset: 0x18 */
} FLASH_TypeDef;

#define FLASH ((FLASH_TypeDef *) FLASH_BASE)

///////////////////////////////////////////////////////////////////////////////
// Function prototypes
///////////////////////////////////////////////////////////////////////////////

void configureFlash();
void clearFlash();
void initFLASH();
#endif
