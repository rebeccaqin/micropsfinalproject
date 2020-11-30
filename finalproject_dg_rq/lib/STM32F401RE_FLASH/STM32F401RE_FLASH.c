// STM32F401RE_FLASH.c
// Source code for FLASH functions

#include "STM32F401RE_FLASH.h"

void configureFlash() {
    FLASH->ACR.LATENCY = 2; // Set to 0 waitstates
    FLASH->ACR.PRFTEN = 1; // Turn on the ART
}

void clearFlash(){
    // Clear flash memory
    while(FLASH->SR.BSY== 1);
    FLASH->CR.SNB = 0b0110;
    FLASH->CR.SER = 1;
    FLASH->CR.STRT = 1;
    while(FLASH->SR.BSY== 1);
    FLASH->CR.SNB = 0b0111;
    FLASH->CR.SER = 1;
    FLASH->CR.STRT = 1;
    while(FLASH->SR.BSY== 1);
}

void initFLASH() {
    FLASH->KEYR = 0x45670123;
    FLASH->KEYR = 0xCDEF89AB; // enable write to CR
    FLASH->OPTKEYR = 0x08192A3B;
    FLASH->OPTKEYR = 0x4C5D6E7F; // enable write to OPTCR
    FLASH->CR.PSIZE = 0b01; // half-word (16-bits)
    FLASH->OPTCR.RDP = 0xaa;
    FLASH->OPTCR.nWRP = 0b11111110; 
    FLASH->OPTCR.SPRMOD = 0;
    clearFlash();
    FLASH->CR.PG = 1;
}
