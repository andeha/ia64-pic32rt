/* pic32mm.hpp | energy-focused, pre-packaged and early. */

PIC32SYMBOL(MM, U1MODE, 0xBF806000)
PIC32SYMBOL(MM, U1MODE, 0xBF806000)

/*
   
   A Curiosity equipped with PIC32MM0256GPM064 in a TQFP-64 (not UQF-28)
   
   S2 - RC10 pin 45. (OCM3F)
   S3 - RC4 pin 36. (OCM1E/INT3)
   S1 - TMS/RP14/RB9 pin 49. (SDA1/INT2/)
   POT - RC8 (AN14/LVDIN/C2INC/)
   OCM3E/RC15 - Blue/RC3 - Green/RD1 - Red
   RD3 Red-led1, RC13 Red-led2
   J6 current measurement. (3V3)
   AN2/C2INB/VMIO/RP13/CN4/RB2: VDDSENSE
   AN3/C2INA/VPIO/CN5/RB3: VPPSENSE
   RP22/PMBE/CN52/RD3 PKMISO
   RP25/PMWR/CN13/RD4 PKMOSI
   Micro B-U7-USB D- and D+ - RB10/RB11 and VBUS/RB6
   
   RTCC/RA15 available on RA15_GPIOXA.
   
   SOT-23 and SC-70.
   
 */

#define 🔎𝑀𝑀(symbol) 🔎(PIC32MM_##symbol)
#define 🔧𝑀𝑀(symbol) 🔧(PIC32MM_##symbol)
#define 🎭𝑀𝑀(sym,msk,...) 🎭((__builtin_uint_t *)(PIC32MM_##sym), PIC32MM_##sym##_##msk __VA_OPT__(,) __VA_ARGS__)
#define 🔧ᵗᵍᵍˡ𝑀𝑀(symbol,msk) 🔧(PIC32MM_##symbol##INV) = PIC32MM_##symbol##_##msk
#define 🔧0𝑀𝑀(symbol,msk) 🔧(PIC32MM_##symbol##CLR) = PIC32MM_##symbol##_##msk
#define 🔧1𝑀𝑀(symbol,msk) 🔧(PIC32MM_##symbol##SET) = PIC32MM_##symbol##_##msk
#define 🐛𝑀𝑀(symbol,msk) (🔎(PIC32MM_##symbol)&PIC32MM_##symbol##_##msk)

MACRO __builtin_uint_t 🔎Count() { __builtin_uint_t val; asm 
 volatile("mfc0 %0, $9, 0; nop" : "=r" (val)); return val; }

MACRO void 🔧Status(__builtin_uint_t value) { asm
 volatile("mtc0 %0, $12, 0; nop" : : "r" (value)); }
MACRO __builtin_uint_t 🔎Status() { __builtin_uint_t val; asm
 volatile("mfc0 %0, $12, 0; nop" : "=r" (val)); return val; }
MACRO __builtin_uint_t 🔎IntCtl() { __builtin_uint_t val; asm
 volatile("mfc0 %0, $12, 1; nop" : "=r" (val)); return val; }

BITMASK (uint32_t) { /* MIPS-status. */
  MIPS_Status_IE  = 0b1 << 0, /* Interrupt enable */
  MIPS_Status_EXL = 0b1 << 1, /* Exception level */
  MIPS_Status_ERL = 0b1 << 2, /* Error level */
  MIPS_Status_UM  = 0b1 << 4, /* User mode */
  MIPS_Status_SR = 0b1 << 20, /* Soft reset */
  PIC32_Status_TS = 0b1 << 21, /* TLB shutdown control */
  MIPS_Status_BEV = 0b1 << 22, /* Bootstrap exception vector control. (Location of exception vector: 1=Bootstrap, 0=Normal.) */
  OPTMIPS_Status_MX = 0b1 << 24, /* MIPS dsp resource enable */
  MIPS_Status_RP  = 0b1 << 27, /* Reduced power */
};

BITMASK (uint32_t) { /* MIPS cause */
  MIPS_Cause_DSPDis = 0x1a /* DSP module state disabled 
   Exception [see 'MIPS® Architecture for Programmers Volume IV-e: MIPS® 
   DSP Module for MIPS32TM Architecture'.] */
};

#define wait              asm ("wait")

namespace Pic32mm_nousb { enum Interrupt { 
Coretime=0, External0=3, External1, External2, External3, External4, 
Timer1=11, Comparator1, Comparator2, RtccAlarm, Adc, Crc, 
VoltageDetect, Spi1Error=20, Spi1Tx, Spi1Rx, Uart1Rx, Uart1Tx, Uart1Err, 
Spi2Err=37, Spi2Tx, Spi2Rx, Uart2Rx, Uart2Tx, Uart2Err 
}; }

PIC32SYMBOL(MM, INTCON, 0xBF80F000)

BITMASK (uint32_t) { /* INTCON */
 PIC32MM_IEC1_VS = 0b1111111<<16
};

#define IOPORT(serie,letter,start)                                          \
  PIC32SYMBOL(serie, ANSEL##letter,  start) /* Analog if `1`. */            \
  PIC32SYMBOL(serie, TRIS##letter,   start+0x10) /* Input if `1`. */        \
  PIC32SYMBOL(serie, PORT##letter,   start+0x20) /* The latched port. */    \
  PIC32SYMBOL(serie, LAT##letter,    start+0x30) /* Direct latch. */        \
  PIC32SYMBOL(serie, ODC##letter,    start+0x40) /* Open-drain=1 norm=0. */ \
  PIC32SYMBOL(serie, CNPU##letter,   start+0x50) /* CN pull-up enable. */   \
  PIC32SYMBOL(serie, CNPD##letter,   start+0x60) /* CN pull-down enable. */ \
  PIC32SYMBOL(serie, CNCON##letter,  start+0x70) /* CN control */           \
  PIC32SYMBOL(serie, CNEN##letter,   start+0x80) /* CN interrupt enable. */ \
  PIC32SYMBOL(serie, CNSTAT##letter, start+0x90) /* CN status. */           \
  PIC32SYMBOL(serie, CNNE##letter,   start+0xA0) /* CN interrupt ēnable. */ \
  PIC32SYMBOL(serie, CNF##letter,    start+0xB0) /* CN status type. */

IOPORT(MM,0xBF802800,C) IOPORT(MM,0xBF802900,D)

#define PIC32MM_KEY1 0xAA996655
#define PIC32MM_KEY2 0x556699AA
#define PIC32MM_SYSKEY 0xBF800030

#define 🔒𝑀𝑀 🔧𝑀𝑀(SYSKEY)=0x0;
#define 🔓𝑀𝑀(syskey1,syskey2) 🔒𝑀𝑀 🔧𝑀𝑀(SYSKEY)=syskey1; 🔧𝑀𝑀(SYSKEY)=syskey2;

