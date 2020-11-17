/*  pic32mzda.hpp | Symbols for the Microchip Pic32mzda chip. */

PIC32SYMBOL(MZDA, INTSTAT, 0xBF810020)
PIC32SYMBOL(MZDA, RPG9R,   0xBF8016a4)
PIC32SYMBOL(MZDA, U2RXR,   0xBF801470)
PIC32SYMBOL(MZDA, U6MODE,  0xBF822A00) /* UARTx mode. */
PIC32SYMBOL(MZDA, U4MODE,  0xBF822600)
PIC32SYMBOL(MZDA, U2MODE,  0xBF822200)
PIC32SYMBOL(MZDA, U6STA,   0xBF822A10) /* UARTx status and control. */
PIC32SYMBOL(MZDA, U4STA,   0xBF822610)
PIC32SYMBOL(MZDA, U2STA,   0xBF822210)
PIC32SYMBOL(MZDA, U6TXREG, 0xBF822A20) /* Transmit buffer register in which data to be transmitted is stored. */
PIC32SYMBOL(MZDA, U4TXREG, 0xBF822620)
PIC32SYMBOL(MZDA, U2TXREG, 0xBF822220)
PIC32SYMBOL(MZDA, U2RXREG, 0xBF822230) /* Recieve buffer register in which data recieved is stored. */
PIC32SYMBOL(MZDA, U4RXREG, 0xBF822630)
PIC32SYMBOL(MZDA, U6RXREG, 0xBF822A30)
PIC32SYMBOL(MZDA, U2BRG,   0xBF822240) /* Baud rate. */
PIC32SYMBOL(MZDA, U4BRG,   0xBF822640)
PIC32SYMBOL(MZDA, U6BRG,   0xBF822a40)
/* PIC32SYMBOL(MZDA, ANSELB,  0xBF860100) /​* Analog if `1`. *​/
PIC32SYMBOL(MZDA, TRISB,   0xBF860110) /​* Input if `1`. *​/
PIC32SYMBOL(MZDA, PORTB,   0xBF860120) /​* The port pins. *​/
PIC32SYMBOL(MZDA, LATB,    0xBF860130) /​* The latch. *​/
PIC32SYMBOL(MZDA, TRISH,   0xBF860710)
PIC32SYMBOL(MZDA, PORTH,   0xBF860720)
PIC32SYMBOL(MZDA, LATH,    0xBF860730) */

#pragma mark Uart

BITMASK (uint32_t) { /* UxMODE */
  PIC32MZDA_U2MODE_MODE_ON = 0b1 << 15, /* UARTx enable bit. */
  PIC32MZDA_U4MODE_MODE_ON = 0b1 << 15,
  PIC32MZDA_U6MODE_MODE_ON = 0b1 << 15,
};

BITMASK (uint32_t) { /* UxSTA */
  PIC32MZDA_U6STA_URXEN = 0b1 << 12, /* Reciever enable. */
  PIC32MZDA_U4STA_URXEN = 0b1 << 12,
  PIC32MZDA_U2STA_URXEN = 0b1 << 12,
  PIC32MZDA_U6STA_UTXEN = 0b1 << 10, /* Transmit enable. */
  PIC32MZDA_U4STA_UTXEN = 0b1 << 10,
  PIC32MZDA_U2STA_UTXEN = 0b1 << 10,
  PIC32MZDA_U6STA_UTXBF = 0b1 <<  9, /* Transmit buffer full status. */
  PIC32MZDA_U4STA_UTXBF = 0b1 <<  9,
  PIC32MZDA_U2STA_UTXBF = 0b1 <<  9,
  PIC32MZDA_U6STA_TRMT  = 0b1 <<  8, /* Transmit shift register is empty bit. */
  PIC32MZDA_U4STA_TRMT  = 0b1 <<  8,
  PIC32MZDA_U2STA_TRMT  = 0b1 <<  8,
  PIC32MZDA_U2STA_OERR  = 0b1 <<  1, /* Receive buffer overrun error status. */
  PIC32MZDA_U4STA_OERR  = 0b1 <<  1,
  PIC32MZDA_U6STA_OERR  = 0b1 <<  1,
  PIC32MZDA_U2STA_URXDA = 0b1 <<  0, /* Receive buffer data available (read-only). */
  PIC32MZDA_U4STA_URXDA = 0b1 <<  0,
  PIC32MZDA_U6STA_URXDA = 0b1 <<  0,
};

#pragma mark - Interrupts-101

enum Interrupt { Coretimer=0, Coresoftware0=1, Coresoftware1=2, ExternalInt0=3,
  Timer1=4, InputCapture1Error=5, InputCapture1=6, OutputCompare1=7, Timer2=9, OutputCompare2=12,
  Timer5=24, Timer6=28, Timer7=32, Timer8=36, Timer9=40, ADCGlobal=44, ADCFifo=45,
  ADCDigitalComparator1=46, ADCDigitalFilter1=52, ADCFault=58, ADCData0=59, 
  UsbSuspendResume=103, Crypto=107, Spi1ReceiveDone=110, Spi1TransferDone=111, 
  InputchangePortB=119, UsbGeneral=132, UsbDMA=133, DMAChannel0=134, 
  I2C2BusCollision=148, I2C2Slave=149, I2C2Master=150, Clock=166, 
  Flash=167, Prefetch=168, SQI=169, SDHC=191, GLCD=192, GPU=193, CTMU=195, 
  ADCEndOfScan=196, ADCAnalogCircuitReady=197, ADCUpdateReady=198, ADC0Early=199, 
  ADCGroupEarly=205, ADC0Warm=207, MPLLFault=215 };

BITMASK (uint32_t) { /* INTSTAT */
  PIC32MZDA_INTSTAT_SIRQ_8̄ = 0b00011111111 /* Interrupt vector. */
};

#define 🔎𝑀𝑍𝐷𝐴(symbol) 🔎(PIC32MZDA_##symbol)
MACRO Interrupt 🔎IRQ() { return Interrupt(PIC32MZDA_INTSTAT_SIRQ_8̄ & 🔎𝑀𝑍𝐷𝐴(INTSTAT)); }

#define 🔎𝑀𝑍(symbol) 🔎(PIC32MZ_##symbol)
#define 🔧𝑀𝑍(symbol) 🔧(PIC32MZ_##symbol)
#define 🎭𝑀𝑍(sym,msk,...) (uint32_t)🎭((__builtin_uint_t *)PIC32MZ_##sym, PIC32MZ_##sym##_##msk __VA_OPT__(,) __VA_ARGS__)
#define 🔧ᵗᵍᵍˡ𝑀𝑍(symbol,msk) 🔧(PIC32MZ_##symbol##INV) = PIC32MZ_##symbol##_##msk
#define 🔧0𝑀𝑍(symbol,msk) 🔧(PIC32MZ_##symbol##CLR) = PIC32MZ_##symbol##_##msk
#define 🔧1𝑀𝑍(symbol,msk) 🔧(PIC32MZ_##symbol##SET) = PIC32MZ_##symbol##_##msk
#define 🐛𝑀𝑍(symbol,msk) (🔎(PIC32MZ_##symbol)&PIC32MZ_##symbol##_##msk)
#define 🔧𝑀𝑍𝐷𝐴(symbol) 🔧(PIC32MZDA_##symbol)
#define 🎭𝑀𝑍𝐷𝐴(sym,msk,...) (uint32_t)🎭((__builtin_uint_t *)PIC32MZDA_##sym, PIC32MZDA_##sym##_##msk __VA_OPT__(,) __VA_ARGS__)
#define 🔧ᵗᵍᵍˡ𝑀𝑍𝐷𝐴(symbol,msk) 🔧(PIC32MZDA_##symbol##INV) = PIC32MZDA_##symbol##_##msk
#define 🔧0𝑀𝑍𝐷𝐴(symbol,msk) 🔧(PIC32MZDA_##symbol##CLR) = PIC32MZDA_##symbol##_##msk
#define 🔧1𝑀𝑍𝐷𝐴(symbol,msk) 🔧(PIC32MZDA_##symbol##SET) = PIC32MZDA_##symbol##_##msk
#define 🐛𝑀𝑍𝐷𝐴(symbol,msk) (🔎(PIC32MZDA_##symbol)&PIC32MZDA_##symbol##_##msk)
MACRO sexdeca 🚫Serialᵖʰʸ() { return *(sexdeca *)0x1FC54020; } /* Physical? */
MACRO sexdeca Serial₀() { return *(sexdeca *)0x9FC54020; } /* Kseg0. (Cacheable) */
MACRO sexdeca Serial₁() { return *(sexdeca *)0xAFC54020; } /* Kseg1. (Not-cacheble) */

#pragma mark - Locking/unlocking

#define PIC32MZDA_KEY1   0xAA996655
#define PIC32MZDA_KEY2   0x556699AA
#define PIC32MZDA_SYSKEY 0xBF800030

#define 🔓𝑀𝑍𝐷𝐴(syskey1,syskey2) 🔒𝑀𝑍𝐷𝐴 🔧𝑀𝑍𝐷𝐴(SYSKEY)=syskey1; 🔧𝑀𝑍𝐷𝐴(SYSKEY)=syskey2;
#define 🔒𝑀𝑍𝐷𝐴 🔧𝑀𝑍𝐷𝐴(SYSKEY)=0x0;

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

BITMASK (uint32_t) { /* MIPS debug */
  MIPS_Status_DM = 0b1 << 0
};

#define debug_break(code) asm ("sdbbp " #code)
#define break(code)       asm ("break " #code)
#define wait              asm ("wait")

#define di asm volatile ("di")
#define ei asm volatile ("ei")
#define ehb asm volatile ("ehb")
  
#pragma mark - Interrupts-201

PIC32SYMBOL(MZDA, IFS0, 0xBF810040)
PIC32SYMBOL(MZDA, IEC0, 0xBF8100c0)
PIC32SYMBOL(MZDA, IFS1, 0xBF810050)
PIC32SYMBOL(MZDA, IEC1, 0xBF8100d0)

BITMASK (uint32_t) { /* IFS1 */
  PIC32MZDA_IFS1_T1IF = 0b1<<4
};

BITMASK (uint32_t) { /* IEC1 */
  PIC32MZDA_IEC1_T1IE = 0b1<<4
};

PIC32SYMBOL(MZDA, IFS2, 0xBF810060)
PIC32SYMBOL(MZDA, IEC2, 0xBF8100e0)
PIC32SYMBOL(MZDA, IFS3, 0xBF810070)
PIC32SYMBOL(MZDA, IEC3, 0xBF8100f0)
PIC32SYMBOL(MZDA, IFS4, 0xBF810080)
PIC32SYMBOL(MZDA, IEC4, 0xBF810100)

BITMASK (uint32_t) { /* IFS4 */
  PIC32MZDA_IFS4_USBIF = 0b1 << 4,
  PIC32MZDA_IFS4_DMA0IF = 0b1 << 6,
};

BITMASK (uint32_t) { /* IEC4 */
  PIC32MZDA_IEC4_USBIE = 0b1<<4
};

PIC32SYMBOL(MZDA, IFS5, 0xBF810090)
PIC32SYMBOL(MZDA, IEC5, 0xBF810110)
PIC32SYMBOL(MZDA, IFS6, 0xBF8100a0)
PIC32SYMBOL(MZDA, IEC6, 0xBF810120)

#pragma mark - Flash and live-update

BITMASK (uint32_t) { /* NVMCON */
  PIC32MZDA_NVMCON_WR = 0b1<<15, /* Write control */
  PIC32MZDA_NVMCON_WREN = 0b1<<14, /* Write enable */
  PIC32MZDA_NVMCON_WRERR = 0b1<<13, /* Write error */
  PIC32MZDA_NVMCON_LVDERR = 0b1<<12, /* Low-voltage detect error */
  PIC32MZDA_NVMCON_PFSWAP = 0b1<<7, /* Program flash bank swap control */
  PIC32MZDA_NVMCON_BFSWAP = 0b1<<6, /* Boot flash bank swap control */
  PIC32MZDA_NVMCON_NVMOP = 0b1111<<0, /* NVM operation */
};

BITMASK (uint32_t) { /* NVMCON2 */
  PIC32MZDA_NVMCON2_SWAPLOCK = 0b11<<6, /* Program flash memory page write-protect 'Unlock'. */
};

PIC32SYMBOL(MZDA, NVMCON, 0xBF800600)
PIC32SYMBOL(MZDA, NVMCON2, 0xBF8006a0)

/* For non-live update: */

MACRO void boot₋erase₋program₋flash() { /* 0b0111 */ }

/* MACRO void program/boot₋swap₋boot₋1₋and₋2() { 🔧ᵗᵍᵍˡ𝑀𝑍𝐷𝐴(NVMCON,BFSWAP); } */

/*
 
 In a 'live update' the new code is kept in the upper half (0xBE004000):
 
 Since new code when live update is in upper half, 0xBE000001 may be assumed 
 by boot to contain a persisted PFSWAP take care of by `boot₋reprogram()`, and 
 `boot₋rollback()`. 
 
 Run only `boot`-prefixed functions from inside the boot application.
 
 */
	
MACRO int boot₋is₋program₋flash₋inverted() { return 🎭𝑀𝑍𝐷𝐴(NVMCON,PFSWAP); }
/* ⬷ a․𝘬․a `instantiated₋in₋boot₋is₋program₋flash₋inverted`. */

MACRO void 
boot₋toggle₋PFSWAP(
  uint32_t key1,
  uint32_t key2
)
{
  🔓𝑀𝑍𝐷𝐴(key1,  key2);
  🔧0𝑀𝑍𝐷𝐴(NVMCON,WREN);
  🎭𝑀𝑍𝐷𝐴(NVMCON2,SWAPLOCK, ^(__builtin_uint_t & shifted) { shifted = 0b00; });
  🔧ᵗᵍᵍˡ𝑀𝑍𝐷𝐴(NVMCON,PFSWAP);
  🔒𝑀𝑍𝐷𝐴
}

MACRO void boot₋erase₋upper₋program₋flash() { /* 0b0110 */ } /* Includes 0x0000 to 0x4000. */
MACRO void boot₋modify₋and₋copy₋shared₋program₋flash₋page(int future₋PFSWAP) { }
MACRO void boot₋program₋upper₋except₋first₋page(const char * intelhex₋text, int bytes) { }
MACRO void boot₋reprogram() { bool curr₋inv = boot₋is₋program₋flash₋inverted(); 
 boot₋toggle₋PFSWAP(PIC32MZDA_KEY1,PIC32MZDA_KEY2);
 boot₋erase₋upper₋program₋flash();
 boot₋modify₋and₋copy₋shared₋program₋flash₋page(!curr₋inv); /* Under-shared copied to upper-shared. */
 const char * intelhex₋text = (const char *)0xabcddcba; int bytes=0;
 boot₋program₋upper₋except₋first₋page(intelhex₋text,bytes); /* i.e 'PersistNVMCOM();' */ }
MACRO void boot₋rollback() { bool curr₋inv = boot₋is₋program₋flash₋inverted();
 boot₋toggle₋PFSWAP(PIC32MZDA_KEY1,PIC32MZDA_KEY2);
 boot₋modify₋and₋copy₋shared₋program₋flash₋page(!curr₋inv); 
} /* ⬷ a․𝘬․a `instantiated₋in₋program₋rollback₋boot`. */

/* See Note in Section 52.5.1 in DS60001193B. */

/* Live update requires us to persist PFSWAP in a 'newer-used by 
  neither program 1 nor program 2' memory cell. (E.g the first cell 
  in both program 1 and program 2.) */

/*
 
  The earlier referred note indicates that 'we already have three 
     program areas' by the following:
  1) Two special locations in boot where we do `boot₋reprogram` and 
      `boot₋rollback`
  2) The first page of both program 1 and program 2 is 'identical' upheld by 
      programming in software (inverted so to speak)
  3) We add a symbol `kseg1_program_flash_curr_aka_upper` to the load script
  4) Modify llvm2pic32 so that we not neccesarily have to include a new boot 
      software
  5) Update the wiki to reflect the new development
  6) Consider micromips and Revision 6.04 of Volume II-B
 
 */

#pragma mark - Reset

PIC32SYMBOL(MZDA, RSWRST, 0xBF801250)

BITMASK (uint32_t) { /* RSWRST */
  PIC32MZDA_RSWRST_SWRST = 0b1<<0, /* Software Reset Trigger. */
};

inline
void
boot₋reset(
  uint32_t key1,
  uint32_t key2
)
{
   🔓𝑀𝑍𝐷𝐴(key1, key2); 🔧1𝑀𝑍𝐷𝐴(RSWRST,SWRST);
   while (1) { uint32_t dummy = 🔎𝑀𝑍𝐷𝐴(RSWRST); } /* Any read triggers a reset. */
   /* Code execution after initiating a MCU reset has been prevented. */
}

BITMASK (uint32_t) { /* RCON */
  PIC32MZDA_RCON_HVD1V8R    = 0b1<<29, /* VDDR1V8 (DDR2) High Voltage Detect. */
  PIC32MZDA_RCON_BCFGERR    = 0b1<<27, /* Primary configuration registers error. */
  PIC32MZDA_RCON_BCFGFAIL   = 0b1<<26, /* Primary/Secondary configuration registers error. */
  PIC32MZDA_RCON_VBPOR      = 0b1<<17, /* VBPOR mode. */
  PIC32MZDA_RCON_VBAT       = 0b1<<16, /* VBAT mode. */
  PIC32MZDA_RCON_DPSLP      = 0b1<<10, /* Deep sleep mode. */
  PIC32MZDA_RCON_CMR        = 0b1<< 9, /* Configuration mismatch reset. */
  PIC32MZDA_RCON_EXTR       = 0b1<< 7, /* External reset (MCLR) pin. */
  PIC32MZDA_RCON_SWR        = 0b1<< 6, /* Software reset. */
  PIC32MZDA_RCON_DMTO       = 0b1<< 5, /* Deadman timer time-out. */
  PIC32MZDA_RCON_WDTO       = 0b1<< 4, /* Watchdog timer time-out. */
  PIC32MZDA_RCON_SLEEP      = 0b1<< 3, /* Wake from sleep. */
  PIC32MZDA_RCON_IDLE       = 0b1<< 2, /* Wake from idle. */
  PIC32MZDA_RCON_BOR        = 0b1<< 1, /* Brown-out reset. */
  PIC32MZDA_RCON_POR        = 0b1<< 0  /* Power-on reset. */
};

PIC32SYMBOL(MZDA, RCON, 0xBF801240)

BITMASK (uint32_t) { /* RNMICON */
  PIC32MZDA_RNMICON_DMTO = 0b1<<25, /* Deadman timer time-out. */
  PIC32MZDA_RNMICON_WDTO = 0b1<<24, /* Watchdog timer time-Out. */
  PIC32MZDA_RNMICON_SWNMI = 0b1<<23, /* Software NMI trigger. */
  PIC32MZDA_RNMICON_GNMI = 0b1<<19, /* General NMI. */
  PIC32MZDA_RNMICON_HLVD = 0b1<<18, /* High/Low-voltage detect. */
  PIC32MZDA_RNMICON_CF = 0b1<<17, /* Clock fail detect. */
  PIC32MZDA_RNMICON_WDTS = 0b1<<16, /* Watchdog timer time-out in sleep mode. */
  PIC32MZDA_RNMICON_NMICNT = 0b111111111111111<<0, /* NMI reset counter value. */
};

PIC32SYMBOL(MZDA, RNMICON, 0xBF801260)

BITMASK (uint32_t) { /* PWRCON */
  PIC32MZDA_PWRCON_VREGS = 0b1<<0, /* Voltage regulator stand-by enable. */
};

PIC32SYMBOL(MZDA, PWRCON, 0xBF801270)

#pragma mark - Timer 1 - Timer 9 (See also 🔎Count and 🔧Compare)

PIC32SYMBOL(MZDA,T2CON,0xBF840200)
BITMASK (uint32_t) { /* T2CON */
  PIC32MZDA_T2CON_ON = 0b1<<15, /* Timer on bit. */
};
PIC32SYMBOL(MZDA,TMR2,0xBF840210)

PIC32SYMBOL(MZDA,T5CON,0xBF840800)
BITMASK (uint32_t) { /* T5CON */
  PIC32MZDA_T5CON_ON = 0b1<<15, /* Timer on bit. */
};
PIC32SYMBOL(MZDA,TMR5,0xBF840810)

#pragma mark - Latches, buttons and light-emitting-diodes

#define IOPORT(serie,letter,start)                                          \
  PIC32SYMBOL(serie, ANSEL##letter,  start) /* Analog if `1`. */            \
  PIC32SYMBOL(serie, TRIS##letter,   start+0x10) /* Input if `1`. */        \
  PIC32SYMBOL(serie, PORT##letter,   start+0x20) /* The latched port. */    \
  PIC32SYMBOL(serie, LAT##letter,    start+0x30) /* Direct latch. */        \
  PIC32SYMBOL(serie, ODC##letter,    start+0x40) /* Open-drain=1 norm=0. */ \
  PIC32SYMBOL(serie, CNPU#letter,    start+0x50) /* CN pull-up enable. */   \
  PIC32SYMBOL(serie, CNPD#letter,    start+0x60) /* CN pull-down enable. */ \
  PIC32SYMBOL(serie, CNCON##letter,  start+0x70) /* CN control */           \
  PIC32SYMBOL(serie, CNEN##letter,   start+0x80) /* CN interrupt enable. */ \
  PIC32SYMBOL(serie, CNSTAT##letter, start+0x90) /* CN status. */           \
  PIC32SYMBOL(serie, CNNE##letter,   start+0xA0) /* CN interrupt ēnable. */ \
  PIC32SYMBOL(serie, CNF##letter,    start+0xB0) /* CN status type. */

/* For 'slew rate' control see some SRCON1x and SRCON0x. */

IOPORT(MZDA,A,0xBF860000) IOPORT(MZDA,C,0xBF860200) IOPORT(MZDA,D,0xBF860300)
IOPORT(MZDA,E,0xBF860400) IOPORT(MZDA,F,0xBF860500) IOPORT(MZDA,G,0xBF860600)
IOPORT(MZDA,H,0xBF860700) IOPORT(MZDA,J,0xBF860800) IOPORT(MZDA,K,0xBF860900)

/*  From within --<Apps>--<pic32rt> and --<Projects>:
 
 ln -sf /Users/andershansson/Projects/ia64+pic32rt/pic32mzda.hpp pic32mzda.hpp
 ln -sf /Users/andershansson/Projects/ia64+pic32rt/pic32mzda⋔₁.hpp pic32mzda⋔₁.hpp
 ln -sf /Users/andershansson/Projects/ia64+pic32rt/pic32mzda⋔₂.hpp pic32mzda⋔₂.hpp
 
 */


