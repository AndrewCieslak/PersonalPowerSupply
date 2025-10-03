
/* #define (also called a 'macro') is simply a text substitution that happens during preprocessor phase, before the actual compiler. And it is obviously not typed.
constexpr on the other hand, happens during actual parsing. And it is indeed typed. Comes without saying that, wherever you can, using constexpr is a bit safer.
uint8_t is unsigned 8 bit integer.



"#pragma once" just replaces: 
#ifndef PINS_H 
#define PINS_H
.
.
#endif

as an include-guard mechanism to make sure if multiple .cpp files have the include, it's only defined once.

PC2, PC1, etc. are just ints, defined in the <avr/io.h> file by their bit positions in their respective port. 
Since we already define the port prior to each definition, the PC, PB, etc. is just for readability.
Thus "constexpr uint8_t" does the same as "#define", as long as "uint8_t" represents the value.

bit numbering notation [X:Y] just means
"the bits that are from X down to Y, including X and Y"
from MSB->LSB, for example ADCH[7:0] references all 8 bits of the register,
it's used to explain how the bits are packed into the register's bit positions
in a 16-bit register like ADC, 
this is important to configure exactly how it should be read

// PORT |= (1<<PIN); to write PIN to 1
// PORT |= ~(1<<PIN); to write PIN to 0


I really don't need to be redefining register names and bit names internally if they mean the same thing and won't be parameterized,
but for each peripheral where it might change, that is good practice. Otherwise, just for my own understanding and readability.





When the hardware detects a change on any enabled pin (a bit flips from 0→1 or 1→0),
it sets the Pin Change Interrupt Flag and vectors to your ISR(PCINTx_vect) handler.
Your ISR executes immediately, letting you read the new pin states (or run your decoding logic) at the exact moment of the change.
Then it returns, and the CPU resumes whatever it was doing.

//   On ATmega328P only channels 0–7 can measure to GND,
//   with the other ADC channels in bit locations 15-8 being configured
//   for differential measurements and optional gain.




display doesn't work, dac output doesn't work, encoder i think works.
*/

#pragma once

#include <stdint.h>
#include <avr/io.h>

//Serial monitor for debugging

// UCSR0B – Control and Status Register B
// Bit  TXEN0: Transmitter Enable
// Bit  RXEN0: Receiver Enable
// Bit  UCSZ02: Character Size bit 2 (combined with UCSZ01/00)

// UCSR0C – Control and Status Register C
// Bit  UCSZ01: Character Size bit 1
// Bit  UCSZ00: Character Size bit 0
// (plus parity and stop‐bit settings)

//3-Wire interface
#define SPI_PORT DDRB
#define MOSI PB3
#define MISO PB4 //probs not necessary
#define SCK PB5

//ADC pins
#define VOUT_ADC PC0
#define DACV_ADC PC5
#define ADC_DDR DDRC //port C for ATmega328P GND referenced ADC pins

// Pin and Port definitions for AD420 (Not including MOSI, MISO, SCK)
#define CS_AD420 PB1
#define CLR_AD420 PB0

#define AD420_CS_PORT PORTB
#define AD420_CS_DDR  DDRB

#define AD420_CLR_PORT PORTB
#define AD420_CLR_DDR  DDRB

// Display Pins and Ports (Not including MOSI, MISO, SCK)
#define CS_DISP PB2
#define DISP_PORT PORTB
#define DISP_DDR  DDRB
#define DISP_PORT_CS PORTB
#define DISP_DDR_CS DDRB

//Encoder Pins and Ports
#define CLK_ENCODER PD3
#define DT_ENCODER PD2

#define ENC_PIN PIND
#define ENC_DDR DDRD
#define ENC_PULLUP PORTD


// ADMUX

// REGISTERS:
// 8-bit ADC Multiplexer Selection Register: selects voltage reference, result adjustment, and input channel
#define ADC_MUX_REG  ADMUX

// BITS:
// bit 7 REFS1: Reference Selection bit 1  
//   together with REFS0, selects VREF:  
//     00 = AREF, external cap at AREF pin  
//     01 = AVcc with external cap at AREF  
//     11 = Internal 1.1 V reference  
#define ADC_REF_SEL1 REFS1

// bit 6 REFS0: Reference Selection bit 0  
//   see REFS1 description for voltage reference mapping  
#define ADC_REF_SEL0 REFS0

// bit 5 ADLAR: ADC Left Adjust Result  
//   0 = right-adjust the 10-bit result in ADCH/ADCL  
//   1 = left-adjust so the high 8 bits are in ADCH  
#define ADC_LEFT_ADJ ADLAR

// bit 4 reserved 

// bit 3 MUX3: Analog Channel Selection bit 3  
#define ADC_MUX3 MUX3

// bit 2 MUX2: Analog Channel Selection bit 2  
#define ADC_MUX2 MUX2

// bit 1 MUX1: Analog Channel Selection bit 1  
#define ADC_MUX1 MUX1

// bit 0 MUX0: Analog Channel Selection bit 0  
#define ADC_MUX0 MUX0

// ADC Control and status register A

// REGISTERS:
// 8-bit ADC Control and Status Register A:  
//   – ADEN: enable/disable ADC  
//   – ADSC: start a conversion  
//   – ADATE: enable auto-triggering  
//   – ADIF: conversion complete flag  
//   – ADIE: ADC interrupt enable  
//   – ADPS[2:0]: ADC clock prescaler select bits  
#define ADC_CTRL_STATUS_REG  ADCSRA

// BITS:
// bit 7 ADEN: ADC Enable  
//   1 = enable ADC; 0 = disable (saves power)
#define ADC_ENABLE ADEN

// bit 6 ADSC: ADC Start Conversion  
//   1 = begin a single conversion (cleared when conversion completes)
#define ADC_START_CONV ADSC

// bit 5 ADATE: ADC Auto Trigger Enable  
//   1 = new conversions triggered automatically by selected source
#define ADC_AUTO_TRIGGER     ADATE

// bit 4 ADIF: ADC Interrupt Flag  
//   set to 1 by hardware when conversion completes; cleared by writing a 1 (write-1-to-clear-bit)
#define ADC_INT_FLAG         ADIF

// bit 3 ADIE: ADC Interrupt Enable  
//   1 = enable ADC conversion complete interrupt
#define ADC_INT_ENABLE       ADIE

// bit 2 ADPS2: ADC Prescaler Select bit 2  
// bit 1 ADPS1: ADC Prescaler Select bit 1  
// bit 0 ADPS0: ADC Prescaler Select bit 0  
//   together select division factor of system clock for ADC:  
//   000 = ÷2, 001 = ÷2, 010 = ÷4, 011 = ÷8, 100 = ÷16, 101 = ÷32, 110 = ÷64, 111 = ÷128
#define ADC_PRESCALER2       ADPS2
#define ADC_PRESCALER1       ADPS1
#define ADC_PRESCALER0       ADPS0

// 8-bit ADC Control and Status Register B: selects auto‐trigger source and comparator mux

// REGISTERS:
#define ADC_CTRL_STATUS_REG_B  ADCSRB

// BITS:

// bit 7 reserved, must be 0.

// bit 6 ACME: Analog Comparator Multiplexer Enable  
//   0 = disable the analog comparator input multiplexer (default)  
//   1 = enable so the comparator output can be sampled by the ADC  
#define ADC_COMP_MUX_ENABLE ACME

// bits 5:3 — Reserved; must be written as 0

// bits 2:0 ADTS2..0: ADC Auto Trigger Source  
// together choose what event starts each new conversion when ADATE=1

/*
constexpr uint8_t ADC_TRIG_FREE_RUNNING       = (0 << ADTS2)|(0 << ADTS1)|(0 << ADTS0);  // 000  
constexpr uint8_t ADC_TRIG_ANALOG_COMP        = (0 << ADTS2)|(0 << ADTS1)|(1 << ADTS0);  // 001  
constexpr uint8_t ADC_TRIG_EXT_INT0           = (0 << ADTS2)|(1 << ADTS1)|(0 << ADTS0);  // 010  
constexpr uint8_t ADC_TRIG_TIMER0_COMP_MATCH  = (0 << ADTS2)|(1 << ADTS1)|(1 << ADTS0);  // 011  
constexpr uint8_t ADC_TRIG_TIMER0_OVF         = (1 << ADTS2)|(0 << ADTS1)|(0 << ADTS0);  // 100  
constexpr uint8_t ADC_TRIG_TIMER1_COMPB_MATCH = (1 << ADTS2)|(0 << ADTS1)|(1 << ADTS0);  // 101  
constexpr uint8_t ADC_TRIG_TIMER1_OVF         = (1 << ADTS2)|(1 << ADTS1)|(0 << ADTS0);  // 110  
constexpr uint8_t ADC_TRIG_TIMER1_CAPTURE     = (1 << ADTS2)|(1 << ADTS1)|(1 << ADTS0);  // 111
*/



// 16-bit ADC Data Register (read-only): reading ADC reads ADCL then ADCH and returns a uint16_t combining both bytes  
// REGISTERS
#define ADC_DATA_REG      ADC  
// 8-bit ADC Data Register Low  (read-only): always holds ADCL  
#define ADC_DATA_LOW_REG  ADCL  
// 8-bit ADC Data Register High (read-only): always holds ADCH  
#define ADC_DATA_HIGH_REG ADCH  

// BITS (mapping of the 10-bit result depending on ADLAR):  

// The ATmega328P shifts a 10-bit number into ADCH and ADCL when it reads the pin voltage,
// whose MSBs are stored in ADCH and LSBs in ADCL. ADLAR just determines if ADCH or ADCL
// will be the register to hold 8 of the 10 bits.

// Thus when ADLAR = 0 (right-adjusted):  
//   ADC_DATA_LOW_REG  [7:0] → result bits [7:0]  
//   ADC_DATA_HIGH_REG [1:0] → result bits [9:8]; bits [7:2] are zero  
// ex: 000000XX xxxxxxxx is result
// Where ADCH is always on left here, and ADCL always on right
// ADCH[7:2] are 0, and ADCH[1:0] & ADCL[7:0] both give result bits

// When ADLAR = 1 (left-adjusted):  
//   ADC_DATA_HIGH_REG [7:0] → result bits [9:2]  
//   ADC_DATA_LOW_REG  [7:6] → result bits [1:0]; bits [5:0] are zero  
// ex: XXXXXXXX xx000000 is result
// Where ADCH is always on left here, and ADCL always on right
// ADCL[5:0] are 0, and ADCH[7:0] & ADCL[7:6] both give result bits



// 8-bit reg, SPI control register, configures SPI peripheral
//REGISTER
#define SPI_CTRL_REG SPCR
//BITS
// bit 7 SPI Interrupt Enable, causes SPI interrupt to execute if SPIF is set and global interrupt SREG is set
#define SPI_INT_ENABLE SPIE
// bit 6 SPI enable (either master or slave), when 1, SPI hardware turned on
#define SPI_ENABLE SPE
// bit 5 Data Order, when 1, LSB first, when 0, MSB first
#define SPI_DATA_ORDER DORD
// bit 4 Master, when driven 1, AVR operates as SPI master driving SCK, when 0, slave
#define MASTER MSTR
// bit 3 sets clock polarity idle level, 0 means first edge in pulse is rising edge. 
#define CLK_POLARITY CPOL
// bit 2 clock phase, determine if data sampled on leading or trailing edge of sck. 0 is sample leading, setup trailing, 1 is setup leading, sample trailing
#define CLK_PHASE CPHA
// bit 1:0 SPI clock rate select, control SCK rate out, if master. SPI2X in SPSR subtracts 1 from this dividing factor when it is 1. max div clk/128, min div clk/2

#define CLK_RATE1 SPR1
#define CLK_RATE0 SPR0

// 8-bit reg, SPI data, reading from it gives last-received byte, writing to it starts new SPI transfer of that byte
//REGISTER
#define SPI_DATA SPDR

// 8-bit reg, SPI status, contains status flags:
//REGISTER
#define SPI_STATUS SPSR
//BITS
// 7 SPIF (set 1 when transfer completes) (clear by reading SPSR and then SPDR) read-only bit
#define SPI_TRANSFER_FLAG SPIF
// 6 WCOL (set if avr wrote SPDR while transfer in progress) read-only bit
#define SPI_ERROR_FLAG WCOL
// 5:1 reserved
// 0 SPI2X (Double SPI speed in master mode, halves the SCK prescaler)
#define SPI_2X SPI2X
// others reserved



// REGISTERS:
// 8-bit reg, holds the prescaler select bits, dividing source clock by factors of 1 to 256 resulting in system clock value
// changes how fast CPU and all peripherals run. if source clock is more than max clock then ckdiv8 fuse can be programmed to set prescale to 0011 on startup
#define CLK_PRESCALE_REG CLKPR

// BITS:
// PORT |= (1<<PIN); to write PIN to 1
// PORT |= ~(1<<PIN); to write PIN to 0

// bits 6:4 are reserved

// bit 7 of CLKPR, write 1 into here to unlock prescaling, then within 4 clock cycles must write the CLKPS3:0 bits 
#define CLK_PRESCALE_CHANGE_ENABLE  CLKPCE

// bits 3:0 of CLKPR are written to CLKPR to set division factor between system clock and selected clock source pin(s)
// bit 3 CLKPS3: Prescaler Select bit 3
// bit 2 CLKPS2: Prescaler Select bit 2
// bit 1 CLKPS1: Prescaler Select bit 1
// bit 0 CLKPS0: Prescaler Select bit 0
//   together select division factor of system clock to an external pin:  
//   0000 = ÷2, 0001 = ÷2, 0010 = ÷4, 0011 = ÷8, 0100 = ÷16, 0101 = ÷32, 0110 = ÷64, 0111 = ÷128, 1000 = ÷256, others are reserved
#define CLK_PRESCALER3       CLKPS3
#define CLK_PRESCALER2       CLKPS2
#define CLK_PRESCALER1       CLKPS1
#define CLK_PRESCALER0       CLKPS0


// === Pin Change Interrupt Control Register ===
// 8-bit register; enables pin-change interrupt “ports” (groups of PCINT pins).
#define PIN_CHANGE_CTRL_REG  PCICR

// BITS:
// PCIE0 (bit 0) – Pin Change Interrupt Enable 0
//    1 = enable PCINT[7:0] (Port B) interrupts
//    0 = disable
#define PIN_CHANGE_INT_EN0 PCIE0

// PCIE1 (bit 1) – Pin Change Interrupt Enable 1
//    1 = enable PCINT[14:8] (Port C) interrupts
//    0 = disable
#define PIN_CHANGE_INT_EN1 PCIE1

// PCIE2 (bit 2) – Pin Change Interrupt Enable 2
//    1 = enable PCINT[23:16] (Port D) interrupts
//    0 = disable
#define PIN_CHANGE_INT_EN2 PCIE2

// Bits 7..3 are reserved and must be written as 0.

// Usage example:
//   // Enable pin-change interrupts on PC1, PC2, PC3 (which map to PCINT9,10,11):
//   PIN_CHANGE_CTRL_REG |= (1<<PCIE1);  

// === Pin Change Mask Register 1 ===
// 8-bit register; each bit enables pin-change interrupt for the corresponding PCINT[14:8] line.
#define PIN_CHANGE_MASK1  PCMSK1

// BITS:
// PCINT8  (bit 0) – Pin Change Interrupt 8  (maps to PC0)
// PCINT9  (bit 1) – Pin Change Interrupt 9  (maps to PC1)
#define PIN_CHANGE_INT9  PCINT9 

// PCINT10 (bit 2) – Pin Change Interrupt 10 (maps to PC2)
#define PIN_CHANGE_INT10 PCINT10 

// PCINT11 (bit 3) – Pin Change Interrupt 11 (maps to PC3)

// (Bits PCINT12–PCINT14 similarly map to PC4–PC6; bit 7 is reserved.)

// Usage example:
//   // Enable interrupts on PC1, PC2, PC3:
//   PIN_CHANGE_MASK1 |= (1<<PCINT9) | (1<<PCINT10);
