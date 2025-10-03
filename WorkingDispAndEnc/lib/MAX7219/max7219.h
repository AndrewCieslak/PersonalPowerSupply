#pragma once

#include <stdint.h>
#include <stdbool.h>
// === MAX7219 Registers ===


// REGISTER:
// 8-bit Decode-Mode Register (address 0x09):
#define REG_DECODE 0x09
//   controls whether each digit’s data byte is treated as either “raw segment bits” 
// (7-segment display, so 7-segments for each digit with a corresponding 7-bit value per digit OR a 4-bit value per digit if only doing 0-9)
// (the 8th bit aka bit 7, no matter if raw or BCD, is the decimal place)
//  or as a 4-bit binary coded decimal (BCD) value (0–9) with the MAX7219 handling segment encoding.
//  In BCD, each decimal is a 4-bit binary value where 0000=0, 0001=1,..., 1001=9

// BITS:
// 0 = no decode (raw segments), 1 = BCD decode (0–9, “dash” for A=10)
//   – MSB bits [7:4] correspond to digits 7–4
//   – LSB bits [3:0] correspond to digits 3–0  
// [7] DEC7: Decode select for digit 7  
// [6] DEC6: Decode select for digit 6  
// [5] DEC5: Decode select for digit 5  
// [4] DEC4: Decode select for digit 4  
// [3] DEC3: Decode select for digit 3  
// [2] DEC2: Decode select for digit 2  
// [1] DEC1: Decode select for digit 1  
// [0] DEC0: Decode select for digit 0  
//   + Bit D7 (in each nibble) is SEG DP (decimal point) independent of decode.
//     D7 = 1 lights the DP; D7 = 0 leaves DP off.
// Bit D7 is 

// Example values:
//   0x00 → all digits raw segment mode  
//   0xFF → all digits BCD-decode mode  
//   0x0F → digits 3–0 decode, digits 7–4 raw  
//   0x55 → alternate digits decode (0101 0101₂)



// Intensity Register (address 0x0A):
#define REG_INTENSITY 0x0A
//   Controls brightness via internal PWM. Lower nibble [3:0] selects duty cycle:
//     0x0 = 1/32  …  0xF = 31/32 of RSET-defined peak current.
//   Bits D7–D4 are reserved and must be 0.
/* 
BITS:
#define INTENSITY_0    = 0x0;  // 1/32
#define INTENSITY_1    = 0x1;  // 3/32
#define INTENSITY_2    = 0x2;  // 5/32
// … through …
#define INTENSITY_F    = 0xF;  // 31/32
*/

// Brightness scale
#define INTENSITY_MIN 0x00
#define INTENSITY_MAX 0x0F

#define REG_SCAN_LIMIT 0x0B
// controls how many of the eight digit outputs the MAX7219 will multiplex, 
// in this case being 8 digits, internally going one digit at a time by "scanning"
// the address of this register is 0x0B, eleven, which is how it is defined.
// only bits 2,1, and 0 in this register are defined, with the others reserved and must be written as zero. 
// [7:3] ignored, SL2, SL1, SL0 are ScanLimit bit names.

#define REG_SHUTDOWN 0x0C
// Shutdown Register (address 0x0C):
//   D0 = 0 → shutdown mode (oscillator off, outputs blanked).
//   D0 = 1 → normal operation.
//   Bits D7–D1 are reserved and must be 0.
/* 
BITS:
#define SHUTDOWN_MODE  = 0 << 0;  // blank display
#define NORMAL_OP_MODE = 1 << 0;  // enable display
*/

// Display-Test Register (address 0x0F):
#define REG_DISPLAY_TEST 0x0F
//   D0 = 0 → normal operation.
//   D0 = 1 → display-test mode (all LEDs on, overrides other registers).
//   Bits D7–D1 are reserved and must be 0.
/* 
BITS:
#define TEST_MODE_OFF  = 0 << 0;  // normal
#define TEST_MODE_ON   = 1 << 0;  // all segments on
*/

// === Font Table ===
static const struct {
    char ascii;
    uint8_t segs;
} MAX7219_Font[] = {
    {'A',0b1110111},{'B',0b1111111},{'C',0b1001110},{'D',0b1111110},{'E',0b1001111},{'F',0b1000111},
    {'G',0b1011110},{'H',0b0110111},{'I',0b0110000},{'J',0b0111100},{'L',0b0001110},{'N',0b1110110},
    {'O',0b1111110},{'P',0b1100111},{'R',0b0000101},{'S',0b1011011},{'T',0b0001111},{'U',0b0111110},
    {'Y',0b0100111},{'[',0b1001110},{']',0b1111000},{'_',0b0001000},{'a',0b1110111},{'b',0b0011111},
    {'c',0b0001101},{'d',0b0111101},{'e',0b1001111},{'f',0b1000111},{'g',0b1011110},{'h',0b0010111},
    {'i',0b0010000},{'j',0b0111100},{'l',0b0001110},{'n',0b0010101},{'o',0b1111110},{'p',0b1100111},
    {'r',0b0000101},{'s',0b1011011},{'t',0b0001111},{'u',0b0011100},{'y',0b0100111},{'-',0b0000001},
    {' ',0b0000000},{'0',0b1111110},{'1',0b0110000},{'2',0b1101101},{'3',0b1111001},{'4',0b0110011},
    {'5',0b1011011},{'6',0b1011111},{'7',0b1110000},{'8',0b1111111},{'9',0b1111011},{'\0',0b0000000},
    
    // the MSB of each byte is the decimal point. 
    // These all are the 7 bit segment combinations to make the respective characters show on the display.

};
/*
* LED Segments:         a
*                     ----
*                   f|    |b
*                    |  g |
*                     ----
*                   e|    |c
*                    |    |
*                     ----  o dp
*                       d
*   Register bits:
*      bit:  7  6  5  4  3  2  1  0
*           dp  a  b  c  d  e  f  g

1 register for each digit
*/


// === MAX7219 Driver Class ===
typedef struct {
} MAX7219;

void MAX7219_Begin(void);
void MAX7219_DisplayChar(int digit, char character, bool dp);
void MAX7219_DisplayText(char* text, int justify);
void MAX7219_Write(uint8_t opcode, uint8_t data);
void MAX7219_ShutdownStart(void);
void MAX7219_ShutdownStop(void);
void MAX7219_DisplayTestStart(void);
void MAX7219_DisplayTestStop(void);
void MAX7219_SetBrightness(char brightness);
void MAX7219_Clear(void);
uint8_t MAX7219_LookupCode(char character, uint8_t dp);

