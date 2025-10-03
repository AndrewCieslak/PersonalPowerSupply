//This code works in the parent folder with its main.cpp
#include <util/delay.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <avr/io.h>
#include "pins.h"
#include "max7219.h"

// display options: 
#define MSBFIRST 1 // this should never change, MSBFIRST is defined as 1
// need to figure out how to use HAL for atmega328p to make it easier, if possible to do so with usbasp (probably just write the project then export it?)
// i wanna change this to hardware SPI, like the ad420.cpp code.

// === Class Implementation ===

void MAX7219_Begin(void) {
    DISP_DDR |= (1 << MOSI) | (1 << SCK);
    DISP_DDR_CS |= (1 << CS_DISP);
    DISP_PORT_CS |= (1 << CS_DISP);
    // ensures CS begins high before configuration
    MAX7219_ShutdownStart();
    MAX7219_Write(REG_DECODE, 0x00);
    // then overwrote back again as another test
    // I overwrote this 9/4/25 to 0xFF from 0x00 in order to just display digits decode mode
    // writes 0 to REG_DECODE register making all digits raw decode mode (7 bits per digit)
    MAX7219_Write(REG_SCAN_LIMIT, 7);
    // writes 7 to SCAN_LIMIT register, which controls how many of the eight digit outputs the MAX7219 will multiplex, 
    // in this case being 8 digits, internally going one digit at a time by "scanning"
    // the address of this register is 0x0B, eleven, which is how it is defined.
    // only bits 2,1, and 0 in this register are defined, with the others reserved and must be written as zero. 
    // [7:3] ignored, SL2, SL1, SL0 are ScanLimit bit names.
    MAX7219_SetBrightness(INTENSITY_MAX/2);
    // sets brightness to max
    MAX7219_Clear();
    // writes all digits to 0
    MAX7219_ShutdownStop();
    // display on
    MAX7219_DisplayTestStop();
    // normal operation
}

void MAX7219_Write(uint8_t opcode, uint8_t data) {    // opcode is the 8-bit register address in 
    // data is the 8-bit data being written
    AD420_CS_PORT |= (1<<CS_AD420); 
    //disable ad420
    DISP_PORT_CS |= (1<< CS_DISP);
    _delay_us(1);

    DISP_PORT_CS &= ~(1 << CS_DISP);
    // writes chip select low (active)

    // THESE 4 LINES ARE NEW 7/27 CODE!!!
    SPDR = opcode;                         
    // start first byte
    while (!(SPSR & (1 << SPIF))){}
    (void)SPDR;    
    
    
    // wait for completion
    SPDR = data;                           
    // second byte
    while (!(SPSR & (1 << SPIF))){}
    (void)SPDR;
            
    DISP_PORT_CS |= (1 << CS_DISP);
    _delay_us(1);
    // writes chip select high (inactive)
    // can and should do delays after each thing here if necessary

}

void MAX7219_ShutdownStop(void) {
    MAX7219_Write(REG_SHUTDOWN, 1);
    // powers on display driver, takes out of shutdown mode
}

void MAX7219_ShutdownStart(void) {
    MAX7219_Write(REG_SHUTDOWN, 0);
    // turns off display driver
}

void MAX7219_DisplayTestStart() {
    MAX7219_Write(REG_DISPLAY_TEST, 1);
    // turns all leds on
}

void MAX7219_DisplayTestStop() {
    MAX7219_Write(REG_DISPLAY_TEST, 0);
    // normal operation, only HIGH leds are on
}

void MAX7219_SetBrightness(char brightness) {
    brightness &= 0x0F;
    MAX7219_Write(REG_INTENSITY, brightness);
    // brightness out of 32
}

void MAX7219_Clear() {
    for (int i = 0; i < 8; i++) {
        MAX7219_Write(i + 1, 0x00);
    }
    // writes 0x00 to the first 8 registers, which all coincide with that digit's inherent register
}

uint8_t MAX7219_LookupCode(char character, uint8_t dp) {
    for (int i = 0; MAX7219_Font[i].ascii; i++) {
        if (character == MAX7219_Font[i].ascii) {
            uint8_t code = MAX7219_Font[i].segs;
            if (dp) code |= (1 << 7);
            return code;
        }
        // loops through the font array to find a matching character, then drives dp high if input was 1, and returns the result byte
    }
    return 0;
    // returns the 8-bit code for the character in first parameter box, plus decimal point 1=show, 0=not show
}

void MAX7219_DisplayChar(int digit, char value, bool dp) {
    MAX7219_Write(digit + 1, MAX7219_LookupCode(value, dp));
    // converts 0,1,2,3,4,5,6,7 and the character, and a 1 true or 0 false into its resulting digit place, decimal point, and character
}

void MAX7219_DisplayText(char* text, int justify) {
    // justify is 0 or 1, if 0, then characters line up on left, if 1, then they line up on right
    int decimal[8] = {0};
    char trimStr[9] = {0};
    int x, y = 0, s = strlen(text);
    if (s > 16) s = 16;
    //     decimal[8] tracks which of the up-to-8 displayed characters should get a DP.
    //     trimStr[9] holds up to 8 non-dot characters plus a terminator.
    //     s = strlen(text) is length of the input string.
    //     Clamps s to 16 to avoid over-running the buffers.

    for (x = 0; x < s; x++) {
        if (text[x] == '.') {
            if (y > 0) decimal[y - 1] = 1;
            // if not first index, then decimal driven high at the previous character.
        } else {
            trimStr[y] = text[x];
            // saves the character in this new array without decimal, so the . doesn't take up a char index
            decimal[y] = 0;
            // saves the non-decimal index as 0, no decimal point
            y++;
            // moves arrays that scan the string forward
        }
    }

    // scans input, marks index of decimal point to attach to previous character if there is a previous character,
    // then stores the character in trimStr[y], then saves non-decimal and/or decimal index in decimal[y]

    if (y > 8) y = 8;
    // clamp to 8 characters (my display)
    for (x = 0; x < y; x++) {
        if (justify == 0) {
            MAX7219_DisplayChar(x, trimStr[x], decimal[x]); // LEFT
            // aligns to left
        } else {
            MAX7219_DisplayChar(8 - y + x, trimStr[x], decimal[x]); // RIGHT
            // aligns to right
        }
    }
    // loops over all of the y characters and aligns them, displaying the characters depending on their decimal point value and character value
}


