#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>

#include "pins.h"
#include "uart.h"



int main(void){

  uart_init();
  
  //LEDs
  DDRD |= (1<<PD2) | (1<<PD3) | (1<<PD5);
  PORTD &= ~((1<<PD2) | (1<<PD3) | (1<<PD5));
  //blue LED ON
  PORTD |= (1<<PD2);

  DDRB  |= (1<<PB3) | (1<<PB5);
  PORTB |= (1<<PB2);
  SPCR = (1<<SPE) | (1<<MSTR);
  SPSR = 0;   
  AD420_CLR_DDR |= (1<<CLR_AD420);
  AD420_CLR_PORT &= ~(1<<CLR_AD420);
  // SPI init

  AD420_CLR_PORT |=  (1<<CLR_AD420);   // CLEAR = 1
  _delay_us(1); // delay >50ns
  AD420_CLR_PORT &= ~(1<<CLR_AD420);   // CLEAR = 0

  uint16_t voltmv = 1000;

  //white LED
  PORTD |= (1<<PD3);  
  //   Converts a desired voltage (0…5000 mV) to a 16-bit DAC code (0…0xFFFF).
  if (voltmv > 5000) voltmv = 5000;
  uint32_t temp = (uint32_t)voltmv * 65535;
  uint16_t code = (uint16_t)(temp / 5000);
  // turns into binary
  uint16_t high_byte =  (uint8_t)(code >> 8);
  // top 8-bits
  uint16_t low_byte = (uint8_t)(code & 0xFF);
  // bottom 8-bits saved
  AD420_CS_PORT &= ~(1<<CS_AD420);
  // start frame
  SPI_DATA = high_byte;
  // begins transfer of byte
  while(!(SPI_STATUS & (1<<SPI_TRANSFER_FLAG)));
  // (spif location, not its status, is in the variable SPIF. the register is volatile and changes, while the bit location is constant)
  // if and only if xxxxxxxx & x0000000 results in 1 does it pass, since last 7 bits always 0, and first just depends on if status is actually 1 or 0
  // the ! is because while loops repeat when true (1), so the exit condition is 0 and must use !.
  (void)SPI_DATA;           
  // reading SPDR clears SPIF
  SPI_DATA = low_byte;
  // begins transfer
  while(!(SPI_STATUS & (1<<SPI_TRANSFER_FLAG)));
  // sends
  (void)SPI_DATA;       
  // acknowledged
  AD420_CS_PORT |= (1<<CS_AD420);
  // LATCH = 1, dumping 16 bits into ad420 immediately after byte sent
  _delay_us(1);
  // AVR-GCC intrinsic function, CPU cycles
  AD420_CS_PORT |= (1<<CS_AD420);
  // LATCH = 1, resetting for next input
  _delay_us(1);
  
  uart_transmit_string("transmitted");
  while(1);
}

