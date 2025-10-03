#ifndef F_CPU
#define F_CPU 14745600UL
#endif

#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#include "pins.h"
#include "max7219.h"
#include "ad420.h"
#include "encoder.h"
#include "uart.h"

volatile uint16_t vout_adc = 0;
volatile uint16_t dac_adc = 0;
volatile bool new_vout = false;
volatile bool new_dac = false;
volatile int16_t pos = 0;
uint16_t VREF_MV = 2500;
uint8_t vDivT = 68; //in kohm
uint8_t vDivB = 9; //in kohm
volatile int16_t mvDAC = 0;
uint16_t REFRESH_RATE_HZ = 60;

#define  ENC_VOLT_MIN_MV   225  // lowest setpoint
#define  ENC_VOLT_MAX_MV   2250  // highest setpoint
#define ENC_STEP_MV 45
#define  ENC_MAX_POS    ((ENC_VOLT_MAX_MV - ENC_VOLT_MIN_MV) / ENC_STEP_MV)   // max clicks between values
#define  ENC_MIN_POS    0     // minimum encoder count
// Variable definitions

int main(void){

  uart_init();
  // Initializes UART communication

  DDRB  |= (1<<PB2) | (1<<PB3) | (1<<PB5);
  PORTB |= (1<<PB2);
  SPCR = (1<<SPE) | (1<<MSTR);
  SPSR = 0;   
  AD420_CLR_DDR |= (1<<CLR_AD420);
  AD420_CLR_PORT &= ~(1<<CLR_AD420);
  // SPI init

  ADC_MUX_REG = (ADC_MUX_REG & ~((1<<ADC_REF_SEL1) | (1<<ADC_REF_SEL0)));
  ADC_CTRL_STATUS_REG = (1 << ADC_ENABLE) | (1<<ADC_PRESCALER2) | (1 << ADC_PRESCALER1) | (1 << ADC_PRESCALER0); 
  ADC_CTRL_STATUS_REG |= (1<<ADC_INT_ENABLE);
  ADC_DDR &= ~((1 << VOUT_ADC) | (1 << DACV_ADC)); 
  ADC_MUX_REG = (DACV_ADC & 0xF0) | (VOUT_ADC & 0x0F);
  // ADC init

  // ^took out setting reg to 0, changed prescal from 8 to 128 and changed adcref to be external

  Encoder_init();
  // Initialize encoder and read initial state (need to fix)

  MAX7219_Begin();
  // Display init

  clearDAC();
  // Clear DAC

  sei();
  // Interrupts init

  ADC_CTRL_STATUS_REG |= (1 << ADC_START_CONV);
  // Begins ADC conversion

  while(1){
    if(new_vout){
      uint16_t mvout = (uint16_t)(((uint64_t)vout_adc * (vDivT + vDivB)) / (vDivB));
      uint8_t outtens       = mvout / 10000;
      uint8_t outones       = (mvout % 10000) / 1000;
      uint8_t outtenths     = (mvout % 1000) / 100;
      uint8_t outhundredths = (mvout % 100) / 10;
      uint8_t sreg = SREG;
      cli();
      MAX7219_DisplayChar(0, '0' + outhundredths, false);
      MAX7219_DisplayChar(1, '0' + outtenths, false);
      MAX7219_DisplayChar(2, '0' + outones, true);
      MAX7219_DisplayChar(3, '0' + outtens, false);
      SREG = sreg;
      //_delay_ms((uint16_t)(1000/REFRESH_RATE_HZ));
      // runs too fast, readings not precise nor stable
      new_vout = false;
      uart_transmit_string("new vout");
    }
    if(new_dac){
      uint16_t mvdac = ENC_VOLT_MIN_MV+pos*ENC_STEP_MV;
      uint8_t setones       = (mvdac % 10000) / 1000;
      uint8_t settenths     = (mvdac % 1000) / 100;
      uint8_t sethundredths = (mvdac % 100) / 10;
      uint8_t setthousandths = (mvdac % 10);
      uint8_t sreg = SREG;
      cli();
      //ad420_newVoltage(mvdac);
      uart_print_uint16(mvdac);
      _delay_us(1);
      MAX7219_DisplayChar(4, '0' + setthousandths, false); 
      MAX7219_DisplayChar(5, '0' + sethundredths, false);
      MAX7219_DisplayChar(6, '0' + settenths, false);
      MAX7219_DisplayChar(7, '0' + setones, true);
      SREG = sreg;
      //_delay_ms((uint16_t)(1000/REFRESH_RATE_HZ));
      // runs too fast, readings not precise nor stable
      new_dac = false;
      uart_transmit_string("new dac");
    }
  }
}

ISR(ADC_vect){
  uint16_t sample = ((uint32_t)VREF_MV * ADCW) / 1023;
  vout_adc = sample;
  new_vout = true;
  ADC_CTRL_STATUS_REG |= (1 << ADC_START_CONV);
}

// PCINT2 is D port. 1 is C and 0 is B.
ISR(PCINT2_vect) {
  static uint8_t last_state = 0;
  uint8_t clk = (ENC_PIN & (1<<CLK_ENCODER)) ? 1 : 0;
  uint8_t dt  = (ENC_PIN & (1<<DT_ENCODER))  ? 1 : 0;
  uint8_t state = (clk << 1) | dt;   // 00,01,10,11
  if (state == 0b00) {  // only count at detent
    if (last_state == 0b10) pos++;    // CW
    else if (last_state == 0b01) pos--; // CCW
    if (pos < ENC_MIN_POS) pos = ENC_MIN_POS;
    if (pos > ENC_MAX_POS) pos = ENC_MAX_POS;
    new_dac = true;
  }
  last_state = state;
}


