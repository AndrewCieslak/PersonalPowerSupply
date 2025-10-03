// encoder_int.c
#include "encoder.h"
#include "pins.h"         
// defines CLK_ENCODER, DT_ENCODER, and ENC_PORT/PIN/DDR

// SW_ENCODER not used here


void Encoder_init(void) {
    // 1) Configure pins as inputs + pull-ups
    ENC_DDR    &= ~((1<<CLK_ENCODER)|(1<<DT_ENCODER));
    ENC_PULLUP |=  ((1<<CLK_ENCODER)|(1<<DT_ENCODER));

    PCICR  |= (1<<PCIE2);
    PCMSK2 |= (1<<PCINT18) | (1<<PCINT19);
    PCIFR  |= (1<<PCIF2);

}



