#pragma once
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdlib.h>

void uart_init();
void uart_transmit(uint8_t data);
uint8_t uart_receive();
void uart_transmit_string(const char* str);
void uart_print_uint16(uint16_t val);
