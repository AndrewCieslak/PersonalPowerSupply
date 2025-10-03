#include "uart.h"

#define BAUDRATE 9600
#define BAUD_PRESCALLER (((F_CPU / (BAUDRATE * 16UL))) - 1)

void uart_init() {
  UBRR0H = (uint8_t)(BAUD_PRESCALLER >> 8);
  UBRR0L = (uint8_t)(BAUD_PRESCALLER);
  UCSR0B = (1 << RXEN0) | (1 << TXEN0);  // Enable receiver and transmitter
  UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);  // 8-bit data format
}

void uart_transmit(uint8_t data) {
  while (!(UCSR0A & (1 << UDRE0)));  // Wait for empty transmit buffer
  UDR0 = data;  // Put data into the buffer, sends the data
}

uint8_t uart_receive() {
  while (!(UCSR0A & (1 << RXC0)));  // Wait for data to be received
  return UDR0;  // Get and return received data from buffer
}

void uart_transmit_string(const char* str) {
  for (size_t i = 0; str[i] != '\0'; ++i) {
    uart_transmit(str[i]);
  }
}

void uart_print_uint16(uint16_t val) {
    char buf[6];
    itoa(val, buf, 10);
    uart_transmit_string(buf);
}

