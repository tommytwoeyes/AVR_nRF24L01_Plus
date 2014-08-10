/**
 * nRF24L01+ library for Atmel AVR ATmega48/88/168/328 microcontrollers
 * 
 * File: nrf24l01p.c
 * Author: Tom Malone
 *
 * Inspired by:
 * 	Neil MacMillan:  http://nrqm.ca/nrf24l01/
 * 	Brennan:         http://blog.diyembedded.com/
 * 	Stefan Engelke:  http://tinkerer.eu/AVRLib/nRF24L01/
 *
*/

#include "nrf24l01p.h"

// Non-Public Constants & Macros

#define CHANNEL			112
#define ADDRESS_LENGTH	5

// Pin definitions for chip select and chip enable on the radio module
#define CE_DDR			DDRB
#define CE_PORT			PORTB
#define CE_PIN			PB1
#define CSN_DDR			DDRB
#define CSN_PORT		PORTB
#define CSN_PIN			PB2			// SS
#define IRQ_DDR			DDRB
#define IRQ_PORT		PORTB
#define IRQ_PIN			PB0			// PCINT0

