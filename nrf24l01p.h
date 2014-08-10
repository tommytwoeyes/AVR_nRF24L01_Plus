/**
 * nRF24L01+ Register Definitions
 * File: nrf24l01p.h
 * Author: Tom Malone
 *
 * Inspired by:
 * 	Neil MacMillan:  http://nrqm.ca/nrf24l01/
 * 	Brennan:         http://blog.diyembedded.com/
 * 	Stefan Engelke:  http://tinkerer.eu/AVRLib/nRF24L01/
 *
*/

#ifndef	NRF24L01P_H_
#define	NRF24L01P_H_

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "nrf24l01p_mnemonics.h"

#define RADIO_ADDRESS_LENGTH	5

// Multiple receivers on single channel (each pipe has distinct address)
typedef enum _radio_pipe {
	RADIO_PIPE_0 		= 0,
	RADIO_PIPE_1 		= 1,
	RADIO_PIPE_2 		= 2,
	RADIO_PIPE_3 		= 3,
	RADIO_PIPE_4 		= 4,
	RADIO_PIPE_5 		= 5,
	RADIO_PIPE_EMPTY	= 7,		// FIFO is empty when pipe number bits in status register are 0b111.
} RADIO_PIPE;

// Radio transmission power options
typedef enum _radio_tx_power {
	RADIO_LOWEST_POWER	= 0,	// -18 dBm (about 16 uW)
	RADIO_LOW_POWER		= 1,	// -12 dBm (about 63 uW)
	RADIO_HIGH_POWER	= 2,	//  -6 dBm (about 251 uW)
	RADIO_HIGHEST_POWER	= 3,	//   0 dBm (1 mW)
} RADIO_TX_POWER;

// Radio data rate options
typedef enum _radio_dr {
	RADIO_1MBPS		= 0,		// Megabytes, not Megabits
	RADIO_2MBPS		= 1,
} RADIO_DATA_RATE;

// Radio RX status
typedef enum _radio_receive {
	RADIO_RX_INVALID_ARGS,		// One of the arguments to Radio_Receive() was invalid
	RADIO_RX_TRANSMITTING,		// Radio was transmitting
	RADIO_RX_FIFO_EMPTY,		// There are no packets in the RX FIFO  (Radio_Receive() does not receive data)
	RADIO_RX_MORE_PACKETS,		// After copying out the head of the RX FIFO, there is another packet remaining in FIFO
	RADIO_RX_SUCCESS,			// There was a packet to receive, it was successfully received, and the RX FIFO is now empty.
} RADIO_RX_STATUS;

// Radio TX status
typedef enum _radio_transmit {
	RADIO_TX_MAX_RT,			// Radio TX maximum number of retries reached
	RADIO_TX_SUCCESS,			
} RADIO_TX_STATUS;

typedef enum _radio_tx_wait {
	RADIO_WAIT_FOR_TX,
	RADIO_RETURN_ON_TX,
} RADIO_TX_WAIT;

typedef enum _radio_ack {		// Whether or not to use Auto Acknowledgment feature
	RADIO_ACK,						// Use Auto-Ack
	RADIO_NO_ACK,					// Don't use Auto-Ack
} RADIO_USE_ACK;

typedef enum _ed {					// Enable/Disable
	DISABLE	= 0,
	ENABLE	= 1,
} ON_OFF;

void Radio_Init();

#endif /* NRF24L01P_H_ */
