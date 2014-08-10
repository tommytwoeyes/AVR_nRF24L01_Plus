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

/**
 * Configure one of the radio's six Rx pipes.
 * This configures a pipe's address and enables or disables the pipe.  Pipes 0 and 1 are enabled by default, and pipes 2-5 are
 *              disabled by default.  The configuration for pipe 0 will be changed while the radio is transmitting, to facilitate auto-
 *              ack.  It will be changed back when the transmission is completed.
 * \param pipe The pipe to configure.
 * \param address The 1- or 5-byte address to give the pipe.  For pipes 0 and 1 all five bytes can be different, but
 *              pipes 2-5 share the four most significant bytes of pipe 1's address.  The LSB of each pipe's address must be unique.
 *              For example:
 *                              Pipe 0: 0x0123456789
 *                              Pipe 1: 0x9876543210
 *                              Pipe 2: 0x98765432AB
 *                              Pipe 3: 0x98765432BC
 *                              Pipe 4: 0x98765432CD
 *                              Pipe 5: 0x98765432DE
 *              If pipe 0 or 1 is being configured, then address must be a 5-byte array.  If the other four pipes are being configured,
 *              then the first byte of address is used as the LSB of the pipe's address (i.e. you only pass a 1-byte address, with the
 *              four MSBytes of the pipe's address left implied).  For example, this will set the first four pipe addresses above:
 *                              uint8_t address[5] = {0x01, 0x23, 0x45, 0x67, 0x89};
 *                              Radio_Configure_Rx(RADIO_PIPE_0, address, ENABLE);
 *                              address = {0x98, 0x76, 0x54, 0x32, 0x10};
 *                              Radio_Configure_Rx(RADIO_PIPE_1, address, ENABLE);
 *                              address[0] = 0xAB;
 *                              Radio_Configure_Rx(RADIO_PIPE_2, address, ENABLE);
 *                              address[0] = 0xBC;
 *                              Radio_Configure_Rx(RADIO_PIPE_3, address, ENABLE);
 *                              ...
 * \param enable Enable or disable the pipe.
 */
 void Radio_Configure_Rx(RADIO_PIPE pipe, uint8_t* address, ON_OFF enable);
 
 /**
 * Configure the radio transceiver.
 * \param dr The data rate at which the radio will transmit and receive data (1 Mbps or 2 Mbps).
 * \param power The transmitter's power output.
 */
 void Radio_Configure(RADIO_DATA_RATE dr, RADIO_TX_POWER power);

#endif /* NRF24L01P_H_ */
