/**
 * nRF24L01+ Register Definitions
 * File: nrf24l01p.h
 * Author: Tom Malone
 *
 * Inspired (heavily) by:
 * 	Neil MacMillan:  http://nrqm.ca/nrf24l01/
 * 	Stefan Engelke:  http://tinkerer.eu/AVRLib/nRF24L01/
 * 	Brennan:         http://blog.diyembedded.com/
 *
*/

#ifndef	NRF24L01P_H_
#define	NRF24L01P_H_

// This should be &ed with the register values defined below when they
// are masked into the R_REGISTER and W_REGISTER instructions, to make
// sure that they do not overwrite the 3 MSBits in those instructions.
#define REGISTER_MASK 0x1F

typedef uint8_t radio_register_t;

/* Memory Map */
#define	CONFIG		0x00		# Configure interrupts, PWR, RX/TX status, and CRC
#define	EN_AA		0x01		# Enable "Auto Acknowledgment" feature
#define	EN_RXADDR		0x02		# Enabled or disable the RX pipes
#define	SETUP_AW		0x03		# Set address width (must be the same on all radios)
#define	SETUP_RETR	0x04		# Set retry delay and number of retries for pipes using Enhanced Shockburst
#define	RF_CH		0x05		# Set the channel to use, from the 2.4 GHz ISM band.
#define	RF_SETUP		0x06		# Set radio data rate, output power, and LNA gain
#define	STATUS		0x07		# Interrupt status, TX FIFO register full, and number of RX pipe that received a packet (in RX mode, of course)
#define	OBSERVE_TX	0x08		# Count lost and resent packets
#define	RPD			0x09		# Snapshot of current received power level in the channel
// Receive address bytes (this (PIPE 0) and following 5 lines)
// (see documentation for explanation of how they fit 6 5-byte addresses
// into 40 bits).  P0 is also used for auto-ack handling.
#define	RX_ADDR_P0	0x0A		# Receive Address DATA PIPE 0
#define	RX_ADDR_P1	0x0B		# Receive Address DATA PIPE 1
#define	RX_ADDR_P2	0x0C		# Receive Address DATA PIPE 2
#define	RX_ADDR_P3	0x0D		# Receive Address DATA PIPE 3
#define	RX_ADDR_P4	0x0E		# Receive Address DATA PIPE 4
#define	RX_ADDR_P5	0x0F		# Receive Address DATA PIPE 5
#define	TX_ADDR		0x10		# Transmit destination address
// Payload data width for each of the Rx pipes (0x01 bytes to 0x20
// bytes, or 0x00 if pipe is not used)
#define	RX_PW_P0		0x11		# Payload Data Width for PIPE 0 (# of bites per payload)
#define	RX_PW_P1		0x12		# Payload Data Width for PIPE 1
#define	RX_PW_P2		0x13		# Payload Data Width for PIPE 2
#define	RX_PW_P3		0x14		# Payload Data Width for PIPE 3
#define	RX_PW_P4		0x15		# Payload Data Width for PIPE 4
#define	RX_PW_P5		0x16		# Payload Data Width for PIPE 5
// Auto-retransmit status (cf. REUSE_TX_PL instruction), Tx FIFO
// full/empty, Rx FIFO full/empty
// (The Rx FIFO is a 3-packet queue shared by all six pipes)
#define	FIFO_STATUS	0x17		# FIFO Status Register
#define	DYNPD		0x1C		# Enable dynamic payload length
#define	FEATURE		0x1D		# Feature Register


/* Register Bit Mnemonics [ bit mask shift values, to be used with _BV() ] */
//// CONFIG Register
#define	MASK_RX_DR	6			# Turn off RX data ready interrupt: 0 - Rx data-ready interrupt is sent to IRQ pin.  1 - Interrupt is not sent to IRQ pin.
#define	MASK_TX_DS	5			# Turn off TX data sent interrupt: 0 - Tx data-sent interrupt is sent to IRQ pin.  1 - Interrupt is not sent to IRQ pin.
#define	MASK_MAX_RT	4			# Turn off "Maximum number of packet resend retries reached" interrupt: 0 - Max-retries-reached interrupt is sent to IRQ pin.  1 - Interrupt is not sent to IRQ pin.
#define	EN_CRC		3			# Enable Cyclic Redundancy Check: 0 - Disable automatic CRC.  1 - Enable automatic CRC.
#define	CRCO			2			# CRC encoding scheme: 0 - Use 1-byte CRC.  1 - Use 2-byte CRC.
#define	PWR_UP		1			# POWER STATUS: 0 - Power down the radio.  1 - Power up the radio
#define	PRIM_RX		0			# RX/TX STATUS: 0 - Radio is a transmitter.  1 - Radio is a receiver.

//// EN_AA Register
#define	ENAA_P5		5			# Enable (1)/Disable (0) Auto Acknowledgment feature for DATA PIPE 5
#define	ENAA_P4		4			# Enable Auto Acknowledgment feature for DATA PIPE 4
#define	ENAA_P3		3			# Enable Auto Acknowledgment feature for DATA PIPE 3
#define	ENAA_P2		2			# Enable Auto Acknowledgment feature for DATA PIPE 2
#define	ENAA_P1		1			# Enable Auto Acknowledgment feature for DATA PIPE 1
#define	ENAA_P0		0			# Enable Auto Acknowledgment feature for DATA PIPE 0

//// EN_RXADDR Register
#define	ERX_P5		5			# Enable (1)/Disable (0) DATA PIPE 5
#define	ERX_P4		4			# Enable DATA PIPE 4
#define	ERX_P3		3			# Enable DATA PIPE 3
#define	ERX_P2		2			# Enable DATA PIPE 2
#define	ERX_P1		1			# Enable DATA PIPE 1
#define	ERX_P0		0			# Enable DATA PIPE 0

//// SETUP_AW Register
#define	AW			0			# Address width (bits 1:0): 01 = 3 bytes, 10 = 4 bytes, 11 = 5 bytes

//// SETUP_RETR Register
// ARD = Auto Retransmit delay (bits 7:4)
//				0000 = Wait 250 + 86 us
//				0001 = Wait 500 + 86 us
//				0010 = Wait 750 + 86 us
//				0011 = Wait 1000 + 86 us
//				....
//				1111 = Wait 4000 + 86 us
#define	ARD			4
// Auto Retransmit count (bits 3:0)
//				0000 = Retransmit disabled
//				0001 = Up to 1 retransmit on fail of auto-ack
//				0010 = Up to 2 retransmits on fail of auto-ack
//				....
//				1111 = Up to 15 retransmits ...
#define	ARC			0			# Auto Retransmit Count

//// We skip the RF_CH register, because it's pretty straightforward.
//// Load the channel number into bits 6:0.

//// RF_SETUP Register
#define	CONT_WAVE		7			# Enables continuous carrier transmit when high
#define	RF_DR_LOW		5			# Set RF Data Rate to 250kbps. See RF_DR_HIGH for encoding.
#define	PLL_LOCK		4			# Force PLL lock signal.  See [http://en.wikipedia.org/wiki/Phase-locked_loop].  This shouldn't be set (only used in test).
// Select between the high-speed data rates. This bit is doesn't care
// if RF_DR_LOW is set.
// Encoding:
//				00 = 1Mbps
//				01 = 2Mbps
//				10 = 250kbps
//				11 = Reserved
#define	RF_DR_HIGH	3
// Radio TX power (bits 2:1). The unit dBm is decibels relative to 1 mW.
//				00 = -18 dBm
//				01 = -12 dBm
//				10 = - 6 dBm
//				11 =   0 dBm
#define	RF_PWR		1

//// STATUS Register
// RX data ready interrupt.
// 0 = RX data ready interrupt was not triggered.
// 1 = RX data ready interrupt was triggered.
// Write 1 to clear after interrupt.
#define	RX_DR		6
// TX data sent interrupt.
// 0 = TX data sent interrupt was not triggered.
// 1 = TX data sent interrupt was triggered.
// Write 1 to clear after interrupt.
#define	TX_DS		5
// Max number of retries sent interrupt.
// 0 = Max retries interrupt was not triggered.
// 1 = Max retries interurupt was triggered.
// Write 1 to clear after interrupt.
// NOTE: If the MAX_RT interrupt is triggered, this needs to be cleared
// before radio can be used again.
#define	MAX_RT		4
// Number of the data pipe that just received data (bits 3:1)
// 000 = pipe 0
// 001 = pipe 1
// 010 = pipe 2
// 011 = pipe 3
// 100 = pipe 4
// 101 = pipe 5
// 110 = not used
// 111 = All RX FIFOs are empty.
#define	RX_P_NO		1
// TX FIFO status
// 0 = There are available locations in TX FIFO
// 1 = TX FIFO is full
#define	TX_FULL		0

//// OBSERVE_TX Register
// Lost packet count (bits 7:4). Counts up to 15 lost packets
// (does not overflow).
// Reset by writing to RF_CH (i.e. radio designer is trying to tell us
// the current channel may be overused)
#define	PLOS_CNT		4
// Resent packet count (bits 3:0). Counts the packets that were resent
// in Enhanced Shockburst mode.
// Reset by sending a new packets.
#define	ARC_CNT		0

//// Skip the RDP Register, because it has only 1 bit.
// 0 = No received power detected.
// 1 = Received power detected.

//// Skip the RX_ADDR, TX_ADDR, AND RX_PW Registers, because they don't
//// use bit shifting.

//// FIFO_STATUS Register
// TX Payload Reuse
// 0 = The last TX payload is not being resent
// 1 = the last TX payload is being resent
#define	TX_REUSE		6
// TX FIFO FULL status
// 0 = TX FIFO is not full.
// 1 = TX FIFO is full.
// NOTE: the FIFOs can contain up to three payloads (max size 32 bytes
// each), so "NOT FULL" != "EMPTY"
#define	TX_FIFO_FULL	5
// TX FIFO EMPTY status
// 0 = TX FIFO is not empty.
// 1 = TX FIFO is empty.
#define	TX_FIFO_EMPTY
// RX FIFO FULL
// 0 = RX FIFO is not full
// 1 = RX FIFO is full
#define	RX_FIFO_FULL	1
// RX FIFO EMPTY
// 0 = RX FIFO is not empty
// 1 = RX FIFO is empty
#define	RX_FIFO_EMPTY	0


/* Radio SPI Commands */
#define	R_REGISTER		0x00			# Read Register
#define	W_REGISTER		0x20			# Write Register
#define	R_RX_PAYLOAD		0x61			# Read receive payload (clears FIFO; LSByte first)
#define	W_TX_PAYLOAD		0xA0			# Write transmit payload
#define	FLUSH_TX			0xE1			# Flush transmit FIFO
#define	FLUSH_RX			0xE2			# Flush receive FIFO (should not be used while an ack is being transmitted)
// Reuse transmit payload.
// Use this to continuously re-transmit the last-transmitted payload
// (in TX mode) as long as CE is held high, until the TX FIFO is flushed
// or the payload is overwritten. This should not be changed while
// transmitting.
// NOTE: This is not the same thing as Enhanced Shockburst.
#define	REUSE_TX_PL		0xE3
// Read RX payload width for the top R_RX_PAYLOAD in the RX FIFO.
// NOTE: Flush RX FIFO if the read value is larger than 32 bytes.
// NOTE: The bits in the FEATURE Register (as shown in the table
// on pg 63 of the datasheet) have to be set.
#define	R_RX_PL_WID		0x60
// Used in RX mode.
// Write Payload to be transmitted together with ACK packet on PIPE PPP
// (PPP valid in range from 000 to 101).
// Maximum three ACK packet payloads can be pending.
// Payloads with same PPP are handled using FIFO principle.
// Write payload: 1-32 bytes.
// A write operation always starts at byte 0.
// NOTE: The bits in the FEATURE Register (table on pg 63 of datasheet)
// have to be set.
#define	W_ACK_PAYLOAD		0xA8			# 1 to 32 bytes (LSByte first)
// Used in TX mode. Diables AUTOACK on this specific packet.
// NOTE: The bits in the FEATURE Register (table on pg 63 of datasheet)
// have to be set.
#define	W_TX_PAYLOAD_NO_ACK	0xB0	# 1 to 32 bytes (LSByte first)
#define	NOP				0xFF			# No operation

#endif /* NRF24L01P_H_
