/**
	nRF24L01+ Register Definitions
	File: nrf24l01p.h
*/

#ifndef	NRF24L01P_H_
#define	NRF24L01P_H_
#endif

/* Memory Map */
#define	nrf24l01p_CONFIG		0x00		# Configuration Register
#define	nrf24l01p_EN_AA			0x01		# Enable "Auto Acknowledgment" feature
#define	nrf24l01p_EN_RXADDR		0x02		# Enabled RX Addresses
#define	nrf24l01p_SETUP_AW		0x03		# Setup of Address Widths
#define	nrf24l01p_SETUP_RETR	0x04		# Setup of Automatic Retransmission
#define	nrf24l01p_RF_CH			0x05		# RF Channel
#define	nrf24l01p_RF_SETUP		0x06		# RF Setup Register
#define	nrf24l01p_STATUS		0x07		# Status Register
#define	nrf24l01p_OBSERVE_TX	0x08		# Transmit Observe Register
#define	nrf24l01p_RPD			0x09		# Received Power Detector (alerts user when foreign signal is detected on current transmission channel)
#define	nrf24l01p_RX_ADDR_P0	0x0A		# Receive Address DATA PIPE 0
#define	nrf24l01p_RX_ADDR_P0	0x0B		# Receive Address DATA PIPE 1
#define	nrf24l01p_RX_ADDR_P0	0x0C		# Receive Address DATA PIPE 2
#define	nrf24l01p_RX_ADDR_P0	0x0D		# Receive Address DATA PIPE 3
#define	nrf24l01p_RX_ADDR_P0	0x0E		# Receive Address DATA PIPE 4
#define	nrf24l01p_RX_ADDR_P0	0x0F		# Receive Address DATA PIPE 5
#define	nrf24l01p_TX_ADDR		0x10		# Transmit Address
#define	nrf24l01p_RX_PW_P0		0x11		# Payload Data Width for PIPE 0 (# of bites per payload)
#define	nrf24l01p_RX_PW_P1		0x12		# Payload Data Width for PIPE 1
#define	nrf24l01p_RX_PW_P2		0x13		# Payload Data Width for PIPE 2
#define	nrf24l01p_RX_PW_P3		0x14		# Payload Data Width for PIPE 3
#define	nrf24l01p_RX_PW_P4		0x15		# Payload Data Width for PIPE 4
#define	nrf24l01p_RX_PW_P5		0x16		# Payload Data Width for PIPE 5
#define	nrf24l01p_FIFO_STATUS	0x17		# FIFO Status Register
#define	nrf24l01p_DYNPD			0x1C		# Enable dynamic payload length
#define	nrf24l01p_FEATURE		0x1D		# Feature Register

/* Register Bit Mnemonics */
#define	nrf24l01p_MASK_RX_DR	6			# Mask interrupt caused by RX_DR (i.e. turn off RX data ready interrupt)
#define	nrf24l01p_MASK_TX_DS	5			# Mask interrupt caused by TX_DS (i.e. turn off TX data sent interrupt)
#define	nrf24l01p_MASK_MAX_RT	4			# Mask interrupt caused by MAX_RT (i.e. turn off "Maximum number of packet resend retries reached" interrupt)
#define	nrf24l01p_EN_CRC		3			# Enable CRC (Cyclic Redundancy Check) 
#define	nrf24l01p_CRCO			2			# CRC encoding scheme
#define	nrf24l01p_PWR_UP		1			# POWER UP/POWER DOWN
#define	nrf24l01p_PRIM_RX		0			# RX/TX Control (i.e. set this module to either receive of transmit mode)
#define	nrf24l01p_ENAA_P5		5			# Enable Auto Acknowledgment feature for DATA PIPE 5
#define	nrf24l01p_ENAA_P4		4			# Enable Auto Acknowledgment feature for DATA PIPE 4
#define	nrf24l01p_ENAA_P3		3			# Enable Auto Acknowledgment feature for DATA PIPE 3
#define	nrf24l01p_ENAA_P2		2			# Enable Auto Acknowledgment feature for DATA PIPE 2
#define	nrf24l01p_ENAA_P1		1			# Enable Auto Acknowledgment feature for DATA PIPE 1
#define	nrf24l01p_ENAA_P0		0			# Enable Auto Acknowledgment feature for DATA PIPE 0
#define	nrf24l01p_ERX_P5		5			# Enable DATA PIPE 5
#define	nrf24l01p_ERX_P4		4			# Enable DATA PIPE 4
#define	nrf24l01p_ERX_P3		3			# Enable DATA PIPE 3
#define	nrf24l01p_ERX_P2		2			# Enable DATA PIPE 2
#define	nrf24l01p_ERX_P1		1			# Enable DATA PIPE 1
#define	nrf24l01p_ERX_P0		0			# Enable DATA PIPE 0
#define	nrf24l01p_AW			0			# RX/TX Address field width
#define	nrf24l01p_ARD			4			# Auto Retransmit Delay
#define	nrf24l01p_ARC			0			# Auto Retransmit Count
#define	nrf24l01p_RF_CH			0
