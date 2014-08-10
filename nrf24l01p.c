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

#define CHANNEL				112
#define ADDRESS_LENGTH		5

// Pin definitions for chip select and chip enable on the radio module
#define CE_DDR				DDRB
#define CE_PORT				PORTB
#define CE_PIN				PB1
#define CSN_DDR				DDRB
#define CSN_PORT			PORTB
#define CSN_PIN				PB2			// SS
#define IRQ_DDR				DDRD
#define IRQ_PORT			PORTD
#define IRQ_PIN				PD3			// INT1
#define INT_SENSE_CTL_BIT1	ISC11		// This and the next INT_SENSE_CTL_BIT
#define INT_SENSE_CTL_BIT0	ISC10		// control how external interrupts are triggered
#define EXT_INTERRUPT		INT1		// Which of the external interrupt pins to use
#define INT_VECTOR			INT1_vect	// Vector for ISR

// Macros for selecting and enabling the radio
#define CSN_HIGH()		CSN_PORT |= (1 << CSN_PIN);
#define CSN_LOW()		CSN_PORT &= ~(1 << CSN_PIN);
#define CE_HIGH()		CE_PORT |= (1 << CE_PIN);
#define CE_LOW()		CE_PORT &= ~(1 << CE_PIN);

// Flag which denotes that the radio is currently transmitting
static volatile uint8_t transmit_lock;

// Tracks the payload widths of the RX pipes
statis volatile uint8_t rx_pipe_widths[6] = {32, 32, 0, 0, 0, 0};

// Holds the transmit address 
// (RX Pipe 0 is set to this address when transmitting with Auto-Ack enabled)
static volatile uint8_t tx_address[5] = {0xe7, 0xe7, 0xe7, 0xe7, 0xe7};

// Holds the receiver address for RX Pipe 0 
// (the address is overwritten when transmitting with Auto-Ack enabled)
static volatile uint8_t rx_pipe0_address[5] = {0xe7, 0xe7, 0xe7, 0xe7, 0xe7};

// The driver uses this to keep track of the success status of the 
// last 16 transmissions
static volatile uint16_t tx_history = 0xFF;

static volatile RADIO_TX_STATUS tx_last_status = RADIO_TX_SUCCESS;

extern void radio_rxhandler(uint8_t pipenumber);

void Radio_Init()
{
	transmit_lock = 0;
	
	// Disable radio during config
	CE_LOW();
	
	// Set as output the microcontroller's Slave Select (SS) and 
	// Chip Enable (CE) pins
	CE_DDR |= (1 << CE_PIN);
	CSN_DDR |= (1 << CSN_PIN);
	
	// Enable radio interrupt. This interrupt is triggered when data are 
	// received, as well as when a transmission completes.
	IRQ_DDR &= ~(1 << IRQ_PIN);
	EICRA |= (1 << INT_SENSE_CTL_BIT1);		// BIT1 = 1/BIT0 = 0: Falling edge triggers external interrupt
	EICRA &= ~(1 << INT_SENSE_CTL_BIT0);
	EIMSK |= (1 << );		// Turn on INT1 external interrupt
	
	// A 10.3 ms delay is required between power off and power on states 
	// (controlled by 3.3V supply)
	_delay_ms(11)
	
	// Configure the radio registers that are not application-dependent
	configure_registers();
	
	// Enable radio as a receiver
	CE_HIGH();
}

/**
 * Retrieve the status register
 */
static uint8_t get_status()
{
	uint8_t status = 0;
	CSN_LOW();
	
	status = SPI_Write_Byte(NOP);
	
	CSN_HIGH();
	
	return status;
}

/**
 * Set a register in the radio
 * 
 * \param reg		The register value defined in nRF24L01.h (e.g. CONFIG, EN_AA, STATUS, etc).
 * \param value	The value to write to the given register (NOTE: the whole register is overwritten)
 * \param len		Length of value ( used in for loop in SPI_Write_Byte() ).
 * \return			The value of the SPDR (SPI Data Register) - probably 
 * 					not relevant when writing data, as we are here.
 */
static uint8_t set_register(radio_register_t reg, uint8_t* value, uint8_t len)
{
	uint8_t status;
	CSN_LOW();
	
	status = SPI_Write_Byte(W_REGISTER | (REGISTER_MASK & reg));
	SPI_Write_Block(value, len);
	
	CSN_HIGH();
	
	return status;
}

/** Retrieve a register value from the radio.
 * 
 * \param reg		The register value defined in nRF24L01.h (e.g. CONFIG, EN_AA, STATUS, etc).
 * \param buffer	A contiguous memory block into which the register contents will be copied.
 * 					If the buffer is too long for the register contents, then the remaining 
 * 					bytes will be overwritten with 0xFF.
 * \param len		Length of the buffer
 * \return status	Value of the SPDR (SPI Data Register)
 */
static uint8_t get_register(radio_register_t reg, uint8_t* buffer, uint8_t len)
 {
	 uint8_t status, i;
	 for (i = 0; i < len; i++)
	 {
		 // If the buffer is too long for the register results, then the 
		 // radio will interpret the extra bytes as instructions.
		 // To remove this risk, we set the buffer elements to NOP instructions
		 buffer[i] = 0xFF;
	 }
	 CSN_LOW();
	 
	 status = SPI_Write_Byte(R_REGISTER | (REGISTER_MASK & reg));
	 SPI_ReadWrite_Block(NULL, buffer, len);
	 
	 CSN_HIGH();
	 
	 return status;
 }

/**
 * Send an instruction to the nRF24L01
 * 
 * \param instruction	The instruction to send (see the bottom of nRF24L01.h)
 * \param data			An array of argument data to the instruction.
 * 						If len == 0, then this may be NULL.
 * \param buffer		An array for the instruction's return data.
 * 						This can be NULL if the instruction has no output.
 * \param len			The length of the data and buffer arrays.
 */
static void send_instruction(uint8_t instruction, uint8_t* data, uint8_t* buffer, uint8_t len)
{
	CSN_LOW();
	
	// Send the instruction
	SPI_Write_Byte(instruction);
	// Pass the arguments, if any
	if (len > 0)
	{
		if (buffer == NULL)
			SPI_Write_Block(data, len);
		else
			SPI_ReadWrite_Block(data, buffer, len);
	}
	
	// Resync SPI
	CSN_HIGH();
}

/**
 * Switch the radio to receive mode. 
 * If the radio is already in receive mode, this does nothing.
 */
static void set_rx_mode()
{
	uint8_t config;
	get_register(CONFIG, &config, 1);
	
	if ((config & (1 << PRIM_RX)) == 0)
	{
		config |= (1 << PRIM_RX);
		set_register(CONFIG, &config, 1);
		
		// The radio takes 130 us to power up the receiver.
		_delay_us(65);
		_delay_us(65);
	}
}

/**
 * Switch the radio to transmit mode.
 * If the radio is already in transmit mode, this does nothing.
 */
static void set_tx_mode()
{
	uint8_t config;
	get_register(CONFIG, &config, 1);
	
	if ((config & (1 << PRIM_RX)) != 0)
	{
		config &= ~(1 << PRIM_RX);
		set_register(CONFIG, &config, 1);
		
		// The radio takes 130 us to power up the transmitter.
		// You can delete this if you're sending large packets 
		// (> 25 byte? not sure)
		// because sending that many bytes over SPI can take this long.
		_delay_us(65);
		_delay_us(65);
	}
}

/**
 * Reset the Pipe 0 address if Pipe 0 is enabled. 
 * This is necessary when the radio is using Enhanced Shockburst, because
 * the Pipe 0 address is set to the transmit address while the radio is 
 * transmitting
 * (this is how the radio receives Auto-Ack packets).
 */
static void reset_pipe0_address()
{
	if (rx_pipe_widths[RADIO_PIPE_0] != 0)
	{
		// Reset the Pipe 0 address if Pipe 0 is enabled.
		set_register(RX_ADDR_P0, (uint8_t*) rx_pipe0_address, ADDRESS_LENGTH);
	}
}

/**
 * Configure radio defaults and turn on the radio in receive mode.
 * This configures the radio to its max-power, max-packet-header-length 
 * settings.
 * If you want to reduce power consumption or increase on-air payload 
 * bandwidth, you'll have to change the config.
 */
static void configure_registers()
{
	uint8_t value;
	
	SPI_Init();
	
	// Set address width to 5 bytes.
	//		0b11 for 5 bytes
	// 		0b10 for 4 bytes
	//		0b01 for 3 bytes
	value = ADDRESS_LENGTH - 2;		
	set_register(SETUP_AW, &value, 1);
	
	// Set Enhanced Shockburst retry to every 586 us, up to 5 times.
	// If packet collisions are a problem even with Auto-Ack enabled, 
	// then consider changing the retry delay to be different on the 
	// different stations, so that they do not keep colliding on each 
	// retry.
	value = 0x15;
	set_register(SETUP_RETR, &value, 1);
	
	// Set to use 2.4 GHz channel (defined at top of this file)
	value = CHANNEL;
	set_register(RF_CH, &value, 1);
	
	// Set radio to 2 Mbps and high power. Leave LNA_HCURR at its default
	value = (1 << RF_DR) | (1 << LNA_HCURR);
	set_register(RF_SETUP, &value, 1);
	
	// Enable 2-byte CRC and power up in receive mode
	value = (1 << EN_CRC) | (1 << CRC0) | (1 << PWR_UP) | (1 << PRIM_RX);
	set_register(CONFIG, &value, 1);
	
	// Clear the interrupt flags, in case the radio's still asserting an 
	// old, unhandled interrupt
	value = (1 << RX_DR) | (1 << TX_DS) | (1 << MAX_RT);
	set_register(STATUS, &value, 1);
	
	// Flush the FIFOs, in case there are old data in them.
	send_instruction(FLUSH_TX, NULL, NULL, 0);
	send_instruction(FLUSH_RX, NULL, NULL, 0);
}

/* Interrupt handler */
ISR(INT_VECTOR) // See #definition at top
{
	// To be completed
}
