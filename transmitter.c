/**
 * Get compass heading readings from HMC6352 Digital Compass
 * and send them to desktop PC using USART.
 */

//------ Includes ------/ 
#include <avr/io.h>
#include <util/delay.h>
#include <avr/power.h>

#include "USART.h"


int main(void) {
	
	//------ Initialization ------//
	clock_prescale_set(clock_div_1);
	initUSART();
	printString("\r\n====== I2C Compass ======\r\n");
	initI2C();
	
	while (1) {
		ic2Start();
		
		// TODO: read HMC6352 datasheet, then write code to talk to it.
	}
	
	return (0);
}
