//----------------------------------------------------------------------------
// C main line
//----------------------------------------------------------------------------

#include <m8c.h>        	// part specific constants and macros
#include "PSoCAPI.h"    	// PSoC API definitions for all User Modules
#include "psocdynamic.h"
#include <stdlib.h>
#pragma interrupt_handler Timer8_1_ISR

// These defines are used as parameters of the commToggle function.
// Passing one or the other in the function call switches the system between TX and RX modes.
#define		RX_MODE						(1)
#define		TX_MODE						(0)

// This is used to create a heirarchy for the use of multiple controllers.
// One controller is programmed as a master (with a define of 1). All other controllers
// are programmed as slaves. The difference is that masters start up in receive mode,
// whereas slaves start up transmitting a request to connect to the robot bus.
#define		MASTER						(0)

// This function receives a mode identifier as a parameter and toggles the
// system configuration between receive and transmit modes for half duplex UART.
void commToggle(int mode);

// This function waits for a transmission to finish in TX mode.
void xmitWait(void);

// Keeps track of whether or not this module is configured.
int configured;

void main()
{
	configured = 0;			// This integer keeps track of whether or not we have received a valid response.
	//PRT1DR |= 0b00000001;	// Turn the LED off (active low).
	M8C_EnableGInt;			// Turn on global interrupts for the transmission timeout timer.
	
	M8C_EnableIntMask(INT_MSK0,INT_MSK0_GPIO); //activate GPIO ISR
	
	// If this controller is the master, start out in RX mode.
	// Otherwise, this controller is a slave and starts in TX mode.
	if(MASTER)
	{
		commToggle(RX_MODE);	// Toggle into RX mode and wait for a request.
		
		while(!configured)
		{
			if(RECEIVE_cGetChar() == 'M')
			{
				if(RECEIVE_cGetChar() == 'a')
				{
					if(RECEIVE_cGetChar() == 'r')
					{
						if(RECEIVE_cGetChar() == 'c')
						{
							if(RECEIVE_cGetChar() == 'o')
							{
								commToggle(TX_MODE);	// Switch to TX mode.
								
								// Transmit the response.
								TRANSMIT_PutChar('P');
								TRANSMIT_PutChar('o');
								TRANSMIT_PutChar('l');
								TRANSMIT_PutChar('o');
								
								//configured = 1;
								xmitWait();				// Wait for the transmission to finish.
								commToggle(RX_MODE);	// Switch back to receive mode.
							}
						}
					}
				}
			}
		}
	}
	else if(!MASTER)
	{
		// Start response timeout timer and enable its interrupt routine.
		// This will wait one timeout period, and transmit the initial message.
		commToggle(RX_MODE);
		
		while(!configured)
		{
			if(RECEIVE_cGetChar() == 'P')
			{
				if(RECEIVE_cGetChar() == 'o')
				{
					if(RECEIVE_cGetChar() == 'l')
					{
						if(RECEIVE_cGetChar() == 'o')
						{
							// Start response timeout timer and enable its interrupt routine.
							//Timer8_1_DisableInt();
							//Timer8_1_Stop();
							
							// This module is configured, so the next module can now be heard.
							//configured = 1;
							
							// Turn on relay-like object.
						}
					}
				}
			}
		}
	}
	
	//PRT1DR &= 0b11111110;		// Turn the LED on (active low).
	
	while(1)
	{
		// Sit and spin for now, but eventually this is where we will connect to the bus
		// and eavesdrop for more commands.
	}
}

// This function does nothing while the transmission finishes.
// It takes about 18 microseconds for a transmission to finish after the last byte is queued.
// This function takes approximately 50 microseconds on the PSoC 29466 micro.
void xmitWait(void)
{	
	int i;

	for(i = 0; i < 25; i++)
	{
		 // Do nothing.
	}
}

// This function allows the program to pass an RX or TX mode flag for switching between modes on the
// half duplex UART serial communication line.
void commToggle(int mode)
{
	if(mode == RX_MODE)
	{
		// Stop from transmitting any more.
		TRANSMIT_Stop();
		// Write a constant high to the data bus.  This in combination with disconnecting the pin
		// from the global bus is to avoid a false start bit (a zero) when we unload the configuration.
		PRT0DR |= 0b10000000;
		// Disconnect P07 from the global bus.
		PRT0GS &= 0b01111111;
		// Unload the transmitter configuration.
		UnloadConfig_transmitter_config();
		// Load the receiver configuration.
		LoadConfig_receiver_config();
		// Reconnect to the global bus.
		PRT0GS |= 0b10000000;
		
		// If this module is configured, allow modules past this to listen on P27.
		// Otherwise, we make sure they're deaf.
		if((configured) && (!MASTER))
		{
			// Turn on relay-like object
			
			// Make sure the timer is off, although a config reload should have it off by default.
			Timer8_1_Stop();
		}
		else if((!configured) && (!MASTER))
		{
			// Turn off relay-like object.
			
			// Start response timeout timer and enable its interrupt routine.
			Timer8_1_EnableInt();
			Timer8_1_Start();
		}
		
		// Start the receiver.
		RECEIVE_Start(RECEIVE_PARITY_NONE);
	}
	else if(mode == TX_MODE)
	{
		// Stop receiving information.
		RECEIVE_Stop();
		// Disconnect from the global bus.
		PRT0GS &= 0b01111111;
		// Unload the receiver configuration.
		UnloadConfig_receiver_config();
		// Load the transmitter configuration.
		LoadConfig_transmitter_config();
		// Reconnect to the global bus.
		PRT0GS |= 0b10000000;
		// Start the transmitter.
		TRANSMIT_Start(TRANSMIT_PARITY_NONE);
	}
}

void Timer8_1_ISR(void)
{
	commToggle(TX_MODE);	// Toggle into TX mode.
			
	// Transmit the characters for "Marco".
	TRANSMIT_PutChar(42);
	TRANSMIT_PutChar(42);
	TRANSMIT_PutChar('0');
	TRANSMIT_PutChar('1');
	
	
	xmitWait();				// Wait for the transmission to finish.
	commToggle(RX_MODE);	// Listen for the response.
	
	PRT1DR |= 0b00000001;
	PRT1DR &= 0b11111110;
	
	M8C_ClearIntFlag(INT_CLR0,Timer8_1_INT_MASK);
}