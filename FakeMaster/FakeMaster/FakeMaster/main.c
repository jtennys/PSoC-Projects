//----------------------------------------------------------------------------
// C main line
//----------------------------------------------------------------------------

#include <m8c.h>        	// part specific constants and macros
#include "PSoCAPI.h"    	// PSoC API definitions for all User Modules
#include "psocdynamic.h"
#include <stdlib.h>
#pragma interrupt_handler TX_TIMEOUT_ISR
#pragma interrupt_handler RX_TIMEOUT_ISR

// These defines are used as parameters of the commToggle function.
// Passing one or the other in the function call switches the system between TX and RX modes.
#define		RX_MODE						(1)
#define		TX_MODE						(0)

// These defines are used as comparisons to find what port the newest module is connected to.
#define		PORT_1						('A')
#define		PORT_2						('B')
#define		PORT_3						('C')
#define		PORT_4						('D')

// These defines are used as transmission indicators.
#define		START_TRANSMIT				(248)	// Indicates the beginning of a transmission.
#define		END_TRANSMIT				(85)	// Indicates the end of a transmission.
#define		HELLO_BYTE					(200)	// Indicates master is ready to talk.
#define		ID_ASSIGNMENT				(201)	// Indicates an ID assignment from the master.
#define		ID_ASSIGN_OK				(202)	// Indicates an ID assignment is acknowledged.
#define		PING						(203)	// Indicates that someone is pinging someone else.
#define		CLEAR_CONFIG				(204)	// Indicates that the master is asking for a config clear.
#define		CONFIG_CLEARED				(205)	// Indicates that a module has cleared its own config.
#define		MASTER_ID					(0)		// The master node's ID.
#define		BROADCAST					(254)	// The broadcast ID for talking to all nodes.
#define		BLANK_MODULE_ID				(251)	// This is the ID of an unconfigured module.

// These defines are used for transmission timing.
#define 	RX_TIMEOUT_DURATION			(50)	// This is receive wait time in 1 ms units.

// This is the maximum number of allowable modules per branch out from the master
#define		MAX_MODULES					(250)

// This function receives a mode identifier as a parameter and toggles the
// system configuration between receive and transmit modes for half duplex UART.
void commToggle(int mode);

// This function pings the index passed to it. Returns 1 on success, 0 on fail.
int pingModule(int module_id);

// This function assigns an ID to a module.
int assignID(int assigned_ID);

int validTransmission(void);

void decodeTransmission(void);

void sayHello(void);

int clearConfig(int module_id);

// This flag is set if there is a timeout.
int TIMEOUT;

int NUM_MODULES;			// Stores the number of modules that have been discovered.
char COMMAND_SOURCE;		// Stores who the current command is from.
char COMMAND_DESTINATION;	// Stores who the current command is for.
char COMMAND_TYPE;			// Stores the type of command that was just read.
char PARAM;					// Stores a parameter that accompanies the command (if any).

void main()
{
	NUM_MODULES = 0;

	M8C_EnableGInt;			// Turn on global interrupts for the transmission timeout timer.
	M8C_EnableIntMask(INT_MSK0,INT_MSK0_GPIO); //activate GPIO ISR
	
	// Unload all configs.
	UnloadConfig_transmitter_config();
	UnloadConfig_receiver_config();
	
	commToggle(RX_MODE);
	
	while(TIMEOUT < 1000)
	{
	
	}
	
	commToggle(RX_MODE);
	
	// This loop continuously probes and listens at intervals
	// set by the RX_TIMEOUT_DURATION variable.
	while(1)
	{
		if(RECEIVE_cReadChar() == START_TRANSMIT)
		{	
			if(validTransmission())
			{
				decodeTransmission();
			}
		}
		if(TIMEOUT >= RX_TIMEOUT_DURATION)
		{
			LCD_1_Start();
			LCD_1_Position(0,0);
			LCD_1_PrHexInt(NUM_MODULES);
			LCD_1_Position(0,4);
			LCD_1_PrCString(" Modules!");
			
			TIMEOUT = 0;
			while(TIMEOUT < RX_TIMEOUT_DURATION);
			
			// If we are not maxed out on modules, look for more.
			if(NUM_MODULES < MAX_MODULES)
			{
				sayHello();
			}
		}
	}
}

int pingModule(int module_id)
{
	int response = 0;
	
	commToggle(TX_MODE);	// Toggle into TX mode.
			
	// Transmit a hello.
	TRANSMIT_PutChar(START_TRANSMIT);
	TRANSMIT_PutChar(START_TRANSMIT);
	TRANSMIT_PutChar(MASTER_ID);
	TRANSMIT_PutChar(module_id);
	TRANSMIT_PutChar(PING);
	TRANSMIT_PutChar(END_TRANSMIT);
	
	// Wait for the transmission to finish.
	while(!( TRANSMIT_bReadTxStatus() & TRANSMIT_TX_COMPLETE));
	
	commToggle(RX_MODE);	// Listen for the response.
	
	RX_TIMEOUT_Stop();
	TIMEOUT = 0;
	RX_TIMEOUT_Start();
	
	while((TIMEOUT < RX_TIMEOUT_DURATION) && (!response))
	{
		if(RECEIVE_cReadChar() == START_TRANSMIT)
		{	
			if(RECEIVE_cGetChar() == START_TRANSMIT)
			{
				if(RECEIVE_cGetChar() == module_id)
				{
					if(RECEIVE_cGetChar() == MASTER_ID)
					{
						if(RECEIVE_cGetChar() == PING)
						{
							response = 1;
						}
					}
				}
			}
		}
	}
	
	RX_TIMEOUT_Stop();
	TIMEOUT = 0;
	RX_TIMEOUT_Start();
	
	return response;
}

int assignID(int assigned_ID)
{
	int success = 0;		// Stores 0 on fail, 1 on success.
	
	commToggle(TX_MODE);	// Switch to TX mode.

	// Transmit the assignment.
	TRANSMIT_PutChar(START_TRANSMIT);
	TRANSMIT_PutChar(START_TRANSMIT);
	TRANSMIT_PutChar(MASTER_ID);
	TRANSMIT_PutChar(BLANK_MODULE_ID);
	TRANSMIT_PutChar(ID_ASSIGNMENT);
	TRANSMIT_PutChar(assigned_ID);
	TRANSMIT_PutChar(END_TRANSMIT);
	
	// Wait for the transmission to finish.
	while(!( TRANSMIT_bReadTxStatus() & TRANSMIT_TX_COMPLETE));
	
	commToggle(RX_MODE);	// Switch back to receive mode.
	
	RX_TIMEOUT_Stop();
	TIMEOUT = 0;
	RX_TIMEOUT_Start();
	
	while((TIMEOUT < RX_TIMEOUT_DURATION) && (!success))
	{
		if(RECEIVE_cReadChar() == START_TRANSMIT)
		{	
			if(RECEIVE_cGetChar() == START_TRANSMIT)
			{
				if(RECEIVE_cGetChar() == assigned_ID)
				{
					if(RECEIVE_cGetChar() == MASTER_ID)
					{
						if(RECEIVE_cGetChar() == ID_ASSIGN_OK)
						{
							success = 1;
						}
					}
				}
			}
		}
	}
	
	RX_TIMEOUT_Stop();
	TIMEOUT = 0;
	RX_TIMEOUT_Start();
	
	return success;
}

int clearConfig(int module_id)
{
	int response = 0;
	
	commToggle(TX_MODE);	// Toggle into TX mode.
			
	// Transmit a hello.
	TRANSMIT_PutChar(START_TRANSMIT);
	TRANSMIT_PutChar(START_TRANSMIT);
	TRANSMIT_PutChar(MASTER_ID);
	TRANSMIT_PutChar(module_id);
	TRANSMIT_PutChar(CLEAR_CONFIG);
	TRANSMIT_PutChar(END_TRANSMIT);
	
	// Wait for the transmission to finish.
	while(!( TRANSMIT_bReadTxStatus() & TRANSMIT_TX_COMPLETE));
	
	commToggle(RX_MODE);	// Listen for the response.
	
	if(module_id != BROADCAST)
	{
		RX_TIMEOUT_Stop();
		TIMEOUT = 0;
		RX_TIMEOUT_Start();
		
		while((TIMEOUT < RX_TIMEOUT_DURATION) && (!response))
		{
			if(RECEIVE_cReadChar() == START_TRANSMIT)
			{	
				if(RECEIVE_cGetChar() == START_TRANSMIT)
				{
					if(RECEIVE_cGetChar() == module_id)
					{
						if(RECEIVE_cGetChar() == MASTER_ID)
						{
							if(RECEIVE_cGetChar() == CONFIG_CLEARED)
							{
								response = 1;
							}
						}
					}
				}
			}
		}
		
		RX_TIMEOUT_Stop();
		TIMEOUT = 0;
		RX_TIMEOUT_Start();
	}
	
	return response;
}

// This function transmits a hello message.
void sayHello(void)
{
	commToggle(TX_MODE);				// Toggle into TX mode.
			
	// Transmit a hello.
	TRANSMIT_PutChar(START_TRANSMIT);
	TRANSMIT_PutChar(START_TRANSMIT);
	TRANSMIT_PutChar(MASTER_ID);
	TRANSMIT_PutChar(BLANK_MODULE_ID);
	TRANSMIT_PutChar(HELLO_BYTE);
	TRANSMIT_PutChar(END_TRANSMIT);
	
	// Wait for the transmission to finish.
	while(!( TRANSMIT_bReadTxStatus() & TRANSMIT_TX_COMPLETE));
	
	commToggle(RX_MODE);				// Listen for the response.
}

// This function returns whether or not a valid transmission has been received.
int validTransmission(void)
{
	int valid_transmit = 0;
	
	if(RECEIVE_cGetChar() == START_TRANSMIT)
	{
		COMMAND_SOURCE = RECEIVE_cGetChar();
		COMMAND_DESTINATION = RECEIVE_cGetChar();
		COMMAND_TYPE = RECEIVE_cGetChar();
		PARAM = RECEIVE_cGetChar();
		
		valid_transmit = 1;
	}
	
	return valid_transmit;
}

// This function decodes the transmission and takes the correct action.
void decodeTransmission(void)
{
	char TEMP_CHILD;
	
	if(COMMAND_TYPE == HELLO_BYTE)			// Someone else is out there!
	{
		// If this is for me, assign them an ID.
		if(COMMAND_DESTINATION == MASTER_ID)
		{
			NUM_MODULES++;					// Increment the number of modules connected.
			
			//TEMP_CHILD = PARAM;

			if(!assignID(NUM_MODULES))
			{
				// If the module did not respond that the ID was assigned,
				// make an effort to ping it in case that transmission was lost
				// before ultimately deciding that the module didn't configure.
				if(!pingModule(NUM_MODULES))
				{
					if(!pingModule(NUM_MODULES))
					{
						if(!pingModule(NUM_MODULES))
						{
							if(!pingModule(NUM_MODULES))
							{
								if(!pingModule(NUM_MODULES))
								{
									NUM_MODULES--;
								}
							}
						}
					}
				}
			}
		
			// Print the child number to the LED's...
//			if(TEMP_CHILD == PORT_1)
//			{
//				PRT1DR |= 0b11111111;
//				PRT1DR &= 0b11110111;
//			}
//			else if(TEMP_CHILD == PORT_2)
//			{
//				PRT1DR |= 0b11111111;
//				PRT1DR &= 0b11011111;
//			}
//			else if(TEMP_CHILD == PORT_3)
//			{
//				PRT1DR |= 0b11111111;
//				PRT1DR &= 0b11010111;
//			}
//			else if(TEMP_CHILD == PORT_4)
//			{
//				PRT1DR |= 0b11111111;
//				PRT1DR &= 0b01111111;
//			}
		}
	}
}

// This function allows the program to pass an RX or TX mode flag for switching between modes on the
// half duplex UART serial communication line.
void commToggle(int mode)
{
	// Disconnect from the global bus and leave the pin high.
	PRT0DR |= 0b10000000;
	PRT0GS &= 0b01111111;

	if(mode == RX_MODE)
	{
		// Unload the transmitter configuration.
		UnloadConfig_transmitter_config();
		// Load the receiver configuration.
		LoadConfig_receiver_config();
		
		// Clear the buffer.
		RECEIVE_CmdReset();
		// Start the receiver.
		RECEIVE_Start(RECEIVE_PARITY_NONE);
		
		// Start response timeout timer and enable its interrupt routine.
		TIMEOUT = 0;
		RX_TIMEOUT_EnableInt();
		RX_TIMEOUT_Start();
	}
	else if(mode == TX_MODE)
	{
		// Unload the receiver configuration.
		UnloadConfig_receiver_config();
		// Load the transmitter configuration.
		LoadConfig_transmitter_config();
		// Start the transmitter.
		TRANSMIT_Start(TRANSMIT_PARITY_NONE);
		
		TIMEOUT = 0;
		TX_TIMEOUT_EnableInt();	// Make sure interrupts are enabled.
		TX_TIMEOUT_Start();		// Start the timer.
		
		while(!TIMEOUT)
		{
			// Do nothing while we wait for one timeout period.
			// This is to allow everyone to get in the right configuration.
		}
		
		TX_TIMEOUT_Stop();		// Stop the timer.
		TIMEOUT = 0;			// Reset the timeout flag.
	}
	
	// Make sure to keep the LED on (active low).
	PRT2DR &= 0b11111110;
	
	// Reconnect to the global bus.
	PRT0GS |= 0b10000000;
}

void TX_TIMEOUT_ISR(void)
{	
	TIMEOUT++;
	
	M8C_ClearIntFlag(INT_CLR0,TX_TIMEOUT_INT_MASK);
}

void RX_TIMEOUT_ISR(void)
{	
	TIMEOUT++;
	
	M8C_ClearIntFlag(INT_CLR0,RX_TIMEOUT_INT_MASK);
}