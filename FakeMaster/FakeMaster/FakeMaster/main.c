//----------------------------------------------------------------------------
// C main line
//----------------------------------------------------------------------------

#include <m8c.h>        	// part specific constants and macros
#include "PSoCAPI.h"    	// PSoC API definitions for all User Modules
#include "psocdynamic.h"
#include <stdlib.h>
#pragma interrupt_handler TX_TIMEOUT_ISR
#pragma interrupt_handler RX_TIMEOUT_ISR

// These defines are used as parameters of the configToggle function.
// Passing one or the other in the function call switches the system between PC, TX, and RX modes.
#define		PC_MODE						(2)
#define		RX_MODE						(1)
#define		TX_MODE						(0)

// These defines are used as comparisons to find what port the newest module is connected to.
#define		PORT_1						('A')
#define		PORT_2						('B')
#define		PORT_3						('C')
#define		PORT_4						('D')

// These defines are used as transmission indicators.
#define		START_TRANSMIT				(252)	// Indicates the beginning of a transmission.
#define		END_TRANSMIT				(253)	// Indicates the end of a transmission.
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
#define 	RX_TIMEOUT_DURATION			(5)		// This is receive wait time in 1 ms units.

// These defines are used for the initial probing stage, where receive waits are longer to make
// sure of transmission failure or success.
#define		HELLO_RX_TIMEOUT			(150)	// This is receive wait time in 1 ms units.
#define		MAX_TIMEOUTS				(5)		// Number of timeouts allowed before hello mode exit.

// This is the maximum number of allowable modules per branch out from the master
#define		MAX_MODULES					(250)

// This function receives a mode identifier as a parameter and toggles the
// system configuration between receive and transmit modes for half duplex UART.
void configToggle(int mode);

// This function pings the index passed to it. Returns 1 on success, 0 on fail.
int pingModule(int module_id);

// This function assigns an ID to a module.
int assignID(int assigned_ID);

int validTransmission(void);

void decodeTransmission(void);

void sayHello(void);

void moveMotor(int motor_id);

int clearConfig(int module_id);
// This function checks the current mode and unloads the configuration for that mode.
void unloadAllConfigs(void);
// This function unloads the configuration corresponding to the number passed to it.
void unloadConfig(int config_num);
// Initialization function for the slave module controllers.
void initializeSlaves(void);
// Static wait time of approximately 50 microseconds for use after starting a transmission.
void xmitWait(void);

// This flag is set if there is a timeout.
int TIMEOUT;

int NUM_MODULES;			// Stores the number of modules that have been discovered.
char COMMAND_SOURCE;		// Stores who the current command is from.
char COMMAND_DESTINATION;	// Stores who the current command is for.
char COMMAND_TYPE;			// Stores the type of command that was just read.
char PARAM;					// Stores a parameter that accompanies the command (if any).

int STATE;					// Stores the current configuration state of the system.

void main()
{	
	int tempValue = 0;
	float angle = 0;
	
	NUM_MODULES = 0;
	
	M8C_EnableGInt;			// Turn on global interrupts for the transmission timeout timer.
	M8C_EnableIntMask(INT_MSK0,INT_MSK0_GPIO); //activate GPIO ISR
	
	unloadAllConfigs();
	configToggle(RX_MODE);
	
	// Sit and wait for the worst case setup time to occur.
	while(TIMEOUT < (HELLO_RX_TIMEOUT*2))
	{

	}
	
	// Initialize all of the slave modules.
	initializeSlaves();
	
	while(1)
	{	
//		while(!UART_1_bCmdCheck()) { }
//		
//		decodeTransmission();

		tempValue = 0;
		
		configToggle(TX_MODE);	// Toggle into TX mode.
		
		TRANSMIT_PutChar(255);			// Start byte one
		TRANSMIT_PutChar(255);			// Start byte two
		TRANSMIT_PutChar(3);			// Servo ID
		TRANSMIT_PutChar(4);			// The instruction length.
		TRANSMIT_PutChar(2);			// The instruction to carry out.
		TRANSMIT_PutChar(36);			// The address to read/write from/to.
		TRANSMIT_PutChar(2);			// The value to write or number of bytes to read.
		TRANSMIT_PutChar(208);			// This is the checksum.
		
		// Wait for the transmission to finish.
		while(!( TRANSMIT_bReadTxStatus() & TRANSMIT_TX_COMPLETE));
		
		xmitWait();
		
		configToggle(RX_MODE);	// Listen for the response.
	
		RX_TIMEOUT_Stop();
		TIMEOUT = 0;
		RX_TIMEOUT_Start();
		
		while(TIMEOUT < RX_TIMEOUT_DURATION)
		{
			if(RECEIVE_cReadChar() == 255)
			{
				PARAM = RECEIVE_cGetChar();
				PARAM = RECEIVE_cGetChar();
				PARAM = RECEIVE_cGetChar();
				PARAM = RECEIVE_cGetChar();
				PARAM = RECEIVE_cGetChar();
				tempValue += PARAM;
				PARAM = RECEIVE_cGetChar();
				tempValue += PARAM*256;
				
				angle = (((float)tempValue)/1023.0)*300.0;
				
				TIMEOUT = RX_TIMEOUT_DURATION;
			}
		}
		
		RX_TIMEOUT_Stop();
		TIMEOUT = 0;
		
		LCD_2_Start();
		LCD_2_Position(0,0);
		LCD_2_PrCString("                ");
		LCD_2_Position(0,0);
		LCD_2_PrCString("Angle: ");
		LCD_2_Position(0,7);
		LCD_2_PrString(ftoa(angle,&tempValue));
		
	}
}

int pingModule(int module_id)
{
	int response = 0;
	
	configToggle(TX_MODE);	// Toggle into TX mode.
			
	// Transmit a hello.
	TRANSMIT_PutChar(START_TRANSMIT);
	TRANSMIT_PutChar(START_TRANSMIT);
	TRANSMIT_PutChar(MASTER_ID);
	TRANSMIT_PutChar(module_id);
	TRANSMIT_PutChar(PING);
	TRANSMIT_PutChar(END_TRANSMIT);
	TRANSMIT_PutChar(END_TRANSMIT);
	
	// Wait for the transmission to finish.
	while(!( TRANSMIT_bReadTxStatus() & TRANSMIT_TX_COMPLETE));
	
	xmitWait();
	
	configToggle(RX_MODE);	// Listen for the response.
	
	RX_TIMEOUT_Stop();
	TIMEOUT = 0;
	RX_TIMEOUT_Start();
	
	while((TIMEOUT < RX_TIMEOUT_DURATION) && (!response))
	{
		if(RECEIVE_cReadChar() == START_TRANSMIT)
		{	
			if(validTransmission())
			{
				if(COMMAND_TYPE == PING)	// This is the response we are looking for.
				{
					// If this is for me, check who it was from.
					if(COMMAND_DESTINATION == MASTER_ID)
					{
						if(COMMAND_SOURCE == module_id)
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
	
	configToggle(TX_MODE);	// Switch to TX mode.

	// Transmit the assignment.
	TRANSMIT_PutChar(START_TRANSMIT);
	TRANSMIT_PutChar(START_TRANSMIT);
	TRANSMIT_PutChar(MASTER_ID);
	TRANSMIT_PutChar(BLANK_MODULE_ID);
	TRANSMIT_PutChar(ID_ASSIGNMENT);
	TRANSMIT_PutChar(assigned_ID);
	TRANSMIT_PutChar(END_TRANSMIT);
	TRANSMIT_PutChar(END_TRANSMIT);
	
	// Wait for the transmission to finish.
	while(!( TRANSMIT_bReadTxStatus() & TRANSMIT_TX_COMPLETE));
	
	xmitWait();
	
	configToggle(RX_MODE);	// Switch back to receive mode.
	
	RX_TIMEOUT_Stop();
	TIMEOUT = 0;
	RX_TIMEOUT_Start();
	
	while((TIMEOUT < RX_TIMEOUT_DURATION) && (!success))
	{
		if(RECEIVE_cReadChar() == START_TRANSMIT)
		{	
			if(validTransmission())
			{
				if(COMMAND_TYPE == ID_ASSIGN_OK)	// This is the response we are looking for.
				{
					// If this is for me, check who it was from.
					if(COMMAND_DESTINATION == MASTER_ID)
					{
						if(COMMAND_SOURCE == assigned_ID)
						{
							success = 1;
						}
					}
				}
			}
		}
	}
	
	LCD_2_Start();
	LCD_2_Position(0,0);
	LCD_2_PrHexInt(NUM_MODULES);
	LCD_2_Position(0,5);
	LCD_2_PrCString("Modules!");
	
	RX_TIMEOUT_Stop();
	TIMEOUT = 0;
	RX_TIMEOUT_Start();
	
	return success;
}

int clearConfig(int module_id)
{
	int response = 0;
	
	configToggle(TX_MODE);	// Toggle into TX mode.
			
	// Transmit a hello.
	TRANSMIT_PutChar(START_TRANSMIT);
	TRANSMIT_PutChar(START_TRANSMIT);
	TRANSMIT_PutChar(MASTER_ID);
	TRANSMIT_PutChar(module_id);
	TRANSMIT_PutChar(CLEAR_CONFIG);
	TRANSMIT_PutChar(END_TRANSMIT);
	TRANSMIT_PutChar(END_TRANSMIT);
	
	// Wait for the transmission to finish.
	while(!( TRANSMIT_bReadTxStatus() & TRANSMIT_TX_COMPLETE));
	
	xmitWait();
	
	configToggle(RX_MODE);	// Listen for the response.
	
	if(module_id != BROADCAST)
	{
		RX_TIMEOUT_Stop();
		TIMEOUT = 0;
		RX_TIMEOUT_Start();
		
		while((TIMEOUT < RX_TIMEOUT_DURATION) && (!response))
		{
			if(RECEIVE_cReadChar() == START_TRANSMIT)
			{	
				if(validTransmission())
				{
					if(COMMAND_TYPE == CONFIG_CLEARED)	// This is the response we are looking for.
					{
						// If this is for me, check who it was from.
						if(COMMAND_DESTINATION == MASTER_ID)
						{
							if(COMMAND_SOURCE == module_id)
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
	configToggle(TX_MODE);				// Toggle into TX mode.
			
	// Transmit a hello.
	TRANSMIT_PutChar(START_TRANSMIT);
	TRANSMIT_PutChar(START_TRANSMIT);
	TRANSMIT_PutChar(MASTER_ID);
	TRANSMIT_PutChar(BLANK_MODULE_ID);
	TRANSMIT_PutChar(HELLO_BYTE);
	TRANSMIT_PutChar(END_TRANSMIT);
	TRANSMIT_PutChar(END_TRANSMIT);
	
	// Wait for the transmission to finish.
	while(!( TRANSMIT_bReadTxStatus() & TRANSMIT_TX_COMPLETE));
	
	xmitWait();
	
	configToggle(RX_MODE);				// Listen for the response.
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
	char* command;
	
	if(command = UART_1_szGetParam())
	{
//		LCD_1_Start();
//		LCD_1_Position(0,0);
//		LCD_1_PrCString("                ");
//		LCD_1_Position(0,0);
//		LCD_1_PrString(command);
		
		if((command[0] == 'm') || (command[0] == 'M'))
		{
			moveMotor(atoi(&command[1]));
		}
	}
	
	UART_1_CmdReset();
}

// This function allows the program to pass an RX or TX mode flag for switching between modes on the
// half duplex UART serial communication line.
void configToggle(int mode)
{
	// Disconnect from the global bus and leave the pin high.
	PRT0DR |= 0b10000000;
	PRT0GS &= 0b01111111;

	// Unload the configuration of the current state.
	// If there is no state, blindly wipe all configurations.
	if(STATE)
	{
		unloadConfig(STATE);
	}
	else
	{
		unloadAllConfigs();
	}
	
	if(mode == PC_MODE)
	{
		LoadConfig_pc_listener();
		
		UART_1_CmdReset(); 						// Initializes the RX buffer
		UART_1_IntCntl(UART_1_ENABLE_RX_INT);   // Enable RX interrupts  
		UART_1_Start(UART_PARITY_NONE);			// Starts the UART.
		
		TIMEOUT = 0;
		STATE = PC_MODE;
	}
	else if(mode == RX_MODE)
	{
		LoadConfig_receiver_config();
		
		// Clear the buffer.
		RECEIVE_CmdReset();
		// Start the receiver.
		RECEIVE_Start(RECEIVE_PARITY_NONE);
		
		// Start response timeout timer and enable its interrupt routine.
		TIMEOUT = 0;
		RX_TIMEOUT_EnableInt();
		RX_TIMEOUT_Start();
		
		STATE = RX_MODE;
	}
	else if(mode == TX_MODE)
	{
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
		
		STATE = TX_MODE;
	}
	
	// Make sure to keep the LED on (active low).
	//PRT2DR &= 0b11111110;
	
	if(STATE == TX_MODE)
	{
		PRT1DR |= 0b00000001;
	}
	else
	{
		PRT1DR &= 0b11111110;
		
	}
	
	// Reconnect to the global bus.
	PRT0GS |= 0b10000000;
}

// This function blindly unloads all user configurations. This will be called once,
// when the system initially has no known state.
void unloadAllConfigs(void)
{
	UnloadConfig_pc_listener();
	UnloadConfig_receiver_config();
	UnloadConfig_transmitter_config();
}

// This function unloads the configuration corresponding to the config number passed to it.
// We do this instead of unloadAllConfigs to cut down on set up time.
void unloadConfig(int config_num)
{
	if(config_num == PC_MODE)
	{
		UnloadConfig_pc_listener();
	}
	else if(config_num == RX_MODE)
	{
		UnloadConfig_receiver_config();
	}
	else if(config_num == TX_MODE)
	{
		UnloadConfig_transmitter_config();
	}
}

void moveMotor(int motor_id)
{
	char checksum;
	char length = 7;
	char instruction = 3;
	char address = 30;
	char value = 6;
	char motor = motor_id;
	
	// Calculate the checksum value for our servo communication.
	checksum = 255-((motor + length + instruction + address + value)%256);
	
	LCD_1_Start();
	LCD_1_Position(0,0);
	LCD_1_PrHexByte(motor);
	
	// Toggle into transmit mode.
	configToggle(TX_MODE);
	
	// Disconnect your children from the global bus, just in case.
	PRT0GS &= 0b11100001;
	
	TRANSMIT_PutChar(255);			// Start byte one
	TRANSMIT_PutChar(255);			// Start byte two
	TRANSMIT_PutChar(motor);		// Servo ID
	TRANSMIT_PutChar(length);		// The instruction length.
	TRANSMIT_PutChar(instruction);	// The instruction to carry out.
	TRANSMIT_PutChar(address);		// The address to read/write from/to.
	TRANSMIT_PutChar(0);			// LSB of goal position
	TRANSMIT_PutChar(3);			// MSB of goal position
	TRANSMIT_PutChar(0);			// LSB of goal speed
	TRANSMIT_PutChar(3);			// MSB of goal speed
	TRANSMIT_PutChar(checksum);		// This is the checksum.
	
	// Wait for the transmission to finish.
	while(!( TRANSMIT_bReadTxStatus() & TRANSMIT_TX_COMPLETE));
	
	xmitWait();
	
	configToggle(PC_MODE);
}

void initializeSlaves(void)
{
	int num_timeouts = 0;
	
	sayHello();
	
	// This loop continuously probes and listens at intervals
	// set by the RX_TIMEOUT_DURATION variable.
	while(num_timeouts < MAX_TIMEOUTS)
	{					
		if(RECEIVE_cReadChar() == START_TRANSMIT)
		{	
			if(validTransmission())
			{
				if(COMMAND_TYPE == HELLO_BYTE)	// Someone else is out there!
				{
					// If this is for me, assign them an ID.
					if(COMMAND_DESTINATION == MASTER_ID)
					{
						NUM_MODULES++;			// Increment the number of modules connected.
						num_timeouts = 0;		// Reset number of timeouts since we found someone.
			
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
					}
				}
			}
		}
		else if(TIMEOUT >= HELLO_RX_TIMEOUT)
		{	
			num_timeouts++;
			
			// If we are not maxed out on modules, look for more.
			if(NUM_MODULES < MAX_MODULES)
			{
				sayHello();
			}
		}
	}
	
	// Switch back to PC mode.
	configToggle(PC_MODE);
}

void xmitWait(void)
{
	int i;
	
	for(i = 0; i < 25; i++)
	{
		// Sit here and spin for about 50 microseconds.
	}
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