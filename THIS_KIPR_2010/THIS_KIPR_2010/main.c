//	Author: Jason Tennyson
//	Date Created: 5/20/2010
//	Last Revision: 6/25/2010
//	Description: This is the arm controller for the robot "This" for the 2010 KIPR Open robotics competition.
//				 Dummy index variables are declared at the top of functions instead of in line with for loops
//				 because this compiler will cut you if you don't. A lot of the code that is commented out was
//				 done to save flash space, which is almost full after this huge program.

#include <m8c.h>        // part specific constants and macros
#include "PSoCAPI.h"    // PSoC API definitions for all User Modules
#include <stdlib.h>		// Standard stuff, for example integer to character array (itoa) conversion.
#include <math.h>		// Math functions!

/*
                                          *SERVO INFO*
-------------------------------------------------------------------------------------------------
The Clock parameter and Period parameter were chosen to make the period length 20ms and
to get the most resolution out of the 16 bit PWM.

PWM clock input = 3 MHz
PWM Period parameter = 60000 (maximum even value under 65535)
Period length = 1/3 MHz * 60000 = 20ms 			(20ms period length is a servo standard)
Minimum PWM value = 3 MHz * 0.5ms = 1500		(0.5ms PW for min is a servo standard)
Center PWM value = 3 MHz * 1.5ms = 4500			(1.5ms PW for center is a servo standard)
Maximum PWM value = 3 MHz * 2.5ms = 7500		(2.5ms PW for max is a servo standard)
Servo Range = 180 degrees

The PulseWidth parameter can be set to 1500 to start the servo at 0, 4500 to start the
servo at center, or any degree value you want where:

PWM value = [(Maximum Value - Minimum Value)/(Servo Range)]*(degrees desired) + Minimum PWM value
PWM value = (6000/180)*(degrees desired) + 1500

Example
-------
PWM value = (6000/180)*60 + 1500
PWM value = 3500

This means that the servo controller can be sent a value accurate to 3 hundredths of a degree.
Therefore, to minimize strain on the PSoC, and fit below the maximum serial receive buffer size,
floating point values should be limited to 2 digits past the decimal place in length.

*/

#define NUM_SERVOS 				6		// The maximum number of servos for one controller board.
#define CHAR_NUM_SERVOS 		'6'		// The character representation of NUM_SERVOS for serial error checking.

// These values represent the max and min boundaries that every servo is limited to by this controller.
#define PWM_MIN 				1500.0	// The absolute minimum PWM value allowed for all servos.
#define PWM_MAX 				7500.0	// The absolute maximum PWM value allowed for all servos.

#define DEFAULT_RANGE			180		// The default servo range, if none specified for initialization.

// The default attack angle (degrees) for a position command, if not given.
// Set to -35 because that is the angle that the gripper points in the starting position.
// Also, at approximately -30 degrees, the gripper has a flat edge.
#define DEFAULT_ATTACK_ANGLE	-35

// Arm servo indices...
#define BASE					0
#define SHOULDER				1
#define ELBOW					4
#define WRIST_FLEX				3
#define WRIST_TWIST				5
#define CLAW					2

// The min and max distance values for where the gripper is allowed to be sent to (in inches).
// They are the measured distance from the center of the bot to the center of the gripper.
#define MIN_GRIPPER_DISTANCE		9		// Distance when the gripper is touching the front of the bot.
#define MAX_GRIPPER_DISTANCE		22.625	// d3 + link 1 + link 2 + LG
#define MIN_GRIPPER_HEIGHT			1.9		// Distance from the ground to the middle of the gripper.
#define MIN_GRIPPER_HEIGHT_TOP		8		// Minimum height of the gripper when hovering over the robot.
#define MIN_WRIST_DISTANCE_BOTTOM	7.4375	// Minimum wrist distance outward if it is directly above the bottom lexan.
#define MIN_WRIST_HEIGHT_BOTTOM		4.5		// Minimum height of the wrist if it is directly above the bottom lexan.
#define MIN_WRIST_DISTANCE_TOP		4.9375	// Minimum wrist distance outward if it is directly above the top lexan.
#define MIN_WRIST_HEIGHT_TOP		7		// Minimum height of the wrist if it is above the top lexan.
#define ADC_RESOLUTION				14		// Number of bits in the ADC, as defined in the device editor.

// The floating point number to multiply radian values by to convert to degrees.
#define RADIANS_TO_DEGREES		57.295779513082320876798154814105
// The floating point number to multiply degree values by to convert to radians.
#define DEGREES_TO_RADIANS		0.01745329251994329576923690768489

// Data is 8 bytes long per servo:
// 1 byte for servo range, 1 byte for if it is inverted, 2 bytes for minimum PWM value,
// 2 bytes for maximum PWM value, 2 bytes for starting PWM value.
#define BYTES_PER_SERVO			8
// Data length calculation for reading/writing to the EEPROM
#define DATA_LENGTH 			48		// BYTES_PER_SERVO*NUM_SERVOS
// The PSoC writes character '0' to empty flash space.  Therefore, we want to write a character
// '0' back to memory when clearing servo data (which has an integer value of 48).
#define CLEAN_BYTE				48

// These are the measurements in inches of the arm used for This.
#define LG						10.9375	// Length from the middle of the gripper to the end of link 2.
#define L3						9.3125	// Distance from the ground to the middle of the shoulder servo.
#define L5						6.0		// Length of link 1.
#define L6						4.75	// Length of link 2.
#define d3						0.9375	// The displacement of the shoulder rotation axis from the center of the bot.
#define DR						0.75	// The displacement of the middle of the grip to the end of link 2.

// These are the maximum degree ranges for all of the servos. If a smaller range is desired, alter this to
// be lower for the servo of your choice where these values are from index 0 to 5 in order.
int		servo_range[NUM_SERVOS];		// The total degree range of a servo.
int		servo_enabled[NUM_SERVOS];		// A servo is only enabled if calibrated.
int		servo_inverted[NUM_SERVOS];		// Used to correct input degree values if min_pwm > max_pwm.
float 	min_degrees[NUM_SERVOS];		// The minimum degree value of a servo after calibration/initialization.
float 	max_degrees[NUM_SERVOS];		// The maximum degree value of a servo after calibration/initialization.
int 	min_pwm[NUM_SERVOS];			// The minimum pwm value of a servo after calibration/initialization.
int 	max_pwm[NUM_SERVOS];			// The maximum pwm value of a servo after calibration/initialization.
float 	servo_degrees[NUM_SERVOS];		// Current servo degree value.
int 	servo_pwm[NUM_SERVOS];			// Current servo pwm value.

// These values are used to store what the current position of the arm *should* be, so that calculations
// relative to these numbers can be made on incoming data.
float	attack_angle;					// Stores the attack angle of the gripper.
float	x_position;						// The current x position.
float	y_position;						// The current y position.
float	z_position;						// The current z position.

// Function to initialize serial communication.
void InitSerial(void);

// Function to write information to the computer.
void WriteSerial(char* output);

// Function to read and interpret a command from the computer.
void ReadSerial(void);

// Function to move all servos at once using their recently updated pwm values.
void MoveAllServos(void);

// Function to move an individual servo using its recently updated pwm value.
int MoveIndividualServo(int index);

// Function to calculate the angles necessary to move to the given position
// at the given attack angle (inverse kinematic calculation).
int InverseKinematicCalculator(float x, float y, float z, float phi);

// Function that does a check on whether the servos are physically capable of reaching
// the desired position, among other things (in the future?).
int InvKinErrorCheck(void);

// Function that inverts degree measurements if they are in need of it.
void InvertDegreesCheck(int index);

// Function that does a range check for one degree value.
int DegreeRangeCheck(int index);

// Function that does a range check for all degree values.
int DegreeRangeCheckAll(void);

// Function that does a range check for one PWM value.
int PWMRangeCheck(int index);

// Function that does a range check for all PWM values.
int PWMRangeCheckAll(void);

// Function to convert all of the new degree values to PWM values.
// This function is called after a kinematic calculation of degree values.
void UpdateAllPWMs(void);

// Function to initialize servos to their stored start values and power them on correctly.
void InitServos(void);

// Function to calibrate one servo over a user-specified degree range.
int CalibrateServo(int servo_index, int range);

// Function that saves all of the new calibration values.  Should be done after ALL calibrations
// are done to avoid writing to the EEPROM more times than is necessary.
int SaveCalibration(void);

// Function to initialize min and max degree values after a servo initialization or calibration.
void InitDegrees(int index);

// Function to write the servo enable status to the user.
void ReturnServoStatus(int index);

// Function to write the servo range to the user.
void ReturnServoRange(int index);

// Function to return light percentage from the light sensor to the computer.
void ReturnLightPercentage(void);

// None of the following memory functions will take effect until a controller reset is performed.
// ----------------------------------------------------------------------------------------------

// Function to flip saved settings between two servos.
int	FlipServoSettings(int index1, int index2);

// Function to move saved settings from one servo index to another.
int	MoveServoSettings(int old_index, int new_index);

// Function that clears all servo settings from the EEPROM.
int DeleteAllServoSettings(void);

// Function that clears the servo settings for one servo.
int DeleteIndividualServoSetting(int index);

void main()
{
	SERVO_INIT_PGA_Start(SERVO_INIT_PGA_HIGHPOWER);		// Start PGA that feeds into the ADC.
	SERVO_INIT_ADC_Start(SERVO_INIT_ADC_HIGHPOWER);		// Start ADC for servo initialization potentiometer.
	SERVO_INIT_ADC_GetSamples(0);						// Take continuous samples.
	
	LCD_1_Start();		// Used for proto board debugging purposes...
	InitSerial();		// Initialize the serial communication.
	InitServos();		// Initialize the servos.

	while(1)
	{	
		ReadSerial();	// Read serial commands until the end of days...
	}
}

// This function initializes the serial communication by resetting the receive buffer
// and enabling interrupts.
void InitSerial()
{
	SERIAL_CmdReset(); 						// This initializes the RX (receive) serial buffer.
	SERIAL_IntCntl(SERIAL_ENABLE_RX_INT);   // This enables RX (receive) interrupts.
    SERIAL_Start(UART_PARITY_NONE);       	// This enables serial communication.
	M8C_EnableGInt;                         // This enables interrupts (RX is the only one enabled).
}

// This function takes a string of characters (char*) and writes
// information to the computer using serial communication.
void WriteSerial(char* output)
{
	SERIAL_PutString(output);
}

// This function reads and interprets serial commands from the computer. It is basically the brain
// of this servo controller. Based on a user command input, this function calls smaller functions
// to perform small tasks for either moving servos or returning data to the user.
void ReadSerial()
{
	char* command_type;			// The command type (the first parameter of all commands).
	char* all_or_individual;	// The command for what the user's target is (one or all servos).
	char* data_format;			// The data format the user wants to use or read.
	char* value;				// A PWM or degree value sent by the user.
	char* range;				// The total range that a new servo will cover (90 or 180 usually).
	char* source;				// The index of the servo whose information is getting copied/moved/flipped.
	char* destination;			// The destination index of the servo information getting copied/moved/flipped.
	char* x;					// The x coordinate of the user's entered position.
	char* y;					// The y coordinate of the user's entered position.
	char* z;					// The z coordinate of the user's entered position.
	char* phi;					// The attack angle of the user's entered position (optional).
	char* servo_pwm_string;		// Just used to appease the itoa function's need of a char* param, currently...
	int index;					// Stores the converted integer servo index value from the user input string.
	int status;					// Status of a float to char array conversion (used to appease the function call).
	int i = 0;					// Generic index variable for loops.
	int j = 0;					// Generic index variable for loops within loops already using i.
	
	if(SERIAL_bCmdCheck()) // Checking if a complete command has arrived.
	{	
		command_type = SERIAL_szGetParam();				// Reads the SET/READ parameter.
		
		if ((command_type[0] == 'L') || (command_type[0] == 'l'))
		{
			ReturnLightPercentage();
		}
		else if ((command_type[0] == 'I') || (command_type[0] == 'i'))		// User is initializing a servo.
		{
			all_or_individual = SERIAL_szGetParam();	// Reads the ALL/INDIVIDUAL parameter.
			
			if ((all_or_individual[0] >= '0') && (all_or_individual[0] <= CHAR_NUM_SERVOS))
			{
				if (range = SERIAL_szGetParam())		// The range that the new servo will cover (ex: 180).
				{
					if (CalibrateServo(atoi(all_or_individual),atoi(range)) == 0)
					{
						SaveCalibration();
					}
				}
				else
				{
					// If no range given, set to default.
					if (CalibrateServo(atoi(all_or_individual),DEFAULT_RANGE) == 0)
					{
						SaveCalibration();
					}
				}
			}
//			else if ((all_or_individual[0] == 'A') || (all_or_individual[0] == 'a'))
//			{
//				i = 0;	// Set the index to start at servo 0.
//					
//				// While we get range values, and are within the number of servos, calibrate all servos.
//				while (i < NUM_SERVOS)
//				{
//					if (range = SERIAL_szGetParam())
//					{
//						CalibrateServo(i,atoi(range));	// Call for a servo calibration over this range for this index.
//					}
//					else
//					{
//						CalibrateServo(i,DEFAULT_RANGE);// If no range specified, set the range to the default value.
//					}
//					i++;							// Increment the index for the next iteration.
//				}
//				SaveCalibration();
//			}
		}
		else if ((command_type[0] == 'P') || (command_type[0] == 'p'))	// User is setting an xyz position.
		{
			// Read the x coordinate.
			if (x = SERIAL_szGetParam())
			{
				// Read the y coordinate.
				if (y = SERIAL_szGetParam())
				{
					// Read the z coordinate.
					if (z = SERIAL_szGetParam())
					{
						// Read the attack angle, if there is one, and call the inverse kinematic calculator.
						if (phi = SERIAL_szGetParam())
						{
							InverseKinematicCalculator(atof(x), atof(y), atof(z), atof(phi));
						}
						else
						{
							InverseKinematicCalculator(atof(x), atof(y), atof(z), DEFAULT_ATTACK_ANGLE);
						}
					}
				}
			}
		}
//		else if ((command_type[0] == 'F') || (command_type[0] == 'f'))	// User is flipping two servos.
//		{
//			// Read first servo index to be flipped.
//			if (source = SERIAL_szGetParam())
//			{
//				// Second servo index to be flipped is read. If it returns a value, and
//				// the source is within the correct index range, then continue with this command.
//				if ((destination = SERIAL_szGetParam()) && ((source[0] >= '0') && (source[0] <= CHAR_NUM_SERVOS)))
//				{
//					// Check that the destination is also in the correct index range.
//					if ((destination[0] >= '0') && (destination[0] <= CHAR_NUM_SERVOS))
//					{
//						// Calls for a swap of two servos' settings.
//						FlipServoSettings(atoi(source), atoi(destination));
//					}
//				}
//			}
//		}
//		else if ((command_type[0] == 'M') || (command_type[0] == 'm'))	// User is moving a servo.
//		{
//			// Read the index of the source information.
//			if (source = SERIAL_szGetParam())
//			{
//				// Second servo index to be flipped is read. If it returns a value, and
//				// the source is within the correct index range, then continue with this command.
//				if ((destination = SERIAL_szGetParam()) && ((source[0] >= '0') && (source[0] <= CHAR_NUM_SERVOS)))
//				{
//					// Check that the destination is also in the correct index range.
//					if ((destination[0] >= '0') && (destination[0] <= CHAR_NUM_SERVOS))
//					{
//						// Calls for a move of two servos' settings.
//						MoveServoSettings(atoi(source), atoi(destination));
//					}
//				}
//			}	
//		}
//		else if ((command_type[0] == 'D') || (command_type[0] == 'd'))	// User is deleting saved settings.
//		{
//			all_or_individual = SERIAL_szGetParam();	// Reads the ALL/INDIVIDUAL parameter.
//			
//			if ((all_or_individual[0] == 'A') || (all_or_individual[0] == 'a'))
//			{
//				// User wants to delete all servo settings.
//				DeleteAllServoSettings();
//			}
//			else if ((all_or_individual[0] >= '0') && (all_or_individual[0] <= CHAR_NUM_SERVOS))
//			{
//				// User wants to delete one servo's settings.
//				DeleteIndividualServoSetting(atoi(all_or_individual));
//			}
//		}
		else if ((command_type[0] == 'S') || (command_type[0] == 's'))	// If the user is setting parameters...
		{
			// If the user chooses to set all values for either PWM or degrees, the 6 parameters after
			// (S)et (A)ll (P)WM/(D)egrees will be interpreted as values for servos 0 through 5 in order.
			// If the user provides less than 6 extra parameters, the servos from 0 to the last parameter
			// provided will be updated. If the user provides more than 6 parameters, one of two things
			// may occur. Either the command will not be recognized because it is larger than the receive
			// buffer, or if the command is smaller than the receive buffer, the parameters after the 6th
			// will be ignored.
			
			all_or_individual = SERIAL_szGetParam();		// Reads the ALL/INDIVIDUAL parameter.
			data_format = SERIAL_szGetParam();				// Reads the desired data format (PWM/DEGREES).
			
			if ((all_or_individual[0] == 'A') || (all_or_individual[0] == 'a'))	// If the user is setting all servos...
			{
				if ((data_format[0] == 'D') || (data_format[0] == 'd'))			// Degree input
				{
					i = 0;	// Set the index to start at servo 0.
					
					// While we get values, and are within range, read the degree values for all servos.
					while ((value = SERIAL_szGetParam()) && (i < NUM_SERVOS))
					{
						// Convert and store the degree string value that the user has provided as a float
						// in the servo_degrees array that stores all servo degree values.
						servo_degrees[i] = atof(value);
						
						// If the servo was inverted because the user put in a min_pwm that was greater
						// than the max_pwm, invert the input degree value accordingly.
						InvertDegreesCheck(i);
						
						// Convert the new degree float value to a PWM value and store it in the
						// servo_pwm array that stores all servo PWM values.
						servo_pwm[i] = (((servo_degrees[i]-min_degrees[i])/servo_range[i])*
						               (max_pwm[i]-min_pwm[i]))+min_pwm[i];

						// Increment the index for the next iteration.
						i++;
					}
				}
//				else if ((data_format[0] == 'P') || (data_format[0] == 'p'))	// PWM input
//				{
//					i = 0;	// Set the index to start at servo 0.
//					
//					// While we get values, and are within range, read the PWM values for all servos.
//					while ((value = SERIAL_szGetParam()) && (i < NUM_SERVOS))
//					{
//						// Convert and store the PWM string value that the user has provided as an integer
//						// in the servo_pwm array that stores all servo PWM values.
//						servo_pwm[i] = atoi(value);
//
//						// Convert the new PWM integer value to a degree value and store it in the
//						// servo_degrees array that stores all servo degree values.
//						servo_degrees[i] = (((float)servo_pwm[i]-min_pwm[i])/(max_pwm[i]-min_pwm[i]))*servo_range[i] +
//						                   min_degrees[i];
//
//						// If the servo was inverted because the user put in a min_pwm that was greater
//						// than the max_pwm, invert the input degree value accordingly.
//						InvertDegreesCheck(i);
//						
//						// Increment the index for the next iteration.
//						i++;
//					}
//				}
				
				
				// Make sure that the values that were just set are ok.
				PWMRangeCheckAll();
				
				// After all values have been interpreted, move the servos.
				MoveAllServos();
			}
			else if ((all_or_individual[0] >= '0') && (all_or_individual[0] <= CHAR_NUM_SERVOS))	// if the user is targeting one servo
			{
				// Extract the index value from this parameter of the user input.
				index = atoi(all_or_individual);
				
				// Grab the value that the user wants this servo to be.
				value = SERIAL_szGetParam();
				
				if ((data_format[0] == 'D') || (data_format[0] == 'd'))	// Degree input
				{
					// Convert and store the degree string value that the user has provided as a float
					// in the servo_degrees array that stores all servo degree values.
					servo_degrees[index] = atof(value);
					
					// If the servo was inverted because the user put in a min_pwm that was greater
					// than the max_pwm, invert the input degree value accordingly.
					InvertDegreesCheck(index);
					
					// Convert the new degree float value to a PWM value and store it in the
					// servo_pwm array that stores all servo PWM values.
					servo_pwm[index] = (((servo_degrees[index]-min_degrees[index])/servo_range[index])*
									   (max_pwm[index]-min_pwm[index]))+min_pwm[index];
				}
//				else if ((data_format[0] == 'P') || (data_format[0] == 'p'))			// PWM input
//				{
//					// Convert and store the PWM string value that the user has provided as an integer
//					// in the servo_pwm array that stores all servo PWM values.
//					servo_pwm[index] = atoi(value);
//					
//					// Convert the new PWM integer value to a degree value and store it in the
//					// servo_degrees array that stores all servo degree values.
//					servo_degrees[index] = (((float)servo_pwm[index]-min_pwm[index])/(max_pwm[index]-min_pwm[index]))*
//										   servo_range[index] + min_degrees[index];
//										   
//					// If the servo was inverted because the user put in a min_pwm that was greater
//					// than the max_pwm, invert the input degree value accordingly.
//					InvertDegreesCheck(index);
//				}
				
				// Make sure that the value that was just set is ok.
				PWMRangeCheck(index);
				
				// Move the individual servo and, if desired, perform a response action based on the result.
				if(MoveIndividualServo(index) >= 0)
				{
					LCD_1_Position(0,0);
					LCD_1_PrCString("Success!        ");
				}
				else
				{
					LCD_1_Position(0,0);
					LCD_1_PrCString("Failure!        ");
				}
			}
		}
		else if ((command_type[0] == 'R') || (command_type[0] == 'r'))
		{	
			if (all_or_individual = SERIAL_szGetParam())
			{
				// Make sure that what was read is within the proper index range.
				if ((all_or_individual[0] >= '0') && (all_or_individual[0] <= CHAR_NUM_SERVOS))
				{
					// Reading is only allowed for individual servos, so we can automatically
					// assume that this parameter is the index value.
					index = atoi(all_or_individual);
				}
			}
			
			data_format = SERIAL_szGetParam();				// Reads the desired data format (PWM/DEGREES).
			
			
			if ((data_format[0] == 'D') || (data_format[0] == 'd'))
			{
				// Write back the current degree value for this servo.
				if (servo_inverted[index])
				{
					// Invert the value back to what the user sent (which is what they perceive the angle to be).
					WriteSerial(ftoa((-1)*servo_degrees[index], &status));
				}
				else
				{
					// The stored value is the value as the user sees it physically, so just send it.
					WriteSerial(ftoa(servo_degrees[index], &status));
				}
			}
//			else if ((data_format[0] == 'P') || (data_format[0] == 'p'))
//			{
//				// Write back the current PWM value for this servo.
//				WriteSerial(itoa(servo_pwm_string, servo_pwm[index], 10));
//			}
		}
		SERIAL_CmdReset(); // Emptying the RX buffer...
	}
}

// This function moves every servo based on the pwm values being stored for all of the
// servos (given that they are enabled). This function will be called after either a
// manual user entry for moving all servos, or after a kinematic calculation.
void MoveAllServos()
{
	int i;

	// Call to move every servo in a loop.
	for(i=0; i < NUM_SERVOS; i++)
	{
		MoveIndividualServo(i);
	}
}

// This function accepts a servo index and uses a case statement to write the
// pwm that is currently being stored for that servo. This function will be called
// directly after a manual user entry for the servo's pwm.
int MoveIndividualServo(int servo_index)
{
	// Checks to see if the servo index is in the current range.
	// The else on this statement could eventually be used to send an extra servo
	// controller a command, allowing you to "daisy chain" them.
	if ((servo_index >= 0) && (servo_index < NUM_SERVOS) && (servo_enabled[servo_index]))
	{
		// Based on the servo index that was passed, move that servo...
		if (servo_index == 0)
		{
			S0_WritePulseWidth(servo_pwm[0]);	// Servo 0 (S0)
		}
		else if (servo_index == 1)
		{
			S1_WritePulseWidth(servo_pwm[1]);	// Servo 1 (S1)
		}
		else if (servo_index == 2)
		{
			S2_WritePulseWidth(servo_pwm[2]);	// Servo 2 (S2)
		}
		else if (servo_index == 3)
		{
			S3_WritePulseWidth(servo_pwm[3]);	// Servo 3 (S3)
		}
		else if (servo_index == 4)
		{
			S4_WritePulseWidth(servo_pwm[4]);	// Servo 4 (S4)
		}
		else if (servo_index == 5)
		{
			S5_WritePulseWidth(servo_pwm[5]);	// Servo 5 (S5)
		}
	}
	else
	{
		return -1;		// Returns -1 if the servo is out of range.
	}
	
	return servo_index;	// If all went well, return the index value of the servo moved.
}

// If the xyz position is illegal, return an error code.
// An illegal value is one that is either outside of the range of the arm or
// inside the range that is taken up by the body of the robot.
int InverseKinematicCalculator(float x, float y, float z, float phi)
{
	float A;		// Horizontal position of the middle of the gripper, after calculation.
	float B;		// Vertical position of the middle of the gripper, after calculation.
	float distance;	// The horizontal distance from the center of the robot to the target.
	float beta;		// Used for inverse kinematic calculation of arm angles.
	float psi;		// Also used for inverse kinematic calculation of arm angles.
	int i;			// Used as an index variable for looping.
	
	int status;	// Just used to appease ftoa conversion parameter...
	
	// Check that the coordinate given is within the work space...
	distance = sqrt(x*x + y*y);
	if (((distance < MIN_GRIPPER_DISTANCE) && (z < MIN_GRIPPER_HEIGHT_TOP)) ||
	     (distance > MAX_GRIPPER_DISTANCE) || 
		 (z < MIN_GRIPPER_HEIGHT))
	{
		return -1;	// Can't perform this movement.
	}
	
	// Have to convert from degrees to radians to use the radian cos and sin functions.
	phi = phi*DEGREES_TO_RADIANS;
	
	// Perform the inverse kinematic calculations...
	
	// Make sure to keep the sign for the distance
	if (y < 0)
	{
		A = ((-1)*distance-d3-LG*cos(phi))+DR*sin(phi);
	}
	else
	{
		A = (distance-d3-LG*cos(phi))+DR*sin(phi);
	}
	
	B = z-LG*sin(phi)-L3-DR*cos(phi);

	// If the wrist is in the wrong spot, try again with the default attack angle.
	if (((A+d3) < MIN_WRIST_DISTANCE_BOTTOM) && ((B+L3) < MIN_WRIST_HEIGHT_BOTTOM) ||
		((A+d3) < MIN_WRIST_DISTANCE_TOP) && ((B+L3) < MIN_WRIST_HEIGHT_TOP))
	{
		// Make sure to keep the sign for the distance
		if (y < 0)
		{
			A = ((-1)*distance-d3)-LG*cos(DEFAULT_ATTACK_ANGLE*DEGREES_TO_RADIANS)+
		        DR*sin(DEFAULT_ATTACK_ANGLE*DEGREES_TO_RADIANS);
		}
		else
		{
			A = (distance-d3)-LG*cos(DEFAULT_ATTACK_ANGLE*DEGREES_TO_RADIANS)+
		        DR*sin(DEFAULT_ATTACK_ANGLE*DEGREES_TO_RADIANS);
		}

		B = z-LG*sin(DEFAULT_ATTACK_ANGLE*DEGREES_TO_RADIANS)-L3-DR*cos(DEFAULT_ATTACK_ANGLE*DEGREES_TO_RADIANS);
		
		if (((A+d3) < MIN_WRIST_DISTANCE_BOTTOM) && ((B+L3) < MIN_WRIST_HEIGHT_BOTTOM) ||
			((A+d3) < MIN_WRIST_DISTANCE_TOP) && ((B+L3) < MIN_WRIST_HEIGHT_TOP))
		{
			return -1;	// Couldn't do it.
		}
	}
	
	beta = RADIANS_TO_DEGREES*atan(B/A);
	
	// Check if beta is intended for quadrant II instead of quadrant I.
	if (beta < 0)
	{
		beta += 180.0;
	}
	
	psi = RADIANS_TO_DEGREES*acos((A*A + B*B + L5*L5 - L6*L6)/(2*L5*sqrt(A*A + B*B)));
	
	// Base servo calculation (base rotation)...
	servo_degrees[BASE] = (-1)*(RADIANS_TO_DEGREES)*atan(x/y);
	
	// Shoulder servo calculation (big servo).
	servo_degrees[SHOULDER] = 180-(beta + psi);
	
	// Make sure that the shoulder tries to go in a positive direction.
	if ((servo_degrees[SHOULDER] < 0) && (y > MIN_GRIPPER_DISTANCE))
	{
		servo_degrees[SHOULDER] += 180;
	}
	
	// Elbow servo calculation.
	servo_degrees[ELBOW] = 150-RADIANS_TO_DEGREES*acos((A*A + B*B - L5*L5 - L6*L6)/(2*L5*L6));
	
	// Wrist flex calculation.
	servo_degrees[WRIST_FLEX] = (phi*RADIANS_TO_DEGREES) + 35 + servo_degrees[SHOULDER] - servo_degrees[ELBOW];
	
	// Check to see if these 4 servos that were just set are meant to be inverted.
	InvertDegreesCheck(BASE);
	InvertDegreesCheck(SHOULDER);
	InvertDegreesCheck(ELBOW);
	InvertDegreesCheck(WRIST_FLEX);
	
	// Clear the screen.
	LCD_1_Position(0,0);
	LCD_1_PrCString("                ");
	LCD_1_Position(1,0);
	LCD_1_PrCString("                ");
	
	// Checks all scenarios that would keep the arm from moving.
	// This currently only includes a check of the degree ranges.
	if (InvKinErrorCheck() < 0)
	{
		return -1;
	}
	
	MoveAllServos();
	
	// Store these values in case other functions need them later.
	attack_angle = phi;
	x_position = x;
	y_position = y;
	z_position = z;
	
	return 0;
}

// This function checks if this index wants its degrees inverted (and inverts them if so).
void InvertDegreesCheck(int index)
{
	if (servo_inverted[index])
	{
		servo_degrees[index] = servo_degrees[index]*(-1);
	}
}

// This function performs a range check for all servos.
int InvKinErrorCheck()
{
	if(DegreeRangeCheckAll() < 0)
	{
		return -1;
	}
	
	return 0;
}

// Function that does a range check for one degree value.  The UpdateAllPWMs function is used because I am
// lazy and am wasting CPU resources by not creating an UpdatePWM function, but maybe I will make it later.
int DegreeRangeCheck(int index)
{
	if((servo_degrees[index] < min_degrees[index]) || (servo_degrees[index] > max_degrees[index]))
	{	
		return -1;
	}
	
	return 0;
}

// Function that does a range check for all degree values.
int DegreeRangeCheckAll(void)
{
	int i;
	char* dummy_string;		// String to appease the itoa function call.
	int status;				// Status of the ftoa conversion.
	
	for(i=0; i < NUM_SERVOS; i++)
	{
		if(DegreeRangeCheck(i) < 0)
		{	
			// Set the degree values back to what they should be.
			for(i=0; i < NUM_SERVOS; i++)
			{
				servo_degrees[i] = (((float)servo_pwm[i]-min_pwm[i])/(max_pwm[i]-min_pwm[i]))*
								   servo_range[i] + min_degrees[i];
										   
				// Invert the value if necessary.
				InvertDegreesCheck(i);
			}
			
			return -1;
		}
	}
	
	// Update all PWM values to match their degree values.
	UpdateAllPWMs();
	
	return 0;
}

int PWMRangeCheck(int index)
{
	if(servo_pwm[index] < min_pwm[index])
	{
		// If the PWM value of the input will send us below the
		// specified PWM minimum for this servo, cap it at that minimum value.
		servo_pwm[index] = min_pwm[index];
		servo_degrees[index] = min_degrees[index];
	}
	else if(servo_pwm[index] > max_pwm[index])
	{
		// If the PWM value of the input will send us past the
		// specified PWM maximum for this servo, cap it at that maximum value.
		servo_pwm[index] = max_pwm[index];
		servo_degrees[index] = max_degrees[index];
	}
	
	return 0;
}

// Function that does a range check for all PWM values.
int PWMRangeCheckAll(void)
{
	int i;
	
	for(i=0; i < NUM_SERVOS; i++)
	{
		PWMRangeCheck(i);
	}
	
	return 0;
}

// This function reads servo information from the EEPROM block of the PSoC. If
// there is no valid data, it does nothing. If there is valid
// data for any of the servos, it will initialize those servos to that and ignore the
// unused servos.
void InitServos(void)
{
	// A dummy index variables for loops.
	int i = 0;
	int j = 0;
	
	BYTE data[DATA_LENGTH];				// Stores all EEPROM data from the read.
	BYTE temp_bytes[BYTES_PER_SERVO];	// Temporarily stores data for one servo.
	
	// Does a read of the EEPROM here.  If nothing is already calibrated, it will force an initialization.
	SETTINGS_EEPROM_E2Read( 0, data, DATA_LENGTH );
	
	for(i=0; i < NUM_SERVOS; i++)
	{
		for(j=0; j < BYTES_PER_SERVO; j++)
		{
			temp_bytes[j] = data[j + i*BYTES_PER_SERVO];
		}
		
		// Read the servo range that was extracted.
		servo_range[i] = temp_bytes[0];
		
		// Read whether or not the servo is to be inverted (z axis pointed in opposite direction).
		servo_inverted[i] = temp_bytes[1];
		
		// Bit shift and convert the bytes for min_pwm into an integer.
		min_pwm[i] = temp_bytes[2]*256 + temp_bytes[3];
		
		// Bit shift and convert the bytes for max_pwm into an integer.
		max_pwm[i] = temp_bytes[4]*256 + temp_bytes[5];
		
		// Bit shift and convert the bytes for servo_pwm into an integer.
		servo_pwm[i] = temp_bytes[6]*256 + temp_bytes[7];

		// If there is a servo range, and not just an empty byte, read the rest of the valid data.
		if(servo_range[i] != CLEAN_BYTE)
		{
			// Initialize the degree values of the servo based on the given PWM values for this servo.
			InitDegrees(i);
	
			servo_enabled[i] = 1;				// Make it publicly known that this servo is enabled.
			MoveIndividualServo(i);				// This will change the pulse width for when the servo starts.
			
			// Start the servo that has been calibrated.
			if (i == 0)
			{
				S0_DisableInt();	// Servo 0 (S0)
				S0_Start();
			}
			else if (i == 1)
			{
				S1_DisableInt();	// Servo 1 (S1)
				S1_Start();
			}
			else if (i == 2)
			{
				S2_DisableInt();	// Servo 2 (S2)
				S2_Start();
			}
			else if (i == 3)
			{
				S3_DisableInt();	// Servo 3 (S3)
				S3_Start();
			}
			else if (i == 4)
			{
				S4_DisableInt();	// Servo 4 (S4)
				S4_Start();
			}
			else if (i == 5)
			{
				S5_DisableInt();	// Servo 5 (S5)
				S5_Start();
			}
		}
		else
		{	
			// This servo did not have a valid initial value, so zero out all of its parameters.
			servo_range[i] = CLEAN_BYTE;	// Make sure that the servo range is what the PSoC would read as empty.
			servo_enabled[i] = 0;
			servo_inverted[i] = 0;
			min_degrees[i] = max_degrees[i] = servo_degrees[i] = 0.0;
			min_pwm[i] = max_pwm[i] = servo_pwm[i] = PWM_MIN;
		}
	}
}

// Servo calibration function...
int CalibrateServo(int servo_index, int range)
{
	unsigned int sample;			// Sample read from the potentiometer hooked to the ADC.
	int pwm_out;					// Result of the conversion of the ADC input to a PWM value.
	int parameter_number = 0;		// Keeps track of which parameter the PSoC is waiting for from the user.
	int cancelled = 0;				// Used so that the servo is not turned on if the initialization was cancelled.
	char* calibration_command;		// Command from serial for what the user wants to do next in calibration.
	
	// Range in degrees that the servo is being calibrated over, so that commands in degrees are possible later.
	servo_range[servo_index] = range;
	
	// Start the servo that is being calibrated.
	if (servo_index == 0)
	{
		S0_DisableInt();	// Servo 0 (S0)
		S0_Start();
	}
	else if (servo_index == 1)
	{
		S1_DisableInt();	// Servo 1 (S1)
		S1_Start();
	}
	else if (servo_index == 2)
	{
		S2_DisableInt();	// Servo 2 (S2)
		S2_Start();
	}
	else if (servo_index == 3)
	{
		S3_DisableInt();	// Servo 3 (S3)
		S3_Start();
	}
	else if (servo_index == 4)
	{
		S4_DisableInt();	// Servo 4 (S4)
		S4_Start();
	}
	else if (servo_index == 5)
	{
		S5_DisableInt();	// Servo 5 (S5)
		S5_Start();
	}
	
	// The following bracketed code was to appease the compiler, which is stupid.
	{
	char* dummy_string;
	
	LCD_1_Position(0,0);
	LCD_1_PrCString("                ");
	LCD_1_Position(0,0);
	LCD_1_PrCString("Calibrating");
	LCD_1_Position(0,12);
	LCD_1_PrString(itoa(dummy_string,servo_index,10));
	LCD_1_Position(0,13);
	LCD_1_PrCString("!");
	}
	
	SERIAL_CmdReset();	// Clear whatever is in the RX buffer before starting initialization.
	
	while(parameter_number < 3)
	{
		// if there is data for the servo initializer to read, read it
		if(SERVO_INIT_ADC_fIsDataAvailable() != 0)
		{
			// grab a sample from the ADC and clear the data ready flag
			sample = SERVO_INIT_ADC_wClearFlagGetData();
	
			// maximum PWM range times fraction of the sample versus the 2^ADC_resolution
			pwm_out = (PWM_MAX-PWM_MIN)*(sample/pow(2,ADC_RESOLUTION)) + PWM_MIN;
		
			{
			char* dummy_string;
			
			LCD_1_Position(1,0);
			LCD_1_PrHexInt(pwm_out);
			//LCD_1_PrString(itoa(dummy_string,pwm_out,10));
			}
		
			// Move the servo to the new value calculated from the ADC input.
			if (servo_index == 0)
			{
				S0_WritePulseWidth(pwm_out);	// Servo 0 (S0)
			}
			else if (servo_index == 1)
			{
				S1_WritePulseWidth(pwm_out);	// Servo 1 (S1)
			}
			else if (servo_index == 2)
			{
				S2_WritePulseWidth(pwm_out);	// Servo 2 (S2)
			}
			else if (servo_index == 3)
			{
				S3_WritePulseWidth(pwm_out);	// Servo 3 (S3)
			}
			else if (servo_index == 4)
			{
				S4_WritePulseWidth(pwm_out);	// Servo 4 (S4)
			}
			else if (servo_index == 5)
			{
				S5_WritePulseWidth(pwm_out);	// Servo 5 (S5)
			}
		}
		
		if(SERIAL_bCmdCheck()) // Checking if a complete command has arrived.
		{	
			calibration_command = SERIAL_szGetParam();				// Reads the calibration command.
			
			// The user can select to either be (o)k with the current PWM value for one of the
			// three expected parameters, cancel (x) the current parameter setting so that it is
			// set to either a garbage value in the case of min/max pwm, or to a default starting
			// position of min_pwm in the case of setting the starting position. When setting the
			// starting position, the user can enter (o)k for the current pwm value to be the starting
			// position, (x) for min_pwm to be the starting position, (m)ax for max_pwm to be the
			// starting position, or (c)enter for the middle of the max and min pwm inputs to be
			// the starting position (useful for servos that have +/- 90 degrees as max and min values).
			
			if ((calibration_command[0] == 'O') || (calibration_command[0] == 'o'))			// Ok
			{
				// Set the parameter being calibrated currently to be equal to the user's desired pwm value.
				if (parameter_number == 0)
				{
					min_pwm[servo_index] = pwm_out;
				}
				else if (parameter_number == 1)
				{
					max_pwm[servo_index] = pwm_out;
				}
				else if (parameter_number == 2)
				{
					servo_pwm[servo_index] = pwm_out;
				}
				
				// Increment the parameter number to allow calibration of the next parameter or the
				// exiting of this servo's calibration of all parameters' settings are done being read.
				parameter_number++;
			}
			else if ((calibration_command[0] == 'X') || (calibration_command[0] == 'x'))	// Cancel
			{
				// The user does not want to use the current pwm_out,
				// which will have varying results (see top comment).
				if (parameter_number < 2)
				{
					// Set the parameter number so that the loop is exited and it is noted
					// that the user cancelled the initialization.
					parameter_number = 2;
					cancelled = 1;
				}
				else if (parameter_number == 2)
				{
					servo_pwm[servo_index] = min_pwm[servo_index];
				}
				
				// Increment the parameter number to allow calibration of the next parameter or the
				// exiting of this servo's calibration of all parameters' settings are done being read.
				parameter_number++;
			}
			else if ((calibration_command[0] == 'C') || (calibration_command[0] == 'c'))	// Set start pos to center.
			{
				// This commands only pertains to parameter number 2 (starting position).
				if(parameter_number == 2)
				{
					// Set the starting servo pwm to the center value (averate of max and min).
					servo_pwm[servo_index] = (min_pwm[servo_index] + max_pwm[servo_index])/2;
					
					// Increment the parameter number so that the PSoC ends the calibration.
					parameter_number++;
				}
			}
			else if ((calibration_command[0] == 'M') || (calibration_command[0] == 'm'))	// Set start pos to max.
			{
				// This commands only pertains to parameter number 2 (starting position).
				if(parameter_number == 2)
				{
					// Set the starting servo pwm to be the max value.
					servo_pwm[servo_index] = max_pwm[servo_index];
					
					// Increment the parameter number so that the PSoC ends the calibration.
					parameter_number++;
				}
			}
			
			SERIAL_CmdReset(); // Emptying the RX buffer...
		}
	}
	
	LCD_1_Position(0,0);
	LCD_1_PrCString("                ");
	LCD_1_Position(1,0);
	LCD_1_PrCString("                ");
	
	// Check to see that the initialization was not cancelled before we turn the servo on.
	if (cancelled == 0)
	{
		// If the max pwm is smaller than the min pwm, we have to swap them and let it be
		// known that this servo's input degree values should be inverted so that the pwm
		// calculations work for the rotation axis (z axis) being pointed in either direction.
		if (min_pwm[servo_index] > max_pwm[servo_index])
		{
			// Swap the min and max pwm values.
			pwm_out = min_pwm[servo_index];
			min_pwm[servo_index] = max_pwm[servo_index];
			max_pwm[servo_index] = pwm_out;
			
			// Set servo inverted value to true.
			servo_inverted[servo_index] = 1;
		}
		else
		{
			// The servo does not need to be inverted, so inverting is turned off.
			servo_inverted[servo_index] = 0;
		}
		
		InitDegrees(servo_index);							// Initialize the degree values based on the input PWMs.
		servo_enabled[servo_index] = 1;						// The servo is now enabled since it has good data.
		MoveIndividualServo(servo_index);					// Move the servo to the starting position.
	}
	else
	{
		// Shut the servo off if the initialization was cancelled.
		if (servo_index == 0)
		{
			S0_Stop();	// Servo 0 (S0)
		}
		else if (servo_index == 1)
		{
			S1_Stop();	// Servo 1 (S1)
		}
		else if (servo_index == 2)
		{
			S2_Stop();	// Servo 2 (S2)
		}
		else if (servo_index == 3)
		{
			S3_Stop();	// Servo 3 (S3)
		}
		else if (servo_index == 4)
		{
			S4_Stop();	// Servo 4 (S4)
		}
		else if (servo_index == 5)
		{
			S5_Stop();	// Servo 5 (S5)
		}
	}
	
	// These blocks are turned off to keep from draining power that doesn't need to be used anymore.
	//SERVO_INIT_PGA_Stop();								// Stop PGA that feeds into the ADC.
	//SERVO_INIT_ADC_Stop();								// Stop ADC for servo initialization potentiometer.
	
	if (cancelled)
	{
		return -1;
	}
	
	return 0;
}

// Function to save the result of the servo calibration.
int SaveCalibration()
{
	int i;							// An index variable for the loop.
	int shift_result;				// Result of a bit shift.
	BYTE bError;					// A placeholder for the return value of the EEPROM write.
	BYTE new_data[DATA_LENGTH];		// An array of bytes for overwriting what is currently on the EEPROM.
	
	// Fill the BYTE array with zeros.
	for(i=0; i < NUM_SERVOS; i++)
	{
		// The first byte of the servo is easy, since the number can simply fit in one byte.
		new_data[0 + i*BYTES_PER_SERVO] = servo_range[i];
		
		// As is the second byte.
		new_data[1 + i*BYTES_PER_SERVO] = servo_inverted[i];
		
		// Converting min_pwm int value into two BYTES.
		// Shift to the right 8 bits (2^8 = 256) so that bits 15 through 8 can fit in one byte (7 through 0).
		shift_result = (min_pwm[i])/256;
		// Store the result of the shift.
		new_data[2 + i*BYTES_PER_SERVO] = shift_result;
		// Shift 8 bits to the left, subtract the result from the original total, and store the result.
		new_data[3 + i*BYTES_PER_SERVO] = min_pwm[i]-(shift_result*256);
		
		// Converting max_pwm int value into two BYTES.
		// Shift to the right 8 bits (2^8 = 256).
		shift_result = (max_pwm[i])/256;
		// Store the result of the shift.
		new_data[4 + i*BYTES_PER_SERVO] = shift_result;
		// Shift 8 bits to the left, subtract the result from the original total, and store the result.
		new_data[5 + i*BYTES_PER_SERVO] = max_pwm[i]-(shift_result*256);
		
		// Converting servo_pwm int value into two BYTES.
		// Shift to the right 8 bits (2^8 = 256).
		shift_result = (servo_pwm[i])/256;
		// Store the result of the shift.
		new_data[6 + i*BYTES_PER_SERVO] = shift_result;
		// Shift 8 bits to the left, subtract the result from the original total, and store the result.
		new_data[7 + i*BYTES_PER_SERVO] = servo_pwm[i]-(shift_result*256);
	}
	
	// Save values to memory.
	bError = SETTINGS_EEPROM_bE2Write(0, new_data, DATA_LENGTH, 25);
	
	// If the write was ok, return 0.  Otherwise, skip over the conditional and return an error code of -1.
	if ( bError == SETTINGS_EEPROM_NOERROR )
	{
		return 0;
	}
	else
	{
		return -1;
	}
}

// This function reads servo information from the EEPROM for two different servos,
// swaps them, and rewrites them back in the other's index.
int	FlipServoSettings(int index1, int index2)
{
//	int i;								// Index for looping through the BYTE arrays.
//	BYTE bError;						// A placeholder for the return value of the EEPROM write.
//	BYTE data[DATA_LENGTH];				// Stores all EEPROM data from the read.
//	BYTE ServoOne[BYTES_PER_SERVO];		// Temporarily stores data for the first servo.
//	
//	SETTINGS_EEPROM_E2Read( 0, data, DATA_LENGTH );		// Read all of the servo data.
//	
//	// Loop to swap all of the bytes.
//	for(i=0; i < BYTES_PER_SERVO; i++)
//	{
//		ServoOne[i] = data[i + index1*BYTES_PER_SERVO];						// Store a byte for servo one.
//		data[i + index1*BYTES_PER_SERVO] = data[i + index2*BYTES_PER_SERVO];// Move a byte from index two to one.
//		data[i + index2*BYTES_PER_SERVO] = ServoOne[i];						// Move the stored byte into index two.
//	}
//	
//	// Write the new data to memory.
//	bError = SETTINGS_EEPROM_bE2Write(0, data, DATA_LENGTH, 25);
//	
//	// If the write was ok, return 0.  Otherwise, skip over the conditional and return an error code of -1.
//	if ( bError == SETTINGS_EEPROM_NOERROR )
//	{
//		return 0;
//	}
//	else
//	{
//		return -1;
//	}
}

// This function reads servo information from the EEPROM for one servo,
// moves it to the new index, and erases the old index.
int	MoveServoSettings(int old_index, int new_index)
{
//	int i;								// Index for looping through the BYTE arrays.
//	BYTE bError;						// A placeholder for the return value of the EEPROM write.
//	BYTE data[DATA_LENGTH];				// Stores all EEPROM data from the read.
//	
//	SETTINGS_EEPROM_E2Read( 0, data, DATA_LENGTH );		// Read all of the servo data.
//	
//	// Loop to move all of the bytes.
//	for(i=0; i < BYTES_PER_SERVO; i++)
//	{
//		data[i + new_index*BYTES_PER_SERVO] = data[i + old_index*BYTES_PER_SERVO];	// Copy a byte from old to new.
//		data[i + old_index*BYTES_PER_SERVO] = CLEAN_BYTE;							// Clear the old byte.
//	}
//	
//	// Write the new data to memory.
//	bError = SETTINGS_EEPROM_bE2Write(0, data, DATA_LENGTH, 25);
//	
//	// If the write was ok, return 0.  Otherwise, skip over the conditional and return an error code of -1.
//	if ( bError == SETTINGS_EEPROM_NOERROR )
//	{
//		return 0;
//	}
//	else
//	{
//		return -1;
//	}
}

// This function writes whether or not the specified servo is enabled to the computer.
void ReturnServoStatus(int index)
{
	char* dummy_string;			// Just to appease the function call, since it returns a char* anyway.
	
	// Write a string representation of servo_enabled for this servo index to the computer.
	WriteSerial(itoa(dummy_string, servo_enabled[index], 10));
}

// This function writes the specified servo's range to the computer.
void ReturnServoRange(int index)
{
	char* dummy_string;			// Just to appease the function call, since it returns a char* anyway.
	
	// Write a string representation of servo_range for this servo index to the computer.
	WriteSerial(itoa(dummy_string, servo_range[index], 10));
}

int DeleteAllServoSettings()
{
//	int i;							// An index variable for the loop.
//	BYTE bError;					// A placeholder for the return value of the EEPROM write.
//	BYTE clean_data[DATA_LENGTH];	// An array of bytes for overwriting what is currently on the EEPROM.
//	
//	// Fill the BYTE array with clean bytes.
//	for(i=0; i < DATA_LENGTH; i++)
//	{
//		clean_data[i] = CLEAN_BYTE;
//	}
//
//	// Overwrite memory with all zeros.
//	bError = SETTINGS_EEPROM_bE2Write(0, clean_data, DATA_LENGTH, 25);
//	
//	// If the write was ok, return 0.  Otherwise, skip over the conditional and return an error code of -1.
//	if ( bError == SETTINGS_EEPROM_NOERROR )
//	{
//		return 0;
//	}
//	else
//	{
//		return -1;
//	}
}

int DeleteIndividualServoSetting(int index)
{
	int i;								// Index for looping through the BYTE arrays.
	BYTE bError;						// A placeholder for the return value of the EEPROM write.
	BYTE data[DATA_LENGTH];				// Stores all EEPROM data from the read.
	
	SETTINGS_EEPROM_E2Read( 0, data, DATA_LENGTH );		// Read all of the servo data.
	
	// Loop to clear all of the bytes saved for this servo index.
	for(i=0; i < BYTES_PER_SERVO; i++)
	{
		data[i + index*BYTES_PER_SERVO] = CLEAN_BYTE;	// Clear the old byte.
	}
	
	// Write the new data to memory.
	bError = SETTINGS_EEPROM_bE2Write(0, data, DATA_LENGTH, 25);
	
	// If the write was ok, return 0.  Otherwise, skip over the conditional and return an error code of -1.
	if ( bError == SETTINGS_EEPROM_NOERROR )
	{
		return 0;
	}
	else
	{
		return -1;
	}
}

void InitDegrees(int index)
{	
	// Calculate the servo's max and min degree range based on known pwm values.
	min_degrees[index] = 0-(((float)servo_pwm[index]-min_pwm[index])/(max_pwm[index]-min_pwm[index]))*
	                    (float)servo_range[index];
	max_degrees[index] = servo_range[index] + min_degrees[index];
	
	servo_degrees[index] = 0;			// All servos start at 0 degrees (their starting point).
}

void UpdateAllPWMs()
{
	int i;
	
	for (i=0; i < NUM_SERVOS; i++)
	{
		servo_pwm[i] = (((servo_degrees[i]-min_degrees[i])/servo_range[i])*
					   (max_pwm[i]-min_pwm[i]))+min_pwm[i];
	}
}

void ReturnLightPercentage()
{
	unsigned int sample;				// Used to store the sample from the ADC.
	float light_percentage = -1;		// Stores the converted percentage of light that is being witnessed.
	int status;							// Status of the ftoa conversion.
	
	while (light_percentage < 0)
	{
		// If there is data to read, read it.
		if(SERVO_INIT_ADC_fIsDataAvailable() != 0)
		{
			// Grab a sample from the ADC and clear the data ready flag.
			sample = SERVO_INIT_ADC_wClearFlagGetData();
			
			// Convert it to a percentage of light being received.
			light_percentage = (sample/pow(2,ADC_RESOLUTION))*100;
			
			LCD_1_Position(0,0);
			LCD_1_PrCString("                ");
			LCD_1_Position(0,0);
			LCD_1_PrCString("Light=");
			LCD_1_Position(0,6);
			LCD_1_PrString(ftoa(light_percentage, &status));
			
			// Return it to the user.
			WriteSerial(ftoa(light_percentage, &status));
		}
	}
}