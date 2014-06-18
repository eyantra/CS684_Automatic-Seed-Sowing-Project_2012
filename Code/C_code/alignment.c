#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include <math.h> //included to support power function
#include "lcd.c"



/********************************************************************************
 Written by: Vinod Desai, NEX Robotics Pvt. Ltd.
 Edited by: Sachitanand Malewar, NEX Robotics Pvt. Ltd.
 AVR Studio Version 4.17, Build 666

 Date: 26th December 2010

 Application example: Robot control over serial port via XBee wireless communication module
 					  (located on the ATMEGA260 microcontroller adaptor board)

 Concepts covered:  serial communication

 Serial Port used: UART0

 There are two components to the motion control:
 1. Direction control using pins PORTA0 to PORTA3
 2. Velocity control by PWM on pins PL3 and PL4 using OC5A and OC5B.

 In this experiment for the simplicity PL3 and PL4 are kept at logic 1.

 Pins for PWM are kept at logic 1.

 Connection Details:

  Motion control:		L-1---->PA0;		L-2---->PA1;
   						R-1---->PA2;		R-2---->PA3;
   						PL3 (OC5A) ----> Logic 1; 	PL4 (OC5B) ----> Logic 1;


  Serial Communication:	PORTD 2 --> RXD1 UART1 receive for RS232 serial communication
						PORTD 3 --> TXD1 UART1 transmit for RS232 serial communication

						PORTH 0 --> RXD2 UART 2 receive for USB - RS232 communication
						PORTH 1 --> TXD2 UART 2 transmit for USB - RS232 communication

						PORTE 0 --> RXD0 UART0 receive for ZigBee wireless communication
						PORTE 1 --> TXD0 UART0 transmit for ZigBee wireless communication

						PORTJ 0 --> RXD3 UART3 receive available on microcontroller expansion socket
						PORTJ 1 --> TXD3 UART3 transmit available on microcontroller expansion socket

Serial communication baud rate: 9600bps
To control robot use number pad of the keyboard which is located on the right hand side of the keyboard.
Make sure that NUM lock is on.

Commands:
			Keyboard Key   ASCII value	Action
				8				0x38	Forward
				2				0x32	Backward
				4				0x34	Left
				6				0x36	Right
				5				0x35	Stop
				7				0x37	Buzzer on
				9				0x39	Buzzer off

 Note:

 1. Make sure that in the configuration options following settings are
 	done for proper operation of the code

 	Microcontroller: atmega2560
 	Frequency: 14745600
 	Optimization: -O0 (For more information read section: Selecting proper optimization
 						options below figure 2.22 in the Software Manual)

 2. Difference between the codes for RS232 serial, USB and wireless communication is only in the serial port number.
 	Rest of the things are the same.

 3. For USB communication check the Jumper 1 position on the ATMEGA2560 microcontroller adaptor board

 4. Auxiliary power can supply current up to 1 Ampere while Battery can supply current up to
 	2 Ampere. When both motors of the robot changes direction suddenly without stopping,
	it produces large current surge. When robot is powered by Auxiliary power which can supply
	only 1 Ampere of current, sudden direction change in both the motors will cause current
	surge which can reset the microcontroller because of sudden fall in voltage.
	It is a good practice to stop the motors for at least 0.5seconds before changing
	the direction. This will also increase the useable time of the fully charged battery.
	the life of the motor.

*********************************************************************************/

/********************************************************************************

   Copyright (c) 2010, NEX Robotics Pvt. Ltd.                       -*- c -*-
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:

   * Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.

   * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in
     the documentation and/or other materials provided with the
     distribution.

   * Neither the name of the copyright holders nor the names of
     contributors may be used to endorse or promote products derived
     from this software without specific prior written permission.

   * Source code can be used for academic purpose.
	 For commercial use permission form the author needs to be taken.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE.

  Software released under Creative Commence cc by-nc-sa licence.
  For legal information refer to:
  http://creativecommons.org/licenses/by-nc-sa/3.0/legalcode

********************************************************************************/

unsigned char data; //to store received data from UDR1


typedef struct _vacantSpace
{
	unsigned int vacantSpace_SeedSow;
	unsigned int vacantSpace_SeedStop;
	unsigned int vacantSpace_Return;
	unsigned int vacantSpace_Rotate;
	unsigned int vacantSpace_Count;
	unsigned int skipWhiteLineJx;
} vacantSpace;


int automaticSeedSowing(unsigned int trough_ID);
int alignment(unsigned int trough_ID, vacantSpace *pstrVacantSpace);

//Function To Initialize UART0
// desired baud rate:9600
// actual baud rate:9600 (error 0.0%)
// char size: 8 bit
// parity: Disabled
void uart0_init(void)
{
 UCSR0B = 0x00; //disable while setting baud rate
 UCSR0A = 0x00;
 UCSR0C = 0x06;
 UBRR0L = 0x47; //11059200 Hz
// UBRR0L = 0x5F; // 14745600 Hzset baud rate lo
 UBRR0H = 0x00; //set baud rate hi
 UCSR0B = 0x98;
}

char fcall[5][5];
int i =0, j = 0;
int botId;

/* Function called by the ISR to interpret the data sent through Zigbee
and call the automaticSeedSowing function with trough ID*/
void function_caller()
{
	int val, par;
	val = atoi(fcall[1]); //fcall[1] would contain the task id 
	switch(val) {
		case 1 : automaticSeedSowing(atoi(fcall[2])); break; //fcall[2] would contain trough id
		default: UDR0 = 0x26;
	}
}

/* ISR to handle the data sent through Zigbee 
The data format is "botId$taskid$troughid#" */

SIGNAL(SIG_USART0_RECV)
{
	cli();
	data = UDR0;

	if(data == 0x23) // #
	{
		if(atoi(fcall[0]) == botId) {           //To verify if correct BOT ID is sent
			if(j != 0) {
				fcall[i][j] = 0;
				sei();
				function_caller();
				cli();
			}
			UDR0 = data;
		}
		i = 0;
		j = 0;
	}
	else if(data == 0x24) // $
	{
		fcall[i][j] = 0;
		i++;
		j = 0;
	}
	else
	{
		fcall[i][j] = data;
		j++;
	}
	sei();
}
void moveToRequiredAisle(unsigned int dst_aisle_count);
void port_init();
void timer5_init();
//void velocity(unsigned char, unsigned char);
//void motors_delay();

unsigned char ADC_Conversion(unsigned char);
unsigned char ADC_Value;
unsigned char flag2 = 0;
unsigned char flag1 = 0;
unsigned char Left_Sharp_Sensor=0;
unsigned char Right_Sharp_Sensor=0;
unsigned long int ShaftCountLeft = 0; //to keep track of left position encoder
unsigned long int ShaftCountRight = 0; //to keep track of right position encoder
unsigned int Degrees; //to accept angle in degrees for turning


void my_lcd_print(char row, char coloumn, unsigned int value, unsigned int digits)
{
	lcd_cursor(2, 1);
	lcd_print(row, coloumn, value, digits);
	return;
}

void my_lcd_string(char *str)
{
	lcd_cursor(1, 1);
	lcd_string(str);
	return;
}



//Function for velocity control
void velocity (unsigned char left_motor, unsigned char right_motor)
{
	OCR5AL = (unsigned char)left_motor;
	OCR5BL = (unsigned char)right_motor;
}



//Function to configure ports to enable robot's motion
void motion_pin_config (void)
{
 DDRA = DDRA | 0x0F;
 PORTA = PORTA & 0xF0;
 DDRL = DDRL | 0x18;   //Setting PL3 and PL4 pins as output for PWM generation
 PORTL = PORTL | 0x18; //PL3 and PL4 pins are for velocity control using PWM.
}

//Function to configure INT4 (PORTE 4) pin as input for the left position encoder
void left_encoder_pin_config (void)
{
 DDRE  = DDRE & 0xEF;  //Set the direction of the PORTE 4 pin as input
 PORTE = PORTE | 0x10; //Enable internal pull-up for PORTE 4 pin
}

//Function to configure INT5 (PORTE 5) pin as input for the right position encoder
void right_encoder_pin_config (void)
{
 DDRE  = DDRE & 0xDF;  //Set the direction of the PORTE 4 pin as input
 PORTE = PORTE | 0x20; //Enable internal pull-up for PORTE 4 pin
}



void left_position_encoder_interrupt_init (void) //Interrupt 4 enable
{
 cli(); //Clears the global interrupt
 EICRB = EICRB | 0x02; // INT4 is set to trigger with falling edge
 EIMSK = EIMSK | 0x10; // Enable Interrupt INT4 for left position encoder
 sei();   // Enables the global interrupt
}

void right_position_encoder_interrupt_init (void) //Interrupt 5 enable
{
 cli(); //Clears the global interrupt
 EICRB = EICRB | 0x08; // INT5 is set to trigger with falling edge
 EIMSK = EIMSK | 0x20; // Enable Interrupt INT5 for right position encoder
 sei();   // Enables the global interrupt
}

//ISR for right position encoder
ISR(INT5_vect)
{
 ShaftCountRight++;  //increment right shaft position count
}


//ISR for left position encoder
ISR(INT4_vect)
{
 ShaftCountLeft++;  //increment left shaft position count
}


//Function used for setting motor's direction
void motion_set (unsigned char Direction)
{
 unsigned char PortARestore = 0;

 Direction &= 0x0F; 		// removing upper nibbel for the protection
 PortARestore = PORTA; 		// reading the PORTA original status
 PortARestore &= 0xF0; 		// making lower direction nibbel to 0
 PortARestore |= Direction; // adding lower nibbel for forward command and restoring the PORTA status
 PORTA = PortARestore; 		// executing the command
}

void forward (void) //both wheels forward
{
  motion_set(0x06);
}

void back (void) //both wheels backward
{
  motion_set(0x09);
}

void left (void) //Left wheel backward, Right wheel forward
{
  motion_set(0x05);
}

void right (void) //Left wheel forward, Right wheel backward
{
  motion_set(0x0A);
}

void soft_left (void) //Left wheel stationary, Right wheel forward
{
 motion_set(0x04);
}

void soft_right (void) //Left wheel forward, Right wheel is stationary
{
 motion_set(0x02);
}

void soft_left_2 (void) //Left wheel backward, right wheel stationary
{
 motion_set(0x01);
}

void soft_right_2 (void) //Left wheel stationary, Right wheel backward
{
 motion_set(0x08);
}

void stop (void)
{
  motion_set(0x00);
}


//Function used for turning robot by specified degrees
void angle_rotate(unsigned int Degrees)
{
 float ReqdShaftCount = 0;
 unsigned long int ReqdShaftCountInt = 0;

 ReqdShaftCount = (float) Degrees/ 4.090; // division by resolution to get shaft count
 ReqdShaftCountInt = (unsigned int) ReqdShaftCount;
 ShaftCountRight = 0;
 ShaftCountLeft = 0;

 while (1)
 {
  if((ShaftCountRight >= ReqdShaftCountInt) | (ShaftCountLeft >= ReqdShaftCountInt))
  break;
 }
 stop(); //Stop robot
}

//Function used for moving robot forward by specified distance

void linear_distance_mm(unsigned int DistanceInMM)
{
 float ReqdShaftCount = 0;
 unsigned long int ReqdShaftCountInt = 0;

 ReqdShaftCount = DistanceInMM / 5.338; // division by resolution to get shaft count
 ReqdShaftCountInt = (unsigned long int) ReqdShaftCount;

 ShaftCountRight = 0;
 while(1)
 {
  if(ShaftCountRight > ReqdShaftCountInt)
  {
  	break;
  }
 }
 stop(); //Stop robot
}

void forward_mm(unsigned int DistanceInMM)
{
 forward();
 linear_distance_mm(DistanceInMM);
}

void back_mm(unsigned int DistanceInMM)
{
 back();
 linear_distance_mm(DistanceInMM);
}

void left_degrees(unsigned int Degrees)
{
// 88 pulses for 360 degrees rotation 4.090 degrees per count
 left(); //Turn left
 angle_rotate(Degrees);
}



void right_degrees(unsigned int Degrees)
{
// 88 pulses for 360 degrees rotation 4.090 degrees per count
 right(); //Turn right
 angle_rotate(Degrees);
}


void soft_left_degrees(unsigned int Degrees)
{
 // 176 pulses for 360 degrees rotation 2.045 degrees per count
 soft_left(); //Turn soft left
 Degrees = Degrees * 2;
 angle_rotate(Degrees);
}

void soft_right_degrees(unsigned int Degrees)
{
 // 176 pulses for 360 degrees rotation 2.045 degrees per count
 soft_right();  //Turn soft right
 Degrees=Degrees*2;
 angle_rotate(Degrees);
}

void soft_left_2_degrees(unsigned int Degrees)
{
 // 176 pulses for 360 degrees rotation 2.045 degrees per count
 soft_left_2(); //Turn reverse soft left
 Degrees=Degrees*2;
 angle_rotate(Degrees);
}

void soft_right_2_degrees(unsigned int Degrees)
{
 // 176 pulses for 360 degrees rotation 2.045 degrees per count
 soft_right_2();  //Turn reverse soft right
 Degrees=Degrees*2;
 angle_rotate(Degrees);
}


/**********************************/

void servo1_pin_config (void)
{
 DDRB  = DDRB | 0x20;  //making PORTB 5 pin output
 PORTB = PORTB | 0x20; //setting PORTB 5 pin to logic 1

}

//TIMER1 initialization in 10 bit fast PWM mode
//prescale:256
// WGM: 7) PWM 10bit fast, TOP=0x03FF
// actual value: 52.25Hz
void timer1_init(void)
{
 TCCR1B = 0x00; //stop
 TCNT1H = 0xFC; //Counter high value to which OCR1xH value is to be compared with
 TCNT1L = 0x01;	//Counter low value to which OCR1xH value is to be compared with
 OCR1AH = 0x03;	//Output compare Register high value for servo 1
 OCR1AL = 0xFF;	//Output Compare Register low Value For servo 1
 ICR1H  = 0x03;
 ICR1L  = 0xFF;
 TCCR1A = 0xAB; /*{COM1A1=1, COM1A0=0; COM1B1=1, COM1B0=0; COM1C1=1 COM1C0=0}
 					For Overriding normal port functionality to OCRnA outputs.
				  {WGM11=1, WGM10=1} Along With WGM12 in TCCR1B for Selecting FAST PWM Mode*/
 TCCR1C = 0x00;
 TCCR1B = 0x0C; //WGM12=1; CS12=1, CS11=0, CS10=0 (Prescaler=256)
}


//Function to configure LCD port
void lcd_port_config (void)
{
 DDRC = DDRC | 0xF7; //all the LCD pin's direction set as output
 PORTC = PORTC & 0x80; // all the LCD pins are set to logic 0 except PORTC 7
}

//ADC pin configuration
void adc_pin_config (void)
{
 DDRF = 0x00;
 PORTF = 0x00;
 DDRK = 0x00;
 PORTK = 0x00;
}




void adc_init()
{
	ADCSRA = 0x00;
	ADCSRB = 0x00;		//MUX5 = 0
	ADMUX = 0x20;		//Vref=5V external --- ADLAR=1 --- MUX4:0 = 0000
	ACSR = 0x80;
	ADCSRA = 0x86;		//ADEN=1 --- ADIE=1 --- ADPS2:0 = 1 1 0
}

//Function For ADC Conversion
unsigned char ADC_Conversion(unsigned char Ch)
{

	unsigned char a;
	if(Ch>7)
	{
		ADCSRB = 0x08;
	}
	Ch = Ch & 0x07;
	ADMUX= 0x20| Ch;
	ADCSRA = ADCSRA | 0x40;		//Set start conversion bit
	while((ADCSRA&0x10)==0);	//Wait for conversion to complete
	a=ADCH;
	ADCSRA = ADCSRA|0x10; //clear ADIF (ADC Interrupt Flag) by writing 1 to it
	ADCSRB = 0x00;
	return a;

}

void print_sensor(char row, char coloumn,unsigned char channel)
{

	ADC_Value = ADC_Conversion(channel);
	my_lcd_print(row, coloumn, ADC_Value, 3);
}

//Function to initialize ports
void port_init()
{
servo1_pin_config();
 motion_pin_config(); //robot motion pins config
 left_encoder_pin_config(); //left encoder pin config
 right_encoder_pin_config(); //right encoder pin config
 lcd_port_config();
adc_pin_config();

}

// Timer 5 initialized in PWM mode for velocity control
// Prescale:256
// PWM 8bit fast, TOP=0x00FF
// Timer Frequency:225.000Hz
void timer5_init()
{
	TCCR5B = 0x00;	//Stop
	TCNT5H = 0xFF;	//Counter higher 8-bit value to which OCR5xH value is compared with
	TCNT5L = 0x01;	//Counter lower 8-bit value to which OCR5xH value is compared with
	OCR5AH = 0x00;	//Output compare register high value for Left Motor
	OCR5AL = 0xFF;	//Output compare register low value for Left Motor
	OCR5BH = 0x00;	//Output compare register high value for Right Motor
	OCR5BL = 0xFF;	//Output compare register low value for Right Motor
	OCR5CH = 0x00;	//Output compare register high value for Motor C1
	OCR5CL = 0xFF;	//Output compare register low value for Motor C1
	TCCR5A = 0xA9;	/*{COM5A1=1, COM5A0=0; COM5B1=1, COM5B0=0; COM5C1=1 COM5C0=0}
 					  For Overriding normal port functionality to OCRnA outputs.
				  	  {WGM51=0, WGM50=1} Along With WGM52 in TCCR5B for Selecting FAST PWM 8-bit Mode*/

	TCCR5B = 0x0B;	//WGM12=1; CS12=0, CS11=1, CS10=1 (Prescaler=64)
}


//Function to initialize all the devices
void init_devices()
{
 cli(); //Clears the global interrupt
 port_init();  //Initializes all the ports
 timer1_init();
 timer5_init(); //for white line follower
 left_position_encoder_interrupt_init();
 right_position_encoder_interrupt_init();
 adc_init();
 sei();   // Enables the global interrupt
}


//Function to rotate Servo 1 by a specified angle in the multiples of 1.86 degrees
void servo_1(float degrees)
{
    float PositionPanServo = 0;
    PositionPanServo = ((float)degrees / 1.86) + 35.0;
    OCR1AH = 0x00;
    OCR1AL = (unsigned char) PositionPanServo;
}



#define LEFT_THRESHOLD_min  	107
#define RIGHT_THRESHOLD_min 	65
#define LEFT_THRESHOLD_max  	117
#define RIGHT_THRESHOLD_max 	75
#define NO_TROUGH_THRESHOLD      50
#define STARTING_TROUGH_INDEX     0
#define STARTING_AISLE_INDEX      0

#define TROUGH_ID				  3
#define TROUGHS_IN_AISLE          4
#define TROUGHS_IN_ONE_ROW   	  2
#define TROUGH_LENGTH_IN_MM	    800
#define BACK_MOVE				 30
#define DELAY					 500

#define ROTATE          		 5
#define ROTATE_MORE     		 10
#define ROTATE_RIGHT_DEGREES     180


void clear_lcd()
{
	lcd_cursor(1,1);
	my_lcd_string("                       ");

	lcd_cursor(2,1);
	my_lcd_string("                       ");
}



void initVacantSpaceStruct(vacantSpace *pstVacantSpace, unsigned int troughID)
{
	char right = 0;
	unsigned int skipWhiteLineJx = 0;
	unsigned int trough_ID = 0;
	unsigned int remainder = 0;

	skipWhiteLineJx = troughID / TROUGHS_IN_AISLE;
	remainder = troughID % TROUGHS_IN_AISLE;

	right = remainder / TROUGHS_IN_ONE_ROW;
	trough_ID = remainder;

	pstVacantSpace->vacantSpace_SeedSow    = (trough_ID  + 1);
	pstVacantSpace->vacantSpace_SeedStop   = (trough_ID  + 2);
	pstVacantSpace->skipWhiteLineJx		   = skipWhiteLineJx;

    /* Initial vacant count has been kept as 1 as when the bot starts
     * from initial position, there is no trough, so it assumes that
     * there is a vacant space.
     */
    pstVacantSpace->vacantSpace_Count      = 1;

	clear_lcd();
	my_lcd_string("Seed Sow");
	my_lcd_print(2,1, pstVacantSpace->vacantSpace_SeedSow, 2);
	_delay_ms(DELAY);

	clear_lcd();
	my_lcd_string("Seed Stop");
	my_lcd_print(2,1, pstVacantSpace->vacantSpace_SeedStop, 2);
	_delay_ms(DELAY);

	clear_lcd();
	my_lcd_string("Skip White Jx");
	my_lcd_print(2,1, pstVacantSpace->skipWhiteLineJx, 2);
	_delay_ms(DELAY);

	clear_lcd();
	my_lcd_string("Right");
	my_lcd_print(2,1, right, 2);
	_delay_ms(DELAY);

	if (right == 1)
	{
		pstVacantSpace->vacantSpace_Rotate = (trough_ID - TROUGHS_IN_ONE_ROW + 2);
		pstVacantSpace->vacantSpace_Return = ((trough_ID - TROUGHS_IN_ONE_ROW) * 2) + 4;
	}
	else
	{
		pstVacantSpace->vacantSpace_Return = ((trough_ID) * 2) + 4;
		pstVacantSpace->vacantSpace_Rotate = (trough_ID + 2);
	}

	clear_lcd();
	my_lcd_string("Return");
	my_lcd_print(2,1, pstVacantSpace->vacantSpace_Return, 2);
	_delay_ms(DELAY);

	clear_lcd();
	my_lcd_string("Rotate");
	my_lcd_print(2,1, pstVacantSpace->vacantSpace_Rotate, 2);
	_delay_ms(DELAY);

	clear_lcd();
	_delay_ms(DELAY);
}

/*
 * retVal = 1 (Seed Sow)
 * retVal = 0 (Stop Sowing)
 * retVal = 2 (Return)
*/


unsigned int decideWhatToDoOnVacantSpace(vacantSpace *pstVacantSpace)
{
	unsigned int vacantSpace_Count = 0;
	unsigned int retVal = 0, forward = 0, came = 0;


	vacantSpace_Count = pstVacantSpace->vacantSpace_Count;

	vacantSpace_Count++;
	pstVacantSpace->vacantSpace_Count = vacantSpace_Count;

	clear_lcd();
	my_lcd_string("WhatToDo?");
	_delay_ms(DELAY);

	clear_lcd();
	my_lcd_string("Vacant Count");
	my_lcd_print(2,1, vacantSpace_Count, 2);
	_delay_ms(DELAY);

    /* Order in which these cases have been written is important.
     * It is possible that after the same vacant space the bot
     * has to rotate and has to stop/start sowing seed.
     */
	if (vacantSpace_Count == pstVacantSpace->vacantSpace_Rotate)
	{
		clear_lcd();
		my_lcd_string("CAME ROTATE");
		_delay_ms(DELAY);

		right_degrees(ROTATE_RIGHT_DEGREES);
		came = 1;

		vacantSpace_Count++;
		pstVacantSpace->vacantSpace_Count = vacantSpace_Count;

        /* When bot rotates, no need to move bot in forward direction */
	}
	else if (vacantSpace_Count == pstVacantSpace->vacantSpace_Return)
	{
		clear_lcd();
		my_lcd_string("CAME RETURN");
		_delay_ms(DELAY);

		retVal = 2;
		forward = 1;
		came = 1;
	}
    else
    {
        forward = 1;
        came = 1;
    }

	if (vacantSpace_Count == pstVacantSpace->vacantSpace_SeedSow)
	{
		clear_lcd();
		my_lcd_string("CAME SEED SOW");
		_delay_ms(DELAY);
		retVal = 1;
		forward = 1;
		came = 1;
	}
	else if (vacantSpace_Count == pstVacantSpace->vacantSpace_SeedStop)
	{
		clear_lcd();
		my_lcd_string("CAME SEED STOP");
		_delay_ms(DELAY);

		retVal = 0;
		forward = 1;
		came = 1;
	}

	if (1 == forward)
	{
		clear_lcd();
		my_lcd_string("CAME FORWARD");
		_delay_ms(DELAY);
		forward_mm(60);
	}

	if (came == 0)
	{
		my_lcd_string("ERROR");
        _delay_ms(DELAY);
	}


	clear_lcd();
	my_lcd_string("retVal");
	my_lcd_print(2,1,retVal, 2);
	_delay_ms(DELAY);

	return retVal;
}




int automaticSeedSowing(unsigned int trough_ID)
{
	vacantSpace   strVacantSpace = {0, };

    init_devices();
    servo_1(1.86 * 4);
	_delay_ms(60);
	servo_1(1.86 * 0);
	_delay_ms(60);
	servo_1(1.86 * 4);
	_delay_ms(60);

    /* Init LCD */
	lcd_set_4bit();
	lcd_init();

    /* Display the trough ID received on LCD */
	lcd_cursor(1, 1);
	my_lcd_string("Trough ID");
	my_lcd_print(2,1, trough_ID, 2);
	_delay_ms(DELAY);

    /* Init the structure strVacantSpace for given trough ID */
	initVacantSpaceStruct(&strVacantSpace, trough_ID);

	clear_lcd();

    /* The function moves the bot to the required aisle. If there are multiple aisle,
     * we assume that to the begining of each asile, there is white line diversion.
     * For details please refer to the report. So basically the bot moves from the
     * starting position to the required aisle following the white line ans skipping
     * the juntions, until the bot reaches the required aisle. Once the bot reaches
     * required aisle, it rotates 90 degree and then moves forward until it the sensors
     * read all black. After that bot is in aisle and it uses sharp sensors to naviagate
     * through the aisle.
     */
    // moveToRequiredAisle(strVacantSpace.skipWhiteLineJx);

    /* The alignement function takes care of the navigation of the bot in the aisle.
     * Bot uses the values of both sharp sensors to align itself so as to move straight
     * in the aisle. Moving straight is important so as to keep track of how much distance
     * the bot has moved as the bot has plant seeds at definite inter-seeding distance.
     */
	alignment(trough_ID, &strVacantSpace);

    return 0;
}

int alignment(unsigned int trough_ID, vacantSpace *pstrVacantSpace)
{
	unsigned char buff, Left_Temp_Sharp_Sensor;
	unsigned int  count = 0;
    unsigned char bCanSowSeed = 0;  	/* Determines if the bot can sow seed or not.
                                         * Is made 1 when bot reaches the trough where
                                         * it has to sow seeds.
                                         */

	unsigned int  move_flag = 0, retVal = 0;


	/* If the bot has to sow seed in the 0th trough in the aisle
     * Trough count starts with 0 and goes till 3 for each aisle
     * in clock-wise direction.
     */
	if (pstrVacantSpace->vacantSpace_SeedSow == 1)
	{
		bCanSowSeed = 1;
	}

    while(1)
	{
		/* choosing the velocity of movement of motors */
        velocity(250, 250);

    	/* Drop seed now */
        if ((count == 5) && (1 == bCanSowSeed))
    	{
            lcd_cursor(1,1);
			my_lcd_string("SOWING!!!");
			count = 0;
            stop();
            _delay_ms(2000);
        	servo_1(1.86 * 4);
        	_delay_ms(60);
        	servo_1(1.86 * 0);
        	_delay_ms(60);
        	servo_1(1.86 * 4);

            _delay_ms(2000);
	    }

    	//say the ideal distance is 80mm
    	//distance of 80mm corresponds to an ADC value of 160
    	/* Get value from left sharp sensor */
        Left_Sharp_Sensor = ADC_Conversion(9);
    	lcd_cursor(2,1);
		print_sensor(2,4,9);	//Prints Value of Left Sharp Sensor

    	/* Get value from right sharp sensor */
    	Right_Sharp_Sensor = ADC_Conversion(13);
    	lcd_cursor(2,1);
		print_sensor(2,8,13);	//Prints Value of Right Sharp Sensor

        /* When bot is properly aligned with both the trough. In that
         * case bot has to just move straight by 10mm.
         */
		if ((Left_Sharp_Sensor  >= LEFT_THRESHOLD_min)
    	&&  (Left_Sharp_Sensor  <= LEFT_THRESHOLD_max)
    	&&  (Right_Sharp_Sensor >= RIGHT_THRESHOLD_min)
    	&&  (Right_Sharp_Sensor <= RIGHT_THRESHOLD_max))
    	{
    		lcd_cursor(1,1);
            lcd_string("forward");
            forward_mm(10);
    		stop();

    		_delay_ms(DELAY);

    		count++;
    	}

    	/* Bot is misalinged and has turned right a bit */
        else if ((Left_Sharp_Sensor  < LEFT_THRESHOLD_min)
             &&  (Left_Sharp_Sensor  > NO_TROUGH_THRESHOLD)
             &&  (Right_Sharp_Sensor > NO_TROUGH_THRESHOLD)
    	  	 &&  (Right_Sharp_Sensor < RIGHT_THRESHOLD_min))
    	{
        	lcd_cursor(1,1);
            lcd_string("Both wrong");
            _delay_ms(500);
            lcd_cursor(1,1);
            lcd_string("left rotate");
            left_degrees(ROTATE); //Rotate robot left by 90 degrees
        	stop();
        	_delay_ms(500);
        	buff = Left_Sharp_Sensor;

        	Left_Temp_Sharp_Sensor = ADC_Conversion(9);
        	if (Left_Temp_Sharp_Sensor > buff)
            {
        	    lcd_cursor(1,1);
                lcd_string("right rotate");
                right_degrees(ROTATE_MORE);
                stop();
				_delay_ms(DELAY);
            }

        	stop();
        	_delay_ms(DELAY);
        }
    	/* Bot is misalinged and has turned right a bit */
		else if ((Left_Sharp_Sensor < LEFT_THRESHOLD_min)
			 &&  (Left_Sharp_Sensor > NO_TROUGH_THRESHOLD))
    	{
        	lcd_cursor(1,1);
            soft_right_degrees(ROTATE);
        	stop();
        	_delay_ms(DELAY);
            buff = Left_Sharp_Sensor;

        	Left_Temp_Sharp_Sensor = ADC_Conversion(9);

            if (Left_Temp_Sharp_Sensor < buff)
            {
        	    lcd_cursor(1,1);
                lcd_string("Reverse left rotate");
                soft_left_2_degrees(ROTATE_MORE);
                stop();
            	_delay_ms(DELAY);
                forward_mm(10);
                stop();
                _delay_ms(DELAY);
                move_flag = 1;
            }

            if (move_flag == 0)
            {
				back_mm(10);
				stop();
				_delay_ms(DELAY);
            }

        	stop();
        	_delay_ms(DELAY);
            move_flag = 0;
    	}

    	/* Bot is misalinged and has turned left a bit */
        else if ((Left_Sharp_Sensor > LEFT_THRESHOLD_max)
             &&  (Right_Sharp_Sensor > NO_TROUGH_THRESHOLD))
    	{
        	lcd_cursor(1,1);
            lcd_string("soft right rotate");
            soft_right_degrees(ROTATE);
        	stop();
        	_delay_ms(DELAY);
    	}
        /* Bot has to a vacant space. At vacant space the reading from both
         * the sharp sensors will be less than a threshold value
         */
        else if ((Right_Sharp_Sensor < NO_TROUGH_THRESHOLD)
             ||  (Left_Sharp_Sensor  < NO_TROUGH_THRESHOLD))
   	    {
			retVal = decideWhatToDoOnVacantSpace(pstrVacantSpace);
			clear_lcd();
			my_lcd_string("retVal");
			my_lcd_print(2,1, retVal, 1);

			if (1 == retVal)
			{
				bCanSowSeed = 1;
				clear_lcd();
				my_lcd_string("SOW SEEDS");
				_delay_ms(DELAY);
				count = 0;
			}
			else
			{
				bCanSowSeed = 0;
				clear_lcd();
				my_lcd_string("STOP SOW SEEDS");
				_delay_ms(DELAY);
			}

			if (2 == retVal)
			{
				clear_lcd();
				my_lcd_string("RETURN");
			}
		}
	}
}


void moveToRequiredAisle(unsigned int dst_aisle_count)
{
	unsigned int tmp_aisle_count = 0;
	unsigned char Left_white_line = 0;
	unsigned char Center_white_line = 0;
	unsigned char Right_white_line = 0;
    unsigned char flag = 0;

	//velocity(200, 200);

	while(1)
	{
		Left_white_line   = ADC_Conversion(3);	//Getting data of Left WL Sensor
		Center_white_line = ADC_Conversion(2);	//Getting data of Center WL Sensor
		Right_white_line  = ADC_Conversion(1);	//Getting data of Right WL Sensor

		flag = 0;

		print_sensor(2,1,3);	//Prints value of White Line Sensor1
		print_sensor(2,5,2);	//Prints Value of White Line Sensor2
		print_sensor(2,9,1);	//Prints Value of White Line Sensor3


        lcd_cursor(1,1);
		my_lcd_string("Follow Line");
        _delay_ms(DELAY);

		if (Center_white_line < 0x28 && Left_white_line < 0x28 && Right_white_line < 0x28)
		{
			flag = 1;

			stop();
			_delay_ms(1000);

			if (dst_aisle_count == tmp_aisle_count)
			{
				lcd_cursor(1, 1);
				my_lcd_string("Rched Dst Aisle");
				stop();
				_delay_ms(1000);

				soft_right_degrees(90); // right by 90 degrees
			}
			else
			{
				lcd_cursor(1, 1);
				my_lcd_string("ELSE!!!");
				stop();
				_delay_ms(DELAY);
				forward_mm(50);
			}

			tmp_aisle_count++;
			lcd_cursor(1, 1);
			my_lcd_print(1, 1, tmp_aisle_count, 2);
			stop();
			_delay_ms(DELAY);
		}

		if ((Center_white_line < 0x28) && (flag == 0))
		{
			flag = 1;
			forward();
			velocity(150, 150);
		}

		if ((Left_white_line > 0x28) && (flag == 0))
		{
			flag = 1;
			forward();
			velocity(130,50);
		}

		if ((Right_white_line > 0x28) && (flag == 0))
		{
			flag = 1;
			forward();
			velocity(50,130);
		}

		if (Center_white_line > 0x28 && Left_white_line > 0x28 && Right_white_line > 0x28)
		{
			lcd_cursor(1,1);
			my_lcd_string("Black!!");
			stop();
			_delay_ms(1000);
			break;
		}
	}

	lcd_cursor(1,1);
    my_lcd_string("Returned!!");
    stop();
    _delay_ms(DELAY);
    return;
}


//Main Function
int main(void)
{
	botId = 2; //BOTID set which is identified by the remote interface
	cli(); //Clears the global interrupts
	uart0_init(); //Initailize UART1 for serial communiaction
	sei();   // Enables the global interrupt
	while(1);
}

