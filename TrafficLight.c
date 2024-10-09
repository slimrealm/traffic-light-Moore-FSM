// TrafficLight.c
// Runs on LM4F120 or TM4C123
// Index implementation of a Moore finite state machine to operate
// a traffic light.
// Your Name: Sam Miller
// created:
// last modified by Sam Miller: 10/18/17

/* This example accompanies the book
   "Embedded Systems: Introduction to ARM Cortex M Microcontrollers",
   ISBN: 978-1469998749, Jonathan Valvano, copyright (c) 2013
   Volume 1 Program 6.8, Example 6.4
   "Embedded Systems: Real Time Interfacing to ARM Cortex M Microcontrollers",
   ISBN: 978-1463590154, Jonathan Valvano, copyright (c) 2013
   Volume 2 Program 3.1, Example 3.1

 Copyright 2013 by Jonathan W. Valvano, valvano@mail.utexas.edu
	You may use, edit, run or distribute this file
	as long as the above copyright notice remains
 THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 VALVANO SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL,
 OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/
 */

#include "PLL.h"
#include "SysTick.h"

#define SYSCTL_RCGC2_R (*((volatile unsigned long *)0x400FE108))

// Port definitions
// Port A
#define GPIO_PORTA_DATA_R (*((volatile unsigned long *)0x400043FC))
#define GPIO_PORTA_DIR_R (*((volatile unsigned long *)0x40004400))
#define GPIO_PORTA_AFSEL_R (*((volatile unsigned long *)0x40004420))
#define GPIO_PORTA_DEN_R (*((volatile unsigned long *)0x4000451C))
#define GPIO_PORTA_AMSEL_R (*((volatile unsigned long *)0x40004528))
#define GPIO_PORTA_PCTL_R (*((volatile unsigned long *)0x4000452C))

// Port E
#define GPIO_PORTE_DATA_R (*((volatile unsigned long *)0x400243FC))
#define GPIO_PORTE_DIR_R (*((volatile unsigned long *)0x40024400))
#define GPIO_PORTE_AFSEL_R (*((volatile unsigned long *)0x40024420))
#define GPIO_PORTE_DEN_R (*((volatile unsigned long *)0x4002451C))
#define GPIO_PORTE_AMSEL_R (*((volatile unsigned long *)0x40024528))
#define GPIO_PORTE_PCTL_R (*((volatile unsigned long *)0x4002452C))

// Port F
#define GPIO_PORTF_DATA_R (*((volatile unsigned long *)0x400253FC))
#define GPIO_PORTF_DIR_R (*((volatile unsigned long *)0x40025400))
#define GPIO_PORTF_AFSEL_R (*((volatile unsigned long *)0x40025420))
#define GPIO_PORTF_DEN_R (*((volatile unsigned long *)0x4002551C))
#define GPIO_PORTF_AMSEL_R (*((volatile unsigned long *)0x40025528))
#define GPIO_PORTF_PCTL_R (*((volatile unsigned long *)0x4002552C))

// Linked data structure
struct State
{
	unsigned long trafficLight;		   // R1,Y1,G1,R2,Y2,G2
	unsigned long walkLight;		   // W1,W2
	unsigned int time;				   // time in seconds
	unsigned long clearServicedInputs; // clear input(flag) when cars or walkers serviced
	struct State *nextState[16];	   // table of all possible next states
}; // struct state
typedef struct State stateName;

// States here (modeling intersection of Warner St. and First St.)
#define WarGrn &s[0]
#define WarGrnFirstWalk &s[1]
#define WarGrnDelay &s[2]
#define WarYellow &s[3]
#define FirstGreen &s[4]
#define FirstYellow &s[5]
#define WarWalk &s[6]

// Declare states here
// each state has trafficLight, walkLoght, time, nextState[16]
// trafficLight outputs shifted left 2 bits since we're starting at PA2 (not PA0)
// walkLight outputs shifted left 4 bits since we're starting at PC4 (not PC0)
stateName s[7] = {
	{0x30, 0x00, 7, 0x8, {WarGrn, WarGrnFirstWalk, WarYellow, WarYellow, WarYellow, WarYellow, WarYellow, WarYellow, // State table for s0
						  WarGrn, WarGrnFirstWalk, WarYellow, WarYellow, WarYellow, WarYellow, WarYellow, WarYellow}},

	{0x30, 0x10, 4, 0x1, {WarGrnDelay, WarGrnDelay, WarGrnDelay, WarGrnDelay, WarGrnDelay, WarGrnDelay, WarGrnDelay, WarGrnDelay, // State table for s1
						  WarGrnDelay, WarGrnDelay, WarGrnDelay, WarGrnDelay, WarGrnDelay, WarGrnDelay, WarGrnDelay, WarGrnDelay}},

	{0x30, 0x00, 3, 0x8, {WarGrn, WarGrn, WarYellow, WarYellow, WarYellow, WarYellow, WarYellow, WarYellow, // State table for s2
						  WarGrn, WarGrn, WarYellow, WarYellow, WarYellow, WarYellow, WarYellow, WarYellow}},

	{0x50, 0x00, 2, 0x0, {FirstGreen, WarWalk, WarWalk, WarWalk, FirstGreen, WarWalk, WarWalk, WarWalk, // State table for s3
						  FirstGreen, WarWalk, WarWalk, WarWalk, FirstGreen, WarWalk, WarWalk, WarWalk}},

	{0x84, 0x00, 7, 0x4, {FirstYellow, FirstYellow, FirstYellow, FirstYellow, FirstGreen, FirstYellow, FirstYellow, FirstYellow, // State table for s4
						  FirstYellow, FirstYellow, FirstYellow, FirstYellow, FirstYellow, FirstYellow, FirstYellow, FirstYellow}},

	{0x88, 0x00, 2, 0x0, {WarWalk, WarGrnFirstWalk, WarWalk, WarGrnFirstWalk, WarWalk, WarGrnFirstWalk, WarWalk, WarGrnFirstWalk, // State table for s5
						  WarGrn, WarGrnFirstWalk, WarGrn, WarGrnFirstWalk, WarGrn, WarGrnFirstWalk, WarGrn, WarGrnFirstWalk}},

	{0x90, 0x30, 4, 0x3, {WarGrn, WarGrn, FirstGreen, FirstGreen, FirstGreen, FirstGreen, FirstGreen, FirstGreen, // State table for s6
						  WarGrn, WarGrn, FirstGreen, FirstGreen, FirstGreen, FirstGreen, FirstGreen, FirstGreen}}};

// Declare port initialization functions
void PortA_Init(void);
void PortE_Init(void);
void PortF_Init(void);

// Use SysTick to invoke the delay in each state.
// Initialize SysTick in a separate function.
int main(void)
{
	volatile unsigned long delay;
	stateName *ptr;
	unsigned long input;			// SW1=0,SW2=0,B1=0,B2=0
	unsigned int currTime;			// time value of current state
	unsigned int i;					// for a for loop
	unsigned long inputFlags = 0x0; // when sensor triggered, the appropriate bits remain high until cars/walkers are serviced

	//  PLL_Init();       // 80 MHz, Program 10.1
	SysTick_Init(); // Program 10.2

	// Initialize ports
	PortA_Init();
	PortE_Init();
	PortF_Init();

	ptr = WarGrn;
	while (1)
	{
		// Code to transition from one state to another.
		GPIO_PORTA_DATA_R = ptr->trafficLight;
		GPIO_PORTE_DATA_R = ptr->walkLight;
		currTime = ptr->time;

		for (i = 1; i <= (currTime * 20); i++)
		{ // loop for appropriate # of seconds
			SysTick_Wait10ms(1);
			input = (GPIO_PORTF_DATA_R & 0x1E) >> 1; // read PF1,PF2,PF3,PF4 (input)
			if (input > 0)
				inputFlags |= input; // raise flags for every sensor and/or button pressed during the current state
		} // for

		inputFlags &= ~(ptr->clearServicedInputs); // clear inputs that were just now serviced
		ptr = ptr->nextState[inputFlags];		   // go to proper next state
	} // while 1
} // main

void PortA_Init(void)
{
	unsigned int clockDelay;
	SYSCTL_RCGC2_R |= 0x01;				// enable clock for Port A
	clockDelay = 1;						// stall so clock can stabilize
	GPIO_PORTA_DIR_R |= (0xFC);			// set bits 2,3,4,5,6,7 to make them outputs
	GPIO_PORTA_AFSEL_R &= (~0xFF);		// clear alternative func.
	GPIO_PORTA_DEN_R |= (0xFC);			// digital enable -  pins 2,3,4,5,6,7
	GPIO_PORTA_AMSEL_R &= (~0xFF);		// disable analog
	GPIO_PORTA_PCTL_R &= (~0xFFFFFF00); // pins 2,3,4,5,6,7 normal I/O
} // PortA_Init

void PortE_Init(void)
{
	unsigned int clockDelay;
	SYSCTL_RCGC2_R |= 0x10;				// enable clock for Port E
	clockDelay = 1;						// stall so clock can stabilize
	GPIO_PORTE_DIR_R |= (0x30);			// set bits 4 and 5 to make them outputs
	GPIO_PORTE_AFSEL_R &= (~0xFF);		// clear alternative func.
	GPIO_PORTE_DEN_R |= (0x30);			// digital enable -  pins 4,5
	GPIO_PORTE_AMSEL_R &= (~0xFF);		// disable analog
	GPIO_PORTE_PCTL_R &= (~0x00FF0000); // pins 4,5 normal I/O
} // PortE_Init

void PortF_Init(void)
{
	unsigned int clockDelay;
	SYSCTL_RCGC2_R |= 0x20;				// enable clock for Port F
	clockDelay = 1;						// stall so clock can stabilize
	GPIO_PORTF_DIR_R &= (~0x1E);		// clear bits 1,2,3,4 to make them inputs
	GPIO_PORTF_AFSEL_R &= (~0xFF);		// clear alternative func.
	GPIO_PORTF_DEN_R |= (0x1E);			// digital enable - pins 1,2,3,4
	GPIO_PORTF_AMSEL_R &= (~0xFF);		// disable analog
	GPIO_PORTF_PCTL_R &= (~0x000FFFF0); // pins 1,2,3,4 are normal I/O
} // PortF_Init
