//*****************************************************************************
//
// timers.c - Timers example.
//
// Copyright (c) 2013-2016 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
// 
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
// 
// This is part of revision 2.1.3.156 of the EK-TM4C1294XL Firmware Package.
//
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "timers.h"

#define TRANSMIT_SERIAL

// Symbol period is the period in microseconds
// 500 = 2000 baud
#define SYMBOL_PERIOD 500
#define BAUD_RATE 40000

#define WORD_LENGTH 10
#define SYNC_SYMBOL 0xD5
#define ETX 0x03
#define STX 0x02

#define SET_LED() GPIOPinWrite(GPIO_PORTM_BASE, GPIO_PIN_3, GPIO_PIN_3);
#define CLR_LED() GPIOPinWrite(GPIO_PORTM_BASE, GPIO_PIN_3, 0x0);


//*****************************************************************************
//
//! \addtogroup example_list
//! <h1>Timer (timers)</h1>
//!
//! This example application demonstrates the use of the timers to generate
//! periodic interrupts.  One timer is set up to interrupt once per second and
//! the other to interrupt twice per second; each interrupt handler will toggle
//! its own indicator throught the UART.
//!
//! UART0, connected to the Virtual Serial Port and running at 115,200, 8-N-1,
//! is used to display messages from this application.
//
//*****************************************************************************

//****************************************************************************
//
// System clock rate in Hz.
//
//****************************************************************************
uint32_t g_ui32SysClock;
uint32_t g_ui32WaitCycles;

//*****************************************************************************
//
// Flags that contain the current value of the interrupt indicator as displayed
// on the UART.
//
//*****************************************************************************
uint32_t g_ui32Flags;

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif

unsigned char frame_buffer [38] ; //buffer for frame
int frame_index = -1; // index in frame
int frame_size = -1  ; // size of the frame to be sent

//state variables of the manchester encoder
unsigned char bit_counter = 0 ;
unsigned char half_bit = 0 ;
unsigned long int manchester_data ;

//*****************************************************************************
//
// The interrupt handler for the second timer interrupt.
//
//*****************************************************************************
void
Timer0IntHandler(void)
{
    //
    // Clear the timer interrupt.
    //
    ROM_TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    if(manchester_data & 0x01){
    	SET_LED();
    }else{
    	CLR_LED();
    }
	bit_counter -- ;
	manchester_data = (manchester_data >> 1);
	if(bit_counter == 0){
		//is there still bytes to send in the frame ?
		manchester_data = 0xAAAAAAAA ; // keep sending ones if nothing to send
		if(frame_index >= 0 ){
			if(frame_index < frame_size){
				to_manchester(frame_buffer[frame_index], &manchester_data);
				frame_index ++ ;
			}else{
				frame_index = -1 ;
				frame_size = -1 ;
			}
		}
		bit_counter = WORD_LENGTH * 2 ;
	}
}

void delayMS(int ms) {
    ROM_SysCtlDelay( (g_ui32SysClock/(3*1000))*ms ) ;  // more accurate
}

void to_manchester(unsigned char data, unsigned long int * data_manchester){
  unsigned int i ;
 (*data_manchester) = 0x02 ; // STOP symbol
 (*data_manchester) = (*data_manchester) << 2 ;
  for(i = 0 ; i < 8; i ++){
    if(data & 0x80) (*data_manchester) |=  0x02  ; // data LSB first
    else (*data_manchester) |= 0x01 ;
    (*data_manchester) = (*data_manchester) << 2 ;
    data = data << 1 ; // to next bit
  }
  (*data_manchester) |= 0x01 ; //START symbol
}

void init_frame(unsigned char * frame){
	frame[0] = 0xAA;
	frame[1] = 0xAA;
	frame[2] = 0xAA;
	frame[3] = SYNC_SYMBOL ;
	frame[4] = STX;
	frame_index = -1 ;
	frame_size = -1 ;
}

int create_frame(char * data, int data_size, unsigned char * frame){
  memcpy(&(frame[5]), data, data_size);
  frame[5+data_size] = ETX;
  return 1 ;
}

int write(char * data, int data_size){
	if (frame_index >=  0) return -1;
	if (data_size > 32) return -1;
	create_frame(data, data_size,frame_buffer);

	ROM_IntMasterDisable();
	frame_index = 0 ;
	frame_size = data_size + 6;

	ROM_IntMasterEnable();

	return 0 ;
}

int transmitter_available(){
	if(frame_index >=  0) return 0 ;
	return 1 ;
}

void init_emitter(){
	manchester_data = 0xFFFFFFFF ;
	bit_counter = WORD_LENGTH * 2 ;
}

//*****************************************************************************
//
// Configure the UART and its pins.  This must be called before UARTprintf().
//
//*****************************************************************************
void
ConfigureUART(void)
{
    //
    // Enable the GPIO Peripheral used by the UART.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //
    // Enable UART0.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    //
    // Configure GPIO Pins for UART mode.
    //
    ROM_GPIOPinConfigure(GPIO_PA0_U0RX);
    ROM_GPIOPinConfigure(GPIO_PA1_U0TX);
    ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Initialize the UART for console I/O.
    //
    UARTStdioConfig(0, 115200, g_ui32SysClock);
}

//*****************************************************************************
//
// This example application demonstrates the use of the timers to generate
// periodic interrupts.
//
//*****************************************************************************




int
main(void)
{
    //
    // Set the clocking to run directly from the crystal at 120MHz.
    //
    g_ui32SysClock = MAP_SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
                                             SYSCTL_OSC_MAIN |
                                             SYSCTL_USE_PLL |
                                             SYSCTL_CFG_VCO_480), 120000000);

    g_ui32WaitCycles = (g_ui32SysClock / BAUD_RATE);
    //
    // Initialize the UART and write status.
    //
    ConfigureUART();

    UARTprintf("\033[2JTimers example\n");
    UARTprintf("T1: 0  T2: 0");


    //Enable PM3
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOM);

    //
    // Enable the GPIO pins for the Lamp.
    //
    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTM_BASE, GPIO_PIN_3);


    //
    // Enable the peripherals used by this example.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);

    //
    // Enable processor interrupts.
    //
    ROM_IntMasterEnable();

    //
    // Configure the two 32-bit periodic timers.
    //
    ROM_TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);

    ROM_TimerLoadSet(TIMER0_BASE, TIMER_A, g_ui32WaitCycles);

    //
    // Setup the interrupts for the timer timeouts.
    //
    ROM_IntEnable(INT_TIMER0A);
    ROM_TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    init_frame(frame_buffer);
    init_emitter();

    //
    // Enable the timers.
    //
    ROM_TimerEnable(TIMER0_BASE, TIMER_A);

    //
    // Loop forever while the timers run.
    //
    char * msg = "Hello World" ;
    char com_buffer [32] ;
    char com_buffer_nb_bytes = 0 ;
    while(1)
    {
	#ifdef TRANSMIT_SERIAL
	  while(transmitter_available() && ROM_UARTCharsAvail(UART0_BASE)){ //constructing the data frame only if transmitter is ready to transmit
		char c = ROM_UARTCharGetNonBlocking(UART0_BASE);
		com_buffer[com_buffer_nb_bytes] = c ;
		com_buffer_nb_bytes ++ ;
		if(com_buffer_nb_bytes >= 32 || c == '\n'){
		  if(write(com_buffer, com_buffer_nb_bytes) < 0){
			UARTprintf("Transmitter is busy! \n");
		  }else{
			com_buffer_nb_bytes = 0 ;
		  }
		}
	  }
	  #else
		static int i = 0 ;
		memcpy(com_buffer, msg, 11);
		com_buffer[11] = i + '0';
		if(write(com_buffer, 12) < 0){
			delayMS(20);
		}else{
			i ++;
			if(i > 9) i = 0 ;
		}
		#endif
    }
}
