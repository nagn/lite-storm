#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/adc.h"
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

//*****************************************************************************
//
//! \addtogroup adc_examples_list
//! <h1>Single Ended ADC (single_ended)</h1>
//!
//! This example shows how to setup ADC0 as a single ended input and take a
//! single sample on AIN0/PE3.
//!
//! This example uses the following peripherals and I/O signals.  You must
//! review these and change as needed for your own board:
//! - ADC0 peripheral
//! - GPIO Port E peripheral (for AIN0 pin)
//! - AIN0 - PE3
//!
//! The following UART signals are configured only for displaying console
//! messages for this example.  These are not required for operation of the
//! ADC.
//! - UART0 peripheral
//! - GPIO Port A peripheral (for UART0 pins)
//! - UART0RX - PA0
//! - UART0TX - PA1
//!
//! This example uses the following interrupt handlers.  To use this example
//! in your own application you must add these interrupt handlers to your
//! vector table.
//! - None.


#define SYMBOL_PERIOD 500
#define BAUD_RATE 40000

#define SAMPLE_PER_SYMBOL 4
#define WORD_LENGTH 10 // a byte is encoded as a 10-bit value with start and stop bits
#define SYNC_SYMBOL 0xD5 // this symbol breaks the premanble of the frame
#define ETX 0x03 // End of frame symbol
#define STX 0x02 //Start or frame symbol

const int EDGE_THRESHOLD = 100;
// #define EDGE_THRESHOLD 2000

uint32_t g_ui32SysClock;
uint32_t g_ui32WaitCycles;

typedef enum receiver_state {
  IDLE, //waiting for sync
  SYNC, //synced, waiting for STX
  START, //STX received
  DATA //receiving DATA
} receiver_state;

receiver_state frame_state = IDLE ;

// global variables for frame decoding
char frame_buffer[38] ;
int frame_index  = -1 ;
int frame_size = -1 ;


//state variables of the thresholder
unsigned int signal_mean = 0 ;
unsigned long acc_sum = 0 ; //used to compute the signal mean value
unsigned int acc_counter = 0 ;

//manechester decoder state variable
long shift_reg = 0;

#define START_SYMBOL 0x02
#define STOP_SYMBOL 0x01
#define START_STOP_MASK  ((STOP_SYMBOL << 20) | (START_SYMBOL << 18) | STOP_SYMBOL) //STOP/START/16bits/STOP
#define SYNC_SYMBOL_MANCHESTER  (0x6665)

//
// This array is used for storing the data read from the ADC FIFO. It
// must be as large as the FIFO for the sequencer in use.  This example
// uses sequence 3 which has a FIFO depth of 1.  If another sequence
// was used with a deeper FIFO, then the array size must be changed.
//
uint32_t sensorValue[1];

inline int is_a_word(long  * manchester_word, int time_from_last_sync, unsigned int * detected_word){
        if(time_from_last_sync >= 20  || frame_state == IDLE){ // we received enough bits to test the sync
            if(((*manchester_word) & START_STOP_MASK) == (START_STOP_MASK)){ // testing first position
                  (*detected_word) = ((*manchester_word) >> 2) & 0xFFFF;
                  if(frame_state == IDLE){
                     if((*detected_word) == SYNC_SYMBOL_MANCHESTER) return 2 ;
                  }
                  return 1 ;
                  // byte with correct framing
            }else if(frame_state != IDLE && time_from_last_sync == 20){
               (*detected_word)= ((*manchester_word) >> 2) & 0xFFFF;
               return 1 ;
            }
          }
          return 0 ;
}

inline int insert_edge( long  * manchester_word, int edge, int edge_period, int * time_from_last_sync, unsigned int * detected_word){
	int new_word_s = 0 ;
	int is_a_word_value = 0 ;
	int sync_word_detect = 0 ;
	if( ((*manchester_word) & 0x01) != edge ){ //mak sure we don't have same edge ...
			if(edge_period > (SAMPLE_PER_SYMBOL+1)){
                unsigned char last_bit = (*manchester_word) & 0x01 ;
                (*manchester_word) = ((*manchester_word) << 1) | last_bit ; // signal was steady for longer than a single symbol,
                (*time_from_last_sync) += 1 ;
                is_a_word_value = is_a_word(manchester_word, (*time_from_last_sync), detected_word);
                if(is_a_word_value > 0){ //found start stop framing
                	new_word_s = 1 ;
                  (*time_from_last_sync) =  0 ;
                  if(is_a_word_value > 1) sync_word_detect = 1 ; //we detected framing and sync word in manchester format
                }
             }
             //storing edge value in word
             if(edge < 0){
              (*manchester_word) = ( (*manchester_word) << 1) | 0x00 ; // signal goes down
             }else{
              (*manchester_word) = ( (*manchester_word) << 1) | 0x01 ; // signal goes up
             }
             (*time_from_last_sync) += 1 ;
             is_a_word_value = is_a_word(manchester_word, (*time_from_last_sync), detected_word);
             if(sync_word_detect == 0 && is_a_word_value > 0){ //if sync word was detected at previous position, don't take word detection into account
            	 new_word_s = 1 ;
               (*time_from_last_sync) =  0 ;
             }
          }else{
        	  new_word_s = -1 ;
          }
          return new_word_s;
}

int oldValue = 0 ;
int steady_count = 0 ;
int dist_last_sync = 0 ;
unsigned int detected_word = 0;
volatile int new_word = 0;
char old_edge_val = 0 ;
int test = 0;
int add_byte_to_frame(char * frame_buffer, int * frame_index, int * frame_size, enum receiver_state * frame_state ,unsigned char data){
  if(data == SYNC_SYMBOL/* && (*frame_index) < 0*/){
    (*frame_index) = 0 ;
    (*frame_size) = 0 ;
    (*frame_state) = SYNC ;
   // UARTprintf("SYNC\n");
    return 0 ;
  }
  if((*frame_state) != IDLE){ // we are synced
  frame_buffer[*frame_index] = data ;
  (*frame_index) ++ ;
    if(data == STX){
//      UARTprintf("START\n");
      (*frame_state) = START ;
       return 0 ;
    }else if(data == ETX){
 //      UARTprintf("END\n");
      (*frame_size) = (*frame_index) ;
      (*frame_index) = -1 ;
      (*frame_state) = IDLE ;
      //Serial.println("END");
       return 1 ;
    }else if((*frame_index) >= 38){ //frame is larger than max size of frame ...
      (*frame_index) = -1 ;
      (*frame_size) = -1 ;
      (*frame_state) = IDLE ;
      return -1 ;
    }else{
      (*frame_state) = DATA ;
    }
    return 0 ;
  }
  return -1 ;
}

void
Timer0IntHandler(void)
{
	//
	// Clear the timer interrupt.
	//
	ROM_TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    // Clear the ADC interrupt flag.
    //
    ADCIntClear(ADC0_BASE, 3);

    //
    // Read ADC Value.
    //
    ADCSequenceDataGet(ADC0_BASE, 3, sensorValue);

    int sensor = sensorValue[0];
//    UARTprintf("Value: %d \r", sensor);
    int edge_val ;
	if((sensor - oldValue) > EDGE_THRESHOLD) {
		edge_val = 1 ;
	} else if((oldValue - sensor) > EDGE_THRESHOLD) {
		edge_val = -1;
	} else  {
		edge_val = 0 ;
	}
	oldValue = sensor;
	if(edge_val == 0 || edge_val == old_edge_val || (edge_val != old_edge_val && steady_count < 2)){
		if( steady_count < (4 * SAMPLE_PER_SYMBOL)){
		  steady_count ++;
		}
	}else{
		  new_word = insert_edge(&shift_reg, edge_val, steady_count, &(dist_last_sync), &detected_word);
		  if(dist_last_sync > (8*SAMPLE_PER_SYMBOL)){ // limit dist_last_sync to avoid overflow problems
			dist_last_sync = 32 ;
		  }
		  //if(new_word >= 0){
			steady_count = 0 ;
		  //}
		}
		old_edge_val = edge_val ;
		// Trigger the ADC conversion.
		//
		ADCProcessorTrigger(ADC0_BASE, 3);
	    //
}

//
//*****************************************************************************

//*****************************************************************************
//
// This function sets up UART0 to be used for a console to display information
// as the example is running.
//
//*****************************************************************************
void
InitConsole(void)
{
    //
    // Enable GPIO port A which is used for UART0 pins.
    // TODO: change this to whichever GPIO port you are using.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //
    // Configure the pin muxing for UART0 functions on port A0 and A1.
    // This step is not necessary if your part does not support pin muxing.
    // TODO: change this to select the port/pin you are using.
    //
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);

    //
    // Enable UART0 so that we can configure the clock.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    //
    // Use the internal 16MHz oscillator as the UART clock source.
    //
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    // Select the alternate (UART) function for these pins.
    // TODO: change this to select the port/pin you are using.
    //
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Initialize the UART for console I/O.
    //
    UARTStdioConfig(0, 115200, 16000000);

}

//*****************************************************************************
//
// Configure ADC0 for a single-ended input and a single sample.  Once the
// sample is ready, an interrupt flag will be set.  Using a polling method,
// the data will be read then displayed on the console via UART0.
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
    // Set up the serial console to use for displaying messages.  This is
    // just for this example program and is not needed for ADC operation.
    //
    InitConsole();

    //
    // Display the setup on the console.
    //
    UARTprintf("ADC ->\n");
    //
    // The ADC0 peripheral must be enabled for use.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);

    //
    // For this example ADC0 is used with AIN0 on port E7.
    // The actual port and pins used may be different on your part, consult
    // the data sheet for more information.  GPIO port E needs to be enabled
    // so these pins can be used.
    // TODO: change this to whichever GPIO port you are using.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

    //
    // Select the analog ADC function for these pins.
    // Consult the data sheet to see which functions are allocated per pin.
    // TODO: change this to select the port/pin you are using.
    //
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3);

    //
    // Enable sample sequence 3 with a processor signal trigger.  Sequence 3
    // will do a single sample when the processor sends a signal to start the
    // conversion.  Each ADC module has 4 programmable sequences, sequence 0
    // to sequence 3.  This example is arbitrarily using sequence 3.
    //
    ADCSequenceConfigure(ADC0_BASE, 0, ADC_TRIGGER_PROCESSOR, 0);
    ADCSoftwareOversampleConfigure(ADC0_BASE, 0, 4);
    ADCSoftwareOversampleStepConfigure(ADC0_BASE, 0, 0, (ADC_CTL_CH0 \
    | ADC_CTL_IE | ADC_CTL_END));

    //
    // Configure step 0 on sequence 3.  Sample channel 0 (ADC_CTL_CH0) in
    // single-ended mode (default) and configure the interrupt flag
    // (ADC_CTL_IE) to be set when the sample is done.  Tell the ADC logic
    // that this is the last conversion on sequence 3 (ADC_CTL_END).  Sequence
    // 3 has only one programmable step.  Sequence 1 and 2 have 4 steps, and
    // sequence 0 has 8 programmable steps.  Since we are only doing a single
    // conversion using sequence 3 we will only configure step 0.  For more
    // information on the ADC sequences and steps, reference the datasheet.
    //

    ADCReferenceSet(ADC0_BASE,ADC_REF_INT);
    //
    // Since sample sequence 3 is now configured, it must be enabled.
    //
    ADCSequenceEnable(ADC0_BASE, 3);

    //
    // Clear the interrupt status flag.  This is done to make sure the
    // interrupt flag is cleared before we sample.
    //
    ADCIntClear(ADC0_BASE, 3);
	// Enable the peripherals used by this example.

    // Trigger the ADC conversion.
    //
    ADCProcessorTrigger(ADC0_BASE, 3);

	//
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
	//
	// Enable processor interrupts.
	//
	ROM_IntMasterEnable();
	//
	// Configure the one 32-bit periodic timers.
	//
	ROM_TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
	ROM_TimerLoadSet(TIMER0_BASE, TIMER_A, g_ui32WaitCycles / SAMPLE_PER_SYMBOL);
	//
	// Setup the interrupts for the timer timeouts.
	//
	ROM_IntEnable(INT_TIMER0A);
	ROM_TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

	//
	// Enable the timers.
	//
	ROM_TimerEnable(TIMER0_BASE, TIMER_A);
    while(1)
    {

    	int i;
		unsigned char received_data;
		char received_data_print ;
		int nb_shift ;
		int byte_added = 0 ;

		if(new_word == 1){
			received_data = 0 ;
			for(i = 0 ; i < 16 ; i = i + 2){ //decoding Manchester
					 received_data = received_data << 1 ;
					 if(((detected_word >> i) & 0x03) == 0x01){
						 received_data |= 0x01 ;
					 }else{
						 received_data &= ~0x01 ;
					 }
			}
			received_data = received_data & 0xFF;
	//		UARTprintf("Received: %x, %c\n", received_data & 0xFF, received_data);
			new_word = 0 ;
			if((byte_added = add_byte_to_frame(frame_buffer, &frame_index, &frame_size, &frame_state,received_data)) > 0){
			  frame_buffer[frame_size-1] = '\0';
				UARTprintf("%s\n", &(frame_buffer[1]));
			}
		}

    }
}
