/*
 * timers.h
 *
 *  Created on: Nov 19, 2016
 *      Author: mlin4_000
 */

#ifndef TIMERS_H_
#define TIMERS_H_


void init_frame(unsigned char * frame);
int create_frame(char * data, int data_size, unsigned char * frame);
int write(char * data, int data_size);
int transmitter_available();
void init_emitter();
void delayMS(int ms);
void to_manchester(unsigned char data, unsigned long int * data_manchester);


#endif /* TIMERS_H_ */
