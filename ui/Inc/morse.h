/*
 * morse.h
 *
 *  Created on: 27.02.2017
 *      Author: Wolfgang
 */

#ifndef MORSE_H_
#define MORSE_H_

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define MAXINT 4294967296
//maximum tone duration (2 seconds)
#define MAXTONE 1000

//calculate average of the first 100 input signals
#define NumberSigs 100
#define KeyUp 86
#define KeyDown 89

extern uint8_t CWsign;

extern volatile char PunktStrich[460];
extern volatile char zeile[72];
extern volatile char NewText;
extern volatile char decodedSign;

extern char RevCode[256];
extern uint8_t MCode[256];
//extern uint16_t buffer1[1024];
extern uint16_t pointerBuf1,pointerBuf2;
//extern uint16_t buffer2[1024];
extern char BufferFi[4][72];
extern char BufferF1[12];
extern uint16_t DitLength;
extern char start;
extern char gestartet;// CW- output has started
				//0: before sending
				//1: sending ./-
				//2: pause between ./-
extern char zeichen;// actual sent Dash/Dot/Pause CW element
extern char TestSignOld,TestSign;
extern char LastSign;
extern char Morsechar;
extern char NewData;
extern uint16_t PSinPointer;
extern uint16_t PSoutPointer;

extern uint8_t  Signal;
extern volatile uint16_t timer2;// increases all 10 ms
extern volatile uint16_t timer3;// increases all 6 ms
extern uint16_t color;// for monitoring
extern uint8_t drl;
extern uint16_t drk;

extern void DrawHigh();
extern void DrawLow();
extern void ShowMorseSign(void);

char* rx_rep_buf;
float cw_buffer[128];

enum cw_receive_state {
	RS_IDLE,
	RS_IN_TONE,
	RS_AFTER_TONE,
};

enum cw_decoder_state {
	DEC_EMPTY,
	DEC_READY
};

enum CW_TIMING_STATE {
	CW_FIRST_TIME_ELEMENT,
	CW_2ND_TIME_ELEMENT,
	CW_SYNCHRONIZED
};

#define	CW_SUCCESS	0
#define	CW_ERROR	-1
//#define	NULL	0



void MakeRevMorseCode();
void MakeMorseCode();
void DecodeMorseSign(void);
int8_t CodeMorse(uint8_t zeichen);
char DecodeMorse();
void SendMorsecode(void);

void CW_TIMER_EVENT(void);
void CW_KEYDOWN_EVENT(void);
void CW_KEYUP_EVENT(void);
void CW_RESET(void);
float moving_average(float average, float input, int weight);
float log2x(int32_t x);

#endif /* MORSE_H_ */
