/*
 * morse.c
 *
 *  Created on: 27.02.2017
 *      Author: Wolfgang
 */
#include "morse.h"

uint16_t countlow, counthigh, lpause, lowmin, lowmax, signaltime_min, signaltime_max, lowmed, prevhigh, prevlow;
uint16_t timer, criterion, counter_a, counter_b;
char Last3[3];
uint8_t IndLast3 = 0, digits = 0, countmin, countmax, countmed, HighCounter;
char phase, i, beginToDecode;
static int space_sent = false;
; // for word space logic

int two_dots;
float sig_avg, noise_floor, noise_filtered, agc_peak, agc_factor;
float norm_peak, norm_avg, norm_value, value1, previous_value;
float metric = 0.0, siglevel, CWupper, CWlower, SigDiff;
int cw_receive_state, cw_rr_current, element_counter;
int cw_noise_spike_threshold, cw_dot_length;
static int prev_duration; // length of last dot/dash
uint32_t CW_pauseTime;
uint32_t smpl_ctr, cw_keyup_timestamp, cw_keydown_timestamp;
volatile int cw_decoder_state;
uint32_t duration; // Time difference in 2 usecs
uint32_t curr_timestamp;
uint16_t length1, length2;
int CW_TIMING_STATE;
const float log2c[16] = {0.0, 0.09, 0.17, 0.25, 0.32, 0.39, 0.46, 0.52, 0.58, 0.64, 0.70, 0.75, 0.81, 0.86, 0.91, 0.95};

#define absmax 640
#define absmin 30
#define Delay 30
#define signcounter 60
#define threshold 2
#define ZLength 100
#define TestLength 3

char zeichenspeicher[ZLength];
uint8_t insertpointer = 0;
uint8_t readpointer = 0;
int32_t OldS;
uint16_t TimerLow;
uint16_t TimerHigh;
char ChangeOccured = 0;
char LowHigh = 0;
char HighLow = 0;
char ToDecode = 0, LastClass;
uint16_t positx, posity;

uint8_t sign, storedSign, state, signal1, countS, lasthigh, CodedMorse_sign;
int32_t S, Sf, oldsmax, oldsmin, S_avg;
float diff, norm_sig, norm_noise;
char sperreLow, sperreHigh;
int32_t filter[3];
int32_t last10[TestLength];
uint8_t index10;
int32_t max10, min10;
uint8_t j = 0;

//Codetabelle Morsezeichen:
const uint8_t codetab[60] = {5, 0x18, 0x1a, 0x0c, 2, 0x12, 0x0e, 0x10, 4, 0x17, 0x0d,
							 0x14, 7, 6, 0x0f, 0x16, 0x1d, 0x0a, 8, 3, 9, 0x11, 0x0b, 0x19, 0x1b,
							 0x1c, 0x3f, 0x2f, 0x27, 0x23, 0x21, 0x20, 0x30, 0x38, 0x3c, 0x3e,
							 0x15, 0x1e, 0x13, 0x8c, 0x1f, 0x55, 0x73, 0x78, 0x6a, 0x4c, 0x61, 0x4d, 0x36,
							 0x6d, 0x5e, 0x31, 0x2a, 0x32, 0x5a, 0x35, 0x22, 0x45, 0x80, 0};
//ASCII- Tabelle -falls nichts gefunden wurde: * ausgeben
const char chartab[60] = {'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J', 'K', 'L',
						  'M', 'N', 'O', 'P', 'Q', 'R', 'S', 'T', 'U', 'V', 'W', 'X', 'Y', 'Z',
						  '0', '1', '2', '3', '4', '5', '6', '7', '8', '9',
						  'a', 'o', 'u', 's', 'c', '.', ',', ':', ';', '?', '-', '_', '(', //�,�,�,�,CH
						  ')', 0x27, '=', '+', '/', '@', 'b', 'v', 'k', 'i', '*'};		   //),',=,+,/,@,KA,VE,SK,ii

float log2x(int32_t x)
{ // Logarithm Base 2
	uint32_t y;
	int8_t z;

	if (x <= 0)
		return -1;
	y = x;
	z = 4;
	while (y < 16)
	{
		z--;
		y = y << 1;
	}
	for (; y > 16; z++)
	{
		(y = y >> 1);
	}
	return z + log2c[(uint8_t)y];
}

float moving_average(float average, float input, int weight)
{
	return ((input - average) / weight + average);
}

uint32_t usec_diff(uint32_t earlier, uint32_t later)
{
	return (earlier >= later) ? (uint32_t)(later + (MAXINT - earlier)) : (later - earlier);
}

//***** ideas and parts from program fldigi: *****

//=======================================================================

void CW_TIMER_EVENT(void) //call period:	2 ms
{
	++smpl_ctr;
	curr_timestamp = smpl_ctr;
	// this should be called quite often (faster than inter-character gap) It looks after timing
	// key up intervals and determining when a character, a word space, or an error char '*' should be returned.
	// CW_SUCCESS is returned when there is a printable character. Nothing to do if we are in a tone

	if (cw_receive_state == RS_IN_TONE)
	{
		duration = usec_diff(cw_keydown_timestamp, curr_timestamp);
		if (duration > MAXTONE)
		{ // permanent tone ?
			cw_receive_state = RS_IDLE;
			return;
		}
		if (duration > 4 * cw_dot_length)
		{ // permanent tone or slower speed
		}
	}
	else if (cw_receive_state == RS_AFTER_TONE)
	{
		// compute length of silence so far
		duration = usec_diff(cw_keyup_timestamp, curr_timestamp);
		// SHORT time since keyup... nothing to do yet
		color = 0xdf9d; // white
		if (duration < (2 * cw_dot_length))
			return;

		// MEDIUM time since keyup... check for character space
		// one shot through this code via receive state logic
		// FARNSWOTH MOD HERE -->
		if (duration > 8 * cw_dot_length)
		{ // a longer pause
			duration = usec_diff(cw_keyup_timestamp, curr_timestamp);
			cw_receive_state = RS_IDLE;
			CW_TIMING_STATE = CW_FIRST_TIME_ELEMENT;
			element_counter = 0;
			CodedMorse_sign = 1; // begin new Sign
			color = 0xdf9d;		 // white
			return;
		}
		if (duration >= 5 * cw_dot_length)
		{
			if (!space_sent)
			{
				decodedSign = ' ';
				ShowMorseSign();
				space_sent = true;
				return;
			}
			else
				return;
		}
		if (2 * duration >= 5 * cw_dot_length)
		{ //
			if ((CW_TIMING_STATE == CW_SYNCHRONIZED) && (element_counter != 0))
			{
				decodedSign = RevCode[CodedMorse_sign]; //////
				element_counter = 0;
				ShowMorseSign();
				space_sent = false;
				CodedMorse_sign = 1; // begin new Sign
				return;
			}
			else
				return; // all is done
		}
	}
	else if (duration > 8 * cw_dot_length)
	{
		duration = usec_diff(cw_keyup_timestamp, curr_timestamp);
		cw_receive_state = RS_IDLE;
		CW_TIMING_STATE = CW_FIRST_TIME_ELEMENT;
		element_counter = 0;
		CodedMorse_sign = 1; // begin new Sign
		color = 0xdf9d;		 // white
	}
}

void CW_KEYDOWN_EVENT(void)
{
	curr_timestamp = smpl_ctr;
	color = 0x1863; //black
		// A receive tone start can only happen while we
		// are idle, not inside of a character.
	if ((cw_receive_state != RS_IN_TONE))
	{

		CW_pauseTime = usec_diff(cw_keyup_timestamp, curr_timestamp);
		if (CW_pauseTime <= cw_noise_spike_threshold)
		{ //negative spike
			//cw_receive_state = RS_SPIKE_MINUS;
			return;
		}

		// save the timestamp
		cw_keydown_timestamp = curr_timestamp;
		// Set state to indicate we are inside a tone.
		cw_receive_state = RS_IN_TONE;
	}
}

void CW_KEYUP_EVENT(void)
{
	int Timecalc;
	curr_timestamp = smpl_ctr; // Save the current timestamp
	color = 0xdf9d;			   // white
							   // The receive state is expected to be inside a tone.
	if (cw_receive_state != RS_IN_TONE)
		return;
	duration = usec_diff(cw_keydown_timestamp, curr_timestamp);
	// If the tone length is shorter than any noise cancelling
	if (duration <= cw_noise_spike_threshold)
	{ // positive spike
		//cw_receive_state = RS_SPIKE_PLUS; // no reaction
		return;
	}
	cw_keyup_timestamp = curr_timestamp; // no spike: store timestamp

	if (duration > MAXTONE)
	{ // 2 seconds
		//cw_receive_state = RS_PERMANENT_TONE;
		//CW_TIMING_STATE=CW_FIRST_TIME_ELEMENT;
		return; //signal is very long
	}
	cw_receive_state = RS_AFTER_TONE;
	if (CW_TIMING_STATE != CW_SYNCHRONIZED)
	{ // We are searching for two different times
		if (CW_TIMING_STATE == CW_FIRST_TIME_ELEMENT)
		{
			length1 = duration;
			CW_TIMING_STATE = CW_2ND_TIME_ELEMENT;
			return;
		}
		if (CW_TIMING_STATE == CW_2ND_TIME_ELEMENT)
		{
			if (duration > 2 * length1)
			{ //dit-dah
				Timecalc = (length1 + duration) / 2;
				if (Timecalc <= 2 * cw_noise_spike_threshold)
					return;
				two_dots = Timecalc;
				cw_dot_length = Timecalc / 2;
				CodedMorse_sign = 5;
			}
			else if (duration < length1 / 2)
			{ //dah-dit
				Timecalc = (length1 + duration) / 2;
				if (Timecalc <= 2 * cw_noise_spike_threshold)
					return;
				two_dots = Timecalc;
				cw_dot_length = Timecalc / 2;
				CodedMorse_sign = 6;
			}
			else if ((3 * duration < 4 * length1) && (4 * duration > 3 * length1))
			{ // same length
				if (2 * CW_pauseTime < duration)
				{ //dah-dah
					Timecalc = (length1 + duration) / 3;
					if (Timecalc <= 2 * cw_noise_spike_threshold)
						return;
					two_dots = Timecalc;
					cw_dot_length = Timecalc / 2;
					CodedMorse_sign = 7;
				}
				else
				{ //dit-dit
					Timecalc = (length1 + duration);
					if (Timecalc <= 2 * cw_noise_spike_threshold)
						return;
					two_dots = Timecalc;
					cw_dot_length = Timecalc / 2;
					CodedMorse_sign = 4;
				}
			}
			element_counter = 2;
			CW_TIMING_STATE = CW_SYNCHRONIZED;
		}
		return;
	}
	if (element_counter >= 7)
	{ // test,if more than 8 elements
		if ((duration < two_dots) && (CodedMorse_sign == 0x80))
		{ //ii correct Irrung
			decodedSign = 'j';
			ShowMorseSign();
			CodedMorse_sign = 1;
			element_counter = 0;
			return;
		}
		else
		{
			CW_TIMING_STATE = CW_FIRST_TIME_ELEMENT;
			cw_receive_state = RS_IDLE;
			CodedMorse_sign = 1;
			element_counter = 0;
			return;
		}
	}
	element_counter++;
	// ok... do we have a dit or a dah?
	// a dot is anything shorter than 2 dot times
	CodedMorse_sign = (CodedMorse_sign << 1);
	// Dot = 0  code: 'A' -> CodedMorse_sign=0101b 'B' ->011000b
	if (duration > two_dots)
	{						  // a dash is anything longer than 2 dot times
		CodedMorse_sign += 1; // Dash = 1
	}
	if (prev_duration > 0)
	{
		// check for dot dash sequence (current should be 3 x last)
		if ((duration > 2 * prev_duration) &&
			(duration < 4 * prev_duration))
		{
			two_dots = (prev_duration + duration) / 2;
			cw_dot_length = two_dots / 2;
		}
		// check for dash dot sequence (last should be 3 x current)
		if ((prev_duration > 2 * duration) &&
			(prev_duration < 4 * duration))
		{
			two_dots = (prev_duration + duration) / 2;
			cw_dot_length = two_dots / 2;
		}
	}
	prev_duration = duration;
	// All is well.  Move to the more normal after-tone state.
}

void CW_RESET(void)
{
	cw_receive_state = RS_IDLE;
	CW_TIMING_STATE = CW_FIRST_TIME_ELEMENT;
	length1 = length2 = 0;
	cw_decoder_state = DEC_EMPTY;
	element_counter = 0;
	prev_duration = 0;
	CW_pauseTime = 0;
	cw_noise_spike_threshold = 9; //  (speed)
	space_sent = false;
	cw_dot_length = 30;
	two_dots = 60;
	smpl_ctr = 0; // reset audio sample counter
	CodedMorse_sign = 1;
	positx = 10;
	posity = 62;
	drl = drk = 0;
	CW_KEYUP_EVENT(); //to avoid permanent "signal"
}

void MakeRevMorseCode()
{ // fill the RevCode table
	uint16_t i;
	for (i = 0; i <= 255; i++)
	{
		RevCode[i] = '*'; // unbekannte Zeichen: *
	}
	for (i = 0; i < 60; i++)
	{
		RevCode[codetab[i]] = chartab[i];
	}
}

void MakeMorseCode()
{ // fill the MCode table
	uint16_t i;
	for (i = 0; i <= 255; i++)
	{
		MCode[i] = '*'; // unbekannte Zeichen: *
	}
	for (i = 0; i < 60; i++)
	{
		MCode[(uint8_t)chartab[i]] = codetab[i];
	}
}
int8_t CodeMorse(uint8_t zeichen)
{ // coding 1 char from keyboard or text memory F1.. F4
	uint8_t code, overflow = 0;
	if (zeichen != ' ')
	{
		code = MCode[zeichen];
		i = 0;
		while (code < 0x80)
		{
			code = code + code;
			i++;
		}
		code = code - 0x80; // leading 1
		while ((i < 7) && (overflow == 0))
		{
			code = code + code;
			if (code >= 0x80)
			{ // 1 --> '-'
				code -= 0x80;
				PunktStrich[PSinPointer++] = '-';
				if (PSinPointer == PSoutPointer)
					overflow = 1;
				else if (PSinPointer >= 456)
					PSinPointer = 0;
			}
			else
				PunktStrich[PSinPointer++] = '.'; //0 --> '.'
			if (PSinPointer == PSoutPointer)
				overflow = 1;
			else if (PSinPointer >= 456)
				PSinPointer = 0;
			i++;
		}
		if (overflow == 1)
			return -1;
	}
	PunktStrich[PSinPointer++] = ' '; // pause
	if (PSinPointer == PSoutPointer)
		overflow = 1;
	else if (PSinPointer >= 456)
		PSinPointer = 0;
	if (overflow == 1)
		return -1;
	return 0;
}

char DecodeMorse()
{ // decoding 1 character/number from key input
	uint8_t i = 1, k;
	char val;
	do
	{
		val = PunktStrich[PSoutPointer++];
		if (PSoutPointer >= 456)
			PSoutPointer = 0; // ring buffer
	} while (val == ' ');
	if (val == '.')
		i = 2;
	else if (val == '-')
		i = 3;
	for (k = 7; k >= 1; k--)
	{
		val = PunktStrich[PSoutPointer++];
		if (PSoutPointer >= 456)
			PSoutPointer = 0; // ring buffer
		if (val == ' ')
			return RevCode[i];
		i *= 2;
		if (val == '-')
			i++;
	}
	return RevCode[i];
}

void SendMorsecode(void)
{

	if (PSinPointer != PSoutPointer)
	{
		if (gestartet == 0)
		{
			timer2 = 0;
			gestartet = 1;
			zeichen = PunktStrich[PSoutPointer++];
			if (PSoutPointer >= 456)
				PSoutPointer = 0;
			if ((zeichen == '.') || (zeichen == '-'))
				MorseKey(true);
		}
	}
	if (gestartet == 1)
	{
		if (zeichen == '.')
		{
			if (timer2 >= DitLength)
			{
				MorseKey(false); // Dit Ende
				timer2 = 0;
				gestartet = 2;
			}
		}
		else if (zeichen == '-')
		{
			if (timer2 >= 3 * DitLength)
			{
				MorseKey(false); // Daa Ende
				timer2 = 0;
				gestartet = 2;
			}
		}
		else
		{
			if (timer2 >= 3 * DitLength)
			{ //pause
				timer2 = 0;
				gestartet = 0;
			}
		}
	}
	else if (gestartet == 2)
	{
		if (timer2 >= DitLength)
		{
			timer2 = 0;
			gestartet = 0;
		}
	}
}

/*
void Zeichen(){//Zeichen decodieren und anzeigen
if(mchar!=1){
found=0;
	for (y=0;(y<60)&&(found==0) ;y++){// Zeichencode suchen
		if(codetab[y]==mchar) found=1;
	}
	cchar=chartab[y-1];// umcodieren
	if(pos<20)
		LCD_Display_Position(2, pos);//zum Testen
	else
		LCD_Display_Position(3, (pos-20));
	LCD_Display_PutChar(cchar);// Anzeige
	displ[pos]=cchar;// speichern
	pos++;
	roll();
	switch(mchar){
		case 0x8c:	{// Sonderbehandlung:
			CharOut('S');// SS statt �
			break;
		}
		case 0x1f:	{
			CharOut('H');// CH
			break;
			}
		case 0x22:	{
			CharOut('E');// VE
			break;
			}
		case 0x45:	{
			CharOut('K');// SK
			break;
			}
		case 0x80:	{
			CharOut('i');// ii Irrung
			break;
			}
		default:{}
	}
	LCD_Display_PutChar(' ');
	mchar=1;//neuer Startwert
	ReInit();
	}
}
*/
