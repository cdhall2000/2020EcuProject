/*

Project Functions

1. The first menu option should cause the vibration motor to vibrate with an intensity that
continuously varies up and down from a minimal level, to the maximum achievable based on the USB
supply voltage, and then back down again. One cycle from the minimum intensity, to the maximum,
and back down to the minimum again should take about 10s. This varying intensity vibration should
continue until any key is pressed on the keypad, at which point the vibration should immediately stop
and the LCD display should return to the main menu.

2. The second menu option should allow the 1 – 9 keys on the keypad to be used to generate
pulses with a range of intensities and durations. Pressing keys in the first column should generate 1
second duration pulses, keys in the second column should generate 1.5 second duration pulses, and
keys in the third column should generate 2 second duration pulses. Keys in the first row should
generate low intensity pulses, keys in the second row should generate medium intensity pulses, and
keys in the third row should generate high intensity pulses. For example, pressing the ‘1’ key should
generate a low intensity vibration for 1s, pressing the ‘5’ key should generate a medium intensity pulse
for 1.5s, and pressing the ‘9’ key should generate a high intensity pulse for 2s. If the ‘#’ key is pressed
at any point, then the LCD display should return to showing the main menu.

3. Selecting the third menu option should enable a sequence of pulses of varying intensities
and/or durations to be played back when the ‘1’, ‘2’ or ‘3’ keys are pressed, with a different sequence
being generated for each of the keys. The ‘1’ key should generate a 3 pulse sequence, the ‘2’ key a 4
pulse sequence, and the ‘3’ key a 5 pulse sequence. If the ‘#’ key is pressed at any point, then the LCD
display should once again return to showing the main menu. The sequences can be pre-defined by you.
Hint: You may want to create an array data structure to store the pre-defined pulse sequences.
Even though the ‘user interface’ is limited to the LCD screen and keypad, marks will be given for making
this as ‘user-friendly’ and intuitive as possible.
Note: You can use your solution to Assignment 1 as the basis (design) for this task, appropriately
modified for the supplied components.

4. This function will enable the user to record their own custom pulse sequence and then
play it back. When this menu option is selected, the user should be presented with a new sub-menu
with two options. The first option should enable the user to record a sequence of three pulses in the
following way. The ‘1’, ‘2’ and ‘3’ keys, when pressed should cause the vibration motor to vibrate at a
low, medium or high intensity respectively. The user should be able to make three key presses in turn
of any of the keys and for any desired duration. The system should record what the three keypresses
were and the duration of each (including the duration of the pauses between them), and the second
sub-menu option should cause the sequence to be played back identically to how it was recorded.

Motor Specs
Nominal Voltage: 3V
Operating Voltages: 1.5V - 4.5V
OCR0A Limits: 77-229

Keypad Specs
Keys required: 3 colombs and 4 rows

*/


#include <Arduino.h>
#include <stdio.h>
#include <math.h>

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <inttypes.h>

#include "delay.h"
#include "lcd.h"
#include "uart.h"

#include "main.h"



//DEFINES
#define COL  5


//#define F_CPU 16000000L




//Global Variables
static volatile uint8_t Timer2InteruptCounter = 0; //Counts how many times the Timer2 has overflowed
static volatile int Timer2MilliCounter = 0; //Counts the number of milliseconds the timer has been counting for
static volatile int Timer2MilliCompare = 0; //A value to compare to Timer2SecondsCounter to see if the desired time has passed

static volatile uint8_t Timer2TicksToMilli = 0;

static uint8_t SequenceEnabled = 0;
static volatile uint8_t SequencePosition = 0;

static volatile uint8_t seqIndex = 0;
static uint8_t seqLength = 1;

static unsigned long firstTime = 0;
static unsigned long prevTime = 0;
static unsigned long newTime = 0;

uint8_t Sequence[2][COL] = {
  {50, 100, 50, 200, 250}, //Intensity out of 255
  {1, 2, 1, 2, 3} //Length of Time in Seconds
};

int predefSequence1[2][5] = {
  {100, 0, 100, 0, 200}, //Intensity out of 255
  {5000, 3000, 5000, 3000, 10000} //Length of Time in Milliseconds
};
int predefSequence2[2][7] = {
  {100, 0, 133, 0, 166, 0, 200}, //Intensity out of 255
  {400, 200, 500, 300, 600, 400, 800} //Length of Time in Milliseconds
};
int predefSequence3[2][9] = {
  {90, 0, 150, 0, 100, 0, 170, 0, 220}, //Intensity out of 255
  {500, 100, 400, 100, 400, 100, 800, 500, 1000} //Length of Time in Milliseconds
};

//int * sequenceLocations[] = { &predefSequence1[0][0], &predefSequence2[0][0], &predefSequence3[0][0] };


void playSequence(uint8_t sequenceIndex);
void callout();

//General Initialisation Function
void initGeneral() {

#if MPU_ARDUINO == 1
	Serial.begin(57600);
	delay_ms(50);
	Serial.println("initGeneral Complete");
#endif

}

void initOutputPins() {
	//Motor Driving Pin
	DDRD |= (1 << PD6); //Turn PD6/OC0A into an output //TODO change lcd enable pin to something other than PD6

	//Keypad Pins
	// PC0 -- KEYPAD COL1 (left column)
	// PC1 -- KEYPAD COL2 (centre column)
	// PC2 -- KEYPAD COL3 (right column)
	// PC3 -- KEYPAD ROW1 (top row)
	// PC4 -- KEYPAD ROW2 (second row)
	// PC5 -- KEYPAD ROW3 (third row)
	// PB5 -- KEYPAD ROW4 (fourth row)

	// Set the columns as ouput
	DDRC |= (1 << PC2) | (1 << PC1) | (1 << PC0);   // Set Port C pins 0, 1 and 2 as outputs (columns)

	// Set row pins to input mode  
	DDRC &= ~(1 << PC3) & ~(1 << PC4) & ~(1 << PC5); // Set PC3, PC4 and PC5 as inputs
	//DDRB &= ~(1<<PB5);  // Set PB5 as input
	DDRB |= (1 << PB5);

	// Turn on the internal resistors for the input pins
	PORTC |= (1 << PC3) | (1 << PC4) | (1 << PC5); // Turn on internal pull up resistors for PC3, PC4 and PC5
	//PORTB |= (1<<PB5);
	// Initialise the column output pins low, so input low if contact made
	PORTC &= ~(1 << PC2) & ~(1 << PC1) & ~(1 << PC0); // set PC0, PC1 and PC2 low

	Serial.println("initOutputs Complete");

}

void initTimers() {
	Serial.println("initTimers Started");
	TCCR0A = 0b10000011; //Sets OC0A to clear on compare match and set at BOTTOM (Non inverting PWM Mode)
	TCCR0B = 0b00000000; //Sets the prescaler to 8 which will give us an output PWM frequency of around 7000Hz 
	TCCR0B |= TIMER0_PRESCALER;

	TCCR2A = 0b00000000; //Sets the timer to normal operation. OC2A disconnected 
	TCCR2B = 0b00000001; //Sets the prescaler to 1
	TCCR2B |= TIMER2_PRESCALER;

	TIMSK2 &= ~(1);//&= ~(1); //This turns the timer off
	TIMSK0 &= ~(1 << 1);//&= ~(1 << 1); //This turns the PWM wave off

	disableBuzz(); //Just to make sure the motor doesn't sputter when the MPU starts up

	//OCR0A = 100;
	//OCR2A = 100;

	Timer2TicksToMilli = (MPU_CPU / 256) / 1000;

	Timer2TicksToMilli = 63;


	Serial.print("initTimers() Complete : "); Serial.println(Timer2TicksToMilli);
}

//Interupt Services
ISR(TIMER2_OVF_vect) {

	Timer2InteruptCounter++; //Increment every 256 clock ticks
	if (Timer2InteruptCounter >= 63) { //After Timer2TicksToMilli matches increments one millisecond should have passed
//256*57=14592 ie. roughly 1/1000th of our clock frequency
		Timer2MilliCounter++; //Increment the seconds passed counter
		Timer2InteruptCounter = 0; //Reset Timer2InteruptCounter
		//PINB |= (1<<PB5);
		if (Timer2MilliCounter >= Timer2MilliCompare) { //When our timer counts to the amount of time we want.
//callout(SequencePosition);
  //PINB |= (1<<PB5);
			callout();
			TIMSK0 &= ~(1 << 1);//&= ~(1 << 1); //This turns the PWM wave off
			//TIMSK0 &= ~(1);//&= ~(1 << 1); //This turns the PWM wave off
			TIMSK2 &= ~(1);//&= ~(1); //This turns the timer off
			Timer2InteruptCounter = 0; //Reset for the next time
			Timer2MilliCounter = 0; //Reset for the next time
			Timer2MilliCompare = 0; //Reset for the next time

			if (SequenceEnabled != 0) { //If we are playing a sequence
				SequencePosition++; //Increment our position in the 2d Sequence array
				if (SequencePosition < seqLength) {//If we are not at the end of the sequence
					playSequence((uint8_t)0); //feedback into the original function
				} else { //If we have reached the end of the function
					SequenceEnabled = 0; //Clear the sequence flag
					//disableBuzz();
				}
			} else {
				//disableBuzz();
			}
		}
	}
}

EMPTY_INTERRUPT(TIMER0_COMPA_vect);


void callout() {
	newTime = millis() / 8;
	unsigned long totalDiff = (newTime - firstTime);
	unsigned long pulseDiff = (newTime - prevTime);

	Serial.print(SequencePosition + 1); Serial.println("-------------------------------------");
	Serial.print("OCR0A = "); Serial.print(OCR0A); Serial.print(" : Time = "); Serial.println(Timer2MilliCompare);
	Serial.print("Total Time Passed = "); Serial.print(totalDiff); Serial.print(" : Pulse Time Passed = "); Serial.println(pulseDiff);

	//testDynamicBuzzState(500);
	delay_ms(500);
	uint8_t bState = dynamicBuzzState();
	if (bState == 1) {
		Serial.println("ON --");
	} else if (bState == 0) {
		Serial.println("OFF --");
	} else {
		Serial.print("BROKEN -- bState="); Serial.println(bState, HEX);
	}

	prevTime = newTime;
}

void staticBuzz(uint8_t intensity, int time) {
	//uint8_t means an 8-bit integer, it is good to specify that as OCR0A is an 8-bit register and errors would occur placing an normal int value as its actually a 32-bit value

	/*
	We have to assume that
	DDRD |= (1 << PD6); //Turn PD6/OC0A into an output
	TCCR0A |= 0b10000011; //Sets OC0A to clear on compare match and set at BOTTOM (Non inverting PWM Mode)
	TCCR0B |= 0b00000010; //Sets the prescaler to 8 which will give us an output PWM frequency of around 7000Hz
	TCCR2A |= 0b00000000; //Sets the timer to normal operation. OC2A disconnected
	TCCR2B |= 0b00000110; //Sets the prescaler to 1
	*/
	cli();

	OCR0A = intensity; //So the duty cycle of the PWM is determined by intensity from 0 (being a 0% duty cycle wave) to 255 (being a 100% duty cycle wave) 
			   //so say an intesity of 50 would give us a ((50/255)*100 = 19.61%) duty cycle wave.

	Timer2MilliCompare = time; //Set our desired amount of time that we want the motor running.

	//Housekeeping just incase motor() is called again before finishing the first time.
	Timer2InteruptCounter = 0;
	Timer2MilliCounter = 0;

	sei();

	TCCR2A |= 0b00000000; //Sets the timer to normal operation. OC2A disconnected 
	//TCCR2A &= ~(0xFF);
	TCCR2B |= 0b00000001; //Sets the prescaler to 1
	//TCCR2B &= ~(0xFF);
	//TCCR2B |= (1);

	TIMSK0 |= (1 << 1); //This turns the PWM wave on
	TIMSK2 |= (1); //This turns the timer on

	//The function will turn itself off after the desired amount of time
}

uint8_t dynamicBuzzState() { //Returns either 1 if TIMER0_COMPA is enabled or 0 if TIMER0_COMPA is disabled.

	uint8_t buzzState;

	buzzState = TIMSK0;

	buzzState &= (1 << 1);

	buzzState = (buzzState >> 1);

	return buzzState;
}

uint8_t isBuzzConnected() { if ((TCCR0A >> 6) != 0) { return 1; } else { return 0; } }

void disableBuzz() { TCCR0A &= ~(1<<7); TCCR0A &= ~(1<<6); }

void enableBuzz() { TCCR0A |= (1<<7); } 

void dynamicBuzzOn(uint8_t intesity) {
	OCR0A = intesity;
	enableBuzz();
	TIMSK0 |= (1 << 1); //This turns the PWM wave on
}

void dynamicBuzzOff() {
	disableBuzz();
	TIMSK0 &= ~(1 << 1);//&= ~(1 << 1); //This turns the PWM wave off
}


uint8_t dynamicBuzzToggle() { uint8_t buzzState; return buzzState; }

void playSequence(uint8_t sequenceIndex) {
	char oldOCR0A = OCR0A;

	if (SequenceEnabled == 0) { //If the playSequence() function is being called for the first time

		TIMSK2 &= ~(1);      //This turns the timer off
		TIMSK0 &= ~(1 << 1); //This turns the PWM wave off

		//Housekeeping to ensure the timer works properly
		Timer2InteruptCounter = 0;
		Timer2MilliCounter = 0;

		SequenceEnabled = 1; //Set the sequence is playing flag
		seqIndex = sequenceIndex;

		//prevTime = millis();

		switch (seqIndex) {
		case 1:
			OCR0A = predefSequence1[0][0];              //Set the motor intensity
			Timer2MilliCompare = predefSequence1[1][0]; //Set the motor on time
			seqLength = (sizeof(predefSequence1) / sizeof(predefSequence1[0][0])) / 2;
			break;
		case 2:
			OCR0A = predefSequence2[0][0];              //Set the motor intensity
			Timer2MilliCompare = predefSequence2[1][0]; //Set the motor on time
			seqLength = (sizeof(predefSequence2) / sizeof(predefSequence2[0][0])) / 2;
			break;
		case 3:
			OCR0A = predefSequence3[0][0];              //Set the motor intensity
			Timer2MilliCompare = predefSequence3[1][0]; //Set the motor on time
			seqLength = (sizeof(predefSequence3) / sizeof(predefSequence3[0][0])) / 2;
			break;
		case 4:
			OCR0A = predefSequence1[0][0];              //Set the motor intensity
			Timer2MilliCompare = predefSequence1[1][0]; //Set the motor on time
			break;
		case 5:
			OCR0A = predefSequence1[0][0];              //Set the motor intensity
			Timer2MilliCompare = predefSequence1[1][0]; //Set the motor on time
			break;
		case 6:
			OCR0A = predefSequence1[0][0];              //Set the motor intensity
			Timer2MilliCompare = predefSequence1[1][0]; //Set the motor on time
			break;
			// default:
			//   return;
			//   break;
		}

#if MPU_ARDUINO == 1
		Serial.print("Sequence Number: "); Serial.print(seqIndex); Serial.print(" Sequence Length: "); Serial.println(seqLength);
		firstTime = millis() / 8;
		prevTime = millis() / 8;
#endif

		TIMSK0 |= 0b00000010; //This turns the PWM wave on
		TIMSK2 |= 0b00000001; //This turns the timer on
	} else { //If the playSequence() function is being called by the TIMER2_OVF_vect interrupt

//Serial.println("ELSE WAS CALLED");

		switch (seqIndex) {
		case 1:
			OCR0A = predefSequence1[0][SequencePosition];              //Set the motor intensity
			Timer2MilliCompare = predefSequence1[1][SequencePosition]; //Set the motor on time
			//seqLength = (sizeof(predefSequence1)/sizeof(predefSequence1[0][0]))/2;
			break;
		case 2:
			OCR0A = predefSequence2[0][SequencePosition];              //Set the motor intensity
			Timer2MilliCompare = predefSequence2[1][SequencePosition]; //Set the motor on time
			//seqLength = (sizeof(predefSequence2)/sizeof(predefSequence2[0][0]))/2;
			break;
		case 3:
			OCR0A = predefSequence3[0][SequencePosition];              //Set the motor intensity
			Timer2MilliCompare = predefSequence3[1][SequencePosition]; //Set the motor on time
			//seqLength = (sizeof(predefSequence3)/sizeof(predefSequence3[0][0]))/2;
			break;
		case 4:
			OCR0A = predefSequence1[0][SequencePosition];              //Set the motor intensity
			Timer2MilliCompare = predefSequence1[1][SequencePosition]; //Set the motor on time
			break;
		case 5:
			OCR0A = predefSequence1[0][SequencePosition];              //Set the motor intensity
			Timer2MilliCompare = predefSequence1[1][SequencePosition]; //Set the motor on time
			break;
		case 6:
			OCR0A = predefSequence1[0][SequencePosition];              //Set the motor intensity
			Timer2MilliCompare = predefSequence1[1][SequencePosition]; //Set the motor on time
			break;
			// default:
			//   return;
			//   break;
		}
	}

	if (OCR0A == 0) {
		//OCR0A = oldOCR0A;
		//TIMSK0 |= 0b00000010; //Turn on function, commented out specifically NOT to turn it on
		disableBuzz();
		TIMSK2 |= 0b00000001; //This turns the timer on

	} else {
		enableBuzz();
		TIMSK0 |= 0b00000010; //This turns the PWM wave on
		TIMSK2 |= 0b00000001; //This turns the timer on

	}
}

uint8_t CheckKeypad() { //Borrowed From ENS2257 Lab 3

// Checks to see if any key has been pressed
// Sets all columns to low, and checks if any row goes low
// Returns a 1 (true) if a key has been pressed, 0 if not

// Set column output pins low, so input low if contact made

	uint8_t rowval;
	uint8_t kp;

	PORTC &= ~(1 << PC2) & ~(1 << PC1) & ~(1 << PC0); // Set PC0, PC1 and PC2 low
	//delay_us(10);  // Short delay to allow signals to settle

	// Read value from Pins on PortC shifted right 3 bits to get Pins 3, 4 and 5 in first 3 bits, with remaining bits set high
	rowval = ((PINC >> 3) | 0xF8);
	// Read value from PortB shifted right by 2 bits to get Pin 5 in bit 4, with remaining bits set high
	rowval &= ((PINB >> 2) | 0xF7);
	rowval &= 0x0F;  // Clear other bits

	kp = (rowval != 0x0F);              // if F, all high so no key has been pressed

	return kp;
}

char ReadKeypad(char lastchar) {
	// Identifies which key has been pressed on the keypad (assumes a key has been pressed)
	// Returns the key pressed as an ASCII number 0..9
	// Takes in lastchar as an input, and returns that if invalid read from keypad

	uint8_t rowval;
	char keych;

	keych = '$';		// Initialise to $ (default 'no key' value)

	PORTC &= ~(1 << PC0);            // Set PC0  low - to check column 1
	PORTC |= (1 << PC2) | (1 << PC1);  // Set  other two column pins high
	//delay_us(10);                   // Short delay to allow signals to settle

	// Read value from Pins on PortC shifted right 3 bits to get Pins 3, 4 and 5 in first 3 bits, with remaining bits set high
	rowval = ((PINC >> 3) | 0xF8);
	// Read value from PortB shifted right by 2 bits to get Pin 5 in bit 4, with remaining bits set high
	rowval &= ((PINB >> 2) | 0xF7);
	rowval &= 0x0F;   // Clear other bits

	switch (rowval) {
	case 0x0E: keych = '1';
		break;
	case 0x0D: keych = '4';
		break;
	case 0x0B: keych = '7';
		break;
	case 0x07: keych = '*';
		break;
	default: keych = '$';
		break;
	}

	if (keych == '$') {                     // If no valid keypress detected
		PORTC &= ~(1 << PC1);               // Set PC1 low - to check column 2
		PORTC |= (1 << PC2) | (1 << PC0);    // Set  other 2 high
		//delay_us(10);                     // Short delay to allow signals to settle

		// Read value from Pins on Port B and C
		rowval = ((PINC >> 3) | 0xF8);
		rowval &= ((PINB >> 2) | 0xF7);
		rowval &= 0x0F;   // Clear other bits

		switch (rowval) {
		case 0x0E: keych = '2';
			break;
		case 0x0D: keych = '5';
			break;
		case 0x0B: keych = '8';
			break;
		case 0x07: keych = '0';
			break;
		default: keych = '$';
			break;
		}
	}

	if (keych == '$') {                      // If still no valid keypress detected
		PORTC &= ~(1 << PC2);                // Set PC2 low - to check column 3
		PORTC |= (1 << PC1) | (1 << PC0);     // set  other 2 high
		//delay_us(10);                      // Short delay to allow signals to settle

		// Read value from Pins on Port B and C
		rowval = ((PINC >> 3) | 0xF8);
		rowval &= ((PINB >> 2) | 0xF7);
		rowval &= 0x0F;   // Clear other bits

		switch (rowval) {
		case 0x0E: keych = '3';
			break;
		case 0x0D: keych = '6';
			break;
		case 0x0B: keych = '9';
			break;
		case 0x07: keych = '#';
			break;
		default: keych = '$';
			break;
		}
	}

	if (keych != '$') {     // If valid keypress detected
		lastchar = keych;    // Update lastchar value
	}
	return lastchar;

}  // END keypad_read


void function1() {
	//PWM needs to sit between 30% and 90% to keep the motor functional

	//Ramp up - 5s
	//255*0.3=76.5 : 76.5 is the minimum intensity that we can set the motor to for it to turn
	//255*0.9=229.5 : 229.5 is the maximum intensity that we can set the motor to without risking burning it out
	//229.5-76.5=153 : steps between min intesity and max intensity
	//(5/153)*1000=32.67 : number of milliseconds each pulse needs to be so the whole ramp up takes 5 seconds

	while (1) {
		for (int i = 76; i <= 229; i++) { //ramp from min to max
			staticBuzz(i, 33);

			while (TIMSK2 != 0) {
				if (CheckKeypad()) {
					return;
				}
				//delay_ms(2);
			}
		}

		//Ramp down - 5s

		for (int i = 229; i >= 76; i--) { //ramp from max to min
			staticBuzz(i, 33);

			while (TIMSK2 != 0) {
				if (CheckKeypad()) {
					return;
				}
				//delay_ms(2);
			}
		}
	}
}

void function2() {
	char keyPressed = '$';

	uint8_t low = 90;
	uint8_t medium = 152;
	uint8_t high = 215;

	while (1) {

		if (CheckKeypad()) {
			keyPressed = ReadKeypad(keyPressed);

			if (keyPressed == '#') {
				return;
			} else if (TIMSK2 == 0) {
				switch (keyPressed) {
				case '1':
					staticBuzz(low, 1000);
					break;
				case '2':
					staticBuzz(low, 1500);
					break;
				case '3':
					staticBuzz(low, 2000);
					break;
				case '4':
					staticBuzz(medium, 1000);
					break;
				case '5':
					staticBuzz(medium, 1500);
					break;
				case '6':
					staticBuzz(medium, 2000);
					break;
				case '7':
					staticBuzz(high, 1000);
					break;
				case '8':
					staticBuzz(high, 1500);
					break;
				case '9':
					staticBuzz(high, 2000);
					break;

				}
			}
		}
	}
}

void function3() {



}

void function4() {
	Serial.println("Press 5 To Start Recording...");
	
	char function4Running = 1;
	unsigned long offTime;
	unsigned long onTime;
	

	#if ARDUINO_TESTBENCH == 1 //Arduino Test Bench

	pinMode(2, INPUT_PULLUP);
	pinMode(3, INPUT_PULLUP);
	pinMode(4, INPUT_PULLUP);
	pinMode(5, INPUT_PULLUP);

	#else //Using Keypad

	#endif

	while (function4Running){
		#if ARDUINO_TESTBENCH == 1 //Arduino Test Bench

		if (digitalRead(2) == LOW && !isBuzzConnected()) {
			//Run Motor LOW
			dynamicBuzzOn(BUZZER_LOW);
			onTime = millis()/8;
			Serial.print("Time since off "); Serial.println(onTime - offTime);
		}
		if (digitalRead(3) == LOW  && !isBuzzConnected()) {
			//Run Motor MED
			dynamicBuzzOn(BUZZER_MED);
			onTime = millis()/8;
			Serial.print("Time since off "); Serial.println(onTime - offTime);
		}
		if (digitalRead(4) == LOW  && !isBuzzConnected()) {
			//Run Motor HIGH
			dynamicBuzzOn(BUZZER_HIGH);
			onTime = millis()/8;
			Serial.print("Time since off "); Serial.println(onTime - offTime);
		}
		if (digitalRead(5) == LOW) {
			//Start Recording
		}

		if (digitalRead(2) == HIGH && digitalRead(3) == HIGH && digitalRead(4) == HIGH && isBuzzConnected()) { dynamicBuzzOff(); offTime = (millis()/8); }

		#else //Using Keypad


		#endif
	}
}



void setup() {
	// put your setup code here, to run once:
	char seqNum[1] = { 0 };


	initGeneral();
	delay_ms(500);
	PORTB |= (1 << PB5);
	initOutputPins();
	delay_ms(500);
	//PINB |= (1<<PB5);
	initTimers();
#if MPU_ARDUINO == 1
	Serial.print("TIMSK0: "); Serial.print(TIMSK0, BIN); Serial.print(" TIMSK2: "); Serial.println(TIMSK2, BIN);
	Serial.print("TCCR0A: "); Serial.print(TCCR0A, BIN); Serial.print(" TCCR0B: "); Serial.println(TCCR0B, BIN);
	Serial.print("TCCR2A: "); Serial.print(TCCR2A, BIN); Serial.print(" TCCR2B: "); Serial.println(TCCR2B, BIN);
	/* Serial.println("What sequence would you like to play? : ");




	while (seqNum[0] <= 0 || seqNum[0] >= 7) {
		while (Serial.available() == 0) {
			delay_ms(10);
		}
		Serial.readBytes(seqNum, 1);
		seqNum[0] -= 48;
		if (seqNum[0] <= 0 || seqNum[0] >= 7) {

			Serial.println();
			Serial.print(seqNum[0], DEC); Serial.println(" - is not a valid number, please enter 1-6 : ");
			seqNum[0] = 0;

		}

		Serial.flush();
	}

	//seqNum[0] += 1;
	playSequence(seqNum[0]);

	delay_ms(500);

	PORTB &= ~(1 << PB5); */
	//singleBuzz((uint8_t) 100, 1000);
	#endif

	Serial.println("Function 4 Called");

	function4();

	//delay(2000);
	//PINB |= (1<<PB5);
	//Serial.println("Sequence Played");
	//Serial.println(TIMSK0);
}

int num = 0;
void loop() {
	/* if (TIMSK2 == 0 && ((TCCR0A >> 6) != 0)) { //If Timer2 is not running and OC0A is not disconnected
		disableBuzz();
	}
	 */

	if (TIMSK2 == 0) {

		
		//Serial.print("Sequence Finished... "); Serial.println(num);
		num = num + 1;
		newTime = millis();

		//Serial.println(newTime - prevTime);

		PINB |= (1 << PB5);
		//Serial.println(dynamicBuzzState());
		delay_ms(1000);
	}
	//delay_ms(100);
	//PINB |= (1<<PB5);
}