# Bubble Display
Project 1 for MG7013 Embedded Systems  
Karl Crofskey, 2183664.

## Project Introduction

The aim of this project is to create an embedded system circuit utilising a seven segment 4 digit 'bubble' display. This is in order to explore how the 7-seg display interfaces with a microcontroller in displaying specific and multiple numeric characters.  

A stop watch application was chosen for this purpose. Two buttons, START/STOP and RESET, will be pushed by the user in order to control the stopwatch application. A tenth of a second was determined to be suitable timing resolution for this application and the required timing accuracy is a maximum of 100ms deviation from a comparison stopwatch, over 10 seconds (started at the same time). Timing will use the millis() arduino function.  

The 7-seg display is of common cathode type (grounds of each digit's LEDs connected) and power (N-channel, enhancement type) mosfets will be used in order to sink the total current of each digit to ground, instead of into a digital I/O pin – this is due to the limted current sinking / sourcing capability of the microcontroller.  

The buttons will be connected to external interrupts and will be debounced in order to prevent multiple button presses from being registered (due to contact switching).  

## Circuit Schematic
<img src="https://i.ibb.co/M6vz32c/schematic.jpg" alt="schematic" border="0">

* Teensy digital pin 11 and 12 are respectively connected to the START/STOP and RESET buttons. These are connected to an internal pull-up resistor so that the voltage is not floating while the button remains unpressed.
* Teensy digital pins 6; 14-20 are the 'anode' or 'LED segment' pins. These are connected to the individual LED segments within a digit, tied between the 4 digits. The specific combination of LED segment activations determines which character is displayed. 
* Teensy digital pins 7-10 are the 'common cathode' or 'digit activation' pins. These are connected to each digit's respective MOSFET gate. When set high, a conducting channel will form between the MOSFET's drain and source, allowing current to flow from that digit's common cathode to ground – lighting the LEDs.
* Current limiting resistors are connected in series with the LED segment anodes in order limit the current going through each LED to less than 5 mA.
* Current limiting resistors are further connected in series with the MOSFET gates. This is to limit the maximum current drawn as the MOSFET capacitor initially charges during switching.
* Pull down resistors could be included between the MOSFET gate and ground. This would tie the voltage to a known level (0V) during the initialization of the microcontroller, where the digital I/O pins are in a high impedance state and the MOSFET would hence act as an antenna. This was determined not to be necessary due to the low impact of current flow during this state (momentary flickering).

### _Current Limiting_
*QDSP6064 7-segment 4 digit bubble display [datasheet](https://cdn.sparkfun.com/datasheets/Components/LED/BB_QDSP_DS.pdf)*  
<img src="https://i.ibb.co/M5BWF7f/7segdata.jpg" alt="7segdata" border="0">

A current limiting resistor is required, for each LED segment, in order to limit the current to 5mA (to protect each from over-current damage).

Given the typical LED forward voltage drop of 1.6V, and using the Teensy 3.2 Vcc of 3.3V, Ohm's law was applied to find the minimum resistance required to achieve this.  

> R = V<sub>R</sub> / i  
> R = (3.3 - 1.6)V / 5mA  
> R = 340Ω  

Hence, a 390Ω resistor was used in series with each of the LED segments as the next highest available resistance.

*IRLU8743 N-channel enhancement type power MOSFET [datasheet](https://www.infineon.com/dgdl/irlr8743pbf.pdf?fileId=5546d462533600a4015356719c7e26ff)*  
<img src="https://i.ibb.co/QbzmJbC/mosfetdata.jpg" alt="mosfetdata" border="0">

A current limiting resistor is further needed in series with the pins connected the gate of the MOSFETS (used to switch the digits on and off). This is because, as the pin is initially turned HIGH, the charging capacitor of the MOSFET causes an inrush current to be drawn - not sufficiently limited by the 0.85Ω gate resistance - potentially damaging the Teensy microcontroller which has a specified current sourcing capability limit.

A 1KΩ resistor was used to limit the current to 3.3mA, a compromise between having low current and the impact on the MOSFET's switching frequency (increased RC time constant).


## Code Design
<img src="https://i.ibb.co/yRNFh1D/codediag.jpg" alt="codediag" border="0">

* The program will use 7seg library to interface with the bubble display. Initialization gives the library the correct digital output pin connected to each seven segment display pin.
* The count of the stopwatch timer is displayed. This is initialized to zero seconds, zero deciseconds.
* If the START button is pressed (START/STOP while paused), the stopwatch count starts increasing. If there was already a count value, the timer resumes from that point.
* If the STOP button is pressed (START/STOP while counting), the stopwatch count pauses at that current value.
* If the RESET button is pressed, the count resets back to the default count of zero (and stops counting if it was doing so).
* The program loops displaying an updated (either increased, or the same) count value.

## Breadboard Implementation
<img src="https://i.ibb.co/1sspBFW/20190508-151012-annotated.jpg" alt="20190508-151012-annotated" border="0">

Above is the physical breadboard circuit. The orange wires coming from the Teensy microcontroller are the digital outputs controlling digit activation. These are connected to the gate of the MOSFET. The white wires control each individual LED segment in a digit.

## Source Code
### modifying 7-seg library
<img src="https://i.ibb.co/fq2mF9J/7seglibmodify-crop.jpg" alt="7seglibmodify-crop" border="0">

The 7-seg library assumes current is sunk from the LED segments into a digital pin (one for each of the common cathodes).  
Hence, in order to turn a digit on, the output of the pin is set LOW.  

The common cathode pins in this project are instead connected to the gate of a power MOSFET (as per the circuit schematic).   
When the output of these pins are HIGH, a channel is formed between the drain and source of the MOSFET, allowing current to flow and the relevant LED segments to turn on.  

Hence, for a COMMON_CATHODE type 7 segment display, the state assigned to DigitOn and DigitOff in the SevSeg.cpp library require inversion:  

```JavaScript
  else //mode == COMMON_CATHODE
  {
    DigitOn = HIGH;
    DigitOff = LOW;
    SegOn = HIGH;
    SegOff = LOW;
  }
```

### Displaying LED digits

```JavaScript
 char tempString[10]; //Used for sprintf
 sprintf(tempString, "%04d", deciSecond); //Convert deciSecond into a string with leading zeros

 myDisplay.DisplayString(tempString, 0b100); //(numberToDisplay, decimal point location IN BINARY)
```
The library function DisplayString() is called to display the decisecond count to the seven segment display. It takes a character array (the alphanumeric characters to display) and a byte (the desired positions of the decimal point, from left) as parameters. 

The stopwatch count is stored in 'tempString' via [sprintf()](https://www.tutorialspoint.com/c_standard_library/c_function_sprintf.htm). Here %d denotes the count 'deciSecond' being of type 'int' (signed integer), 0 indicates the format of left-padding with zeroes, and 4 indicates the character width.

The function performs the following, looping for digits 1 to 4:
* Switch the relevant digit on, via digitalWrite(digit*, DigitOn)
* Display the specific character, by setting each relevant segment via digitalWrite(segment*, SegOn). CharacterArray[] in sevseg.h contains a mapping between characters and the relevant A-G segments.
* Display the decimal point
* Delay based on desired brightness
* Switch all segments off
* Switch the relevant digit off

### Stopwatch Implementation

```JavaScript
//stopwatch functionality  
  if (Reset_flag){
    Reset_flag = 0;//reset flags to default state
    Start_flag = 0;
    Stop_flag = 1;
    total_time_elapsed_ms = 0; //clear stopwatch time
  }
  if (Start_flag){ //display increasing time
    time_elapsed_ms = millis()-starttime_millis; //time of current run
    total_time_elapsed_ms = time_elapsed_ms + time_stopped_ms; //cumulative time before reset
    deciSecond = total_time_elapsed_ms / 100;
  }
  else{ //display stopped time
    time_stopped_ms = total_time_elapsed_ms; //save stop time value
    deciSecond = time_stopped_ms / 100;
  }
```
The stopwatch has two boolean flags, Start_Flag and Reset_Flag, which control the state of operation. These are set when the relevant button press triggers an external interrupt.


### Button Interrupts and Debouncing



## System Verification

## Video

## Appendix
### main.cpp
```JavaScript
/*
 March 6, 2014
 Spark Fun Electronics
 Nathan Seidle
 Updates by Joel Bartlett

 This code is originally based Dean Reading's Library deanreading@hotmail.com
 http://arduino.cc/playground/Main/SevenSegmentLibrary
 He didn't have a license on it so I hope he doesn't mind me making it public domain: 
 This code is public domain but you buy me a beer if you use this and we meet someday (Beerware license).

 This sketch provides a simple counter example for the HP Bubble display from SparkFun.
 https://www.sparkfun.com/products/12710

 Pinout for HP Bubble Display:
 1:  Cathode 1
 2:  Anode E
 3:  Anode C
 4:  Cathode 3
 5:  Anode dp
 6:  Cathode 4
 7:  Anode G
 8:  Anode D
 9:  Anode F
 10: Cathode 2
 11: Anode B
 12: Anode A
 */

#include <Arduino.h>
#include <SevSeg.h>

void ISR_StartStop();
void ISR_Reset();

//Create an instance of the object.
SevSeg myDisplay;

//Create global variables
uint32_t starttime_millis;
uint8_t startstop_buttonPin = digitalPinToInterrupt(11);
uint8_t reset_buttonPin = digitalPinToInterrupt(12);
const uint32_t DEBOUNCEDELAY_MS = 200;
bool Start_flag = 0;
bool Stop_flag = 1;
bool Reset_flag = 0;

void setup()
{
  //button pin declarations
  pinMode(startstop_buttonPin, INPUT_PULLUP);
  pinMode(reset_buttonPin, INPUT_PULLUP);
  attachInterrupt(startstop_buttonPin,ISR_StartStop,FALLING);
  attachInterrupt(reset_buttonPin,ISR_Reset,FALLING);

  int displayType = COMMON_CATHODE; //Your display is either common cathode or common anode
 
  //This pinout is for a bubble dispaly
   //Declare what pins are connected to the GND pins (cathodes)
   int digit1 = 7; //Pin 1 on BubbleDisplay
   int digit2 = 8; //Pin 10
   int digit3 = 9; //Pin 4
   int digit4 = 10; //Pin 6

   //Declare what pins are connected to the segments (anodes)
   int segA = 20; //Pin 12
   int segB = 19; //Pin 11
   int segC = 14; //Pin 3
   int segD = 17; //Pin 8
   int segE = 15; //Pin 2
   int segF = 18; //Pin 9
   int segG = 16; //Pin 7
   int segDP = 6; //Pin 5

  int numberOfDigits = 4; //1,2,or 4 digits

  myDisplay.Begin(displayType, numberOfDigits, digit1, digit2, digit3, digit4, segA, segB, segC, segD, segE, segF, segG, segDP);
  
  myDisplay.SetBrightness(100); //Set the display to 100% brightness level
}

void loop()
{
  static int deciSecond = 0;
  static uint32_t time_elapsed_ms = 0;
  static uint32_t time_stopped_ms = 0;
  static uint32_t total_time_elapsed_ms = 0;

  //Example ways of displaying a decimal number
  char tempString[10]; //Used for sprintf
  sprintf(tempString, "%04d", deciSecond); //Convert deciSecond into a string with leading zeros

  //Produce an output on the display
  myDisplay.DisplayString(tempString, 0b100); //(numberToDisplay, decimal point location IN BINARY)

  //stopwatch functionality  
  if (Reset_flag){
    Reset_flag = 0;//reset flags to default state
    Start_flag = 0;
    Stop_flag = 1;
    total_time_elapsed_ms = 0; //clear stopwatch time
  }
  if (Start_flag){ //display increasing time
    time_elapsed_ms = millis()-starttime_millis; //time of current run
    total_time_elapsed_ms = time_elapsed_ms + time_stopped_ms; //cumulative time before reset
    deciSecond = total_time_elapsed_ms / 100;
  }
  else{ //display stopped time
    time_stopped_ms = total_time_elapsed_ms; //save stop time value
    deciSecond = time_stopped_ms / 100;
  }

}

void ISR_StartStop(){
  static uint32_t lastSSButtonPress_ms = 0;
  if ((millis()-lastSSButtonPress_ms) >= DEBOUNCEDELAY_MS){ //debounce button
    lastSSButtonPress_ms = millis(); //update button press time
    Start_flag = !Start_flag; //toggle start stop flags
    Stop_flag = !Stop_flag;
    if(Start_flag){ //update starting reference point
      starttime_millis = millis();
    }
  }
}
void ISR_Reset(){
  static uint32_t lastRButtonPress_ms = 0;
  if ((millis()-lastRButtonPress_ms) >= DEBOUNCEDELAY_MS){  //debounce button
    lastRButtonPress_ms = millis();
    Reset_flag = 1; //set flag to reset time
  }
}
```
### sevseg.cpp
```JavaScript
/*
 This library allows an Arduino to easily display numbers and characters on a 4 digit 7-segment
 display without a separate 7-segment display controller.

 If you have feature suggestions or need support please use the github support page: https://github.com/sparkfun/SevSeg

 Original Library by Dean Reading (deanreading@hotmail.com: http://arduino.cc/playground/Main/SevenSegmentLibrary), 2012
 Improvements by Nathan Seidle, 2012

 Now works for any digital pin arrangement, common anode and common cathode displays.
 Added character support including letters A-F and many symbols.

 Hardware Setup: 4 digit 7 segment displays use 12 digital pins. You may need more pins if your display has colons or
 apostrophes.

 There are 4 digit pins and 8 segment pins. Digit pins are connected to the cathodes for common cathode displays, or anodes
 for common anode displays. 8 pins control the individual segments (seven segments plus the decimal point).

 Connect the four digit pins with four limiting resistors in series to any digital or analog pins. Connect the eight segment
 pins to any digital or analog pins (no limiting resistors needed). See the SevSeg example for more connection information.

 SparkFun has a large, 1" 7-segment display that has four digits.
 https://www.sparkfun.com/products/11408
 Looking at the display like this: 8.8.8.8. pin 1 is on the lower row, starting from the left.
 Pin 12 is the top row, upper left pin.

 Pinout:
 1: Segment E
 2: Segment D
 3: Segment DP
 4: Segment C
 5: Segment G
 6: Digit 4
 7: Segment B
 8: Digit 3
 9: Digit 2
 10: Segment F
 11: Segment A
 12: Digit 1


 Software:
 Call SevSeg.Begin in setup.
 The first argument (boolean) tells whether the display is common cathode (0) or common
 anode (1).
 The next four arguments (bytes) tell the library which arduino pins are connected to
 the digit pins of the seven segment display.  Put them in order from left to right.
 The next eight arguments (bytes) tell the library which arduino pins are connected to
 the segment pins of the seven segment display.  Put them in order a to g then the dp.

 In summary, Begin(type, digit pins 1-4, segment pins a-g, dp)

 The calling program must run the DisplayString() function repeatedly to get the number displayed.
 Any number between -999 and 9999 can be displayed.
 To move the decimal place one digit to the left, use '1' as the second
 argument. For example, if you wanted to display '3.141' you would call
 myDisplay.DisplayString("3141", 1);


 */

#include "SevSeg.h"

SevSeg::SevSeg()
{
  //Initial values
  DecAposColon = 0; //This variable tracks the decimal place, apostrophe, and colon (if the display has support)

}
void SevSeg::Begin(boolean mode_in, byte numOfDigits,
	byte dig1, byte dig2, byte dig3, byte dig4,
	byte digitCol, byte digitApos,
	byte segA, byte segB, byte segC, byte segD, byte segE, byte segF, byte segG,
	byte segDP,
	byte segCol, byte segApos)
{
  //Bring all the variables in from the caller
  numberOfDigits = numOfDigits;
  digit1 = dig1;
  digit2 = dig2;
  digit3 = dig3;
  digit4 = dig4;
  digitApostrophe = digitApos;
  digitColon = digitCol;
  segmentA = segA;
  segmentB = segB;
  segmentC = segC;
  segmentD = segD;
  segmentE = segE;
  segmentF = segF;
  segmentG = segG;
  segmentDP = segDP;
  segmentApostrophe = segApos;
  segmentColon = segCol;

  //Assign input values to variables
  //mode is what the digit pins must be set at for it to be turned on. 0 for common cathode, 1 for common anode
  mode = mode_in;
  if(mode == COMMON_ANODE)
  {
    DigitOn = HIGH;
    DigitOff = LOW;
    SegOn = LOW;
    SegOff = HIGH;
  }
  else
  {
    DigitOn = HIGH; //<-modified here
    DigitOff = LOW;
    SegOn = HIGH;
    SegOff = LOW;
  }

  DigitPins[0] = digit1;
  DigitPins[1] = digit2;
  DigitPins[2] = digit3;
  DigitPins[3] = digit4;
  SegmentPins[0] = segmentA;
  SegmentPins[1] = segmentB;
  SegmentPins[2] = segmentC;
  SegmentPins[3] = segmentD;
  SegmentPins[4] = segmentE;
  SegmentPins[5] = segmentF;
  SegmentPins[6] = segmentG;
  SegmentPins[7] = segmentDP;

  //Turn everything Off before setting pin as output
  //Set all digit pins off. Low for common anode, high for common cathode
  for (byte digit = 0 ; digit < numberOfDigits ; digit++)
  {
    digitalWrite(DigitPins[digit], DigitOff);
    pinMode(DigitPins[digit], OUTPUT);
  }
  //Set all segment pins off. High for common anode, low for common cathode
  for (byte seg = 0 ; seg < 8 ; seg++)
  {
    digitalWrite(SegmentPins[seg], SegOff);
    pinMode(SegmentPins[seg], OUTPUT);
  }

  if (digitColon != 255)
  {
	digitalWrite(digitColon, DigitOff);
	pinMode(digitColon, OUTPUT);
	digitalWrite(segmentColon, SegOff);
	pinMode(segmentColon, OUTPUT);
  }
  if (digitApostrophe != 255)
  {
	digitalWrite(digitApostrophe, DigitOff);
	pinMode(digitApostrophe, OUTPUT);
	digitalWrite(segmentApostrophe, SegOff);
	pinMode(segmentApostrophe, OUTPUT);
  }
}

//Begin
/*******************************************************************************************/
//Set pin modes and turns all displays off
//This second begin is used when the display does not support a colon and apostrophe
//The digitApostrophe, segmentApostrophe, and dig/segColon are set to 255 and the normal .Begin is called
void SevSeg::Begin(boolean mode_in, byte numOfDigits,
	byte dig1, byte dig2, byte dig3, byte dig4,
	byte segA, byte segB, byte segC, byte segD, byte segE, byte segF, byte segG,
	byte segDP)
{
  Begin(mode_in, numOfDigits, dig1, dig2, dig3, dig4, 255, 255, segA, segB, segC,
		segD, segE, segF, segG, segDP, 255, 255);
}

//Set the display brightness
/*******************************************************************************************/
//Given a value between 0 and 100 (0% and 100%), set the brightness variable on the display
//We need to error check and map the incoming value
void SevSeg::SetBrightness(byte percentBright)
{
	//Error check and scale brightnessLevel
	if(percentBright > 100) percentBright = 100;
	brightnessDelay = map(percentBright, 0, 100, 0, FRAMEPERIOD); //map brightnessDelay to 0 to the max which is framePeriod
}


//Refresh Display
/*******************************************************************************************/
//Given a string such as "-A32", we display -A32
//Each digit is displayed for ~2000us, and cycles through the 4 digits
//After running through the 4 numbers, the display is turned off
//Will turn the display on for a given amount of time - this helps control brightness
void SevSeg::DisplayString(const char* toDisplay, byte DecAposColon)
{
	//For the purpose of this code, digit = 1 is the left most digit, digit = 4 is the right most digit
	for(byte digit = 1 ; digit < (numberOfDigits+1) ; digit++)
	{
		switch(digit)
		{
			case 1:
				digitalWrite(digit1, DigitOn);
				break;
			case 2:
				digitalWrite(digit2, DigitOn);
				break;
			case 3:
				digitalWrite(digit3, DigitOn);
				break;
			case 4:
				digitalWrite(digit4, DigitOn);
				break;
			//This only currently works for 4 digits
		}

		//Here we access the array of segments
		//This could be cleaned up a bit but it works
		//displayCharacter(toDisplay[digit-1]); //Now display this digit
		// displayArray (defined in SevSeg.h) decides which segments are turned on for each number or symbol
		unsigned char characterToDisplay = toDisplay[digit-1];
		if (characterToDisplay & 0x80)	// bit 7 enables bit-per-segment control
		{	// Each bit of characterToDisplay turns on a single segment (from A-to-G)
			if (characterToDisplay & 0x01) digitalWrite(segmentA, SegOn);
			if (characterToDisplay & 0x02) digitalWrite(segmentB, SegOn);
			if (characterToDisplay & 0x04) digitalWrite(segmentC, SegOn);
			if (characterToDisplay & 0x08) digitalWrite(segmentD, SegOn);
			if (characterToDisplay & 0x10) digitalWrite(segmentE, SegOn);
			if (characterToDisplay & 0x20) digitalWrite(segmentF, SegOn);
			if (characterToDisplay & 0x40) digitalWrite(segmentG, SegOn);
		}
		else
		{
			const uint8_t chr = pgm_read_byte(&characterArray[characterToDisplay]);
			if (chr & (1<<6)) digitalWrite(segmentA, SegOn);
			if (chr & (1<<5)) digitalWrite(segmentB, SegOn);
			if (chr & (1<<4)) digitalWrite(segmentC, SegOn);
			if (chr & (1<<3)) digitalWrite(segmentD, SegOn);
			if (chr & (1<<2)) digitalWrite(segmentE, SegOn);
			if (chr & (1<<1)) digitalWrite(segmentF, SegOn);
			if (chr & (1<<0)) digitalWrite(segmentG, SegOn);
		}
		//Service the decimal point, apostrophe and colon
		if ((DecAposColon & (1<<(digit-1))) && (digit < 5)) //Test DecAposColon to see if we need to turn on a decimal point
			digitalWrite(segmentDP, SegOn);

		delayMicroseconds(brightnessDelay + 1); //Display this digit for a fraction of a second (between 1us and 5000us, 500-2000 is pretty good)
		//The + 1 is a bit of a hack but it removes the possible zero display (0 causes display to become bright and flickery)
		//If you set this too long, the display will start to flicker. Set it to 25000 for some fun.

		//Turn off all segments
		digitalWrite(segmentA, SegOff);
		digitalWrite(segmentB, SegOff);
		digitalWrite(segmentC, SegOff);
		digitalWrite(segmentD, SegOff);
		digitalWrite(segmentE, SegOff);
		digitalWrite(segmentF, SegOff);
		digitalWrite(segmentG, SegOff);
		digitalWrite(segmentDP, SegOff);

		//Turn off this digit
		switch(digit)
		{
			case 1:
			  digitalWrite(digit1, DigitOff);
			  break;
			case 2:
			  digitalWrite(digit2, DigitOff);
			  break;
			case 3:
			  digitalWrite(digit3, DigitOff);
			  break;
			case 4:
			  digitalWrite(digit4, DigitOff);
			  break;
			//This only currently works for 4 digits
		}
		// The display is on for microSeconds(brightnessLevel + 1), now turn off for the remainder of the framePeriod
		delayMicroseconds(FRAMEPERIOD - brightnessDelay + 1); //the +1 is a hack so that we can never have a delayMicroseconds(0), causes display to flicker
	}

	//After we've gone through the digits, we control the colon and apostrophe (if the display supports it)

	//Turn on the colon and/or apostrophe
	if ((digitColon != 255) || (digitApostrophe != 255))
	{
		if (DecAposColon & (1<<4)) //Test to see if we need to turn on the Colon
		{
			digitalWrite(digitColon, DigitOn);
			digitalWrite(segmentColon, SegOn);
		}
		if (DecAposColon & (1<<5)) //Test DecAposColon to see if we need to turn on Apostrophe
		{
			digitalWrite(digitApostrophe, DigitOn);
			digitalWrite(segmentApostrophe, SegOn);
		}
		delayMicroseconds(brightnessDelay + 1); //Display this digit for a fraction of a second (between 1us and 5000us, 500-2000 is pretty good)

		//Turn off the colon and/or apostrophe
		digitalWrite(digitColon, DigitOff);
		digitalWrite(segmentColon, SegOff);
		digitalWrite(digitApostrophe, DigitOff);
		digitalWrite(segmentApostrophe, SegOff);
		delayMicroseconds(FRAMEPERIOD - brightnessDelay + 1); //the +1 is a hack so that we can never have a delayMicroseconds(0), causes display to flicker
	}

}
```
### sevseg.h
```JavaScript
//Written by Dean Reading, 2012.  deanreading@hotmail.com
//See .cpp file for info

#ifndef SevSeg_h
#define SevSeg_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <avr/pgmspace.h>

#define COMMON_CATHODE 0
#define COMMON_ANODE 1

#define BLANK 16 //Special character that turns off all segments (we chose 16 as it is the first spot that has this)

// framePeriod controls the length of time between display refreshes
// It's also closely linked to the brightness setting
#define FRAMEPERIOD 2000
//Total amount of time (in microseconds) for the display frame. 1,000us is roughly 1000Hz update rate
//A framePeriod of:
//5000 is flickery
//3000 has good low brightness vs full brightness
//2000 works well
//500 seems like the low brightness is pretty bright, not great


//This is the combined array that contains all the segment configurations for many different characters and symbols
const uint8_t characterArray[] PROGMEM = {
//  ABCDEFG  Segments      7-segment map:
  0b1111110, // 0   "0"          AAA
  0b0110000, // 1   "1"         F   B
  0b1101101, // 2   "2"         F   B
  0b1111001, // 3   "3"          GGG
  0b0110011, // 4   "4"         E   C
  0b1011011, // 5   "5"         E   C
  0b1011111, // 6   "6"          DDD
  0b1110000, // 7   "7"
  0b1111111, // 8   "8"
  0b1111011, // 9   "9"
  0b1110111, // 10  "A"
  0b0011111, // 11  "b"
  0b1001110, // 12  "C"
  0b0111101, // 13  "d"
  0b1001111, // 14  "E"
  0b1000111, // 15  "F"
  0b0000000, // 16  NO DISPLAY
  0b0000000, // 17  NO DISPLAY
  0b0000000, // 18  NO DISPLAY
  0b0000000, // 19  NO DISPLAY
  0b0000000, // 20  NO DISPLAY
  0b0000000, // 21  NO DISPLAY
  0b0000000, // 22  NO DISPLAY
  0b0000000, // 23  NO DISPLAY
  0b0000000, // 24  NO DISPLAY
  0b0000000, // 25  NO DISPLAY
  0b0000000, // 26  NO DISPLAY
  0b0000000, // 27  NO DISPLAY
  0b0000000, // 28  NO DISPLAY
  0b0000000, // 29  NO DISPLAY
  0b0000000, // 30  NO DISPLAY
  0b0000000, // 31  NO DISPLAY
  0b0000000, // 32  ' '
  0b0000000, // 33  '!'  NO DISPLAY
  0b0100010, // 34  '"'
  0b0000000, // 35  '#'  NO DISPLAY
  0b0000000, // 36  '$'  NO DISPLAY
  0b0000000, // 37  '%'  NO DISPLAY
  0b0000000, // 38  '&'  NO DISPLAY
  0b0100000, // 39  '''
  0b1001110, // 40  '('
  0b1111000, // 41  ')'
  0b0000000, // 42  '*'  NO DISPLAY
  0b0000000, // 43  '+'  NO DISPLAY
  0b0000100, // 44  ','
  0b0000001, // 45  '-'
  0b0000000, // 46  '.'  NO DISPLAY
  0b0000000, // 47  '/'  NO DISPLAY
  0b1111110, // 48  '0'
  0b0110000, // 49  '1'
  0b1101101, // 50  '2'
  0b1111001, // 51  '3'
  0b0110011, // 52  '4'
  0b1011011, // 53  '5'
  0b1011111, // 54  '6'
  0b1110000, // 55  '7'
  0b1111111, // 56  '8'
  0b1111011, // 57  '9'
  0b0000000, // 58  ':'  NO DISPLAY
  0b0000000, // 59  ';'  NO DISPLAY
  0b0000000, // 60  '<'  NO DISPLAY
  0b0000000, // 61  '='  NO DISPLAY
  0b0000000, // 62  '>'  NO DISPLAY
  0b0000000, // 63  '?'  NO DISPLAY
  0b0000000, // 64  '@'  NO DISPLAY
  0b1110111, // 65  'A'
  0b0011111, // 66  'b'
  0b1001110, // 67  'C'
  0b0111101, // 68  'd'
  0b1001111, // 69  'E'
  0b1000111, // 70  'F'
  0b1011110, // 71  'G'
  0b0110111, // 72  'H'
  0b0110000, // 73  'I'
  0b0111000, // 74  'J'
  0b0000000, // 75  'K'  NO DISPLAY
  0b0001110, // 76  'L'
  0b0000000, // 77  'M'  NO DISPLAY
  0b0010101, // 78  'n'
  0b1111110, // 79  'O'
  0b1100111, // 80  'P'
  0b1110011, // 81  'q'
  0b0000101, // 82  'r'
  0b1011011, // 83  'S'
  0b0001111, // 84  't'
  0b0111110, // 85  'U'
  0b0000000, // 86  'V'  NO DISPLAY
  0b0000000, // 87  'W'  NO DISPLAY
  0b0000000, // 88  'X'  NO DISPLAY
  0b0111011, // 89  'y'
  0b0000000, // 90  'Z'  NO DISPLAY
  0b1001110, // 91  '['
  0b0000000, // 92  '\'  NO DISPLAY
  0b1111000, // 93  ']'
  0b0000000, // 94  '^'  NO DISPLAY
  0b0001000, // 95  '_'
  0b0000010, // 96  '`'
  0b1110111, // 97  'a' SAME AS CAP
  0b0011111, // 98  'b' SAME AS CAP
  0b0001101, // 99  'c'
  0b0111101, // 100 'd' SAME AS CAP
  0b1101111, // 101 'e'
  0b1000111, // 102 'F' SAME AS CAP
  0b1011110, // 103 'G' SAME AS CAP
  0b0010111, // 104 'h'
  0b0010000, // 105 'i'
  0b0111000, // 106 'j' SAME AS CAP
  0b0000000, // 107 'k'  NO DISPLAY
  0b0110000, // 108 'l'
  0b0000000, // 109 'm'  NO DISPLAY
  0b0010101, // 110 'n' SAME AS CAP
  0b0011101, // 111 'o'
  0b1100111, // 112 'p' SAME AS CAP
  0b1110011, // 113 'q' SAME AS CAP
  0b0000101, // 114 'r' SAME AS CAP
  0b1011011, // 115 'S' SAME AS CAP
  0b0001111, // 116 't' SAME AS CAP
  0b0011100, // 117 'u'
  0b0000000, // 118 'b'  NO DISPLAY
  0b0000000, // 119 'w'  NO DISPLAY
  0b0000000, // 120 'x'  NO DISPLAY
  0b0000000, // 121 'y'  NO DISPLAY
  0b0000000, // 122 'z'  NO DISPLAY
  0b0000000, // 123 '0b'  NO DISPLAY
  0b0000000, // 124 '|'  NO DISPLAY
  0b0000000, // 125 ','  NO DISPLAY
  0b0000000, // 126 '~'  NO DISPLAY
  0b0000000, // 127 'DEL'  NO DISPLAY
};


class SevSeg {

public:
  SevSeg();

  //Public Functions
  void DisplayString(const char*, byte);
//  void NewNumber(int number_in, byte DecPlace_in);
  void Begin(boolean mode_in, byte numOfDigits, byte digit1, byte digit2, byte digit3, byte digit4, byte segment1, byte segment2, byte segment3, byte segment4, byte segment5, byte segment6, byte segment7, byte segmentDP);
  void Begin(boolean mode_in, byte numOfDigits, byte digit1, byte digit2, byte digit3, byte digit4, byte digitColon, byte digitApostrophe, byte segment1, byte segment2, byte segment3, byte segment4, byte segment5, byte segment6, byte segment7, byte segmentDP, byte segmentColon, byte segmentApostrophe);
  void SetBrightness(byte percentBright);

  //Public Variables

private:
  //Private Functions
  void displayCharacter(byte characterToDisplay); //Illuminates the correct segments
  void SplitNumber(int);

  //Private Variables
  boolean mode, DigitOn, DigitOff, SegOn, SegOff;

  byte digit1, digit2, digit3, digit4;
  byte digitApostrophe, digitColon, segmentApostrophe, segmentColon;
  byte segmentA, segmentB, segmentC, segmentD, segmentE, segmentF, segmentG, segmentDP;

  byte numberOfDigits;

  unsigned int brightnessDelay;

  byte DigitPins[4];
  byte SegmentPins[8];
  boolean lights[4][8];
  byte nums[4];

  byte DecAposColon;
};

#endif
```

## References
7 seg 4 dig Bubble Display [QDSP6064](https://cdn.sparkfun.com/datasheets/Components/LED/BB_QDSP_DS.pdf)  
N-channel enhancement type power MOSFET [IRLU8743 IPAK](https://www.infineon.com/dgdl/irlr8743pbf.pdf?fileId=5546d462533600a4015356719c7e26ff)

