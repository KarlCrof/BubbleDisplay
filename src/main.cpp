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
int deciSecond = 0;
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
   int segDP= 6; //Pin 5

  int numberOfDigits = 4; //1,2,or 4 digits

  myDisplay.Begin(displayType, numberOfDigits, digit1, digit2, digit3, digit4, segA, segB, segC, segD, segE, segF, segG, segDP);

  myDisplay.SetBrightness(100); //Set the display to 100% brightness level

}

void loop()
{
  //Example ways of displaying a decimal number
  char tempString[10]; //Used for sprintf
  //sprintf(tempString, "%4d", deciSecond); //Convert deciSecond into a string that is right adjusted
  //sprintf(tempString, "%d", deciSecond); //Convert deciSecond into a string that is left adjusted
  sprintf(tempString, "%04d", deciSecond); //Convert deciSecond into a string with leading zeros
  //sprintf(tempString, "%4d", deciSecond * -1); //Shows a negative sign infront of right adjusted number
  //sprintf(tempString, "%4X", deciSecond); //Count in HEX, right adjusted

  //Produce an output on the display
  myDisplay.DisplayString(tempString, 0b100); //(numberToDisplay, decimal point location IN BINARY)

  //Other examples
  //myDisplay.DisplayString(tempString, 0); //Display string, no decimal point
  //myDisplay.DisplayString("-23b", 3); //Display string, decimal point in third position

  //Check if 10ms has elapsed
  //if (millis() - starttime_millis >= 100)
  //{
  //  starttime_millis = millis();
  //  deciSecond++;
  //}

  static uint32_t time_elapsed_ms = 0;
  static uint32_t time_stopped_ms = 0;
  static uint32_t total_time_elapsed_ms = 0;
  
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