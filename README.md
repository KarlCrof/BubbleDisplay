# Bubble Display
Project 1 for MG7013 Embedded Systems  
Karl Crofskey, 2183664.

## Project Introduction

* LED basics
* 7-seg pinout
* MOSFET pinout

## Circuit Schematic
<img src="https://i.ibb.co/M6vz32c/schematic.jpg" alt="schematic" border="0">

* explain connections
* explain common cathode
* pull down resistor

### _Current Limiting_
*QDSP6064 7-segment 4 digit bubble display datasheet*  
<img src="https://i.ibb.co/M5BWF7f/7segdata.jpg" alt="7segdata" border="0">

A current limiting resistor is required, for each LED segment, in order to limit the current to 5mA (to protect each from over-current damage).

Given the typical LED forward voltage drop of 1.6V, and using the Teensy 3.2 Vcc of 3.3V, Ohm's law was applied to find the minimum resistance required to achieve this.  

> R = V<sub>R</sub> / i  
> R = (3.3 - 1.6)V / 5mA  
> R = 340Ω  

Hence, a 390Ω resistor was used in series with each of the LED segments as the next highest available resistance.

*IRLU8743 N-channel enhancement type power MOSFET datasheet*  
<img src="https://i.ibb.co/QbzmJbC/mosfetdata.jpg" alt="mosfetdata" border="0">

A current limiting resistor is further needed in series with the pins connected the gate of the MOSFETS (used to switch the digits on and off). This is because, as the pin is initially turned HIGH, the charging capacitor of the MOSFET causes an inrush current to be drawn - not sufficiently limited by the 0.85Ω gate resistance - potentially damaging the Teensy microcontroller which has a specified current sourcing capability limit.

A 1KΩ resistor was used to limit the current to 3.3mA, a compromise between having low current and the impact on the MOSFET's switching frequency (increased RC time constant).


## Code Design


## Breadboard Implementation
<img src="https://i.ibb.co/1sspBFW/20190508-151012-annotated.jpg" alt="20190508-151012-annotated" border="0">

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

* see characterArray[] in sevseg.h -> how does digit map to segments
* DisplayString() - cycling digits, setting LED segments, displaying dp

### Stopwatch Implementation

* button interrupt and deboucing
* cumulative count
* reset


## System Verification

## Video

## Appendix


## References
7 seg 4 dig Bubble Display [QDSP6064](https://cdn.sparkfun.com/datasheets/Components/LED/BB_QDSP_DS.pdf)  
N-channel enhancement type power MOSFET [IRLU8743 IPAK](https://www.infineon.com/dgdl/irlr8743pbf.pdf?fileId=5546d462533600a4015356719c7e26ff)
