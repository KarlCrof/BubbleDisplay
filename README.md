# Bubble Display
Project 1 for MG7013 Embedded Systems  
Karl Crofskey, 2183664.

## Project Requirements


## Circuit Schematic
<img src="https://i.ibb.co/wMDmfvX/schematic.jpg" alt="schematic" border="0">

* explain connections
* explain common cathode
* pull down resistor / mosfet resistor

### current limiting
<img src="https://i.ibb.co/M5BWF7f/7segdata.jpg" alt="7segdata" border="0">

* 390 resistor value


## Code Design


## Breadboard Implementation


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
