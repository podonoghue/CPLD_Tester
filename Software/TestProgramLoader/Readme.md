# CPLD Programmer
Software for stand-alone CPLD programmer

This program is used with the CPLD programmer hardware to create a 
stand-alone programmer.  This is used to load a test design to the CPLD
for use with the CPLD tester.   

The loaded hardware design just blinks all the CPLD pins for testing.

Features: 
* Stand-alone hardware not requiring PC connection.
* Incorporates the hardware design to load to the CPLD.
* Hardware programming starts automatically 1 second after power-on
* Programming status is provided by an LED (slow blink OK, fast blink failure)
