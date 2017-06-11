# LPCXpresso

This project contains a an example monitoring the light sensor, temperature sensor, trim potentiometer and rotatory switch and displaying the values on the OLED display.

## THE PROBLEM

**The values received from the temperature sensor, light sensor as well as the changes caused by the trim potentiometer or the rotatory switch should be displayed on the OLED display as graphs in relation to the passed time and at the same time the valued should be displayed on the 7 segment display.
<br>The display of different graphs should be caused by taster.
<br>The time of displaying the changes should be configurable.
<br>In the next phase of the project we should enable recording of measurements in the internal EEPROM memory so later it is possible to display the stored results on display.**

---
The project makes use of code from the following library projects:
- CMSISv1p30_LPC17xx : for CMSIS 1.30 files relevant to LPC17xx
- MCU_Lib        	 : for LPC17xx peripheral driver files
- EaBaseBoard_Lib    : for Embedded Artists LPCXpresso Base Board peripheral drivers

*These library projects must exist in the same workspace in order
for the project to successfully build.*
