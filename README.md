The master generates a series of message packets containing servo postion data plus 8 bits of ON/OFF data.
In response a 2-byte message is returned, comrising an 'ACK' plus 8-bits of status data. The master to slave message
has a command byte at the start currently always a fixed value to indicate that the data should be interrpreted as servo & digital
commands. In the future it couold be expanded to allow the sending of failsafe settings.

Uses LoRa mode as I cant get FSK to work.

It uses the RadioLib library to provide the control for the SX1278 RF module, with control from a
3.3V Arduibo Pro-mini. A rotary (hex) switch is used to create the RF channel used. Currently the channel number is set to
<switch input> *2 +3.
This generates channels 3, 5, 7, ..... 31, 33 so providing a buffer zone away from the RF band edges to prevent any out of band issues.
Note that due to the RF (LoRa) bandwidth selected it uses most of 3 channels, hence the use of channel 3 as the lowest. 

To prevent too much time taken up on display tasks the measured RF channel parameters are display one by one in a sequence, 
one item displayed every so many Tx/Rx message pairs, current 10, so about every 1.5 secs. Eventually an OLED display is intended, 
but see commens about OLED displays below.

It uses adafruit libraries for the ADS1015 4 channel analogue input module. This provides a 12-bit resolution, but this is converted to 
an 8-bit value 0 - 255. Scaling is provided by specifying the min & max ADC values (that equate to 0 and 255 respectivly). 
These are found by reporting the ADC values with the joysticks at their extreme positions using a simple test sketch.

As the MCP23017 is simple no libraries are used to control, simply in-line type code only. Bank A inputs are used to create the 
digiutal channel data byte sent to the slave, bank B can be used within the master. Currently bits 0 & 1 are used to 
increment/decrement the offset for analogue channel 0, and bits 2 & 3 are used to incrememnt/decrement the offset for
analogue channel 1.

The slave expects a series of message packets containing message header, servo postion data plus 8 bits of ON/OFF data.
Currently the message header is always 0x5A (MSG_CMND), but could be expanded e.g. to send fail-safe settings.
In response a 2-byte message is returned, comrising an 'ACK' plus 8-bits of status data

Uses LoRa mode as I can't get FSK to work.

It uses the RadioLib library to provide the control for the SX1278 RF module, with control from a
3.3V Arduibo Pro-mini.

To prevent too much time taken up on display tasks the measured RF channel parameters are display one by one in a sequence, 
one item displayed every so many Tx/Rx message pairs, current 10, so about every 1.5 secs. An OLED display was tried but the 
library code is too large and will not fit, so if required a low-level alternative is required.

It uses adafruit PWM library to control PCA9685 16-channel PWM controller, channels 0 to 5 are the analogue channels, 8 to 15 are 
the digital (on/off) channels. Channel 7 is used to generate a 50% square wave nominally 50Hz (20msec) that is used to allow the 
Arduino to calibrate the PWM clock frequency.

Each servo channel has a defined min and max PW value that is used to scale from the 0 - 255 message value to the servo limits, 
and each servo may be inverted such that the min & max physical limits are reveresed.
These min & max PW can be found using a simple test sketch that allows these limits to be found such that the servo is not driven 
beyond its physical limits, or the required limits if a reduced physical range is required.
