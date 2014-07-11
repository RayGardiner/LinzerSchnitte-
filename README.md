LinzerSchnitte-
===============

LinzerSchnitte is a project created by the Ars Electronica Future Lab to make available the open source hardware and software used to control wide area lighting effects as shown in the Klangwolke 2012

Version 3.04 Release Notes.

This version is a major rewrite of the Firmware,  new features added include...

1. Ability to specify any six frequencies for an individual receiver.  This required removing the Goertzel coefficient lookup tables and calculating the coefficients on the fly.

2. Increased RDS group addresses to six.  

3. Added configurable serial diagnostic output.  
   3.1 Graph of Goertzel magnitude for tone 1
   3.2 RDS Data output
   3.3 RDS Group 6 Only
   3.4 Signal Strength Graph RSSI
   3.5 Bitmap display
   
4. Added configurable startup modes
   4.1 Start in Tone decode mode
   4.2 Disable Tone decoding
   4.3 Signal Strength Mode
   4.4 Bitmap Mode
   
   4.5 Pattern Modes  ( Blink, Breath, Sparkle, Twinkle ) 
   
5. Added configurable output modes.
   5.1 On/Off mode ( for coffee makers :) )
   5.2 On/Off toggle
   5.3 PWM Switching mode
   5.4 PWM Magnitude mode
   
6. Added options to configure radio.
   6.1 Select antenna, either internal or external
   6.2 Save radio station last auto-tuned
   
7. Added bitmap configuration.

See also the Linzer Schnitte EEPROM editor for setting options listed above.

Ray Gardiner  16/6/2014 
