File: /LinzerSchnitte/Readme.txt


The code is compiled using the microchip C18 compiler,  the free version of the C18 compiler is available for download from the microchip website.
http://www.microchip.com/stellent/idcplg?IdcService=SS_GET_PAGE&nodeId=1406&dDocName=en010014

The Mplab IDE is the preferred development environment, either Mplab-X or Mplab8x is fine.

For the Klangwolke version, the receive frequency is hardcoded in the Tune routine, (right at the end of the file)


int Tune(int antenna, int freq) {

    Si4705_SET_PROPERTY(0x1107, antenna); // select antenna source
    Si4705_TUNE(freq);
    ms(500);
    Si4705_TUNE_STATUS();
    return (Si4705_RSQ_STATUS());
}

Tune (internal, 10790)  // will tune to 107.90 Mhz and use the on board antenna
Tune (external, 10200)  // will tune to 102.00 Mhz (FM4 Freinberg, Linz) using the external antenna

One of the first changes to be made will be to move the default frequency to eeprom.

To reflash the board with new firmware, apply power with the two pins alongside the AEC logo to enter usb bootloader mode.


Regards
Ray