//*******************************************************
//  The Ars Electronica LinzerSchnitte
//
//  Copyright (c) 2012, Ray Gardiner
//  All rights reserved.
//
//  Redistribution and use in source and binary forms, with or without
//  modification, are permitted provided that the following conditions are met:
//
//1. Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//2. Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.

//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
//  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
//  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
//  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
//  ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
//  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
//  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
//  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
//  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
//  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
//
//*******************************************************



#include "LS2014.h"


//uncomment the following.. this is required just for LED Suits.. 
//that requires inverted output.
//#define INVERT_PWM_OUTPUT 0

// EEPROM Data********************************************************/
// The following data will be incorporated into the hex file
// programmed into cpu eeprom.
//
#pragma romdata eedata=0xF00000
rom unsigned char eedata_values[0x40] = {

    // eeprom location 0x00 is used as a flag to trigger bootloader,
    // if it's not 0xa5 then
    // enter bootloader mode,  if it's 0xa5 then run application code
    //
    0xa5,       // 0x00
    0x03, 0x03, // 0x01 Code version Major:Minor
    0x06,       // 0x03 Startup Mode
                // 01 == blink pattern
                // 02 == breath pattern 
                // 03 == twinkle pattern
                // 04 == sparkle pattern 
                // 05 == signal strength mode
                // 06 == tone decode enabled
                // 07 == tone decode disabled 

    0x02,       // 0x04 == Output Mode
                // 01 == PWM Disabled
                // 02 == PWM Enabled
                // 03 == RGB Enabled

    0x03,       // 0x05 Number of Tones

    0x03,       // 0x06 Number of Samples (1000 = 03e8)
    0xe8,       // 0x07

    // device serial number is used in RDS commands,
    // refer to custom usb programmer for automatic
    // serial number generation
    //
    0x00, 0x00, // 0x08 08 device serial number == unique address
    0x2a, 0x26, // 0x0a 10 default frequency 107.90 Mhz
    0x00, 0x64, // 0x0c 12 threshold default = 100
    0x00, 0x19, // 0x0e 14 hysteresis default = 25

    0x01, 0x2C, // 0x10 16 tone 1 300
    0x01, 0x90, // 0x12 16 tone 2 400
    0x01, 0xF4, // 0x14 20 tone 3 500


    0x00, 0x00, // 0x16 22 tone 4 0  // space for extra tones
    0x00, 0x00, // 0x18 24 tone 5 0
    0x00, 0x00, // 0x1a 26 tone 6 0
    
    0x01,       // 0x1c 28 ramp on time   100 == 1 second
    0x01,       // 0x1d 29 ramp off time
    0x01,       // 0x1e 30 fade_in
    0x01,       // 0x1f 31 fade_out


    0xff, 0xf0, // 0x20 32 address G1 word group address
    0xf0, 0x1a, // 0x32 34 address G2 word group address
    0xf0, 0x00, // 0x24 36 address G3 word group address

    0xff, 0xf1, // 0x26 38 address G4 word group address
    0xff, 0xf2, // 0x28 40 address G5 word group address
    0xff, 0xf3, // 0x2a 42 address G6 word group address

    0x00,       // 0x2c 44 antenna_type external
    0x01,       // 0x2d 45 serial-debug mode
    0x00,       // 0x2e 46 save station default to no
    0xff,       // 0x2f 47

    0x00,0x00,0x00,0x01, // 0x30 48 bit mask for row 1
    0x00,0x00,0x00,0x01, // 0x34 52 bit mask for row 2
    0x00,0x00,0x00,0x00, // 0x38 56 bit mask for row 3
    0x00,0x00,0x00,0x00, // 0x3C 60 bit mask for row 4


};

// eeprom mapping
#define version_major 1
#define version_minor 2
#define start_mode_address 3
#define output_mode_address 4
#define number_of_tones_address 5
#define number_of_samples_address 6
#define serial_address  8
#define freq_address    10
#define threshold_address 12
#define hysteresis_address 14
#define tone1_address   16
#define tone2_address   18
#define tone3_address   20
#define tone4_address   22
#define tone5_address   24
#define tone6_address   26

#define pattern_on_address 28
#define pattern_off_address 29
#define fade_in_address 30
#define fade_out_address 31

#define group1_address	32
#define group2_address  34
#define group3_address  36
#define group4_address	38
#define group5_address  40
#define group6_address  42

#define antenna_type_address 44
#define serial_mode_address 45
#define save_station_address 46
#define bitmap 48

// enumerate output modes
#define on_off      1
#define pwm_ramps   2
#define rgb         3
#define toggle      4
#define pwm_mag     5

// called from interrupt service routines
void InterruptHandlerHigh(void);
void sample(void);
void pwm_manager(void);
void ms(int);
//-----------------------------------------------------------------------------
//memory map
//-----------------------------------------------------------------------------
#pragma udata access zp_ram=0x40

near unsigned int N; // number of samples in Goertzel
near BYTE trigger;
near signed long raw_audio;
near unsigned int cf;
near signed long Q0; // Goertzel Q values
near signed long Q1;
near signed long Q2;
near signed long T; // temp for Q1 calculation

near unsigned int  mstimer;
near unsigned char pwm_clk;
near unsigned char pwm;
near unsigned int  auto_off;
near unsigned char out;

#pragma udata
unsigned long next = 1;

signed char result[6]; // temporary storage for 48 bit result

signed int SIG; // signal to threshold
float M; // goertzel signal magnitude
float coeff; // goertzel cooefficient

unsigned int  f1, f2, f3, f4, f5, f6; // goertzel frequencies
unsigned char m1, m2, m3, m4, m5, m6; // goertzel magnitudes

unsigned int FREQ; // FM radio Frequency

unsigned char hi;
unsigned char lo;
unsigned char resp[10];
unsigned char s[0x20];

unsigned char S[4];
unsigned char A[2];
unsigned char B[2];

unsigned char D[2];
unsigned char O[2];
unsigned char C[2];

unsigned int RDS_A;
unsigned int RDS_B;
unsigned int RDS_C;
unsigned int RDS_D;

unsigned char CMD;
unsigned int ADDR;
unsigned int DATA;
unsigned char GROUP;
unsigned int SERIAL;
unsigned int G1;
unsigned int G2;
unsigned int G3;
unsigned int G4;
unsigned int G5;
unsigned int G6;


unsigned char enable_goertzel;
unsigned int enable_fade;
unsigned int fade_in;
unsigned int fade_out;

unsigned char pattern;
unsigned char kill_pattern;
unsigned int data_hi;
unsigned int data_lo;

unsigned int off_time;
unsigned int on_time;

unsigned int off_timer;
unsigned int on_timer;

unsigned char threshold_level;
unsigned char hysteresis;
unsigned char threshold_off;          // threshold_level - hysteresis

unsigned int ramp_up_time;
unsigned int ramp_dn_time;
unsigned int ramp_dn;
unsigned int ramp_up;
unsigned int ramp_ctr;
unsigned int ramp_step;
unsigned int pwm_step;
unsigned char pattern_complete;

unsigned char signal_strength_mode;
unsigned char start_mode;

unsigned char RSSI;  // received signal strength indicator
unsigned char SNR;  // signal to noise ratio
unsigned char ST;  // % stereo
unsigned char SS;  // signal strength  SNR*RSSI

unsigned char output_mode;
unsigned char serial_mode;

unsigned char bm[0x10];

unsigned char green_led_timer;
unsigned char red_led_timer;

unsigned char txchar;
unsigned char txtemp;
unsigned int rds_count;
unsigned int ctr;
unsigned char out_flag;
unsigned char save_station;
unsigned char bitmap_enable;

/////////////////////////////////////////////////////////////////////

/* --- BEGIN: changes required for bootloader --------------------------- */

extern void _startup(void); // See c018i.c in the C18 compiler dir

#pragma code _RESET_INTERRUPT_VECTOR = 0x001000

void _reset(void) {
    _asm goto _startup _endasm
}
#pragma code
#pragma code _HIGH_INTERRUPT_VECTOR = 0x001008

void _high_ISR(void) {
    _asm
    // here every 16 DCLOCKS ( DCLK==1.2Mhz)
    BTFSS PIR1, aTMR2IF, 0
            GOTO InterruptHandlerHigh
            BCF PIR1, aTMR2IF, 0
            goto sample

            _endasm
}

#pragma code _LOW_INTERRUPT_VECTOR = 0x001018

void _low_ISR(void) {
    _asm goto InterruptHandlerHigh _endasm
}


#pragma code
#pragma interrupt InterruptHandlerHigh

void InterruptHandlerHigh(void) {
    if (PIR1bits.TMR1IF) {
        PIR1bits.TMR1IF = 0;
        TMR1H = TMR1_PRELOAD_HI;
        TMR1L = TMR1_PRELOAD_LO;
        // every ms
        if (mstimer) {
            mstimer--;
        }

        if (green_led_timer) {
            GREEN_LED = 0;
            green_led_timer--;
            if (green_led_timer == 0) {
                GREEN_LED = 1;
            }
        }
        if (red_led_timer) {
            RED_LED = 0;
            red_led_timer--;
            if (red_led_timer == 0) {
                RED_LED = 1;
            }
        }
        pwm_manager();
    }
}

#pragma code

void pwm_manager(void) {

    if (ramp_up) {
        pattern_complete = 0; // used to trigger reload of values

        if (ramp_up == 1) {
            pwm = 255u;
            ramp_up = 0;
            on_timer = on_time;
        }
        else {
            if (ramp_step == 1) {
                pwm++;
                if (pwm == 255u) {
                    ramp_up = 0;
                    on_timer = on_time;
                }
            } else {
                if (ramp_step == 0) {
                    ramp_ctr--;
                    if ((pwm + pwm_step) > 255u) {
                        pwm = 255u;
                        ramp_up = 0;
                        on_timer = on_time;
                    } else {
                        pwm = pwm + pwm_step;
                    }
                } else {
                    ramp_ctr--;
                    if (ramp_ctr <= 0) {
                        ramp_ctr = ramp_step;
                        pwm++;
                        if (pwm == 255u) {
                            ramp_up = 0;
                            on_timer = on_time;
                        }
                    }
                }
            }
        }
    }
    if (on_timer) {
        on_timer--;
        if (on_timer <= 0) {
            ramp_step = ramp_dn_time >> 8;
            pwm_step = 256 / ramp_dn_time;
            ramp_ctr = ramp_step;
            ramp_dn = ramp_dn_time;
        }
    }
    if (ramp_dn) {
        if (ramp_dn == 1) {
            pwm = 0;
            ramp_dn = 0;
            off_timer = off_time;
        } else {
            if (ramp_step == 1) {
                if (pwm) {
                    pwm--;
                    if (pwm <= 0) {
                        ramp_dn = 0;
                        off_timer = off_time;
                    }
                } else {
                    ramp_dn = 0;
                    off_timer = off_time;
                }
            } else {
                if (ramp_step == 0) {
                    ramp_ctr--;
                    if (pwm >= pwm_step) {
                        pwm = pwm - pwm_step;
                    }
                    else {
                        pwm = 0;
                        ramp_dn = 0;
                        off_timer = off_time;
                    }
                } else {
                    ramp_ctr--;
                    if (ramp_ctr <= 0) {
                        ramp_ctr = ramp_step;
                        if (pwm) {
                            pwm--;
                        }
                    }
                    if (pwm <= 0) {
                        ramp_dn = 0;
                        off_timer = off_time;
                    }
                }
            }
        }
    }
    if (off_timer) {
        off_timer--;
        if (off_timer <= 0) {
            pattern_complete = 1;
        }
    }

    if (auto_off) {
        auto_off--;
        if (auto_off <= 0) {
            pwm = 0;
        }
    }
}

/* --- digital audio interface  -------------------------------- */

#pragma code sample

void sample (void) {
    _asm
    // we are here every 13.3 useconds

            BTG trigger, 0, 0 // divide by 2
            BTFSS trigger, 0, 0
            RETFIE 1

            // we are here every 26.6 useconds
            // why not generate a PWM output, something to do...:-)
            //

            INCFSZ pwm_clk, F, ACCESS   // 1     1 2     2  1  1 
                                        // if skip  clock generates a ramp ,
                                        // wraps at 0xff every 6.8 ms 146.48
                                        //  Hz (37,500/256)
            BRA pwm_off_check           // 1     2          2  1
                                        // check if time to turn off
            MOVF pwm, W, ACCESS         // 1     1       1     1
            BZ pwm_off_and_exit         // 1     1 2     2     1 if branch
            BSF BLUE                    // 1     1  	       1
                                        // pwm non zero turn on
            BRA blue_led_update         //		       2
pwm_off_check:
            MOVF pwm, W, ACCESS         // 1     1             1
            CPFSGT pwm_clk, ACCESS      // 1     1 2           1
                                        // if skip ...
                                        // if pwm == ramp value
                                        // turn off and exit
            BRA blue_led_update         // 1     2           2 if not just exit
pwm_off_and_exit:
            INCFSZ pwm, 0, ACCESS       // 1     1             1 2 
                                        // if skip dummy increment to
                                        // check if pwm=0xff, if so stay on
            BCF BLUE                    // 1     1  	1 pwm non zero turn on
            
blue_led_update:

dfs_gen:
            // generate frame sync DFS

samp1:      BTFSC DCLK
            BRA samp1 // wait till DCLK is high
samp2:      BTFSS DCLK
            BRA samp2 // wait till DCLK falling edge
            BSF DFS // start frame sync pulse
samp3:      BTFSC DCLK
            BRA samp3 // wait for DCLK rising edgE
samp4:      BTFSS DCLK
            BRA samp4 // wait till DCLK falling edge
            NOP NOP
            BCF DFS // stop frame sync pulse

            BTFSS trigger, 6, ACCESS // enable audio capture and processing
            RETFIE 1

            //----------------------------------------------------------------
            NOP
            BCF raw_audio, 7, ACCESS
            NOP // BCF	  GREEN
            BTFSC DOUT
            BSF raw_audio, 7, ACCESS
            NOP NOP NOP NOP NOP NOP

            BCF raw_audio, 6, ACCESS
            NOP // BTG	GREEN
            BTFSC DOUT
            BSF raw_audio, 6, ACCESS
            NOP NOP NOP NOP NOP NOP

            BCF raw_audio, 5, ACCESS
            NOP // BTG	GREEN
            BTFSC DOUT
            BSF raw_audio, 5, ACCESS
            NOP NOP NOP NOP NOP NOP

            BCF raw_audio, 4, ACCESS
            NOP // BTG	GREEN
            BTFSC DOUT
            BSF raw_audio, 4, ACCESS
            NOP NOP NOP NOP NOP NOP

            BCF raw_audio, 3, ACCESS
            NOP // BTG	GREEN
            BTFSC DOUT
            BSF raw_audio, 3, ACCESS
            NOP NOP NOP NOP NOP NOP

            BCF raw_audio, 2, ACCESS
            NOP // BTG	GREEN
            BTFSC DOUT
            BSF raw_audio, 2, ACCESS
            NOP NOP NOP NOP NOP NOP

            BCF raw_audio, 1, ACCESS
            NOP // BTG	GREEN
            BTFSC DOUT
            BSF raw_audio, 1, ACCESS
            NOP NOP NOP NOP NOP NOP

            BCF raw_audio, 0, ACCESS
            NOP // BTG	GREEN
            BTFSC DOUT
            BSF raw_audio, 0, ACCESS

            NOP 
            CLRF raw_audio + 1, ACCESS
            CLRF raw_audio + 2, ACCESS
            CLRF raw_audio + 3, ACCESS

            BTFSS raw_audio, 7, ACCESS           // msb
            BRA g31
            SETF raw_audio + 1, ACCESS
            SETF raw_audio + 2, ACCESS           // sign extend raw audio 
                                                 // to 32 signed for later
                                                 // 32 bit addition
            SETF raw_audio + 3, ACCESS
g31:


            //	CLRF	raw_audio+0,ACCESS	  // zero raw_audio to test
                                                  // Goertzel with null input
            //	CLRF	raw_audio+1,ACCESS
            //	CLRF	raw_audio+2,ACCESS
            //	CLRF	raw_audio+3,ACCESS

            //  Per sample Goertzel processing
            //	Q0 = (Q1 * cf) >> 15  - Q2 + raw_audio;

            MOVLB 0x00

            MOVF Q1, W, ACCESS                  // move previous Q1 to Q2
            MOVWF Q2, ACCESS
            MOVF Q1 + 1, W, ACCESS
            MOVWF Q2 + 1, ACCESS
            MOVF Q1 + 2, W, ACCESS
            MOVWF Q2 + 2, ACCESS
            MOVF Q1 + 3, W, ACCESS
            MOVWF Q2 + 3, ACCESS

            MOVF Q0, W, ACCESS                  // move previous Q0 to Q1
            MOVWF Q1, ACCESS
            MOVWF T, ACCESS

            MOVF Q0 + 1, W, ACCESS
            MOVWF Q1 + 1, ACCESS
            MOVWF T + 1, ACCESS

            MOVF Q0 + 2, W, ACCESS
            MOVWF Q1 + 2, ACCESS
            MOVWF T + 2, ACCESS

            MOVF Q0 + 3, W, ACCESS
            MOVWF Q1 + 3, ACCESS
            MOVWF T + 3, ACCESS


            BTFSS Q1 + 3, 7, ACCESS             // is Q1 negative
            BRA Q1positive

            // negate the copy of Q1, (T is a copy of Q1 )

            COMF T, F, ACCESS
            COMF T + 1, F, ACCESS
            COMF T + 2, F, ACCESS
            COMF T + 3, F, ACCESS
            INCF T, F, ACCESS
            BTFSC STATUS, 2, ACCESS // skpnz
            INCF T + 1, F, ACCESS
            BTFSC STATUS, 2, ACCESS // skpnz
            INCF T + 2, F, ACCESS
            BTFSC STATUS, 2, ACCESS // skpnz
            INCF T + 3, F, ACCESS

Q1positive:
            MOVF T, W, ACCESS
            MULWF cf, ACCESS

            MOVF PRODL, W, ACCESS
            MOVWF result + 0, BANKED

            MOVF PRODH, W, ACCESS
            MOVWF result + 1, BANKED

            MOVF T + 2, W, ACCESS
            MULWF cf, ACCESS

            MOVF PRODL, W, ACCESS
            MOVWF result + 2, BANKED

            MOVF PRODH, W, ACCESS
            MOVWF result + 3, BANKED

            MOVF T + 3, W, ACCESS
            MULWF cf + 1, ACCESS

            MOVF PRODL, W, ACCESS
            MOVWF result + 4, BANKED

            MOVF PRODH, W, ACCESS
            MOVWF result + 5, BANKED

            MOVF T + 1, W, ACCESS
            MULWF cf, ACCESS
            MOVF PRODL, W, ACCESS
            ADDWF result + 1, F, BANKED
            MOVF PRODH, W, ACCESS
            ADDWFC result + 2, F, BANKED
            CLRF WREG, ACCESS
            ADDWFC result + 3, F, BANKED
            ADDWFC result + 4, F, BANKED
            ADDWFC result + 5, F, BANKED

            MOVF T + 3, W, ACCESS
            MULWF cf, ACCESS
            MOVF PRODL, W, ACCESS
            ADDWF result + 3, F, BANKED
            MOVF PRODH, W, ACCESS
            ADDWFC result + 4, F, BANKED
            CLRF WREG, ACCESS
            ADDWFC result + 5, F, BANKED

            MOVF T, W, ACCESS
            MULWF cf + 1, ACCESS
            MOVF PRODL, W, ACCESS
            ADDWF result + 1, F, BANKED
            MOVF PRODH, W, ACCESS
            ADDWFC result + 2, F, BANKED
            CLRF WREG, ACCESS
            ADDWFC result + 3, F, BANKED
            ADDWFC result + 4, F, BANKED
            ADDWFC result + 5, F, BANKED

            MOVF T + 1, W, ACCESS
            MULWF cf + 1, ACCESS
            MOVF PRODL, W, ACCESS
            ADDWF result + 2, F, BANKED
            MOVF PRODH, W, ACCESS
            ADDWFC result + 3, F, BANKED
            CLRF WREG, ACCESS
            ADDWFC result + 4, F, BANKED
            ADDWFC result + 5, F, BANKED

            MOVF T + 2, W, ACCESS
            MULWF cf + 1, ACCESS
            MOVF PRODL, W, ACCESS
            ADDWF result + 3, F, BANKED
            MOVF PRODH, W, ACCESS
            ADDWFC result + 4, F, BANKED
            CLRF WREG, ACCESS
            ADDWFC result + 5, F, BANKED

            BCF STATUS, 0, ACCESS // multiply by 2
            RLCF result + 0, F, BANKED
            RLCF result + 1, F, BANKED
            RLCF result + 2, F, BANKED
            RLCF result + 3, F, BANKED
            RLCF result + 4, F, BANKED
            RLCF result + 5, F, BANKED

            MOVF result + 5, W, BANKED
            MOVWF Q0 + 3, ACCESS
            MOVF result + 4, W, BANKED
            MOVWF Q0 + 2, ACCESS
            MOVF result + 3, W, BANKED
            MOVWF Q0 + 1, ACCESS
            MOVF result + 2, W, BANKED
            MOVWF Q0 + 0, ACCESS

            BTFSS Q1 + 3, 7, ACCESS // was Q1 negative?
            BRA done_goertzel

            COMF Q0, F, ACCESS
            COMF Q0 + 1, F, ACCESS
            COMF Q0 + 2, F, ACCESS
            COMF Q0 + 3, F, ACCESS
            INCF Q0, F, ACCESS
            BTFSC STATUS, 2, ACCESS // skpnz
            INCF Q0 + 1, F, ACCESS
            BTFSC STATUS, 2, ACCESS // skpnz
            INCF Q0 + 2, F, ACCESS
            BTFSC STATUS, 2, ACCESS // skpnz
            INCF Q0 + 3, F, ACCESS

done_goertzel:

            // Q0 is now (Q1*cf >>15)

            MOVF Q2, W, ACCESS
            SUBWF Q0, F, ACCESS

            MOVF Q2 + 1, W, ACCESS
            SUBWFB Q0 + 1, F, ACCESS

            MOVF Q2 + 2, W, ACCESS
            SUBWFB Q0 + 2, F, ACCESS

            MOVF Q2 + 3, W, ACCESS
            SUBWFB Q0 + 3, F, ACCESS

            // Q0 is now (Q1*cf >>15)-Q2

            MOVF raw_audio, W, ACCESS
            ADDWF Q0, F, ACCESS

            MOVF raw_audio + 1, W, ACCESS
            ADDWFC Q0 + 1, F, ACCESS

            MOVF raw_audio + 2, W, ACCESS
            ADDWFC Q0 + 2, F, ACCESS

            MOVF raw_audio + 3, W, ACCESS
            ADDWFC Q0 + 3, F, ACCESS

            // Q0 is now (Q1*cf >>15)-Q2+raw_audio

            DECF N, F, ACCESS
            MOVLW 0
            SUBWFB N + 1, F, ACCESS

            MOVF N, W, ACCESS
            IORWF N + 1, W, ACCESS
            BNZ goertzel_continues

            // if N==0, stop capture and goertzel processing, 
            // and signal data is ready

            BSF trigger, 5, 0 // data ready
            BCF trigger, 6, 0 // stop capture
            NOP // BSF		RED

goertzel_continues:

            NOP // BSF		GREEN
            RETFIE 1

            _endasm

}



void ms(int dly) {
    mstimer = dly;
    while (mstimer) {
        ClrWdt();
    }
}

void initSerial(void) {
    TXSTA = 0;
    RCSTA = 0;
    BAUDCON = 0;
    TXSTAbits.BRGH = 1;
    SPBRG = 25u; //115200baud
    RCSTAbits.SPEN = 1;
    TXSTAbits.TXEN = 1;
}

unsigned char ee_read8(unsigned char addr) {
    EEADR = addr;
    EECON1bits.EEPGD = 0; // data
    EECON1bits.CFGS = 0; // eeprom
    EECON1bits.RD = 1; // read
    return (EEDATA);
}

void ee_write8(unsigned char data, unsigned char addr) {
    EEADR = addr;
    EEDATA = data;
    EECON1bits.EEPGD = 0; // data eeprom
    EECON1bits.CFGS = 0; // eeprom
    EECON1bits.WREN = 1;
    INTCONbits.GIE = 0; // disable interrupts
    _asm
    MOVLW 0x55
            MOVWF EECON2, 0
            MOVLW 0xAA
            MOVWF EECON2, 0
            BSF EECON1, 1, 0 //Performs write
            _endasm
    while (EECON1bits.WR);
    EECON1bits.WREN = 0;
    INTCONbits.GIE = 1; // disable interrupts

}

void ee_write16(unsigned int data, unsigned char addr) {
    unsigned char hi;
    unsigned char lo;
    hi = (data >> 8)&0xff;
    lo = (data)&0xff;
    ee_write8(hi, addr);
    ee_write8(lo, addr + 1);
}

unsigned int ee_read16(unsigned char addr) {
    unsigned char hi;
    unsigned char lo;

    hi = ee_read8(addr);
    lo = ee_read8(addr + 1);
    return (hi * 256 + lo);

}

////////////////// Debugging Support ////////////////////////////////////

void tx(char c) {
    while (PIR1bits.TXIF == 0) {
        continue;
    }
    TXREG = c;
}

void crlf(void) {
    tx(0x0d);
    tx(0x0a);
}

unsigned char bin2ascii(unsigned char c) {
    // takes binary 0x00-0xff and returns ascii
    if (c > 0x09) {
        return (c + 0x37);
    }
    return (c + 0x30);
}

void txhex(unsigned char c) {
    hi = (c >> 4)&0x0f;
    lo = c & 0x0f;
    tx(bin2ascii(hi));
    tx(bin2ascii(lo));
}

void txhex16(unsigned int x) {
    txhex((x >> 8)&0xff);
    txhex((x)&0xff);
    tx(' ');
}


unsigned char d[3];

void txdec8_buff ( int x) {
// convert to decimal and leave in buffer d[3];

    d[0] = 0x00;
    d[1] = 0x00;
    d[2] = 0x00;
    if (x == 0) {
        tx('0');
        return;
    }
    while (x >= 0) {
        x = x - 100u;
        d[0]++;
    }
    x = x + 100u;
    d[0]--;
    while (x >= 0) {
        x = x - 10u;
        d[1]++;
    }
    x = x + 10u;
    d[1]--;
    while (x >= 0) {
        x = x - 1u;
        d[2]++;
    }
    x = x + 1u;
    d[2]--;

}
void txdec8(int x) {
    unsigned char j;
    txdec8_buff(x);
    if (d[0]==0x30) { d[0]=0x20; } // suppress leading zero.
    for (j = 0; j < 3; j++) {
        d[j] = bin2ascii(d[j]); // convert to ascii
        tx(d[j]); 
    }
}

void txdec8_2(int x) {
    unsigned char j;
    txdec8_buff(x);
    for (j = 1; j < 3; j++) {
        d[j] = bin2ascii(d[j]); // convert to ascii
        tx(d[j]);
    }
}
void print_frequency ( int f )
{
    // deleted to make space for noise reject filtering..
    //
    unsigned char fh,fl;
    fh=f/100;
    fl=f-fh*100;

    txdec8(fh);
    tx('.');
    txdec8_2(fl);


}

void txbin8 ( unsigned char c )
{
    unsigned char i;
    unsigned char mask = 0x80;
    for (i=0;i<8;i++)
    {
        if (c&mask) { tx('1'); } else { tx('0'); }
        mask=mask >>1;
    }
    tx(' ');
}
////////////////// End Debugging Support ////////////////////////////////////

int check_red_button(void) {
    int s;
    red_led_timer = 1;
    ms(2);
    RED_LED = 1;

    TRISCbits.TRISC2 = 1;
    if ((PORTC & 0b00000100) == 0) {

        Si4705_SEEK();
        //s=Si4705_TUNE_STATUS();
        // wait for release

        while ((PORTC & 0b00000100) == 0) {
            continue;
        }
        ms(2000);
        s=Si4705_TUNE_STATUS();
        crlf();
        tx('F');
        tx('=');
        print_frequency(s);
        if (save_station==1) { ee_write16(s,freq_address); }
    }
    TRISCbits.TRISC2 = 0;
}

void blank_pattern(void) {
    ramp_up = 0;
    on_timer = 0;
    on_time = 0;
    ramp_dn = 0;
    off_timer = 0;
    off_time = 0;
    auto_off = 0;
}

int ramp_setup(int n) {
    if (n == 0) n = 1;
    ramp_step = n >> 8u;
    pwm_step = 256u / n;
    ramp_ctr = ramp_step;
    return n;
}

void pwm_on(int n) {
    ramp_up_time = n;
    ramp_up = ramp_setup(n); // needed for tone ramping ( fade-in )

}

void pwm_off(int n) {
    ramp_dn_time = n;
    ramp_dn = ramp_setup(n);
}

unsigned char invert;
unsigned char armed;
unsigned char noise_reject;

void activate_output ( unsigned char m )
{
    int x;

    blank_pattern();

    switch (output_mode)
    {
        case on_off:
                // turn on..  ramps disabled in pwm_manager
                pwm_on(1); break;
        case pwm_ramps:
                // normal fade-in fade out mode
                //if (pwm==0)  {
                    pwm_on(fade_in);
                //}
                break;
        case rgb:
                // reserved for rgb output mode
                break;

        case toggle:
            noise_reject--;
            if ((noise_reject<=0)||(enable_goertzel==0)) {
                // reject noise if doing goertzel
                if (invert) { pwm_off(1); } else { pwm_on(1); }
                armed=1;
                noise_reject=3;
            }
            break;


        case pwm_mag:
                // pwm value depends on goertzel magnitude above threshold
                // m is 0-200,  so scale
                // pwm=(unsigned char) ((int)(m-threshold_level)<<8)/156;
                // not enough code room!!! to calculate properly...

                 pwm=(m-threshold_level)*2;
                 
                break;
    }
}

void de_activate_output ( void)
{

    blank_pattern();

    switch (output_mode)
    {
        case on_off:
                // turn off..  
                pwm_off(1);  break;
        case pwm_ramps:
                // normal fade-in fade out mode
                //blank_pattern();
                pwm_off(fade_out);
                break;
        //case rgb:
                // reserved for rgb output mode
        //        break;
        case toggle:
                if (armed) {
                    invert=!invert;
                    armed=0;
                }
                noise_reject=5;
                break;
        case pwm_mag:
                pwm_off(1); break;
    }
}


#define pi 3.1415926

unsigned char Goertzel ( int f ) 
{

    unsigned long mag;
    unsigned char mx;
    
    //  tone decoding can be disabled by clearing the 'enable_goertzel' flag
    //  decoding range is limited to 300Hz to 9300Hz
    //
    if ((enable_goertzel==0)||(f==0xffff)||(f==0x0000) || \
        (f < 300)||(f > 9300))
    {
        return (0);
    }

    //	returns the magnitude of the signal at f

    coeff = 2*cos(2*pi*(float)(f)/37500);
    cf = (unsigned int)(coeff*32768);
    Q0 = 0;
    Q1 = 0;
    Q2 = 0;

    trigger.b5 = 0;

    N = ee_read16(number_of_samples_address);

    //// start interrupt sampling and processing, this captures the
    //// digital audio each DFS frame and calculates Q0,Q1,Q2 for each sample
    //// sample rate is 37500Hz

    trigger.b6 = 1;
    PIE1bits.TMR1IE = 0; // stop Timer 1  interrupts
    while (trigger.b5 == 0) {
        continue;
    } // wait till sample completes
    trigger.b6 = 0;
    //// stop interrupt sampling
    PIE1bits.TMR1IE = 1; // re-enable Timer 1 interrupts

    M = (float)(Q1)*(Q1)+(float)(Q2)*(float)(Q2)-(float)(Q1)*(float)(Q2)*coeff;
    
    mag = (long) (M);

    mag = (mag >> 12u);

    mag = mag * (f + 1200u) >> 11;  // black magic.. correct for roll-off at high freq

    if (mag > 255u) { mag = 255u;  }

    mx = (unsigned char)(mag);

    if ( mx > threshold_level) {
        // turn on if above threshold
        activate_output(mx);
    }
    return (mx);
}


void refresh_addresses(void) {
    SERIAL = ee_read16(serial_address);

    G1 = ee_read16(group1_address);
    G2 = ee_read16(group2_address);
    G3 = ee_read16(group3_address);
    G4 = ee_read16(group4_address);
    G5 = ee_read16(group5_address);
    G6 = ee_read16(group6_address);

    f1 = ee_read16(tone1_address);
    f2 = ee_read16(tone2_address);
    f3 = ee_read16(tone3_address);
    f4 = ee_read16(tone4_address);
    f5 = ee_read16(tone5_address);
    f6 = ee_read16(tone6_address);

    threshold_level = ee_read16(threshold_address);
    hysteresis = ee_read16(hysteresis_address);

}


void set_tone(unsigned int addr, unsigned int data) {
    refresh_addresses();
    // specify a tone for this address
    if (addr == G1) { ee_write16(data, tone1_address); return; }
    if (addr == G2) { ee_write16(data, tone2_address); return; }
    if (addr == G3) { ee_write16(data, tone3_address); return; }
    if (addr == G4) { ee_write16(data, tone4_address); return; }
    if (addr == G5) { ee_write16(data, tone5_address); return; }
    if (addr == G6) { ee_write16(data, tone6_address); return; }
}


void set_data(void) {
    data_hi = ((DATA >> 8)&0xff)*10u;
    data_lo = (DATA & 0xff)*10u;
}



void new_pattern(unsigned char t) {
    pwm_off(1);
    ms(10);
    auto_off = 0;
    pattern = t;
    blank_pattern();
    pattern_complete = 1;
    set_data();
}


void check_bm ( unsigned char row )
{
    out_flag=0;

    if ((bm[row*4+0]|bm[row*4+1]|bm[row*4+2]|bm[row*4+3])!=0)
    {
        // there are some non-zero bits in this row..
        enable_goertzel = 0;

        if ((C[0] && bm[row*4+0])!=0)  { out_flag=1; }
        if ((C[1] && bm[row*4+1])!=0)  { out_flag=1; }
        if ((D[0] && bm[row*4+2])!=0)  { out_flag=1; }
        if ((D[1] && bm[row*4+3])!=0)  { out_flag=1; }

        if (out_flag) {
            activate_output(255);
        }
        else {
            de_activate_output();
        }
    }
}

void RDS_Process(void) {
    Si4705_RDS_STATUS(); // result in A[],B[],C[],D[]

    if (GROUP == 6) {
        refresh_addresses();
       
        if ((ADDR==0xffff)||(ADDR==SERIAL)     || \
            (ADDR==G1)||(ADDR==G2)||(ADDR==G3) || \
            (ADDR==G4)||(ADDR==G5)||(ADDR==G6) 
           )
            {
            switch (CMD) {
                case 0u: break; // null command does nothing
                case 1u: break; // reserved for KW2012 version
                case 2u: break; // reserved for KW2012 version
                case 3u: enable_goertzel = 0;
                    blank_pattern();
                    auto_off = 10 * DATA;
                    pwm = 255u;
                    break;
                case 4u: enable_goertzel = 0;
                    auto_off = 10 * DATA;
                    if (auto_off == 0) {
                        pwm = 0;
                    }
                    break;
                case 5u: set_tone(ADDR, DATA);
                    refresh_addresses();
                    break;
                case 6u: ee_write16(DATA, threshold_address);
                    refresh_addresses();
                    break;
                case 7u: ee_write16(DATA, hysteresis_address);
                    refresh_addresses();
                    break;
                case 8u: Si4705_TUNE(DATA);
                    refresh_addresses();
                    break;
                case 9u: enable_goertzel = 1;
                    pwm_off(1);
                    break;
                case 10u: enable_goertzel = 0;
                    blank_pattern();
                    pwm_off(1);
                    break;
                case 11u: enable_fade = 1;
                    set_data();
                    blank_pattern();
                    pwm_off(1);
                    fade_in = data_hi;
                    fade_out = data_lo;
                    break;

                case 12u: enable_fade = 0;
                    blank_pattern();
                    pwm_off(1);
                    fade_in = 1;
                    fade_out = 1;
                    break;
                case 13u: new_pattern(1);
                    break; // blink
                case 14u: new_pattern(2);
                    break; // breath
                case 15u: new_pattern(3);
                    break; // sparkle
                case 16u: new_pattern(4);
                    break; // twinkle
                case 17u: pattern = 0;
                    blank_pattern();
                    pwm_off(1);
                    break;

                default: break;
            }
        }
        // bit map command handler no address check for bitmap, since the address
        // is used for top 16 bits of data
        if (bitmap_enable) {
            switch (CMD) {
                     case 20u:  check_bm(0);   break;
                     case 21u:  check_bm(1);   break;
                     case 22u:  check_bm(2);   break;
                     case 23u:  check_bm(3);   break;
                    default: break;
            }
        }
    }
}

unsigned char rand(void) {
    next = next * 1103515245u + 12345u;
    return (next >> 16u) & 0xff;
}

unsigned int random(unsigned int x) {
    unsigned long y;
    // return random number between 10 and x
    y = (long) (rand() * x);
    y = 10 * ((y >> 8u) + 10u);
    //printdecimal16(y);
    return y;
}

void pattern_handler ( void)
{
    int p;
    if (pattern == 1) { // blink
        if (pattern_complete) {
            on_time = data_hi;
            off_time = data_lo;
            ramp_dn_time = 1;
            ramp_up_time = 1;
            pwm_on(ramp_up_time);
        }
    }
    if (pattern == 2) { // breath
        if (pattern_complete) {
            on_time = 1;
            off_time = data_lo * 10;
            ramp_dn_time = data_hi * 10;
            ramp_up_time = data_hi * 10;
            pwm_on(ramp_up_time);
        }
    }
    if (pattern == 3) { // sparkle
        if (pattern_complete) {
            on_time = random(data_hi);
            off_time = random(data_lo);
            ramp_dn_time = 1;
            ramp_up_time = 1;
            pwm_on(ramp_up_time);
        }
    }
    if (pattern == 4) { // twinkle
        if (pattern_complete) {
            p = random(data_hi);
            on_time = 1;
            off_time = random(data_lo);
            ramp_up_time = p / 2;
            ramp_dn_time = p / 2;
            pwm_on(ramp_up_time);
        }
    }
}

void signal_strength_handler( void )
{
    int x;

    Si4705_RSQ_STATUS();

    x = (int)(RSSI)*(int)(SNR);
    if (x>255) { x=255; }
    SS = (unsigned char)(x);

    if (SS>threshold_level) { activate_output(SS);  }
    if (SS<threshold_off)   { de_activate_output(); }
}

void goertzel_tone_decoder ( void )
{

    if (ctr % 3 == 0) { red_led_timer = 3; }

    m1=Goertzel(f1);
    m2=Goertzel(f2);
    m3=Goertzel(f3);
    m4=Goertzel(f4);
    m5=Goertzel(f5);
    m6=Goertzel(f6);

    if ((m1 < threshold_off) && \
        (m2 < threshold_off) && \
        (m3 < threshold_off) && \
        (m4 < threshold_off) && \
        (m5 < threshold_off) && \
        (m6 < threshold_off)
    )
    {
         de_activate_output();
    }
}

void graph ( unsigned char x)
{
    unsigned char i;
    for (i=0;i<x;i++) { tx('*'); }
}

void clear_rds_commands(void)
{
    GROUP = 0;
    CMD   = 0;
    ADDR  = 0;
    C[0]  = 0; C[1] = 0;
    D[0]  = 0; D[1] = 0;
}

void print_rx_status ( void )
{
    crlf();
    tx('R'); tx('S'); tx('S'); tx('I'); tx('=');
    txdec8(RSSI);   tx(' ');
    tx('S'); tx('N'); tx('R'); tx('=');
    txdec8(SNR);    tx(' ');

}

void main(void) {

    int i, p, x;


    ANSEL = 0;
    ANSELH = 0;
    TRISB = 0xff;
    LATB = 0xff;
    trigger.b7 = 0; //
    trigger.b6 = 0; // disable audio sampling

    pwm_off(1);

    TRISC = 0x80;
    // LATC  =0b11100110;
    // Timer 1 setup for 1ms interrupt
    T1CON = 0b10000101;
    TMR1H = TMR1_PRELOAD_HI;
    TMR1L = TMR1_PRELOAD_LO;
    IPR1bits.TMR1IP = 1;
    PIE1bits.TMR1IE = 1;

    // Timer 2 setup for 1.2Mhz clock
    T2CON = 0b01111100; // 16:1 post scale Timer on 1:1 Prescale
    CCP1CON = 0b00001111; // PWM
    PSTRCON = 0b00000001; // Output to P1A only RC5 --> RCLK --> DCLK
    PR2 = PR2_PRELOAD; // 1.2 Mhz
    ECCP1AS = 0b00000000; // no Auto shutdown
    PWM1CON = 0b10000000; // auto restart if shut down occurs
    CCPR1L = CCP_DUTY; // try for a square wave
    //
    IPR1bits.TMR2IP = 1; // Hi priority
    PIE1bits.TMR2IE = 1; // Enable generates DFS clock on RC1
    //
    //green_led_timer = 100;
    //red_led_timer = 100;

    INTCONbits.PEIE = 1; //
    RCONbits.IPEN = 1; //
    INTCONbits.GIE = 1; // Enable interrupts

    initSerial();
    crlf();

    tx('V');                            // display version number
    tx(ee_read8(version_major)+0x30);
    tx('.');
    tx(ee_read8(version_minor)+0x30);


    crlf();
    tx('I');                            // I == I2C INIT
    Si4705_I2C_INIT();
    tx('R');
    Si4705_I2C_RESET();                 // R == Si4705 Reset
    ms(100);
    tx('P');                            // P == Si4705 Power up
    Si4705_POWER_UP();
    ms(100);
    tx('S');                            // S = Si4705 Set up RCLK
    Si4705_SET_RCLK();
    ms(100);

    tx('T'); Tune(ee_read8(antenna_type_address),ee_read16(freq_address));
    
    print_frequency(ee_read16(freq_address));

    crlf();
    tx('D');
    Si4705_DIGITAL_AUDIO_SETUP();
    tx('R');
    Si4705_RDS_CONFIG();

    ms(200);
    BLUE_LED = 0;
    trigger.b5 = 0;
    trigger.b6 = 0; // enable/disable audio sampling
    trigger.b7 = 0; // enable/disable audio sampling

    signal_strength_mode=0;  // default to off
    enable_goertzel=0;       // default to off
    pattern=0;               // default to off
    bitmap_enable =0;        // default to off

    x=ee_read8(fade_in_address);
    if (x<2) { fade_in=1; } else { fade_in=10*x; }
    x=ee_read8(fade_out_address);
    if (x<2) { fade_out=1; } else { fade_out=10*x; }

    DATA=ee_read8(pattern_on_address)*256+ee_read8(pattern_off_address);

    // copy 128 bit bitmap mask to working buffer
    for (i=0;i<0x10;i++) {
        bm[i] = ee_read8(bitmap+i);
    }


    start_mode = ee_read8(start_mode_address);
    serial_mode = ee_read8(serial_mode_address);
    save_station= ee_read8(save_station_address);
    output_mode = ee_read8(output_mode_address);

    //crlf();  txhex(output_mode); txhex(start_mode); txhex(serial_mode);

    switch (start_mode) {
        case 1: new_pattern(1);         break; // 01 == blink pattern
        case 2: new_pattern(2);         break; // 02 == breath pattern
        case 3: new_pattern(3);         break; // 03 == sparkle pattern
        case 4: new_pattern(4);         break; // 04 == twinkle pattern
        case 5: signal_strength_mode=1; break; // 05 == signal strength mode
        case 6: enable_goertzel = 1;    break; // 06 == tone decode enabled
        case 7: enable_goertzel = 0;    break; // 07 == tone decode disabled
        case 8: bitmap_enable =1;       break; // 08 == bitmap mode enabled
    }
    
    refresh_addresses();
    next = SERIAL + 5631; // seed random generator

    pwm=255;
    auto_off=1000;
    ms(2000);
   

    crlf();

    threshold_off=threshold_level-hysteresis;

    while (1) {

        if (pattern)  pattern_handler();
        if (signal_strength_mode) signal_strength_handler();

        if (enable_goertzel) {
            goertzel_tone_decoder();
        }
        else {
            ms(10); // insert delay if not doing goertzel
        }

        RDS_Process();       // always check for RDS commands
        
        switch (serial_mode) {
            case 0:  //

            case 1:  // plot goertzel magnitude m1
                crlf();
                txdec8(m1);
                graph(m1);
                break;
            case 2:  // plot raw rds data received
                if (S[1]!=0)  // if new?
                {
                    crlf();
                    txhex16(RDS_A);
                    txhex16(RDS_B);
                    txhex16(RDS_C);
                    txhex16(RDS_D);
                }
                break;

            case 3:
                if (GROUP==6) {
                    crlf();
                    txdec8(CMD);
                    tx (' ');
                    txhex16(ADDR);
                    txhex16(DATA);
                }
                break;
            case 4: // graph signal strength 
                print_rx_status();
                txdec8(SS);     tx(' ');
                graph(SS);
                break;
            case 5: // display bitmap
                if (GROUP==6) {
                    crlf();
                    txdec8(CMD);
                    tx (' ');
                    txbin8(C[0]);
                    txbin8(C[1]);
                    txbin8(D[0]);
                    txbin8(D[1]);
                }
                break;

        }

        clear_rds_commands();
        check_red_button();  // auto tune button
        ctr++;
        if (ctr>500) {  // 500*20-100ms == 5 to 10 seconds
            ctr=0;
            Si4705_RSQ_STATUS();
            print_rx_status();
            print_frequency(Si4705_TUNE_STATUS());
        }
    }
}

