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

#include <p18cxxx.h>
#include "typedefs.h"
#include "KW2012_Hardware.h"

// Not enough code space for these libraries... :(
//#include <stdlib.h>
//#include <math.h>

//uncomment the following.. this is required just for LED Suits.. or anything else
//that requires inverted output.
//#define INVERT_PWM_OUTPUT 1

// EEPROM Data********************************************************/
// The following data will be incorporated into the hex file programmed into cpu eeprom.
//
#pragma romdata eedata=0xF00000
rom unsigned char eedata_values[0x40] = {

    // eeprom location 0x00 is used as a flag to trigger bootloader, if it's not 0xa5 then
    // enter bootloader mode,  if it's 0xa5 then run application code
    //
    0xa5, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,

    // device serial number is used in RDS commands,  refer to custom usb programmer for automatic
    // serial number generation
    //
    0x09, 0x19, // 0x08 device serial number == unique address
    0x2a, 0x26, // 0x0a default frequency 107.90 Mhz  --- hard coded in Tune for Klangwolke
    0x00, 0x32, // 0x0c threshold default = 100
    0x00, 0x19, // 0x0e hysteresis default = 25

    0x01, 0x2C, // 0x10 tone 1 300  Vienna 2
    0x01, 0xF4, // 0x12 tone 2 500
    0x02, 0x58, // 0x14 tone 3 600


    0xff, 0xff, // 0x16 tone 4 600  // space for extra tones
    0xff, 0xff, // 0x18 tone 5 400
    0xff, 0xff, // 0x1a tone 6 500
    0xff, 0xff, // 0x1c tone 7 600
    0xff, 0xff, // 0x1e tone 8 400


    0xff, 0xf0, // 0x20 address G1 word group address
    0xf0, 0x1a, // 0x32 address G2 word group address
    0xf0, 0x00, // 0x24 address G3 word group address

    0xff, 0xff, // 0x26             // space for extra addresses
    0xff, 0xff, // 0x28
    0xff, 0xff, // 0x2a
    0xff, 0xff, // 0x2c
    0xff, 0xff, // 0x2e

    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, // 0x30 not used
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, // 0x38 not used
};

// eeprom mapping,  use the following to address locations in eeprom
//
#define serial_address  0x08
#define freq_address    0x0a
#define threshold_address 0x0c
#define hysteresis_address 0x0e

#define tone1_address   0x10
#define tone2_address   0x12
#define tone3_address   0x14

#define group1_address	0x20
#define group2_address  0x22
#define group3_address  0x24

#pragma romdata


// Goertzel algorithm cooefficients
rom const float coefficients[] = {
    1.9975, 1.9955, 1.9930, 1.9899, 1.9863, 1.9821, 1.9773, 1.9720, 1.9661, 1.9597, // 300
    1.9527, 1.9452, 1.9372, 1.9286, 1.9194, 1.9097, 1.8995, 1.8888, 1.8775, 1.8657, //1300
    1.8533, 1.8405, 1.8271, 1.8132, 1.7988, 1.7839, 1.7685, 1.7526, 1.7362, 1.7193, //2300
    1.7020, 1.6842, 1.6658, 1.6471, 1.6278, 1.6081, 1.5880, 1.5674, 1.5464, 1.5249, //3300
    1.5030, 1.4807, 1.4579, 1.4348, 1.4112, 1.3873, 1.3630, 1.3383, 1.3132, 1.2877, //4300
    1.2619, 1.2357, 1.2092, 1.1823, 1.1551, 1.1276, 1.0998, 1.0717, 1.0432, 1.0145, //5300
    0.9855, 0.9562, 0.9266, 0.8968, 0.8667, 0.8364, 0.8058, 0.7750, 0.7440, 0.7128, //6300
    0.6814, 0.6498, 0.6180, 0.5861, 0.5540, 0.5217, 0.4893, 0.4567, 0.4240, 0.3912, //7300
    0.3583, 0.3253, 0.2922, 0.2590, 0.2257, 0.1924, 0.1590, 0.1256, 0.0921, 0.0586, //8300
    0.0251 //9300
};

// Goertzel algorithm cooefficients, scaled to maximum unsigned 16 bit
rom const unsigned int cf_values[] = {
    65453u, 65388u, 65306u, 65205u, 65085u, 64948u, 64792u, 64618u, 64426u, 64215u,
    63987u, 63741u, 63477u, 63195u, 62895u, 62578u, 62243u, 61890u, 61520u, 61133u,
    60729u, 60308u, 59870u, 59415u, 58943u, 58455u, 57950u, 57429u, 56892u, 56339u,
    55770u, 55186u, 54586u, 53971u, 53340u, 52695u, 52034u, 51360u, 50670u, 49967u,
    49249u, 48518u, 47773u, 47015u, 46243u, 45459u, 44661u, 43852u, 43030u, 42195u,
    41349u, 40492u, 39623u, 38742u, 37851u, 36950u, 36038u, 35115u, 34183u, 33242u,
    32291u, 31331u, 30362u, 29385u, 28399u, 27406u, 26404u, 25396u, 24380u, 23357u,
    22328u, 21293u, 20251u, 19204u, 18152u, 17094u, 16032u, 14965u, 13894u, 12819u,
    11740u, 10658u, 9573u, 8486u, 7396u, 6304u, 5210u, 4115u, 3018u, 1921u,
    823
};


//P R O T O T Y P E S ***************************************/

void Si4705_I2C_INIT(void);
void Si4705_I2C_RESET(void);
int  Si4705_RSQ_STATUS(void);
void Si4705_POWER_UP(void);
void Si4705_SET_RCLK(void);
void Si4705_TUNE(unsigned int);
int  Si4705_STATUS_CHECK(void);
void Si4705_DIGITAL_AUDIO_SETUP(void);
void Si4705_TUNE_STATUS(void);
void Si4705_RDS_CONFIG(void);
void Si4705_RDS_STATUS(void);
void Si4705_SEEK(void);
void Si4705_SET_PROPERTY(unsigned int, int);

// called from interrupt service routines
void InterruptHandlerHigh(void);
void sample(void);
void pwm_manager(void);

void ms(int);
int Tune ( int,int);

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

static near unsigned int  mstimer;
static near unsigned char pwm_clk;
static near unsigned char pwm;
static near unsigned int  auto_off;

#pragma udata
static unsigned long next = 1;

static signed char result[6]; // temporary storage for 48 bit result

static unsigned int sample_length; // sample length
static signed int SIG; // signal to threshold
static float M; // goertzel signal magnitude
static float coeff; // goertzel cooefficient
static unsigned int f1, f2, f3; // goertzel frequencies

static unsigned int FREQ; // FM radio Frequency
static unsigned char hi;
static unsigned char lo;
static unsigned char resp[10];
static unsigned char s[0x20];

static unsigned char S[4];
static unsigned char A[2];
static unsigned char B[2];
static unsigned char C[2];
static unsigned char D[2];
static unsigned char O[2];

static unsigned int RDS_A;
static unsigned int RDS_B;
static unsigned int RDS_C;
static unsigned char CMD;
static unsigned int ADDR;
static unsigned int DATA;
static unsigned int EDATA;
static unsigned int FDATA;
static unsigned char GROUP;

static unsigned int SERIAL;
static unsigned int G1;
static unsigned int G2;
static unsigned int G3;


static unsigned char enable_goertzel;
static unsigned int enable_fade;
static unsigned int fade_in;
static unsigned int fade_out;

static unsigned int pattern;
static unsigned char kill_pattern;
static unsigned int data_hi;
static unsigned int data_lo;

static unsigned int off_time;
static unsigned int on_time;

static unsigned int off_timer;
static unsigned int on_timer;

static unsigned int threshold_level;
static unsigned int hysteresis;

static unsigned int ramp_up_time;
static unsigned int ramp_dn_time;
static unsigned int ramp_dn;
static unsigned int ramp_up;
static unsigned int ramp_ctr;
static unsigned int ramp_step;
static unsigned int pwm_step;
static unsigned int pattern_complete;

static unsigned char green_led_timer;
static unsigned char red_led_timer;

static unsigned char txchar;
static unsigned char txtemp;

// MP:
static unsigned char state;
static unsigned char toggleComplete;


/////////////////////////////////////////////////////////////////////

/* --- BEGIN: changes required for bootloader ------------------------------ */

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

            INCFSZ pwm_clk, F, ACCESS   // 1     1 2     2  1  1 if skip clock generates a ramp , wraps at 0xff every 6.8 ms 146.48 Hz (37,500/256)
            BRA pwm_off_check           // 1     2          2  1 check if time to turn off
            MOVF pwm, W, ACCESS         // 1     1       1     1
            BZ pwm_off_and_exit         // 1     1 2     2     1 if branch

#if defined (INVERT_PWM_OUTPUT)
            BCF BLUE
#else
            BSF BLUE                    // 1     1  	       1 pwm non zero turn on
#endif

            BRA dfs_gen                 //		       2
pwm_off_check:
            MOVF pwm, W, ACCESS         // 1     1             1
            CPFSGT pwm_clk, ACCESS      // 1     1 2           1  if skip ...if pwm == ramp value turn off and exit
            BRA dfs_gen                 // 1     2             2 if not just exit
pwm_off_and_exit:
            INCFSZ pwm, 0, ACCESS       // 1     1             1 2 if skip dummy increment to check if pwm=0xff, if so stay on

#if defined (INVERT_PWM_OUTPUT)
            BSF BLUE
#else
            BCF BLUE                    // 1     1  	       1 pwm non zero turn on
#endif


            // generate frame sync DFS
dfs_gen:

samp1: BTFSC DCLK
            BRA samp1 // wait till DCLK is high
samp2: BTFSS DCLK
            BRA samp2 // wait till DCLK falling edge
            BSF DFS // start frame sync pulse
samp3: BTFSC DCLK
            BRA samp3 // wait for DCLK rising edgE
samp4: BTFSS DCLK
            BRA samp4 // wait till DCLK falling edge
            NOP NOP
            BCF DFS // stop frame sync pulse

            BTFSS trigger, 6, ACCESS // enable audio capture and processing
            RETFIE 1

            //------------------------------------------------------------------------
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
            SETF raw_audio + 2, ACCESS           // sign extend raw audio to 32 signed for later 32 bit addition
            SETF raw_audio + 3, ACCESS
g31:


            //	CLRF	raw_audio+0,ACCESS	  // zero raw_audio to test Goertzel with null input
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

            // if N==0, stop capture and goertzel processing, and signal data is ready
            BSF trigger, 5, 0 // data ready
            BCF trigger, 6, 0 // stop capture
            NOP // BSF		RED

goertzel_continues:

            NOP // BSF		GREEN
            RETFIE 1

            _endasm

}

void initGoertzel(int f) {
    unsigned char k;
    // initialize goertzel co-oefficients for frequency f
    k = (f - 300) / 100;
    cf = cf_values[k];
    coeff = coefficients[k];
    Q0 = 0;
    Q1 = 0;
    Q2 = 0;
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

///////////////////////////////////////  Debugging Support ////////////////////////////////////////
//

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
    `
    tx(bin2ascii(lo));
}

void txhex16(unsigned int x) {
    txhex((x >> 8)&0xff);
    txhex((x)&0xff);
    tx(' ');
}

/*
void printdecimal16 ( int x )
{
        unsigned char sign;
        unsigned char leading_zero_blanking;
        unsigned char right_justify;
        unsigned char j;
        unsigned char d[5]; for (j=0;j<5;j++) { d[j]=0; }

        if (x==0) {  tx('0'); return; }

        if (x<0) { sign='-'; x = -1*x; } else { sign=' ';	}

        // x + ve
        while (x>=0) {  x = x-10000u;      d[0]++; } x=x+10000u; d[0]--;
        while (x>=0) {  x = x-1000u;       d[1]++; } x=x+1000u; d[1]--;
        while (x>=0) {  x = x-100u;        d[2]++; } x=x+100u; d[2]--;
        while (x>=0) {  x = x-10u;         d[3]++; } x=x+10u; d[3]--;
        while (x>=0) {  x = x-1u;          d[4]++; } x=x+1u; d[4]--;

        right_justify=1;
        leading_zero_blanking=1;
        tx(sign);
        for (j=0;j<5;j++) {
                d[j]=bin2ascii(d[j]);  // convert to ascii
                if (leading_zero_blanking) {
                                if (d[j]==0x30) { d[j]=0x20; } else { leading_zero_blanking=0; }
                }
                if (right_justify==0) {
                        if(d[j]!=0x20) { tx(d[j]); } // only print if non-space
                }
                else {
                        tx(d[j]);  // print leading spaces
                }
        }
}
 */

void txdec8(int x) {
    unsigned char j;
    unsigned char d[3];
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

    for (j = 0; j < 3; j++) {
        d[j] = bin2ascii(d[j]); // convert to ascii
        tx(d[j]); // print leading spaces
    }
}

/////////////////////////////////////// End Debugging Support ////////////////////////////////////////

int check_red_button(void) {
    red_led_timer = 1;
    ms(2);
    RED_LED = 1;

    TRISCbits.TRISC2 = 1;
    if ((PORTC & 0b00000100) == 0) {
        tx('D');
        Si4705_SEEK();
        Si4705_TUNE_STATUS();
        // wait for release
        while ((PORTC & 0b00000100) == 0) {
            continue;
        }
        //
    }
    else {
        //	tx('.');
    }
    TRISCbits.TRISC2 = 0;
}

unsigned int Goertzel(int f) {
    unsigned long mag;

    //  tone decoding can be disabled by clearing the 'enable_goertzel' flag
    if (enable_goertzel == 0) {
        return (0);
    }
    if (f == 0xffff) {
        return (0);
    }
    if (f == 0x0000) {
        return (0);
    }
    if (f < 300) {
        return (0);
    }
    if (f > 9300) {
        return (0);
    }
    //	returns the magnitude of the signal at f
    initGoertzel(f);
    trigger.b5 = 0;
    N = sample_length;
    //// start interrupt sampling and processing, this captures the digital audio each DFS frame and calculates Q0,Q1,Q2 for each sample
    //// sample rate is 37500Hz
    trigger.b6 = 1;
    PIE1bits.TMR1IE = 0; // stop Timer 1  interrupts
    while (trigger.b5 == 0) {
        continue;
    } // wait till sample completes
    trigger.b6 = 0;
    //// stop interrupt sampling
    PIE1bits.TMR1IE = 1; // re-enable Timer 1 interrupts

    M = (float) (Q1)*(Q1) + (float) (Q2)*(float) (Q2) - (float) (Q1)*(float) (Q2) * coeff;
    mag = (long) (M);

    mag = (mag >> 12u);
    mag = mag * (f + 1200) >> 11;
    if (mag > 200) {
        mag = 200;
    }
    return (int) (mag);
}

// eeprom mapping

#define serial_address  0x08
#define freq_address    0x0a

#define tone1_address   0x10
#define tone2_address   0x12
#define tone3_address   0x14

#define group1_address	0x20
#define group2_address  0x22
#define group3_address  0x24

void refresh_addresses(void) {
    G1 = ee_read16(group1_address);
    G2 = ee_read16(group2_address);
    G3 = ee_read16(group3_address);
    SERIAL = ee_read16(serial_address);
    f1 = ee_read16(tone1_address);
    f2 = ee_read16(tone2_address);
    f3 = ee_read16(tone3_address);

    threshold_level = ee_read16(threshold_address);
    hysteresis = ee_read16(hysteresis_address);

}

void add_address(unsigned int addr) {
    refresh_addresses();
    // specify a tone for this address
    if (G1 == 0xffff) {
        ee_write16(addr, group1_address);
        return;
    }
    if (G2 == 0xffff) {
        ee_write16(addr, group2_address);
        return;
    }
    if (G3 == 0xffff) {
        ee_write16(addr, group3_address);
        return;
    }
}

void del_address(unsigned int addr) {
    refresh_addresses();
    if (addr == G1) {
        ee_write16(0xffff, group1_address);
        ee_write16(0xffff, tone1_address);
        return;
    }
    if (addr == G2) {
        ee_write16(0xffff, group2_address);
        ee_write16(0xffff, tone2_address);
        return;
    }
    if (addr == G3) {
        ee_write16(0xffff, group3_address);
        ee_write16(0xffff, tone3_address);
        return;
    }
}

void set_tone(unsigned int addr, unsigned int data) {
    refresh_addresses();

    // specify a tone for this address
    if (addr == G1) {
        ee_write16(data, tone1_address);
        return;
    }
    if (addr == G2) {
        ee_write16(data, tone2_address);
        return;
    }
    if (addr == G3) {
        ee_write16(data, tone3_address);
        return;
    }

}

void ee_config(void) {
    unsigned char i;
    crlf();
    tx('C');
    tx('F');
    tx('G');
    tx('=');
    txhex16(ee_read16(8));
    txhex16(ee_read16(0xa));
    txhex16(ee_read16(0xc));
    txhex16(ee_read16(0xe));
    for (i = 0; i < 3; i++) {
        tx('[');
        txhex(i + 1);
        tx(' ');
        txhex16(ee_read16(0x20 + i * 2));
        tx(',');
        txhex16(ee_read16(0x10 + i * 2));
        tx(']');
    }
    crlf();
}

void set_data(void) {
    data_hi = ((DATA >> 8)&0xff)*10u;
    data_lo = (DATA & 0xff)*10u;
    tx('D');
    tx('=');
    txhex16(data_hi);
    txhex16(data_lo);
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
    if (n == 0) {
        n = 1;
    }
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

void new_pattern(unsigned char t) {
    pwm_off(1);
    ms(10);
    auto_off = 0;
    pattern = t;
    blank_pattern();
    pattern_complete = 1;
    set_data();
}

void RDS_Process(void) {
    Si4705_RDS_STATUS(); // result in A[],B[],C[],D[]

    if (GROUP == 6) {
        refresh_addresses();
        if (ADDR == SERIAL) {
            // commands which are REQUIRED to be addressed to specific units
            if (CMD == 1) {
                add_address(DATA);
            }
            if (CMD == 2) {
                del_address(DATA);
            }
            ee_config();
        }
        if ((ADDR == 0xffff) || (ADDR == SERIAL) || (ADDR == G1) || (ADDR == G2) || (ADDR == G3)) {
            switch (CMD) {
                case 0u: break; // null command does nothing
                case 1u: break;
                case 2u: break;
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
                    ee_config();
                    break;
                case 6u: ee_write16(DATA, threshold_address);
                    refresh_addresses();
                    ee_config();
                    break;
                case 7u: ee_write16(DATA, hysteresis_address);
                    refresh_addresses();
                    ee_config();
                    break;
                case 8u: Si4705_TUNE(DATA);
                    refresh_addresses();
                    ee_config();
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
    }
    GROUP = 0;
    CMD = 0;
    ADDR = 0;
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

#define internal 1
#define external 0

void main(void) {
    int ctr;
    int i, p;

    int SIGint;
    int SIGext;
	
    long m1, m2, m3, m4, m5, m6;

    ANSEL = 0;
    ANSELH = 0;
    TRISB = 0xff;
    LATB = 0xff;
    trigger.b7 = 0; //
    trigger.b6 = 0; // disable audio sampling
    //	buffer_index=0;
    pwm = 0;
    ramp_up = 0;
    ramp_dn = 0;

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
    green_led_timer = 100;
    red_led_timer = 100;

    INTCONbits.PEIE = 1; //
    RCONbits.IPEN = 1; //
    INTCONbits.GIE = 1; // Enable interrupts

    initSerial();
    crlf();
    tx('V');
    tx('2');
    tx('.');
    tx('1');
#if defined (INVERT_PWM_OUTPUT)
    tx('I');
#endif

    crlf();
    tx('I');
    Si4705_I2C_INIT();
    tx('R');
    Si4705_I2C_RESET();
    ms(100);
    tx('P');
    Si4705_POWER_UP();
    ms(100);
    tx('S');
    Si4705_SET_RCLK();
    ms(100);

    SIGext = Tune(external,9450u);

    //Si4705_SET_PROPERTY( 0x1107, 1);	// select internal antenna
    //tx('T'); Si4705_TUNE(9750u);
    //tx('T'); Si4705_TUNE(9770u);

    //tx('T'); Si4705_TUNE(10790u);	// later make this eeprom based
    //ms(500);
    //Si4705_TUNE_STATUS();

    crlf();
    tx('D');
    Si4705_DIGITAL_AUDIO_SETUP();
    tx('R');
    Si4705_RDS_CONFIG();

    ms(200);

#if defined (INVERT_PWM_OUTPUT)
    BLUE_LED = 1;
#else
    BLUE_LED = 0;
#endif

    trigger.b5 = 0;
    trigger.b6 = 0; // enable/disable audio sampling
    trigger.b7 = 0; // enable/disable audio sampling

    enable_goertzel = 1;

    pattern = 0;
    refresh_addresses();
    next = SERIAL + 5631; // seed random generator

    fade_in = 1;
    fade_out = 1;

    ee_config();

    pwm = 255;
    auto_off = 1000;
    ms(2000);
    crlf();

    toggleComplete = 1;

    while (1) {
        ctr++;

        if (ctr % 50 == 0) {
            Si4705_RSQ_STATUS();
        }

        if (pattern) {
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
        } else {
            sample_length = 500u;  // usually 500

            // spectrum analyser for threshold verification
            /*
             while(1) {
                    tx(0x0c);
                    for (i=0;i<91; i++)
                    {
                            f1=i*100+300;

                            m1=Goertzel(f1); crlf();  txhex16(f1); txhex16(m1); tx(' '); for (p=0;p<m1;p++) { tx('*'); }

                    }
                    ms(500);
            }
             */

            if (enable_goertzel) {
                if ((ctr % 3 == 0)) {
                    red_led_timer = 3;
                }

                m1 = Goertzel(f1); //crlf();  tx(' '); for (p=0;p<m1;p++) { tx('1'); }
                m2 = Goertzel(f2); //crlf();  tx(' '); for (p=0;p<m2;p++) { tx('2'); }
                
				/* OLD
				//m3 = Goertzel(f3); //crlf();  tx(' '); for (p=0;p<m3;p++) { tx('3'); }

                // if any of the three tones is above threshold turn on
				
					
                if ((m1 > threshold_level) || (m2 > threshold_level) || (m3 > threshold_level) ) {
                    if (pwm==0)  {
                        blank_pattern();
                        pwm_on(fade_in);
                    }
				}
                // if ALL of the three tones are below threshold-hystersis turn off
				if ((m1 < (threshold_level - hysteresis)) && (m2 < (threshold_level - hysteresis)) && (m3 < (threshold_level - hysteresis))) {
                    blank_pattern();
                    pwm_off(fade_out);
                }
				*/
				// MP: Toggle LED strip everytime the m1 sine wave is detected
				if (m1 > threshold_level)
				{
					if (toggleComplete)
					{
//						state ^= 0x01;	// XOR state with 0x01
						if(state == 0x00 && pwm==0)
						{
                                                    state = 0x01;
                                                    toggleComplete=0;
                                                    blank_pattern();
                                                    pwm_on(fade_in);
						}
						else
						{
                                                    state = 0x00;
                                                    toggleComplete=0;
                                                    blank_pattern();
                                                    pwm_off(fade_out);
						}
					}

				}
                                else
                                {
                                    toggleComplete = 1;
                                }

				// Switch off when m2 sine wave is detected (sync)
				if (m2 > threshold_level)
				{
					state = 0x00;	// XOR state with 0x01
					blank_pattern();
					pwm_off(fade_out);
				}
            }

        }

        RDS_Process();

        if ((enable_goertzel == 0) || (pattern != 0)) {
            ms(10);
        } // insert delay if not doing goertzel

        check_red_button();

    }
}

void ms(int dly) {
    mstimer = dly;
    while (mstimer) {
        ClrWdt();
    }
}

//--------------------------------------------------------------------------

void Si4705_I2C_INIT(void) {
    tx('I');
    TRISBbits.TRISB6 = 1;
    TRISBbits.TRISB4 = 1;
    SSPSTAT = 0b11000000;
    SSPCON1 = 0b00111000;
    SSPCON2 = 0b00000000;
    SSPADD = 0x1d;
    SSPCON1bits.WCOL = 0;
    PIR2bits.BCLIF = 0;
}
//-------------------------------------------------------------------
//#define Si4705_GP02 RC0
//#define Si4705_RST  RC3

// Si4705 macro's for i2c bits
#define I2C_BUSY    PIR1bits.SSPIF
#define I2C_START   SSPCON2bits.SEN
#define I2C_STOP    SSPCON2bits.PEN
#define I2C_ACKSTAT SSPCON2bits.ACKSTAT
#define I2C_ACKDT   SSPCON2bits.ACKDT
#define I2C_ACKEN   SSPCON2bits.ACKEN
#define I2C_RX_EN   SSPCON2bits.RCEN

#define Si4705_write_address  0x22   //Si4705 i2c device address *2 +r/w
#define Si4705_read_address   0x23   //Si4705 i2c device address *2 +r/w

void Si4705_STATUS_WAIT(void) {
    while (Si4705_STATUS_CHECK() != 0x80) {
    }
}

void Si4705_I2C_RESET(void) {
    Si4705_RST = 0;
    Si4705_GP02 = 0;
    ms(8);
    Si4705_RST = 1;
    ms(8);
    Si4705_GP02 = 1;
    ms(8);
    Si4705_STATUS_WAIT();
}

void Si4705_I2C_START(void) {
    I2C_BUSY = 0;
    I2C_START = 1;
    while (I2C_BUSY == 0) {
    };
    I2C_BUSY = 0;
}

void Si4705_I2C_STOP(void) {
    I2C_BUSY = 0;
    I2C_STOP = 1;
    while (I2C_STOP == 1) {
    };
    I2C_BUSY = 0;
}

int Si4705_I2C_TX(unsigned char c) {
    SSPBUF = c;
    I2C_BUSY = 0;
    while (I2C_BUSY == 0) {
    };
    return I2C_ACKSTAT;
}

unsigned char Si4705_I2C_RX(void) {
    I2C_RX_EN = 1;
    I2C_BUSY = 0;
    while (I2C_RX_EN == 1) {
    };
    return SSPBUF;
}

void Si4705_I2C_TX_NACK(void) {
    int timeout;
    timeout = 100;
    I2C_ACKDT = 1;
    I2C_ACKEN = 1;
    I2C_BUSY = 0;
    while ((I2C_BUSY == 0) && (timeout != 0)) {
        timeout--;
    };
    if (timeout == 0) {
    }
    //while (I2C_ACKEN=1) { continue; };
}

void Si4705_I2C_TX_ACK(void) {
    int timeout;
    timeout = 100u;
    I2C_ACKDT = 0;
    I2C_ACKEN = 1;
    I2C_BUSY = 0;
    while ((I2C_BUSY == 0) && (timeout != 0)) {
        timeout--;
    };
    if (timeout == 0) {
    }
    //while (I2C_ACKEN=1) { tx('+'); };
}

int Si4705_STATUS_CHECK(void) {
    int res;
    Si4705_I2C_START();
    Si4705_I2C_TX(Si4705_read_address);
    res = Si4705_I2C_RX();
    Si4705_I2C_TX_NACK();
    Si4705_I2C_STOP();
    return (res & 0x80);
}

void Si4705_SET_PROPERTY(unsigned int property, int value) {
    unsigned char p_hi;
    unsigned char p_lo;
    unsigned char val_hi;
    unsigned char val_lo;

    p_hi = (property >> 8) & 0xff;
    p_lo = property & 0xff;
    val_hi = (value >> 8) & 0xff;
    val_lo = value & 0xff;

    Si4705_I2C_START();
    Si4705_I2C_TX(Si4705_write_address);
    Si4705_I2C_TX(0x12);
    Si4705_I2C_TX(0x00);
    Si4705_I2C_TX(p_hi);
    Si4705_I2C_TX(p_lo);
    Si4705_I2C_TX(val_hi);
    Si4705_I2C_TX(val_lo);
    Si4705_I2C_STOP();

    Si4705_STATUS_WAIT();
}

void Si4705_SET_RCLK(void) {
    Si4705_SET_PROPERTY(0x201, RCLK);
    Si4705_SET_PROPERTY(0x202, RCLK_DIV);

}

void Si4705_TUNE(unsigned int freq) {
    unsigned char f_hi;
    unsigned char f_lo;

    f_hi = (freq >> 8)&0xff;
    f_lo = freq & 0xff;

    Si4705_I2C_START();
    Si4705_I2C_TX(Si4705_write_address);
    Si4705_I2C_TX(0x20);
    Si4705_I2C_TX(0x00);
    Si4705_I2C_TX(f_hi);
    Si4705_I2C_TX(f_lo);
    Si4705_I2C_TX(0x00);
    Si4705_I2C_STOP();

    Si4705_STATUS_WAIT();
}

void Si4705_SEEK(void) {
    Si4705_I2C_START();
    Si4705_I2C_TX(Si4705_write_address);
    Si4705_I2C_TX(0x21);
    Si4705_I2C_TX(0b00000100);
    Si4705_I2C_STOP();

    Si4705_STATUS_WAIT();
}

void Si4705_POWER_UP(void) {
    Si4705_I2C_START();
    Si4705_I2C_TX(Si4705_write_address);
    Si4705_I2C_TX(0x01);
    Si4705_I2C_TX(0x00);
    Si4705_I2C_TX(0b10110101);
    Si4705_I2C_STOP();

    Si4705_STATUS_WAIT();
}

void Si4705_GET_RESPONSE(unsigned char n) {
    unsigned char i;
    Si4705_I2C_START();
    Si4705_I2C_TX(Si4705_read_address);
    for (i = 0; i < n - 1; i++) {
        resp[i] = Si4705_I2C_RX();
        Si4705_I2C_TX_ACK();
    }
    resp[n - 1] = Si4705_I2C_RX();
    Si4705_I2C_TX_NACK();
    Si4705_I2C_STOP();
}

void RSSI(unsigned char r) { // Received Signal Strength Indicator
    // prints RSSI=xxx db,
    tx('R');
    tx('S');
    tx('S');
    tx('I');
    tx('=');
    txdec8((int) (r & 0x7f));
    tx(',');
}

void SNR(unsigned char snr) { // Received Signal to Noise Ratio
    // prints SNR=xxx db,
    tx('S');
    tx('N');
    tx('R');
    tx('=');
    txdec8((int) (snr & 0x7f));
}

int Si4705_RSQ_STATUS(void) {
    int i;
    crlf();
    tx('R');
    tx('S');
    tx('Q');
    tx('=');

    Si4705_I2C_START();
    Si4705_I2C_TX(Si4705_write_address);
    Si4705_I2C_TX(0x23);
    Si4705_I2C_TX(0x01);
    Si4705_I2C_STOP();

    Si4705_STATUS_WAIT();

    Si4705_GET_RESPONSE(8);

    if (resp[3]&0x80) {
        tx('S');
        tx('t');
        tx('=');
        txdec8((int) (resp[3]&0x7f));
        tx('%');
        tx(',');
    }

    RSSI(resp[4]);
    SNR(resp[5]);

    //for (i=0;i<8;i++) {
    //	txhex(resp[i]); tx(' ');
    //}

    return (resp[4]);
}

void Si4705_DIGITAL_AUDIO_SETUP(void) {
    //	Si4705_SET_PROPERTY( 0x102, 0b00000000 );  // falling DCLK edge left justified 16 bit
    //	Si4705_SET_PROPERTY( 0x102, 0b00000100 );  //  mono i2s
    Si4705_SET_PROPERTY(0x102, 0b01000011); //  DSP mode 16 bit stereo
    Si4705_SET_PROPERTY(0x104, 37500);

    Si4705_SET_PROPERTY(0x1105, 0); // force stereo, no blending
    Si4705_SET_PROPERTY(0x1106, 0); // force stereo, no blending
}

void Si4705_TUNE_STATUS(void) {
    int i;
    int f;
    crlf();
    tx('T');
    tx('=');

    Si4705_I2C_START();
    Si4705_I2C_TX(Si4705_write_address);
    Si4705_I2C_TX(0x22);
    Si4705_I2C_TX(0x01);
    Si4705_I2C_STOP();

    Si4705_STATUS_WAIT();

    Si4705_GET_RESPONSE(8);

    f = resp[2]*256 + resp[3];
    txhex16(f);
    tx(' '); // tx('M'); tx('h'); tx('z'); tx(',');
    RSSI(resp[4]);
    SNR(resp[5]);
    tx(',');
    txdec8(resp[7]);
    tx('p');
    tx('F');

    //tx('A'); tx('C'); tx('A'); tx('P'); tx('='); printdecimal16( (int)(resp[7]) ); tx('p'); tx('F');
    //for (i=0;i<8;i++) {
    //	txhex(resp[i]); tx(' ');
    //}
}

void Si4705_RDS_CONFIG(void) {
    Si4705_SET_PROPERTY(0x1502, 0xff01);
}

void Si4705_RDS_STATUS(void) {
    int i, k;
    green_led_timer = 3;
    //if(Si4705_FIFO_STATUS()==0) { return; }

    Si4705_I2C_START();
    Si4705_I2C_TX(Si4705_write_address);
    Si4705_I2C_TX(0x24);
    Si4705_I2C_TX(0b00000001);
    Si4705_I2C_STOP();

    Si4705_STATUS_WAIT();

    Si4705_I2C_START();
    Si4705_I2C_TX(Si4705_read_address);

    S[0] = Si4705_I2C_RX();
    Si4705_I2C_TX_ACK(); //status
    S[1] = Si4705_I2C_RX();
    Si4705_I2C_TX_ACK(); //RDS int status
    S[2] = Si4705_I2C_RX();
    Si4705_I2C_TX_ACK(); //RDS Sync
    S[3] = Si4705_I2C_RX();
    Si4705_I2C_TX_ACK(); //RDS FIFO

    A[0] = Si4705_I2C_RX();
    Si4705_I2C_TX_ACK(); //Block A PI hi
    A[1] = Si4705_I2C_RX();
    Si4705_I2C_TX_ACK(); //Block A PI lo



    B[0] = Si4705_I2C_RX();
    Si4705_I2C_TX_ACK(); //Block B
    B[1] = Si4705_I2C_RX();
    Si4705_I2C_TX_ACK(); //Block B

    C[0] = Si4705_I2C_RX();
    Si4705_I2C_TX_ACK(); //Block C
    C[1] = Si4705_I2C_RX();
    Si4705_I2C_TX_ACK(); //Block C

    D[0] = Si4705_I2C_RX();
    Si4705_I2C_TX_ACK(); //Block D
    D[1] = Si4705_I2C_RX();
    Si4705_I2C_TX_ACK(); //Block D


    O[0] = Si4705_I2C_RX();
    Si4705_I2C_TX_NACK(); //BLE error


    Si4705_I2C_STOP();

    //tx('['); txhex(S[3]); tx(']');

    if (S[3] == 0) {
        return; // nothing in FIFO
    }

    if (S[1] != 0) {


        GROUP = (B[0] / 16)&0x0f;


        if (GROUP == 6) {
            CMD = (B[1]&0x1f);



            crlf();
            tx('R');
            tx('D');
            tx('S');
            tx('=');
            txhex(S[0]);
            tx(' ');
            txhex(S[1]);
            tx(' ');
            txhex(S[2]);
            tx(' ');
            txhex(S[3]);
            tx(' ');

            //tx('A'); tx('='); txhex(A[0]); txhex(A[1]); tx(',');
            //tx('B'); tx('='); txhex(B[0]); txhex(B[1]); tx(',');
            //tx('C'); tx('='); txhex(C[0]); txhex(C[1]); tx(',');
            //tx('D'); tx('='); txhex(D[0]); txhex(D[1]); tx(',');
            //tx('G'); tx('=');  txhex(GROUP); tx('+'); txhex(CMD);

            RDS_A = A[0]*256u + A[1];
            txhex16(RDS_A);
            RDS_B = B[0]*256u + B[1];
            txhex16(RDS_B);
            ADDR = C[0]*256u + C[1];
            txhex16(ADDR);
            DATA = D[0]*256u + D[1];
            txhex16(DATA);

        }
    } 
}


int Tune(int antenna, int frequency) {

    Si4705_SET_PROPERTY(0x1107, antenna); // select antenna source
    Si4705_TUNE(frequency);
    ms(500);
    Si4705_TUNE_STATUS();
    return (Si4705_RSQ_STATUS());
}
