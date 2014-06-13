//*******************************************************
//  The Ars Electronica LinzerSchnitte
//
//  Copyright (c) 2012,2013,2014 Ray Gardiner  ray@etheira.net
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
#include "Si4705.h"

void Si4705_I2C_INIT(void) {
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
#define Si4705_GP02 RC0
#define Si4705_RST  RC3

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



int Si4705_RSQ_STATUS(void) {
    int i;
    Si4705_I2C_START();
    Si4705_I2C_TX(Si4705_write_address);
    Si4705_I2C_TX(0x23);
    Si4705_I2C_TX(0x01);
    Si4705_I2C_STOP();

    Si4705_STATUS_WAIT();

    Si4705_GET_RESPONSE(8);

    ST  = resp[3];
    RSSI =resp[4];
    SNR  =resp[5];

    return (resp[4]);
}

void Si4705_DIGITAL_AUDIO_SETUP(void) {
    //	Si4705_SET_PROPERTY( 0x102, 0b00000000 );
    //  falling DCLK edge left justified 16 bit
    //	Si4705_SET_PROPERTY( 0x102, 0b00000100 );  //  mono i2s
    Si4705_SET_PROPERTY(0x102, 0b01000011); //  DSP mode 16 bit stereo
    Si4705_SET_PROPERTY(0x104, 37500);

    Si4705_SET_PROPERTY(0x1105, 0); // force stereo, no blending
    Si4705_SET_PROPERTY(0x1106, 0); // force stereo, no blending
}

int Si4705_TUNE_STATUS(void) {
    int i;
    int w;
//    crlf();
//    tx('T');
//    tx('=');

    Si4705_I2C_START();
    Si4705_I2C_TX(Si4705_write_address);
    Si4705_I2C_TX(0x22);
    Si4705_I2C_TX(0x01);
    Si4705_I2C_STOP();

    Si4705_STATUS_WAIT();

    Si4705_GET_RESPONSE(8);

    w = (int)(resp[2])*256 + resp[3];

    //return (f);


     // tx(' '); // tx('M'); tx('h'); tx('z'); tx(',');
 //   dRSSI(resp[4]);
 //   dSNR(resp[5]);
 //   tx(',');
 //   txdec8(resp[7]);
 //   tx('p');
 //   tx('F');

    //tx('A'); tx('C'); tx('A'); tx('P'); tx('=');
    // printdecimal16( (int)(resp[7]) ); tx('p'); tx('F');
    //for (i=0;i<8;i++) {
    //	txhex(resp[i]); tx(' ');
    //}
    //txhex16(w);
     return w;
}

void Si4705_RDS_CONFIG(void) {
    Si4705_SET_PROPERTY(0x1502, 0xff01);
}

void  Si4705_RDS_STATUS(void) {

    //int i, k;
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

        RDS_A = ((int)(A[0])<<8)+ A[1];
        RDS_B = ((int)(B[0])<<8)+ B[1];
        RDS_C = ((int)(C[0])<<8)+ C[1];
        RDS_D = ((int)(D[0])<<8)+ D[1];


        GROUP = (B[0] >>4 )&0x0f;  // ignore AB flag

        if (GROUP == 6) {

            CMD = (B[1]&0x1f);
            ADDR = RDS_C;
            DATA = RDS_D;
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
