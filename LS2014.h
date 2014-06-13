/* 
 * File:   LS2014.h
 * Author: ray@etheira.net
 *
 * Created on 8 June 2014, 6:21 PM
 */

#ifndef LS2014_H
#define	LS2014_H


#include <p18cxxx.h>
#include "typedefs.h"
#include "Si4705.h"
// Not enough code space for these libraries... :(
//#include <stdlib.h>
#include <math.h>


extern void ms(int);
extern unsigned char resp[];
extern void tx(char);
extern void txdec8(int);
extern void txhex(unsigned char);
extern void txhex16(unsigned int);
extern void crlf(void);

extern unsigned char RSSI;
extern unsigned char SNR;
extern unsigned char ST;

extern unsigned char green_led_timer;
extern unsigned char red_led_timer;

// RDS raw data
extern  unsigned char S[4];
extern  unsigned char A[2];
extern  unsigned char B[2];
extern  unsigned char C[2];
extern  unsigned char D[2];
extern  unsigned char O[2];

extern  unsigned int RDS_A;
extern  unsigned int RDS_B;
extern  unsigned int RDS_C;
extern  unsigned int RDS_D;

extern unsigned char CMD;
extern unsigned int ADDR;
extern unsigned int DATA;
//extern unsigned int EDATA;
//extern unsigned int FDATA;
extern unsigned char GROUP;
extern unsigned int rds_count;

//-----------------------------------------------------------------------------
//System clock configuration  see spreadsheet KW2012_Clocks.xls for details
//-----------------------------------------------------------------------------
#define PR2_PRELOAD 9u
#define CCP_DUTY  	5u
// with a TMR2 PR2 preload of 9 the RCLK clock will be 12,000,000/(PR2+1) = 1,200,000
//
// 1,200,000/36 = 33,333  = 0x8235
#define RCLK 		33333u
#define RCLK_DIV 	36u
//
// 1,200,000/16 ==> 75,000/2 ==> 37,500   0x927c
#define DCLK_FREQ 	 37500u

#define TMR1_PRELOAD_HI 209u
#define TMR1_PRELOAD_LO 32u

//-----------------------------------------------------------------------------
// C IO defines
#define RC0 	LATCbits.LATC0
//#define RC1 	LATCbits.LATC1
#define RC2 	LATCbits.LATC2
#define RC3 	LATCbits.LATC3
#define RC4 	LATCbits.LATC4
//#define RC5 	LATCbits.LATC5
#define RC6 	LATCbits.LATC6
//#define RC7 	LATCbits.LATC7

#define RED_LED	  	RC2   	/* Red Led and UP Push Button */
#define GREEN_LED 	RC4   	/* Green Led */
#define BLUE_LED  	RC6   	/* Blue Led, and FET output */
//-----------------------------------------------------------------------------
// Assembler
#define RINT  LATC,0,0		/* Si4705 interrupt output if enabled */
#define DFS   LATC,1,0		/* Digital Audio Frame Select */
#define RED   LATC,2,0   	/* Red Led and UP Push Button */
#define RST   LATC,3,0   	/* Si4705 Reset */
#define GREEN LATC,4,0   	/* Green Led */
#define DCLK  PORTC,5,0		/* Digital Audio Clock uses PWM output */
#define BLUE  LATC ,6,0   	/* Blue Led, and FET output */
#define DOUT  PORTC,7,0   	/* Digital Audio Data */
#define PIR1  0xf9e
#define aTMR2IF 0x01

#define W 0
#define F 1
#define ACCESS 0
#define BANKED 1
#define Z 2

#endif	/* LS2014_H */

