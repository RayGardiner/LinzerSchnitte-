//-----------------------------------------------------------------------------
//	KW2012  KlangWolke Receiver PCB  Hardware V 1.6
//  Ray Gardiner June 2012
//
#ifndef LS2014_H
#define	LS2014_H

//-----------------------------------------------------------------------------
// Configuration Bits 
//-----------------------------------------------------------------------------

		#pragma config USBDIV=OFF	   	// usb clock no divide
		#pragma config CPUDIV=NOCLKDIV  // cpu clock no divide

		#pragma config IESO=OFF			// internal oscillator switchover allowed
		#pragma config FCMEN=OFF		// fail safe clock monitor enabled
		#pragma config PCLKEN=ON		// primary clock enabled
		#pragma config PLLEN=ON 		// 4X pll   enabled 12 Mhz crystal
		#pragma config FOSC=HS			// FOSC=0010 external HS oscillator
	
		#pragma config BORV=19			// brown out 
		#pragma config BOREN=SBORDIS	// brown out reset in hardware only
		#pragma config PWRTEN=ON		// power up timer enabled

		#pragma config WDTPS=32768		// watchdog timer prescaler=1:32768
		#pragma config WDTEN=ON			// watchdog timer enabled

		#pragma config MCLRE=ON			// MCLR enabled RA3 input disabled
		#pragma config HFOFST=ON		// Fast start up bit

		#pragma config BBSIZ=OFF		// bootblock 1k
		#pragma config LVP=OFF			// single supply icsp disabled PGM pin
		#pragma config STVREN=ON		// stack overflow will cause reset

		#pragma config CP0=OFF			// block 0 not code protected
		#pragma config CP1=OFF			// block 1 not code protected

		#pragma config CPD=OFF			// data eeprom not protected
		#pragma config CPB=OFF			// boot block is code protected

		#pragma config WRT0=ON			// block 0 write protected
		#pragma config WRT1=OFF			// block 1 not write protected

		#pragma config WRTD=OFF			// data eeprom not write protected
		#pragma config WRTB=ON			// boot block write protected
		#pragma config WRTC=ON			// config registers write protected

		#pragma config EBTR1=OFF		// table read block 1 allowed
		#pragma config EBTR0=OFF		// table read block 0 allowed

		#pragma config XINST=OFF
		#pragma config EBTRB=OFF		//  boot block table read allowed
 

 
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



#endif