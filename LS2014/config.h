/* 
 * File:   config.h
 * Author: ray
 *
 * Created on 8 June 2014, 7:24 PM
 */

#ifndef CONFIG_H
#define	CONFIG_H

#ifdef	__cplusplus
extern "C" {
#endif
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
		#pragma config BOREN=SBORDIS            // brown out reset in hardware only
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


#ifdef	__cplusplus
}
#endif

#endif	/* CONFIG_H */

