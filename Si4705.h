/* 
 * File:   Si4705.h
 * Author: ray@etheira.net
 *
 * Created on 8 June 2014, 5:49 PM
 */

#ifndef SI4705_H
#define	SI4705_H

#ifdef	__cplusplus
extern "C" {
#endif

#define Si4705_GP02 RC0		/* Si4705 interrupt output if enabled */
#define Si4705_DFS  RC1		/* Digital Audio Frame Select */
#define Si4705_RST  RC3    	/* Si4705 Reset */
#define Si4705_DCLK RC5		/* Digital Audio Clock uses PWM output */
#define Si4705_DOUT RC7   	/* Digital Audio Data */


    

extern void Si4705_I2C_INIT(void);
extern void Si4705_I2C_RESET(void);
extern int  Si4705_RSQ_STATUS(void);
extern void Si4705_POWER_UP(void);
extern void Si4705_SET_RCLK(void);
extern void Si4705_TUNE(unsigned int);
extern int  Si4705_STATUS_CHECK(void);
extern void Si4705_DIGITAL_AUDIO_SETUP(void);
extern int Si4705_TUNE_STATUS(void);
extern void Si4705_RDS_CONFIG(void);
extern void Si4705_RDS_STATUS(void);
extern void Si4705_SEEK(void);
extern void Si4705_SET_PROPERTY(unsigned int, int);
extern int Tune ( int,int);



#ifdef	__cplusplus
}
#endif

#endif	/* SI4705_H */

