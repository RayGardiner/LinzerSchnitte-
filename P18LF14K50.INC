        LIST

;==========================================================================
;  MPASM PIC18LF14K50 processor include
; 
;  (c) Copyright 1999-2012 Microchip Technology, All rights reserved
;==========================================================================

        NOLIST

;==========================================================================
;  This header file defines configurations, registers, and other useful
;  bits of information for the PIC18LF14K50 microcontroller.  These names
;  are taken to match the data sheets as closely as possible.
;
;  Note that the processor must be selected before this file is included.
;  The processor may be selected the following ways:
;
;       1. Command line switch:
;               C:\MPASM MYFILE.ASM /PIC18LF14K50
;       2. LIST directive in the source file
;               LIST   P=PIC18LF14K50
;       3. Processor Type entry in the MPASM full-screen interface
;       4. Setting the processor in the MPLAB Project Dialog
;==========================================================================

;==========================================================================
;
;       Verify Processor
;
;==========================================================================
        IFNDEF __18LF14K50
           MESSG "Processor-header file mismatch.  Verify selected processor."
        ENDIF

;==========================================================================
;       18xxxx Family        EQUates
;==========================================================================
FSR0            EQU 0
FSR1            EQU 1
FSR2            EQU 2

FAST            EQU 1

W               EQU 0
A               EQU 0
ACCESS          EQU 0
BANKED          EQU 1
;==========================================================================

;==========================================================================
;       16Cxxx/17Cxxx Substitutions
;==========================================================================
  #define DDRA  TRISA      ; PIC17Cxxx SFR substitution
  #define DDRB  TRISB      ; PIC17Cxxx SFR substitution
  #define DDRC  TRISC      ; PIC17Cxxx SFR substitution
  #define DDRD  TRISD      ; PIC17Cxxx SFR substitution
  #define DDRE  TRISE      ; PIC17Cxxx SFR substitution

;==========================================================================
;
;       Register Definitions
;
;==========================================================================

;----- Register Files -----------------------------------------------------
UEP0             EQU  H'0F53'
UEP1             EQU  H'0F54'
UEP2             EQU  H'0F55'
UEP3             EQU  H'0F56'
UEP4             EQU  H'0F57'
UEP5             EQU  H'0F58'
UEP6             EQU  H'0F59'
UEP7             EQU  H'0F5A'
UEIE             EQU  H'0F5B'
UADDR            EQU  H'0F5C'
UFRML            EQU  H'0F5D'
UFRMH            EQU  H'0F5E'
UEIR             EQU  H'0F5F'
UIE              EQU  H'0F60'
UCFG             EQU  H'0F61'
UIR              EQU  H'0F62'
USTAT            EQU  H'0F63'
UCON             EQU  H'0F64'
SRCON0           EQU  H'0F68'
SRCON1           EQU  H'0F69'
CM2CON0          EQU  H'0F6B'
CM2CON1          EQU  H'0F6C'
CM1CON0          EQU  H'0F6D'
SSPMSK           EQU  H'0F6F'
SLRCON           EQU  H'0F76'
WPUA             EQU  H'0F77'
WPUB             EQU  H'0F78'
IOCA             EQU  H'0F79'
IOCB             EQU  H'0F7A'
ANSEL            EQU  H'0F7E'
ANSELH           EQU  H'0F7F'
PORTA            EQU  H'0F80'
PORTB            EQU  H'0F81'
PORTC            EQU  H'0F82'
LATA             EQU  H'0F89'
LATB             EQU  H'0F8A'
LATC             EQU  H'0F8B'
DDRA             EQU  H'0F92'
TRISA            EQU  H'0F92'
DDRB             EQU  H'0F93'
TRISB            EQU  H'0F93'
DDRC             EQU  H'0F94'
TRISC            EQU  H'0F94'
OSCTUNE          EQU  H'0F9B'
PIE1             EQU  H'0F9D'
PIR1             EQU  H'0F9E'
IPR1             EQU  H'0F9F'
PIE2             EQU  H'0FA0'
PIR2             EQU  H'0FA1'
IPR2             EQU  H'0FA2'
EECON1           EQU  H'0FA6'
EECON2           EQU  H'0FA7'
EEDATA           EQU  H'0FA8'
EEADR            EQU  H'0FA9'
RCSTA            EQU  H'0FAB'
TXSTA            EQU  H'0FAC'
TXREG            EQU  H'0FAD'
RCREG            EQU  H'0FAE'
SPBRG            EQU  H'0FAF'
SPBRGH           EQU  H'0FB0'
T3CON            EQU  H'0FB1'
TMR3             EQU  H'0FB2'
TMR3L            EQU  H'0FB2'
TMR3H            EQU  H'0FB3'
ECCP1AS          EQU  H'0FB6'
PWM1CON          EQU  H'0FB7'
BAUDCON          EQU  H'0FB8'
BAUDCTL          EQU  H'0FB8'
PSTRCON          EQU  H'0FB9'
REFCON0          EQU  H'0FBA'
VREFCON0         EQU  H'0FBA'
REFCON1          EQU  H'0FBB'
VREFCON1         EQU  H'0FBB'
REFCON2          EQU  H'0FBC'
VREFCON2         EQU  H'0FBC'
CCP1CON          EQU  H'0FBD'
CCPR1            EQU  H'0FBE'
CCPR1L           EQU  H'0FBE'
CCPR1H           EQU  H'0FBF'
ADCON2           EQU  H'0FC0'
ADCON1           EQU  H'0FC1'
ADCON0           EQU  H'0FC2'
ADRES            EQU  H'0FC3'
ADRESL           EQU  H'0FC3'
ADRESH           EQU  H'0FC4'
SSPCON2          EQU  H'0FC5'
SSPCON1          EQU  H'0FC6'
SSPSTAT          EQU  H'0FC7'
SSPADD           EQU  H'0FC8'
SSPBUF           EQU  H'0FC9'
T2CON            EQU  H'0FCA'
PR2              EQU  H'0FCB'
TMR2             EQU  H'0FCC'
T1CON            EQU  H'0FCD'
TMR1             EQU  H'0FCE'
TMR1L            EQU  H'0FCE'
TMR1H            EQU  H'0FCF'
RCON             EQU  H'0FD0'
WDTCON           EQU  H'0FD1'
OSCCON2          EQU  H'0FD2'
OSCCON           EQU  H'0FD3'
T0CON            EQU  H'0FD5'
TMR0             EQU  H'0FD6'
TMR0L            EQU  H'0FD6'
TMR0H            EQU  H'0FD7'
STATUS           EQU  H'0FD8'
FSR2L            EQU  H'0FD9'
FSR2H            EQU  H'0FDA'
PLUSW2           EQU  H'0FDB'
PREINC2          EQU  H'0FDC'
POSTDEC2         EQU  H'0FDD'
POSTINC2         EQU  H'0FDE'
INDF2            EQU  H'0FDF'
BSR              EQU  H'0FE0'
FSR1L            EQU  H'0FE1'
FSR1H            EQU  H'0FE2'
PLUSW1           EQU  H'0FE3'
PREINC1          EQU  H'0FE4'
POSTDEC1         EQU  H'0FE5'
POSTINC1         EQU  H'0FE6'
INDF1            EQU  H'0FE7'
WREG             EQU  H'0FE8'
FSR0L            EQU  H'0FE9'
FSR0H            EQU  H'0FEA'
PLUSW0           EQU  H'0FEB'
PREINC0          EQU  H'0FEC'
POSTDEC0         EQU  H'0FED'
POSTINC0         EQU  H'0FEE'
INDF0            EQU  H'0FEF'
INTCON3          EQU  H'0FF0'
INTCON2          EQU  H'0FF1'
INTCON           EQU  H'0FF2'
PROD             EQU  H'0FF3'
PRODL            EQU  H'0FF3'
PRODH            EQU  H'0FF4'
TABLAT           EQU  H'0FF5'
TBLPTR           EQU  H'0FF6'
TBLPTRL          EQU  H'0FF6'
TBLPTRH          EQU  H'0FF7'
TBLPTRU          EQU  H'0FF8'
PC               EQU  H'0FF9'
PCL              EQU  H'0FF9'
PCLATH           EQU  H'0FFA'
PCLATU           EQU  H'0FFB'
STKPTR           EQU  H'0FFC'
TOS              EQU  H'0FFD'
TOSL             EQU  H'0FFD'
TOSH             EQU  H'0FFE'
TOSU             EQU  H'0FFF'

;----- UEP0 Bits -----------------------------------------------------
EPSTALL          EQU  H'0000'
EPINEN           EQU  H'0001'
EPOUTEN          EQU  H'0002'
EPCONDIS         EQU  H'0003'
EPHSHK           EQU  H'0004'


;----- UEP1 Bits -----------------------------------------------------
EPSTALL          EQU  H'0000'
EPINEN           EQU  H'0001'
EPOUTEN          EQU  H'0002'
EPCONDIS         EQU  H'0003'
EPHSHK           EQU  H'0004'


;----- UEP2 Bits -----------------------------------------------------
EPSTALL          EQU  H'0000'
EPINEN           EQU  H'0001'
EPOUTEN          EQU  H'0002'
EPCONDIS         EQU  H'0003'
EPHSHK           EQU  H'0004'


;----- UEP3 Bits -----------------------------------------------------
EPSTALL          EQU  H'0000'
EPINEN           EQU  H'0001'
EPOUTEN          EQU  H'0002'
EPCONDIS         EQU  H'0003'
EPHSHK           EQU  H'0004'


;----- UEP4 Bits -----------------------------------------------------
EPSTALL          EQU  H'0000'
EPINEN           EQU  H'0001'
EPOUTEN          EQU  H'0002'
EPCONDIS         EQU  H'0003'
EPHSHK           EQU  H'0004'


;----- UEP5 Bits -----------------------------------------------------
EPSTALL          EQU  H'0000'
EPINEN           EQU  H'0001'
EPOUTEN          EQU  H'0002'
EPCONDIS         EQU  H'0003'
EPHSHK           EQU  H'0004'


;----- UEP6 Bits -----------------------------------------------------
EPSTALL          EQU  H'0000'
EPINEN           EQU  H'0001'
EPOUTEN          EQU  H'0002'
EPCONDIS         EQU  H'0003'
EPHSHK           EQU  H'0004'


;----- UEP7 Bits -----------------------------------------------------
EPSTALL          EQU  H'0000'
EPINEN           EQU  H'0001'
EPOUTEN          EQU  H'0002'
EPCONDIS         EQU  H'0003'
EPHSHK           EQU  H'0004'


;----- UEIE Bits -----------------------------------------------------
PIDEE            EQU  H'0000'
CRC5EE           EQU  H'0001'
CRC16EE          EQU  H'0002'
DFN8EE           EQU  H'0003'
BTOEE            EQU  H'0004'
BTSEE            EQU  H'0007'


;----- UADDR Bits -----------------------------------------------------
ADDR0            EQU  H'0000'
ADDR1            EQU  H'0001'
ADDR2            EQU  H'0002'
ADDR3            EQU  H'0003'
ADDR4            EQU  H'0004'
ADDR5            EQU  H'0005'
ADDR6            EQU  H'0006'


;----- UFRML Bits -----------------------------------------------------
FRM0             EQU  H'0000'
FRM1             EQU  H'0001'
FRM2             EQU  H'0002'
FRM3             EQU  H'0003'
FRM4             EQU  H'0004'
FRM5             EQU  H'0005'
FRM6             EQU  H'0006'
FRM7             EQU  H'0007'


;----- UFRMH Bits -----------------------------------------------------
FRM8             EQU  H'0000'
FRM9             EQU  H'0001'
FRM10            EQU  H'0002'


;----- UEIR Bits -----------------------------------------------------
PIDEF            EQU  H'0000'
CRC5EF           EQU  H'0001'
CRC16EF          EQU  H'0002'
DFN8EF           EQU  H'0003'
BTOEF            EQU  H'0004'
BTSEF            EQU  H'0007'


;----- UIE Bits -----------------------------------------------------
URSTIE           EQU  H'0000'
UERRIE           EQU  H'0001'
ACTVIE           EQU  H'0002'
TRNIE            EQU  H'0003'
IDLEIE           EQU  H'0004'
STALLIE          EQU  H'0005'
SOFIE            EQU  H'0006'


;----- UCFG Bits -----------------------------------------------------
PPB0             EQU  H'0000'
PPB1             EQU  H'0001'
FSEN             EQU  H'0002'
UPUEN            EQU  H'0004'
UTEYE            EQU  H'0007'


;----- UIR Bits -----------------------------------------------------
URSTIF           EQU  H'0000'
UERRIF           EQU  H'0001'
ACTVIF           EQU  H'0002'
TRNIF            EQU  H'0003'
IDLEIF           EQU  H'0004'
STALLIF          EQU  H'0005'
SOFIF            EQU  H'0006'


;----- USTAT Bits -----------------------------------------------------
PPBI             EQU  H'0001'
DIR              EQU  H'0002'
ENDP0            EQU  H'0003'
ENDP1            EQU  H'0004'
ENDP2            EQU  H'0005'
ENDP3            EQU  H'0006'


;----- UCON Bits -----------------------------------------------------
SUSPND           EQU  H'0001'
RESUME           EQU  H'0002'
USBEN            EQU  H'0003'
PKTDIS           EQU  H'0004'
SE0              EQU  H'0005'
PPBRST           EQU  H'0006'


;----- SRCON0 Bits -----------------------------------------------------
SRPR             EQU  H'0000'
SRPS             EQU  H'0001'
SRNQEN           EQU  H'0002'
SRQEN            EQU  H'0003'
SRLEN            EQU  H'0007'

SRCLK0           EQU  H'0004'
SRCLK1           EQU  H'0005'
SRCLK2           EQU  H'0006'


;----- SRCON1 Bits -----------------------------------------------------
SRRC1E           EQU  H'0000'
SRRC2E           EQU  H'0001'
SRRCKE           EQU  H'0002'
SRRPE            EQU  H'0003'
SRSC1E           EQU  H'0004'
SRSC2E           EQU  H'0005'
SRSCKE           EQU  H'0006'
SRSPE            EQU  H'0007'


;----- CM2CON0 Bits -----------------------------------------------------
C2R              EQU  H'0002'
C2SP             EQU  H'0003'
C2POL            EQU  H'0004'
C2OE             EQU  H'0005'
C2OUT            EQU  H'0006'
C2ON             EQU  H'0007'

C2CH0            EQU  H'0000'
C2CH1            EQU  H'0001'


;----- CM2CON1 Bits -----------------------------------------------------
C2SYNC           EQU  H'0000'
C1SYNC           EQU  H'0001'
C2HYS            EQU  H'0002'
C1HYS            EQU  H'0003'
C2RSEL           EQU  H'0004'
C1RSEL           EQU  H'0005'
MC2OUT           EQU  H'0006'
MC1OUT           EQU  H'0007'


;----- CM1CON0 Bits -----------------------------------------------------
C1R              EQU  H'0002'
C1SP             EQU  H'0003'
C1POL            EQU  H'0004'
C1OE             EQU  H'0005'
C1OUT            EQU  H'0006'
C1ON             EQU  H'0007'

C1CH0            EQU  H'0000'
C1CH1            EQU  H'0001'


;----- SSPMSK Bits -----------------------------------------------------
MSK0             EQU  H'0000'
MSK1             EQU  H'0001'
MSK2             EQU  H'0002'
MSK3             EQU  H'0003'
MSK4             EQU  H'0004'
MSK5             EQU  H'0005'
MSK6             EQU  H'0006'
MSK7             EQU  H'0007'


;----- SLRCON Bits -----------------------------------------------------
SLRA             EQU  H'0000'
SLRB             EQU  H'0001'
SLRC             EQU  H'0002'


;----- WPUA Bits -----------------------------------------------------
WPUA3            EQU  H'0003'
WPUA4            EQU  H'0004'
WPUA5            EQU  H'0005'


;----- WPUB Bits -----------------------------------------------------
WPUB4            EQU  H'0004'
WPUB5            EQU  H'0005'
WPUB6            EQU  H'0006'
WPUB7            EQU  H'0007'


;----- IOCA Bits -----------------------------------------------------
IOCA0            EQU  H'0000'
IOCA1            EQU  H'0001'
IOCA3            EQU  H'0003'
IOCA4            EQU  H'0004'
IOCA5            EQU  H'0005'


;----- IOCB Bits -----------------------------------------------------
IOCB4            EQU  H'0004'
IOCB5            EQU  H'0005'
IOCB6            EQU  H'0006'
IOCB7            EQU  H'0007'


;----- ANSEL Bits -----------------------------------------------------
ANS3             EQU  H'0003'
ANS4             EQU  H'0004'
ANS5             EQU  H'0005'
ANS6             EQU  H'0006'
ANS7             EQU  H'0007'


;----- ANSELH Bits -----------------------------------------------------
ANS8             EQU  H'0000'
ANS9             EQU  H'0001'
ANS10            EQU  H'0002'
ANS11            EQU  H'0003'


;----- PORTA Bits -----------------------------------------------------
RA0              EQU  H'0000'
RA1              EQU  H'0001'
RA3              EQU  H'0003'
RA4              EQU  H'0004'
RA5              EQU  H'0005'

AN3              EQU  H'0004'

OSC2             EQU  H'0004'
OSC1             EQU  H'0005'

CLKOUT           EQU  H'0004'
CLKIN            EQU  H'0005'


;----- PORTB Bits -----------------------------------------------------
RB4              EQU  H'0004'
RB5              EQU  H'0005'
RB6              EQU  H'0006'
RB7              EQU  H'0007'

SDI              EQU  H'0004'
RX               EQU  H'0005'
SCL              EQU  H'0006'
TX               EQU  H'0007'

SDA              EQU  H'0004'
; DT is a reserved word
; DT               EQU  H'0005'
SCK              EQU  H'0006'
CK               EQU  H'0007'

AN10             EQU  H'0004'
AN11             EQU  H'0005'


;----- PORTC Bits -----------------------------------------------------
RC0              EQU  H'0000'
RC1              EQU  H'0001'
RC2              EQU  H'0002'
RC3              EQU  H'0003'
RC4              EQU  H'0004'
RC5              EQU  H'0005'
RC6              EQU  H'0006'
RC7              EQU  H'0007'

AN4              EQU  H'0000'
AN5              EQU  H'0001'
AN6              EQU  H'0002'
AN7              EQU  H'0003'
AN8              EQU  H'0006'
AN9              EQU  H'0007'

C12INP           EQU  H'0000'
C12IN1M          EQU  H'0001'
C12IN2M          EQU  H'0002'
C12IN3M          EQU  H'0003'
C12OUT           EQU  H'0004'
CCP1             EQU  H'0005'
NOT_SS           EQU  H'0006'

INT0             EQU  H'0000'
INT1             EQU  H'0001'
INT2             EQU  H'0002'
PGM              EQU  H'0003'
SRQ              EQU  H'0004'
T0CKI            EQU  H'0005'
T13CKI           EQU  H'0006'
T1OSCO           EQU  H'0007'

VREFP            EQU  H'0000'
VREFM            EQU  H'0001'
CVREF            EQU  H'0002'
T1OSCI           EQU  H'0006'

P1D              EQU  H'0002'
P1C              EQU  H'0003'
P1B              EQU  H'0004'
P1A              EQU  H'0005'
SS               EQU  H'0006'
SDO              EQU  H'0007'


;----- LATA Bits -----------------------------------------------------
LATA4            EQU  H'0004'
LATA5            EQU  H'0005'


;----- LATB Bits -----------------------------------------------------
LATB4            EQU  H'0004'
LATB5            EQU  H'0005'
LATB6            EQU  H'0006'
LATB7            EQU  H'0007'


;----- LATC Bits -----------------------------------------------------
LATC0            EQU  H'0000'
LATC1            EQU  H'0001'
LATC2            EQU  H'0002'
LATC3            EQU  H'0003'
LATC4            EQU  H'0004'
LATC5            EQU  H'0005'
LATC6            EQU  H'0006'
LATC7            EQU  H'0007'


;----- DDRA Bits -----------------------------------------------------
TRISA4           EQU  H'0004'
TRISA5           EQU  H'0005'

RA4              EQU  H'0004'
RA5              EQU  H'0005'


;----- TRISA Bits -----------------------------------------------------
TRISA4           EQU  H'0004'
TRISA5           EQU  H'0005'

RA4              EQU  H'0004'
RA5              EQU  H'0005'


;----- DDRB Bits -----------------------------------------------------
TRISB4           EQU  H'0004'
TRISB5           EQU  H'0005'
TRISB6           EQU  H'0006'
TRISB7           EQU  H'0007'

RB4              EQU  H'0004'
RB5              EQU  H'0005'
RB6              EQU  H'0006'
RB7              EQU  H'0007'


;----- TRISB Bits -----------------------------------------------------
TRISB4           EQU  H'0004'
TRISB5           EQU  H'0005'
TRISB6           EQU  H'0006'
TRISB7           EQU  H'0007'

RB4              EQU  H'0004'
RB5              EQU  H'0005'
RB6              EQU  H'0006'
RB7              EQU  H'0007'


;----- DDRC Bits -----------------------------------------------------
TRISC0           EQU  H'0000'
TRISC1           EQU  H'0001'
TRISC2           EQU  H'0002'
TRISC3           EQU  H'0003'
TRISC4           EQU  H'0004'
TRISC5           EQU  H'0005'
TRISC6           EQU  H'0006'
TRISC7           EQU  H'0007'

RC0              EQU  H'0000'
RC1              EQU  H'0001'
RC2              EQU  H'0002'
RC3              EQU  H'0003'
RC4              EQU  H'0004'
RC5              EQU  H'0005'
RC6              EQU  H'0006'
RC7              EQU  H'0007'


;----- TRISC Bits -----------------------------------------------------
TRISC0           EQU  H'0000'
TRISC1           EQU  H'0001'
TRISC2           EQU  H'0002'
TRISC3           EQU  H'0003'
TRISC4           EQU  H'0004'
TRISC5           EQU  H'0005'
TRISC6           EQU  H'0006'
TRISC7           EQU  H'0007'

RC0              EQU  H'0000'
RC1              EQU  H'0001'
RC2              EQU  H'0002'
RC3              EQU  H'0003'
RC4              EQU  H'0004'
RC5              EQU  H'0005'
RC6              EQU  H'0006'
RC7              EQU  H'0007'


;----- OSCTUNE Bits -----------------------------------------------------
SPLLEN           EQU  H'0006'
INTSRC           EQU  H'0007'

TUN0             EQU  H'0000'
TUN1             EQU  H'0001'
TUN2             EQU  H'0002'
TUN3             EQU  H'0003'
TUN4             EQU  H'0004'
TUN5             EQU  H'0005'


;----- PIE1 Bits -----------------------------------------------------
TMR1IE           EQU  H'0000'
TMR2IE           EQU  H'0001'
CCP1IE           EQU  H'0002'
SSPIE            EQU  H'0003'
TXIE             EQU  H'0004'
RCIE             EQU  H'0005'
ADIE             EQU  H'0006'


;----- PIR1 Bits -----------------------------------------------------
TMR1IF           EQU  H'0000'
TMR2IF           EQU  H'0001'
CCP1IF           EQU  H'0002'
SSPIF            EQU  H'0003'
TXIF             EQU  H'0004'
RCIF             EQU  H'0005'
ADIF             EQU  H'0006'


;----- IPR1 Bits -----------------------------------------------------
TMR1IP           EQU  H'0000'
TMR2IP           EQU  H'0001'
CCP1IP           EQU  H'0002'
SSPIP            EQU  H'0003'
TXIP             EQU  H'0004'
RCIP             EQU  H'0005'
ADIP             EQU  H'0006'


;----- PIE2 Bits -----------------------------------------------------
TMR3IE           EQU  H'0001'
USBIE            EQU  H'0002'
BCLIE            EQU  H'0003'
EEIE             EQU  H'0004'
C2IE             EQU  H'0005'
C1IE             EQU  H'0006'
OSCFIE           EQU  H'0007'


;----- PIR2 Bits -----------------------------------------------------
TMR3IF           EQU  H'0001'
USBIF            EQU  H'0002'
BCLIF            EQU  H'0003'
EEIF             EQU  H'0004'
C2IF             EQU  H'0005'
C1IF             EQU  H'0006'
OSCFIF           EQU  H'0007'


;----- IPR2 Bits -----------------------------------------------------
TMR3IP           EQU  H'0001'
USBIP            EQU  H'0002'
BCLIP            EQU  H'0003'
EEIP             EQU  H'0004'
C2IP             EQU  H'0005'
C1IP             EQU  H'0006'
OSCFIP           EQU  H'0007'


;----- EECON1 Bits -----------------------------------------------------
RD               EQU  H'0000'
WR               EQU  H'0001'
WREN             EQU  H'0002'
WRERR            EQU  H'0003'
FREE             EQU  H'0004'
CFGS             EQU  H'0006'
EEPGD            EQU  H'0007'


;----- EEADR Bits -----------------------------------------------------
EEADR0           EQU  H'0000'
EEADR1           EQU  H'0001'
EEADR2           EQU  H'0002'
EEADR3           EQU  H'0003'
EEADR4           EQU  H'0004'
EEADR5           EQU  H'0005'
EEADR6           EQU  H'0006'
EEADR7           EQU  H'0007'


;----- RCSTA Bits -----------------------------------------------------
RX9D             EQU  H'0000'
OERR             EQU  H'0001'
FERR             EQU  H'0002'
ADDEN            EQU  H'0003'
CREN             EQU  H'0004'
SREN             EQU  H'0005'
RX9              EQU  H'0006'
SPEN             EQU  H'0007'

ADEN             EQU  H'0003'


;----- TXSTA Bits -----------------------------------------------------
TX9D             EQU  H'0000'
TRMT             EQU  H'0001'
BRGH             EQU  H'0002'
SENDB            EQU  H'0003'
SYNC             EQU  H'0004'
TXEN             EQU  H'0005'
TX9              EQU  H'0006'
CSRC             EQU  H'0007'


;----- T3CON Bits -----------------------------------------------------
TMR3ON           EQU  H'0000'
TMR3CS           EQU  H'0001'
NOT_T3SYNC       EQU  H'0002'
T3CCP1           EQU  H'0003'
RD16             EQU  H'0007'

T3SYNC           EQU  H'0002'
T3CKPS0          EQU  H'0004'
T3CKPS1          EQU  H'0005'


;----- ECCP1AS Bits -----------------------------------------------------
ECCPASE          EQU  H'0007'

PSSBD0           EQU  H'0000'
PSSBD1           EQU  H'0001'
PSSAC0           EQU  H'0002'
PSSAC1           EQU  H'0003'
ECCPAS0          EQU  H'0004'
ECCPAS1          EQU  H'0005'
ECCPAS2          EQU  H'0006'


;----- PWM1CON Bits -----------------------------------------------------
PRSEN            EQU  H'0007'

PDC0             EQU  H'0000'
PDC1             EQU  H'0001'
PDC2             EQU  H'0002'
PDC3             EQU  H'0003'
PDC4             EQU  H'0004'
PDC5             EQU  H'0005'
PDC6             EQU  H'0006'


;----- BAUDCON Bits -----------------------------------------------------
ABDEN            EQU  H'0000'
WUE              EQU  H'0001'
BRG16            EQU  H'0003'
CKTXP            EQU  H'0004'
DTRXP            EQU  H'0005'
RCIDL            EQU  H'0006'
ABDOVF           EQU  H'0007'

SCKP             EQU  H'0004'


;----- BAUDCTL Bits -----------------------------------------------------
ABDEN            EQU  H'0000'
WUE              EQU  H'0001'
BRG16            EQU  H'0003'
CKTXP            EQU  H'0004'
DTRXP            EQU  H'0005'
RCIDL            EQU  H'0006'
ABDOVF           EQU  H'0007'

SCKP             EQU  H'0004'


;----- PSTRCON Bits -----------------------------------------------------
STRA             EQU  H'0000'
STRB             EQU  H'0001'
STRC             EQU  H'0002'
STRD             EQU  H'0003'
STRSYNC          EQU  H'0004'


;----- REFCON0 Bits -----------------------------------------------------
FVR1S0           EQU  H'0004'
FVR1S1           EQU  H'0005'
FVR1ST           EQU  H'0006'
FVR1EN           EQU  H'0007'


;----- VREFCON0 Bits -----------------------------------------------------
FVR1S0           EQU  H'0004'
FVR1S1           EQU  H'0005'
FVR1ST           EQU  H'0006'
FVR1EN           EQU  H'0007'


;----- REFCON1 Bits -----------------------------------------------------
D1NSS            EQU  H'0000'
DAC1OE           EQU  H'0005'
D1LPS            EQU  H'0006'
D1EN             EQU  H'0007'

D1PSS0           EQU  H'0002'
D1PSS1           EQU  H'0003'


;----- VREFCON1 Bits -----------------------------------------------------
D1NSS            EQU  H'0000'
DAC1OE           EQU  H'0005'
D1LPS            EQU  H'0006'
D1EN             EQU  H'0007'

D1PSS0           EQU  H'0002'
D1PSS1           EQU  H'0003'


;----- REFCON2 Bits -----------------------------------------------------
DAC1R0           EQU  H'0000'
DAC1R1           EQU  H'0001'
DAC1R2           EQU  H'0002'
DAC1R3           EQU  H'0003'
DAC1R4           EQU  H'0004'


;----- VREFCON2 Bits -----------------------------------------------------
DAC1R0           EQU  H'0000'
DAC1R1           EQU  H'0001'
DAC1R2           EQU  H'0002'
DAC1R3           EQU  H'0003'
DAC1R4           EQU  H'0004'


;----- CCP1CON Bits -----------------------------------------------------
CCP1M0           EQU  H'0000'
CCP1M1           EQU  H'0001'
CCP1M2           EQU  H'0002'
CCP1M3           EQU  H'0003'
DC1B0            EQU  H'0004'
DC1B1            EQU  H'0005'
P1M0             EQU  H'0006'
P1M1             EQU  H'0007'


;----- ADCON2 Bits -----------------------------------------------------
ADFM             EQU  H'0007'

ADCS0            EQU  H'0000'
ADCS1            EQU  H'0001'
ADCS2            EQU  H'0002'
ACQT0            EQU  H'0003'
ACQT1            EQU  H'0004'
ACQT2            EQU  H'0005'


;----- ADCON1 Bits -----------------------------------------------------
NVCFG0           EQU  H'0000'
NVCFG1           EQU  H'0001'
PVCFG0           EQU  H'0002'
PVCFG1           EQU  H'0003'


;----- ADCON0 Bits -----------------------------------------------------
ADON             EQU  H'0000'
GO_NOT_DONE      EQU  H'0001'

GO               EQU  H'0001'
CHS0             EQU  H'0002'
CHS1             EQU  H'0003'
CHS2             EQU  H'0004'
CHS3             EQU  H'0005'

DONE             EQU  H'0001'

NOT_DONE         EQU  H'0001'

GO_DONE          EQU  H'0001'


;----- SSPCON2 Bits -----------------------------------------------------
SEN              EQU  H'0000'
RSEN             EQU  H'0001'
PEN              EQU  H'0002'
RCEN             EQU  H'0003'
ACKEN            EQU  H'0004'
ACKDT            EQU  H'0005'
ACKSTAT          EQU  H'0006'
GCEN             EQU  H'0007'


;----- SSPCON1 Bits -----------------------------------------------------
CKP              EQU  H'0004'
SSPEN            EQU  H'0005'
SSPOV            EQU  H'0006'
WCOL             EQU  H'0007'

SSPM0            EQU  H'0000'
SSPM1            EQU  H'0001'
SSPM2            EQU  H'0002'
SSPM3            EQU  H'0003'


;----- SSPSTAT Bits -----------------------------------------------------
BF               EQU  H'0000'
UA               EQU  H'0001'
R_NOT_W          EQU  H'0002'
S                EQU  H'0003'
P                EQU  H'0004'
D_NOT_A          EQU  H'0005'
CKE              EQU  H'0006'
SMP              EQU  H'0007'

R                EQU  H'0002'
D                EQU  H'0005'


NOT_W            EQU  H'0002'
NOT_A            EQU  H'0005'

R_W              EQU  H'0002'
D_A              EQU  H'0005'

NOT_WRITE        EQU  H'0002'
NOT_ADDRESS      EQU  H'0005'


;----- T2CON Bits -----------------------------------------------------
TMR2ON           EQU  H'0002'

T2CKPS0          EQU  H'0000'
T2CKPS1          EQU  H'0001'
T2OUTPS0         EQU  H'0003'
T2OUTPS1         EQU  H'0004'
T2OUTPS2         EQU  H'0005'
T2OUTPS3         EQU  H'0006'


;----- T1CON Bits -----------------------------------------------------
TMR1ON           EQU  H'0000'
TMR1CS           EQU  H'0001'
NOT_T1SYNC       EQU  H'0002'
T1OSCEN          EQU  H'0003'
T1RUN            EQU  H'0006'
RD16             EQU  H'0007'

T1SYNC           EQU  H'0002'
T1CKPS0          EQU  H'0004'
T1CKPS1          EQU  H'0005'


;----- RCON Bits -----------------------------------------------------
NOT_BOR          EQU  H'0000'
NOT_POR          EQU  H'0001'
NOT_PD           EQU  H'0002'
NOT_TO           EQU  H'0003'
NOT_RI           EQU  H'0004'
SBOREN           EQU  H'0006'
IPEN             EQU  H'0007'

BOR              EQU  H'0000'
POR              EQU  H'0001'
PD               EQU  H'0002'
TO               EQU  H'0003'
RI               EQU  H'0004'


;----- WDTCON Bits -----------------------------------------------------
SWDTEN           EQU  H'0000'

SWDTE            EQU  H'0000'


;----- OSCCON2 Bits -----------------------------------------------------
LFIOFS           EQU  H'0000'
HFIOFL           EQU  H'0001'
PRI_SD           EQU  H'0002'


;----- OSCCON Bits -----------------------------------------------------
SCS0             EQU  H'0000'
SCS1             EQU  H'0001'
IOFS             EQU  H'0002'
OSTS             EQU  H'0003'
IDLEN            EQU  H'0007'

IRCF0            EQU  H'0004'
IRCF1            EQU  H'0005'
IRCF2            EQU  H'0006'


;----- T0CON Bits -----------------------------------------------------
PSA              EQU  H'0003'
T0SE             EQU  H'0004'
T0CS             EQU  H'0005'
T08BIT           EQU  H'0006'
TMR0ON           EQU  H'0007'

T0PS0            EQU  H'0000'
T0PS1            EQU  H'0001'
T0PS2            EQU  H'0002'


;----- STATUS Bits -----------------------------------------------------
C                EQU  H'0000'
DC               EQU  H'0001'
Z                EQU  H'0002'
OV               EQU  H'0003'
N                EQU  H'0004'


;----- INTCON3 Bits -----------------------------------------------------
INT1IF           EQU  H'0000'
INT2IF           EQU  H'0001'
INT1IE           EQU  H'0003'
INT2IE           EQU  H'0004'
INT1IP           EQU  H'0006'
INT2IP           EQU  H'0007'

INT1F            EQU  H'0000'
INT2F            EQU  H'0001'
INT1E            EQU  H'0003'
INT2E            EQU  H'0004'
INT1P            EQU  H'0006'
INT2P            EQU  H'0007'


;----- INTCON2 Bits -----------------------------------------------------
RABIP            EQU  H'0000'
TMR0IP           EQU  H'0002'
INTEDG2          EQU  H'0004'
INTEDG1          EQU  H'0005'
INTEDG0          EQU  H'0006'
NOT_RABPU        EQU  H'0007'

RABPU            EQU  H'0007'


;----- INTCON Bits -----------------------------------------------------
RABIF            EQU  H'0000'
INT0IF           EQU  H'0001'
TMR0IF           EQU  H'0002'
RABIE            EQU  H'0003'
INT0IE           EQU  H'0004'
TMR0IE           EQU  H'0005'
PEIE_GIEL        EQU  H'0006'
GIE_GIEH         EQU  H'0007'

INT0F            EQU  H'0001'
T0IF             EQU  H'0002'
INT0E            EQU  H'0004'
T0IE             EQU  H'0005'
PEIE             EQU  H'0006'
GIE              EQU  H'0007'

GIEL             EQU  H'0006'
GIEH             EQU  H'0007'


;----- STKPTR Bits -----------------------------------------------------
STKUNF           EQU  H'0006'
STKFUL           EQU  H'0007'

SP0              EQU  H'0000'
SP1              EQU  H'0001'
SP2              EQU  H'0002'
SP3              EQU  H'0003'
SP4              EQU  H'0004'
STKOVF           EQU  H'0007'



;==========================================================================
;
;       RAM Definitions
;
;==========================================================================
       __MAXRAM  H'0FFF'
       __BADRAM  H'0300'-H'0F52'
       __BADRAM  H'0F65'-H'0F67'
       __BADRAM  H'0F6A'
       __BADRAM  H'0F6E'
       __BADRAM  H'0F70'-H'0F75'
       __BADRAM  H'0F7B'-H'0F7D'
       __BADRAM  H'0F83'-H'0F88'
       __BADRAM  H'0F8C'-H'0F91'
       __BADRAM  H'0F95'-H'0F9A'
       __BADRAM  H'0F9C'
       __BADRAM  H'0FA3'-H'0FA5'
       __BADRAM  H'0FAA'
       __BADRAM  H'0FB4'-H'0FB5'
       __BADRAM  H'0FD4'

;==========================================================================
;
;   IMPORTANT: For the PIC18 devices, the __CONFIG directive has been
;              superseded by the CONFIG directive.  The following settings
;              are available for this device.
;
;   CPU System Clock Selection bits:
;     CPUDIV = NOCLKDIV    No CPU System Clock divide
;     CPUDIV = CLKDIV2     CPU System Clock divided by 2
;     CPUDIV = CLKDIV3     CPU System Clock divided by 3
;     CPUDIV = CLKDIV4     CPU System Clock divided by 4
;
;   USB Clock Selection bit:
;     USBDIV = OFF         USB clock comes directly from the OSC1/OSC2 oscillator block; no divide
;     USBDIV = ON          USB clock comes from the OSC1/OSC2 divided by 2
;
;   Oscillator Selection bits:
;     FOSC = LP            LP oscillator
;     FOSC = XT            XT oscillator
;     FOSC = HS            HS oscillator
;     FOSC = ERCCLKOUT     External RC oscillator, CLKOUT function on OSC2
;     FOSC = ECCLKOUTH     EC, CLKOUT function on OSC2 (high)
;     FOSC = ECH           EC (high)
;     FOSC = ERC           External RC oscillator
;     FOSC = IRC           Internal RC oscillator
;     FOSC = IRCCLKOUT     Internal RC oscillator, CLKOUT function on OSC2
;     FOSC = ECCLKOUTM     EC, CLKOUT function on OSC2 (medium)
;     FOSC = ECM           EC (medium)
;     FOSC = ECCLKOUTL     EC, CLKOUT function on OSC2 (low)
;     FOSC = ECL           EC (low)
;
;   4 X PLL Enable bit:
;     PLLEN = OFF          PLL is under software control
;     PLLEN = ON           Oscillator multiplied by 4
;
;   Primary Clock Enable bit:
;     PCLKEN = OFF         Primary clock is under software control
;     PCLKEN = ON          Primary clock enabled
;
;   Fail-Safe Clock Monitor Enable:
;     FCMEN = OFF          Fail-Safe Clock Monitor disabled
;     FCMEN = ON           Fail-Safe Clock Monitor enabled
;
;   Internal/External Oscillator Switchover bit:
;     IESO = OFF           Oscillator Switchover mode disabled
;     IESO = ON            Oscillator Switchover mode enabled
;
;   Power-up Timer Enable bit:
;     PWRTEN = ON          PWRT enabled
;     PWRTEN = OFF         PWRT disabled
;
;   Brown-out Reset Enable bits:
;     BOREN = OFF          Brown-out Reset disabled in hardware and software
;     BOREN = ON           Brown-out Reset enabled and controlled by software (SBOREN is enabled)
;     BOREN = NOSLP        Brown-out Reset enabled in hardware only and disabled in Sleep mode (SBOREN is disabled)
;     BOREN = SBORDIS      Brown-out Reset enabled in hardware only (SBOREN is disabled)
;
;   Brown-out Reset Voltage bits:
;     BORV = 30            VBOR set to 3.0 V nominal
;     BORV = 27            VBOR set to 2.7 V nominal
;     BORV = 22            VBOR set to 2.2 V nominal
;     BORV = 19            VBOR set to 1.9 V nominal
;
;   Watchdog Timer Enable bit:
;     WDTEN = OFF          WDT is controlled by SWDTEN bit of the WDTCON register
;     WDTEN = ON           WDT is always enabled. SWDTEN bit has no effect.
;
;   Watchdog Timer Postscale Select bits:
;     WDTPS = 1            1:1
;     WDTPS = 2            1:2
;     WDTPS = 4            1:4
;     WDTPS = 8            1:8
;     WDTPS = 16           1:16
;     WDTPS = 32           1:32
;     WDTPS = 64           1:64
;     WDTPS = 128          1:128
;     WDTPS = 256          1:256
;     WDTPS = 512          1:512
;     WDTPS = 1024         1:1024
;     WDTPS = 2048         1:2048
;     WDTPS = 4096         1:4096
;     WDTPS = 8192         1:8192
;     WDTPS = 16384        1:16384
;     WDTPS = 32768        1:32768
;
;   HFINTOSC Fast Start-up bit:
;     HFOFST = OFF         The system clock is held off until the HFINTOSC is stable.
;     HFOFST = ON          HFINTOSC starts clocking the CPU without waiting for the oscillator to stablize.
;
;   MCLR Pin Enable bit:
;     MCLRE = OFF          RA3 input pin enabled; MCLR disabled
;     MCLRE = ON           MCLR pin enabled; RA3 input pin disabled
;
;   Stack Full/Underflow Reset Enable bit:
;     STVREN = OFF         Stack full/underflow will not cause Reset
;     STVREN = ON          Stack full/underflow will cause Reset
;
;   Single-Supply ICSP Enable bit:
;     LVP = OFF            Single-Supply ICSP disabled
;     LVP = ON             Single-Supply ICSP enabled
;
;   Boot Block Size Select bit:
;     BBSIZ = OFF          1kW boot block size
;     BBSIZ = ON           2kW boot block size
;
;   Extended Instruction Set Enable bit:
;     XINST = OFF          Instruction set extension and Indexed Addressing mode disabled (Legacy mode)
;     XINST = ON           Instruction set extension and Indexed Addressing mode enabled
;
;   Background Debugger Enable bit:
;     DEBUG = ON           Background debugger enabled, RA0 and RA1 are dedicated to In-Circuit Debug
;     DEBUG = OFF          Background debugger disabled, RA0 and RA1 configured as general purpose I/O pins
;
;   Code Protection bit:
;     CP0 = ON             Block 0 code-protected
;     CP0 = OFF            Block 0 not code-protected
;
;   Code Protection bit:
;     CP1 = ON             Block 1 code-protected
;     CP1 = OFF            Block 1 not code-protected
;
;   Boot Block Code Protection bit:
;     CPB = ON             Boot block code-protected
;     CPB = OFF            Boot block not code-protected
;
;   Data EEPROM Code Protection bit:
;     CPD = ON             Data EEPROM code-protected
;     CPD = OFF            Data EEPROM not code-protected
;
;   Table Write Protection bit:
;     WRT0 = ON            Block 0 write-protected
;     WRT0 = OFF           Block 0 not write-protected
;
;   Table Write Protection bit:
;     WRT1 = ON            Block 1 write-protected
;     WRT1 = OFF           Block 1 not write-protected
;
;   Configuration Register Write Protection bit:
;     WRTC = ON            Configuration registers write-protected
;     WRTC = OFF           Configuration registers not write-protected
;
;   Boot Block Write Protection bit:
;     WRTB = ON            Boot block write-protected
;     WRTB = OFF           Boot block not write-protected
;
;   Data EEPROM Write Protection bit:
;     WRTD = ON            Data EEPROM write-protected
;     WRTD = OFF           Data EEPROM not write-protected
;
;   Table Read Protection bit:
;     EBTR0 = ON           Block 0 protected from table reads executed in other blocks
;     EBTR0 = OFF          Block 0 not protected from table reads executed in other blocks
;
;   Table Read Protection bit:
;     EBTR1 = ON           Block 1 protected from table reads executed in other blocks
;     EBTR1 = OFF          Block 1 not protected from table reads executed in other blocks
;
;   Boot Block Table Read Protection bit:
;     EBTRB = ON           Boot block protected from table reads executed in other blocks
;     EBTRB = OFF          Boot block not protected from table reads executed in other blocks
;
;==========================================================================
;==========================================================================
;
;       Configuration Bits
;
;   NAME            Address
;   CONFIG1L        300000h
;   CONFIG1H        300001h
;   CONFIG2L        300002h
;   CONFIG2H        300003h
;   CONFIG3H        300005h
;   CONFIG4L        300006h
;   CONFIG5L        300008h
;   CONFIG5H        300009h
;   CONFIG6L        30000Ah
;   CONFIG6H        30000Bh
;   CONFIG7L        30000Ch
;   CONFIG7H        30000Dh
;
;==========================================================================

; The following is an assignment of address values for all of the
; configuration registers for the purpose of table reads
_CONFIG1L        EQU  H'300000'
_CONFIG1H        EQU  H'300001'
_CONFIG2L        EQU  H'300002'
_CONFIG2H        EQU  H'300003'
_CONFIG3H        EQU  H'300005'
_CONFIG4L        EQU  H'300006'
_CONFIG5L        EQU  H'300008'
_CONFIG5H        EQU  H'300009'
_CONFIG6L        EQU  H'30000A'
_CONFIG6H        EQU  H'30000B'
_CONFIG7L        EQU  H'30000C'
_CONFIG7H        EQU  H'30000D'

;----- CONFIG1L Options --------------------------------------------------
_CPUDIV_NOCLKDIV_1L  EQU  H'E7'    ; No CPU System Clock divide
_CPUDIV_CLKDIV2_1L   EQU  H'EF'    ; CPU System Clock divided by 2
_CPUDIV_CLKDIV3_1L   EQU  H'F7'    ; CPU System Clock divided by 3
_CPUDIV_CLKDIV4_1L   EQU  H'FF'    ; CPU System Clock divided by 4

_USBDIV_OFF_1L       EQU  H'DF'    ; USB clock comes directly from the OSC1/OSC2 oscillator block; no divide
_USBDIV_ON_1L        EQU  H'FF'    ; USB clock comes from the OSC1/OSC2 divided by 2

;----- CONFIG1H Options --------------------------------------------------
_FOSC_LP_1H          EQU  H'F0'    ; LP oscillator
_FOSC_XT_1H          EQU  H'F1'    ; XT oscillator
_FOSC_HS_1H          EQU  H'F2'    ; HS oscillator
_FOSC_ERCCLKOUT_1H   EQU  H'F3'    ; External RC oscillator, CLKOUT function on OSC2
_FOSC_ECCLKOUTH_1H   EQU  H'F4'    ; EC, CLKOUT function on OSC2 (high)
_FOSC_ECH_1H         EQU  H'F5'    ; EC (high)
_FOSC_ERC_1H         EQU  H'F7'    ; External RC oscillator
_FOSC_IRC_1H         EQU  H'F8'    ; Internal RC oscillator
_FOSC_IRCCLKOUT_1H   EQU  H'F9'    ; Internal RC oscillator, CLKOUT function on OSC2
_FOSC_ECCLKOUTM_1H   EQU  H'FA'    ; EC, CLKOUT function on OSC2 (medium)
_FOSC_ECM_1H         EQU  H'FB'    ; EC (medium)
_FOSC_ECCLKOUTL_1H   EQU  H'FC'    ; EC, CLKOUT function on OSC2 (low)
_FOSC_ECL_1H         EQU  H'FD'    ; EC (low)

_PLLEN_OFF_1H        EQU  H'EF'    ; PLL is under software control
_PLLEN_ON_1H         EQU  H'FF'    ; Oscillator multiplied by 4

_PCLKEN_OFF_1H       EQU  H'DF'    ; Primary clock is under software control
_PCLKEN_ON_1H        EQU  H'FF'    ; Primary clock enabled

_FCMEN_OFF_1H        EQU  H'BF'    ; Fail-Safe Clock Monitor disabled
_FCMEN_ON_1H         EQU  H'FF'    ; Fail-Safe Clock Monitor enabled

_IESO_OFF_1H         EQU  H'7F'    ; Oscillator Switchover mode disabled
_IESO_ON_1H          EQU  H'FF'    ; Oscillator Switchover mode enabled

;----- CONFIG2L Options --------------------------------------------------
_PWRTEN_ON_2L        EQU  H'FE'    ; PWRT enabled
_PWRTEN_OFF_2L       EQU  H'FF'    ; PWRT disabled

_BOREN_OFF_2L        EQU  H'F9'    ; Brown-out Reset disabled in hardware and software
_BOREN_ON_2L         EQU  H'FB'    ; Brown-out Reset enabled and controlled by software (SBOREN is enabled)
_BOREN_NOSLP_2L      EQU  H'FD'    ; Brown-out Reset enabled in hardware only and disabled in Sleep mode (SBOREN is disabled)
_BOREN_SBORDIS_2L    EQU  H'FF'    ; Brown-out Reset enabled in hardware only (SBOREN is disabled)

_BORV_30_2L          EQU  H'E7'    ; VBOR set to 3.0 V nominal
_BORV_27_2L          EQU  H'EF'    ; VBOR set to 2.7 V nominal
_BORV_22_2L          EQU  H'F7'    ; VBOR set to 2.2 V nominal
_BORV_19_2L          EQU  H'FF'    ; VBOR set to 1.9 V nominal

;----- CONFIG2H Options --------------------------------------------------
_WDTEN_OFF_2H        EQU  H'FE'    ; WDT is controlled by SWDTEN bit of the WDTCON register
_WDTEN_ON_2H         EQU  H'FF'    ; WDT is always enabled. SWDTEN bit has no effect.

_WDTPS_1_2H          EQU  H'E1'    ; 1:1
_WDTPS_2_2H          EQU  H'E3'    ; 1:2
_WDTPS_4_2H          EQU  H'E5'    ; 1:4
_WDTPS_8_2H          EQU  H'E7'    ; 1:8
_WDTPS_16_2H         EQU  H'E9'    ; 1:16
_WDTPS_32_2H         EQU  H'EB'    ; 1:32
_WDTPS_64_2H         EQU  H'ED'    ; 1:64
_WDTPS_128_2H        EQU  H'EF'    ; 1:128
_WDTPS_256_2H        EQU  H'F1'    ; 1:256
_WDTPS_512_2H        EQU  H'F3'    ; 1:512
_WDTPS_1024_2H       EQU  H'F5'    ; 1:1024
_WDTPS_2048_2H       EQU  H'F7'    ; 1:2048
_WDTPS_4096_2H       EQU  H'F9'    ; 1:4096
_WDTPS_8192_2H       EQU  H'FB'    ; 1:8192
_WDTPS_16384_2H      EQU  H'FD'    ; 1:16384
_WDTPS_32768_2H      EQU  H'FF'    ; 1:32768

;----- CONFIG3H Options --------------------------------------------------
_HFOFST_OFF_3H       EQU  H'F7'    ; The system clock is held off until the HFINTOSC is stable.
_HFOFST_ON_3H        EQU  H'FF'    ; HFINTOSC starts clocking the CPU without waiting for the oscillator to stablize.

_MCLRE_OFF_3H        EQU  H'7F'    ; RA3 input pin enabled; MCLR disabled
_MCLRE_ON_3H         EQU  H'FF'    ; MCLR pin enabled; RA3 input pin disabled

;----- CONFIG4L Options --------------------------------------------------
_STVREN_OFF_4L       EQU  H'FE'    ; Stack full/underflow will not cause Reset
_STVREN_ON_4L        EQU  H'FF'    ; Stack full/underflow will cause Reset

_LVP_OFF_4L          EQU  H'FB'    ; Single-Supply ICSP disabled
_LVP_ON_4L           EQU  H'FF'    ; Single-Supply ICSP enabled

_BBSIZ_OFF_4L        EQU  H'F7'    ; 1kW boot block size
_BBSIZ_ON_4L         EQU  H'FF'    ; 2kW boot block size

_XINST_OFF_4L        EQU  H'BF'    ; Instruction set extension and Indexed Addressing mode disabled (Legacy mode)
_XINST_ON_4L         EQU  H'FF'    ; Instruction set extension and Indexed Addressing mode enabled

_DEBUG_ON_4L         EQU  H'7F'    ; Background debugger enabled, RA0 and RA1 are dedicated to In-Circuit Debug
_DEBUG_OFF_4L        EQU  H'FF'    ; Background debugger disabled, RA0 and RA1 configured as general purpose I/O pins

;----- CONFIG5L Options --------------------------------------------------
_CP0_ON_5L           EQU  H'FE'    ; Block 0 code-protected
_CP0_OFF_5L          EQU  H'FF'    ; Block 0 not code-protected

_CP1_ON_5L           EQU  H'FD'    ; Block 1 code-protected
_CP1_OFF_5L          EQU  H'FF'    ; Block 1 not code-protected

;----- CONFIG5H Options --------------------------------------------------
_CPB_ON_5H           EQU  H'BF'    ; Boot block code-protected
_CPB_OFF_5H          EQU  H'FF'    ; Boot block not code-protected

_CPD_ON_5H           EQU  H'7F'    ; Data EEPROM code-protected
_CPD_OFF_5H          EQU  H'FF'    ; Data EEPROM not code-protected

;----- CONFIG6L Options --------------------------------------------------
_WRT0_ON_6L          EQU  H'FE'    ; Block 0 write-protected
_WRT0_OFF_6L         EQU  H'FF'    ; Block 0 not write-protected

_WRT1_ON_6L          EQU  H'FD'    ; Block 1 write-protected
_WRT1_OFF_6L         EQU  H'FF'    ; Block 1 not write-protected

;----- CONFIG6H Options --------------------------------------------------
_WRTC_ON_6H          EQU  H'DF'    ; Configuration registers write-protected
_WRTC_OFF_6H         EQU  H'FF'    ; Configuration registers not write-protected

_WRTB_ON_6H          EQU  H'BF'    ; Boot block write-protected
_WRTB_OFF_6H         EQU  H'FF'    ; Boot block not write-protected

_WRTD_ON_6H          EQU  H'7F'    ; Data EEPROM write-protected
_WRTD_OFF_6H         EQU  H'FF'    ; Data EEPROM not write-protected

;----- CONFIG7L Options --------------------------------------------------
_EBTR0_ON_7L         EQU  H'FE'    ; Block 0 protected from table reads executed in other blocks
_EBTR0_OFF_7L        EQU  H'FF'    ; Block 0 not protected from table reads executed in other blocks

_EBTR1_ON_7L         EQU  H'FD'    ; Block 1 protected from table reads executed in other blocks
_EBTR1_OFF_7L        EQU  H'FF'    ; Block 1 not protected from table reads executed in other blocks

;----- CONFIG7H Options --------------------------------------------------
_EBTRB_ON_7H         EQU  H'BF'    ; Boot block protected from table reads executed in other blocks
_EBTRB_OFF_7H        EQU  H'FF'    ; Boot block not protected from table reads executed in other blocks


;----- DEVID Equates --------------------------------------------------
_DEVID1          EQU  H'3FFFFE'
_DEVID2          EQU  H'3FFFFF'

;----- IDLOC Equates --------------------------------------------------
_IDLOC0          EQU  H'200000'
_IDLOC1          EQU  H'200001'
_IDLOC2          EQU  H'200002'
_IDLOC3          EQU  H'200003'
_IDLOC4          EQU  H'200004'
_IDLOC5          EQU  H'200005'
_IDLOC6          EQU  H'200006'
_IDLOC7          EQU  H'200007'

        LIST
