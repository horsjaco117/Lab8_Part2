; Assembly Code for PIC16F883
; This version maintains the ADC operation and Timer2 triggering as before.
; It implements variable duty cycle PWM using CCP1 module (output on RC2/CCP1 pin).
; The 10-bit ADC result directly sets the PWM duty cycle
; Since the ADC is left-justified (ADFM=0), this mapping is direct.
; The PWM period is determined by Timer2 (PR2=0xF0, prescaler 1:16), matching the ADC sampling rate.
; An LED connected to RC2 would now have brightness proportional to the analog input
; Initial duty cycle set to 0.

; LAB 8
; Jacob Horsley
; RCET
; Fifth Semester
; ADC and Servo Motors
; Device: PIC16F883
; GITHUB: https://github.com/horsjaco117/Lab8_Part2
;-------------------------------------------------------------------------------
; Configuration
    ; CONFIG1
    CONFIG FOSC = XT ; Oscillator Selection bits (XT oscillator)
    CONFIG WDTE = OFF ; Watchdog Timer Enable bit (WDT disabled)
    CONFIG PWRTE = OFF ; Power-up Timer Enable bit (PWRT disabled)
    CONFIG MCLRE = ON ; RE3/MCLR pin function select bit (MCLR)
    CONFIG CP = OFF ; Code Protection bit (Program memory code protection disabled)
    CONFIG CPD = OFF ; Data Code Protection bit (Data memory code protection disabled)
    CONFIG BOREN = OFF ; Brown Out Reset Selection bits (BOR disabled)
    CONFIG IESO = OFF ; Internal External Switchover bit (disabled)
    CONFIG FCMEN = OFF ; Fail-Safe Clock Monitor Enabled bit (disabled)
    CONFIG LVP = OFF ; Low Voltage Programming Enable bit (RB3 is digital I/O)
    ; CONFIG2
    CONFIG BOR4V = BOR40V ; Brown-out Reset Selection bit (set to 4.0V)
    CONFIG WRT = OFF ; Flash Program Memory Self Write Enable bits (Write protection off)
   
; Include Statements
#include <xc.inc>
#define W 0
#define F 1
   
; Code Section
;------------------------------------------------------------------------------
; Register/Variable Setup
    RESULT_HI EQU 0X21	      ;For ADC High Byte
    RESULT_LO EQU 0X22	      ;For ADC Low Byte
    W_TEMP EQU 0X23	      ;These temp values are probably from a different project...
    STATUS_TEMP EQU 0X24
    TEMP EQU 0X25	    ;DO NOT ERASE THIS IS AN IMPORTANT VARIABLe

; Start of Program
; Reset vector address
    PSECT resetVect, class=CODE, delta=2
    GOTO Start
; Interrupt vector
    PSECT isrVect, class=CODE, delta=2 ;Jumps to the interrupt that handles TMR2
    GOTO INTERRUPT
; Setup Code
    PSECT code, class=CODE, delta=2
Start:
Setup:
    ; Bank 3 (ANSELH, ANSEL)
    BSF STATUS, 5 ; Go to Bank 3
    BSF STATUS, 6
    CLRF ANSELH ; Set pins to digital I/O
    MOVLW 0X02 ;Pin 3 selected (AN1 on RA1)
    MOVWF ANSEL ;Sets Pins to analog
   
    ; Bank 2 (CM2CON0, CM2CON1)
    BSF STATUS, 5
    BCF STATUS, 6 ; Go to Bank 2
    CLRF CM2CON1 ; Disable Comparator 2
    CLRF CM2CON0 ; Disable Comparator 1
   
    ; Bank 1 (TRISB, WPUB, IOCB, OPTION_REG, TRISC, TRISA, PIE1, PR2)
    BSF STATUS, 5
    BCF STATUS, 6 ; Go to Bank 1
    MOVLW 0x00 ; PortB as Outputs
    MOVWF TRISB
    MOVLW 0x00 ; Disable weak pull-ups on Port B
    MOVWF WPUB
    MOVLW 0x00 ; Disable interrupt-on-change for RB0/RB1
    MOVWF IOCB
    CLRF OPTION_REG ; Enable global pull-ups, clear Timer0 settings
    MOVLW 0X00
    MOVWF TRISC ; Port C as outputs (for traffic lights and PWM on RC2)
    MOVLW 0x03 ; RA0, RA1 as inputs
    MOVWF TRISA ; Set RA0, RA1 as inputs for buttons/analog
    MOVLW 0x42 ; ADIE and TMR2IE enabled
    MOVWF PIE1
    MOVLW 0xFF ; Timer2 period set (PWM period = (PR2+1)*4*Tosc*prescaler)
    MOVWF PR2
    MOVLW 0XC5 ; VREF and clock select
    MOVWF ADCON1
    BSF ADCON1, 7 ; Temporarily set ADFM=1 (right justified)
    
    ; Bank 0 (INTCON, ports, peripherals)
    BCF STATUS, 5 ; Go to Bank 0
    BCF STATUS, 6

    MOVLW 0XC5 ; AN1 selected, ADON=1, ADCS=01 (Fosc/8)
    MOVWF ADCON0
    MOVLW 0x00 ; Interrupts disabled initially
    MOVWF INTCON
    MOVLW 0X00  ; Clear peripheral interrupt flags
    MOVWF PIR1 
    MOVLW 0XC0 ; Enable GIE and PEIE
    MOVWF INTCON
    CLRF CCP1CON ; Disable PWM initially
    CLRF CCPR1L  ; Set initial duty cycle to 0
    MOVLW 0X0C   ; Enable PWM mode (CCP1CON = 0b00001100)
    MOVWF CCP1CON
    MOVLW 0X00
    MOVWF PORTB ; Clear Port B
    CLRF CCP2CON ; Disable second PWM
    CLRF PORTA ; Clear Port A
    CLRF PORTC ; Clear Port C
    CLRF RCSTA ; Disable serial control
    CLRF SSPCON ; Disable serial port
    CLRF T1CON ; Disable Timer1
    CLRF PSTRCON ; Disable PWM pulse steering
    CLRF CM2CON1 ; Disable Comparator 2
    MOVLW 0x00
    MOVWF TMR2 ; Reset Timer2
    MOVLW 0x06 ; Prescaler 1:4, TMR2ON=1
    MOVWF T2CON
    BSF ADCON0, 1 ; Start initial ADC conversion

    BANKSEL ADCON1 ; Set to left justified (ADFM=0)
    BCF ADCON1, 7
   
    BCF STATUS, 5 ;Ensuring the code is in bank 1
    BCF STATUS, 6

; Main Program Loop
MAINLOOP:
    GOTO MAINLOOP
    
INTERRUPT:
    MOVWF W_TEMP	;Saves all the data from mainloop
    SWAPF STATUS, W
    MOVWF STATUS_TEMP
    
    BTFSS   PIR1, 6     ; Check ADIF
    GOTO CHECK_OTHER
    BCF PIR1, 6         ; Clear ADIF
    BANKSEL ADRESH      ; Ensure Bank 0 for ADRESH
    MOVF ADRESH, W	;High register from ADC is saved
    MOVWF RESULT_HI
    BANKSEL ADRESL      ; Bank 1 for ADRESL
    MOVF ADRESL, W	;Low register from ADC is saved too
    BANKSEL RESULT_LO   ; Back to Bank 0
    MOVWF RESULT_LO
    
   ; Set PWM duty cycle from ADC result (Referenced from PWM example)
    MOVF RESULT_HI, W   ; ADRESH = duty bits 9:2
    MOVWF CCPR1L	;Sets the PWM for the defined period of the Timer
    MOVF CCP1CON, W     ; Preserve mode bits
    ANDLW 0x0F          ; Clear bits 7:4, but keep 3:0 for mode
    MOVWF W_TEMP        ; Temp store
    MOVF RESULT_LO, W   ; ADRESL bits 7:6 = duty bits 1:0
    ANDLW 0xC0          ; Isolate bits 7:6
    MOVWF TEMP          ; Move to temporary file register
    RRF TEMP, 1         ; Shift right: now bits 6:5 (use 1 for destination to file)
    RRF TEMP, 1         ; Shift right again: bits 5:4
    MOVF TEMP, W        ; Load shifted value back to W
    IORWF W_TEMP, W     ; Combine with cleared CCP1CON
    MOVWF CCP1CON       ; Update CCP1CON with new duty LSBs
    
    BTFSC RESULT_HI, 7  ;Bit test for basic light test     
    GOTO LIGHT_SWAP2   
    GOTO LIGHT_SWAP          

LIGHT_SWAP:	;Troubleshooting lights
    BCF PORTC, 7
    BSF PORTC, 4
    
    GOTO EXIT_ISR

LIGHT_SWAP2:	;Troubleshooting lights
    BSF PORTC, 7
    BCF PORTC, 4
    
    GOTO EXIT_ISR
    
CHECK_OTHER:
    BTFSS PIR1, 1       ; Check TMR2IF
    GOTO EXIT_ISR
    BCF PIR1,1          ; Clear TMR2IF
    
START_ADC:
    BSF ADCON0, 1       ; Start next ADC conversion
  
EXIT_ISR:
    SWAPF STATUS_TEMP, W;Loads all the data from the mainloop
    MOVWF STATUS
    SWAPF W_TEMP, F
    SWAPF W_TEMP, W
    RETFIE
END