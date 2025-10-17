; Modified Assembly Code for PIC16F883
; This modified version maintains the ADC operation as in the original code, periodically triggering ADC conversions via Timer2 overflows.
; Additionally, Timer2 is now used to toggle an LED connected to PORTB, bit 0 (RB0), at a rate determined by the ADC result.
; Specifically, the blink period is modulated by the value in MODTIME (set to 0x09 or 0x05 based on the MSB of the ADC high byte).
; A blink counter (BLINK_COUNT) is decremented each Timer2 interrupt; when it reaches zero, the LED is toggled, and the counter is reloaded from MODTIME.
; The original assignment of RESULT_HI to PORTB (for testing) has been removed to avoid overwriting the LED state.
; Other unused variables (e.g., COUNT1 to COUNT7) remain as placeholders but are not utilized in this implementation.
; Corrections have been applied to apparent typos in the original code (e.g., redundant or erroneous MOVWF instructions in delay settings).

; LAB 8
; Jacob Horsley
; RCET
; Fifth Semester
; ADC and lights
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
   
; Code Section
;------------------------------------------------------------------------------
; Register/Variable Setup
    RESULT_HI EQU 0X21	      ;For ADC High Byte
    RESULT_LO EQU 0X22
    W_TEMP EQU 0X23
    STATUS_TEMP EQU 0X24
    COUNT1 EQU 0x25
    COUNT2 EQU 0x26
    COUNT3 EQU 0x27
    COUNT4 EQU 0X28
    COUNT5 EQU 0X29
    COUNT6 EQU 0X2A
    COUNT7 EQU 0X2B
    COUNT1A EQU 0X2C
    COUNT2A EQU 0X2D
    COUNT3A EQU 0X2E
    COUNT4A EQU 0X2F
    COUNT5A EQU 0X30
    COUNT6A EQU 0X31
    COUNT7A EQU 0X32
    MODTIME EQU 0X33
    MODTIME0 EQU 0X34
    BLINK_COUNT EQU 0x35     ; New variable for blink timing counter
   
 
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
    MOVLW 0X02 ;Pin 3 selected
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
    MOVWF TRISC ; Port C as outputs (for traffic lights)
    MOVLW 0x03 ; RA0 as an input
    MOVWF TRISA ; Set RA0, RA1 as inputs for buttons
    MOVLW 0x42 ; ADIE enabled, timer 2 enabled too
    MOVWF PIE1
    MOVLW 0xF0 ; Timer2 period set
    MOVWF PR2
    MOVLW 0XC5 ; Voltage reference and clock select
    MOVWF ADCON1
    BSF ADCON1, 7 ; Temporarily set ADFM=1 (right justified)
    
    ; Bank 0 (INTCON, ports, peripherals)
    BCF STATUS, 5 ; Go to Bank 0
    BCF STATUS, 6

    MOVLW 0XC5 ;AN1 pin selected, ADON=1, ADCS=01
    MOVWF ADCON0
    MOVLW 0x00 ; Interrupts disabled initially
    MOVWF INTCON
    MOVLW 0X00  ;Clear peripheral interrupt flags
    MOVWF PIR1 
    MOVLW 0XC0 ; Enable GIE and PEIE
    MOVWF INTCON
    CLRF CCP1CON ; Disable PWM
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
    MOVLW 0x05 ; Prescaler 1:4, TMR2ON=1
    MOVWF T2CON
    BSF ADCON0, 1 ; Start initial ADC conversion

    BANKSEL ADCON1 ; Set to left justified
    BCF ADCON1, 7
   
    BCF STATUS, 5
    BCF STATUS, 6

    ; Initialize blink counter (default to 0x05 for startup)
    MOVLW 0x05
    MOVWF BLINK_COUNT

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
    MOVF ADRESH, W
    MOVWF RESULT_HI
    BANKSEL ADRESL      ; Bank 1 for ADRESL
    MOVF ADRESL, W
    BANKSEL RESULT_LO   ; Back to Bank 0
    MOVWF RESULT_LO
    
    ; Removed: MOVF RESULT_HI, W ; MOVWF PORTB (to avoid overwriting LED state)
    
    BTFSC RESULT_HI, 7        
    GOTO SET_LONG_DELAY   
    GOTO SET_OTHER_DELAY          

SET_OTHER_DELAY:
    MOVLW 0X09
    MOVWF MODTIME
    MOVLW 0X09   ; Corrected typo from original (was MOVWF 0X09)
    MOVWF MODTIME0
    
    BCF PORTC, 7
    BSF PORTC, 4
    
    GOTO EXIT_ISR

SET_LONG_DELAY:
    MOVLW 0X05
    MOVWF MODTIME
    MOVLW 0X05
    MOVWF MODTIME0
    
    BSF PORTC, 7
    BCF PORTC, 4
    
    GOTO EXIT_ISR
    
CHECK_OTHER:
    BTFSS PIR1, 1       ; Check TMR2IF
    GOTO EXIT_ISR
    BCF PIR1,1          ; Clear TMR2IF
    
    ; LED toggle logic using Timer2
    DECFSZ BLINK_COUNT, F
    GOTO START_ADC
    MOVLW 0x01          ; Mask for RB0
    XORWF PORTB, F      ; Toggle the LED on PORTB, bit 0
    MOVF MODTIME, W     ; Reload counter from MODTIME (set by ADC)
    MOVWF BLINK_COUNT
    
START_ADC:
    BSF ADCON0, 1       ; Start next ADC conversion
  
EXIT_ISR:
    
    SWAPF STATUS_TEMP, W;Loads all the data from the mainloop
    MOVWF STATUS
    SWAPF W_TEMP, F
    SWAPF W_TEMP, W
    RETFIE
    
    
END