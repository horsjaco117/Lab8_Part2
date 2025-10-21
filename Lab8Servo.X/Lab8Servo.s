; LAB 8
; Jacob Horsley
; RCET
; Fifth Semester
; ADC and lights (modified to servo control)
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
    W_TEMP EQU 0X23
    STATUS_TEMP EQU 0X24
    ADC_SAVE EQU 0X22
    TIME EQU 0X33
     
 
; Start of Program
; Reset vector address
    PSECT resetVect, class=CODE, delta=2
    GOTO Start
; Interrupt vector
    PSECT isrVect, class=CODE, delta=2
    GOTO INTERRUPT
; Setup Code
    PSECT code, class=CODE, delta=2
Start:
Setup:
    ; Bank 3 (ANSELH, ANSEL)
    BSF STATUS, 5 ; Go to Bank 3
    BSF STATUS, 6
    CLRF ANSELH ; Set pins to digital I/O
    MOVLW 0X01 ; Pin AN0 selected
    MOVWF ANSEL ; Sets Pins to analog
   
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
    MOVWF TRISC ; Port C as outputs
    MOVLW 0x01 ; RA0 as an input
    MOVWF TRISA ; Set RA0 as input for ADC
    MOVLW 0x02 ; TMR2IE enabled
    MOVWF PIE1
    MOVLW 0xFF ; Timer2 period set for ~20ms (with pre/postscaler)
    MOVWF PR2
    CLRF ADCON1 ; Left justify
   
    ; Bank 0 (INTCON, ports, peripherals)
    BCF STATUS, 5 ; Go to Bank 0
    BCF STATUS, 6

    MOVLW 0X41 ; AN0 pin selected, ADON=1, ADCS=01 (Fosc/8)
    MOVWF ADCON0
    MOVLW 0x00 ; Interrupts disabled initially
    MOVWF INTCON
    MOVLW 0X00  ; Clear peripheral interrupt flags
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
    MOVLW 0x26 ; Prescaler 1:16, postscaler 1:5, TMR2ON=1
    MOVWF T2CON
    BSF ADCON0, 1 ; Start initial ADC conversion
   
; Main Program Loop
MAINLOOP:
    MOVF ADRESH, W ; Display ADC high byte on PORTC
    MOVWF PORTC
    GOTO MAINLOOP
    
INTERRUPT:
    
    MOVWF W_TEMP	; Save W
    SWAPF STATUS, W
    MOVWF STATUS_TEMP	; Save STATUS
    
    BCF PIR1, 1         ; Clear TMR2IF
    
    CLRF ADC_SAVE       ; Clear position accumulator
    
    BTFSC ADRESH, 3     ; Check bit 3
    CALL BIT_3
    BTFSC ADRESH, 4     ; Check bit 4
    CALL BIT_4
    BTFSC ADRESH, 5     ; Check bit 5
    CALL BIT_5
    BTFSC ADRESH, 6     ; Check bit 6
    CALL BIT_6
    BTFSC ADRESH, 7     ; Check bit 7
    CALL BIT_7
    
    MOVF ADC_SAVE, W    ; Load position
    ADDWF PCL, F        ; Jump table
    
    GOTO P1
    GOTO P2
    GOTO P2
    GOTO P3
    GOTO P4
    GOTO P4
    GOTO P5
    GOTO P6
    GOTO P6
    GOTO P7
    GOTO P8
    GOTO P8
    GOTO P9
    GOTO P10
    GOTO P10
    GOTO P11
    GOTO P12
    GOTO P12
    GOTO P13
    GOTO P14
    GOTO P14
    GOTO P15
    GOTO P16
    GOTO P16
    GOTO P17
    GOTO P18
    GOTO P18
    GOTO P19
    GOTO P20
    GOTO P20
    GOTO P21
    GOTO P21
    
P1:
    MOVLW 0x32
    MOVWF TIME
    BSF PORTB, 0        ; Modified: Pulse high on PORTB bit 0 (RB0)
    GOTO DELAY
    
P2:
    MOVLW 0x3C
    MOVWF TIME
    BSF PORTB, 0
    GOTO DELAY
    
P3:
    MOVLW 0x46
    MOVWF TIME
    BSF PORTB, 0
    GOTO DELAY
    
P4:
    MOVLW 0x50
    MOVWF TIME
    BSF PORTB, 0
    GOTO DELAY
    
P5:
    MOVLW 0x5A
    MOVWF TIME
    BSF PORTB, 0
    GOTO DELAY
    
P6:
    MOVLW 0x64
    MOVWF TIME
    BSF PORTB, 0
    GOTO DELAY
    
P7:
    MOVLW 0x6E
    MOVWF TIME
    BSF PORTB, 0
    GOTO DELAY
    
P8:
    MOVLW 0x78
    MOVWF TIME
    BSF PORTB, 0
    GOTO DELAY
    
P9:
    MOVLW 0x82
    MOVWF TIME
    BSF PORTB, 0
    GOTO DELAY
    
P10:
    MOVLW 0x8C
    MOVWF TIME
    BSF PORTB, 0
    GOTO DELAY
    
P11:
    MOVLW 0x96
    MOVWF TIME
    BSF PORTB, 0
    GOTO DELAY
    
P12:
    MOVLW 0xA0
    MOVWF TIME
    BSF PORTB, 0
    GOTO DELAY
    
P13:
    MOVLW 0xAA
    MOVWF TIME
    BSF PORTB, 0
    GOTO DELAY
    
P14:
    MOVLW 0xB4
    MOVWF TIME
    BSF PORTB, 0
    GOTO DELAY
    
P15:
    MOVLW 0xBE
    MOVWF TIME
    BSF PORTB, 0
    GOTO DELAY
    
P16:
    MOVLW 0xC8
    MOVWF TIME
    BSF PORTB, 0
    GOTO DELAY
    
P17:
    MOVLW 0xD2
    MOVWF TIME
    BSF PORTB, 0
    GOTO DELAY
    
P18:
    MOVLW 0xDC
    MOVWF TIME
    BSF PORTB, 0
    GOTO DELAY
    
P19:
    MOVLW 0xE6
    MOVWF TIME
    BSF PORTB, 0
    GOTO DELAY
    
P20:
    MOVLW 0xF2
    MOVWF TIME
    BSF PORTB, 0
    GOTO DELAY
    
P21:
    MOVLW 0xFF
    MOVWF TIME
    BSF PORTB, 0
    GOTO DELAY
    
DELAY:
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    DECFSZ TIME, F
    GOTO DELAY
    
    BCF PORTB, 0        ; Modified: Pulse low on PORTB bit 0 (RB0)
    BSF ADCON0, 1       ; Start next ADC conversion
    
EXIT_ISR:
    SWAPF STATUS_TEMP, W
    MOVWF STATUS
    SWAPF W_TEMP, F
    SWAPF W_TEMP, W
    RETFIE
    
BIT_3:
    MOVLW 0x01
    ADDWF ADC_SAVE, F
    RETURN
    
BIT_4:
    MOVLW 0x02
    ADDWF ADC_SAVE, F
    RETURN
    
BIT_5:
    MOVLW 0x04
    ADDWF ADC_SAVE, F
    RETURN
    
BIT_6:
    MOVLW 0x08
    ADDWF ADC_SAVE, F
    RETURN
    
BIT_7:
    MOVLW 0x10
    ADDWF ADC_SAVE, F
    RETURN
    
END