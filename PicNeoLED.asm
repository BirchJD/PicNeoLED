                  LIST     P = P12F629

                  INCLUDE  "../PIC_INCLUDE/P12F629.INC"

                  __CONFIG _HS_OSC & _WDT_ON & _PWRTE_ON & _MCLRE_OFF & _BOREN_ON & _CP_ON & _CPD_ON
;/*********************************************************************/
;/* REQUIRES 20MHz EXTERNAL OSCILATOR FOR REQUIRED HIGH SPEED SIGNALS */
;/*********************************************************************/

;/***************/
;/* GPIO Lines. */
;/***************/
; GP0       - WS2812B LED Serial Data.
; GP3       - Switch.

;/*************/
;/* Constants */
;/*************/
; PORT A
GPIO_NEO_LED_DATA EQU      GP0                  ; WS2812B LED Serial Data.
GPIO_SWITCH       EQU      GP3                  ; Switch.

;/**********************/
;/* Application flags. */
;/**********************/
F_CYCLE           EQU      0x00                 ; Flag to cycle LEDs.

NEO_ARRAY_SIZE    EQU      0x08                 ; Number of NEO LEDs in series.


;/***************************************/
;/* RAM Registers in BANK 0 (64 Bytes). */
;/***************************************/
CBLOCK            0x20
                  INT_W                         ; Temporary store for W during interupt.
                  INT_STATUS                    ; Temporary store for STATUS during interupt.
                  INT_FSR                       ; Temporary store for FSR during interupt.

                  APP_FLAGS                     ; Application flags.

                  RAND                          ; Random number.
                  TEMP                          ; Temporary store.
                  TEMP_FSR                      ; Temporary FSR store.
                  NEO_LED_COUNT                 ; Count LEDs.
                  NEO_LED_RGB_COUNT             ; Count LED colour byte data.
                  NEO_LED_TEMP                  ; Temporary store.

                  NEO_LED_FADE_COUNT            ; Pointer to current fade value.
                  NEO_LED_FADE1                 ; Direction to fade colour elements of LED 1.
                  NEO_LED_FADE2                 ; Direction to fade colour elements of LED 2.
                  NEO_LED_FADE3                 ; Direction to fade colour elements of LED 3.
                  NEO_LED_FADE4                 ; Direction to fade colour elements of LED 4.
                  NEO_LED_FADE5                 ; Direction to fade colour elements of LED 5.
                  NEO_LED_FADE6                 ; Direction to fade colour elements of LED 6.
                  NEO_LED_FADE7                 ; Direction to fade colour elements of LED 7.
                  NEO_LED_FADE8                 ; Direction to fade colour elements of LED 8.

                  NEO_LED_G1_DATA               ; WS2812B LED 1 Green Data.
                  NEO_LED_R1_DATA               ; WS2812B LED 1 Red Data.
                  NEO_LED_B1_DATA               ; WS2812B LED 1 Blue Data.

                  NEO_LED_G2_DATA               ; WS2812B LED 2 Green Data.
                  NEO_LED_R2_DATA               ; WS2812B LED 2 Red Data.
                  NEO_LED_B2_DATA               ; WS2812B LED 2 Blue Data.

                  NEO_LED_G3_DATA               ; WS2812B LED 3 Green Data.
                  NEO_LED_R3_DATA               ; WS2812B LED 3 Red Data.
                  NEO_LED_B3_DATA               ; WS2812B LED 3 Blue Data.

                  NEO_LED_G4_DATA               ; WS2812B LED 4 Green Data.
                  NEO_LED_R4_DATA               ; WS2812B LED 4 Red Data.
                  NEO_LED_B4_DATA               ; WS2812B LED 4 Blue Data.

                  NEO_LED_G5_DATA               ; WS2812B LED 5 Green Data.
                  NEO_LED_R5_DATA               ; WS2812B LED 5 Red Data.
                  NEO_LED_B5_DATA               ; WS2812B LED 5 Blue Data.

                  NEO_LED_G6_DATA               ; WS2812B LED 6 Green Data.
                  NEO_LED_R6_DATA               ; WS2812B LED 6 Red Data.
                  NEO_LED_B6_DATA               ; WS2812B LED 6 Blue Data.

                  NEO_LED_G7_DATA               ; WS2812B LED 7 Green Data.
                  NEO_LED_R7_DATA               ; WS2812B LED 7 Red Data.
                  NEO_LED_B7_DATA               ; WS2812B LED 7 Blue Data.

                  NEO_LED_G8_DATA               ; WS2812B LED 8 Green Data.
                  NEO_LED_R8_DATA               ; WS2812B LED 8 Red Data.
                  NEO_LED_B8_DATA               ; WS2812B LED 8 Blue Data.
ENDC



                  CODE

;/**********************************/
;/* Reset program location vector. */
;/**********************************/
                  ORG      0x0000

                  BSF      STATUS, RP0          ; Select Register bank 1
                  MOVLW    (1 << GPIO_SWITCH)
                  MOVWF    TRISIO               ; Set pins to input.
                  GOTO     INIT



;/*************************************/
;/* Interupt program location vector. */
;/*************************************/
                  ORG      0x0004

INT_HANDLE        MOVWF    INT_W                ; Store registers from application during interupt.
                  MOVFW    STATUS
                  MOVWF    INT_STATUS
                  MOVFW    FSR
                  MOVWF    INT_FSR

                  BCF      STATUS, RP0          ; Select Register bank 0


                  BTFSS    INTCON, T0IF         ; Did a TIMER0 interupt trigger?
                  GOTO     INT_TIMER0_END       ; TIMER0: LED update timer.

                  SWAPF    TMR1L, W             ; Simulate a random number generator from the current timer value of the other timer.
                  XORWF    RAND
                  MOVFW    TMR1H
                  XORWF    RAND
                  BSF      APP_FLAGS, F_CYCLE   ; Periodically flag to update the colour values of the LEDs.

TIMER0_END        BCF      INTCON, T0IF         ; End of timer 0 interupt.


INT_TIMER0_END    BTFSS    PIR1, TMR1IF         ; Did a TIMER1 interupt trigger?
                  GOTO     INT_TIMER1_END       ; TIMER1: Colour randomize timer.

                  MOVFW    TMR0                 ; Simulate a random number generator from the current timer value of the other timer.
                  XORWF    RAND
                  MOVFW    NEO_LED_FADE_COUNT   ; Randomize the fade bit values for another LED on each timer interupt.
                  ADDLW    NEO_LED_FADE1
                  MOVWF    FSR
                  MOVFW    RAND
                  MOVWF    INDF
                  INCF     NEO_LED_FADE_COUNT   ; Do the next LED fade value on the next interupt.
                  MOVFW    NEO_LED_FADE_COUNT   ; If the last LED is done, point back to the first LED again.
                  XORLW    NEO_ARRAY_SIZE
                  BTFSC    STATUS, Z
                  CLRF     NEO_LED_FADE_COUNT

TIMER1_END        BCF      PIR1, TMR1IF         ; End of timer 1 interupt.


INT_TIMER1_END    MOVFW    INT_FSR              ; Restore registers for application to continue.
                  MOVWF    FSR
                  MOVFW    INT_STATUS
                  MOVWF    STATUS
                  MOVFW    INT_W
                  BSF      STATUS, Z
                  BTFSS    INT_STATUS, Z
                  BCF      STATUS, Z
                  RETFIE



;/*******************************/
;/* Initialise microcontroller. */
;/*******************************/
INIT              MOVLW    (1 << GPIO_SWITCH)
                  MOVWF    WPU                  ; Weak pull-ups on keys.
                  MOVWF    IOC                  ; Interupt on change of keys.

                  MOVLW    (1 << PSA)|(1 << PS0)|(1 << PS1)|(1 << PS2)
                  MOVWF    OPTION_REG           ; Prescale watch dog timer.
                  MOVLW    (1 << T1IE)          ; Start timer 1.
                  MOVWF    PIE1

                  MOVLW    0xFF                 ; Tune micro-controller frequency for fastest.
                  MOVWF    OSCCAL

                  BCF      STATUS, RP0          ; Select Register bank 0

                  MOVLW    0x20                 ; Clear all BANK 0 RAM values.
                  MOVWF    FSR
                  MOVLW    0x40
                  CALL     CLEAR_RAM

                  MOVLW    0x07                 ; Switch comparitor off.
                  MOVWF    CMCON

                  MOVLW    HIGH ORG_DATA_TABLES ; All lookup tables are in the top page of memory.
                  MOVWF    PCLATH

                  CLRF     GPIO                 ; Initialize GPIO Port values.

                  MOVLW    (1 << GIE)|(1 << PEIE)|(1 << GPIE)|(1 << T0IE)  ; Enable interupts.
                  MOVWF    INTCON               ; Configure timer interupts and enable interupts.
                  CLRF     PIR1                 ; Clear interupt flags.

                  MOVLW    (1 << TMR1ON)|(1 << NOT_T1SYNC)|(1 << T1CKPS0) ; |(1 << T1CKPS1)
                  MOVWF    T1CON                ; Configure timer 1.

LOOP              CLRWDT                        ; Tell CPU application still running when active.
                  BTFSS    APP_FLAGS, F_CYCLE   ; Update the NEO LEDs when flagged to.
                  GOTO     LOOP
                  BCF      APP_FLAGS, F_CYCLE   ; Clear NEO LED update flag.
                  CALL     NEO_LED_CYCLE        ; Update NEO LED colours.
                  CALL     NEO_LED_REFRESH      ; Refresh NEO LEDs.
                  GOTO     LOOP



;/******************************/
;/* Demo fading NEO LED array. */
;/******************************/
NEO_LED_CYCLE     MOVLW    NEO_LED_G1_DATA      ; Starting at the last LED data in the array of NEO LEDs.
                  MOVWF    FSR
                  MOVLW    NEO_ARRAY_SIZE       ; Itterate through all of the LED data in the NEO array.
                  MOVWF    NEO_LED_COUNT
NEO_LED_INIT_DATA MOVFW    FSR                  ; Temporarily store the indiect memory pointer.
                  MOVWF    TEMP_FSR
                  MOVFW    NEO_LED_COUNT        ; Get the fade data for the current LED.
                  ADDLW    (NEO_LED_FADE1 - 1)
                  MOVWF    FSR
                  MOVFW    INDF
                  MOVWF    TEMP
                  MOVFW    TEMP_FSR             ; Restore the indiect memory pointer.
                  MOVWF    FSR
                  MOVLW    0x03                 ; Count the three primary colour data.
NEO_RGB_COUNT     RRF      TEMP                 ; Get the next bit of the fade data.
                  BTFSS    STATUS, C            ; If 0 fade primary colour down, if 1 fade primary colour up.
                  INCF     INDF
                  BTFSC    STATUS, C
                  DECF     INDF
                  BTFSS    STATUS, Z            ; If primary colour is value zero, undo the fade of the colour,
                  GOTO     NEO_NO_UNDO          ; to prevent colour value going from full off to full on.
                  BTFSS    STATUS, C            ; If 1 fade primary colour down, if 0 fade primary colour up.
                  DECF     INDF
                  BTFSC    STATUS, C
                  INCF     INDF
NEO_NO_UNDO       INCF     FSR                  ; Point to next primary colour for LED data or next LED data.
                  ADDLW    0xFF                 ; Updata all primary colour data.
                  BTFSS    STATUS, Z
                  GOTO     NEO_RGB_COUNT
                  DECFSZ   NEO_LED_COUNT        ; Update all LED data.
                  GOTO     NEO_LED_INIT_DATA
                  RETURN



;/******************************************/
;/* Refresh the data on the NEO LED array. */
;/******************************************/
NEO_LED_REFRESH   BCF      INTCON, GIE          ; Disable interupts.

                  MOVLW    NEO_LED_G1_DATA      ; Point to first LED data byte.
                  MOVWF    FSR
                  MOVLW    NEO_ARRAY_SIZE       ; Update all LEDs in array.
                  MOVWF    NEO_LED_COUNT
NEO_LED_UPDATE    CALL     NEO_SET_LED_RGB      ; Send RGB data to an LED in the array.
                  DECFSZ   NEO_LED_COUNT
                  GOTO     NEO_LED_UPDATE
                  CALL     NEO_RESET_LED_RGB    ; Send a low signal to indicate the end of data and a reset back to first LED.

                  BSF      INTCON, GIE          ; Enable interupts.
                  RETURN



;/*************************************************/
;/* At the end of sending array data to NEO LEDs, */
;/* a 100uS low signal is required to reset to    */
;/* start of array, for next LED data update.     */
;/*************************************************/
NEO_RESET_LED_RGB MOVLW    0xFF
NEO_RESET_LOOP    NOP
                  NOP
                  ADDLW    0xFF
                  BTFSS    STATUS, Z
                  GOTO     NEO_RESET_LOOP
                  RETURN



;/***********************************/
;/* Send the RGB data to a NEO LED. */
;/***********************************/
NEO_SET_LED_RGB   MOVLW    0x03                 ; Send three primary colour data.
                  MOVWF    NEO_LED_RGB_COUNT
SET_LED_NEXT      MOVFW    INDF                 ; Get the next primary colour data byte.
                  MOVWF    NEO_LED_TEMP
                  MOVLW    0x08                 ; Send eight bits of data for the byte.
NEO_SET_LED_LOOP  RLF      NEO_LED_TEMP         ; Get the next data bit into the carry flag.
                  BTFSC    STATUS, C            ; Send high or low signal depending on the carry flag.
                  GOTO     NEO_DO_BIT_HIGH
                  CALL     NEO_BIT_LOW
                  GOTO     NEO_DO_BIT_LOW
NEO_DO_BIT_HIGH   CALL     NEO_BIT_HIGH
NEO_DO_BIT_LOW    ADDLW    0xFF                 ; Send the remaining bits of data for the byte.
                  BTFSS    STATUS, Z
                  GOTO     NEO_SET_LED_LOOP
                  INCF     FSR                  ; Send the remaining primary colour data.
                  DECFSZ   NEO_LED_RGB_COUNT
                  GOTO     SET_LED_NEXT
                  RETURN



;/*************************************/
;/* Send a low data bit to a NEO LED. */
;/* TIMING BASED ON A 20MHz CRYSTAL   */
;/* TO PROVIDE ACCURAGE PERIODS FOR   */
;/* THE NEO LED ARRAY.                */
;/*************************************/
NEO_BIT_LOW       BSF      GPIO, GPIO_NEO_LED_DATA
                  BSF      GPIO, GPIO_NEO_LED_DATA
                  BCF      GPIO, GPIO_NEO_LED_DATA
                  RETURN



;/**************************************/
;/* Send a high data bit to a NEO LED. */
;/* TIMING BASED ON A 20MHz CRYSTAL    */
;/* TO PROVIDE ACCURAGE PERIODS FOR    */
;/* THE NEO LED ARRAY.                 */
;/**************************************/
NEO_BIT_HIGH      BSF      GPIO, GPIO_NEO_LED_DATA
                  BSF      GPIO, GPIO_NEO_LED_DATA
                  BSF      GPIO, GPIO_NEO_LED_DATA
                  BSF      GPIO, GPIO_NEO_LED_DATA
                  BCF      GPIO, GPIO_NEO_LED_DATA
                  RETURN



;/*****************************/
;/* Reset a RAM area to 0x00. */
;/*                           */
;/* CALL WITH:                */
;/* FSR - Start RAM address.  */
;/* W   - Byte clear count.   */
;/*****************************/
CLEAR_RAM         CLRF     INDF
                  INCF     FSR
                  ADDLW    0xFF                 ; Subtract 1.
                  BTFSS    STATUS, Z
                  GOTO     CLEAR_RAM
                  RETURN



                  ORG      0x0300

ORG_DATA_TABLES



;/**************************/
;/* Write data to user ID. */
;/**************************/
                  ORG      0x2000 ; Area for PIC ID - compiler will complain.

                  DE       0x00, 0x0A, 0x00, 0x01 ; Product ID, Software Version



;/*************************/
;/* Write data to EEPROM. */
;/*************************/
                  ORG      0x2100

ORG_EEPROM_MEMORY


END

