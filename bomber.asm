    processor 6502

;=======================================================
; Include required files with VCS register memory mapping and macros
;=======================================================
    include "vcs.h"
    include "macro.h"

;=======================================================
; Declare the variables starting from memory address $80
;=======================================================
    seg.u Variables
    org $80

JetXPos		byte		; player0 x-pos
JetYPos		byte		; player0 y-pos
BomberXPos	byte		; player1 x-pos
BomberYPos	byte 		; player1 y-pos
MissileXPos	byte		; missile x-pos
MissileYPos	byte		; missile y-pos
Score		byte 		; 2-digit score stored as BCD
Timer		byte 		; 2-digit timer stored as BCD
Temp		byte		; auxiliar variable to store temporary score
OnesDigitOffset	word		; lookup table offset for the score 1's digit
TensDigitOffset	word		; lookup table offset for the score 10's digit
JetSpritePtr	word		; pointer to player0 sprite lookup table
JetColorPtr	word		; pointer to player0 color lookup table
BomberSpritePtr	word		; pointer to player1 sprite lookup table
BomberColorPtr	word		; pointer to player1 color lookup table
JetAnimOffset   byte		; player0 sprite frame offset for animation
Random          byte		; random number generated to set enemy position
ScoreSprite	byte		; store the sprite bit pattern for the score
TimerSprite	byte		; store the sprite bit pattern for the timer
TerrainColor	byte		; store the color of the terrain
RiverColor	byte 		; store the color of the river

;=======================================================
; Define constants
;=======================================================
JET_HEIGHT = 9			; player0 sprite height (# rows in lookup table)
BOMBER_HEIGHT = 9		; player1 sprite height (# rows in lookup table)
DIGITS_HEIGHT = 5		; scoreboard digit height (# rows in lt)

;=======================================================
; Start our ROM code at memory address $F000
;=======================================================
    seg Code
    org $F000

Reset:
    CLEAN_START			; call macro to reset memory and registers

;=======================================================
; Initialize RAM variables
;=======================================================
    lda #10
    sta JetYPos			; JetYPos = 10
    lda #68
    sta JetXPos			; JetXPos = 60
    lda #83
    sta BomberYPos		; BomberYPos = 83
    lda #62
    sta BomberXPos		; BomberXPos = 54
    lda #%11010100
    sta Random			; Random = $D4
    lda #0
    sta Score
    sta Timer			; Score = Timer = 0

;=======================================================
; Declare a MACRO to check if we should display the missile 0
;=======================================================
    MAC DRAW_MISSILE
        lda #%00000000
        cpx MissileYPos		; compare X (current scanline) with missYPos
        bne .SkipMissileDraw	; if X != missile Y position, skip draw
.DrawMissile:
        lda #%00000010		; else: enable missile 0 display
        inc MissileYPos 
.SkipMissileDraw:
        sta ENAM0		; store correct value in the TIA miss register
    ENDM

;=======================================================
; Initialize pointers to the correct lookup table addresses
;=======================================================
    lda #<JetSprite
    sta JetSpritePtr		; lo-byte pointer for jet sprite lookup table
    lda #>JetSprite
    sta JetSpritePtr+1		; hi-byte pointer for jet sprite lookup table

    lda #<JetColor
    sta JetColorPtr
    lda #>JetColor
    sta JetColorPtr+1

    lda #<BomberSprite
    sta BomberSpritePtr
    lda #>BomberSprite
    sta BomberSpritePtr+1

    lda #<BomberColor
    sta BomberColorPtr
    lda #>BomberColor
    sta BomberColorPtr+1

;=======================================================
; Start the main display loop and frame rendering
;=======================================================
StartFrame:

;=======================================================
; Display VSYNC and VBLANK
;=======================================================
    lda #2
    sta VSYNC			; turn on VSYNC
    sta VBLANK			; turn on VBLANK
    REPEAT 3
        sta WSYNC		; display 3 recommended lines of VSYNC
    REPEND
    lda #0
    sta VSYNC			; turn off VSYNC
    REPEAT 31			; 31 + 6 lines of subroutines and tasks
        sta WSYNC		; display recommended lines of VBLANK
    REPEND

;=======================================================
; Calculations and tasks performed in VBLANK
;=======================================================
    lda JetXPos
    ldy #0
    jsr SetObjectXPos		; set player0 horizontal position

    lda BomberXPos
    ldy #1
    jsr SetObjectXPos		; set player1 horizontal position

    lda MissileXPos
    ldy #2
    jsr SetObjectXPos		; set missile horizontal position

    jsr CalculateDigitOffset	; calculate scoreboard digits lt offset

    jsr GenerateJetSound	; configure and enable jet engine audio

    sta WSYNC
    sta HMOVE   

    lda #0
    sta VBLANK

;=======================================================
; Display scoreboard lines
;=======================================================
    lda #0			; clear TIA registers before each new frame
    sta PF0
    sta PF1
    sta PF2
    sta GRP0
    sta GRP1
    ;lda #$1C			; set playfield/scoreboard color to white
    ;sta COLUPF
    ;lda #%00000000		; do not reflect
    sta CTRLPF			; disable playfield reflection
    sta COLUBK

    lda #$1E
    sta COLUPF			; set the playfield/scoreboard color to yellow

    ldx #DIGITS_HEIGHT		; initialize X counter with 5 (digit height)
    
.ScoreDigitLoop
    ldy TensDigitOffset		; get the tens digit offset for the score
    lda Digits,Y		; load the bit pattern from lookup table
    and #$F0			; mask the graphics for the ones digit
    sta ScoreSprite		; save the score tens digit pattern

    ldy OnesDigitOffset		; get the ones digit offset for the Score
    lda Digits,Y		; load the digit bit pattern from lt
    and #$0F			; mask the graphics for the tens digit
    ora ScoreSprite		; merge it with the saved tens digit sprite
    sta ScoreSprite		; save it
    sta WSYNC			; wait for the end of the scanline
    sta PF1			; update the playfield to display score sprite

    ldy TensDigitOffset+1	; get the left digit offset for the Timer
    lda Digits,Y		; load the digit pattern from lt
    and #$F0			; mask the graphics for the ones digit
    sta TimerSprite 		; save the timer tens digit pattern

    sta OnesDigitOffset+1	; get the ones digit offset for the Timer
    lda Digits,Y		; load digit pattern from the lt
    and #$0F			; mask the graphics for the tens digit
    ora TimerSprite		; merge with the saved tens digit graphics
    sta TimerSprite		; save it

    jsr Sleep12Cycles		; wastes some cycles

    sta PF1			; update the playfield for Timer display

    ldy ScoreSprite		; preload for the next scanline
    sta WSYNC			; wait for next scanline

    sty PF1			; update playfield for the Score display
    inc TensDigitOffset
    inc TensDigitOffset+1
    inc OnesDigitOffset
    inc OnesDigitOffset+1	; inc all digits for the next line of data

    jsr Sleep12Cycles

    dex				; --X
    sta PF1			; update the playfield for the Timer display
    bne .ScoreDigitLoop		; if dex != 0, branch to ScoreDigitLoop

    sta WSYNC

    lda #0
    sta PF0
    sta PF1
    sta PF2
    sta WSYNC
    sta WSYNC
    sta WSYNC

;=======================================================
; Display the 84 visible scanlines of our main game
; (because of the 2-scanline kernel)
;=======================================================
GameVisibleLine:
    lda TerrainColor
    sta COLUPF			; set the terrain background color

    lda RiverColor
    sta COLUBK			; set the river background color

    lda #%00000001
    sta CTRLPF			; enable playfield reflection

    lda #$F0
    sta PF0			; set PF0 bit pattern
    lda #$FC
    sta PF1
    lda #0
    sta PF2

    ldx #83			; X counts the number of remaining scanlines
.GameLineLoop:
    DRAW_MISSILE		; macro to check if we should draw the missile

.InsideJetSprite:
    txa 			; transfer X to the accumulator
    sec				; make sure the carry flag is set before sbc
    sbc	JetYPos			; subtract sprite Y-coordinate
    cmp #JET_HEIGHT		; is the current scanline inside the sprite height?
    bcc .DrawSpriteP0		; if result < SpriteHeight, call draw routine
    lda #0
.DrawSpriteP0:
    clc				; clear carry flag before addition
    adc JetAnimOffset		; jump to sprite frame address in memory
    tay				; load Y so we can work with the pointer
    lda (JetSpritePtr),Y	; load player0 bitmap data from lookup table
    sta WSYNC			; wait for scanline
    sta GRP0			; set graphics for player0
    lda (JetColorPtr),Y		; load player color from lookup table
    sta COLUP0			; set color of player0

.InsideBomberSprite:
    txa
    sec
    sbc BomberYPos
    cmp #BOMBER_HEIGHT
    bcc .DrawSpriteP1
    lda #0
.DrawSpriteP1:
    tay
    lda #%00000101		; stretch player 1 sprite
    sta NUSIZ1			; stretch player 1 sprite
    lda (BomberSpritePtr),Y	; Y is the only register allowed to work with pointers
    sta WSYNC
    sta GRP1
    lda (BomberColorPtr),Y
    sta COLUP1

    dex				; --X
    bne .GameLineLoop		; repeat next main game scanline until finished

    lda #0
    sta JetAnimOffset		; reset jet animation frame to zero each frame

    sta WSYNC			; wait for a scanline

;=======================================================
; Display overscan
;=======================================================
    lda #2
    sta VBLANK			; turn VBLANK on again
    REPEAT 30
        sta WSYNC		; display 30 recommended lines of VBLANK overscan
    REPEND
    lda #0
    sta VBLANK

;=======================================================
; Process joystick input for player0
;=======================================================
CheckP0Up:
    lda #%00010000		; player0 joystick up
    bit SWCHA
    bne CheckP0Down		; if bit pattern doesn't match, bypass Up block
    lda JetYPos
    cmp #70
    bpl CheckP0Down
    inc JetYPos
    lda #0
    sta JetAnimOffset		; reset sprite animation to first frame

CheckP0Down:
    lda #%00100000		; player0 joystick down
    bit SWCHA
    bne CheckP0Left		; if, bypass Down block
    lda JetYPos
    cmp #5
    bmi CheckP0Left
    dec JetYPos
    lda #0
    sta JetAnimOffset

CheckP0Left:
    lda #%01000000		; player0 joystick left
    bit SWCHA
    bne CheckP0Right		; if, bypass Left block
    lda JetXPos
    cmp #35
    bmi CheckP0Right
    dec JetXPos
    lda #JET_HEIGHT		; 9
    sta JetAnimOffset		; set animation offset to the second frame

CheckP0Right:
    lda #%10000000		; player0 joystick right
    bit SWCHA
    bne CheckButtonPressed	; if, bypass Right block
    lda JetXPos
    cmp #100
    bpl CheckButtonPressed
    inc JetXPos
    lda #JET_HEIGHT		; 9
    sta JetAnimOffset		; set animation offset to the second frame

CheckButtonPressed:
    lda #%10000000		; if button is pressed
    bit INPT4
    bne EndInputCheck
    
    lda JetXPos
    clc
    adc #5
    sta MissileXPos		; set the missXpos equal to player 0
    lda JetYPos
    clc
    adc #8
    sta MissileYPos             ; set the missYPos equal to player 0

EndInputCheck:			; fallback when no input was performed

;=======================================================
; Calculations to update position for next frame
;=======================================================
UpdateBomberPosition:
    lda BomberYPos
    clc
    cmp #0 			; compare bomber y-position with 0
    bmi .ResetBomberPosition    ; if it is < 0, reset y-pos back to the top
    dec BomberYPos		; else, decrement Y-position for next frame
    jmp EndPositionUpdate 
.ResetBomberPosition
    jsr GetRandomBomberPos	; call subroutine for random x-position
    
.SetScoreValues
    sed				; set BCD mode for score and timer values
    lda Timer
    clc
    adc #1
    sta Timer			; add 1 to the Timer variable
    cld				; disable BCD mode after updating S&T

EndPositionUpdate:		; fallback for the position update code

;=======================================================
; Check for object collision
;=======================================================
CheckCollisionP0P1:
    lda #%10000000		; CXPPMM register bit 7 detects P0P1 collision
    bit CXPPMM
    bne .P0P1Collided		; collision P0P1 happened
    jsr SetTerrainRiverColor
    jmp CheckCollisionM0P1	; skip to next check
.P0P1Collided
    jsr GameOver		; call GameOver subroutine

CheckCollisionM0P1:
    lda #%10000000		; CXM0P bit 7 detects M0 and P1 collision
    bit CXM0P			; check CXM0P bit 7 with the above pattern
    bne .M0P1Collided		; collision missile 0 and player 1 happened
    jmp EndCollisionCheck
.M0P1Collided:
    sed
    lda Score
    clc
    adc #1
    sta Score			; adds 1 to the score using decimal mode
    cld

    lda #0
    sta MissileYPos		; reset the missile position

EndCollisionCheck:		; fallback
    sta CXCLR			; clear all collision flags before next frame

;=======================================================
; Loop back to start a brand new frame
;=======================================================
    jmp StartFrame		; continue to display the next frame

;=======================================================
; Subroutine to generate audio for the jet engine sound
;=======================================================
GenerateJetSound subroutine
    lda #3
    sta AUDV0			; set the new audio volume register

    lda JetYPos			; loads A with jet y-pos
    lsr
    lsr
    lsr				; divide A by 8 using right-shifts
    sta Temp			; save Y/8 value in a Temp variable
    lda #31
    sec
    sbc Temp			; subtract 31 - (Y/8)
    sta AUDF0			; set the new audio frequency register

    lda #8
    sta AUDC0			; set thte new audio tone type register

    rts

;=======================================================
; Subroutine to set the colors for the terrain and river to green & blue
;=======================================================
SetTerrainRiverColor subroutine
    lda #$C2
    sta TerrainColor		; set terrain color to green
    lda #$84
    sta RiverColor		; set river color to blue
    rts

;=======================================================
; Subroutine to handle object horizontal position with fine offset
;=======================================================
; A is the target x-coordinate position in pixels of our object
; Y is the object type 
; (0: player0, 1: player1, 2: missile0, 3: missile1, 4: ball)
;=======================================================
SetObjectXPos subroutine
    sta WSYNC			; start a fresh new scanline
    sec				; makee sure carry flag is set before sbc
.Div15Loop
    sbc #15			; subtract 15 from the accumulator
    bcs .Div15Loop		; loop until carry flag is clear
    eor #7			; handle offset range from -8 to 7
    asl
    asl
    asl
    asl				; four shift lefts to get only the top 4 bits
    sta HMP0,Y			; store the fine offset to the correct HMxx
    sta RESP0,Y			; fix object position in 15-step increment
    rts

;=======================================================
; Subroutine for the game over condition
;=======================================================
;
;=======================================================
GameOver subroutine
    lda #$30
    sta TerrainColor		; set terrain color to red
    sta RiverColor		; set river color to red
    lda #0
    sta Score			; Score = 0
    rts

;=======================================================
; Subroutine to generate a LFSR random number
;=======================================================
; Generate a LFSR random number
; Divide the random value by 4 to limit the size of the result to match
; the river. Add 30 to compensate for the left green playfield.
;=======================================================
GetRandomBomberPos subroutine
    lda Random
    asl
    eor Random
    asl
    eor Random
    asl
    asl
    eor Random
    asl
    rol Random 			; perform a series of shift and bit operations

    lsr
    lsr				; divide the value by 4 with 2 right shifts
    sta BomberXPos 		; save it to the variable BomberXPos
    lda #30
    adc BomberXPos		; adds 30 + BomberXPos to compensate left PF
    sta BomberXPos		; and sets the new value to the bomber x-pos

    lda #96
    sta BomberYPos		; sets the y-pos to the top of the screen

    rts

;=======================================================
; Subroutine to handle scoreboard digits to be displayed on the screen
;=======================================================
; Convert the high and low nybbles of the variables Score and Timer
; into the offsets of digits lookup table so the values can be displayed.
; Each digit has a height of 5 bytes in the lookup table.
;
; For the low nybble we need to multiply by 5 (height)
;	- we can use left shifts to perform multiplication by 2
; 	- for any number N, the value of N*5 = (N*2*2)+N
;
; For the high nybble, since it is already times 16, we need to divide it
; and then multiply by 5:
;	- use right shifts to perform division by 2
;	- for any number N, the value of (N/16)*5 = (N/2/2)+(N/2/2/2/2)
;=======================================================
CalculateDigitOffset subroutine
    ldx #1			; X register is the loop counter
.PrepareScoreLoop		; loop twice, first X = 1, then X = 0
    lda Score,X			; load A with the Timer (X = 1) or Score (X = 0)
				; Score address + X is Score or Timer
    				; because they are consecutive in memory
    and #$0F			; remove tens digit by masking second nybble
    sta Temp			; save the value of A into Temp
    asl				; shift left (now N*2)
    asl				; shift left (now N*4)
    adc Temp			; add the value saved in Temp (+N)
    sta OnesDigitOffset,X	; save A in OnesDigitOffset + 1 or + 0

    lda Score,X			; load A with Timer or Score
    and #$F0			; remove ones digit by masking first nybble
    lsr				; shift right (now N/2)
    lsr				; shift right (now N/4)
    sta Temp			; save the value of A into Temp
    lsr				; shift right (now N/8)
    lsr				; shift right (now N/16)
    adc Temp			; add the value saved in Temp (N/16+N/4)
    sta TensDigitOffset,X	; store A in TensDigitOffset+1 or + 0

    dex
    bpl .PrepareScoreLoop	; branch if X >= 0
    rts

;=======================================================
; Subroutine to waste 12 CPU cycles
;=======================================================
; jsr takes 6 cycles
; rts takes 6 cycles
;=======================================================
Sleep12Cycles subroutine
    rts

;=======================================================
; Declare ROM lookup tables
;=======================================================
Digits:
        .byte %01110111
        .byte %01010101
        .byte %01010101
        .byte %01010101
        .byte %01110111
        
        .byte %00010001
        .byte %00010001
        .byte %00010001
        .byte %00010001        
        .byte %00010001
        
        .byte %01110111
        .byte %00010001
        .byte %01110111
        .byte %01000100
        .byte %01110111
        
        .byte %01110111
        .byte %00010001
        .byte %00110011
        .byte %00010001
        .byte %01110111
        
        .byte %01010101
        .byte %01010101
        .byte %01110111
        .byte %00010001
        .byte %00010001
        
        .byte %01110111
        .byte %01000100
        .byte %01110111
        .byte %00010001
        .byte %01110111
           
        .byte %01110111
        .byte %01000100
        .byte %01110111
        .byte %01010101
        .byte %01110111
        
        .byte %01110111
        .byte %00010001
        .byte %00010001
        .byte %00010001
        .byte %00010001
        
        .byte %01110111
        .byte %01010101
        .byte %01110111
        .byte %01010101
        .byte %01110111
        
        .byte %01110111
        .byte %01010101
        .byte %01110111
        .byte %00010001
        .byte %01110111
        
        .byte %00100010
        .byte %01010101
        .byte %01110111
        .byte %01010101
        .byte %01010101
         
        .byte %01100110
        .byte %01010101
        .byte %01100110
        .byte %01010101
        .byte %01100110
        
        .byte %00110011
        .byte %01000100
        .byte %01000100
        .byte %01000100
        .byte %00110011
        
        .byte %01100110
        .byte %01010101
        .byte %01010101
        .byte %01010101
        .byte %01100110
        
        .byte %01110111
        .byte %01000100
        .byte %01100110
        .byte %01000100
        .byte %01110111
        
        .byte %01110111
        .byte %01000100
        .byte %01100110
        .byte %01000100
        .byte %01000100
 
JetSprite:
    .byte #%00000000         ;
    .byte #%00010100         ;   # #
    .byte #%01111111         ; #######
    .byte #%00111110         ;  #####
    .byte #%00011100         ;   ###
    .byte #%00011100         ;   ###
    .byte #%00001000         ;    #
    .byte #%00001000         ;    #
    .byte #%00001000         ;    #

;JET_HEIGHT = . - JetSprite	; current location (memory address) - 9

JetSpriteTurn:
    .byte #%00000000         ;
    .byte #%00001000         ;    #
    .byte #%00111110         ;  #####
    .byte #%00011100         ;   ###
    .byte #%00011100         ;   ###
    .byte #%00011100         ;   ###
    .byte #%00001000         ;    #
    .byte #%00001000         ;    #
    .byte #%00001000         ;    #

BomberSprite:
    .byte #%00000000         ;
    .byte #%00001000         ;    #
    .byte #%00001000         ;    #
    .byte #%00101010         ;  # # #
    .byte #%00111110         ;  #####
    .byte #%01111111         ; #######
    .byte #%00101010         ;  # # #
    .byte #%00001000         ;    #
    .byte #%00011100         ;   ###

JetColor:
    .byte #$00
    .byte #$FE
    .byte #$0C
    .byte #$0E
    .byte #$0E
    .byte #$04
    .byte #$BA
    .byte #$0E
    .byte #$08

JetColorTurn:
    .byte #$00
    .byte #$FE
    .byte #$0C
    .byte #$0E
    .byte #$0E
    .byte #$04
    .byte #$0E
    .byte #$0E
    .byte #$08

BomberColor:
    .byte #$00
    .byte #$32
    .byte #$32
    .byte #$0E
    .byte #$40
    .byte #$40
    .byte #$40
    .byte #$40
    .byte #$40

;=======================================================
; Complete ROM size with exactly 4 KB
;=======================================================
    org $FFFC			; move to position $FFFC
    word Reset			; write 2 bytes with the program reset address
    word Reset			; write 2 bytes with the interruption vector
