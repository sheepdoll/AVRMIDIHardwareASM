; Octet compatability version  mega88 only

#define HWNOTES48
;#def HWNOTES128


.include "m88def.inc"


;	pins are: (graphic is Ic pin layout)

	; portD mapping
	; PD7 is not used 
	; PD6 is used to set "omni" or octet compatable mode by shorting to ground
	; PD5 is Shift register clock SRCK and configured as a timer cascade
	; PD4 is shift register gate signal	
	; PD3 is LED out
	; PD2 is LED out
	; PD1 is MIDI out
	; PD0 is MIDI in

;   Octet RE hardware						Arduino compatible Hardware 
	--------------------------				----------------------------------
;	01 RST  6		5 ADDR0	28				01 RST	 		A5	ADDR0	28	SCL
;	02 NC   0		4 ADDR1	27				02 RXD	D0		A4	ADDR1	27	SDA
;	03 TXD  1		3 ADDR2	26				03 TXD	D1		A3	ADDR2	26
;	04 LED  2		2 ADDR3 25				04 IND1	D2		A2	ADDR3	25
;	05 LED  3		1 OCMO  24				05 IND2	D3		A1	ADDR4	24
;	06 SRGT	4		0 INVT	23				06 OEn	D4		A0	ADDR5	23
;	07 VCC			  GND	22				07 VCC				GND		22 
;	08 GND			  AREF	21				08 GND				AREF	21
;	09 XTL			  AVCC	20				09 XTL				AVCC	20
;	10 XTL			5 SCK	19				10 XTL			D13 SCK		19  OPT2
;	11 SRCK 5		4 MISO	18				11 SRCK	D5		D12 MISO	18  OPT3
;	12 OMNI 6		3 MOSI	17				12 RCK	D6		D11 MOSI	17  OPT4
;	13 NC	7		2 SSn	16				13 SRD	D7		D10 SDSS	16
;	14 SRD  0		1 NC	15				14 INDR	D8		D9  INDG	15


.equ kBaud31250		= 15	; this sets the MIDI baud rate to 31250
; MIDI and file system
;FFlags:
.equ playactive  	= 7		; set when song is using interrupts
.equ sysxcntu   	= 6		; file playback is abs time
.equ eot         	= 5		; buffer is empty
.equ playall     	= 4		; set when chaining songs
.equ paused      	= 3		; mode is paused
.equ running     	= 2		; running status active
.equ fseof       	= 1		; physical end of file
.equ pending     	= 0		; last cluster is not in SRAM
;
; Disassembly of Um0v_19.bin - from $1000 to $FFFF

; MIDI and file system

.def zero		= r1	; keep a zero handy
.def const7		= r2	; temporary shift register (cause 88 breaks things)
.def nibbleMask	= r3	; should always be 0x0F
.def iRH		= r4	; mask for port C reads and writes
.def iRT		= r5	; out ring tail
.def oRH		= r6	; in ring head
.def oRT		= r7	; in ring tail
.def ashift		= r8	; extended wave generation
.def bshift		= r9	; extended wave generation

.def MIDI0		= r10
.def MIDI1		= r11
.def MIDI2		= r12
.def MainChnl	= r13	; set to 0x3F constant for in ring mask 
.def status		= r14	; saved status byte for run
.def ssreg    	= r15

.def Needed		= R16	; used in RX interrupt
.def FFLAGS		= R17	; Octet state flags
.def L0040		= R18	;
.def IOMask		= R19	; used in midi parser
.def idx		= R20	; temporary register
.def c_tmp     	= r21 	; passed arguments
.def ARGL     	= r22 	; passed arguments
.def ARGH     	= r23 	; to OS, usually a pointer
.def ACC      	= r24 	; Old fashioned double wideaccumulator
.def BCC      	= r25 	;  


.DSEG                		; Start data segment 
.ORG sram_start            	; Start data segment 
#ifdef HWNOTES48
BITStateTable:	.byte 6		; six bytes for the bitmap  these bits ring through the shift regiser
#endif
#ifdef HWNOTES128
BITStateTable:	.byte 16	; these bits ring through the shift regiser
#endif
NoteStateTable: .byte 1*128 ; bitmap table of active notes
EndNoteStateTable:							


L0F00: .byte 1	;	=	$0F00	; shadows 5D
L0041: .byte 1	;	=	$0041
L0042: .byte 2	;	=	$0042
L0044: .byte 2	;	=	$0044
L0046: .byte 2	;	=	$0046
L0048: .byte 2	;	=	$0048
L004A: .byte 2	;	=	$004A
L004C: .byte 2	;	=	$004C
L004E: .byte 2	;	=	$004E
L0050: .byte 2	;	=	$0050
;Ring1In: .byte 2	;	=	$0052
;Ring1Out: .byte 2	; =	$0054
;Ring2In: .byte 2	;	=	$0056
;Ring2Out: .byte 2	; =	$0058
L005A: .byte 1	;	=	$005A
L005B: .byte 1	;	=	$005B
L005C: .byte 1	;	=	$005C

L0061: .byte 1	;	=	$0061
L0063: .byte 1	;	=	$0063
L0065: .byte 1	;	=	$0065
L0067: .byte 1	;	=	$0067
L0069: .byte 1	;	=	$0069
L006B: .byte 1	;	=	$006B
L006D: .byte 1	;	=	$006D
L006F: .byte 1	;	=	$006F
;Needed	=	$0096
L0099: .byte 1	;	=	$0099
L009A: .byte 1	;	=	$009A
L009D: .byte 1	;	=	$009D
L009E: .byte 1	;	=	$009E
L009F: .byte 1	;	=	$009F
L00A0: .byte 1	;	=	$00A0
L00B1: .byte 1	;	=	$00B1
L00B3: .byte 1	;	=	$00B3
L00B5: .byte 1	;	=	$00B5
L00B6: .byte 1	;	=	$00B6
L00C8: .byte 1	;	=	$00C8
L00CB: .byte 1	;	=	$00CB
L00FA: .byte 1	;	=	$00FA
L00FB: .byte 1	;	=	$00FB
L00FC: .byte 1	;	=	$00FC
;MainChnl =	$00FD
SubChnl: .byte 1	;	=	$00FE
L00FF: .byte 1	;	=	$00FF
L0104: .byte 1	;	=	$0104
L08FF: .byte 1	;	=	$08FF
L0A00: .byte 1	;	=	$0A00
L0AFF: .byte 1	;	=	$0AFF
L0B00: .byte 1	;	=	$0B00
; ram shadow of eeprom tables
.org $200  ; alignment for debug
E000: .byte 2 ; prefix
E002: .byte 10 ; Serial Number
PRODUCT:
	.byte 6
DATE:
	.byte 8
HardwareRev:	
	.byte 2
FirmwareRev:
	.byte 2
LE01E:
	.byte 2
SoftMode1:
	.byte 1
SoftMode2:
	.byte 1
PartAChnl:
	.byte 1
PartBChnl:
	.byte 1
LE024:
	.byte 3
SwitchBounce:	
	.byte 2
MFGRString:
	.byte 62
	;	.123456789.123456789.123456789.123456789  .
	.;db "Octet Design Corp. - MIDI Processor UM0",0x00
	;.db 0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20
	;	1    2    3    4    5    6    7    8    9    .    1    2    3    4
	;.db 0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20
SustainChnl:
	.byte 2
CHANNELS:
	.byte 16
STARTNOTES:
	.byte 16
STOPNOTES:
	.byte 16
OUTSTARTNOTE:
	.byte 16
wparmlen:
 	  .byte 55
E0E0: .byte 32

iRingHead: .byte 256
oRingHead: .byte 256
RingTail:



.eseg
.org 0

WorldRecord:
	.db	$04, $A9
;SERNUM:
	.db "0023171819" ; this is the serial #
;PRODUCT:
	.db "   UM0",0x30 + (__MONTH__ / 10)
;DATE:
	.db 0x30 + __MONTH__ - ((__MONTH__ / 10) * 10),'-'
	.db 0x30 + (__DAY__ / 10),0x30 + __DAY__ - ((__DAY__ / 10) * 10)
	.db '-',0x30 + (__YEAR__ / 10)
	.db 0x30 + __YEAR__ - ((__YEAR__ / 10) * 10)
;HardwareRev:	
	.db 0x03,0x03
;FirmwareRev:
	.db 0x02,0x03
;LE01E:
	.db 0x01,0x04
;SoftMode1:
	.db 0x00
;SoftMode2:
	.db 0x01
;PartAChnl:
	.db	$00
;PartBChnl:
	.db	$00
;LE024:
	.db 0x00,0x00,0x00
;SwitchBounce:	
	.db 0x0A,0x05
;MFGRString:
	.db "Octet Design Corp. - MIDI Processor UM0",0x00
	.db 0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20
	.db 0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20
;SustainChnl:
;   sustain hold mode 
;   0 = Hold notes
;   1 = P2 mode
;	2 = use channel n
	.db 0x01,0x31
;CHANNELS:
.db 0x01,0x00,0x00,0x00,0x00,0x02,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10
;STARTNOTES:
.db 0x29,0x1A,0x12,0x46,0x51,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
;STOPNOTES:
.db 0x58,0x45,0x19,0x50,0x51,0x2c,0x7F,0x7F,0x7F,0x7F,0x7F,0x7F,0x7F,0x7F,0x7F,0x7F
;OUTSTARTNOTE:
.db 0x01,0x01,0x05,0x21,0x20,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
	.byte 55
	.db "UM1 MIDI INTERFACE",0x00
	.byte 12

EndWorld:
;

.cseg

.org 0
	rjmp Main		; reset device

.org INT0addr	; External Interrupt 0
	reti
;	rjmp USBDown
	
.org INT1addr	; External Interrupt 1
	reti
;	rjmp USBRead

; ***** INTERRUPT VECTORS ************************************************
.org	PCI0addr	; Pin Change Interrupt Request 0
	reti
.org	PCI1addr	; Pin Change Interrupt Request 0
	reti
.org	PCI2addr	; Pin Change Interrupt Request 1
	reti
.org	WDTaddr		; Watchdog Time-out Interrupt
	reti
.org	OC2Aaddr	; Timer/Counter2 Compare Match A
	reti
.org	OC2Baddr	; Timer/Counter2 Compare Match A
	reti
.org	OVF2addr	; Timer/Counter2 Overflow
	reti
.org	ICP1addr	; Timer/Counter1 Capture Event
	reti
.org	OC1Aaddr	; Timer/Counter1 Compare Match A
	reti 
.org	OC1Baddr	; Timer/Counter1 Compare Match B
	reti;	rjmp NuLine
.org	OVF1addr	; Timer/Counter1 Overflow
	rjmp STARTSEQ
.org	OC0Aaddr	; TimerCounter0 Compare Match A
	reti 
.org	OC0Baddr	; TimerCounter0 Compare Match B
	rjmp Ov0BTST
.org	OVF0addr	; Timer/Couner0 Overflow
	rjmp SHIFTOUT
.org	SPIaddr		; SPI Serial Transfer Complete
	reti
.org	URXCaddr	; USART Rx Complete
	rjmp	L13EA
.org	UDREaddr	; USART, Data Register Empty
	reti ;rjmp	L1409
.org	UTXCaddr	; USART Tx Complete
	reti
.org	ADCCaddr	; ADC Conversion Complete
	reti
.org	ERDYaddr	; EEPROM Ready
	reti
.org	ACIaddr		; Analog Comparator
	reti ;rjmp CISflip
.org	TWIaddr		; Two-wire Serial Interface
	reti
.org	SPMRaddr	; Store Program Memory Read
	reti


.org INT_VECTORS_SIZE

; Interrupt handlers



STARTSEQ:
	; reset shift register counters
	in ssreg,SREG  ;if we have time
	sbi DDRD,DDD3								; too bright when always on
	ldi R16,1 <<OCF0B | 1 << OCF0A | 1 << TOV0 
	out TIFR0,r16
	mov bshift,const7
	ldi YL,0
	ld ashift,Y+
	cbi DDRD,DDD3
	rjmp PRESHIFT 			; Fencepost start at tick 0

;	out SREG,ssreg
;	reti


SHIFTOUT:
	in ssreg,SREG  ;if we have time
PRESHIFT:
	sbrs ashift,7
	rjmp L0254
	sbic PinC,5
	rjmp L0255
	sbi PORTB,Portb0
	rjmp L0254
L0255:
	cbi PORTB,Portb0
L0254:
	lsl ashift
	dec bshift
	brne L0269
	mov bshift,const7
	ld ashift,Y+
L0269:
	out SREG,ssreg
	reti

Ov0BTST:
	in ssreg,SREG  ;if we have time
	sbic PinC,5
	rjmp L0265
	cbi PORTB,Portb0  ; just clear the data clock
	reti
L0265: 
	sbi PORTB,Portb0  ; just clear the data clock
	out SREG,ssreg
	reti

L13DC:
	; Init or change the baud rate

	ldi	r17,00			; HI not used?

	sts	UBRR0H,r17		; set the Baud divisor
	sts	UBRR0L,r16
		
;	ldi	r16,1<<U2X		; Double speed
;	sts	UCSR0A,r16		
	
	ldi	r16,1<<UCSZ01 | 1<<UCSZ00 ; 8N1
	sts	UCSR0C,r16

	;					 set the interrupt handler flags
	ldi	r16,1<<RXCIE0 | 1<<RXEN0 | 1<<TXEN0
	sts	UCSR0B,r16		

;	sts	D036A,r17		; in ring in
;	sts	D0369,r17		; in ring out
;	sts	D037C,r17		; out ring in
;	sts	D037B,r17		; out ring out
	ret
; pc=0x13EA(0x27D4)
;

; RX handler, optimized for speed - this dedicates the X register 
; to the handler

L13EA:					; Receive RX handler
	in	ssreg,SREG		; protect status
	push XL
	push XH
	push BCC
	WDR	
L0367:					; keep from resetting in active mode
	lds BCC,UCSR0A
	sbrs BCC,RXC0  ; poll for next output byte to send
	rjmp L0367
	lds BCC,UDR0
	;cpi BCC,0xFE		; ignore active sense
	;breq L0380
	mov	 XL,iRH				
	ldi	 XH,00				
	subi XL,low((~iRingHead)+1) 
	sbci XH,high((~iRingHead)+1)
	st	X,BCC		; save input byte into ring
	inc	 iRH					; use a full 256 bytes for ring
L0380:
	pop BCC					; this method gurentees that the ring
	pop XH					; can not overflow
	pop XL
	out	SREG,ssreg		; restore state
	reti

;



;*************************************
; MEP mph MLH Main reset entry point
;*************************************
Main:

; init serial ports for MIDI baud rate
	ldi	r16,low(E000-1) ; setup stack
	out	SPL,r16
	ldi	r16,high(E000-1) ; align tables for easy debug 
	out	SPH,r16

	ldi L0040,0x12		; set nominal flags


;   did the dog bite?  turn it off
	CLI
	WDR
	in ARGL,MCUSR
	sbrs ARGL,WDRF
	rjmp On_Power

	; here becouse the dog reset setup to echo 0xFF to indicate 
	; this was a soft reset or timeout 
	andi ARGL,(0xFF & (0<<WDRF))
	out MCUSR,ARGL
	ldi L0040,0x52 ; overwrite flags for echo
On_Power:

	; we should run with the dog enabled

	ldi ARGH,(1<<WDE) | (1<<WDP2) | (1 << WDP0) ; closer to 500ms
	rcall L1113						; kick the dog

	clr zero
	clr FFLAGS

	clr iRH			; clear the out ring pointers
	clr iRT			; 
	clr oRH			; clear the in ring pointers
	clr oRT


	ldi YL,low(BITStateTable+0xFF)
	ldi YH,high(BITStateTable+0xFF)
L0413:
	st -Y,zero
	cp YL,zero
	brne L0413

	ldi	r16,kBaud31250	; kBaud 15 for midi clock at 8 Mhz	
	rcall	L13DC		; init uart


	ldi ACC,0x08
	mov const7,ACC

	clr status			; buffer used for real time status

	; Option switches	; there are 64 possible configurations
	ldi ACC,0x3F		; mask for input and output rings
	out PORTC,ACC		
						; four switches are used to set channel
						; the remaining two switches specify start octave


	; Enable portD internal pull up resistors and set data direction
	; PD7 is not used 
	; PD6 is used to set "omni" or octet compatable mode by shorting to ground
	; PD5 is Shift register clock SRCK and configured as a timer cascade
	; PD4 is shift register gate signal	
	; PD3 is LED out
	; PD2 is LED out
	; PD1 is MIDI out
	; PD0 is MIDI in
	ldi  ACC, 1 << DDD7  | 1 << DDD6 | 1 << DDD4
	out PORTD,ACC

	ldi ACC,1 << DDD5  | 1 << DDD4 ;| 1 << SI_DD
	out DDRD,ACC       		

	

	; PB5 is SPI SCK
	; PB4 is SPI MISO
	; PB3 is SPI MOSI
	; PB2 is shift register SP
	; PB1 is OC1 cascade clock not used as hardware
	; PB0 is Shift register in (SRIN) data input into the shift register

	; enable shift register drive lines
	ldi ACC, 1 << DDB1  | 1 << DDB2  | 1 << DDB0
	out DDRB,ACC       		

	; cascade clocks to generate proper waveform for idle mode
	; this may also be practical for scanning in the run length mode 
 	ldi ACC,0x13
	out OCR0A,ACC  ; PD6 -> header

	ldi ACC,10
	out OCR0B,ACC  ; PD5 -> T1 -> SCK cascade

	ldi ACC, 1 << COM0B1 | 1 << COM0B0 | 1 << WGM01 | 1 << WGM00
	out TCCR0A,ACC

	ldi ACC, 1 << WGM02 | 1 << CS01
	out TCCR0B,ACC


	; load the main timer - this register can control up to 65,000
	; events 
#ifdef HWNOTES48
	ldi ZH,high(48)   ; active shift lines
	ldi ZL,low(48) 	  ; 
#endif
#ifdef HWNOTES128
	ldi ZH,high(128)   ; active shift lines
	ldi ZL,low(128) 	  ; 
#endif

	sts ICR1H,ZH		; Timer is in ICP top mode
	sts ICR1L,ZL		; 

;	ldi ZH,high(47)   	; we want OCA to shadow the 50 Khz clock
;	ldi ZL,low(47) 	  	; 

;	sts OCR1AH,ZH		; PB1 -> (not connected in hardware)
;	sts OCR1AL,ZL		; 

#ifdef HWNOTES48
	ldi ZH,high(47)   ; active shift lines
	ldi ZL,low(47) 	  ; 
#endif
#ifdef HWNOTES128
	ldi ZH,high(127)   	; this is the duration and duty of the start pulse
	ldi ZL,low(127) 	  ; 
#endif

	sts OCR1BH,ZH		; PB2 ->
	sts OCR1BL,ZL		; 
		; 

	; somewhat complex setup of clock. Mode 14 should allow for our
	; si pulse
	ldi ACC,0 << COM1A1 | 0 << COM1A0 |	1 << COM1B1 | 1 << COM1B0 | 1 << WGM11

	sts TCCR1A,ACC

	ldi ACC,1 << WGM13 | 1 << WGM12 | 1 << CS12 | 1 << CS11	
	sts TCCR1B,ACC			; set timer to trip on write 0 to T1

	
	mov ashift,zero		; t5 and 6track the number of pixels actually scanned
	mov bshift,zero		; tick 03 and 4this will clear the timer

	ldi ACC,0 << OCIE0A | 1 << OCIE0B | 1 << TOIE0		; enable timer 0 interrupt
	sts TIMSK0,ACC


	ldi ACC,0 << OCIE1A | 0 << OCIE1B | 1 << TOIE1		; enable timer 0 interrupt
	sts TIMSK1,ACC

	rcall STARTSEQ 		; reti will start interrupts via a reti

	cbi PORTD,PortD4	; Enable register
;	sbi DDRD,DDD3		; System on  ( we could flash errors if diagnostics are
						; Needed)
	sbrs L0040,6
	rjmp On_Reset
	cbr L0040,1<<6
	ldi BCC,0xFF		; echo back system reset message		;
L0582:
	lds c_tmp,UCSR0A
	sbrs c_tmp,UDRE0  ; poll for next output byte to send
	rjmp L0582
	sts UDR0,BCC
							 
On_Reset:



	ldi ACC,0x3F
	sts	L0F00,ACC
;	mov	L005D,ACC
	
	ldi BCC,$0F
	clr ACC
LE127:
	rcall setSerialD ;  mostly sets flags hc11 calls this 16 times
	dec BCC
	;brge LE127

	ldi ACC,0
	sts	L0042,ACC
	sts	L0044,ACC
	sts	L0046,ACC
	sts	L0048,ACC
	sts	L004A,ACC
	sts	L004C,ACC
	sts	L004E,ACC
	sts	L0050,ACC
	sts	L0061,ACC
	sts	L0063,ACC
	sts	L0065,ACC
	sts	L0067,ACC
	sts	L0069,ACC
	sts	L006B,ACC
	sts	L006D,ACC
	sts	L006F,ACC

; copy EEprom to ram buffer  (mostly this is to facilitate the write back on reload)
	; could check for active write presence
	ldi ZL,low(WorldRecord)
	ldi ZH,high(WorldRecord)
	ldi XL,low(E000)
	ldi XH,high(E000)
L0575:
	out EEARL,ZL
	out EEARH,ZH
	sbi EECR,EERE
	in ARGL,EEDR
	st X+,ARGL
	adiw ZL,1
	cpi XL,low(E000+$FF)
	ldi c_tmp,high(E000+$FF)
	cpc XH,c_tmp
	brlo L0575


	ldi	ACC,0x10
	sts L00FB,ACC
.if 0
	ldX	#$2140
	ldaA	#$FF
LE15B	ldaB	#$FF
	stD	0, X
	stD	2, X
	stD	4, X
	stD	6, X
	stD	8, X
	stD	10, X
	stD	12, X
	stD	14, X
	ldaB	#$10
	aBX	
	dec	>L00FB
	bne	LE15B
.endif

	; ldaA	SoftMode2 read from eeprom
	;staA	L00C8
	;ldaA	LE067
	;staA	L00CB
;	sbr	L0040, 1<<1| 1<<4 ; force midi through on
	;ldaA	SoftMode1
	;staA	L00FF
	; read the main channel from switches
	in BCC,PinC
	;ldaB	PartAChnl
	andi BCC,0b00001111
	mov	MainChnl,BCC
	;ldaB	PartBChnl
	;andB	#%00001111
	sts	SubChnl,MainChnl ; shadow channel state
	clr BCC	
	sbrs	L0040, 2 ;#%00000100, LE1A8
	rjmp LE1A8
	; ldaA	LE024 -from eeprom
	;bitA	#%00010000
	;beq	LE1A8
	;ldaB	L0B00
	;andB	#%00001111
	;subB	MainChnl
LE1A8:
	sts L00FA,BCC
	;ldX	#$E069
	;ldY	#$2000
	;ldaA	#$10
	;staA	L00FB

.if 0
LE1B5	ldaA	0, X
	tAB	
	andB	#%11110000
	addA	L00FA
	andA	#%00001111
	aBA	
	staA	0, Y
	incX	
	incY	
	dec	>L00FB
	bne	LE1B5

	ldaA	MainChnl ; more eeprom stuff
	addA	L00FA
	andA	#%00001111
	staA	MainChnl
	ldaA	SubChnl
	addA	L00FA
	andA	#%00001111
	staA	SubChnl
	ldaA	#$06
	staA	L00FB
	ldX	#$E079
	ldY	#$2010
LE1E5	ldD	0, X
	stD	0, Y
	ldD	2, X
	stD	2, Y
	ldD	4, X
	stD	4, Y
	ldD	6, X
	stD	6, Y
	ldaB	#$08
	aBX	
	aBY	
	dec	>L00FB
	bne	LE1E5
.endif 
	clr ACC	
	sts L00B5,ACC
	sts L00B6,ACC
		
	clr MIDI0
	clr Needed
	sts L005A,ACC
	sts L005B,ACC




;-----------------------------------------------------------------------
; Simple midi parser
;-----------------------------------------------------------------------
PARSEMIDI:
;	sbrs FFLAGS,playactive	; do not reset the dog when active sense is on
	WDR						; let the input ring read keep the dog fed
	rcall _putchar			; this will flush the output buffer when polled
	cp iRH,iRT				; hc11 uses interrup to signal when message
							; parse can take place.  this does not
							; good practice, especially as the compatable
							; hardware uses an interrupt timer for
							; the big shift register

	breq  PARSEMIDI      	;  L142A Ring is empty


	;Read the next byte of data from the ring
	rcall	_getchar    	; will stall here till a byte is ready to dequeue

	sbi DDRD,DDD2			; flash LED 
RETRYSTATE:

	mov MIDI0,BCC			; status and channel packed

	sbrc BCC,7
	rjmp L0448				; command bit is set

	tst status				; check running status state
	breq L0448				; not a running status		

	mov MIDI0,status		; set running status buffer
	mov MIDI1,BCC
	sbr FFLAGS,1 << running

L0448:
	tst MIDI0				; test for bogus status
	brne L0746				; this may be a M$ thing that pads with zeros
	rjmp CANIGNORE			; when a wire command is inserted in the 
							; middle of a message
L0746:

	mov status,MIDI0		; update status for voice messages only
	cbr FFLAGS,1 << running ; this is used to skip the read
							; the status buffer will determine if
							; running status is active


	mov ACC,MIDI0
	swap ACC		
	andi ACC,0x0F     	 	; status >> 4 & 0x0F

	; use a look up table to get the instruction length (this can also
	; be in EEPROM if program space is tight) 

	ldi ZL,low(chantype * 2)
	ldi ZH,high(chantype * 2)

    add ZL,ACC
    adc ZH,zero
    lpm 
	; r0 is set to the variable named needed as it is somewhat trasient here
	mov Needed,r0
	tst r0 ;cp r0,zero;
	brne L0479
	
	rcall WIRECMD			; note the ARGL is not touched 
	rjmp CANIGNORE			; done processing

L0479:
	sbrc FFLAGS,running      ; skip read when running status happens
	rjmp ISRUNNING

	cbr L0040,1<<4			 ; do not echo garbage

		
	rcall _getchar      	; We have a voice category message to parse
 
 	sbrc BCC,7     		; A clock or something could happen anytime
	rcall WIRECMD 			; Process clock or wire command with a call
    
	mov MIDI1,BCC 			; the data byte

	rcall putB_2800	;  echo midi through


ISRUNNING:

	ldi c_tmp,1     		; if needed > 1
	cp r0,c_tmp
	brlo PLY300
	breq PLY300

STORE3:
	rcall _getchar   		; get the third byte
    ; _putchar echo
	sbrc BCC,7         	; Allow wire clock messages
	rcall WIRECMD     		; Process clock or wire command with a call

	mov MIDI2,BCC 
	rcall putB_2800	;  echo midi through


PLY300:
	sbr L0040,1<<4			 ; echo good bytes


	; voice message detected	
	; echo for midi through




  	; filter events here ...

	; at this point we have a one or two byte midi message If it is a note on
	; or a note off we need to map it.

	; test mode switches here if coded C0-3 are channel, C4 is omni
	; C5 is Spencer mode. Ignore if not omni and not selected channel

	cpi ACC,0x0A 			; note on
	brlo PC+2
	rjmp OTHERMSG

	; test here for "omni" mode option (EE prom mapping table) (this will be PD 6)

	sbic PinC,4
	rjmp OCTETMIDIOUT

	; Note processing section

	in ARGH,PinC			; Port C is the switch bank
	mov ARGL,ARGH			; (switches 1 through 4 are channel mask)
	swap ARGL				; (switch 5 & 6 are start octive)
	andi ARGH,0x0F			; Mask channel selector bits										; 
	
	mov c_tmp,MIDI0			; test to see if note is turned on in active channel
	andi c_tmp,0x0F			
	cp ARGH,c_tmp
	breq L0640

	rjmp CANIGNORE			;  Midi message is not one we understand


L0640:
	
	clr ARGL				; ARGL now points to the first magnet to play
	ldi ARGH,128			; set the top limit
	clr r0					; start magnet is always zero

	; scaled MIDI out		-- simply send the MIDI to the wire
	rcall SENDMIDIOUT		; ARGL contains the upper limit, R0 contains the offset added	
							; to shift the octave to the correct start note
	
	rjmp CANIGNORE			; jump to the bottom of the send midi parser loop;
				


; This is the octet compatable mode (short PinC 4 to get here )					

OCTETMIDIOUT: 

	mov ARGH,MIDI0
	andi ARGH,0x0F

	ldi idx,0
NX0597:					; re-entry point, mapping can map notes to more
						; than one output
	ldi ZL,low(CHANNELS)
	ldi ZH,high(CHANNELS)
	add ZL,idx
	adc ZH,zero			; get index into start point

L0585:
	ld r0,Z
	cp ARGH,r0
	breq L0598 

	adiw ZL,1			
	inc idx
	cpi idx,15
	brne L0585
	rjmp CANIGNORE		; done scanning through tables for all active note events
	
	; convert channels to note state table positions
L0598:
	; valid channel is in c_tmp
	ldi ZL,low(STOPNOTES)
	ldi ZH,high(STOPNOTES)
	add ZL,idx
	adc ZH,zero			; get index into start point
	ld ARGH,Z

L0621:
	ldi ZL,low(STARTNOTES)
	ldi ZH,high(STARTNOTES)
	add ZL,idx
	adc ZH,zero			; get index into start point
	ld ARGL,Z
	
	; map channel to byte
	
	ldi ZL,low(OUTSTARTNOTE)
	ldi ZH,high(OUTSTARTNOTE)
	add ZL,idx
	adc ZH,zero			; get inded into start point


	rcall SENDMIDIOUT
	
	rjmp NX0597			; sweep for couplers



SENDMIDIOUT:
	; on entry 
	; ARGL is MIDI start limit
	; ARGH is MIDI stop  limit
	; r0 is start magnet


	mov c_tmp,MIDI1			; lower limit check (start octave is subtracted from the
	sub c_tmp,ARGL			; Requested midi note to shift the note events into a range
							; where the magnets can play them)
	sbrc c_tmp,7			; test for negative -- no need to process further		
	rjmp CANIGNORE			; jump to the bottom of the send midi parser loop;

	cp	MIDI1,ARGH			; notes octave shifted higer than the switches are ignored
	brlo L0760			 
	rjmp CANIGNORE

L0760:

	add c_tmp,r0			; add in the note start offset, which is actually the 
							; valve magnet start offset  -- this is only used when
							; table mapping is active

							; this has to be inserted here after the limit check
							; as the limits are defined in MIDI note space

	mov ARGL,c_tmp			; orig note was saved in this register

; correction to handle when couplers are active
; Note state table will index which note is playing with a simple counting semaphore

	ldi ZL,low(NoteStateTable)
	ldi ZH,high(NoteStateTable)
	add ZL,c_tmp 	; this should be the magnet we are looking for
	adc ZH,zero
	ld  r0,Z 				; this should be the semaphore count

	; handle the case where a velocity of 0 is a note off
	tst MIDI2 ;cp MIDI2,zero
	breq CouplerOFF

	mov c_tmp,MIDI0
	andi c_tmp,0xF0
	cpi c_tmp,0x80

	breq CouplerOFF
	inc r0
	st Z,r0
	rjmp L0761
	
CouplerOFF:
	dec r0
	brlt L0767			; semaphore went negative (off before on)
	st Z,r0
	breq L0761				; note needs to turn off
	; hold note on here as more than one tracker port is open
	rjmp L0824
	
L0767:					;  clear the semaphore and turn note off
	clr r0
	st Z,r0
L0761:
	; get the mask from the mapping table ROM version
	mov c_tmp,ARGL
#ifdef HWNOTES48
	andi c_tmp,0x0F
#endif
#ifdef HWNOTES128
	andi c_tmp,0x07	; there are 8 outputs on a chip
#endif
	ldi ZL,low(defaultNoteMapMask * 2)	; this probably stays in flash
	ldi ZH,high(defaultNoteMapMask * 2)
	add ZL,c_tmp
	adc ZH,zero
	lpm					; r0 contains the mask
	mov IOMask,r0		; this mask will turn the bit on or off

#ifdef HWNOTES48
	ror c_tmp				; even or odd byte into cary
	swap ARGL				; high part this is the byte pair to write	
	rol ARGL				; cary back into byte to separate even from odd
	andi ARGL,0b0000111		; effect is ARGL >> 3	
#endif
#ifdef HWNOTES128
	lsr ARGL				; calculate chip address			
	lsr ARGL				
	lsr ARGL				
	andi ARGL,0b0001111		; effect is ARGL >> 3	
#endif


	; calculate ram note state table address

#ifdef HWNOTES48
	ldi ZL,low(BITStateTable)
	ldi ZH,high(BITStateTable)
	add ZL,ARGL			; the 16 note pair
	adc ZH,zero
#endif
#ifdef HWNOTES128
ldi ZL,low(BITStateTable+15)
	ldi ZH,high(BITStateTable+15)
	sub ZL,ARGL			; the shift regisster latches every 128 clocks
	sbc ZH,zero			; so the pattern is reversed with the first
						; chip addressed on the last clock
#endif


	; handle the case where a velocity of 0 is a note off
	tst MIDI2 ;cp MIDI2,zero
	breq NOTEOFF

	mov c_tmp,MIDI0
	andi c_tmp,0xF0
	cpi c_tmp,0x80
	breq NOTEOFF
	; turn note on  Z points to the byte to mask
	ld c_tmp,Z
	or c_tmp,IOMask	
	rjmp NOTESET	

NOTEOFF:

	; could use counting semaphores on 48 bytes to only turn off notes
	; turned on by a given channel  alternate is 16 x 3 channels of on
	; info which get stored into the actual on state?

	; Z -> base channel
	; MPY channel x 3 and add to Z  -- active channel
	; set or unset bit in active channel
	; merge active channel or 16 x 3 back to main channel & store for next sweep

	com IOMASK
	ld c_tmp,Z
	and c_tmp,IOMASK
NOTESET:
	st Z,c_tmp			; the next timer sweep will post the note on to the latch	
	
L0824:
	ret					; this is the note active return section

OTHERMSG:
	cpi ACC,0x0B 			; control change
	breq PC+2
	rjmp CANIGNORE
	
	; the main controller message we want to deal with is the
	; all notes off message
	; eventualy this will be correct and deal with channels
	; for now just turn everything off

	mov c_tmp,MIDI1
	cpi c_tmp,0x78
	breq ALLOFF
	cpi c_tmp,0x7B
	brne CANIGNORE
	
	CLI
ALLOFF:
	ldi YL,low(EndNoteStateTable)
	ldi YH,high(EndNoteStateTable)
L0718:
	st -Y,zero
	cp YL,zero
	brne L0718
	rcall STARTSEQ


CANIGNORE:				; Cleanup MIDI detection 
	cbi DDRD,DDD2
	rjmp PARSEMIDI

;--------------------------------------------------------------------------------
; Parser to handle system common and system realtime messages
;--------------------------------------------------------------------------------
WIRECMD: 
	; at this point we have detected a one byte message or the SYSEX
	; what is it?  (may want to use a jump table here)

	clr status			; wire commands cancel running status
	cbr FFLAGS,1 << running

	; ignore bad bytes
	mov c_tmp,BCC
	andi c_tmp,0xF0
	cpi c_tmp,0xF0
	breq L0589
	; bad byte, if this is called from a parse when called from a voice message
	; parse we are in trouble
	; check for a high bit set -- do our best to re sync the system
EXITRETRY:
	sbrs BCC,7 
	ret					; ignore bogus data

	tst Needed			; test for in the middle of a message
	brne L1153			; if so wait for a valid byte of data


L1096:
	; retry to sync back to a valid message
	pop ZH				; prevent recursion
	pop ZL
	rjmp RETRYSTATE

L0589:

	mov ARGL,BCC
	andi ARGL,0x0F     	 ; status >> 4 & 0x0F
	lsl ARGL			 ; multiply by two to get table entry

	; use a jump table to set a case tree.	
	ldi ZL,low(SysCommon * 2)
	ldi ZH,high(SysCommon * 2)

    add ZL,ARGL			 ; add the address to the offset
    adc ZH,zero			 	
    lpm 				 ; get the low byte address
	mov c_tmp,r0		 ; save the address for reload
	adiw ZL,01			 ; bump the z pointer
	lpm					 ; get the high address
	mov ZL,c_tmp		 ; restore the low address
	mov ZH,r0			 ; set the high address	
	ijmp				 ; and we are off to see the wizard.

KILLSYSCOM:				 ; de constructor for system common messages
	clr status			 ; system common messages clear the running status
	pop ZH				 ; prevent recursion
	pop ZL
	rjmp CANIGNORE		 ; done with the event wait for the next one

EXITREALTIME:


	;rcall _putchar		 ; RT message should echo

	cp MIDI0,BCC
	breq L0839			 ; skip nekid wire thingys
L1153:
	rcall _getchar		 ; real time message happened in the middle of
						 ; something else
	sbrc BCC,7	
	rjmp L1153 ;EXITRETRY		 ; this will check if something went really wrong


L0839:
	ret
	;cbi DDRD,DDD2		 ; clear early?
	;rjmp PARSEMIDI
						 ; like a command byte in the middle of a message	
;----------------------------------------------------------------------------
; System common messages
;----------------------------------------------------------------------------
SYSEXCOM:	
;
; DHC11 - 68HC11 Disassembler v1.1 (c) Copyright 2000 Tech Edge Pty. Ltd.
;
;		http://www.techedge.com.au/utils/dhc11.htm
;
; this version of the sysex parser is ported from the Octet UMx boards	

;	rjmp KILLSYSCOM
	ldi ARGL,low(KILLSYSCOM)
	ldi ARGH,high(KILLSYSCOM) ; push a return address onto the stack
	push ARGL
	push ARGH				  ; so we do not have to change all the returns to rjmps

	rcall _getchar
	cpi BCC,0x7E			  ; generic midi manufacture
	breq L1165 
	ret
L1165:
	rcall	_getchar
	mov MIDI1,BCC
	cpi BCC,$06					; help sytax
	brne L1173
	rjmp ValidHelp
L1173:
	rcall	_getchar
	mov MIDI2,BCC
	cpi BCC,$06
	brne	LE48C
	rjmp	ValidCom06
;
LE48C:
	mov BCC,MIDI1
	cpi BCC,$20
	breq	LE49D
	sbrc BCC,5
	nop ;rjmp ValidDump
	andi BCC,0b00001111
	cp BCC,MainChnl
	breq LE49D
	ret	
;
LE49D:
	mov BCC,MIDI2
	cpi BCC,$03
	brne	LE4A6
	rjmp	ValidCom03
;
LE4A6:
	cpi BCC,$01
	brne	LE4AD
	rjmp	ValidCom01
;
LE4AD:
	cpi BCC,$02
	brne	LE4B4
	rjmp	ValidCom02
;
LE4B4:
	cpi BCC,$04
	brne	LE4BB
	rjmp	ValidCom04
;
LE4BB:
	cpi BCC,$05
	brne	LE4C2
	rjmp	ValidCom05
;
LE4C2:	ret	

;
.if 0
ValidDump:
	cp BCC,MainChnl
	breq	LE4C8
	ret	
;
LE4C8:
	mov	BCC,MIDI2
	cpi BCC,$02
	breq	LE4D6
	cpi BCC,$03
	brne	LE4D5
	rjmp	LE537
;
LE4D5:	ret
.endif	
;
LE4D6:	
;	rcall	_getchar ; we use a different state machine
	rcall	_getchar
	cpi BCC,$05
	breq	LE4E9
	cpi BCC,$06
	breq	LE521
	cpi BCC,$07
	breq	LE52C
	ret	
;
LE4E9:
	ldi idx,$0010
	ldi ACC,$A8
	ldi XL,low($0110) ; save the data to the note state table for testing
	ldi XH,high($0110)
LE4F2:	
	rcall	_getchar
	mov MIDI2,bcc
	rcall	_getchar
	rol BCC	
	rol BCC	
	mov BCC,MIDI2
	rol BCC	
	push ACC	
	rcall	LE870
	pop ACC	
	st -X,BCC			; shift register is reversed
	sts L0F00,ACC

	;bitB	SPSR  ; send now
	;staB	SPDR
	
	inc ACC		
	dec idx	
	brne	LE4F2

LE513:
	;brclr	SPSR, #%10000000, LE513
	;bset	PortD, #%00100000
	;bclr	PortD, #%00100000
	rcall	_getchar
	ret	
;
LE521:
	ldi idx,$08
	ldi ACC,$88
	ldi XL,low($0110)
	ldi XH,high($0110)
	rjmp	LE4F2
;
LE52C:	
	ldi idx,$08
	ldi ACC,$90
	ldi XL,low($0109)
	ldi XH,high($0109)
	rjmp	LE4F2
;
LE537:	ret	
;

ValidHelp:
	; use the extra flash memory to display a simple "Help" manual on the
	; back channel

	ldi ZL,low(Help*2)
	ldi ZH,high(Help*2)
	ldi idx,4
L1298:
	lpm
	rcall _getchar ;might want to create something called peek char
	cp BCC,r0
	brne L1304
	adiw Z,1
	dec idx
	brne L1298
L1304:
	tst idx
	brne L1319   ; continue with octet stuff
	rcall _getchar
	cpi BCC,$F7
	breq L1313
	ret
	; dump the text Z still points to flash
L1313:
	ldi BCC,$F0
	rcall PutB_Single ; dump it now
L1321:
	WDR
	lpm
	mov BCC,r0 ; need to move it for the test
	cpi BCC,0xF7
	breq L1319
	rcall PutB_Single ; dump it now
	adiw Z,1
	rjmp L1321
L1319:	
	rcall PutB_Single ; dump it now
	ret


ValidCom06:
	; note this is broken becouse we read the first 5 bytes of the message
	clr	c_tmp
	sts L00FA,c_tmp
LE53B:	
	rcall	_getchar
	cpi BCC,$F7
	breq	LE58F
	sts L00FB,BCC
	ldi idx,$07
LE548:	
	ror	BCC ;>L00FB
	sts L00FB,BCC
	brcc	LE564
	lds BCC,L00FA
	andi BCC, 0b00000111
	ldi ZL,low(E7D6*2)
	ldi ZH,high(E7D6*2)
	add ZL,BCC
	adc ZH,zero
	lpm
	mov ACC,r0
	lds BCC,L00FA
	lsr BCC	
	lsr BCC	
	lsr BCC	
	ldi XL,low($0061)
	ldi XH,high($0061)
	add XL,BCC
	adc XH,zero
	ld r0,X
	or ACC,r0
	rjmp LE579
;
LE564:
	lds BCC,L00FA
	andi BCC,0b00000111
	ldi ZL,low(E7D6*2)
	ldi ZH,high(E7D6*2)
	add ZL,BCC
	adc ZH,zero
	lpm
	mov ACC,r0
	lds BCC,L00FA
	lsr BCC	
	lsr BCC	
	lsr BCC	
	ldi XL,low($0061)
	ldi XH,high($0061)
	add XL,BCC
	adc XH,zero
	ld r0,X
	and ACC,r0

LE579:
	st X,ACC
	ldi c_tmp,$A8
	add BCC,c_tmp
	sts L0F00,BCC
	; bitB	SPSR send now
	; bitB	SPDR
	; staA	SPDR
	lds c_tmp,L00FA
	dec idx	
	brne	LE548
	rjmp	LE53B
;
LE58F:
	;bset	PortD, #%00100000
	;bclr	PortD, #%00100000
	ret	
;
ValidCom04:
.if 0
	call	_getchar   ; analysis indicates this is used by vibrato system
	staB	MIDI2
	call	_getchar
	staB	L00FB
	call	_getchar
	rorB	
	ldaB	L00FB
	rolB	
	staB	L00FA
	call	_getchar
	staB	L00FB
	call	_getchar
	rorB	
	ldaB	L00FB
	rolB	
	staB	L00FB
	ldX	L00FA
	ldaB	MIDI2
	cmpB	#$02
	beq	LE5C4
	cmpB	#$03
	beq	LE5E2
.endif
	ret
.if 0	
;
LE5C4	cmpX	#$E000
	bcs	LE5CA
	reti	
;
LE5CA	call	_getchar
	staB	L00FB
	call	_getchar
	rorB	
	ldaB	L00FB
	rolB	
	tBA	
	call	_getchar
	cmpB	#$F7
	beq	LE5DF
	reti	
;
LE5DF	staA	0, X
	reti	
;
LE5E2	call	_getchar
	cmpB	#$F7
	beq	LE5EA
	reti	
;
LE5EA	brclr	L005D, #%00100000, LE5F8
	ldaB	L005D
	andB	#%11011111
	staB	L0F00
	bset	L0040, #%00010000
LE5F8	ldaB	#$F0
	call	putB_2800
	ldaB	#$7E
	call	putB_2800
	ldaB	MainChnl
	oraB	#%00010000
	call	putB_2800
	ldaB	#$04
	call	putB_2800
	ldaB	#$03
	call	putB_2800
	ldaB	0, X
	tBA	
	lsrB	
	call	putB_2800
	tAB	
	andB	#%00000001
	call	putB_2800
	ldaB	#$F7
	call	putB_2800
	reti
.endif	
;
ValidCom05:				; not supported function
	rcall	_getchar
	cpi BCC,$01
	breq	LE62E
	ret	
;
LE62E:	ret	
;
ValidCom01:				; main quiery command
	rcall	_getchar
	cpi BCC,$01
	breq	LE637
	ret	
;
LE637:
	rcall	_getchar
	cpi BCC,$F7
	breq	LE63F
	ret	
;
LE63F:
;	sbrs L005D,5
	rjmp LE64D

;	mov BCC,L005D			; enable midi through
	andi BCC,0b11011111
	sts L0F00,BCC
	sbr	L0040,1<<4; #%00010000
LE64D:	
	ldi BCC,$F0
	rcall	putB_2800
	ldi BCC,$7E
	rcall	putB_2800
	mov BCC,MainChnl
	ori BCC,0b00010000
	rcall	putB_2800
	ldi BCC,$01
	rcall	putB_2800
	ldi BCC,$02
	rcall	putB_2800
	ldi XL,low(E002)	;serial number
	ldi XH,high(E002)
LE66B:
	ld BCC,X+
	rcall	putB_2800
	;inc X
	cpi XL,low(E002+10)
	ldi c_tmp,high(E002+10)
	cpc XH,c_tmp	
	brne	LE66B
	
	lds BCC,L00FF
	mov ACC,BCC
	lsr BCC	
	rcall	putB_2800
	mov BCC,ACC
	andi BCC,0b00000001
	rcall	putB_2800
	mov BCC,MainChnl
	rcall	putB_2800
	lds BCC,SubChnl
	rcall	putB_2800
	ldi BCC,$F7
	rcall	putB_2800
	ret	
;
ValidCom02:
	; This function is the heart of the system, it returns all the set
	; parameters.  The setup program calls this often whenever the
	; data may have changed
	rcall	_getchar
	cpi BCC,$01
	breq	LE69B
	tst bcc
	brne L1564
	rjmp LE4D6
L1564:
	ret	
;
LE69B:
	ldi XL,low(E002)	;serial number
	ldi XH,high(E002)
LE69E:	
	rcall	_getchar
	ld r0,X+
	cp BCC,r0
	breq	LE6A6
	ret	
;
LE6A6:
	cpi XL,low(E002+10)
	ldi c_tmp,high(E002+10)
	cpc XH,c_tmp	
	brne	LE69E
	rcall	_getchar
	cpi BCC,$F7
	breq	LE6B4
	ret	
;
LE6B4:
	ldi BCC,$F0
	rcall	putB_2800
	ldi BCC,$7E
	rcall	putB_2800
	mov BCC,MainChnl
	ori BCC,0b00010000
	rcall	putB_2800
	ldi BCC,$01
	rcall	putB_2800
	ldi BCC,$02
	rcall	putB_2800
;	sbrs L005D,5 ;#%00100000
	rjmp LE6DD
;	andi L005D,0b11011111
;	sts L0F00,L005D
	sbr	L0040,1<<4;  #%00010000
LE6DD:
	ldi XL,low(E000)	; the whole world
	ldi XH,high(E000)
LE6E0:
	ld BCC,X+
	andi BCC,0b01111111
	rcall	putB_2800
	cpi XL,low(E000+$A9)
	ldi c_tmp,high(E000+$A9)
	cpc XH,c_tmp	
	brne	LE6E0
	ldi BCC,$FF
	mov ACC,BCC
	lsr BCC	
	rcall	putB_2800
	mov BCC,ACC
	andi BCC,0b00000001
	rcall	putB_2800
	ldi BCC,$00		; this must be extention from UM1
	andi BCC,0b00001111
	rcall	putB_2800
	ldi BCC,$F7
	rcall	putB_2800
	ret
;
ValidCom03:
	; these for the most part are the update commands.  Most of the reason for
	; porting this code to avr is to enable the mapping tables to be written
	; using the octet setup software
	rcall	_getchar
	cpi BCC,$01
	breq	LE70F
	ret	

LE70F:
	ldi XL,low(E002)	;serial number
	ldi XH,high(E002)
LE712:	
	rcall	_getchar
	ld r0,X+
	cp BCC,r0
	breq	LE71A
	ret	
;
LE71A:
	cpi XL,low(E002+10)
	ldi c_tmp,high(E002+10)
	cpc XH,c_tmp	
	brne	LE712

	ldi XL,low(E0E0)	;string
	ldi XH,high(E0E0)
LE723:
	ld BCC,X
	tst BCC
	breq	LE732
	rcall	_getchar
	ld c_tmp,X
	cp BCC,c_tmp
	breq	LE72F
	ret	
;
LE72F:
	adiw XL,1	
	rjmp LE723
;
LE732:	
	rcall	_getchar
	cpi BCC,$F7
	breq LE73A
	ret	
;
LE73A:
;	sbrs L005D,5
	rjmp LE748
;	mov BCC,L005D
	andi BCC,0b11011111
	sts L0F00,BCC
	sbr	L0040,4;, #%00010000
LE748:
	;di	
	;ldS	#$00F9
	rjmp	LFA02
;
; alternate get character
;
_getchar:
	cp iRH,iRT			; poll for next character
	breq  _getchar      ; if the watchdog is enabled 
						; the timeout will reset the system		

	WDR					; wake the dog
	push XL				; will wrap					
	push XH
	mov	 XL,iRT		; read the saved byte
	ldi	 XH,00
	subi XL,low((~iRingHead)+1) 
	sbci XH,high((~iRingHead)+1)
	ld   BCC,X			; use b register for hc11 porting compatibility
	pop XH
	pop XL
	inc	  iRT			; ++ ; buffer fits into a 256 byte ring	
						; echo byte on MIDI Through
	sbrc L0040,4 ; 0b00010000
	rjmp LE761
;	sbrc L005D,5 ; 0b00100000
	ret
LE761:
	rcall putB_2800
	rcall	_putchar	; implies call to putB_2800
	ret


.if 0
;
On_ICAP:
	bset	TFLG2, #%01000000
	brclr	L0040, #%00000001, LE78F
	ldD	L00B3
	subD	#$0001
	beq	LE785
	stD	L00B3
	jr	LE78F
;
LE785	ldD	L00B1
	stD	L00B3
	ldaA	L0000
	xorA	#%01000000
	staA	L0000
LE78F	brclr	L0041, #%00000001, LE79B
	dec	>L009D
	bne	LE79B
	jmp	On_Reset
;
LE79B	brset	L0040, #%00010000, LE7B2
	ldaA	L00B5
	beq	LE7B2
	decA	
	staA	L00B5
	cmpA	#$26
	bne	LE7B2
	bclr	L005D, #%01000000
	ldaA	L005D
	staA	L0F00
LE7B2	reti	
;
On_Timer:
	bset	TFLG, #%10000000
	ldD	TOC1
	addD	#$0BB8
	stD	TOC1
	inc	>L009A
	bne	LE7C5
	inc	>L0099
LE7C5	reti	
;
.endif
;
Mask00:
	.db	$01, $02, $04, $08, $10, $20, $40, $80
Mask01:
	.db	$FE, $FD, $FB, $F7, $EF, $DF, $BF, $7F
Mask02:
	.db	$10, $20, $40, $80, $01, $02, $04, $08
Mask03:
E7D6:
	.db	$EF, $DF, $BF, $7F, $FE, $FD, $FB, $F7

;
LE7E6:
	ret
.if 0
		tBA	
andB	#%00000111
	ldX	#$E7D6
	aBX	
	ldaB	0, X
	pushB	
	tAB	
	lsrB	
	lsrB	
	lsrB	
	pushB	
	ldaB	MIDI0
	andB	#%00001111
	clrA	
	xgDX	
	brclr	66, X, #%00000001, LE811
	xgDX	
	ldaA	#$10
	mul	
	addD	#$2140
	xgDX	
	popB	
	popA	
	pushA	
	aBX	
	oraA	0, X
	staA	0, X
	jr	LE812
;
LE811	popB	
LE812	popA	
	ldX	#$0061
	aBX	
	oraA	0, X
	staA	0, X
	jr	setSerialD
;
.endif
LE81D:
	ret
.if 0
	tBA	
	andB	#%00000111
	ldX	#$E7DE
	aBX	
	ldaB	0, X
	pushB	
	tAB	
	lsrB	
	lsrB	
	lsrB	
	pushB	
	ldaB	MIDI0
	andB	#%00001111
	clrA	
	xgDX	
	brclr	66, X, #%00000001, LE846
	xgDX	
	ldaA	#$10
	mul	
	addD	#$2140
	xgDX	
	popB	
	popA	
	aBX	
	andA	0, X
	staA	0, X
	ret	
;
LE846	popB	
	popA	
	ldX	#$0061
	aBX	
	andA	0, X
	staA	0, X
.endif
setSerialD:
	push	BCC	
;	andi	L005D,~0B10111111 ; 68hc11 is bclr
	ldi c_tmp,$88
	add BCC,c_tmp
;	or BCC,L005D
	sts L0F00,BCC
	ldi ACC,$FF	; spit out a restart message (This may need to be held on
	pop BCC		; power up
	;sts UDR0,ACC
	ret

.if 0
	bitB	SPSR
	bitB	SPDR
	staA	SPDR
	nop	
	nop	
	nop	
	nop	
	nop	
	nop	
	nop	
	bset	PortD, #%00100000
	bclr	PortD, #%00100000
	popB	
	ret

.endif
LE870:	
	mov ACC,BCC ;tBA	
	rol ACC	
	rol BCC	
	rol ACC	
	rol BCC	
	rol ACC	
	rol BCC	
	rol ACC	
	rol BCC	
	ret
.if 0	
;
func08:
	pushB	
	tBA	
	rorA	
	rorA	
	rorA	
	rorA	
	rorA	
	rolB	
	rorA	
	rolB	
	rorA	
	rolB	
	rorA	
	rolB	
	popA	
	rorA	
	rolB	
	rorA	
	rolB	
	rorA	
	rolB	
	rorA	
	rolB	
	ret	
;
func07:
	pushB	
	tBA	
	rolA	
	rolA	
	rolA	
	staA	MIDI1
	rolA	
	rolA	
	rolA	
	rorB	
	rolA	
	rorB	
	rolA	
	rorB	
	popA	
	rolA	
	rorB	
	rolA	
	rorB	
	rolA	
	rorB	
	rolA	
	rorB	
	lsrB	
	ldaA	MIDI1
	andA	#%01000000
	ret	
;
LE8B0:	ldaA	#$10
	mul	
	addD	#$2140
	xgDY	
	clrB	
	ldX	#$0061
LE8BC	ldaA	0, X
	andA	0, Y
	staA	0, X
	bset	0, Y, #%11111111
	call	setSerialD
	incB	
	incX	
	incY	
	cmpB	#$10
	bne	LE8BC
	ret	
;
func09:
	ldX	#$002E
LE8D6	brset	TCNT, #%10000000, LE8D6
LE8DA	brclr	TCNT, #%10000000, LE8DA
	decX	
	bne	LE8D6
	ret	
;
func10:
	ldX	#$E024
	ldaB	0, X
	oraB	#%00010000
	ldY	LE917
	pushY	
	ldY	LE915
	pushY	
	ldY	JumpVector3
	pushY	
	ldY	JumpVector2
	pushY	
	tSY	
	di	
	call	0, Y
	ei	
	xgDY	
	addB	#$08
	xgDY	
	tYS	
	ret	
;
;
JumpVector2:
	dw	jmpVec0_00
JumpVector3:
	dw	On_Reset
LE915:	dw	$26FC
LE917:	db	$39
;
.endif

putB_2800:
	push BCC
	push XL
	push XH	
	; de refrence ring out
	
	mov	 XL,oRH			; deref outRing[r18++]
	ldi	 XH,00
	subi XL,low((~oRingHead)+1) 
	sbci XH,high((~oRingHead)+1)
	st X,BCC
	pop XH
	pop XL
	inc oRH
	;stX	Ring1Out
	pop BCC	
	ret	

.if 0
;
putB_3000:
	pushX	
	pushB	
	ldX	Ring1Out
	decX	
	cmpX	#$27FF
	bne	LE939
	ldX	#$2FFF
LE939	stX	Ring1Out
	staB	0, X
	popB	
	popX	
	ret	
;
putB_27FF:
	pushX	
	pushB	
	ldX	Ring2In
	staB	0, X
	incX	
	cmpX	#$3200
	bne	LE94F
	ldX	#$3000
LE94F	stX	Ring2In
	popB	
	popX	
	ret	
;
.endif


_putchar:

	cp	 oRH,oRT   ; test outRing busy
	brne LE9AA	   ; queue the character for transmission
	
	ret	; not sure why fancy stuff was done
;
LE9AA:
	push BCC
	push XL
	push XH	
	; de refrence ring out
	
	mov	 XL,oRT			; deref outRing[r18++]
	ldi	 XH,00
	subi XL,low((~oRingHead)+1) 
	sbci XH,high((~oRingHead)+1)
	ld BCC, X
	pop XH
	pop XL
	
	; any fancy stuff should be done here (like the testing for SYSEX start.end)
	;  probably should not echo real time messages
	sbrs BCC,7
	rjmp L2205	; echo normal messages
	mov c_tmp, BCC
	andi c_tmp,$F0
	breq L2205 ; echo wire messages
	sbrc BCC,3 ; echo sys common
	rjmp L2112 ; do not echo system realtime
	; output the next byte in the ring
L2205:
	lds c_tmp,UCSR0A
	sbrs c_tmp,UDRE0  ; poll for next output byte to send
	rjmp L2205
	sts UDR0,BCC
L2112:
	pop BCC	
	inc oRT
	ret	


LFA02:	; rjmp	LFC98
	; original rom has duplicate bootloader here

;LFC98: 
WriteWorld:
	; this is a write world system should we choose to disable interrupts
	; then first tri state the playing notes (although sending setup
	; commands at the same time as midi events could cause problems
;	di
	sbi PORTD,PortD4	; disable register
	CLI  ; exits through a reboot	
	clr ACC	
;LFC9A:
AckWorldState:
	WDR
	ldi BCC,$F0			; first send the Ack
	rcall	PutB_Single		
	ldi BCC,$7E
	rcall	PutB_Single
	mov	BCC, MainChnl
	ori BCC,0b00010000
	rcall	PutB_Single
	ldi BCC,$70
	rcall	PutB_Single
	rcall	PutA_Single  ; output ACC register
	ldi BCC,$F7
	rcall	PutB_Single
	tst ACC	
	brne	LFCC1		; start state machine

SixWorldState:
	ldi ACC, $06
	sts L00FC,ACC

	; init program memory state machine
LFCC1:	
	; this is the bootloader  - init a few "safe values"
	ldi	r16,low(E000-1)  ; setup stack for bootloader params
	out	SPL,r16
	ldi	r16,high(E000-1) ; 
	out	SPH,r16
	lds idx,L00FC		 
	dec idx ;	>L00FC	 ; decrement state by one
	sts L00FC,idx
	brne	NextBlock
NewWorld:
	; reboot the board with the new parameters
	rjmp	0 ; execute the new code
;

NextBlock:
	WDR
	rcall	GetB_Single	; read the incoming program data
	;bitB	#%10000000
	sbrs BCC,7
	rjmp	NextBlock	; bit 7 is clear ignore extra parameter bytes
	cpi BCC,$FF
	breq	NewWorld ; reset and reboot -- abord message
	cpi BCC,$F7
	breq	NextBlock	; early termimation of sysex ignore till we have valid 
					; program load data
	cpi BCC,$F0
	brne	fourthWorld	; set to state 4
	rcall	GetB_CKSMA 	; get next parameter byte
	cpi BCC,$7E		; is our experimental sysex
	brne	fourthWorld	; set to state 4
	rcall	GetB_CKSMA 	; next byte and compare against main channel
	andi BCC,0b00001111
	cp BCC,MainChnl
	brne	fourthWorld	; set to state 4
	rcall	GetB_CKSMA	; next byte is a save parameter byte
	cpi BCC,$03
	breq	parseAddr   ; paramiter prefix is ours

fourthWorld:
	ldi ACC,$04
	sts L00FC,ACC
	rjmp	AckWorldState	; send ack and wait for next byte
;

parseAddr:	
	rcall	GetB_CKSMA 	; command byte
	push BCC	
	clr ACC	
	rcall	GetB_CKSMA	; high address top 7
	push BCC	
	rcall	GetB_CKSMA	; low address bit 9
	ror BCC			; rotate low bit into cary
	pop BCC			; get high address
	rol BCC			; rotate together
	sts	L00FA,BCC	; save address into address counter
	rcall	GetB_CKSMA   ; low address top 7
	push BCC	
	rcall	GetB_CKSMA	; low address bit 1
	ror BCC	
	pop BCC	
	rol BCC	
	sts L00FB,BCC	; save low address into counter
	
	lds XH,L00FA	; de reference the encoded address
	lds XL,L00FB
		  
	sbrs L0040,2
	rjmp  LFD2E ;brclr	;0b00000100  ;

	ldi BCC,$0B		;  
	;xgDX	
	;bitA	#%00000001
	;xgDX	
	sbrs XH,1 ;breq	LFD2E
	ori BCC,0b00000100 ; address is odd?

LFD2E:	
	rcall	GetB_CKSMA	; next byte
	sts L00FB,BCC	; count
	clr c_tmp
	sts L00FA,c_tmp ; clear address marker

	lds ZH,L00FA	; Z seems to be a count
	lds ZL,L00FB

	pop BCC			; command byte has been on the stack for sometime
	cpi BCC,$02
	breq	WriteMemParm	; write E000 world
	cpi BCC,$03
	breq	LFD4D	; firmware upgrade in
	cpi BCC,$04
	breq	LFD8F 	; end of sequence
	ldi ACC,$04
	sts L00FC,ACC
	rjmp	AckWorldState	; set state to 4 and wait for next parameter set
;
LFD4D:	
	rcall	GetB_CKSMA	; seems to be firmware upgrade in
	cpi BCC,$F7
	breq	LFD59
	ldi ACC,$03
	rjmp	AckWorldState	; set state to 3
;
LFD59:				
	ldi BCC,$F0				; ack firmware upgrade
	rcall	PutB_Single
	ldi BCC,$7E
	rcall	PutB_Single
	mov BCC,MainChnl
	ori BCC,0b00010000
	rcall	PutB_Single
	ldi BCC,$03
	rcall	PutB_Single
	ldi BCC,$03
	rcall	PutB_Single

LFD75:
	ld BCC,X				; x points to code to send back
	mov ACC,BCC				
	lsr BCC	
	rcall	PutB_Single		; high bits of memory to read
	mov BCC,ACC
	andi BCC,0b00000001
	rcall	PutB_Single		; low bit of memory to read
	adiw X,1	
	sbiw Z,1				; next byte 
	brne	LFD75

	ldi BCC,$F7
	rcall	PutB_Single		; end of sequence
	rjmp	SixWorldState	; restart at state 6
;
LFD8F:
	rcall	GetB_CKSMA
	cpi BCC,$F7
	breq	LFD9B			; restart system when valid
	ldi ACC,$03				; state machine 
	rjmp	AckWorldState	; handle next state
;
LFD9B:
	rjmp	0;, X		; reboot
;
WriteMemParm:
	cpi XH,high($E000) ; only writes to parameter space are supported
	brne FlushData
	mov ZL,XL	; paramter memory space is byte aligned at 0x200
	ldi ZH,high(E000)    ; notice this loads from the label
	; the consumer setup program seems to start the parameter write
	; just after the firmware revision at the parameter soft mode 0xE020
	lds XH,L00FA	; load count into x
	lds XL,L00FB

LFDA4:
	WDR
	rcall	GetB_CKSMA		; read parameter hi bits
	push BCC	
	rcall	GetB_CKSMA		; read parameter low bit
	ror BCC	
	pop BCC	
	rol BCC					; merge parameter
	st Z+,BCC				; save the new parameters in memory			
	sbiw X,1	
	brne	LFDA4

	rcall	GetB_CKSMA		; read one byte beyond the count
	sbrs BCC,7				; test for high bit set
	rjmp LFDC1
	ldi ACC,$01				; set the state machine to 1
	rjmp	AckWorldState	; handle next state
;
LFDC1:
	rcall	GetB_CKSMA		; test for end of parameter sequence
	cpi BCC,$F7
	brne	LFDCB
	rjmp	WriteWorld		; start over
;
LFDCB:	
	ldi ACC,$03				; set the state machine to 3
	rjmp	AckWorldState	; handle next state


FlushData:
; old code for reference
.if 0

;

LFDD0:
;	stY	>L00FA
	sts L00FA,ZL
	sts L00FB,ZH
;	xgDX	
;	addD	>L00FA
	add XL,ZL
	adc XH,ZH
;	xgDX	
;	ldY	#$FE3D
	ldi ZL,low($FE3D)	; fixed point in 68HC11 space
	ldi ZH,High($FE3D)

	; push this code onto the stack 
LFDDD:
	ld BCC,Z			; does not really work
	push BCC			; this will overflow the stack	
	sbiw Z,1
	ldi ARGL,low($FE2B)	; fixed point in 68HC11 space
	ldi ARGH,High($FE2B)
	;cmpY	#$FE2B
	cp ZL,ARGL
	cpc ZH,ARGH
	brne	LFDDD


	; set the stack to the code pushed
	;tSY
	out SPL,ZL			; really bad as the stack is void
	out SPH,ZH
	;xgDY	
	;staB	>L00FA
	sts L00FA,ZL
	lds BCC,L00FB
	clr ACC	
	; xgDY
	mov ZH,ACC
	mov ZL,BCC
	
	; push downloaded code onto stack	
LFDF6:
	rcall	GetB_CKSMA
	push BCC	
	rcall	GetB_CKSMA
	ror BCC	
	pop BCC	
	rol BCC	
	push BCC				; get parameter	and push onto stack
	sbiw ZL,1
	brne	LFDF6

	rcall	GetB_CKSMA		; test for additional command sequence
	;bitA	#%01111111 beq	LFE11
	sbrc ACC,7
	rjmp LFE11
	ldi ACC,$01
	rjmp	AckWorldState	; handle next state

LFE11:
	rcall	GetB_CKSMA
	cpi BCC,$F7
	breq	LFE1D
	ldi ACC,$03
	rjmp	AckWorldState	; set state to 3 and test
;
LFE1D:
	; valid exit, run bootloader code from sram
	lds BCC,L00FA
	clr ZH	
	; xgDY	
	lds ZL,L00FB
com08:
	ijmp					; to the bootloader
;

com09:				; FE2C
	rjmp	WriteWorld	;  this is may be the firmware bootloader
;
com09A:
	decX	
	popB	
	cmpB	0, X
	beq	LFE38
	staB	0, X
LFE34	cmpB	0, X
	bne	LFE34
LFE38	decA	
	bne	com09A
	jmp	com09		;
.endif

GetB_Single:
LFE3E: 
;	brclr	SCSR, #%00100000, GetB_Single
	lds c_tmp,UCSR0A
	sbrs c_tmp,RXC0  ; poll for next output byte to recieve
	rjmp LFE3E
	lds BCC,UDR0
	ret	
;
PutB_Single:
LFE45: 
;	brclr	SCSR, #%10000000, LFE45
	lds c_tmp,UCSR0A
	sbrs c_tmp,UDRE0  ; poll for next output byte to send
	rjmp LFE45
	sts UDR0,BCC
	ret	
;
PutA_Single:
LFE4C:
;	brclr	SCSR, #%10000000, LFE4C
	lds c_tmp,UCSR0A
	sbrs c_tmp,UDRE0  ; poll for next output byte to send
	rjmp LFE4C
	sts UDR0,ACC
	ret	
;
LFE53: ; 
GetB_CKSMA:
WDR ; wait forever
;	ldaB	TCNT
;	addB	#$F0

LFE57:
;	brset	SCSR, #%00100000, LFE64
	lds c_tmp,UCSR0A
	sbrs c_tmp,RXC0  ; poll for next outUCSR0A 7 160put byte to send
	rjmp LFE57

;	cmpB	TCNT
;	bne	LFE57
;	ldaA	#$02
;	jmp	AckWorldState		; 
;
LFE64:
	lds BCC,UDR0
	add ACC,BCC ; checksum	
	ret	

;============================================================================================
; dynamic MIDI messages


QTRFRMCOM:
	rcall _getchar 
	; check for RT message  (can recurse?)
	rjmp KILLSYSCOM

SNGPOSPTRCOM:
	rcall _getchar
	; error check
	rcall _getchar
	; error check
	  
	rjmp KILLSYSCOM

SONGSELCOM:	
	rcall _getchar
	; error check
	rjmp KILLSYSCOM

UNDEF4COM:
UNDEF5COM:
TUNEREQCOM:
	rjmp KILLSYSCOM

EOXCOM:
	sbrc FFLAGS,sysxcntu 
	ret						; let the caller function handle the sysex
	rjmp KILLSYSCOM
							; the EOX used to divide the sysex into pages.

; System real time messages
CLOCKRT:
	; MIDI clock message 12 in a quarter note
	rjmp EXITREALTIME

TICKRT:
	; Sent one every 10 milliseconds by some playback record devices
	rjmp EXITREALTIME

STARTRT:
	; useful if we were a sequencer
	rjmp EXITREALTIME

CONTINUERT:
	; not really usefull
	rjmp EXITREALTIME

STOPRT:
	; nothing to stop
	rjmp EXITREALTIME

UNDEFDRT:

	rjmp EXITREALTIME

ACTSENSERT:
	; set up active sense timer using the watchdog at 500ms

;	mov ACC,ARGL
	sbrc FFLAGS,playactive
	rjmp EXITREALTIME
	sbr FFLAGS,1 << playactive
	ldi ARGH,(1<<WDE) | (1<<WDP2) | (1 << WDP0) ; closer to 500ms
	rcall L1113
;	mov ARGL,ACC
	rjmp EXITREALTIME

L1113:
	WDR
	CLI	; bad, but what can we do , on must be atomic
	lds ARGL,WDTCSR					; start timed sequence
	ori ARGL,(1<<WDCE) | (1<<WDE)
	sts WDTCSR,ARGL					; use sts to get 4 cycles
	mov ARGL,ARGH
	sts WDTCSR,ARGL
	SEI ; end atomic sequence
	ret

SYSRESETRT:
	sbi PORTD,PortD4	; Disable shift register
	cbi DDRD,DDD3		; blank IO

	ldi ARGL,0xFF		; echo reset 
	rcall _putchar

L0787:					; wait for completion
	cp r4,r5
	brne L0787

	lds ACC,UCSR0B		; 	make sure we are not xmitting
	cbr	ACC,1<<UDRIE0	;   if we are tranmitting someone is trying
	sts	UCSR0B,ACC		;   to violate physics	

	ldi ARGH,(1<<WDE)	;   minimum time

	rcall L1113			;   enable the dog


;	rjmp 0 ; should let the dog bite
	; kick the dog
reset: rjmp reset			; let the dog bite  if enabled.

chantype:
.db 0, 0, 0, 0, 0, 0, 0, 0, 2, 2, 2, 2, 1, 1, 2, 0 
wirerealtime:
;      0 1 2 3 4 5 6 7 8 9 A B C D E F
.db 0xFF,1,2,1,0,0,0,0,0,0,0,0,0,0,0,0   

SysCommon:
;system common/real time jump table
.db byte1(SYSEXCOM),byte2(SYSEXCOM),byte1(QTRFRMCOM),byte2(QTRFRMCOM),byte1(SNGPOSPTRCOM),byte2(SNGPOSPTRCOM),byte1(SONGSELCOM),byte2(SONGSELCOM)
.db byte1(UNDEF4COM),byte2(UNDEF4COM),byte1(UNDEF5COM),byte2(UNDEF5COM),byte1(TUNEREQCOM),byte2(TUNEREQCOM),byte1(EOXCOM),byte2(EOXCOM)

; System real time messages
.db byte1(CLOCKRT),byte2(CLOCKRT),byte1(TICKRT),byte2(TICKRT),byte1(STARTRT),byte2(STARTRT),byte1(CONTINUERT),byte2(CONTINUERT)
.db byte1(STOPRT),byte2(STOPRT),byte1(UNDEFDRT),byte2(UNDEFDRT),byte1(ACTSENSERT),byte2(ACTSENSERT),byte1(SYSRESETRT),byte2(SYSRESETRT)


defaultNoteMapMask:
#ifdef HWNOTES128
.db 128,64,32,16,8,4,2,1	; single chip addresses
#endif
;.db 128,128,64,64,32,32,16,16,8,8,4,4,2,2,1,1	; dual chip addresses
;.db 1,2,4,8,16,32,64,128	; even chip addresses
#ifdef HWNOTES48
.db 128,1,64,2,32,4,16,8,8,16,4,32,2,64,1,128 ; even odd interleve
#endif
;.db 1,128,2,64,4,32,8,16,16,8,32,4,64,2,128,1 ; even odd interleve
;   1  2  3 4  5 6  7  8 9  a b  c d  e f   10


Help: .db "HELP"

	.db "MIDI Primary Octet UMx compatable decoder board",'\n'
	.db "This file is sent as a SYSEX backchannel dump",'\n'
	.db " Switch options are 1 through 4 set Midi Channel ",'\n'
	.db " Switch 5 sets Octet compatable program mode ",'\n'
	.db "   On -- enable EEprom writes",'\n'
	.db "   Off -- normal run mode ",'\n','\n'
	.db " Switch 6 sets output pin polarity ",'\n'
	.db "   On -- outputs are Active low",'\n'
	.db "   Off -- outputs are Active high",'\n','\n','\n'
	.db "Warning: this board is only a primary driver and outputs are limited ",'\n'
	.db " to TTL current ratings at 5 volts\nadditional current drivers are needed",'\n'
	.db " to switch magnets.",'\n'

;This board supports the following sysex commands

	; fmt F0 7E chnl  byte1 byte2 <parameters> 0xF7
	;           0x20  1     1     -- system inquiry returns serial #
	;           0x1k  1     2     -- dump parameters request
	;           0x1k  2     0     -- dump data shift register to host
	;           0x1k  2     1     -- dump configuration
	;           0x1k  3     0     -- dump request from host
	;           0x1k  3     1     -- serial# UM1 ID dump request
	;			0x1k  3     2     -- store eeprom parameter data 
	;			0x1k  3     3     -- firmware update 
	;           0x1k  70    0     -- eeprom write ack
	;			0x1k  4     2     -- vibrato test
	;           0x1k  2     4     -- vibrato resp
;
;

	.db 0x20,0xF7

