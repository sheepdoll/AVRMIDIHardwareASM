; Merged AVR MIDI88 Driver  -- added section to
; allow 44 or 128 note versions

.include "m88def.inc"
; define some compiler flags
#define HWNOTES48
;#def HWNOTES128


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



; System Process equates
;----------------------------------------------------------------------
.equ NOOFTASKS 		= 1  	; tasks are numbered 0 - 2  
.equ STACKSIZE 		= 16 	; hope there is room
							; 

.equ kBaud31250		= 15	; this sets the MIDI baud rate to 31250

.equ ProcMainID		= 0		; Task ID of the main process


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


.def zero		= r1	; keep a zero handy
.def const7		= r2	; temporary shift register (cause 88 breaks things)
.def nibbleMask	= r3	; should always be 0x0F
.def oRH		= r4	; mask for port C reads and writes
.def oRT		= r5	; out ring tail
.def iRH		= r6	; in ring head
.def iRT		= r7	; in ring tail
.def ashift		= r8	; extended wave generation
.def bshift		= r9	; extended wave generation

.def MIDI0		= r10
.def MIDI1		= r11
.def MIDI2		= r12
.def rfu13		= r13	; set to 0x3F constant for in ring mask 
.def status		= r14	; saved status byte for run
.def ssreg    	= r15

.def temp1		= R16	; used in RX interrupt
.def temp2		= R17	; used in RX interrupt
.def FFLAGS		= R18	;
.def IOMask		= R19	; used in midi parser
.def idx		= R20	; temporary register
.def c_tmp     	= r21 	; passed arguments
.def ARGL     	= r22 	; passed arguments
.def ARGH     	= r23 	; to OS, usually a pointer
.def ACC      	= r24 	; Old fashioned double wideaccumulator
.def BCC      	= r25 	;  

; TASKSTATE semephors and task no 76543210
;                                 KSSSIITT
.equ keywaiting		= 7	; a keypress occured		
.equ task2Semaphore	= 6 ; to be used by
.equ task1Semaphore	= 5 ; file load and
.equ task0Semaphore	= 4 ; playback
; bits 3 and 2 are task ID when timer was interrupted
; bits 0 and 1 are task ID of current process

;==============================================================================
;               
;                     D A T A   S E G M E N T 
;
;==============================================================================
.equ MSGQ_SIZE = 64

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

P0328:				; calculated start of inRing
	.byte	2
M032A:				; inRing command text
	.byte	63
D0369:				; inRing in index command count
	.byte	1
D036A:				; inRing out calculated start
	.byte	1		
M036B:				; 
	.byte	1
M036C:				; outRing
	.byte	15
D037B:				; outRing in index  reply count
	.byte	1
D037C:				; outRing out index reply sent count
	.byte	1		

SYSXBFR:
	.byte 255		; most of our sram is for sysexes
; interrupt table
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
	rjmp	L1409
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
	sbic PinC,5
	rjmp L0265
	cbi PORTB,Portb0  ; just clear the data clock
	reti
L0265: 
	sbi PORTB,Portb0  ; just clear the data clock
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

	sts	D036A,r17		; in ring in
	sts	D0369,r17		; in ring out
	sts	D037C,r17		; out ring in
	sts	D037B,r17		; out ring out
	ret
; pc=0x13EA(0x27D4)
;

; RX handler, optimized for speed - this dedicates the X register 
; to the handler

L13EA:					; Receive RX handler
	in	ssreg,SREG		; protect status
	WDR					; keep from resetting in active mode
	lds r16,UDR0
	inc	 r6
	and r6,r13		    ; mask ring to size r13 is a constant
	mov	 XL,r6
	ldi	 XH,00
	subi XL,low(0xFFFF-P0328) ;kD7	; 329 is the real start
	sbci XH,high(0xFFFF-P0328);kFC
	st	X,r16		; save input byte into ring
	out	SREG,ssreg		; restore state
	reti

;

L1409:					; TX complete handler
	in	 ssreg,SREG		; save status state
	WDR					; message may be important keep alive
	cp	 r4,r5			; when counters equal
	breq L1422			; ring is empty
	sbi DDRD,DDD3
	inc	 r5				; cycle the ring
	;and  r5,r13		; large sysex data dumps can happen

	mov	 XL,r5			; de reference the array index
	ldi	 XH,00
	subi XL,low(0xFFFF-D036A) ; k95 +36B is the real start 
	sbci XH,high(0xFFFF-D036A); kFC
	ld	 r16,X			; get data from ring
	sts UDR0,r16
L1422:
	cbi DDRD,DDD3		; display indicator on LED
	reti	
	
					; keep the system quiet when no data to echo
;	lds r16,UCSR0B		; slight difference this touches r16
;	cbr	r16,1<<UDRIE0	; which will be voided on return
;	sts	UCSR0B,r16
	
L0322:	
;	reti

; pc=0x142A(0x2854)
;
;*************************************
; access input ring

;*************************************
;
_getchar:        
	cp r6,r7
	breq  _getchar      		;  L142A Ring is empty

	WDR
	inc	  r7					; ++	
	and   r7,r13
	push ZL						; a method to protect Z during SYSEX messages
	push ZH
	mov	 ZL,r7
	ldi	 ZH,00
	subi ZL,low(0xFFFF-P0328) 	; kD7	; 329 is the real start
	sbci ZH,high(0xFFFF-P0328)	; kFC
	ld   ARGL,Z					; r16 = inRing[r16]
	pop ZH
	pop ZL
	ret




; pc=0x143C(0x2878)
;
_putchar:				; L143C  transmit byte
	inc	 r4
;	and  r4,r13			; large data dumps can happen

L1440:
	cp	 r4,r5			; test outRing busy
	breq L1440			; ring empty

	WDR
	mov	 ZL,r4			; deref outRing[r18++]
	ldi	 ZH,00
	subi ZL,low(0xFFFF-D036A)	; k95 +36A 
	sbci ZH,high(0xFFFF-D036A); kFC
	std	 Z+00,ARGL		; save byte into ring

	lds c_tmp,UCSR0B		; slight difference this touches r16
	sbr	c_tmp,1<<UDRIE0	; which will be voided on return
	sts	UCSR0B,c_tmp
	ret




;*************************************
; MEP mph MLH Main reset entry point
;*************************************
Main:

; init serial ports for MIDI baud rate
	ldi	r16,low(RAMEND) ; setup stack
	out	SPL,r16
	ldi	r16,high(RAMEND) 
	out	SPH,r16

;   did the dog bite?  turn it off
	CLI
	WDR
	in ARGL,MCUSR
	andi ARGL,(0xFF & (0<<WDRF))
	out MCUSR,ARGL

	; we should run with the dog enabled

	ldi ARGH,(1<<WDE) | (1 << WDP0) ; really long time
	rcall L1113						; kick the dog




	clr zero
	clr FFLAGS

	clr r4			; clear the out ring pointers
	clr r5			; 
	clr r6			; clear the in ring pointers
	clr r7


	ldi YL,low(BITStateTable+0xFF)
	ldi YH,high(BITStateTable+0xFF)
L0413:
	st -Y,zero
	cp YL,zero
	brne L0413

	ldi	r16,kBaud31250	; kBaud 15 for midi clock at 8 Mhz	
	rcall	L13DC		; init uart


	ldi ACC,0x3F		; mask for input and output rings
	mov r13,ACC
	ldi ACC,0x08
	mov const7,ACC

	clr status			; buffer used for real time status

	; Option switches	; there are 64 possible configurations
	out PORTC,r13		; By chance the mask for the output pull
						; up is 0x3F
						; four switches are used to set channel
						; the remaining two switches specify start octave


	; Enable portD internal pull up resistors and set data direction
	; PD7 is not used 
	; PD6 is used to set "omni" or octet compatable mode by shorting to ground
	; PD5 is Shift register clock SCL and configured as a timer cascade
	; PD4 is shift register gate signal	
	; PD3 is LED out
	; PD2 is LED out
	; PD1 is MIDI out
	; PD0 is MIDI in
	ldi  ACC, 1 << DDD7  | 1 << DDD6 | 1 << DDD4
	out PORTD,ACC

	sbi PORTD,PortD4	; Keep enable high
	; Enable timer cascade and Clear shift register
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
						 
	

;-----------------------------------------------------------------------
; Simple midi parser
;-----------------------------------------------------------------------
PARSEMIDI:
	sbrs FFLAGS,playactive	; do not reset the dog when active sense is on
	WDR						; let the input ring read keep the dog fed
	cp r6,r7
	breq  PARSEMIDI      	;  L142A Ring is empty


	;Read the next byte of data from the ring
	rcall	_getchar    	; will stall here till a byte is ready to dequeue


	sbi DDRD,DDD2			; flash LED 

RETRYSTATE:

	mov MIDI0,ARGL			; status and channel packed

	sbrc ARGL,7
	rjmp L0448				; command bit is set

	tst status				; check running status state
	breq L0448				; not a running status		

	mov MIDI0,status		; set running status buffer
	mov MIDI1,ARGL
	sbr FFLAGS,1 << running

L0448:
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
	cp r0,zero;
	brne L0479
	
	rcall WIRECMD			; note the ARGL is not touched 
	rjmp CANIGNORE			; done processing

L0479:
	sbrc FFLAGS,running      ; skip read when running status happens
	rjmp ISRUNNING
		
	rcall _getchar      	; We have a voice category message to parse
	sbrc ARGL,7     		; A clock or something could happen anytime
	rcall WIRECMD 			; Process clock or wire command with a call
    
	mov MIDI1,ARGL 			; the data byte

ISRUNNING:

	ldi c_tmp,1     		; if needed > 1
	cp r0,c_tmp
	brlo PLY300
	breq PLY300

STORE3:
	rcall _getchar   		; get the third byte
	sbrc ARGL,7         	; Allow wire clock messages
	rcall WIRECMD     		; Process clock or wire command with a call

	mov MIDI2,ARGL 


PLY300:
	; voice message detected	

	mov status,MIDI0		; update status for voice messages only
	cbr FFLAGS,1 << running ; this is used to skip the read
							; the status buffer will determine if
							; running status is active

  	; filter events here ...

	; at this point we have a one or two byte midi message If it is a note on
	; or a note off we need to map it.

	; test mode switches here if coded C0-3 are channel, C4 is omni
	; C5 is Spencer mode. Ignore if not omni and not selected channel

	cpi ACC,0x0A 			; note on
	brlo PC+2
	rjmp OTHERMSG


#ifdef OLDMAPCASE
	sbic PINC,PINC5
	rjmp MAPCFG0			; default mapping method

	; only message 8 or 9 can get here
	in ARGL,PINC
	ori ARGL,0b11000000     ; use inverted logic on portC 
	com ARGL				; this will make no swictches Config 0

	andi ARGL,0x03			; start with only 3 configurations (There are
	lsl  ARGL				; used to create a jump table to set a case tree).	
	
	ldi ZL,low(MAPCASE * 2)
	ldi ZH,high(MAPCASE * 2)

    add ZL,ARGL			 ; add the address to the offset
    adc ZH,zero			 	
    lpm 				 ; get the low byte address
	mov c_tmp,r0		 ; save the address for reload
	adiw ZL,01			 ; bump the z pointer
	lpm					 ; get the high address
	mov ZL,c_tmp		 ; restore the low address
	mov ZH,r0			 ; set the high address	
	ijmp				 ; and we are off to see the wizard.

MAPCFG3:
	; this is a simple map of 165 to caliola
	rjmp CANIGNORE

MAPCFG1:
	; this is a simple map of w125 to caliola
	rjmp CANIGNORE

MAPCFG2:
	rjmp CANIGNORE


MAPCFG1:	


	rjmp CANIGNORE

MAPCFG0:
	; this is the defaul octet compatable mode


	; setup a special diagnostic mode which maps the keys to the pnuematic
	; this overrides the channel 3 setting 92 29 64
	mov ARGH,MIDI0
	andi ARGH,0x0F

	cpi ARGH,0x02
	brne L0671

	ldi ZL,low(DIAGMAP * 2)
	ldi ZH,high(DIAGMAP * 2)
	mov ARGL,MIDI1
	subi ARGL,0x29 ; set the diagnostic to start at note 29 F1

	add ZL,ARGL
	adc ZH,zero

	lpm			  ; r0 is new Midi message
	mov MIDI1,r0

	; config switch 5 open is Flash mode (default)
	; closed is EEProm mode
L0671:
#else
	; test here for "omni" mode option (EE prom mapping table) (this will be PD 6)

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
				


; This is the octet compatable mode (short PinD 6 to get here )					

OCTETMIDIOUT: 
#endif


	mov ARGH,MIDI0
	andi ARGH,0x0F

	ldi ZL,low(CHANNELS * 2)
	ldi ZH,high(CHANNELS * 2)
	ldi idx,0

L0585:
	lpm 	
	cp ARGH,r0
	breq L0598 
NX0597:					; re-entry point, mapping can map notes to more
	adiw Zl,1			; than one output
	inc idx
	cpi idx,15
	brne L0585
	rjmp CANIGNORE		; done scanning through tables for all active note events
	
	; convert channels to note state table positions
L0598:
	; valid channel is in c_tmp
	push ZL
	push ZH   			; low on registers -- if using eeprom not an issue
	ldi ZL,low(STOPNOTES * 2)
	ldi ZH,high(STOPNOTES * 2)
	add ZL,idx
	adc ZH,zero			; get inded into start point
	lpm
	pop ZH
	pop ZL
#ifdef OLDMAPCASE
	mov c_tmp,r0
	sub c_tmp,MIDI1		; upper limit check
	sbrc c_tmp,7		; test for negative -- no need to process further		
	rjmp NX0597			; re enter loop
#endif
	mov ARGH,r0			 ; low om registers ARGH contains upper note limit

L0621:
	push ZL
	push ZH   			; low on registers -- if using eeprom not an issue
	ldi ZL,low(STARTNOTES * 2)
	ldi ZH,high(STARTNOTES * 2)
	add ZL,idx
	adc ZH,zero			; get index into start point
	lpm
	pop ZH
	pop ZL
#ifdef OLDMAPCASE
	mov c_tmp,MIDI1		; lower limit check
	sub c_tmp,r0
	sbrc c_tmp,7		; test for negative -- no need to process further		
	rjmp NX0597			; out of range re enter loop
#endif
	mov ARGL,r0
	
	; map channel to byte
	
	push ZL
	push ZH   			; low on registers -- if using eeprom not an issue
	ldi ZL,low(OUTSTARTNOTE * 2)
	ldi ZH,high(OUTSTARTNOTE * 2)
	add ZL,idx
	adc ZH,zero			; get inded into start point
	lpm
;	pop ZH
;	pop ZL
#ifdef OLDMAPCASE
	add c_tmp,r0		; c_tmp still points to table index
	mov ARGL,c_tmp


; 2008 04 07 edit
;
#else
	rcall SENDMIDIOUT

	pop ZH
	pop ZL
	
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

#endif
; correction to handle when couplers are active
; Note state table will index which note is playing with a simple counting semaphore

	ldi ZL,low(NoteStateTable)
	ldi ZH,high(NoteStateTable)
	add ZL,c_tmp 	; this should be the magnet we are looking for
	adc ZH,zero
	ld  r0,Z 				; this should be the semaphore count

	; handle the case where a velocity of 0 is a note off
	cp MIDI2,zero
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
	cp MIDI2,zero
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
#ifdef OLDMAPCASE
	pop ZH
	pop ZL
	
	rjmp NX0597			; sweep for couplers
#endif
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

	; ignore bad bytes
	mov ACC,ARGL
	andi ACC,0xF0
	cpi ACC,0xF0
	breq L0589
	; bad byte, if this is called from a parse when called from a voice message
	; parse we are in trouble
	; check for a high bit set -- do our best to re sync the system
EXITRETRY:
	sbrs ARGL,7 
	ret					; ignore bogus data

	; retry to sync back to a valid message
	clr status			; start over
	cbr FFLAGS,1 << running
	pop ZH				; prevent recursion
	pop ZL
	rjmp RETRYSTATE

L0589:

	mov ACC,ARGL
	andi ACC,0x0F     	 ; status >> 4 & 0x0F
	lsl ACC			 ; multiply by two to get table entry

	; use a jump table to set a case tree.	
	ldi ZL,low(SysCommon * 2)
	ldi ZH,high(SysCommon * 2)

    add ZL,ACC			 ; add the address to the offset
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


	rcall _putchar		 ; RT message must return ARGL == echo

L0827:					 ; wait for completion
	cp r4,r5
	brne L0827			 ; If we do not do this the tranmitter stays busy all
						 ; the time  (these messages are single byte msgs) 

	lds ACC,UCSR0B		 ; 	make sure we are not xmitting
	cbr	ACC,1<<UDRIE0	 ;   if we are tranmitting someone is trying
	sts	UCSR0B,ACC		 ;   to violate physics	

	cp MIDI0,ARGL
	breq L0839			 ; skip nekid wire thingys
	rcall _getchar		 ; real time message happened in the middle of
						 ; something else	
	rjmp EXITRETRY		 ; this will check if something went really wrong
L0839:
	ret
	;cbi DDRD,DDD2		 ; clear early?
	;rjmp PARSEMIDI
						 ; like a command byte in the middle of a message	
;----------------------------------------------------------------------------
; System common messages
;----------------------------------------------------------------------------
SYSEXCOM:	
	sbr FFLAGS,1<<sysxcntu	; A bus can be driven through this exeption
;
; Version 3  takes the parse tree from the Octet firmware update.  This is mostly
; so that we can do quick field updates using the octet setup program.
; there is also quite a bit of left over flash memory, so a simple help file can
; be sent in responce to the sysex "HELP"

	ldi ZL,low(M036C)		; save the sysex into the output echo buffer
	ldi ZH,high(M036C)
	st Z+,ARGL
L0483:
	rcall _getchar      	; There is at least one data byte   
	; check here for a sysex too large  (what is too large?)
	; the stack pointer ?
	in ACC,SPL
	in BCC,SPH
	cp ZL,ACC
	cpc ZH,BCC
	brge VALIDATESYSEX		; stop storing table data when SYSEX crashes
	st Z+,ARGL				; into the stack - contunue to parse
BAD_SYSEX:	
	cpi ARGL,0xF7
	breq VALIDATESYSEX		; sysex should be comple

	sbrc ARGL,7     		; A clock or something could happen anytime
	rcall WIRECMD 			; Process clock or wire command with a call
	rjmp L0483

VALIDATESYSEX:
	cbr FFLAGS,1<<sysxcntu	; indicate we are really and truly done
	mov MIDI0,ARGL			; this should be a F7, but if the sysex is
	;st Z+,ARGL				; too large then we will need to check it
							; and continue to dump the garbage

	mov ACC,ZL				; get length of sysex
	mov BCC,ZH

	ldi ARGL,low(M036C)		; save the sysex into the output echo buffer
	ldi ARGH,high(M036C)

	sub ACC,ARGL
	sbc BCC,ARGH			; we now have a length  (note any length over
							; 255 is wonked)

	push ZL					; swap temporary register
	push ZH 				; we need to use Z as an index to the start
	mov ZL,ARGL				; of the buffer, we also will need to retain
	mov ZH,ARGH				; the end of the buffer so we can add
	pop ARGH				; return messages to valid sysex messages
	pop ARGL
	ldd c_tmp,Z+1			; get the MFGR byte 

	cpi c_tmp,0x7E			; we only know experimental codes
							; perhaps later we can display song
							; titles or Kareoke lyrics on an LCD
	breq PC+2
	rjmp CLEANSYX

	; process the sysex here -- likely this will be another jump table
	; most likely this will be state read and configuration messages, 
	; Identical to what the Octet UM0 does.
	; The most useful would be to update the mapping table
	; Alternativly a call to the bootloader can be used for field updates
	; If an LCD pannel is connected to the expansion port
	; the PD song name could be displayed

	; fmt F0 7E <1|2><channel> CB1 CB2
	;              2  x       >1   1    -- system inquiry returns serial #
	;              k  k        2   0    -- documented data set
	;              1  x      <>2   1    -- dumps everything (answer back?)
	;              k  k        3   0    -- documented data read
	;              1  x       >3   1    -- serial# UM1 ID dump request
	;			   1  x       >3   2    -- set parameter data 
	;			   1  x       <3   3    -- firmware update
	;              1  x     <>70   0    -- program ACK echo for NAK - 3 or 4 times
	;              1  x       <1   2    -- part of setup code
	;			   1  x       >4   2    -- vibrato test
	;              1  x       <2   4    -- vibrato resp

	ldd c_tmp,Z+2			; this seems to be the command byte

	andi c_tmp,0x30		; mask out channel request

	cpi c_tmp,0x20
	brne L1233
	; this seems to be the query info command it to identify and return the serial # 
	
.equ asz20 = low((szA20-Answer20)*2)
	ldi idx,asz20				; messy when reading rom, our X and Y pointers are busy
	push ARGL					; this is the length from the swap above
	push ARGH					; this will be popped into the z register

	mov ACC,ARGL
	mov BCC,ARGH


;	ldi ARGL,low(SYSXBFR)		; make a copy in ram of what we are to send
;	push ARGL
;	ldi ARGH,high(SYSXBFR)
;	push ARGH

	; set preamble
	ldi ARGL,low(Answer20*2)	; append the sysex into the output echo buffer
	ldi ARGH,high(Answer20*2)	; This is the flash data version

;	rjmp CPYANSWER
BUFFANSWER:
	mov ZL,ARGL					; set the read pointer to flash mem
	mov ZH,ARGH					
	lpm							; read the flash
	adiw ZL,1				
	mov ARGL,ZL					; save the pointer to flash
	mov ARGH,ZH
	pop ZH						; set the sram pointer
	pop ZL
	st Z+,r0					; save the data byte
	push ZL
	push ZH 
	dec idx						; bump the counter
	brne BUFFANSWER				

	mov ZL,ACC					; get pointer to the answer section
	mov ZH,BCC				

	; set channel
;	ldi idx,asz20

;	ldi ZL,low(M036C)			; update the lenght to XMIT
;	ldi ZH,high(M036C)
;	add ZL,idx
;	adc ZH,zero

	in idx,PinC
	andi idx,0x0F
	
	std Z+18,idx
	std Z+17,idx
	ori idx,0x10
	std Z+2,idx

	pop BCC
	pop ACC						; done with the copy

	ldi ZL,low(M036C)			; update the lenght to XMIT
	ldi ZH,high(M036C)


	sub ACC,ZL
	sbc BCC,ZH					; we now have a length  (note any length over
								; 255 is wonked)
	; set serial

	
	; set eox	

	rjmp CLEANSYX

L1233:
	cpi c_tmp,0x10
	breq PC+2
	rjmp NXTCMDV

	; most of the sequences start with 10

	; need to check if req is our serial #

	;11 ; not seen 
	;12 ; seems to be from reply
	;21 ; seems to be quirey
	;31 ; Dump preamble
	;13 ; dump answerback
	;70 ; write prep sync?


	ldd c_tmp,Z+3				; this seems to be the command byte
	ldd idx,Z+4					; merge them together to form a table hash
	swap c_tmp
	andi c_tmp,0xF0
	andi idx,0x0F
	or c_tmp,idx

	; do the jmp table thing?

	cpi c_tmp,0x21
	brne Ans32

Ans21:
	.equ asz1012 = low((szA1012-Answer1012)*2)
	ldi idx,asz1012
	push ARGL					; this is the length from the swap abov
	push ARGH					; save it for swap back

	mov ACC,ARGL
	mov BCC,ARGH


	ldi ARGL,low(Answer1012*2)	; append the sysex into the output echo buffer
	ldi ARGH,high(Answer1012*2)	; This is the flash data version

BIGBUFA:
	mov ZL,ARGL					; set the read pointer to flash mem
	mov ZH,ARGH					
	lpm							; read the flash
	adiw ZL,1				
	mov ARGL,ZL					; save the pointer to flash
	mov ARGH,ZH
	pop ZH						; set the sram pointer
	pop ZL
	st Z+,r0					; save the data byte
	push ZL
	push ZH 
	dec idx						; bump the counter
	brne BIGBUFA				

	mov ZL,ACC					; get pointer to the answer section
	mov ZH,BCC				

	; set channel
;	ldi idx,asz20

;	ldi ZL,low(M036C)			; update the lenght to XMIT
;	ldi ZH,high(M036C)
;	add ZL,idx
;	adc ZH,zero

	in ARGL,PinC
	mov ARGH,ARGL
	andi ARGL,0x0F


	ldi ARGH,127			; full board
	clr r0

	clr zero				; hack to make the table dynamic
	ldi idx,110				; woould be better to put this into eeprom
	add ZL,idx
	adc ZH,zero

	st Z,ARGL
	std Z+16,r0
	std Z+32,ARGH

	mov ZL,ACC					; get pointer to the answer section
	mov ZH,BCC				



	std Z+40,ARGL
	std Z+39,ARGL
	ori ARGL,0x10
	std Z+2,ARGL

	pop BCC
	pop ACC						; done with the copy

	ldi ZL,low(M036C)			; update the lenght to XMIT
	ldi ZH,high(M036C)


	sub ACC,ZL
	sbc BCC,ZH					; we now have a length  (note any length over
								; 255 is wonked)
	; set serial

	
	; set eox	

	rjmp CLEANSYX




;	rjmp CPYANSWER

Ans32:
	cpi c_tmp,0x32
	brne Ans31
	; this is the write data command  -- looks like the whole world is written
	; 70 nn ID nn LN 	
	; parameters identified
	; 10 -- channel  diags and UM1 I/O settings (softmode/bounce delay)
	; 33 hold settings
	; 34 channel addresses and mask bit
	; 3C note starts
	; 44 note ends
	; 4c out starts

	; data is stored in 2 bytes to decode  b2 >> 1 to cary b1 << 1 from cary
	;

	; probable decode pattern
	; 
	; count to index & loop
	; point z to buffer & read bit to temp
	; bump z and read data
	; shit to form parameter byte
	; save parameter somewhere 

	; case on parameter group
	; get parameters from EE to buffer
	; compare ee to new parm?
	; update ee if different
	; continue looping through parms till done

	; system has a checksum use or ignore?


	; special case on 10  -- keep a flag and only update when triple checked?
	; load wear on EEprom? Or just update the whole mess at once?


	; only the 1070 answer preamble is returned
;	pop ZH						;  remove the pointer to end of memory
;	pop ZL
	ldi ARGL,low(M036C)		; save the sysex into the output echo buffer
	ldi ARGH,high(M036C)	; overwriting write data message
;	push ZL
;	push ZH

	rjmp RPLY1070

Ans31:
	cpi c_tmp,0x31
	brne AnsXX

RPLY1070:
	.equ asz1070 = low((szA1070-Answer1070)*2)
	ldi idx,asz1070
	push ARGL					; this is the length from the swap abov
	push ARGH					; save it for swap back
	ldi ARGL,low(Answer1070*2)	; append the sysex into the output echo buffer
	ldi ARGH,high(Answer1070*2)	; This is the flash data version
	rjmp CPYANSWER





AnsXX:
	brne NXTCMDV


CPYANSWER:
	mov ZL,ARGL					; set the read pointer to flash mem
	mov ZH,ARGH					
	lpm							; read the flash
	adiw ZL,1				
	mov ARGL,ZL					; save the pointer to flash
	mov ARGH,ZH
	pop ZH						; set the sram pointer
	pop ZL
	st Z+,r0					; save the data byte
	push ZL
	push ZH 
	dec idx						; bump the counter
	brne CPYANSWER				
	pop BCC
	pop ACC						; done with the copy

	ldi ZL,low(M036C)			; update the lenght to XMIT
	ldi ZH,high(M036C)

	sub ACC,ZL
	sbc BCC,ZH					; we now have a length  (note any length over
								; 255 is wonked)



NXTCMDV:

CLEANSYX:
	lds c_tmp,UCSR0B		; 	make sure we are not xmitting
	cbr	c_tmp,1<<UDRIE0	;   if we are tranmitting someone is trying
	sts	UCSR0B,c_tmp		;   to violate physics	

	clr r5
	mov r4,ACC			;   still contains the length
							

;	Echo sysex and any return data

; wait for xmitter to complete then stop checking ring
	
					; keep the system quiet when no data to echo
	lds ACC,UCSR0B		; 
	sbr	ACC,1<<UDRIE0	; 
	sts	UCSR0B,ACC
L1419:					; Well behaved midi will not send new data
	cp r4,r5			; until old data is processed. Wonky midi
	brne L1419			; will try and blast SYSEX data and blow the buffer

	; if our buffer is maxed out shout we echo more data?
	; tst BCC if != 0 then continue to process data
	; either by a block move or extending the ring size

	lds ACC,UCSR0B		; 	make sure we are not xmitting
	cbr	ACC,1<<UDRIE0	;   if we are tranmitting someone is trying
	sts	UCSR0B,ACC		;   to violate physics	

	rjmp KILLSYSCOM

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
	ldi ZL,low(M036C)		; save the sysex into the output echo buffer
	ldi ZH,high(M036C)		; this does nothing but echo 
	rjmp VALIDATESYSEX		; some keyboards may block store the sysex with
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

	mov ACC,ARGL
	sbrc FFLAGS,playactive
	rjmp EXITREALTIME
	sbr FFLAGS,1 << playactive
	ldi ARGH,(1<<WDE) | (1<<WDP2) | (1 << WDP0) ; closer to 500ms
	rcall L1113
	mov ARGL,ACC
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

#ifdef OLDMAPCASE
MAPCASE:
; Mapping configurations  -- these are selected by the configuration switches, There are
; 64 possible switch combinations. Logically the first 4 are channel number.  The remaining two swictches are for mode
; Possible modes are -- octet compatability vs table mode.  This gives 16 master channels or 16 mapping tables
; Tables map from midi to events,  most of these modes will map to caliola
.db byte1(MAPCFG0),byte2(MAPCFG0),byte1(MAPCFG1),byte2(MAPCFG1),byte1(MAPCFG2),byte2(MAPCFG2),byte1(MAPCFG3),byte2(MAPCFG3)
.db byte1(MAPCFG0),byte2(MAPCFG0),byte1(MAPCFG0),byte2(MAPCFG0),byte1(MAPCFG0),byte2(MAPCFG0),byte1(MAPCFG0),byte2(MAPCFG0)
#endif





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



Answer20:
;     0   1    2   3    4     56789ABCDF    F
.db 0xF0,0x7E,0x10,0x01,0x02,"0000171842",0x00
.db 0x00,0x00,0x00,0xF7
;     10^^^11^^^12^--13these bytes seem to indicate chanel info
szA20:

Answer1070:  ; in responce to 1031
.db 0xF0,0x7E,0x10,0x70,0x00,0xF7
szA1070:

.if 0
Answer1012:	; in responce to 1021
;   0    1    2    3    4    5    6    7    8    9    a    b    c    d    e    f    
.db 0xF0,0x7E,0x10,0x01,0x02,0x04,0x29,0x30,0x30,0x30,0x30,0x30,0x31,0x34,0x37,0x38 ;00
.db 0x34,0x20,0x20,0x20,0x55,0x4D,0x30,0x30,0x37,0x2D,0x31,0x31,0x2D,0x30,0x30,0x01 ;10
.db 0x00,0x01,0x03,0x01,0x04,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x05,0x05,0x4F,0x63	;20
.db 0x74,0x65,0x74,0x20,0x44,0x65,0x73,0x69,0x67,0x6E,0x20,0x43,0x6F,0x72,0x70,0x2E ;30
.db 0x20,0x2D,0x20,0x4D,0x49,0x44,0x49,0x20,0x50,0x72,0x6F,0x63,0x65,0x73,0x73,0x6F ;40
.db 0x72,0x20,0x55,0x4D,0x30,0x00,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20 ;50
.db 0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x02,0x32,0x01,0x00 ;60
.db 0x00,0x00,0x00,0x02,0x02,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x29,0x1A ;70
.db 0x12,0x46,0x51,0x12,0x26,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x54,0x45 ;80
.db 0x19,0x50,0x51,0x25,0x51,0x7F,0x7F,0x7F,0x7F,0x7F,0x7F,0x7F,0x7F,0x7F,0x00,0x00 ;90
.db 0x04,0x20,0x1F,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x7F,0x01 ;A0
.db 0x00,0xF7 
szA1012:
.endif

.if 1
Answer1012:


;                +---------channel
;                |    +----cmmd b1
;                |    |    cmnd b2
;                v    v    v
.db 0xF0,0x7E,0x10,0x01,0x02,0x04
;nasty messy alignment stuff (first byte of string is last byte of upper table)
SERNUM:
.db 0x29,"0000011958 " ; this is the serial #
PRODUCT:
.db "  UM0",0x30 + (__MONTH__ / 10)
DATE:

.db 0x30 + __MONTH__ - ((__MONTH__ / 10) * 10),'-'
.db 0x30 + (__DAY__ / 10),0x30 + __DAY__ - ((__DAY__ / 10) * 10)
.db '-',0x30 + (__YEAR__ / 10)
.db 0x30 + __YEAR__ - ((__YEAR__ / 10) * 10),0x03 ; <--- HW rev

;.db "4-24-07",0x01
UNDEF1:

;         firmware rev          +----- soft mode 1
;            |                  |
;          +----+               |         +----+ affects channel (main channel)
;   HW     v    v               v         v    v
.db 0x01,0x01,0x0A,0x01,0x04,0x04,0x01,0x00,0x00,0x00,0x00,0x00,0x0A,0x05
;                                    ^                            ^-----^-----probably switchbounce
;                                    +-- Soft mode 2
;	 123456789.123456789.123456789.12345678	
MFGSTR:
.db "Octet Design Corp. - MIDI Processor UM0",0x00

.db 0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20
.db 0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20

;   sustain hold mode 
;   0 = Hold notes
;   1 = P2 mode
;	2 = use channel n
.db 0x02,0x31

CHANNELS:
.db 0x01,0x00,0x00,0x00,0x00,0x02,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10
STARTNOTES:
.db 0x29,0x1A,0x12,0x46,0x51,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
STOPNOTES:
.db 0x58,0x45,0x19,0x50,0x51,0x2c,0x7F,0x7F,0x7F,0x7F,0x7F,0x7F,0x7F,0x7F,0x7F,0x7F
OUTSTARTNOTE:
.db 0x01,0x01,0x05,0x21,0x20,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00

.db 0x7F,0x01,0x00,0xF7 
szA1012:

DIAGMAP:
;    f  f#g  g#|a  a# b  c| c# d  d# e|f  f# g g#|a  a# b  c |c# d d# e
.db 11,1,44,13,42,40,10,15,38,17,8,36,34,19,32,6,21,30,23,28,25,4,27,26
.db 24,29,3,22,31,20,33,18,5,35,37,16,7,39,41,14,43,9,12,2

.equ block_1 = -1		; physical valves start at number 1
.equ block_3 = 2*16-1	; this is the drum valve which maps to a logical 0
.equ block_2 = 16-1		; the first note is number 2 on block 1

.db block_1+12,block_1+ 2,block_1+14,block_3+13	; valve 1 D#2,F1,C5
.db block_3+11,block_3+ 9,block_1+16,block_1+11	; valve 2 A#4,G#4,G2,D2
.db block_3+ 7,block_2+ 2,block_3+ 5,block_1+ 9	; valve 3
.db block_3+ 3,block_2+ 4,block_1+ 7,block_3+ 1	; valve 4
.db block_2+ 6,block_2+15,block_2+13,block_2+ 8	; valve 5
.db block_2+10,block_1+ 5,block_2+12,block_2+11	; valve 6 
.db block_2+ 9,block_2+14,block_2+ 7,block_1+ 4	; valve 7 
.db block_2+16,block_2+ 5,block_2+ 3,block_3+ 2	; valve 8
.db block_1+ 6,block_3+ 4,block_2+ 1,block_3+ 6	; valve 9
.db block_1+ 8,block_3+ 8,block_1+15,block_3+10	; valve 10
.db block_3+12,block_1+10,block_1+ 3,block_1+13	; valve 11





.endif

