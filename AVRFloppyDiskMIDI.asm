;
; MIDI PLayback code for ATMEL AT90S8515
.include "8515def.inc"

; System equates
.equ NOOFTASKS = 2  ; tasks are numbered 0 - 3
.equ STACKSIZE = 32 ; size must be a even multiple of 2

;Register variables

;Program load and FDC status registers
;R0 used by LPM 
.def TL0 = r1 ; timer 0 reload value
.def TL1 = r2 ; timer 1 reload value low
.def TH1 = r3 ; timer 1 reload value high

;R4  remander from divison
;R5 

;R6   MIDI timer # of midi tix elapsed
;R7
;R8  
;R9

.def Mf_ct1   = r10  ; this is the time the midi is parsed to
.def Mf_ct2   = r11   
.def Mf_ct3   = r12  
.def Mf_ct4   = r13

.def Txbyte   = r14
.def ssreg    = r15

.def c_tmp    = r16 ; used for compares and extended math
.def idx      = r17 ; for short loop counters
.def PAGE     = r18 ; ram page calculation
.def RESULT   = r19 ; also used as file format
.def FDCFLAGS = r20 ; 
.def FFLAGS   = r21 ; more flags these are mostly for file load and playback
.def ARGL     = r22 ; to OS, usually a pointer
.def ARGH     = r23 ; passed arguments
.def ACC      = r24 ; genereral use registers
.def BCC      = r25 ; secondary general use register
; double registers
;X26/27
;Y28/29
;Z30/31
;.def MIDIIPtr = r28 ; used for accessing the ring from the program
;.def MIDIOPtr = r30 ; used for serial transmission 



;****************************************************************************************
;								Floppy disk equates and defs

;****************************************************************************************

; error codes
.equ ERR_MEMFULL     = 0xFE
.equ ERR_TRANSFER    = 0xFD
.equ ERR_SEEK        = 0xFC
.equ ERR_RECALIBRATE = 0xFB
.equ ERR_PARAMETERS  = 0xFA
.equ ERR_BADMIDI     = 0xF9

.equ ERR_TIMEOUT     = 0x40

; disk parameters
.equ HC_SIZE		  = 2880 ; largest drive supported
.equ MAX_SECTORS      = 18   ; largest drive supported
.equ DTL              = 0xFF ; determines lenght of transfer (sector size)
.equ SPEC1            = 0xDF ; our drives are always this
.equ SPEC2            = 0x03 ; set for no DMA!
.equ MOTOR_OFF        = 7000 ; motor timeout (3000 is example)
.equ WAKEUP           = 2000 ; timeout on I/0, fdc wont quit

; misc parameters
.equ MAX_ERRORS       = 6
.equ MAX_RESULTS      = 7
.equ NR_DRIVES        = 1
.equ DIVISOR          = 128
.equ SECTOR_SIZE_CODE = 2
.equ TIMEOUT          = 500
.equ BASE_SECTOR      = 1
.equ NO_SECTOR        = 0
.equ NO_CYL           = 0xFFFF
.equ NO_DENS          = 100
.equ NR_HEADS         = 2 

; FDCFLAG bits
.equ timeoutF  = 7 ; FDC timed out
.equ wait_poll = 6 ; used to escape wait loop
.equ fdc_irq   = 5 ; end timer if set;
.equ needreset = 4 ; try and make the floppy listen
.equ fdcreset  = 3 ; Used by OCB to time reset pulse
.equ mtrState  = 2 ; state of motor for timer
.equ xfercpl   = 1 ; used for timing motor
.equ rcr       = 0 ; recalibrate

.equ FDC = 0x08 ;memory maped location of FDC chip 0x800 
.equ FDC_DOR  = FDC + 2 ; state and motor control register
.equ FDC_MSR  = FDC + 4 ; main status register - read only
.equ FDC_DSR  = FDC + 4 ; write only 
.equ FDC_DATA = FDC + 5 ; floppy FIFO
.equ FDC_RATE = FDC + 7 ; also known as FDC_DIR
.equ FDC_DIR  = FDC + 7 ; also known as FDC_RATE

.equ FDC_CCR  = FDC + 0 ; 37C78 special function registers
.equ FDC_CCRDATA = FDC + 1;

; DIR
.equ diskchange = 7     ; only active when motor is on

; MSR bit definitions
.equ MASTER    = 7      ; FDC is master
.equ DIRECTION = 6      ; IO read or write direction
.equ CTL_BSY   = 4      ; FDC is busy

; DOR
.equ MOTOR_SHIFT = 4    ; bits for control of motor
.equ ENABLE_INT  = 0x0C ; used for setting DOR Port

; STO
.equ ST0_BITS  = 0x38 ; for seek clear
.equ TRANS_ST0 = 0x00 ; not used in polling
.equ SEEK_ST0  = 0x20 ; mask for seek

; ST1
.equ BAD_SECTOR    = 0x05 ; force recalibrate if these bits are set
.equ WRITE_PROTECT = 0x02 ; set when diskette is write protected 

; ST2
.equ BAD_CYL = 0x1F       ; force recalibrate when set

; ST3
.equ ST3_FAULT	= 0x80	; drive is sick
.equ ST3_WP     = 0x40  ; set when diskette is write protected
.equ ST3_READY  = 0x20  ; all is ready to rock and roll


;****************************************************************************************
;								floppy disc commands
;****************************************************************************************

.equ FDC_SEEK  = 0x0F 
.equ FDC_READ  = 0x46 
.equ FDC_WRITE = 0xC5 
.equ FDC_SENSECMD = 0x08
.equ FDC_RECALIBRATE = 0x07
.equ FDC_SPECIFY = 0x03
.equ FDC_READ_ID = 0x4A
.equ FDC_FORMAT  = 0x4D
.equ FDC_CONFIGURE = 0x13

.equ BS_jmpBoot = 0;
.equ BS_OEMNAME     = 3
.equ BPB_BytsPerSec  = 11
.equ BPB_SecPerClus = 13 
.equ BPB_ResvdSecCnt = 14 
.equ BPB_NumFATs    = 16 
.equ BPB_RootEntCnt = 17
.equ BPB_TotSec16   = 19
.equ BPB_Media      = 21
.equ BPB_FATSz16    = 22
.equ BPB_SecPerTrk  = 24
.equ BPB_NumHeads   = 26
.equ BPB_filler     = 28 
.equ BS_DrvNum      = 36
.equ BS_Reserved1   = 37
.equ BS_BootSig     = 38
.equ BS_VolID       = 39
.equ BS_VolLab      = 43
.equ BS_FileSysType = 54

; directory definitions

.equ ATTR_READ_ONLY = 0;0x01
.equ ATTR_HIDDEN    = 1;0x02
.equ ATTR_SYSTEM    = 2;0x04
.equ ATTR_VOLUME_ID = 3;0x08
.equ ATTR_DIRECTORY = 4;0x10
.equ ATTR_ARCHIVE   = 5;0x20

.equ ATTR_LONG_NAME = 0x0F ; ATTR_READ_ONLY | ATTR_HIDDEN | ATTR_SYSTEM | ATTR_VOLUME_ID

;ddir 
.equ DIR_Name  = 0         ; 11 Short name.
.equ DIR_Attr         = 11 ; 1 File attributes: The upper two bits of the attribute byte are reserved and should
                           ; always be set to 0 when a file is created and never modified or
                           ; looked at after that.
.equ DIR_NTRes        = 12 ; 1 Reserved for use by Windows NT. Set value to 0 when a file is
                           ; created and never modify or look at it after that.
.equ DIR_CrtTimeTenth = 13 ; 1 Millisecond stamp at file creation time. This field actually
                           ; contains a count of tenths of a second. The granularity of the
                           ; seconds part of DIR_CrtTime is 2 seconds so this field is a
                           ; count of tenths of a second and its valid value range is 0-199
                           ; inclusive.
.equ DIR_CrtTime      = 14 ; 2 Time file was created.
.equ DIR_CrtDate      = 16 ; 2 Date file was created.
.equ DIR_LstAccDate   = 18 ; 2 Last access date. Note that there is no last access time, only a
                           ; date. This is the date of last read or write. In the case of a write,
                           ; this should be set to the same date as DIR_WrtDate.
.equ DIR_FstClusHI    = 20 ; 2 High word of this entry’s first cluster number (always 0 for a
                           ; FAT12 or FAT16 volume).
.equ DIR_WrtTime      = 22 ; 2 Time of last write. Note that file creation is considered a write.
.equ DIR_WrtDate      = 24 ; 2 Date of last write. Note that file creation is considered a write.
.equ DIR_FstClusLO    = 26 ; 2 Low word of this entry’s first cluster number.
.equ DIR_FileSize     = 28 ; 4 32-bit DWORD holding this file’s size in bytes.

; Ldir
.equ ORD = 0;
.equ Name1    = 1
.equ Attr     = 11
.equ Type     = 12
.equ Chksum   = 13 
.equ Name2    = 14
.equ FstClsLO = 26
.equ Name3    = 28

.equ sizeof_direntry = 32

; LCD
.equ NOSEL         = 0x1000
.equ LCDDATAIN     = 0x03 
.equ LCDDATAOUT    = 0x02;
.equ LCDCOMMANDOUT = 0x00;
.equ LCDBUSYIN     = 0x01;

; MIDI and file system
;FFlags

.equ playactive  = 7; set when song is using interrupts
.equ fseof       = 6; no more btytes to read, (we still have bytes to send)
.equ eot         = 5; buffer is empty, if this and eof are set and we are not playing, then
             ; we are done tranmitting the song this gets set when there are 2 FF's in the ring
.equ playall     = 4; set when chaining songs
.equ sysxcntu    = 3; for ignoring sysex messages
.equ running     = 2; running status active
.equ pending     = 1; a block of midi is being transmitted in this timeslice 
.equ rfu         = 0;

;=================================================================================================
;               
;                     D A T A   S E G M E N T 
;
;=================================================================================================

.DSEG                ; Start data segment 
.ORG 0x60            ; Set SRAM address to hex 37 

; OS globals
TTSL:   .BYTE NOOFTASKS *2 ; task timers
SPTS:   .BYTE NOOFTASKS ; task saved stack pointers
SPTH:   .BYTE NOOFTASKS

;index into results data structure
RESULTS:
ST0:       
ST3:     .byte 1 ; status register 0, staus register 3 returned by DRIVE _SENSE
ST1:
ST_PCN:  .byte 1 ; status register 1, current cylindar
ST2:     .byte 1 ; status register 2
ST_CYL:  .byte 1 ; slot where controller returns cylinder
ST_HEAD: .byte 1 ; slot where controller returns head
ST_SEC:  .byte 1 ; slot where controller returns sector
ST_RFU:  .byte 2
;floppy globals -- pointed to by Z
; floppy data structure
;--------------------  offset  datalen
.equ fl_curcyl       = 0     ; 1   current cylinder as read
.equ fl_cylinder     = 1     ; 1   cylinder number addressed
.equ fl_sector       = 2     ; 1   sector addressed
.equ fl_head         = 3     ; 1   head number addressed 
.equ f_sectors       = 4     ; 1   number of sectors in format 9 | 18
.equ current_spec1   = 5     ; 1   only specify when we chance densities

;Filesystem globals
.equ block           = 6     ; 2   current block to start transfers at
.equ f_count         = 8     ; 3   this many bytes to transfer
.equ f_nexttrack     = 11    ; 2   dont do blocks above this number
.equ target          = 13    ; 3   where the data is transfered to
.equ nbytes          = 16    ; 3   bytes to transfer
.equ star_tp         = 19    ; 2   saved computed tranfer pointer

;Directory and file load 
.equ cluster           = 21    ; 2
.equ RootDirSectors    = 23    ; 2
.equ FirstDataSector   = 25    ; 2
.equ FirstSectorofRoot = 27    ; 2
.equ s_count           = 29    ; 2
.equ c_count           = 31    ; 1
.equ file_size         = 32    ; 3 
.equ star_dir          = 35    ; 2   pointer to current song's dir entry
.equ user_phys         = 37    ; 3   it is a signed long and could conceivably go negative
.equ offset            = 40    ; 2   index into FAT table

; MIDI globals
.equ Mf_toberead       = 42    ; 4 (3)    
.equ division          = 45    ; 2
.equ tempo             = 47    ; 3
.equ format            = 50    ; 1
.equ ntrks             = 51    ; 1
;.equ eof_count         = 52    ; 3     
.equ playpos           = 52    ; 3
.equ Mf_deltatime      = 55    ; 4   
.equ status            = 59    ; 1
;lookfor
;needed
;varnum

;OS globals 
.equ ERRORS            = 60    ; 
.equ task_num          = 61    ; 1  this is the current running task ID number OS global
.equ d                 = 62    ; 1  drive ID
.equ smcucr            = 63    ; 1  for LCD hardware patch
.equ sizeof_fp         = 64    ;  
f_fp: .byte sizeof_fp                            

; transfer data structure pointed to by Y
.equ tr_count = 0                   ; byte count to transfer
.equ tr_fsectors = tr_count    + 2  ; number of sectors in track to transfer
.equ io_nbytes   = tr_fsectors + 2  ; 
.equ tr_block    = io_nbytes   + 2  ; block address
.equ tr_phys     = tr_block    + 2  ; physical 24 bit address
.equ sizeof_tp   = tr_phys     + 3
tp: .byte sizeof_tp * 18


.ORG RAMEND-(NOOFTASKS * STACKSIZE); - STACKSIZE
STKRESERVE: .BYTE (NOOFTASKS * STACKSIZE); - STACKSIZE
;****************************************************************************************
;								External RAM definitions
;****************************************************************************************
.equ EXTRAM = 0x8000 ;page 0 window 8000 through 9FFF
;.equ TPA    0xA000 ;also mapped from 0x2000 through 0x7FFF 
; external memory start at 8000 and contains 4 32K pages
; the first page contains the FAT and the Directory followed by
; the song.

.ORG EXTRAM
; page 0 data structures, make sure PAGE register is set to 00 in port D
; before accessing these data structures

MIDIQ:      .BYTE 256             ; circular buffer for parsing MIDI to serial out
MSGQ:       .BYTE 256             ; circular buffer for writing LCD
BOOTSECTOR: .BYTE 512
fat_buffer: .byte 0x600           ; 1536 bytes
foobar:     .byte 1               ; see if the bug is byte related
directory_buffer: .byte 0x1800    ; 6144 bytes 12 sectors 6K for yamaha,pds

TPA: .ORG directory_buffer + 6144 ; song starts here

;***************************************************************************
;* Interrupt vectors
;********************	
.cseg
.org 0
	rjmp Main
	reti				; External 0 interrupt  Vector 
	reti 				; External 1 interrupt  Vector 
	reti				; Timer 1 Capture Event
	reti				; Timer 1 Compare Match A
	rjmp HandleFDCRST	; Timer 1 Compare Match B
	rjmp HandleT1		; Timer 1 Overflow Event
	rjmp HandleT0		; Timer 0 Overflow  Vector 
	reti				; Serial Transfer Complete
	reti    			; UART Rx,Complete
	reti				; UART Data Register Empty
	reti				; UART Tx Complete 
	reti				; Analogue Comparator Vector 
		
;*****************************************

HandleFDCRST:
	WDR
	in ssreg,SREG
	sbrc FDCFLAGS,fdcreset 
   rjmp FDCPULSE
	sbr FDCFLAGS,1 << fdcreset 
	push ACC
	push BCC

	ldi ACC,15
	mov BCC,TL1 
	add ACC,BCC

	out TCNT1H,TH1
	out TCNT1L,TL1

	out OCR1BH,TH1
	out OCR1BL,ACC

	ldi ACC,0x20   ;00100000
	out TCCR1A,ACC ;clear the pin ?

	pop BCC
	pop ACC
	out SREG,ssreg
	reti

FDCPULSE:
   cbr FDCFLAGS,1 << fdcreset 
	push ACC

	clr ACC
	out TCCR1A,ACC ;disable capture
;	out TCCR1B,ACC ; stop clock

	in  ACC,TIMSK  ; disable only our interrupt
	andi ACC,0b11011111 ;TOIE1|OCIE1B|TOIE0
	out TIMSK,ACC

	pop ACC
	out SREG,ssreg
	reti

HandleT1:
	WDR
	out TCNT1H,TH1
	out TCNT1L,TL1
	in ssreg,SREG
	push c_tmp
	ldi c_tmp,0xFF
	sub r6,c_tmp
	sbc r7,c_tmp
	sbc r8,c_tmp
	sbc r9,c_tmp
	pop c_tmp

	out SREG,ssreg
	reti

HandleT0: ; timer interrupt 0 TINT:
	WDR
   ;reload timer
	out TCNT0,TL0
	;save state
	in ssreg,SREG

	;push registers and callers address on current task's stack
	push ssreg
	push YL
	push YH
	push ACC
	push BCC
	ldd ACC,Z+task_num
	push ACC
	push c_tmp        ; save these so pre-emption does not trash them
	push idx          ; this also frees some low registers

 	ldi YH,high(SPTS) ; Load stack pointer from offset
 	ldi YL,low(SPTS)  ; Save at return location
	add YL,ACC
	brcc PC+2
	inc YH
	in  acc,SPL
	st  Y,acc
	in  acc,SPH
	std Y+NOOFTASKS,acc
		
 	ldi YL,low(TTSL + NOOFTASKS * 2 - 2) ; 
 	ldi YH,high(TTSL+ NOOFTASKS * 2 - 2) ; Start the counters for 16 bit timers

	ldi idx,NOOFTASKS                    ; Scan all timers timers are set to -1 to stop
K2:

	ldd BCC,Y+1             ; task time high byte
	ld  ACC,Y               ; task time low byte
	cpi ACC,low(0xFFFF)     ; compare low byte
	ldi c_tmp,high(0xFFFF) 
	cpc BCC,c_tmp
	breq K1

TICKEVENT:
	; task timer is active
	
	;ACC & BCC form a 16 bit timer value
	sbiw ACC,1
	st   Y,ACC  ; save new count
	std  Y+1,BCC  ; save new count
	cpi  ACC,0
	ldi  c_tmp,0
	cpc  BCC,c_tmp
	brne K1    ; continue

	;ldd c_tmp,Z+task_num
	;ldi c_tmp,NOOFTASKS
	mov c_tmp,idx
	dec c_tmp
	std Z+task_num,c_tmp
	
	sbr FDCFLAGS,1 << wait_poll ; task timed out

QSHIFT:
 	ldi YH,high(SPTS) ; Load stack pointer from offset
 	ldi YL,low(SPTS)  ; Save at return location
	add YL,c_tmp
	brcc PC+2
	inc YH
	ld  acc,Y
	out SPL,acc
	ldd acc, Y+NOOFTASKS
	out SPH, acc
	
KEXIT:
	pop idx
	pop c_tmp
	pop ACC
	std Z+task_num,ACC
	pop BCC
	pop ACC
	pop YH
	pop YL
	pop ssreg

	out SREG,ssreg
	reti	
K1:  

	sbiw Yl,2
	dec idx
	brne K2 ; search for next timer
	rjmp KEXIT
; end of timer 


;_SIGNL:
	;unblock task on next tick
;   ret
   
_WAITS:
   ;fall through to timer
_WAITT:
   CLI ; make atomic
	push YH  ; Y may point to a data struture 
	push YL
	push BCC
	ldd BCC,Z+task_num ;TASKNUM
	rcall _SETTIME
	pop BCC
	pop YL
	pop YH
	cbr FDCFLAGS,1 << wait_poll
	SEI ; allow timeout
Poll:
   WDR
	sbrs FDCFLAGS,fdc_irq
	rjmp p001 
	sbic PIND,3
	rjmp p000
p001:
	sbrs FDCFLAGS, wait_poll
	rjmp Poll
p000:
	cbr FDCFLAGS,1 << wait_poll
	cbr FDCFLAGS,1 << fdc_irq
	CLI
	ldd ACC,Z+task_num ;TASKNUM ; clear any remaining time
	ldi ARGH,0xFF
	ldi ARGL,0xFF
	SEI 
	ret

;DEFER:
;	in ssreg,SREG
	
	;push registers and callers address on current task's stack
;  push ssreg
;	push XL
;	push XH
;	push YL
;	push YH
;	push ACC
;	push BCC
;	push TASKNUM ; callers task
;	rjmp QSHIFT 

_SETTIME:
	ldi YH,high(TTSL)	
 	ldi YL,low(TTSL)
	lsl BCC    ; multiply by 2	
	add YL,BCC
	brcc PC+2
	inc YH
	st  Y,ARGL
	std Y+1,ARGH
	ret

; create a delayed task
; task will run after tick counter expires/
_CREAT:
	; ACC task #
	; XL,XH -> function addr
	; ARGL,ARGH time to wait before function runs
	ldd BCC,Z+task_num
	cp ACC,BCC
	breq EXISTS
	CLI
	mov BCC,ACC    ; save task id 
	rcall _SETTIME

	in ARGL,SPL
	in ARGH,SPH; save stack ptr

 	ldi YL,low(SPTS)  ; Save at return location
 	ldi YH,high(SPTS) ; Load stack pointer from offset
	;lsl ACC    ; mpy  by 2
	add YL,ACC
	brcc PC+2
	inc YH
	ld  c_tmp,Y
	out SPL,c_tmp
	ldd c_tmp,Y+NOOFTASKS
	out SPH,c_tmp
	push XL ; save return add on new stack
	push XH
	push ssreg
	push YL
	push YH
	push ACC
	push BCC
	
	push BCC           ; this is new task id
	ldd c_tmp,Z+task_num ; this is return task id
	push c_tmp        ; save these so pre-emption does not trash them
	push idx          ; this also frees some low registers

	in ACC,SPL
	in BCC,SPH
	st Y,ACC
	std Y+NOOFTASKS,BCC

	out SPL,ARGL
	out SPH,ARGH

	SEI
EXISTS:
   ret	

; lcd init
lcdinit:

	ldi ARGL,0x30 ; function set
	rcall wcmd  
	ldi ARGL,0x30 ; function set
	rcall wcmd  
	ldi ARGL,0x30 ; function set
	rcall wcmd  
	ldi ARGH,0x00
	ldi ARGL,5
	rcall _WAITT  ; 5 tix
	ldi ARGL,0x38 ; function set
	rcall wcmd  
	ldi ARGL,0x08 ; display off
	rcall wcmd  
	ldi ARGL,0x01 ; display clear
	rcall wcmd  
	ldi ARGH,0x00
	ldi ARGL,5
	rcall _WAITT  ; 
	ldi ARGL,0x06 ; set cursor 
	rcall wcmd  
	ldi ARGL,0x0C ; home cursor 
	rcall wcmd  
	ldi ARGH,0x00
	ldi ARGL,5
	rcall _WAITT  ; 
	ret;

lineout:
	; values passed are location in ACC and memory ptr in args
    rcall wcmd   ; will trash args register with the call to the timer
lineloop:
	CLI
   	WDR
	push ZH             ; protect Z
	push ZL
	mov ZL,XL
	mov ZH,XH
	lpm
	adiw XL,1
	pop ZL              ; z must point to globals to shif memory mode
	pop ZH          
	mov ARGL,r0        ; test for eol
	cpi ARGL,0x00
   	breq ldone ;PC+4
   	SEI
	rcall wdat 
   	rjmp lineloop
ldone:
	SEI
	ret
	
lcd_out:
	dec BCC             
	breq e_lcdOut      
	ld ARGL,X+          
	cpi ARGL,0          ; some simple filters
	breq lcd_out        
	cpi ARGL,0xFF
	breq lcd_out
	rcall wdat
	rjmp lcd_out
e_lcdOut:
	ret	


; ldc command
wcmd:
	CLI
	in ACC,MCUCR       ; turn off ram
	std Z+smcucr,ACC
	andi ACC,0x3F
	out MCUCR,ACC 

;bsyw:
;	clr ACC
;	ldi ARGH,LCDBUSYIN
;	out PORTC,ARGH     ; sets RS & R/!W
;   sbi PORTC,2        ; start !E strobe
;nop
;	out DDRA,ACC   ; set as in
;	cbi PORTC,2    ; strobe in the data
;	sbic PINA,7
;	rjmp bsyw
;	ser ACC
;	out DDRA,ACC   ; set as in


	out PORTA,ARGL         ; place command onto bus
	ldi ARGH,LCDCOMMANDOUT
	out PORTC,ARGH         ; sets RS & R/!W
	sbi PORTC,2            ; start !E strobe
 	nop                    ; delay .25 uS
	cbi PORTC,2            ; strobe in the data

	ldd ACC,Z+smcucr
	out MCUCR,ACC          ; restore memory
	ldi ARGH,0x00          ; settle 
	ldi ARGL,1
	SEI
    rcall _WAITT          
	ret

; lcd data
wdat:
	CLI
	in ACC,MCUCR       ; turn off ram
	std Z+smcucr,ACC
	andi ACC,0x3F
	out MCUCR,ACC

	out PORTA,ARGL ; place command onto bus
	ldi ARGH,LCDDATAOUT
	out PORTC,ARGH      ; sets RS & R/!W
	sbi PORTC,2    ; start !E strobe
	nop            ; delay .25 uS
	cbi PORTC,2    ; strobe in the data

	ldd ACC,Z+smcucr
	out MCUCR,ACC          ;restore memory
	ldi ARGL,1
	ldi ARGH,0
	SEI
	rcall _WAITT  ; 
	ret

; lcd buffered display

; Switches
; show next song
; mode,volume, tempo, sleep

; FDC - FLOPPY DISK controller
;=================================================================================================
;                                           fdc_init
;=================================================================================================

; initialise the floppy controller by issuing a reset and getting the 
; resulting sense data. Start the motor for scanning to see if a floppy is inserted

initFDC:

	;set the flags for future use
			
	sbr FDCFLAGS,1 << rcr	    ; recalibrate required 	
	sbr FDCFLAGS,1 << needreset 

	rcall fdc_reset             ; reset controller
	
	; display power on Welcome message 
	ldi XL,low(Text1 * 2)	;Make the X reg point at the table
	ldi XH,high(Text1 * 2)	;preparing for the LPM instruction
	ldi ARGL,0x80 ;  ; Entry Mode cursor line 1
	rcall lineout
	ldi XL,low(BLANKL * 2)	;Make the X reg point at the table
	ldi XH,high(BLANKL * 2)	;preparing for the LPM instruction
	ldi ARGL,0xC0 ;  ; Entry Mode cursor line 1
	rcall lineout
	
	rcall start_motor           ; install motor off task and start motor proper
	
	ret

;=================================================================================================
;                                           fdc_reset
;=================================================================================================
fdc_reset:
	; issue a reset to the controller, this is done after a catastrophie like the controller
	;refusing to respond
	
	
	; set up timer 1
	; this will also be used for timeout values
	
	ldi BCC,3 ; use secondary for  loop counter
	mov idx,BCC

RESET3:
	CLI ; make atomic
	ldi ACC,0xFF	;reload value 1mS
	mov TH1,ACC
	ldi ACC,0xC1	;reload value 1mS
	mov TL1,ACC

	out TCNT1H,TH1
	out TCNT1L,TL1

	; set up the output compare, to generate the reset pulse
	inc ACC         ; acc still == TL1
	out OCR1BH,TH1
	out OCR1BL,ACC

	; Port E (Timer 1) setup
	; CAPA:CAPA:CAPB:CAPB:R:R:PWM:PWM
	ldi ACC,0x30   ;00110000
	out TCCR1A,ACC ;set the pin ?

	in  ACC,TIMSK
	ori ACC,0b10100000 ;TOIE1|OCIE1B|TOIE0
	out TIMSK,ACC

	; The timer will now pulse the reset line 

	ldi ARGH,0   ; settle time
	ldi ARGL,5
	SEI
	rcall _WAITT 

	; set some flags
    cbr FDCFLAGS,1 << fdcreset 
	sbr FDCFLAGS,1 << needreset ; when the floppy does not respond we hit it over the head
	                            ; with a reset.

	;point Y register to floppy base address
	ldi YH,FDC_DOR
	ldi YL,0x00
	clr ACC
	st  Y,ACC    ; clear register for software reset
	ldi ARGH,0
	ldi ARGL,1
;	ldi ACC,0xFE ; 1 tick of the clock about 250 uS
;	out TCNT0,ACC
	rcall _WAITT; 
	ldi ACC,0x1C ; this will turn motor on
	st  Y,ACC    ; pull the fdc out of reset, Y should remain stable


	ldi ARGH,high(1000) ;wait 1 second, for motor to settle
	ldi ARGL,low(1000)

	;sbis PIND,PIND3     ; see if interrupt happened when we were not looking    
	sbr FDCFLAGS,1 << fdc_irq 
	rcall _WAITT; 

	cbr FDCFLAGS,1 << needreset ; we reset 
	; test for interrupt timeout 

	dec idx
	breq BADFDC000
	sbic PIND,PB3
	rjmp fdc_sense  ; exit via sense command
	rjmp RESET3     ; tell me three times

BADFDC000:
	ldi XL,low(BADFDC * 2)	;Make the Z reg point at the table
	ldi XH,high(BADFDC * 2)	;preparing for the LPM instruction
	ldi ARGL,0xC0 ;  ; Entry Mode cursor line 2
	rcall lineout
	;CLI ; die gracefully
	rjmp Nothing


;=================================================================================================
;                                           fdc_sense
;=================================================================================================
 
fdc_sense:

    ldi ARGL,FDC_SENSECMD ; test sense command
	rcall fdc_out ;send command

	ldi ARGH,0    ; wait for command to settle
	ldi ARGL,7
	rcall _WAITT

	rcall fdc_results ; get results from controller
	ret

;=================================================================================================
;                                           fdc_out
;=================================================================================================
 
fdc_out:
	push YH
	;send a byte in ARGL to the floppy disc controller
	;try reading the MSR
	ldi YH,FDC_MSR

OUT000:
	ld  ACC,Y 
	cpi ACC,0xD0
	brne PC+2
	rcall fdc_results
	andi ACC,0xC0 ; MASTER ! direction
	cpi ACC,0x80  ; compare to master only
	; test for timeout?
	brne OUT000

	; ready to write
	inc YH ; point Y to FIFO aka FDC_DATA
 	st Y,ARGL
	pop YH
	ret

;=================================================================================================
;                                           fdc_results
;=================================================================================================
fdc_results:
   ;this gets called after a reset, a seek or recalibrate command	
	push ZH
	push ZL
	push YH

	ldi ZH,high(RESULTS)
	ldi ZL,low(RESULTS)

	ldi ACC,8
	mov idx,ACC
	ldi YH,FDC_MSR
RESULTS001:
	ld  ACC,Y 
	andi ACC,0xD0 ; RQM | DIR | BSY
	; test for timeout here ie will watchdog
	cpi ACC,0x80
	breq et
	cpi ACC,0xD0 ; ready to read
	brne RESULTS001
   dec idx
	breq et   ; too many status elements
   
   ; ok to read
	inc YH ; point to FIFO
	ld ARGL,Y ; test sense command STO
	dec YH
	st Z+,ARGL

	;ldi ARGH,0       ; settle before next result
	;ldi ARGL,2
	;rcall _WAITT
	rjmp RESULTS001
et:
	ld  ACC,Y 
	pop YH
	pop ZL
	pop ZH
	ret


;=================================================================================================
;                                           start_motor
;=================================================================================================
start_motor:

	sbrs FDCFLAGS,mtrState
	rjmp ISOFF
	; motor already on, keep it on another 7 seconds
	CLI
	ldi ARGH,high(7000)
	ldi ARGL,low(7000)
	ldi BCC,0x01
	rcall _SETTIME
	SEI
	ret 
ISOFF:
 	sbr FDCFLAGS,1 << mtrState	; motor on	
	ldi YH,FDC_DOR
	ldi ACC,0x1C
	st  Y,ACC    ; clear register for software reset
	ldi ARGH,high(1000)
	ldi ARGL,low(1000)
	rcall _WAITT
	
	ldi XH,high(motor_wait)
	ldi XL,low(motor_wait)
	ldi ARGH,high(6000)
	ldi ARGL,low(6000)
	ldi ACC,01    ;new task #
	rcall _CREAT; 
	ret

;=================================================================================================
;                                           motor_wait
;=================================================================================================
motor_wait:
	; turn motor off
	CLI
	cbr FDCFLAGS,1 << wait_poll
	ldi BCC,0x0C
	ldi YH,FDC_DOR
	st  Y,BCC
 	cbr FDCFLAGS,1 << mtrState	; motor off	
	std Z+task_num,c_tmp
	SEI
	rjmp QSHIFT  ; exit via the interrupt routine

;=================================================================================================
;                                           f_finish
;=================================================================================================
f_finish:

	; on entry Z should point to f-fp

	ldd ACC,Z+f_count         ; load f_count
	ldd BCC,Z+f_count+1  
	cpi ACC,0
	ldi c_tmp,0
	cpc BCC,c_tmp
	brne f_010                ; f_count == 0 -- nothing to transfer
	ldi RESULT,ERR_PARAMETERS    
	ret                       ; return

f_010:	
	rcall start_motor         ; turn motor on, or add time for command to complete

	ldi RESULT,0              ; return result = 0
	
	ldi ACC,low(BASE_SECTOR)
	std Z+fl_sector,ACC       ; track_sector = BASE_SECTOR

f_LOOP:
	; loop for transfer while f_count has value

	ldi ACC, 0             ; clear error retry count
   std Z+ERRORS,ACC
    ; loop for error retry
f_RETRY:

	sbrc FDCFLAGS,needreset
	rcall fdc_reset
	
	ldd ACC,Z+current_spec1
	cpi ACC,0xDF             ; test current_spec1 specl #= 0 for this drive
	breq f_SETRATE
   	
	;send a new specify command to the drive
	ldi ARGL,FDC_SPECIFY ; 
	rcall fdc_out            ; send command
	ldi ARGL,SPEC1           ; only spec defined for the densities we are using 
	std Z+current_spec1,ARGL ; save it so we do not have to send it each time 
	rcall fdc_out            ; send data
	ldi ARGL,0x02            ; normal mode 
	rcall fdc_out     
	
	
	; there are some possibilities here to optomise the transfer rat
	; we need to calculate the expected rate the ATMEL can handle and
	; compare it to what FIFO leval works best
	
	; try running in non burst FIFO moce
	CLI
	ldi ACC,0x55
	ldi YH,FDC_CCR
	st Y,ACC
	st Y,ACC
	ldi ACC,0x05
	st Y,ACC
	dec ACC
	inc YH ;FDC_CCRDATA
	st Y,ACC
	dec YH ; FDC_CCR
	ldi ACC,0xAA
	st Y,ACC
	SEI		        

f_SETRATE:
	
	; set the data rate
	; this is a bit confusing as there is one register for pc_at, but
	; another register with the same definition. well we are not a pc AT ...
	
	
	; FDC_DSR = 0x02 | 0x00 
	ldi ACC,0x02       ; 250 kb for 720 disk
	ldi BCC,0x03       ; test for low density
	ldd ARGL,Z+d
	cp  ARGL,BCC
	breq PC+2
	clr ACC            ; 500 kb for 144 mb disk	
	ldi YH,FDC_DSR
	st  Y,ACC                

	mov ARGL,RESULT
	rcall f_seek;
	; ARGL now contains error code 
	mov RESULT,ARGL

	; set up tranfer array

	ldd  ACC,Z+fl_sector
	cpi ACC,NO_SECTOR         ;f_fp->fl_sector != NO_SECTOR
	breq f_transfer
f_LDSECTR:	
	
	ldi  BCC,low(BASE_SECTOR)
	ldd  c_tmp,Z+f_sectors
	add  BCC,c_tmp
	cp   ACC,BCC              ; f_fp->fl_sector >= BASE_SECTOR 
	brlo f_000
	ldi  ACC,low(BASE_SECTOR)
	std  Z+fl_sector,ACC      ; f_fp->fl_sector = BASE_SECTOR
f_000: 
	subi ACC,low(BASE_SECTOR)  ; f_fp->fl_sector - BASE_SECTOR

	; compute transfer offset 
	ldi ARGL,sizeof_tp
	rcall mpy8u          ; results returned in ARGL,ARGH
	
	ldi YL,low(tp)       ; load in transfer array 
	ldi YH,high(tp)
	add YL,ARGL
	adc YH,ARGH          ; y is now &tp[f_fp->fl_sector - BASE_SECTOR]
	std Z+star_tp,YL
	std Z+star_tp+1,YH       ; save so we do not have to calculate it after the read
	
	ldd ACC,Y+tr_count
	ldd BCC,Y+(tr_count+1)
	cpi ACC,0
	ldi c_tmp,0
	cp BCC,c_tmp
	brne f_CLIPFC ;brge f_CLIPFC        ; exit loop
	ldd ACC,Z+fl_sector 	 
	inc ACC
	std Z+fl_sector,ACC  ; f_fp->fl_sector++
	ldd BCC,Z+f_sectors
	cp ACC,BCC
	breq f_000
	brlo f_000
	ldi RESULT,ERR_PARAMETERS
	rjmp done_trex
		
f_CLIPFC:
	; clip f_count
	; a and b should still contain the tr_count request that remains
	ldd ARGL,Z+f_count    ; load f_count
	ldd ARGH,Z+f_count+1
	
	cp  Acc,ARGL            ; tp->tr_count > fcount
	cpc BCC,ARGH
	brlt f_tran000
	breq f_tran000
	std Y+tr_count,ARGL      ; tp->tr_count = fcount
	std Y+(tr_count+1),ARGH
f_tran000:	
	cpi RESULT,0 ; test for no errors in seek
	breq f_transfer
	rjmp done_ex
f_transfer:
	; transfer data
	
	
	;	clr RESULT
;	rfu before calling read above, call the read ID
;   if the transfer fails, then it will point to the place where we were
;   before the error.

	
	;send a new specify command to the drive
	ldi ARGL,FDC_SPECIFY ; 
	rcall fdc_out            ; send command
	ldi ARGL,SPEC1            ; only spec defined for the densities we are using 
	rcall fdc_out            ; send data
	ldi ARGL,SPEC2           ; special non DMA mode 
	rcall fdc_out             

	;Z should still point to floppy track structure
	push YH
	ldi ACC,0x14
	ldi YH,FDC_DOR
	st  Y,ACC
	pop YH

	ldi ARGL,FDC_READ ; 
	rcall fdc_out 
    
	ldd ARGL,Z+fl_head
	lsl ARGL
	lsl ARGL             ; head << 2 | f_drive

    ; f_drive is always 0 
	rcall fdc_out 
   	
	ldd ARGL,Z+fl_cylinder
	rcall fdc_out 
   
	ldd ARGL,Z+fl_head
	rcall fdc_out 
    
	ldd ARGL,Z+fl_sector
	rcall fdc_out 
    
	ldi ARGL,SECTOR_SIZE_CODE ; sector size code 512 blocs
	rcall fdc_out 
    
;	ldd ARGL,Z+f_sectors ; this may actually be tr_sectors, to get the right status returned
	ldd ARGL,Y+tr_fsectors
	;ldd ARGL,Y+tr_fsectors+1   ; last sector to transfer
	rcall fdc_out 
   
	ldi ARGL,0x2A      ; GAP 720 disk
	ldi BCC,0x03       ; test for low density
	ldd ARGH,Z+d
	cp  ARGH,BCC
	breq PC+2
	ldi ARGL,0x1B      ; GAP for 144 mb disk	
	rcall fdc_out 
   
	ldi ARGL,DTL       
	rcall fdc_out      ; this call will cause the FDC to start the read

read_ex:
	ldd r4,Y+tr_count
	ldd r5,Y+tr_count+1 ; number of bytes to read
	
	
	ldd PAGE,Y+tr_phys+2  ; set page register
	rcall setPage;
	
	ldd XL,Y+tr_phys
	ldd XH,Y+tr_phys+1    ; Z points to memory to transfer into

	add r4,XL
	adc r5,XH
		
	ldi YH,FDC_MSR        ; Y points to FDC
	cbr FDCFLAGS,1 << xfercpl
	

	clr r6
	ldi c_tmp,7 ; max results to xfer
	mov r7,c_tmp
	clr c_tmp

	clr ACC					 ; transfer count	
	clr BCC

	; disable timers and interupts?
	CLI
	out TCCR0,ACC    ; stop all clocks
	out TCCR1B,ACC
	out TIMSK,ACC	;

read_busy:
	ld ARGL,Y             ; read MSR
	sbrs ARGL,5           ; bit 5 is the non DMA polling bit 
	rjmp read_busy       ; this bit is set to 1 during the execution phase of the 

j22_5:
	ld  ARGL,Y 
	sbrc ARGL,5
   mov ARGH,ARGL

	andi ARGL,0xD0 ; RQM | DIR | BSY
	breq exit_ex
	; test for timeout here ie will watchdog
	cpi ARGL,0x80
	breq exit_ex

	cpi ARGL,0xD0 ; ready to read
	brne j22_5
   
   ; ok to read
	inc YH ; point to FIFO
	ld ARGL,Y 
	nop
	dec YH 

;	cpi ARGH,0xF0
;	brne j22_9                ; Seems to get set to D0 when a disk error occurs 
;xxxsbrc FDCFLAGS,xfercpl     ; Status *SHOULD* be at end of buffer. 
;	rjmp j22_7

;j22_9:
;   ldi RESULT,ERR_TRANSFER ; the transfer did no happen
;	cp r7,c_tmp               ; save the status in the status location
;	breq j22_5                ; try using tr_sectors rather than f_sectors as the last
;	st X+,ARGL                ; transfer sectors and see if that gives
;	dec r7                    ; correct info
;	rjmp j22_5

;j22_7:
	cp XL,r4
	cpc XH,r5
	brge j22_5

	st X+,ARGL

j22_6:
	cpi XL,0
	cpc XH,c_tmp          ; test Z for memory page overflow
	brne j22_5
	ldi XH,high(0x8000)   ; new page
	inc PAGE
	rcall setPage         
	cpi PAGE,0
	brne j22_5
	ldi RESULT,ERR_MEMFULL ; cant read more than we have memory to read
	sbr FDCFLAGS,1 << xfercpl	
	rjmp j22_5

exit_ex:
	; the read is now complete

	ldi ACC,0x04 	; div 256 == 64uS
	out TCCR0,ACC
	out TCNT0,TL0
	ldi ACC,0x82	;Enable all timer ints (3-26)
	out TIMSK,ACC	;
	SEI
	
	;send a new specify command to the drive
	ldi ARGL,FDC_SPECIFY ; 
	rcall fdc_out            ; send command
	ldi ARGL,SPEC1           ; only spec defined for the densities we are using 
	rcall fdc_out            ; send data
	ldi ARGL,0x02            ; normal mode 
	rcall fdc_out             
	ldi YH,FDC_DOR
	ldi ACC,0x1C
	st  Y,ACC    ; Guy on phone tec support says to send this
	st  Y,ACC    ; 3 times in case the system does not
	st  Y,ACC    ; respond to the mode change

	; X should contain the number of bytes transfered
	
	; if RESULT == ERR_TRANSFER, don't even try and recover
	
	
	ldi ARGL,FDC_READ_ID ; try reading this before the read command?
	rcall fdc_out        
	clr ARGL
	rcall fdc_out 

	ldi ARGH,0           ; wait for command to settle
	ldi ARGL,1
	rcall _WAITT

	; RESULT set will indicate if we ran out of memory
	
	; now test for other errors that could have happened
	
	rcall fdc_results

	cpi RESULT,ERR_MEMFULL
	breq done_trex            ;  fatal error can not retry
	
	lds ACC,ST1
	andi ACC,0x6F           ; if any of these bits are set it is an error	
	cpi ACC,0
	brne RESBAD	
	;lds ACC,ST2
	;cpi ACC,0
	;brne RESBAD
	
; results are good
	
	;test that we got enough data, the transfered delta should be tracked in io_nbytes
	cp XL,r4
	cpc XH,r5
	breq NEXTSECTR
RESBAD:
	ldi RESULT,ERR_TRANSFER
	; sbr FDCFLAGS,1 << needreset

NEXTSECTR:
	ldi ZL,low(f_fp)        ; restore f_fp pointer
	ldi ZH,high(f_fp)       ; using X for read frees this restore 
	
	lds ACC,ST_SEC
	std Z+fl_sector,ACC     ; set next sector to start reading at ; used for retries

done_ex:
	cpi RESULT,0
	breq done_tr            ; no errors, we are done with the track transfer    

	cpi RESULT,ERR_TIMEOUT
	breq done_trex          ; no point on retry of a timeout

	; if error set up for retry
	ldd ACC,Z+ERRORS
	inc ACC
	std Z+ERRORS,ACC
	cpi ACC,MAX_ERRORS
	brge done_trex          ; too many errors

	; force recalibration if 1/2 the number of errors
	cpi ACC,(MAX_ERRORS/2)
	brge PC+2
	rjmp f_RETRY
	sbr FDCFLAGS,1 << rcr  ; try recalibrating
	clr ACC
	std Z+current_spec1,ACC
	rjmp f_RETRY

done_tr:
    ; dec f_count
	ldd ACC,Z+f_count      ; load f_count
	ldd BCC,Z+f_count+1

	ldd YL,Z+star_tp
	ldd YH,Z+star_tp+1     ; load in transfer array from star pointer
	ldd ARGL,Y+tr_count
	ldd ARGH,Y+tr_count+1  ; number of bytes to read
	
	sub  ACC,ARGL          ; f_count =- tp->tr_count
	sbc BCC,ARGH
	
	std Z+f_count,ACC
	std Z+f_count+1,BCC
	
	cpi ACC,0
	ldi c_tmp,0
	cpc BCC,c_tmp
	breq PC+2              ; f_count == 0
	rjmp f_LOOP
	
done_trex:	
	rcall defuse	
	ret;
	
;=================================================================================================
;                                           f_seek
;=================================================================================================
f_seek:

	sbrc FDCFLAGS,rcr
	rcall recalibrate
		
	; test for need reset (calibration failed )
	cpi ARGL,0
	breq PC+2		; 
	rjmp seek_ex    ; return results are set in ARGL this gets passed up the chain

	; test for cur cylinder == hard cylinder
	sbrs FDCFLAGS,needreset
	rjmp seek000
seek_err:
	ldi ARGL,ERR_SEEK ; cant seek if fdc is sick
	ret

seek000:
	; calculate seek delta	
	ldd ACC,Z+fl_curcyl       ; test and see if we need to seek
	ldd BCC,Z+fl_cylinder     ;
	
	cp  ACC,BCC
	brne seek001
	clr ARGL                 ; return no error, 
	ret
	
seek001:  
	sub ACC,BCC	          ; get seek delta curcyl-cyl

	mov ARGL,ACC            ; save delta for later compare

	andi ACC, 0x80          ; test if went negative
	cpi ACC,0
	breq seek002
	subi ARGL,low(-1)
	inc  ARGL               ;(~seek_delta)+1

seek002: 
	; seek delta is now in argument registers
	; it should  be positive, but may be over 80
	cpi ARGL,80
	brlt seek003
	ldi ARGL,low(80)

seek003:
	cpi ARGL,low(0)
	brne seek004
	ldi ARGL,1

seek004:
	ldi ACC,6
	rcall mpy8U         ; seek delta * 6
	
	subi ARGL,low(-15)  ; seek delta + 15
	
	push ARGH   
	push ARGL    ; save seek delta timeout for later
    
	ldi ARGL,FDC_SEEK ; 
	rcall fdc_out 
	ldd ARGL,Z+fl_head
	lsl ARGL
	lsl ARGL
    ; drive is not used (always 0)
	rcall fdc_out 

   ldd ARGL,Z+fl_cylinder
	rcall fdc_out 

	; prepare for sense 
	CLI ; let the wait enable ints
	pop ARGL
	pop ARGH
	sbr FDCFLAGS,1 << fdc_irq 
	rcall _WAITT

    ldi ARGL,FDC_SENSECMD ; test sense command
	rcall fdc_out ;send command

	ldi ARGH,0    ; wait for command to settle
	ldi ARGL,5
	rcall _WAITT

    rcall fdc_results ; get results from controller

	; test ST0 for valid pattern
	lds ACC,ST0
	andi ACC,ST0_BITS           ; if any of these bits are set it is an error
	cpi ACC,SEEK_ST0	
	breq seek005
	;seek failed
seek006:
	ldi ARGH,0    ; wait for command to settle
	ldi ARGL,15
	rcall _WAITT
	rjmp seek_err
seek005:
	lds ACC,ST1
	ldd ARGL,Z+fl_cylinder      ;
	cp ACC,ARGL
	brne seek006

	ldi ARGH,0    ; wait for command to settle
	ldi ARGL,2
	rcall _WAITT
	
	ldd ARGL,Z+fl_cylinder    
	std Z+fl_curcyl,ARGL   ; set current cylinder to this cylinder
	clr ARGL				     ; clear error return
	clr ARGH
seek_ex:
	ret

;=================================================================================================
;                                           recalibrate
;=================================================================================================
recalibrate:

	rcall start_motor
	
	ldi ARGL,FDC_RECALIBRATE ; 
	rcall fdc_out 
	
	ldi ARGL,0 ; only drive 0
	rcall fdc_out 

   ; prepare for sense 
	ldi ARGL,low(1000)
	ldi ARGH,high(1000)
	sbr FDCFLAGS,1 << fdc_irq 
	rcall _WAITT

	ldi ARGL,FDC_SENSECMD ; test sense command
	rcall fdc_out ;send command

	ldi ARGH,0    ; wait for command to settle
	ldi ARGL,7
	rcall _WAITT

	rcall fdc_results ; get results from controller
	clr ARGL				  
	std Z+fl_curcyl,ARGL       ; force a seek next time

	; test ST0 for valid pattern
	lds ACC,ST0
	andi ACC,ST0_BITS           
	cpi ACC,SEEK_ST0			  ; if any of these bits are set it is an error
	breq rcr005
	;recalibrate failed
rcr006:
	ldi ARGL,low(NO_CYL)				  
	std Z+fl_curcyl,ARGL       ; force a seek next time
	sbr FDCFLAGS,1 << needreset
	ldi ARGL,ERR_RECALIBRATE
	ret
rcr005:
	lds ACC,ST_PCN
	ldi ARGL,0
	cp ACC,ARGL
	brne rcr006

	; recalibrate succeeded 
	ldi ARGH,0    ; wait for command to settle
	ldi ARGL,7
	rcall _WAITT
	
	; clear calibration flag
	cbr FDCFLAGS,1 << rcr
   clr ARGL				  
	clr ARGH
	ret 
	

;=================================================================================================
;                                           set_transfer
;=================================================================================================
set_transfer:

	; point Y to TP
	ldd ACC,Z+block
	ldd BCC,Z+block+1
	ldd ARGL,Z+f_sectors
	clr ARGH
	
	rcall div16u            ; get modulo
	;modulo is in r4:r5
		
	; compute transfer offset 
	ldi ARGL,sizeof_tp
	clr ARGH
	rcall mpy16u            ; results returned in ARGL,ARGH
	
	ldi YL,low(tp)          ; load in transfer array 
	ldi YH,high(tp)
	add YL,ARGL
	adc YH,ARGH             ; y is now &tp[block % f_sectors]
	std Z+star_tp,YL          ; save so we do not have to recalculate
	std Z+star_tp+1,YH
		

	ldd ACC,Z+block
	ldd BCC,Z+block+1        ; from this block
	std Y+tr_block,ACC
	std Y+tr_block+1,BCC

transfer_offset:
	; compute transfer offset 
;	ldi ACC,NR_HEADS     ; always 2
	ldd ARGL,Z+f_sectors
	lsl ARGL             ; NR_HEADS * f_sectors
	clr ARGH
;	rcall mpy8u          ; results returned in ARGL,ARGH

	ldd ACC,Z+block
	ldd BCC,Z+block+1
	rcall div16u         ; result is in A and B , r4 and r5 

	std Z+fl_cylinder,ACC       ; block / (NR_HEADS * f_sectors)
	
	mov ACC,r4				   ; r4:r5 is (block % (NR_HEads * f_sectors))
	clr r5 
	ldd ARGL,Z+f_sectors
	clr ARGH
	
	rcall div16u               ; divide by f_sectors again to get 
	std Z+fl_head,ACC           ; (block % (NR_HEads * fsectors))/fsectors
	;clr BCC 

	; compute next track
	;push ACC ; save head for later computation

	ldd ARGL,Z+fl_cylinder        ; (fl_cylinder * NR_HEADS
	;ldi ACC,NR_HEADS
	;rcall mpy8u                     ; results returned in ARGL,ARGH
	lsl ARGL
	add ACC,ARGL
	;pop XL                         ; + fl_head
	;clr XH                         ; we use X so that we can take advantage
	;add XL,ARGL							 ; of the two byte increment instruction
	;adc XH,ARGH

	adiw ACC,1							 ; + 1)
	;adiw XL,1							 ; + 1)
	;mov ACC,XL
;	mov BCC,XH	                
	ldd ARGL,Z+f_sectors           ; * fsectors
	rcall mpy8u          			 ; results returned in ARGL,ARGH

	std Z+f_nexttrack,ARGL           ;  
	std Z+f_nexttrack+1,ARGH           ; 

  	; if we had more memory error checking would be good here
	
	;
	  
ret

;=================================================================================================
;                                           track_readBlock
;=================================================================================================
track_readBlock:

	; set up transfer parameters
	rcall set_transfer

	ldd ACC,Z+block          ; set safe EOT
	ldd BCC,Z+block+1
	ldd ARGL,Z+f_sectors
	clr ARGH
	
	rcall div16u            ; get modulo
	;modulo is in r4:r5
	; Y should still point to tp

	std Y+tr_fsectors,r4
	std Y+tr_fsectors+1,r5   ; last sector to transfer
	
	
	ldd ACC,Z+f_count            ; load f_count
	ldd BCC,Z+f_count+1

	std Y+tr_count,ACC
	std Y+tr_count+1,BCC

	ldd ACC,Z+target
	ldd BCC,Z+target+1 
	std Y+tr_phys,ACC
	std Y+tr_phys+1,BCC
	ldd ACC,Z+target+2
	std Y+tr_phys+2,ACC
	
	rcall f_finish             ; now read it
	ret

;=================================================================================================
;                                           Defuse
;=================================================================================================
defuse:


	clr ACC
	ldi BCC,MAX_SECTORS
	ldi YL,low(tp)
	ldi YH,high(tp)
	
def000:
	; tr_count is 0 offset
	st Y,ACC
	std Y+1,ACC
	adiw YL,sizeof_tp
	
	dec BCC
	brne def000	
	
	std Z+f_count,ACC
	std Z+f_count+1,ACC
	ret

;=================================================================================================
;                                            XXXXXXX
;=================================================================================================
	
; fdc load FAT
; fdc load directory
; fdc load file
; fdc show next song
; fdc show prior song

; MIDI
; parse header
; play song

setPage:
	push ACC
	swap PAGE
	andi PAGE,0b00110000
	in ACC,PORTD
	andi ACC, 0b11001111
	or PAGE,ACC
	out PORTD,PAGE
	swap PAGE
	andi PAGE,0x03
	pop ACC
	ret



Main:

   ldi    ACC, high(RAMEND)
   out    SPH, ACC
   ldi    ACC, low(RAMEND)
   out    SPL, ACC

	CLI
	ldi	ACC,0x00	;Disable all ints. (3-26)
	out	GIMSK,ACC	;

   WDR
	ldi	ACC,0x1F	
	out	WDTCR,ACC	;load WDTCR with value
	ldi	ACC,0x17	;00010111 Wrute 0 to WDTCR.3 to disable WDT
	out	WDTCR,ACC	;
	
	;port A setup
   ;Low order Mux Addr/Data bus
	ldi	ACC,0xFF;
	out	DDRA,ACC	;Set port A data direction (3-49)
   ldi   ACC,0x00
	out	PORTA,ACC 	

	;port B setup
   ; Switch matrix and LED out
	ldi	ACC,0xF8	
	out	DDRB,ACC	;Set port B data direction (3-49)
   ldi   ACC,0x07
	out	PORTB,ACC 	

	;port C setup
	;High order Addr bus 
	;SRAMCS:SRAMA14:SRAMA13:FDC_CS:LCD_CS:A10:A9:A8 
	ldi ACC,0xFF 
	out   DDRC,ACC
	ldi	ACC,0x00 ;!W
	out	PORTC,ACC	

	;Port D setup
	; !RD:!W:SRAMA17:SRAMA16:FDC_INT:Int0:Rxd:TxMIDI
	ldi	ACC,0xF2	;11110010 
	out	DDRD,ACC	;Set port D data direction
	ldi	ACC,0x04   
	out	PORTD,ACC	
   
	;safe harbor all registers
   ldi ACC,0x00 ;TINT TIMER
   mov r0,ACC   ;Bank0 X/ lpm
   mov r1,ACC   ;Bank0 Y
   mov r2,ACC   ;task time
   mov r3,ACC   ;R0@ bank1 code access
   mov r4,ACC   ;ZL Bank1 code access
   mov r5,ACC   ;ZH Bank1 code access
   mov r6,ACC   ;IR local
   mov r7,ACC   ;VF data
   mov r8,ACC   ;SIGS     ; bit signals
   mov r9,ACC   ;SIGW     ; waiting flags
   mov r10,ACC  ;Bank1 X  ; points to stack table
   mov r11,ACC  ;Bank1 Y  ; address of task in READYQ for task
   mov r12,ACC  ;temp@interrupt
   mov r13,ACC  ;ssreg
   mov r14,ACC  ;newsc
   mov r15,ACC  ;newsc
   
	ldi r16,0x00 ; timer variable
   ldi r17,0x00 ; timer variable
   ldi r18,0x00 ; timer reload count TLH
   ldi r19,0x00 ; timer reload count TL0
   ldi r20,0x00 ; ACC
   ldi r21,0x00 ; BCC
   ldi r22,0x00 ; Arguments high
   ldi r23,0x00 ; Arguments low
   ldi r24,0x00 ;
   ldi r25,0x00 ;
   ldi r26,0x00 ;XL
   ldi r27,0x00 ;XH
   ldi r28,0x00 ;YL
   ldi r29,0x00 ;YH
   ldi r30,0x00 ;ZL
   ldi r31,0x00 ;ZH

; set all SRAM high.
	ldi ACC,0xFF 
	ldi XH,0x00; high(RAMEND)
   ldi XL,0x60;low(RAMEND)  
clrBase:
   st X+,ACC
   cpi R26,low(RAMEND)
   brlo clrBase
   cpi R27,high(RAMEND)
   brlo clrBase

   
;====================================================================
;This section allows ints which are apparently 
;caused by reset, to occur harmlessly.
;Databook 3-21, 3-26

	ldi	ACC,0x00	;
	out	GIMSK,ACC	;Enable int 0
	
	ldi	ACC,0x00	;
	out	TIMSK,ACC	;Disable  all timer ints
	
	SEI			;Global int enable
	nop
	nop
	nop	;Let the ints happen, so they
	nop	;clear out
	nop
	nop
	CLI	;Turn off all ints (3-21) Clear SREG.7
;*******************************************************************
;Set up uart (3-42)
SET_SERIAL:
;disable serial interrupts, disable transmit and receive
	ldi	ACC,0x00
	out	UCR,ACC	;load UCR with all 0's
;determine 8/9 bit communication
	cbi	UCR,CHR9	;8 bit - clear CHR9 bit in UCR
;31.25 Kbaud
LOAD_3125:
	ldi	ACC,0x07 ;for 4MHz crystal
	out	UBRR,ACC

;
;*******************************************************************
;Set EEProm Inactive (3-41)
;
	ldi	ACC,0x00	;00000000
 	out	EECR,ACC	;
;
;*******************************************************************
;
;*******************************************************************
;Set up the MCU Control register (3-29)
;
   ; set for low interrupts,no sleep and external memory
	ldi	ACC,0xCC	 ;11001100 00110000
	out	MCUCR,ACC ;
;
;
;*******************************************************************
;Enable_watchdog:
;*******************************************************************

   WDR
	ldi	ACC,0x0F	;prescaler 011 - 120mS
;	out	WDTCR,ACC


; Disable Comparitor
	sbi ACSR,ACD

; Kernal starts here

	; setup stacks   
   ldi YL,low(SPTS)
   ldi YH,high(SPTH)

	ldi idx,NOOFTASKS       ; init counter

	ldi ACC,low(STKRESERVE+STACKSIZE)	
	ldi BCC,high(STKRESERVE+STACKSIZE)	   

	; point task stack to task #
	mov ARGL,ACC
	mov ARGH,BCC

I2:
	std Y+NOOFTASKS,BCC
	st  Y+,ACC

	adiw ACC,STACKSIZE
	dec idx
	brne I2

	out SPL,ARGL
	out SPH,ARGH

;set-up interrupt timer and inital phases

	ldi ACC,0x04 	; div 256 == 64uS
	out TCCR0,ACC

	
	; set up timer 0 -- this handles the wait timers
	ldi ACC,0xEF	;reload value 1mS
	mov TL0,ACC
	out TCNT0,TL0

	
	; set up timer 1 - used for FDC_RESET as well as clocking MIDI
	ldi ACC,0x03 	; div 64 == 16us
	out TCCR1B,ACC

	ldi ACC,0xFF	;reload value 1mS
	mov TH1,ACC
	ldi ACC,0xC1	;reload value 1mS
	mov TL1,ACC

	ldi ACC,15
	mov BCC,TL1 
	add ACC,BCC

	out TCNT1H,TH1
	out TCNT1L,TL1

	out OCR1BH,TH1
	out OCR1BL,ACC

	sbr FDCFLAGS,1 << fdcreset

	; Port E (Timer 1) setup
	; CAPA:CAPA:CAPB:CAPB:R:R:PWM:PWM
	ldi ACC,0x20   ;00100000
	out TCCR1A,ACC ;clear the OC1B pin ?

	ldi ACC,0x82	;Enable all timer ints (3-26)
	out TIMSK,ACC	;

	ldi ZL,low(f_fp)  ; Z must always point to the low mem globals struct
	ldi ZH,high(f_fp) ; only the lpm instruction uses Z

	clr ACC
	std Z+task_num,ACC ; start with task 0


	SEI				;enable global

	; wait for power stable 
	ldi ARGH,0x00
	ldi ARGL,254
   rcall _WAITT  ; stall for a while
	
	rcall lcdinit ; wake-up the lcd and display hello message. 
	; display welcome message
 	
 	ldi XL,low(Text * 2)	;Make the Z reg point at the table
	ldi XH,high(Text * 2)	;preparing for the LPM instruction
	ldi ARGL,0x80 ;  ; Entry Mode cursor line 1
	rcall lineout
	ldi XL,low(Text1 * 2)	;Make the Z reg point at the table
	ldi XH,high(Text1 * 2)	;preparing for the LPM instruction
	ldi ARGL,0xC0 ;  ; Entry Mode cursor line 2
	rcall lineout


	; test memory
	; Memory is mapped as follows
	; 2000 -> 3FFF  Window 1 
	; 4000 -> 7FFF  Window 2
	; 8000 -> FFFF  Window 3
	; Port D.4 and D.5 are extended memory addresses.
	; becouse of internal register file and sram some pages
	; can not be accessed below 8000
	ldi XL,low(TESTM * 2)	;Make the Z reg point at the table
	ldi XH,high(TESTM * 2)	;preparing for the LPM instruction
	ldi ARGL,0xC0 ;  ; Entry Mode cursor line 1
	rcall lineout

	ldi PAGE,0x00 ; number of pages to test.
	ldi YH,0x80   ; start testing memory here
	ldi YL,0x00  
	ldi ZH,0x80   ; start testing memory here
	ldi ZL,0x00  
	ldi ARGH,0xAA
	ldi ARGL,0x55
	ldi BCC,1

pagetesttop:
	rcall setPage

memtesttop:
	cpi ZH,0
	breq nextpage
	st  Y+,ARGH
	st  Y+,ARGL
	ld  ACC,Z+
	cp  ACC,ARGH
	brne memerr0
	ld  ACC,Z+
	cp  ACC,ARGL
	breq memtesttop
	
memerr0:
	ldi XL,low(BADMEM * 2)	;Make the Z reg point at the table
	ldi XH,high(BADMEM * 2)	;preparing for the LPM instruction
	ldi ARGL,0xC0 ;  ; Entry Mode cursor line 1
	rcall lineout
	rjmp Nothing

nextpage:
	inc PAGE
	cpi PAGE,0x04
	brlo pagetesttop
	
	in ACC,PORTD
	andi ACC, 0b11001111
	out PORTD,ACC
	clr PAGE 

memtestdone:
	cpi BCC,0
	breq fulltestdone
	dec BCC
	ldi YH,0x80  ; start testing memory here
	ldi YL,0x00  
	ldi ZH,0x80  ; start testing memory here
	ldi ZL,0x00  
	ldi ARGH,0x55 ; reverse
	ldi ARGL,0xAA
	rjmp pagetesttop

fulltestdone:
	ldi ZL,low(f_fp)  ; Z must always point to the low mem globals struct
	ldi ZH,high(f_fp) ; only the lpm instruction uses Z

	ldi XL,low(PASMEM * 2)	;Make the Z reg point at the table
	ldi XH,high(PASMEM * 2)	;preparing for the LPM instruction
	ldi ARGL,0xC0           ; Entry Mode cursor line 1
	rcall lineout


	; init FDC this will start the motor and the sleep timer
	rcall initFDC
	; wait for disk

CHKDSK:
;	ldi YH,FDC_DIR
;	ld  ACC,Y 
;	sbrs ACC,diskchange
	rjmp DISCIN
	ldi ARGH,0
	ldi ARGL,5
	rcall _WAITT
	ldi XL,low(NODISC * 2)	; Make the X reg point at the table
	ldi XH,high(NODISC * 2)	; preparing for the LPM instruction
	ldi ARGL,0x80           ; Entry  Mode cursor line 1
	rcall lineout
	ldi XL,low(BLANKL * 2)	; Make the X reg point at the table
	ldi XH,high(BLANKL * 2)	; preparing for the LPM instruction
	ldi ARGL,0xC0           ; Entry Mode cursor line 2
	rcall lineout

DISCOUT:
	ld  ACC,Y 
	sbrs ACC,diskchange
	rjmp DISCIN
	sbrc FDCFLAGS,mtrState  ;Motor is on
	rjmp DISCOUT

	rcall lcdinit           ; force an ungarvked display
	ldi XL,low(NODISC * 2)	; Make the X reg point at the table
	ldi XH,high(NODISC * 2)	; preparing for the LPM instruction
	ldi ARGL,0x80           ; Entry Mode cursor line 1
	rcall lineout
	ldi XL,low(ANYBTN * 2)	; Make the X reg point at the table
	ldi XH,high(ANYBTN * 2)	; preparing for the LPM instruction
	ldi ARGL,0xC0           ; Entry Mode cursor line 2
	rcall lineout

WAITKEY000:
	; wait for a keypress 
	WDR
	; poll the interupt line
	sbic PIND,PIND2
	rjmp WAITKEY000
	
; wake up from sleep 
	sbrs FDCFLAGS,mtrState ;Motor is on
	rcall start_motor
	rjmp CHKDSK

DISCIN:
;	sbrs FDCFLAGS,mtrState ;Motor is on
;	rcall start_motor
	ldi XL,low(BLANKL * 2)	; Make the X reg point at the table
	ldi XH,high(BLANKL * 2)	; preparing for the LPM instruction
	ldi ARGL,0x80           ; Entry Mode cursor line 1
	rcall lineout
	ldi XL,low(PREPLD * 2)	; Make the X reg point at the table
	ldi XH,high(PREPLD * 2)	; preparing for the LPM instruction
	ldi ARGL,0xC0           ; Entry Mode cursor line 2
	rcall lineout

; read special sector to determine density

	; point Z to f_fp
	ldi ZL, low(f_fp)
	ldi ZH, high(f_fp)

	; load some safe values into variables 
	clr ACC
	std Z+current_spec1,ACC

	ldi ACC,low(NO_CYL)      
	std Z+fl_curcyl,ACC 

	ldi ACC,low(NO_SECTOR)     
	std Z+fl_sector,ACC     

	ldi ACC,0x3       ; test 720 first
	std Z+d,ACC

	ldi ACC,low(36)       ; test 720 first
	std Z+block,ACC  ; special test block
	ldi ACC,high(36)       ; test 720 first
	std Z+block+1,ACC ;

;	ldi ACC,low(13)       ; test 720 first
;	std Z+block,ACC  ; special test block
;	ldi ACC,high(13)       ; test 720 first
;	std Z+block+1,ACC ;
	
	ldi ACC,9       ; test 720 first
nextdrive:
	sbr FDCFLAGS,1 << needreset
	std Z+f_sectors,ACC         ; 9 sectors per track

	rcall defuse

	ldi ACC,low(512)  
	std Z+f_count,ACC    ; 1 block
	ldi ACC,high(512)  
	std Z+f_count+1,ACC; 1 block
	
	ldi ACC,low(BOOTSECTOR)
	std Z+target,ACC
	ldi ACC,high(BOOTSECTOR)
	std Z+target+1,ACC
	clr ACC
	std Z+target+2,ACC
	
	rcall track_readBlock;
	
	; test the results for good
	mov ACC,RESULT
	cpi ACC,0
	breq DRIVE000

	ldd ACC,Z+d
	cpi ACC,0x06
	breq DRIVE002	

	; if failed then try 1.44MB
	ldi ACC,0x6        ; drive type for 1.44
	std Z+d,ACC

	ldi ACC,low(17)    ; special test block
	std Z+block,ACC  
	ldi ACC,high(17)   ; test 720 first
	std Z+block+1,ACC   

	ldi ACC,18         ; 1.44mb has 18 sectors per track
	rjmp nextdrive
DRIVE001:

	; if failed display error message on LCD
DRIVE002:
	ldi XL,low(BADDSK * 2)	; Make the X reg point at the table
	ldi XH,high(BADDSK * 2)	; preparing for the LPM instruction
	ldi ARGL,0x80           ; Entry Mode cursor line 1
	rcall lineout
	ldi XL,low(ANYBTN * 2)	; Make the X reg point at the table
	ldi XH,high(ANYBTN * 2)	; preparing for the LPM instruction
	ldi ARGL,0xC0           ; Entry Mode cursor line 2
	rcall lineout

WAITKEY001:
	; wait for a keypress 
	WDR
	; poll the interupt line
	sbic PIND,PIND2
	rjmp WAITKEY001

	rjmp Main ; start over from scratch

DRIVE000:	

	; pianodisk test goes here
	;
	;

;-------------------------------------------------------------------------------	
;                          read dos or YAMAHA format
;-------------------------------------------------------------------------------	


; read the boot and the the FAT as a single transfer

	ldd ACC,Z+f_count        ; test for left over transfers
	ldd BCC,Z+f_count+1
	cpi ACC,0
	ldi c_tmp,0
	cpc BCC,c_tmp
	breq PC+2
	rcall defuse

	ldi ACC,low(0)           ; point to block 0
	std Z+block,ACC  
	ldi ACC,high(0) 
	std Z+block+1,ACC

	ldi ACC,low(0x800)       ; read 3 sectors
	std Z+f_count,ACC      
	ldi ACC,high(0x800)  
	std Z+f_count+1,ACC    
	
	ldi ACC,low(BOOTSECTOR)  ; to the boot partition
	std Z+target,ACC         
	ldi ACC,high(BOOTSECTOR) 
	std Z+target+1,ACC       
	clr ACC                  
	std Z+target+2,ACC       
	                         
	rcall track_readBlock;   ; will force a recalibrate?

	; test the results for good
	mov ACC,RESULT
	cpi ACC,0
	breq PC+2
	rjmp DRIVE002            ; fatal error, let the user try again

;-------------------------------------------------------------------------------	
;                                read the directory
;-------------------------------------------------------------------------------	

; this routine will read in block of data by track


;RootDirSectors = ((BPB_RootEntCnt * 32) + (BPB_BytsPerSec - 1)) / BPB_BytsPerSec
	ldi YL,low(BOOTSECTOR)
	ldi YH,high(BOOTSECTOR)
	
	ldd  ACC,Y+BPB_RootEntCnt
	ldi ARGL,32
	rcall mpy8u

	std Z+nbytes,ARGL    
	std Z+nbytes+1,ARGH
	clr ACC
	std Z+nbytes+1,ARGH
	 
	ldd  ACC,Y+BPB_BytsPerSec
	ldd  BCC,Y+BPB_BytsPerSec+1
	subi ACC,0x01
	sbci BCC,0x00
	
	add ACC,ARGL
	adc BCC,ARGH

	ldd  ARGL,Y+BPB_BytsPerSec
	ldd  ARGH,Y+BPB_BytsPerSec+1
	rcall div16u
	std Z+RootDirSectors,ACC
	std Z+RootDirSectors+1,BCC
	
	; If(BPB_FATSz16 != 0)
       ;FATSz = BPB_FATSz16;
    ;Else
    ;  FATSz = BPB_FATSz32;
	
;FirstDataSector = BPB_ResvdSecCnt + (BPB_NumFATs * FATSz) + RootDirSectors

	ldd  ARGL,Y+BPB_FATSz16
	ldd  ARGH,Y+BPB_FATSz16+1
	
	ldd  r4,Y+BPB_NumFATs
	ldi c_tmp,0
	mov r5,c_tmp
	;ldd  r5,Y+BPB_NumFATs+1
	rcall mpy16u

	ldd  ACC,Y+BPB_ResvdSecCnt
	ldd  BCC,Y+BPB_ResvdSecCnt+1

	add  ACC,ARGL
	adc  BCC,ARGH	
	
	std Z+FirstSectorofRoot,ACC
	std Z+FirstSectorofRoot+1,BCC
	
	std Z+block,ACC         
	std Z+block+1,BCC

	ldd  ARGL,Z+RootDirSectors
	ldd  ARGH,Z+RootDirSectors+1

	add  ACC,ARGL
	adc  BCC,ARGH
	
	std Z+FirstDataSector,ACC
	std Z+FirstDataSector+1,BCC
	
	ldi ACC,low(directory_buffer)  ; store 
	std Z+target,ACC               ;    directory
	ldi ACC,high(directory_buffer) ;           data
	std Z+target+1,ACC             ;               here
	clr ACC
	std Z+target+2,ACC

	;ldi ACC,low(7)                 ; FirstSectorofRoot  
	;std Z+block,ACC         
	;ldi ACC,high(7) 
	;std Z+block+1,ACC

	;ldi ACC,low(0xE00)             ; RootDirectorySectors << 9
	;std Z+nbytes,ACC    
	;ldi BCC,high(0xE00)  
	;std Z+nbytes+1,BCC 

WNBYTES:
	ldd ACC,Z+nbytes
	ldd BCC,Z+nbytes+1
	cpi ACC,0
	ldi c_tmp,0
	cpc BCC,c_tmp
	brne PC+2
	rjmp RDIR000                  ; while nbytes

	rcall set_transfer            ; set transfer params

	ldd ARGL,Z+nbytes
	ldd ARGH,Z+nbytes+1
	std Z+f_count,ARGL    
	std Z+f_count+1,ARGH           ; f_count = nbytes;

	; compute transfer offset  f_count >> 9
	; since 9 is 1 right shift more than 8 we swap the bytes then do a shift
	mov ACC,ARGH                  ; this is equivelent of fcount >> 8
	lsr ACC                       ; this is lsr >> 1
	clr BCC                       ; & with 00 FF fcount is limited to 1 track
	                              ; this works out to 18 << 9 or 9 K , this will
											; still fit into a short

	ldd XL,Z+block
	ldd XH,Z+block+ 1
	add ACC,XL
	adc BCC,XH                    ; + block

	ldd r4,Z+f_nexttrack
	ldd r5,Z+f_nexttrack+1

	cp  ACC,r4                    ; > f_nexttrack
   cpc BCC,r5
	brlt RDIR001
	breq RDIR001

	;f_nexttrack is still in r4:r5 , block is still in X
   sub r4,XL
	sbc r5,XH                    ; f_nexttrack - block 

   ; f_nexttrack - block << 9
	mov ARGH,r4                   ; swap bytes to << 8
	lsl ARGH                      ; << 1       
	clr ARGL
		
	std Z+f_count,ARGL            ; f_count =  (f_nexttrack - block) << 9
	std Z+f_count+1,ARGH 

RDIR001:

	ldd ACC,Z+nbytes
	ldd BCC,Z+nbytes+1

	sub ACC,ARGL                 ; f_count is still in ARG from pipeline
   sbc BCC,ARGH                 ; nbytes -= f_count

	std Z+nbytes,ACC
	std Z+nbytes+1,BCC

	cpi ARGL,low(0x1200)
	ldi c_tmp,high(0x1200)
	cpc ARGH,c_tmp
	brlo RDIR003                 ; f_count < 0x1200 -> should be f_sectors << 9
	rjmp RDIR005

RDIR003:
	ldd ACC,Z+block
	ldd BCC,Z+block+1
	ldd ARGL,Z+f_sectors
	clr ARGH	
	rcall div16u                ; get modulo in r4:r5

	ldi ACC,0
	cp r4,ACC
	ldi c_tmp,0
	cpc r5,c_tmp                ; !(block % f_sectors)
   brne RDIR005

	ldd ACC,Z+f_count     
	ldd BCC,Z+f_count+1
	                            ; compute transfer offset  f_count >> 9
	mov ACC,BCC                 ; >> 8
	lsr ACC                     ; >> 1
	clr BCC
		
	std Z+tr_fsectors,ACC
	std Z+tr_fsectors+1,BCC     ; tr_fsectors = f_count >> 9

RDIR005:

	ldd ARGL,Z+f_count     
	ldd ARGH,Z+f_count+1
	std Y+tr_count, ARGL
	std Y+tr_count+1,ARGH         ; transfer this many bytes

	ldd ACC,Z+target        
	ldd BCC,Z+target+1 
	std Y+tr_phys,ACC
	std Y+tr_phys+1,BCC
	ldd PAGE,Z+target+2
	std Y+tr_phys+2,PAGE          ; to this location in memory

	add ACC,ARGL
	adc BCC,ARGH
	ldi c_tmp,0
	adc PAGE,c_tmp                ; will wrap the memory
 
	andi PAGE,0x01                ; we only have 17 bits
	mov  c_tmp,BCC                ; prepare to shift
	rol  c_tmp                     ; shift bit 15 into carry
	rol  PAGE                     ; shift bit 15 and 16 into page format
	andi PAGE,0x03             
	ori  BCC,0x80                 ; memory is now in page format

	std Z+target,ACC
	std Z+target+1,BCC
	std Z+target+2,PAGE           ; target += f_count


	rcall f_finish                ; read the track

	; test the results for good
	mov ACC,RESULT
	cpi ACC,0
	breq PC+2
	rjmp DRIVE002                 ; fatal error, let the user try again

	ldd ACC,Z+f_nexttrack
	ldd BCC,Z+f_nexttrack+1

	std Z+block,ACC
	std Z+block+1,BCC             ; block = f_nexttrack
   
	;rjmp WNBYTES                  ; while number of bytes

RDIR000:

	; display the first song title


	ldi YL,low(directory_buffer)
	ldi YH,high(directory_buffer)

	;std Z+FFlags,FDCFLAGS  ; flags are shared with MIDI

	clr FFLAGS           ; start with a single mode, nothing pending, no extended dir

	; start up the UART    ; send some controller messages just in case?

	rcall lcdinit          ; lcd is hosed again? 

	ldi ARGL,0x80          ; prepare the display
	rcall wcmd


	; test here for PD if flash ram permits -- PD will use a shadow of the following
	; as it has it's own wierdness
	
	; pre scan the directory to count possible songs as well as see if we have
	; the extended YAMAHA directory
	; this will also allow us to recover any extra dir sectors not used as part of the
	; TPA
	; if PIANODIR set flag EXT_DIR
	
	

RDIRRTOP:
	ldd ACC,Y+DIR_Name  ; get the first byte of the entry
	cpi ACC,0           ; last entry start over
	brne PC+2
	rjmp Main

	cpi ACC,0xE5        ; Deleted file marker
	brne PC+2
	rjmp NEXTE

	; test here for magic Yamaha rom songs


	ldd ACC,Y+DIR_Attr  ; this will tell us what the file is
	andi ACC,0x0F       ; marker for a long file name
	cpi ACC,0x0F        
	breq PC+2
	rjmp SHORTE
	
	; file is a long name
LONGE:
	; search for short name
	ldd ACC,Y+ORD
	mov BCC,ACC
	andi BCC,0x40
	;cpi  BCC,0x40
	brne PC+2
	rjmp NEXTE          ; wait for it
	mov BCC,ACC
	andi ACC,0x3F
	ldi ARGL,32
	ldi ARGH,0
	rcall mpy8u

	add YL,ARGL
	adc YH,ARGH

	ldd ACC,Y+DIR_Attr  ; this will tell us what the file is
	andi ACC,0x1E       ; system file
	breq PC+2
	rjmp NEXTE  
	        
	rcall PREPMIDI      ; read file and determine playback type
	cpi RESULT,0
	brne PC+2
	rjmp NEXTE

	; now we are ready to display the name

	mov XL,YL
	mov XH,YH
LNAMEOUT:
	ldi BCC,11          ; 10 chars in the first field

	sbiw XL,32          ; point back to the file
	ld RESULT,X+

	rcall lcd_out

	adiw XL,Name2-Attr	
	ldi BCC,13          ;split the secont fild so that we can test
	rcall lcd_out      ; 4 of 12
	
	adiw XL,Name3-FstClsLO	
	ldi BCC,5          ; 4 chars in the last field
	rcall lcd_out

	sbiw XL,32          ; point back to the file
	;was this the last entry?
	andi RESULT,0x40    ; save ord for later test
	breq LNAMEOUT
	rjmp PREPFILE


SHORTE:
	ldd ACC,Y+DIR_Attr  ; this will tell us what the file is
	andi ACC,0x1E       ; if any of these bits are set then do not
	breq PC+2          ; show the filename
	rjmp NEXTE

	rcall PREPMIDI
	cpi RESULT,4
	brlo PC+2
	rjmp NEXTE
	
	ldd ACC,Y+DIR_Name
	ldi BCC,9           ; 8 chars for dos
	mov XL,YL
	mov XH,YH
LDNAME8:
	rcall lcd_out

PREPFILE:
	; here we can show additional stats about the file, probably show
	; song: n of ntotal nnnnn bytes 
	
	sbr FDCFLAGS,1 << playall
	rcall play_song             ; creates a new task for MIDI playback
WAITPLAY:
	WDR
	; here is where we can process button options
	; Z should still be pointing to the globals
	; Y points to the dos directory entry
	
	;ldd ACC,Z+LAST_KEYPRESS
	;ldd BCC,Z+CURT_KEYPRESS
	cp ACC,BCC
	breq WAITPLAY               ; no new keypress has been detected
	; switch keypress
	
	; case pause
	; case stop
	; if the MIDI is playing
	;   cancel tasks at the next completed transfer
	;   send the notes off, properly we should have a track of what it on and what is off
	;                       the notes should be shut off one by one, for now we will use the
	;                       same notes and controllers off.
	; ; stop resets the playback pointer pause does not
	; ; suspend the timer that is ticking MIDI ticks
	; breq WAITPLAY
	
	; case Play
	sbr FDCFLAGS,1 << playall   ; Fdc flag register is shared with the midi flag register
	rcall play_song             ; this will create a new task for the midi
	; case single
	cbr FDCFLAGS,1 << playall
	rcall play_song
	
	; case enter
	   ; increment the mode
	; the modes are ...
	; none, timer ticks seconds and B:B:T
	; volume -- the volume controll leval is displayed on the front panel 0 to 127
	; tempo  -- this is a tricky one, display ticks per second.
	; karioke -- we need to parse these anyway, so they are fun to display
	
	; up down left and right
	; if not playing left and right add or subtract 64 or 32 to Y and jump the the next/prior entry
	; actually becouse of long names and invalid files, we should store the prior file offset
	; for these backwards jumps
	
	; when playing up and down queue a volume control message and left and right queue a tempo msg
	
	; when stopped or when paused volume/tempo commands are not queued but set immediatally 
	
	; midi playback is interrupt driven, so this loop will be interrupted by timeouts and waits
	; called time to next event.	
	
	rjmp WAITPLAY

NEXTE:
	; inc the directory entry
	adiw YL,32
	; compare against root directory count
	
	ldi ARGL,low(6*32)
	ldi ARGH,high(6*32) ; temp for test

	cp YL,ARGL
	cpc YH,ARGH
	brge PC+2
	rjmp RDIRRTOP

	; did the user wake us up with play or with single?

; wait for a keypress
Nothing:
	WDR
   rjmp Nothing


PREPMIDI:
	; read first allocation unit , check and see if it is MIDI,PD or ESEQ
	
;	ldi ARGL,low(8000)
;	ldi ARGH,high(8000)
;	rcall _WAITT
		
	; put nbytes into registers
	ldd ACC,Y+DIR_FileSize     ; bits 0 to 7 of file size
	ldd BCC,Y+DIR_FileSize+1   ; bits 8 to 15 of file size
	ldd ARGL,Y+DIR_FileSize+2  ; bits 16 to 23 of file size
	ldd ARGH,Y+DIR_FileSize+3  ; bits 24 to 31 of file size
	
	mov c_tmp,ARGL
	andi c_tmp,0xFE            ; mask for file to big, if any of these bits are set file is too big
	breq PREPFIRST
PREPFULL:
	ldi RESULT,ERR_MEMFULL     ; ignore the file and do not display it. if extra flash this would
	ret                        ; be a good thing to ask the user to play anyway or to figure out
PREPFIRST:
	andi ARGH,0xFF             ; how to access the floppy during playback.
	brne PREPFULL 
	
	andi ARGL,0x01             ; keep in 17 bit format
	
	mov c_tmp,ACC
	andi c_tmp,0xFF            ; must have a multiple of 512 bytes to read
	;breq SETFLSZ
	; nbytes is in accumulator and argL
	;mov c_tmp,ACC
	                    ; & 0x1FF00
	; nbytes += BPB_BytsPerSec * BPB_SecPerClus

	;ldd  ACC,Y+BPB_BytsPerSec
	;ldd  BCC,Y+BPB_BytsPerSec+1

	;ldd ARGL,Y+BPB_SecPerClus
	;ldi ARGH,0
	;rcall mpy16u
		
	
	; if nbytes >> 8 & 0x1
	;   nbytes += BPB_BytsPerSec/2 -- make even

SETFLSZ:
	std Z+star_dir,YL
	std Z+star_dir+1,YH         ; save the pointer to the directory entry 

	std Z+nbytes,ACC           
	std Z+nbytes+1,BCC
	std Z+nbytes+2,ARGL

	std Z+Mf_toberead,ACC           
	std Z+Mf_toberead+1,BCC
	std Z+Mf_toberead+2,ARGL
	
	std Z+file_size,ACC           
	std Z+file_size+1,BCC
	std Z+file_size+2,ARGL
	
	clr FFLAGS                ; clear file loading flags
	std Z+f_count,FFLAGS      ; f_count = 0
	std Z+f_count+1,FFLAGS
	std Z+f_count+2,FFLAGS
	
	rcall defuse
	
	; if disk_id == PDS_DISK
	;    block = address from directory
	; else 
	;    
	ldd YL,Z+star_dir
	ldd YH,Z+star_dir+1         ; save the pointer to the directory entry 
	
	ldd ACC,Y+DIR_FstClusLO
	ldd BCC,Y+DIR_FstClusHI
	std Z+cluster,ACC
	std Z+cluster+1,BCC
	

	ldi XL,low(TPA)             ; X generally points to memory 
	ldi XH,high(TPA)
	clr PAGE
	std Z+target,XL
	std Z+target+1,XH
	std Z+target+2,PAGE

	std Z+playpos,XL
	andi XH,0x7F         ; temp fix for page shift
	std Z+playpos+1,XH
	std Z+playpos+2,PAGE

	std Z+star_dir,YL
	std Z+star_dir+1,YH         ; save the pointer to the directory entry 
	
	rcall track_sector1         ; preload the first track of the disk
	                            ; this code is re entrant as it will be re-entered
	                            ; later after the user decides what to do
	
	; parse the MIDI header
	; restore the pointer to the directory entry
	ldd YL,Z+star_dir
	ldd YH,Z+star_dir+1         ; save the pointer to the directory entry 
	
	ldi XL,low(TPA)             ; X generally points to memory 
	ldi XH,high(TPA)
	clr PAGE
	rcall setpage               ; always start with page 0

	ldi ACC,0x84;               ; set standard MIDI "safe" values
	std Z+division,ACC          
	ldi ACC,0x03;
	std Z+division+1,ACC
	
	ldi ACC,0x20;               ; this tempo aproximates 120 BPM
	std Z+tempo,ACC
	ldi ACC,0xA1;
	std Z+tempo+1,ACC
	ldi ACC,0x07;
	std Z+tempo+1,ACC
	
	ser RESULT                 ; results will become format, it is also the return value of the function	
	
	rcall Mf_getc 				; the magic marker
	cpi ARGL,'M'
	brne NOTMIDI001
	rcall Mf_getc
	cpi ARGL,'T'
	brne NOTMIDI001
	rcall Mf_getc
	cpi ARGL,'h'
	brne NOTMIDI001
	rcall Mf_getc
	cpi ARGL,'d'
	brne PC+2
	rjmp ISMIDI
	
NOTMIDI001:

	ldi XL,low(TPA)             ; X generally points to memory 
	ldi XH,high(TPA)
	clr PAGE
	std Z+target,XL             ; reset file pointers
	std Z+target+1,XH
	std Z+target+2,PAGE

	std Z+playpos,XL            
	std Z+playpos+1,XH
	std Z+playpos+2,PAGE
	
	ldd ACC,Z+file_size           
	ldd BCC, Z+file_size+1
	ldd ARGL,Z+file_size+2	

	std Z+Mf_toberead,ACC           
	std Z+Mf_toberead+1,BCC
	std Z+Mf_toberead+2,ARGL   ; then try another file type
	
	rcall Mf_getc
	cpi ARGL,0xFE
	breq PC+2
	rjmp NOTMIDI002
	
	; now test for ESEQ
	
	rcall Mf_getc
	mov ARGH,ARGL
	rcall Mf_getc	
	clr c_tmp
	cp ARGL,c_tmp
	cpc ARGH,c_tmp
	breq PC+2
	rjmp NOTMIDI002A
	
	ldi BCC,0x03
ESEQ000:
	rcall Mf_getc 				
	dec BCC
	brne ESEQ000                ; skip 4	
	
	rcall Mf_getc 				; the magic marker
	cpi ARGL,'C'
	brne NOTMIDI102
	rcall Mf_getc
	cpi ARGL,'O'
	brne NOTMIDI102
	rcall Mf_getc
	cpi ARGL,'M'
	brne NOTMIDI102
	rcall Mf_getc
	cpi ARGL,'-'
	brne NOTMIDI102
	rcall Mf_getc 				
	cpi ARGL,'E'
	brne NOTMIDI102
	rcall Mf_getc
	cpi ARGL,'S'
	brne NOTMIDI102
	rcall Mf_getc
	cpi ARGL,'E'
	brne NOTMIDI102
	rcall Mf_getc
	cpi ARGL,'Q'
	breq PC+2
NOTMIDI102:
	rjmp NOTMIDI002
	 
	ldi RESULT,0x03             ; we have an eseq file 
	std Z+format,RESULT
	
	ldi BCC,0x0C
ESEQ001:
	rcall Mf_getc 				; 0x0F through 0x1A
	dec BCC
	brne ESEQ001                ; skip  more of the header	
    ;should be at the 0x1B
    
	;rcall Mf_getc               ; all this is to get the number of bytes to read set proper
	;std Z+skip_size, ARGL    
	;rcall Mf_getc
	;std Z+skip_size+1,ARGL    
	;rcall Mf_getc
	;std Z+skip_size+2,ARGL    
	;rcall Mf_getc
	;std Z+skip_size+3,ARGL    
	 
	rcall Mf_getc
	mov r4,ARGL
	rcall Mf_getc
	mov r5,ARGL
	rcall Mf_getc
	mov r6, ARGL    ; temp
	rcall Mf_getc
	mov r7,ARGL
	
;	ldd ACC,Z+skip_size
;	ldd BCC,Z+skip_size
;	ldd ARGL,Z+skip_size
;	ldd ARGH,Z+skip_size
	
	clr c_tmp
	ldi PAGE,8                ; skip this many extra header bytes 
	
	add r4,PAGE
	adc r5,c_tmp
	adc r6,c_tmp
	adc r7,c_tmp

	add ACC ,r4
	adc BCC ,r5
	adc ARGL,r6
	adc ARGH,r7

	std Z+Mf_toberead  ,ACC    
	std Z+Mf_toberead+1,BCC 
	andi ARGL,0x01            ; mask anyway this is a typecast conversion  
	std Z+Mf_toberead+2,ARGL  ; there is not much we can do if the file is too big
	
;	ldi BCC,0x03
;ESEQ000:
;	rcall Mf_getc 				
;	dec BCC
;	brne ESEQ000              ; one of these is abs tempo, no doc on how it works 	
    
	ldd PAGE,Z+Mf_toberead+2 
	ldi c_tmp,0
	ldi ARGH,1
	clc
	ror PAGE                     ; this will shift A16 into carry
ESEQ003:	
	rcall Mf_getc  
	;sbc ARGH 
	;sbc c_tmp 
	;sbc c_tmp                 ; 17 bit decrement
	;brne ESEQ003
	
	; file should be positioned to play
	 
NOTMIDI002:
    WDR
    ; this scan usually indicates a PD format
	rcall Mf_getc  
    cpi ARGL,0xFF
    breq NOTMIDI002
    
    ser RESULT
    cpi ARGL,0xFE
    brne BADFMT
NOTMIDI002A:    
	ldi RESULT,0x04
	rjmp COMMONPLAYBK

ISMIDI:

	; make this the usual subr  read32bit
	rcall Mf_getc               
;	std Z+Mf_toberead,+3 ARGL ; better not    
	rcall Mf_getc
	std Z+Mf_toberead+2,ARGL    
	rcall Mf_getc
	std Z+Mf_toberead+1,ARGL    
	rcall Mf_getc
	std Z+Mf_toberead  ,ARGL    
	
	rcall Mf_getc ;  hi byte of format, this is ignored
	rcall Mf_getc
	std Z+format,ARGL
	mov RESULT,ARGL
	
	rcall Mf_getc ;  hi byte of format, this is ignored
	rcall Mf_getc
	std Z+ntrks,ARGL
	
;	cpi RESULT,0 
;	brne COMMONPLAYBK
	
;	cpi ARGL,0x01
;	breq SMF0

SMF0: 
	rcall Mf_getc
	std Z+division+1,ARGL    
	rcall Mf_getc
	std Z+division  ,ARGL 
	
	; flush any extra in case the length is not 6 ?
	
	; now prep the track   
	rcall Mf_getc 				; the magic marker
	cpi ARGL,'M'
	brne BADFMT
	rcall Mf_getc
	cpi ARGL,'T'
	brne BADFMT
	rcall Mf_getc
	cpi ARGL,'r'
	brne BADFMT
	rcall Mf_getc
	cpi ARGL,'k'
	brne BADFMT

	; make this the usual subr  read32bit
	rcall Mf_getc               
;	std Z+Mf_toberead,+3 ARGL ; better not    
	rcall Mf_getc
	std Z+Mf_toberead+2,ARGL    
	rcall Mf_getc
	std Z+Mf_toberead+1,ARGL    
	rcall Mf_getc
	std Z+Mf_toberead  ,ARGL    



	rjmp COMMONPLAYBK

BADFMT:
	ser RESULT    ; only one track supported
	ret 
	;rcall read32bit           ; this will store the bytes to read

COMMONPLAYBK:

	std Z+format,RESULT
	
	; test format for 3      ; yamaha
	
	; test format for 4      ; pd

	; need to lookup the old version as I liked the vol and tempo better
	
	; setup but do not enable timers and tempo
	
	; one timer ticks miliseconds , the other MIDI ticks
	
	; tempo defaults to 120BPM volume controller is set initially at 64
	
	; enter these into the MIDI queue
	
	ret

;====================================================================================================
;										 track_sector1
;====================================================================================================
track_sector1:
				
	ldi YL,low(BOOTSECTOR)
	ldi YH,high(BOOTSECTOR)
			
	; on entry accumulator points to the decoded cluster
	
	sbiw ACC,2                             ; (cluster - 2 
	
	mov ARGL,ACC
	mov ARGH,BCC

	ldd r4,Y+BPB_SecPerClus
	clr r5
	rcall mpy16u                           ; * BPB_SecPerClus 
	
	ldd ACC,Z+FirstDataSector
	ldd BCC,Z+FirstDataSector+1            ; + FirstDataSector

	add ACC,ARGL
	adc BCC,ARGH	
	std Z+BLOCK,ACC
	std Z+BLOCK+1,BCC                     ; block = (cluster - 2 ) * BPB_SecPerClus + FirstDataSector
	
	ldd ARGL,Z+f_sectors
	clr ARGH
	rcall div16u
	
	std Z+s_count,r4                       ; scount = block % f_sectors	
	std Z+s_count+1,r5                     
	
	rcall transfer_offset                  ; calc cyl head and nexttrack
	; test cylinder for overflow ?

TOP007LOOP:	
	; for(c_count = 0; c_count < BPB_SecPerClus;c_count++)

	clr ACC
	std Z+c_count,ACC
	
TOPCCOUNTLOOP:	
	ldi YL,low(BOOTSECTOR)
	ldi YH,high(BOOTSECTOR)
    ldd BCC,Y+BPB_SecPerClus	
	cp ACC,BCC
	brlo PC+2
	rjmp CCOUNTEX
	

	ldd ACC,Z+f_count                     ; f_count is in offset from 0 format 
	ldd BCC,Z+f_count+1 
	ldd PAGE,Z+f_count+2 
	
	ldi  c_tmp,0
	cp   ACC,c_tmp
	cpc  BCC,c_tmp
	cpc  PAGE,c_tmp
	brne PC+2                             ; f_count > 0 is unsigned ?
	rjmp track_sectorN
	
	rcall ck_lastcluster                  ; may also check for PD disk
	cpi RESULT,0
	brne CCOUNT001                        

	ldd  ACC,Z+f_nexttrack
	ldd  BCC,Z+f_nexttrack+1
	sbiw ACC,1                            ; f_nextrack -1 

	ldd ARGL,Z+f_sectors
	clr ARGH
	rcall div16u                          ; % f_sectors
	
	ldd ACC,Z+s_count
	ldd BCC,Z+s_count+1
	
	cp  ACC,r4
	cpc BCC,r5
	brlo track_sectorN
CCOUNT001:
	rcall f_finish
	; this is where we return to tell wheather or not we are playable
CCOUNTNEXT:
	ret
	
;====================================================================================================
;										 track_sectorN
;====================================================================================================
track_sectorN:
	rcall ck_lastcluster          ; (cluster <= 0xFFFE ) && (cluster >= 0x0FF8)
	cpi RESULT,0
	breq PC+2
	rjmp CCOUNTEX               ; break	

	ldd ARGL,Z+Block
	ldd ARGH,Z+Block+1
	
	ldd ACC,Z+f_count             ; f_count is in offset from 0 format 
	ldd BCC,Z+f_count+1 
	ldd PAGE,Z+f_count+2 

	ldi  c_tmp,0
	cp   ACC,c_tmp
	cpc  BCC,c_tmp
	cpc  PAGE,c_tmp
	breq PC+2                     ; if f_count == 0  
	rjmp CCOUNT005
	; block >= f_next track is left undone
	rcall transfer_offset                  ; calc cyl head and nexttrack

CCOUNT005:
							 ;user_phys = nbytes;	
	ldd ACC,Z+nbytes
	std Z+user_phys,ACC
	ldd BCC,Z+nbytes+1
	std Z+user_phys+1,BCC
	ldd PAGE,Z+nbytes+2
	std Z+user_phys+1,PAGE   ; user_phys = nbytes	
        
	;if (block + (user_phys >> SECTOR_SHIFT) > f_nexttrack)

	; shift target by 9  ; we could divide by 512,but our divide is only 16 bits and we have 17
	; so we  ...
	mov ARGL,BCC         ; this does a >> 8
	mov ARGH,PAGE
	clc                  ; we are going to rotate and do not want to introduce a bit
	ror ARGH             ; this shifts addr 16 into carry
	ror ARGL             ; this should do the >> 1 to complete the divide


	ldd ACC,Z+Block
	ldd BCC,Z+Block+1    ; get the block addr
	add ARGL,ACC
	adc ARGH,BCC         ; add it to user_phys
	
	ldd r4,Z+f_nexttrack   
	ldd r5,Z+f_nexttrack+1 
	
	cp  ARGL,r4
	cpc ARGH,r5          ; address > next track
	brlo CCOUNT006
	breq CCOUNT006
          ;user_phys = (f_nexttrack - block) << SECTOR_SHIFT;
	
	; can not span reads across tracks
	; block is in ACC, f_nexttrack is in r4:r5
	sub r4,ACC
	sbc r5,BCC
	; now we shift it by 9
	mov PAGE,r5
	mov ARGH,r4
	clr ARGL                 ; << 8
	clc                      ; keep any partials from sneaking into address
	rol ARGH
	rol PAGE                 ; << 1
	andi PAGE,0x01           ; keep the upper address lines clear

	std Z+user_phys,ARGL        ; let the transfer convert the logical address into a physical one
	std Z+user_phys+1,ARGH      ; let the transfer convert the logical address into a physical one
	std Z+user_phys+2,PAGE      ; let the transfer convert the logical address into a physical one

CCOUNT006:
	
	; now we can actually set up the transfer parameters that will queue the bytes from the disk
	;
	; s_count is the start of the track
	; e_count is the end of the track
	; f_count is the number of bytes between s_count and e_count
	;
	; files always start on an even cluster
	;
;/* Store the I/O parameters in the ftrans slots for the sectors to
;* read.  The first slot specifies all sectors, the ones following
;* it each specify one sector less.  This allows I/O to be started
;* in the middle of a block.
;*/
;        nbytes -= bootSector.BPB_BytsPerSec;
	ldd ACC,Z+nbytes
	ldd BCC,Z+nbytes+1           ; the ever so common block mod sectors
	ldd ARGL,Y+BPB_BytsPerSec
	ldd ARGH,Y+BPB_BytsPerSec+1
	sub ACC,ARGL
	sbc BCC,ARGH
	std Z+nbytes,ACC
 	std Z+nbytes+1,BCC

;        f_count += bootSector.BPB_BytsPerSec;  /* transfers can not span 16K */
	ldd ACC,Z+f_count
	ldd BCC,Z+f_count+1           ; the ever so common block mod sectors
	ldd ARGL,Y+BPB_BytsPerSec
	ldd ARGH,Y+BPB_BytsPerSec+1
	add ACC,ARGL
	adc BCC,ARGH
	std Z+f_count,ACC
 	std Z+f_count+1,BCC




;	    s_count = block % f_sectors;
	ldd ACC,Z+block
	ldd BCC,Z+block+1           ; the ever so common block mod sectors
	ldd ARGL,Z+f_sectors
	clr ARGH	
	rcall div16u                ; get modulo in r4:r5
	
	std Z+s_count,r4
	std Z+s_count+1,r5
	
	; compute transfer offset 
	ldi ARGL,sizeof_tp
	clr ARGH
	rcall mpy16u               ; results returned in ARGL,ARGH

	ldi YL,low(tp)
	ldi YH,high(tp)
	add YL,ARGL
	adc YH,ARGH                ; y is now &tp[block % f_sectors]
	std Z+star_tp,YL           ; save so we do not have to recalculate
	std Z+star_tp+1,YH

	ldd ACC,Z+user_phys        ; user phys is typecast here to a short
	ldd BCC,Z+user_phys+1 
	std Y+tr_count,ACC
	std Y+tr_count+1,BCC
;	ldd ACC,Z+user_phys+2
;	std Y+tr_count+2,ACC       ; better not, thier should never be more than 2 * 18 * 512 bytes in a transfer
	
;    	tp->tr_fsectors = ((f_nexttrack-1)% f_sectors)+1;

	ldd ACC,Z+f_nexttrack
	ldd BCC,Z+f_nexttrack+1
	sbiw ACC,1

	ldd ARGL,Z+f_sectors
	clr ARGH	
	rcall div16u               ; get modulo in r4:r5

	mov ACC,r4
	mov BCC,r5
	
	adiw ACC,1	
	std Y+tr_fsectors,ACC
	std Y+tr_fsectors+1,BCC    ; should this be passed onto the read so as to give correct counts on return?

	ldd ACC,Z+nbytes           ; in the old system this was used
	ldd BCC,Z+nbytes+1         ; to return errors
	std Y+io_nbytes,ACC
	std Y+io_nbytes+1,BCC

	ldd ACC,Z+block
	ldd BCC,Z+block+1           
	std Y+tr_block,ACC
	std Y+tr_block+1,BCC
	
	ldd ACC, Z+target        ; target will get incremented AFTER the transfer params are set
	ldd BCC, Z+target+1 
	ldd PAGE,Z+target+2
	std Y+tr_phys,  ACC
	std Y+tr_phys+1,BCC
	std Y+tr_phys+2,PAGE       
	
	ldi ARGL,low(512)
	ldi ARGH,high(512)
	clr c_tmp 
	add ACC,ARGL
	adc BCC,ARGH
	adc PAGE,c_tmp
	std Z+target,ACC
	std Z+target+1,BCC
	std Z+target+2,PAGE

	; boundry condition? memory is full???
	
	; ok lets define user_phys after all
	
	ldd ACC, Z+user_phys
	ldd BCC, Z+user_phys+1
	ldd PAGE,Z+user_phys+2
	
	ldi ARGL,low(512)
	ldi ARGH,high(512)
	clr c_tmp 
	sub ACC,ARGL
	sbc BCC,ARGH
	sbc PAGE,c_tmp
	std Z+user_phys,ACC
	std Z+user_phys+1,BCC
	std Z+user_phys+2,PAGE
	
;		if(disk_id != PDS_DISK)
;		  block++;
	ldd ACC,Z+block
	ldd BCC,Z+block+1
	adiw ACC,1
	std Z+block,ACC
	std Z+block+1,BCC

	
	ldd ACC,Z+c_count
	inc ACC
	std Z+c_count,ACC
	rjmp TOPCCOUNTLOOP
			
CCOUNTEX:
	; not the last cluster
	
	rcall ck_lastcluster  ; will check for pd inside call if there is room in flash for format
	cpi RESULT,0x00
	brne FOREVER007
	
	; get the next set of fat entry parameters

	; if disk_id == PDS_DISK
	;   offset =  block + 2
	; else ...
	
	ldd ARGL,Z+cluster
	ldd ARGH,Z+cluster+1
	ldi ACC,0x03
	mov r4,ACC
	clr r5
	rcall mpy16u                           ; cluster * 3
	clc
	ror ARGH
	ror ARGL                                ; (cluster * 3)/2
	std Z+offset,  ARGL
	std Z+offset+1,ARGH
	
	clr PAGE
	rcall setpage                          ; point to ram section where FAT Buffer is kept
	ldi XL,low(fat_buffer)
	ldi XH,high(fat_buffer)
	
	add XL,ARGL
	adc XH,ARGH
	
 ;     low_cluster = fat_buffer[offset];
 ;     high_cluster = fat_buffer[offset + 1];
	ld ACC,X+
	ld BCC,X
	
	; if disk_id == PDS_DISK 
	;   block = b
	; else ...
	
	;ldd ACC,Z+cluster
	;ldd BCC,Z+cluster+1
	ldd ARGL,Z+cluster
	sbrc ARGL,0                              ; if(cluster % 2) == 0 
	rjmp CLISODD
	; cluster is even
	andi BCC,0x0F                          ; clear high nibble
    rjmp CKEND007
CLISODD:	
	swap BCC
	mov c_tmp,BCC
	andi c_tmp,0xf0
	andi BCC,0x0F   ;high >> 4
	swap ACC        ; low >> 4
	andi ACC,0x0f
	or ACC,c_tmp                 ; >> 4
	;andi BCC,0x0F                         ; clear high nibble	 
CKEND007:  
	std Z+cluster,ACC
	std Z+cluster+1,BCC                   ; set new cluster value
	
	rcall ck_lastcluster
	breq FOREVER007 

	;block = ((cluster - 2) * bootSector.BPB_SecPerClus) + FirstDataSector;
	ldd ACC,Z+cluster
	ldd BCC,Z+cluster+1

	sbiw ACC,2

	mov r4,ACC
	mov r5,BCC

	ldi YL,low(BOOTSECTOR)
	ldi YH,high(BOOTSECTOR)
    ldd ARGL,Y+BPB_SecPerClus
	ldi ARGH,0
	rcall mpy16u	

	ldd ACC,Z+FirstDataSector
	ldd BCC,Z+FirstDataSector+1

	add ARGL,ACC
	adc ARGH,BCC

	std Z+Block,ARGL
	std Z+Block+1,ARGH

	rjmp TOP007LOOP
	
FOREVER007:	
	; this gets nasty as there may still be part of a sector to transfer
	
	; if f_count > 0 
	
	; this will need some serious work

	;
	; lots and lots to do here
	;
	;
	
	
	ret

	
;====================================================================================================
;                                        ck_lastcluster
;====================================================================================================
ck_lastcluster:

	clr RESULT                            ; start false
	
	ldd ARGL,Z+BLOCK
	ldd ARGH,Z+BLOCK+1

;	cpi ARGL,0xFE
;	ldi c_tmp,0xFF
;	cpc ARGH,c_tmp
;	brge CCOUNT010                        ; block >= 0xFFFE test for PD disk
	
	ldd ACC,Z+cluster
	ldd BCC,Z+cluster+1
	
	cpi ACC,0xFF
	ldi c_tmp,0x0F
	cpc BCC,c_tmp
	brlo CCOUNT002                        ; cluster <= 0xFFFE 
	breq CCOUNT002
	rjmp CCOUNT003
CCOUNT002:
	cpi ACC,0xF8
	ldi c_tmp,0x0F
	cpc BCC,c_tmp                         ; cluster >= 0x0FF8
	brge CCOUNT010
	ret                                   ; not end of file
CCOUNT010:
	ser RESULT
CCOUNT003:
	ret	
	

;====================================================================================================
;                                        play_song
;====================================================================================================
play_song:
	sbrc FFLAGS,playactive
	ret
	
	;first part of song has been parsed, recover the floppy flags and
	; read in the rest of the 
	
	; file is loaded into memory
	
	; change to task 2
	;ldi ACC,0x02 
	;std Z+task_num,ACC
	;rcall set_stack           ; set the stack to offset + task number
	
	; enable timers
	
	sbr FFLAGS,1 << playactive
	
	; parse the midi

; when MIDI is playing there are a lot of interrupts happening, So it is best to keep what we 
; can in the registers. this also limits the use of things that can split 2 byte loads 

; X  Y is the in register to the ring , the ring remains 256
; Z points to the function variables  unless handling an lpm

	clr r6                ; this is the workhorse counter, that is used to control
	clr r7                ; when the note events are sent it is called militimer
	clr r8
	clr r9

PLYBKTOP:

	mov c_tmp,FFLAGS
	andi c_tmp,0xE0
	cpi c_tmp,0x60   ; !active,eof,eot
	brne PC+2
	rjmp PLYBK007    ; this is the exit from playing

	sbrc FFLAGS,fseof   
	rjmp PLYBKTOP         ; still bytes in ring
	
		
	; if Mf_currentime <= militimer
	CLI            ; the main compare must be atomic
    cp  Mf_ct1,r6
    cpc Mf_ct2,r7
    cpc Mf_ct3,r8
    cpc Mf_ct3,r9
    SEI
    brlo GetNextEvent
    breq GetNextEvent
    rjmp SPINCLOCK
    
GetNextEvent:

; Keep format in results so that it is handy
	cpi RESULT,2         ;                                                 X <- target for removal
	brge PLYBK000         ;                                                 X
	
	; format is SMF      
	
	rcall readvarinum     
	
	std Z+Mf_deltatime    ,BCC  ; this will put the delta into ACC,BCC,ARGH,ARGL 
	std Z+Mf_deltatime + 1,ACC  ; these form a variable called Mf_deltatime	
	std Z+Mf_deltatime + 2,ARGH ; note this is low endian  
	std Z+Mf_deltatime + 3,ARGL 

PLYBK000:
	; PD delta calculations
    ; RFU
   
	; ESEQ does not calc it deltas here
	
	
	; this part of the parse is common to all MIDI formats

	rcall Mf_getc         ; the first byte will be in ARGL, the second will be in ARGH
						  ; the third byte location is TBD
	
	;sbrc FFLAGS,sysxcntu 
	;rjmp PLYBK001
	
	;cpi ARGL,0xF7          
	;breq PLYBK001         ; c != 0xF7
	
	; this is an error where the sysex is bogus
	;ldi c_tmp,ERR_BADMIDI
	;std Z+ERRORS,c_tmp
	;ldi FFLAGS,0x40       ; stop all playback - for now a bad file ends the queue
	;rjmp PLYBKTOP
	
	
PLYBK001:

	sbrc ARGL,7         ; is this a staus byte?  
	rjmp NOTRUNNING
	; byte is not a staus byte
		
	ldd ARGL,Z+status   ; get stored status into ARGL

; the commented code is more of a check for a HW error, as the only way
; for status to be set would be if something trashed the value, It could happen
; at the start if there has been no status sent whatsoever, this is better addressed
; by setting status to a note off initally	
	
;	cpi status,0 
;	brne RUNNINGSTATUS  ; status has value	
;	; this is probably a fatal error unexp running status
;	ldi c_tmp,ERR_BADSTATUS
;	std Z+ERRORS,c_tmp
;	ldi FFLAGS,0xC0       ; stop all playback - for now a bad file ends the queue
	
;RUNNINGSTATUS:

	sbr FFLAGS,1<<running ; set running status active
	rjmp NEEDED

NOTRUNNING:
	std Z+status,ARGL     ; save status for later runs
	cbr FFLAGS,1<<running ; de activate running status

NEEDED:

	; this may become a EEPROM read 
	
	ldd c_tmp,Z+status  ; can we afford the extra space hit? Y an Z are busy when playing	
	swap c_tmp
	andi c_tmp,0x0F     ; status >> 4 & 0x0F
	
	push ZH
	push ZL 
	ldi ZL,low(chantype * 2)
	ldi ZH,high(chantype * 2)

	clr r0
    add ZL,c_tmp
    adc ZH,r0
    lpm 
    pop ZL
    pop ZH
    
    ; r0 is set to the variable named needed as it is somewhat trasient here
    ldi c_tmp,0
	cp r0,c_tmp ; 
    breq METADATA
    
	mov ARGH,ARGL ; status is now in the high byte
	
	sbrc FFLAGS,running      ; running ? c1 = c : c1 = Mf_getc 
    rcall Mf_getc     
    
    ; ARGL is now c2 
    
    ; filter events here ...
    
	ldd ACC,Z+Mf_deltatime + 3  ; notice that the byte order is swapped on reload  
	ldd BCC,Z+Mf_deltatime + 2  ; this is to facilitate the arithmatic
	ldd r4 ,Z+Mf_deltatime + 1
	ldd r5 ,Z+Mf_deltatime  
	
	CLI             ; must be atomic
    add Mf_ct1,ACC
    adc Mf_ct2,BCC
    adc Mf_ct2,r4
    adc Mf_ct2,r5   ; Mf_currentime += Mf_deltatime
    
    clr ACC
    clr BCC
    clr r4
    clr r5          ; Mf_deltatime = 0 
    SEI
    
    
    rcall play_bsy
    st X,ARGH
    rcall play_bsy  ; this is a ring buffer, to save some compares it is 256 bytes  
    st X,ARGL       ; the cost is that we do not get to use X or Y for other functions	
    
    ldi c_tmp,1     ; if needed > 1
    cp r0,c_tmp
    brlo PLY300
    breq PLY300
    
    rcall Mf_getc   ; get the third byte
    
    ; this is where we would divide and conquer the velocity (slash and burn?)

	rcall play_bsy    
	st X,ARGL       ; queue the third byte if needed        
	
PLY300:
    cp XL,YL
    brne PLY301
    ; pointers collide  
	rcall play_bsy        ; this will keep things busy until a byte comes clear or 
	                      ; xmisson of the tuple ends  
     
PLY301:
    				
    sbrc FFLAGS,pending
    rjmp PLYBKTOP         ; no need to mark the end as it will get marked when the data is done XMITTING

	ser c_tmp
	inc XL		  ; the FF marks the end of the data and tells the interupt handler to wait
	st X,ARGL             ; more than one FF set byte means thier is nothing to transfer
	                      ; this has a potential problem in that if the queue is full then
	                      ; we will trash valid data
    
    ; start transmitter 
    sbis UCR,TXEN
	sbi	UCR,TXEN		  ; enable serial transmit               
	
	ld Txbyte,Y           ; get the first value from the ring
	out	UDR,Txbyte        ; the int system will take over from here.
    
    rjmp PLYBKTOP         ; continue
    
METADATA:
	
	cpi RESULT,2
	brge PLYBK005
	; SMF Meta handler
	
	; ARGL has the byte most recently read from the file
	cpi ARGL,0xFF
	brne CKSYSX
	
	rcall Mf_getc        ; read in the type 
	mov idx,ARGL         ; save it for later
	
	rcall readvarinum    ; read in the delta 
	mov c_tmp,ARGL
	
	cpi idx,0x51         ; tempo is one a few metas we know about
	brne CKMETA001
	
	rcall Mf_getc
	std Z+tempo,ARGL
	rcall Mf_getc
	std Z+tempo+1,ARGL
	rcall Mf_getc
	std Z+tempo+2,ARGL
	
	; somehow we need to queue the tempo change into the 
	; data stream, perhaps as a user defined controller?
	
	rjmp SPILLDATA
	
CKMETA001: 	
	cpi idx,0x2F
	brne CKMETA002
	
	;this is the end of the song, there is probably still notes being sequenced
	;so we need to wait until they finish playing
	sbrs FFLAGS,fseof
	rjmp PLYBKTOP
	
CKMETA002:
	rjmp SPILLDATA
	
CKSYSX:
	; totally ignore them
	
CKSYSX2:
	;
BOGUSDATA:
	; what else can we do?
	rjmp PLYBKTOP


SPILLDATA:

	; the value of lookfor is all over the map, this is a nasty compare
	ldd r4,Z+Mf_toberead
	cp  r4,c_tmp
	ldd r4,Z+Mf_toberead+1
	cpc r4,ARGH
	ldd r4,Z+Mf_toberead+2
	cpc r4,ACC
	ldd r4,Z+Mf_toberead+3
	cpc r4,BCC
	brlo SPILLEX
	breq SPiLLEX
	rcall Mf_getc        ; read in the type 
	rjmp SPILLDATA

SPILLEX:
	rjmp PLYBKTOP

; ****************************** end of SMF Meta parse

	
PLYBK005:
	cpi RESULT,3
	brne PLYBK006
	; ESEQ handler
	
	rjmp PLYBKTOP
	
PLYBK006:	
	; nothing else left to do other than  try the  PD handler
	
; sometime we should add  BAR/ANN if there is room

	rjmp PLYBKTOP

SPINCLOCK:

	; timer 0 is counting miliseconds 
	; when midi plays this keeps a clock ticking seconds and minutes
	
	;if it changes, output the new line
	
	;see how much time we have to the next event
	
	CLI
	mov ARGL,Mf_ct1
	mov ARGH,Mf_ct2
	mov ACC ,Mf_ct3
	mov BCC ,Mf_ct4
	
	sub ARGL, r6
	sbc ARGH, r7
	sbc ACC ,r8
	sbc BCC ,r9
	SEI
	
	sbrc BCC,7        ; result is negative, so we have no time to waste
	rjmp PLYBKTOP
	
	clr c_tmp
	cpi ARGL,15       ; we have time to read and debounce a keypress
	cpc ARGH,c_tmp
	cpc ACC ,c_tmp
	cpc BCC ,c_tmp
	brge PC+2
	rjmp PLYBKTOP
	rcall _WAITT	  ; spend time in other task
	rjmp PLYBKTOP

PLYBK007:	
	ret



;====================================================================================================
;                                        play_bsy
;====================================================================================================
play_bsy:
	ld c_tmp,X
	cpi c_tmp,0xFF         ; byte must be set before we can add to the queue
	breq PLYRDY            
	sbrs  FFLAGS,pending   ; there is data in the transmission buffer   
    rjmp PLYRDY
    inc XL                  ; this will hold us till the buffer clears some space
   
   	; test here for a timeout, in other word the system has stalled and there is
   	; no room in the ring for more data as long as there are interupts clearing the ring
   	; this should not happen
   
    rjmp play_bsy

PLYRDY:
	ret



;====================================================================================================
;                                        Mf_getc
;====================================================================================================
Mf_getc:
	;get the next byte from memory

	sbrc FFLAGS,fseof
	ret

	push XH
	push XL
		
	ldd XL,Z+playpos
	ldd XH,Z+playpos+1
	ldd PAGE,Z+playpos+2
	
	clc
	mov ARGL,XH           ; Save X becouse we rotate out A15
	rol ARGL              ; shift A15 into carry
	rol PAGE              ; shift carry into the page temp 
	rcall setPage         ; set port D
	ori XH,0x80           ; move to upper window

	ld ARGL,X
	
	ldd XL,Z+playpos      ;surprisingly it is fewer bytes just to reload the address
	ldd XH,Z+playpos+1
	ldd PAGE,Z+playpos+2
	
	; tricky complimentd add
	subi XL  ,0xFF        ;this actually adds one to the registers 
	sbci XH  ,0xFF        ;0 - (-1) is 1
	sbci PAGE,0xFF

	std Z+playpos,  XL
	std Z+playpos+1,XH
	std Z+playpos+2,PAGE
	pop XL
	pop XH

	push c_tmp
	ldd  c_tmp,Z+file_size
	cp  XL,c_tmp
	ldd  c_tmp,Z+file_size+1
	cpc XH,c_tmp
	ldd  c_tmp,Z+file_size
	cpc PAGE,c_tmp 
	brlo PC+2
	sbr FFLAGS,1<<fseof
	pop c_tmp
	
GETEX:
	ret


;====================================================================================================
;                                        readvarinum
;====================================================================================================
readvarinum:
	; will destroy r4 and c_tmp
	
	clr BCC
	clr ACC
	clr ARGH
	
	rcall Mf_getc
    sbrs ARGL,7        ; short delta
    ret                ; exit on a clear bit, byte 1
	 
    andi ARGL,0x7F     ;
	rcall LeftShift7   
	rcall Mf_getc
    sbrs ARGL,7
    rjmp isclear       ; exit byte 2
	rcall LeftShift7
	rcall Mf_getc
    sbrs ARGL,7
    rjmp isclear       ; exit byte 3
	rcall LeftShift7
	rcall Mf_getc
	sbrc ARGL,7
	rjmp mustexit000   ; this is an error we only have a 32 bit timer
isclear:
    clr r4
    andi ARGL,0x7F
    
    add  ARGL,c_tmp
    adc  ARGH,r4
    adc  ACC ,r4
    adc  BCC ,r4
mustexit000:   
    ret
	

;===============================================================================
;                     left shift 7
;===============================================================================
LeftShift7:    
    andi ARGL,0x7F
    mov  r5,BCC       ; for overflows
	mov  BCC,ACC      ;  << 8
	mov  ACC,ARGH      
	mov  ARGH,ARGL    
    ldi  ARGL,0 	
    clc
    ror  r5
    ror  BCC
    ror  ACC
    ror  ARGH          ; >> 1
 	ror  ARGL        
 	mov  c_tmp,ARGL    ; save becous the low byte gets destroyed?
	ret	



;***************************************************************************
;*
;* "mpy16u" - 16x16 Bit Unsigned Multiplication
;*
;* This subroutine multiplies the two 16-bit register variables 
;* mp16uH:mp16uL and mc16uH:mc16uL.
;* The result is placed in m16u3:m16u2:m16u1:m16u0.
;*  
;* Number of words	:14 + return
;* Number of cycles	:153 + return
;* Low registers used	:None
;* High registers used  :7 (mp16uL,mp16uH,mc16uL/m16u0,mc16uH/m16u1,m16u2,
;*                          m16u3,mcnt16u)	
;*
;***************************************************************************

;***** Subroutine Register Variables

;.def	mc16uL	=r4  		;multiplicand low byte
;.def	mc16uH	=r5 		;multiplicand high byte
;.def	mp16uL	=r22 ;ARGL		;multiplier low byte
;.def	mp16uH	=r23 ;ARGH		;multiplier high byte
;.def	m16u0	= r22; ARGL		;result byte 0 (LSB)
;.def	m16u1	= r23; ARGH		;result byte 1
;.def	m16u2	= r20 ;ACC		;result byte 2
;.def	m16u3	= r21 ;BCC		;result byte 3 (MSB)
;.def	mcnt16u	= r16 ;c_tmp		;loop counter

;***** Code

mpy16u:	
	clr	r25		;clear 2 highest bytes of result
	clr	r24
	ldi	r16,16	;init loop counter
	lsr	r23
	ror	r22

m16u_1:	
   brcc	noad8		;if bit 0 of multiplier set
	add	r24,r4	;add multiplicand Low to byte 2 of res
	adc	r25,r5	;add multiplicand high to byte 3 of res
noad8:	
	ror	r25		;shift right result byte 3
	ror	r24		;rotate right result byte 2
	ror	r23		;rotate result byte 1 and multiplier High
	ror	r22		;rotate result byte 0 and multiplier Low
	dec	r16		;decrement loop counter
	brne	m16u_1		;if not done, loop more
	ret

;***************************************************************************
;*
;* "div16u" - 16/16 Bit Unsigned Division
;*
;* This subroutine divides the two 16-bit numbers 
;* "dd8uH:dd8uL" (dividend) and "dv16uH:dv16uL" (divisor). 
;* The result is placed in "dres16uH:dres16uL" and the remainder in
;* "drem16uH:drem16uL".
;*  
;* Number of words	:19
;* Number of cycles	:235/251 (Min/Max)
;* Low registers used	:2 (drem16uL,drem16uH)
;* High registers used  :5 (dres16uL/dd16uL,dres16uH/dd16uH,dv16uL,dv16uH,
;*			    dcnt16u)
;*
;***************************************************************************

;***** Subroutine Register Variables

;.def	drem16uL = r4  
;.def	drem16uH = r5
;.def	dres16uL = r24 ;ACC
;.def	dres16uH = r25 ;BCC
;.def	dd16uL	= r24 ;ACC
;.def	dd16uH	= r25 ;BCC
;.def	dv16uL	= r22 ;ARGL
;.def	dv16uH	= r23 ;ARGH
;.def	dcnt16u	= r16 ;c_tmp


;***** Code

div16u:	
	clr	r4; drem16uL	;clear remainder Low byte
	sub	r5,r5 ;drem16uH,drem16uH;clear remainder High byte and carry
	ldi	r16,17	;dcnt16u init loop counter
d16u_1:	
	rol	r24;dd16uL		;shift left dividend
	rol	r25 ;dd16uH
	dec	r16 ;dcnt16u		;decrement counter
	brne	d16u_2		;if done
	ret			;    return
d16u_2:
	rol	r4 ;drem16uL	;shift dividend into remainder
	rol	r5 ;drem16uH
	sub	r4,r22 ;drem16uL,dv16uL	;remainder = remainder - divisor
	sbc	r5,r23 ;drem16uH,dv16uH	;
	brcc	d16u_3		;if result negative
	add	r4               ,r22 ;drem16uL,dv16uL	;    restore remainder
	adc	r5,r23 ;drem16uH,dv16uH
	clc			;    clear carry to be shifted into result
	rjmp	d16u_1		;else
d16u_3:	sec			;    set carry to be shifted into result
	rjmp	d16u_1

   
;***************************************************************************
;*
;* "mpy8u" - 8x8 Bit Unsigned Multiplication
;*
;* This subroutine multiplies the two register variables mp8u and mc8u.
;* The result is placed in registers m8uH, m8uL
;*  
;* Number of words	:9 + return
;* Number of cycles	:58 + return
;* Low registers used	:None
;* High registers used  :4 (mp8u,mc8u/m8uL,m8uH,mcnt8u)	
;*
;* Note: Result Low byte and the multiplier share the same register.
;* This causes the multiplier to be overwritten by the result.
;*
;***************************************************************************

;***** Subroutine Register Variables

;.def	mc8u	= r24 ;ACC		;multiplicand
;.def	mp8u	= r22 ;ARGL		;multiplier
;.def	m8uL	= r22 ;ARGL		;result Low byte
;.def	m8uH	= r23 ;ARGH		;result High byte
;.def	mcnt8u = r16 ;c_tmp		;loop counter

;***** Code


mpy8u:	
	clr	r23 ;m8uH		;clear result High byte
	ldi	r16,8 ;mcnt8u,8	;init loop counter
	lsr	r22 ;mp8u		;rotate multiplier
	
m8u_1:	
	brcc	m8u_2		;carry set 
	add 	r23,r24;m8uH,mc8u	;   add multiplicand to result High byte
m8u_2:	
	ror	r23;m8uH		;rotate right result High byte
	ror	r22;m8uL		;rotate right result L byte and multiplier
	dec	r16;mcnt8u		;decrement loop counter
	brne	m8u_1		;if not done, loop more
	ret

chantype:
.db 0, 0, 0, 0, 0, 0, 0, 0,    ; 0x00 through 0x70
.db 2, 2, 2, 2, 1, 1, 2, 0     ; 0x80 through 0xf0 

MSK: .DB 1,2,4,8,16,32,64,128
SCANP: .db 1,2,4,8
           ; 123456789.123456789.123456789.123456789 "
Text:   .db "Power On                                ",0x00,0x00
Text1:  .db "Starting Up ...                         ",0x00,0x00
TESTM:  .db "Memory test",0x00
BADMEM: .db "Memory test fail!",0x00
PASMEM: .db "Memory test pass.",0x00
BADFDC: .db "Unable to reset floppy controller.",0x00,0x00
NODISC: .db "Please insert a disc into drive. ",0x00
BADDSK: .db "Unable to read disc, try a diffrent disc",0x00,0x00
ANYBTN: .db "Press any button to continue...  ",0x00
PREPLD: .db "Loading disk ...                 ",0x00
BLANKL: .db "                                        ",0x00,0x00

           ; 123456789.123456789.123456789.123456789 "
MSGMNU: .db 0x20,0x20,"Menu",0x00,0x00  ;0
MSGSTP: .db 0x13,0x20,"Stop",0x00,0x00  ;1
MSGPLY: .db 0x1D,0x20,"Play",0x00,0x00  ;2
MSGSKR: .db 0x1D,0x20,"RT  ",0x00,0x00  ;3
MSGSKL: .db 0x1E,0x20,"LFT ",0x00,0x00  ;4
MSGSKU: .db 0x1F,0x20,"UP  ",0x00,0x00  ;5
MSGSKD: .db 0x1C,0x20,"DN  ",0x00,0x00  ;6
MSGREC: .db 0x20,0x53,"ingle",0x00  ;7

MSGFNC: .db 0x20,0x20,"MODE",0x00,0x00  ;8
MSGPWR: .db 0x94,0x20,"PWR ",0x00,0x00  ;9

MSGPAU: .db 0x12,0x50,"ause",0x00,0x00  ;A
MSGNXT: .db 0x1D,0x1D,"NXT ",0x00,0x00  ;B
MSGPRV: .db 0x1E,0x1E,"PRV ",0x00,0x00  ;C
MSGVLU: .db 0x1F,0x1F,"VOL+",0x00,0x00  ;D
MSGVLD: .db 0x1C,0x1C,"VOL-",0x00,0x00  ;E
MSGMTE: .db 0x20,0x20,"MUTE",0x00,0x00  ;F

