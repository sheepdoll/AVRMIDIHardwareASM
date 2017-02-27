This folder contains legacy ASM code for a number of AVR based MIDI projects that date to the mid to late 2000s These projects pre-date Arduino as they use similar hardware this code has been replaced with more maintainable C code.

The main focus was to control a small pipe organ known as a Band Organ.  Often used with Merry-Go-Rounds they are sometimes also called Fairgrounds organs.

The original hardware was known as an OCTET UM0.  The first version of the code was written using the sysex definitions in the manual and dumping the MIDI messages to create a functional equivalent.  The processor uses was an Armel Mega88.  This code is in the ASM source.  It did not support changing the MIDI mappings through the Octet Windows95 setup program.

On the passing of Ron Perry who owned Midiator systems I did an attempt to port the Software update Rom code on the MIDIAtor website to AVR.  This was to allow the Windows95 setup program to update the setting and mapping tables.  

Octet boards were mostly used by novelty systems in places like hotel lobbies and casinos.  This presents some delimia in releasing the code as the MIDI DRM is controlled by a serial number stored in the flash memory.  On the other hand the published SYSEX routines in the Octet manual (which use the experimental MIDI manufacture ID) Do also allow access this when the Sysex messages are dumped.  This code requires the serial number to be entered at compile time.

The code is also missing some of the flash update routines as these would overwrite the code with 68HC11 firmware rather than the AVR firmware.  The OctetAvr.asm source does contain a mix of commented out HC11 code (from the online software updater) and AVR code.

The default shift register mapping also differs.  So that even with the same serial number protected MIDI files may not play.  It is for that reason I am placing the ASM files here as it is unlikely anyone can get the old hardware.  It is also useful to have the code in a CVS state as my local archives are spread across a number of CD disks, flash drives and other drives.  Given that this code is over 20 years old in places and most of the original creators are either dead or of advanced age with major health problems. I think it time to release it to the younger generation to learn from. 

In addition to the MIDI out boards, there is also a source file for A SMSC37C38 floppy disk controller memory mapped along with 128k of external Sram to an AT90S8515.  This is called AVRFloppyDiskMIDI.asm.  The most interesting part is a small real time microkernel that allows for some simple real time task management to handle the Floppy and LCD DMA. 



