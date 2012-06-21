#ifndef OSCILATOR_H
#define OSCILATOR_H

//---------------------------------------------------------------------
// OSCILATOR
//--------------------------------------------------------------------

//Note: FCY must be defined after #include <libpic30.h> since we're defining our own delay functions
#define FCY 40000000UL	// FCY = FOSC/2 = 80Mhz / 2 = 40Mhz

/*
 __delay_ms that is defined in libpic30.h  
 overflows if d argument is greater than  2^32 / FCY 
We'll define and use our own delay macros. 
We can do this without loosing precision,
as long as our FCY is multiple of 1000 and 1000000.
These functions will have a better range:
delayMs -> d <= 2^32 / (FCY / 1000) ~ 107374 (for 40Mhz Fcy)
delayUs -> d <= 2^32 / (FCY / 1000000) ~ 107,374,182 (for 40Mhz Fcy)
*/

#define __delay_ms(d) __delay32( (unsigned long) (d)*(FCY/1000))
#define __delay_us(d) __delay32( (unsigned long) (d)*(FCY/1000000))


//Using external oscilator. See:
//http://ww1.microchip.com/downloads/en/DeviceDoc/70216C.pdf , page 24

_FOSCSEL(FNOSC_FRC);							// Select Internal FRC at POR
_FOSC(FCKSM_CSECMD & OSCIOFNC_OFF & POSCMD_HS);	// Enable Clock Switching and Configure POSC in HS mode

_FWDT(FWDTEN_OFF);		// Watchdog Timer Enabled/disabled by user software
                        // (LPRC can be disabled by clearing SWDTEN bit in RCON register) 


void oscilator_init(){
	
// Configure Oscillator to operate the device at 80Mhz, 20 Mhz Crystal
	// Fosc= Fin * M/(N1*N2) = 20 * 24 / (3 * 2) = 80Mhz   , Fcy=Fosc/2 = 80 Mhz / 2 = 40 Mhz
	PLLFBD=24-2;				// M=PLLDIV+2
	CLKDIVbits.PLLPOST=0;		// N1=2/4/8  PLLPOST=0/1/3
	CLKDIVbits.PLLPRE=3-2;		// N2=PLLPRE+2 

	// Disable Watch Dog Timer
	RCONbits.SWDTEN=0;

	// Initiate Clock Switch to Primary Oscillator with PLL (NOSC = 0b011)
	__builtin_write_OSCCONH(0x03);
	__builtin_write_OSCCONL(0x01);

	// Wait for Clock switch to occur
	while (OSCCONbits.COSC != 0b011);

    // Problems Here cannot pass
/*
	// Wait for PLL to lock
	while(OSCCONbits.LOCK!=1) {};


*/

// setup internal clock for 80MHz/40MIPS
  // 7.37/2=3.685*43=158.455/2=79.2275
// Disable Watch Dog Timer
//	RCONbits.SWDTEN=0;
//	OSCTUNbits.TUN5=23;
 // CLKDIVbits.PLLPRE=0;        // PLLPRE (N2) 0=/2
//  PLLFBD=43;                  // pll multiplier (M) = +2
//  CLKDIVbits.PLLPOST=0;       // PLLPOST (N1) 0=/2
     // Wait for PLL to lock
//	while(OSCCONbits.LOCK!=1) {};

	__delay_ms(10);	//wait for clock to stabilize (otherwise various problems observed in particular with USART)
}




#endif


