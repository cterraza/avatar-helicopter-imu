/***********************************************************************
 *                                                                     *
 * This file contains the implementation of timer.h                    *
 *                                                                     *
 ***********************************************************************
 *                                                                     * 
 *    Author:         Tom Pycke                                        *
 *    Filename:       timer.c                                          *
 *    Date:           13/10/2007                                       *
 *    File Version:   1.00                                             *
 *    Other Files Required: timer.h                                    *
 *                          common.h                                   *
 *                                                                     *
 ***********************************************************************
 *                                                                     *
 * Other Comments:                                                     *
 *  Implements TIMER1 from dsPic to return time in seconds since last  *
 *  call.                                                              *
 *                                                                     *
 ***********************************************************************/
 
void __attribute__((__interrupt__, __auto_psv__)) _T1Interrupt(void); 


static const float ticks_per_second = (float)FCY;
static const float ticks_per_ms = (float)FCY/1000;
static const float ticks_per_us = (float)FCY/1000000;

unsigned long Overflow_count;

void __attribute__((__interrupt__, __auto_psv__)) _T1Interrupt(void)
{
 _T1IF = 0; //Clear Interrupt Flag
Overflow_count++;
}


void timer_init_ms()
{
	
	T1CONbits.TCS = 0;   // Internal clock (FCY)
	T1CONbits.TCKPS = 0; // No prescaler
	_T1IP = 1;            // Set Timer 1 interrupt priority
 	_T1IF = 0;            // Clear Timer 1 interrupt flag
 	_T1IE = 1;            // Enable Timer 1 interrupt
	T1CONbits.TON = 1;
	TMR1 = 0;            // Reset counter
	Overflow_count=0;
}


void timer_reset()
{
	T1CONbits.TON = 0;
	TMR1 = 0;            // Reset counter
	Overflow_count=0;
	T1CONbits.TON = 1;
}

/* 
 * Returns the time in seconds since the last call 
 * of this function (or last reset) 
 */
float timer_dt()
{
	unsigned long  ticks = TMR1;
	float ticks_ms;
	ticks=ticks+ (unsigned long)(Overflow_count*65535);
	ticks_ms =  (float)ticks/ticks_per_ms;
//	printf(" Over %lu %lu %3.9fms",ticks,Overflow_count,ticks_ms);
	timer_reset();
//in Micro Seconds
	//printf(" Ticks %lu %1.3f",ticks,FCY);
	return  ticks_ms;
}
