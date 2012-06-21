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
 
 


static const float ticks_per_second = (float)FCY;
static const float ticks_per_ms = (float)FCY/1000;

void timer_init_ms()
{
	T1CONbits.TCS = 0;   // Internal clock (FOSC/4)
	T1CONbits.TCKPS = 0; // No prescaler
	T1CONbits.TON = 1;
	TMR1 = 0;            // Reset counter
}


void timer_reset()
{
	T1CONbits.TON = 0;
	TMR1 = 0;            // Reset counter
	T1CONbits.TON = 1;
}

/* 
 * Returns the time in seconds since the last call 
 * of this function (or last reset) 
 */
float timer_dt()
{
	float  ticks = (float)TMR1;
	timer_reset();	
//in Milli Seconds
	return ticks/ticks_per_ms;
}
