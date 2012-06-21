#include<timer.h>

unsigned int timer_value;

void __attribute__((__interrupt__)) _T1Interrupt(void)

{

    //PORTDbits.RD1 = 1;    /* turn off LED on RD1 */

    WriteTimer1(0);

    IFS0bits.T1IF = 0;    /* Clear Timer interrupt flag */

}

/* Enable Timer1 Interrupt and Priority to "1" */

    ConfigIntTimer1(T1_INT_PRIOR_1 & T1_INT_ON);

    WriteTimer1(0);

    match_value = 0xFFF;

    OpenTimer1(T1_ON & T1_GATE_OFF & T1_IDLE_STOP &

               T1_PS_1_1 & T1_SYNC_EXT_OFF &

               T1_SOURCE_INT, match_value);