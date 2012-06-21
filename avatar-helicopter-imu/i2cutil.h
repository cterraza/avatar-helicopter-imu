#ifndef I2CUTIL_H
#define I2CUTIL_H

//---------------------------------------------------------------------
// I2C
// PIC33F Commuication Routine
// By P Hunt 29/10/2011
//---------------------------------------------------------------------
#include <i2c.h>

//---------------------------------------------------------------------
// Init I2C Port 1 
//---------------------------------------------------------------------
int rkIndex[6];
    float rkVal[6][4]; // Four Runge-Kutta integrator values for each of three axes
unsigned char i2c_init(void )

{
 unsigned int config2, config1;
  /* Baud rate is set for 100 Khz */

  config2 = 0x188; //100Khz @ 40MHZ

  /* Configure I2C for 7 bit address mode */

  config1 = (I2C1_ON & I2C1_IDLE_CON & I2C1_CLK_HLD &

             I2C1_IPMI_DIS & I2C1_7BIT_ADD &

             I2C1_SLW_DIS & I2C1_SM_DIS &

             I2C1_GCALL_DIS & I2C1_STR_DIS &

             I2C1_NACK & I2C1_ACK_DIS & I2C1_RCV_DIS &

             I2C1_STOP_DIS & I2C1_RESTART_DIS &

             I2C1_START_DIS);

  OpenI2C1(config1,config2);

	__delay_ms(100);
IdleI2C1();
Runger_Kutta_Init();

}

char i2c_read_reg(char addr,char reg)
{

//////////////////////////////////////
// Write Address
/////////////////////////////////////
// Write address + register
// Then Write address and read back data
	unsigned char value;

	IdleI2C1();
  	StartI2C1();
  	/* Wait till Start sequence is completed */
  	while(I2C1CONbits.SEN);
  	/* Clear interrupt flag */
  	IFS1bits.MI2C1IF = 0;
  	/* Write Slave address and set master for transmission */
//	__delay_ms(10);
  	MasterWriteI2C1(addr); //Address Write 
  	/* Wait till address is transmitted */
 	 while(I2C1STATbits.TBF);  // 8 clock cycles
 	 while(!IFS1bits.MI2C1IF); // Wait for 9th clock cycle
 	 IFS1bits.MI2C1IF = 0;     // Clear interrupt flag
 	 while(I2C1STATbits.ACKSTAT)
	{printf("I2C  Read Procedure Write Err add %c",addr);
	}
 //////////////////////////////////////////////////////////////
// Write Register
////////////////////////////////////////
  /* Transmit string of data */
  	MasterWriteI2C1(reg);//register or instruction  
	 /* Wait till address is transmitted */
  	while(I2C1STATbits.TBF);  // 8 clock cycles
 	 while(!IFS1bits.MI2C1IF); // Wait for 9th clock cycle
 	 IFS1bits.MI2C1IF = 0;     // Clear interrupt flag
  	while(I2C1STATbits.ACKSTAT);
  	/* Transmit string of data */
  	RestartI2C1();
  	/* Wait till stop sequence is completed */
  	while(I2C1CONbits.PEN);
	__delay_ms(1);
	addr=addr+1;
 //////////////////////////////////////////////////////////////
// Read Data Routine
////////////////////////////////////////
// Write address first
//------------------------------------------------------------

	MasterWriteI2C1(addr);//Read Data Address Read
	/* Wait till address is transmitted */

  	while(I2C1STATbits.TBF);  // 8 clock cycles
	while(!IFS1bits.MI2C1IF); // Wait for 9th clock cycle

  	IFS1bits.MI2C1IF = 0;     // Clear interrupt flag
	while(DataRdyI2C1());
//-------------------------------------------------------------
// Read Data
//--------------------------------------------------------------

	value = MasterReadI2C1(); /* Wait till stop sequence is completed */
//	printf("Complete Read Value.....%x\n",value);
	NotAckI2C1();
	StopI2C1();
  	while(I2C1CONbits.PEN);

	IdleI2C1();
	return value;
}

void i2c_write_reg(char addr,char reg,char value)
{

//////////////////////////////////////
// Write Address
/////////////////////////////////////
// Write address + register + data
	IdleI2C1();
  	StartI2C1();
  	/* Wait till Start sequence is completed */
  	while(I2C1CONbits.SEN);
  	/* Clear interrupt flag */
  	IFS1bits.MI2C1IF = 0;
  	/* Write Slave address and set master for transmission */
	//__delay_ms(100);
  	MasterWriteI2C1(addr); //Address Write 
  	/* Wait till address is transmitted */
 	 while(I2C1STATbits.TBF);  // 8 clock cycles
 	 while(!IFS1bits.MI2C1IF); // Wait for 9th clock cycle
 	 IFS1bits.MI2C1IF = 0;     // Clear interrupt flag
 	 while(I2C1STATbits.ACKSTAT)
	{printf("I2C Write Procedure Err add %x",addr);
	}

  //////////////////////////////////////////////////////////////
// Write Register
////////////////////////////////////////

	//__delay_ms(100);
  	MasterWriteI2C1(reg);//register or instruction  
	 /* Wait till address is transmitted */
  	while(I2C1STATbits.TBF);  // 8 clock cycles
 	 while(!IFS1bits.MI2C1IF); // Wait for 9th clock cycle
 	 IFS1bits.MI2C1IF = 0;     // Clear interrupt flag
  	while(I2C1STATbits.ACKSTAT);
	
//////////////////////////////////////////////////////////////////////
/// Write Value
//////////////////////////////////////////////////////////////////////

//__delay_ms(100);
	MasterWriteI2C1(value);//register or instruction  
	 /* Wait till address is transmitted */
  	while(I2C1STATbits.TBF);  // 8 clock cycles
 	 while(!IFS1bits.MI2C1IF); // Wait for 9th clock cycle
 	 IFS1bits.MI2C1IF = 0;     // Clear interrupt flag
  	while(I2C1STATbits.ACKSTAT);
	  
  IdleI2C1();
}
//------------------------------------------------------------------------------------------
// I2C Read Data
//------------------------------------------------------------------------------------------


///////////////////////////////////////
void i2c_close(void)
{
CloseI2C1();
}

float Runger_Kutta(float Value, int sensor)
{

// Runge-Kutta smoothing.
   
        rkVal[sensor][rkIndex[sensor]] = Value;
        Value = (1*rkVal[sensor][rkIndex[sensor]] +
                2*rkVal[sensor][(rkIndex[sensor]+1)%4] +
                2*rkVal[sensor][(rkIndex[sensor]+2)%4] +
                1*rkVal[sensor][(rkIndex[sensor]+3)%4])/6;
    
    rkIndex[sensor] = (rkIndex[sensor] + 1) % 4; // Increment index by 1 but loop back from 3 back to 0.
return Value;
}
void Runger_Kutta_Init(void)
{
int i,j;
    for ( i=0; i<3; i++)
        for ( j=0; j<4; j++)
            rkVal[i][j] = 0;

    for ( i=0; i<6; i++) {
     rkIndex[i] = 0;
   }
}
#endif
