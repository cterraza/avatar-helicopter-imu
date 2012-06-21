#ifndef MOTOR_H
#define MOTOR_H


//---------------------------------------------------------------------
// MOTOR CONTROL (PWM MODULE 1 )
//---------------------------------------------------------------------
#define MOTOR_PWM_FREQ 1000		//motor PWM frequency in Hz
			

#define MOTOR_DUTY(percent) 	(P1DC1 = (2UL*P1TPER+2)*percent/100)	//(100% <=> P1TPER*2 since duty cycle resolution is Tcy/2)
#define MOTOR2_DUTY(percent) 	(P2DC1 = (2UL*P2TPER+2)*percent/100)	//see http://ww1.microchip.com/downloads/en/DeviceDoc/70187C.pdf  pg.33

int motorDuty[4] = {0,0,0,0};	//FRONT,BACK,LEFT,RIGHT

void motor_set_duty(unsigned char i,float duty){
	motorDuty[i] = float_to_int(put_in_range(duty,0,100));
}

void motor_apply_duty(){
	//(100% <=> P1TPER*2 since duty cycle resolution is Tcy/2)	
	//see http://ww1.microchip.com/downloads/en/DeviceDoc/70187C.pdf  pg.33
	P1DC1 = (2UL*P1TPER+2)*motorDuty[0]/100;
	P1DC2 = (2UL*P1TPER+2)*motorDuty[1]/100;
	P1DC3 = (2UL*P1TPER+2)*motorDuty[2]/100;
	P2DC1 = (2UL*P2TPER+2)*motorDuty[3]/100;
}

void motor_init(){
	//setup output pins
	_LAT(pinMotor0) = 1;
	_LAT(pinMotor1) = 1;
	_LAT(pinMotor2) = 1;
	_LAT(pinMotor3) = 1;

	_TRIS(pinMotor0) = 0;
	_TRIS(pinMotor1) = 0;
	_TRIS(pinMotor2) = 0;
	_TRIS(pinMotor3) = 0;
	printf("Debug 1\n");
	//setup PWM ports
	//PWM1, MOTORS 0,1,2	
	PWM1CON1 = 0;				//clear all bits (use defaults)
	PWM1CON1bits.PMOD1 = 1; 	//PWM1Ly,PWM1Hy are in independent running mode
	PWM1CON1bits.PEN1L = 1; 	//PWM1L1 PWM OUTPUT
	PWM1CON1bits.PEN1H = 0; 	//PWM1H1 PWM OUTPUT Disabled

	PWM1CON1bits.PMOD2 = 1; 	//PWM2Ly,PWM2Hy are in independent running mode
	PWM1CON1bits.PEN2L = 1; 	//PWM1L2 PWM OUTPUT
	PWM1CON1bits.PEN2H = 0; 	//PWM1H2 PWM OUTPUT Disabled

	PWM1CON1bits.PMOD3 = 1; 	//PWM3Ly,PWM2Hy are in independent running mode
	PWM1CON1bits.PEN3L = 1; 	//PWM1L3 NORMAL I/O
	PWM1CON1bits.PEN3H = 0; 	//PWM1H3 PWM OUTPUT Disabled
		printf("Debug 2\n");

//	//PWM2, MOTOR 3
	PWM2CON1 = 0;				//clear all bits (use defaults)
	PWM2CON1bits.PMOD1 = 1; 	//PWM2Ly,PWM2Hy are in independent running mode
	PWM2CON1bits.PEN1L = 1; 	//PWM2L1 NORMAL I/O
	PWM2CON1bits.PEN1H = 0; 	//PWM2H1 PWM OUTPUT Disabled

	//PWM mode and prescaler
	//PWM1, MOTORS 0,1,2	
	P1TCON = 0;					//clear all bits (use defaults)
	P1TCONbits.PTMOD = 0b00;	//Free-runing mode 
	P1TCONbits.PTCKPS = 0b11;	// 1:64 prescaler
	//PWM2, MOTOR 3
	P2TCON = 0;					//clear all bits (use defaults)
	P2TCONbits.PTMOD = 0b00;	//Free-runing mode 
	P2TCONbits.PTCKPS = 0b11;	// 1:64 prescaler
	printf("Debug 3\n");

	//setup desired frequency by setting period for 1:64 prescaler
	//PWM1, MOTORS 0,1,2	
	P1TPER = (FCY  / 64 / MOTOR_PWM_FREQ) - 1;	
	//PWM2, MOTOR 3
	P2TPER = (FCY  / 64 / MOTOR_PWM_FREQ) - 1;	
	printf("Debug 4\n");
	//update duty cycle 
	motor_apply_duty();	
		printf("Debug 5\n");
	//ENABLE PWM
	//PWM1, MOTORS 0,1,2
	P1TMR = 0;
	P1TCONbits.PTEN = 1;	
	//PWM2, MOTOR 3
	P2TMR = 0;
	P2TCONbits.PTEN = 1;	

}



#endif
