//---------------------------------------------------------------------
// I/O PINS
//---------------------------------------------------------------------
#define pinLed1(f) 		f(B,15)		//Led, OUTPUT
#define pinLed2(f) 		f(B,0)		//Led, OUTPUT
#define pinSound(f)		f(B,0)		//Sound, OUTPUT
#define pinRX(f)  		f(B,10)		//RP25, UART RX, INPUT , IF CHANGED UPDATE uart_init()
#define pinTX(f)  		f(B,11)		//RP24, UART TX, OUTPUT, IF CHANGED UPDATE uart_init()


#define pinMotor0(f)  	f(B,15)	//Front Motor, PWM1L1
#define pinMotor1(f)  	f(B,14)	//Back Motor, PWM1H1
#define pinMotor2(f)  	f(B,13)	//Left Motor, PWM1L2
#define pinMotor3(f)  	f(B,12)	//Right Motor, PWM1H2//May conflict with ISCP


#include "p33FJ64MC802.h"

#include <libpic30.h>



#include "oscilator.h"

#include <pwm.h>
#include <timer.h>
#include <pwm.h>
#include <uart.h>
#include <stdio.h>
#include <adc.h>

#include <math.h>


//---------------------------------------------------------------------
// LIBS
//---------------------------------------------------------------------


#include "macroutil.h"
#include "config.h"
#include "adcutil.h"
#include "pulsin.h"
#include "timer_dt.h"
#include "motor.h"
#include "uartutil.h"
#include "led.h"
#include "matrix.h"
#include "vector3d.h"
//#include "calibrate.h"
#include "i2cutil.h"
#include "ADXL345.h"
#include "ITG3200.h"
#include "imu.h"
#include "control.h"
//#include "ars.h"

float PredictAccG_roll(float a_z, float a_y, float a_x)
{
return  -(atan2(-a_z, a_y));
}
//---------------------------------------------------------------------
// MAIN
//---------------------------------------------------------------------


int main (void)
{
unsigned char value;
	//Configure all ports as inputs
	TRISA = 0xFFFF; TRISB = 0xFFFF;// TRISC = 0xFFFF;



	//---------------------------------------------------------------------
	// OSCILATOR
	//---------------------------------------------------------------------
	oscilator_init();
	
	//config_load();

	//---------------------------------------------------------------------
	// STATUS (LED/SOUND)
	//---------------------------------------------------------------------


    status_init();
	STATUS(BLINK_SLOW,_ON,_ON);
	__delay_ms(10);
	STATUS_INIT;

	//---------------------------------------------------------------------
	// ADC	
	//---------------------------------------------------------------------
	
//	adc_init();
	timer_init_ms();
	
	//---------------------------------------------------------------------
	// UART
	//---------------------------------------------------------------------
	uart_init();
	printf("Uart Init Ok\n");
	//---------------------------------------------------------------------
	// I2C
	//---------------------------------------------------------------------
	i2c_init();
	printf("i2c Init Ok\n");
//__delay_ms(100);

    ADXL345_begin();
//	__delay_ms(100);

	printf("ADXL Init Ok\n");

	ITG3200_begin();
//__delay_ms(100);
	printf("ITG3200 Ok\n");
	//STATUS(BLINK_FAST,_ON,_ON);  
	
	printf("Init Gyro\n");
;
//__delay_ms(100);
	printf("Debug Print\n");
//	STATUS_NORMAL;
//__delay_ms(100);
// Acceleromter calibration
int AcclCalibration;
AcclCalibration =1;
//#define AccCali
#ifdef AccCali
while(AcclCalibration){
	int i;
	double  adjAccl;
	for(i=0;i<10;i++){
	ADXL345_update();
}
	float Acclx=getAcclXOutput();
	float Accly=getAcclYOutput();
	float Acclz=getAcclZOutput();
	float   Gravity=sqrt(Acclx*Acclx+Accly*Accly+Acclz*Acclz);
	adjAccl= 1/Gravity;
	ADXL345_Scaler =adjAccl * ADXL345_Scaler;
	printf("%1.3f %1.3f %1.3f Gravity %1.9f y %1.9f %1.3f\n",Acclx,Accly,Acclz,Gravity,ADXL345_Scaler,adjAccl);
	if (Gravity == 1.0000) AcclCalibration=0;
	}
	printf("Accelerometer Calibrated \n");
#endif
	//---------------------------------------------------------------------
	// MOTOR
	//---------------------------------------------------------------------
	motor_init();
	printf("motor init ok\n");
__delay_ms(100);
while(1){
_LAT(pinMotor0) = 1;
	_LAT(pinMotor1) = 1;
	_LAT(pinMotor2) = 1;
	_LAT(pinMotor3) = 1;

	_TRIS(pinMotor0) = 0;
	_TRIS(pinMotor1) = 0;
	_TRIS(pinMotor2) = 0;
	_TRIS(pinMotor3)= 0;


}
	int i=100;
	while(1){
		_LAT(pinLed1) =! _LAT(pinLed1);
		motor_set_duty(0,i);
		motor_set_duty(1,i);
		motor_set_duty(2,i);
		motor_set_duty(3,i);
		__delay_ms(2000);
		i = i - 10;
		if(i<0) i = 100;
		printf("Motor %x \n");
		motor_apply_duty();
	};
	


	//---------------------------------------------------------------------
	// IMU
	//---------------------------------------------------------------------
	imu_init();
//#define imu_debug
#ifdef imu_debug
	//IMU debug comment out for production
	float interval_ms=0;
	while (1){
	ITG3200_update();
	ADXL345_update();

	interval_ms = interval_ms+( timer_dt()*1000);
		
	//	printf("Time %f3\n",interval_ms);
		if(interval_ms > 5 ){
			//we have fresh adc samples
			imu_update(interval_ms);
	float	anglePitch=(atan2(dcmGyro[2][1],dcmGyro[2][2]))*180/3.14;
	float	angleRoll=-(asin(dcmGyro[2][0]))*180/3.14;
//	float	angleYaw=(-atan2(dcmGyro[1][0],dcmGyro[0][0]))*180/3.14;
	float 	driftX=(sin(anglePitch*(3.14/180))*(sin(angleRoll*(3.14/180))));
	float   CalcDriftX=getAcclXOutput()-driftX;
	float 	Acclx=getAcclXOutput();

//printf("\n");
		printf(" Angle P%1.3f R%1.3f Drift=%1.3f Acclx=%1.3f Calc=%1.3f \n",anglePitch,angleRoll,driftX,Acclx,CalcDriftX);
//if(0 == imu_sequence % 4){
//			hdlc_send_byte(float_to_int(anglePitch));
//				hdlc_send_byte(float_to_int(angleRoll));
//				hdlc_send_byte(float_to_int(driftX));
//				hdlc_send_sep();
//}
	interval_ms=0;
			
		}
}

#endif
	//---------------------------------------------------------------------
	// MAIN LOOP
	//---------------------------------------------------------------------

	#define MAX_CONTROL	10.0					//maximum control in duty cycle units 
	#define MAX_THR  (100.0-MAX_CONTROL*2)		//maximum allowed throtle 
	float controlRoll,controlPitch, controlYaw;	//control for each axis
	float Kp_RollAndPitch;						//feedback coeficients (adjustable via ch5/6 on transmitter)
	float Kd_RollAndPitch,Kd_Yaw;						
	float throtle = 0;

	float RwTrg[2];								//target inclination
	float yawTrg;								//target Yaw rotation rate in  deg / ms

	Kd_Yaw = 50;

  

	
	while (1){
		unsigned char panic = 0;
	
		 float interval_ms;

	//	unsigned long interval_us = adc_new_data();
		float dt =  dt+ timer_dt();
		interval_ms=(float)dt;
	//	printf("  Seconds %1.9f ",dt);
	//	printf("\n");
	/*	pulsin_process();

		//any "panic" situation will result in gradual throtle decrease
		if(PULSIN_PANIC){ 
			STATUS_PANIC_SIGNAL;
			panic = 1;
		}

		if(BATTERY_VOLTS < 9.0){ 
			STATUS_PANIC_BATTERY;
			panic = 1;
		}

*/
	//	printf(" %1.3f \n",interval_ms);
	//	if(dt > 0.020 ){		//loop running at 7.056ms see adcutil.h
			//we have fresh adc samples
			ITG3200_update();
			ADXL345_update();
			imu_update(dt);
		//	dt=0;
			//development safety feature, decrease throtle if inclination is too big (imminent crash)
		/*
			if(fabs(dcmEst[0][0]) < 0.4 && fabs(dcmEst[0][1]) < 0.4){
				STATUS_PANIC_TILT;
				panic = 1;
			}
			
			if(panic){
				throtle = MAX(0,throtle - 0.1);	//this will cut throtle from 100 to 0 in 100/0.1 * 7.056ms = 7.056s
			}else{
				throtle = MIN(MAX_THR,cmd[THR]);
				STATUS_NORMAL;
			}

			*/
			//Max(K) ~= MAX_CONTROL / Max(measured) = 10 / 0.3 deg/ms = 33
			//To set Kd_Yaw move left stick to bottom-left , then adjust VRA, release stick, then revert VRA to previous level
			//Led will turn off while in Kd_Yaw adjustment mode
			/*if(0 == cmd[THR] && cmd[YAW] < -70){
				Kd_Yaw = map_to_range(cmd[VRA],0.0,100.0,	0.0,100.0);
				STATUS_ADJUST;
			}
*/
		//	throtle = 10;
			//Kp for Roll and Pitch
			
			//Kp_RollAndPitch = map_to_range(cmd[VRA],0.0,100.0,	0.0,100.0);
			Kp_RollAndPitch = map_to_range(0,0.0,100.0,	0.0,100.0);
			//Kd for Roll and Pitch
			//Kd_RollAndPitch = map_to_range(cmd[VRB],0.0,100.0, 	0.0,200.0);
			Kd_RollAndPitch = map_to_range(0,0.0,100.0, 	0.0,200.0);
			
			yawTrg = map_to_range(cmd[YAW],-100.0,100.0, -0.1, 0.1);
			float dErrorYaw =  (getGyroZOutput()- yawTrg);  		//  deg / ms	//CCW rotation <=> rateYaw >0

			RwTrg[0] = map_to_range(cmd[PTC],-100.0,100.0, -0.3, 0.3);
			
			float errorPitch =  dcmGyro[0][0] - RwTrg[0];
			float dErrorPitch = -getGyroXOutput(); 
			
			RwTrg[1] = map_to_range(cmd[ROL],-100.0,100.0, -0.3, 0.3);
			float errorRoll = dcmGyro[0][1] - RwTrg[1];
			float dErrorRoll = -getGyroYOutput(); 
			

			controlYaw = 	control_pid(0,0,dErrorYaw,				0,0,Kd_Yaw);
			controlPitch =	control_pid(errorPitch,0,dErrorPitch,	Kp_RollAndPitch,0,Kd_RollAndPitch);
			controlRoll =	control_pid(errorRoll,0,dErrorRoll,		Kp_RollAndPitch,0,Kd_RollAndPitch);


		//	motor_set_duty(0,throtle + controlYaw + controlPitch);		//FRONT
		//	motor_set_duty(1,throtle + controlYaw - controlPitch);		//BACK
		//	motor_set_duty(2,throtle - controlYaw + controlRoll);		//LEFT
		//	motor_set_duty(3,throtle - controlYaw - controlRoll);		//RIGHT


				
				
	

//		//	if(0 == imu_sequence % 4){
//				//send debug data every few cycles
//				//SerialChart file: PicQuadController_DEBUG_CONTROL.scc
////				hdlc_send_byte(float_to_int(controlPitch));
////				hdlc_send_byte(float_to_int(controlRoll));
////				hdlc_send_byte(float_to_int(controlYaw));
//				hdlc_send_byte(float_to_int(anglePitch));
//				hdlc_send_byte(float_to_int(angleRoll));
//				hdlc_send_byte(float_to_int(angleYaw));
//////				hdlc_send_byte(float_to_int(getAcclXOutput()));
//////				hdlc_send_byte(float_to_int(getAcclYOutput()));
//////				hdlc_send_byte(float_to_int(getAcclZOutput()));
//				hdlc_send_sep();				
//	//		}
//
//
		
/*
			if(0 == imu_sequence % 4){
				//send debug data every few cycles
				//SerialChart file: PicQuadController_DEBUG1.scc
				//hdlc_send_word(interval_us);
			
				hdlc_send_byte(float_to_int(errorPitch * 100));
				hdlc_send_byte(motorDuty[0]);
				hdlc_send_byte(motorDuty[1]);
				hdlc_send_byte(motorDuty[2]);
				hdlc_send_byte(motorDuty[3]);
				hdlc_send_sep();
				printf("motor Debug\n");
			}
			

/*			
			if(0 == imu_sequence % 4){
				//send debug data every few cycles
				//SerialChart file: PicQuadController_TX.scc
				hdlc_send_byte(cmd[THR]);
				hdlc_send_byte(cmd[YAW]);
				hdlc_send_byte(cmd[ROL]);
				hdlc_send_byte(cmd[PTC]);
				hdlc_send_byte(cmd[VRA]);
				hdlc_send_byte(cmd[VRB]);
				hdlc_send_sep();
			}
			
*/		

		}
		

		
	//	motor_apply_duty();
		
//	}



	return 0;
}

//---------------------------------------------------------------------
// THE END
//---------------------------------------------------------------------
