//---------------------------------------------------------------------
// I/O PINS
//---------------------------------------------------------------------
#define pinLed1(f) 		f(B,1)		//Led, OUTPUT
#define pinLed2(f) 		f(B,0)		//Led, OUTPUT
#define pinSound(f)		f(B,0)		//Sound, OUTPUT
#define pinRX(f)  		f(B,2)		//RP25, UART RX, INPUT , IF CHANGED UPDATE uart_init() connect to TX
#define pinTX(f)  		f(B,3)		//RP24, UART TX, OUTPUT, IF CHANGED UPDATE uart_init()


#define pinMotor0(f)  	f(B,15)	//Front Motor, PWM1L1
#define pinMotor1(f)  	f(B,13)	//Back Motor, PWM1L2
#define pinMotor2(f)  	f(B,11)	//Left Motor, PWM1L3// 
#define pinMotor3(f)  	f(B,9)	//Right Motor, PWM2H1//Conflict I2C so need to use alternative


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
//#include "timer_dt.h"
#include "motor.h"
#include "uartutil.h"
#include "led.h"
#include "matrix.h"
#include "vector3d.h"
#include "i2cutil.h"
#include "ADXL345.h"
#include "ITG3200.h"
#include "imu.h"
#include "control.h"
#include "pulsin.h"


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
	// UART
	//---------------------------------------------------------------------
	uart_init();
	printf("Uart Init Ok\n");

	
	//---------------------------------------------------------------------
	// STATUS (LED/SOUND)
	//---------------------------------------------------------------------


    status_init();
//	STATUS(BLINK_SLOW,_ON,_ON);
//	__delay_ms(10);
//	STATUS_INIT;

	//---------------------------------------------------------------------
	// ADC	
	//---------------------------------------------------------------------
	
//	adc_init();
//	timer_init_ms();
	
	

	//---------------------------------------------------------------------
	// I2C
	//---------------------------------------------------------------------
	i2c_init();
	printf("i2c Init Ok\n");

    

	ITG3200_begin();
	printf("ITG3200 Ok\n");
	ADXL345_begin();
	printf("ADXL Init Ok\n");

	//STATUS(BLINK_FAST,_ON,_ON);  
	
	printf("Init Gyro\n");

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
//#define GyroTest
#ifdef GyroTest
while(1){
  ITG3200_update();
float GyroX=getGyroXOutput();
float GyroY=getGyroYOutput();
float GyroZ=getGyroZOutput();
int  ID=getGyroIDOutput();
float Temp= getGyroTempOutput();
printf("%1.3f %1.3f %1.3f  %1.3f %i \n",GyroX,GyroY,GyroZ,Temp,ID);
}

#endif
	//---------------------------------------------------------------------
	// MOTOR
	//---------------------------------------------------------------------
	motor_init();
	printf("motor init ok\n");
__delay_ms(100);

//#define motor_debug
#ifdef motor_debug

//while(1){
//	_LAT(pinMotor0) = 1;
//	_LAT(pinMotor1) = 1;
//	_LAT(pinMotor2) = 1;
//	_LAT(pinMotor3) = 1;
//
//	_TRIS(pinMotor0) = 0;
//	_TRIS(pinMotor1) = 0;
//	_TRIS(pinMotor2) = 0;
//	_TRIS(pinMotor3)= 0;
//
//
//}




	int i=0;
	while(i<50){
		_LAT(pinLed1) =! _LAT(pinLed1);
		motor_set_duty(0,i);
		motor_set_duty(1,i);
		motor_set_duty(2,i);
		motor_set_duty(3,i);
		motor_apply_duty();
		__delay_ms(50);
		i = i + 1;
	//	if(i> 90) i = 0;
		printf("Motor %x \n");
	
	}
		__delay_ms(250);
		i=0;
		motor_set_duty(0,i);
		motor_apply_duty();
		motor_set_duty(1,i);
		__delay_ms(250);
		motor_apply_duty();
		motor_set_duty(2,i);
		__delay_ms(250);
		motor_apply_duty();
		motor_set_duty(3,i);
		__delay_ms(250);
		motor_apply_duty();
		__delay_ms(250);
	
	
#endif

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

	//interval_ms = timer_dt();
		
	
	//	if(interval_ms > 0 ){
			//we have fresh adc samples
			//	printf("Time %lu ",interval_us);
			imu_update(interval_ms);
			float* K = dcmEst[2];                                           //K(body) vector from DCM matrix        
        	float pitch_roll = acos(K[2]);                          //total pitch and roll, angle necessary to bring K to [0,0,1]
                                                                                                                //cos(K,K0) = [Kx,Ky,Kz].[0,0,1] = Kz
			 float Kxy = sqrt(K[0]*K[0] + K[1]*K[1]);
			float anglePitch = - (pitch_roll * asin(K[0]/Kxy))*50;//*180/PI ; 
            float angleRoll = (pitch_roll * asin(K[1]/Kxy))*50;//*180/PI;
			
			//float angleRoll =	(atan2(dcmEst[2][2],dcmEst[2][1]))*180/3.14;
			//float anglePitch =	-(asin(dcmEst[2][0]))*180/3.14;
//	float	angleYaw=(-atan2(dcmGyro[1][0],dcmGyro[0][0]))*180/3.14;
	float 	driftX=(sin(anglePitch*(3.14/180))*(sin(angleRoll*(3.14/180))));
	float   CalcDriftX=getAcclXOutput()-driftX;
	float 	Acclx=getAcclXOutput();
	float   Gyrox=getGyroXOutput();
	float 	Accly=getAcclYOutput();
	float 	Acclz=getAcclZOutput();
	float   Gyroy=getGyroYOutput();

//printf("\n");
	
//if(0 == imu_sequence % 4){
//			hdlc_send_byte(float_to_int(anglePitch));
//			hdlc_send_byte(float_to_int(angleRoll));
//			hdlc_send_byte(float_to_int(Acclx*100));
//			hdlc_send_byte(float_to_int(Gyrox*1000));
//			hdlc_send_byte(float_to_int(Accly*100));
//			hdlc_send_byte(float_to_int(Gyroy*1000));
//			//hdlc_send_byte(float_to_int(Acclz*100));
//			//hdlc_send_byte(float_to_int(Kxy*100));
//			//hdlc_send_byte(float_to_int(K[0]*100));
//			//hdlc_send_byte(float_to_int(K[1]*100));
//			//hdlc_send_byte(float_to_int(K[2]*100));
//
////	float GyroYpitch = GyroYpitch+ (getGyroYOutput()*(interval_us/1000));
////	printf("pitch %1.3f roll %1.3f Gyro ",pitch.value,roll.value,GyroYpitch);
////	printf(" Angle P%1.3f R%1.3f Drift=%1.3f Acclx=%1.3f Calc=%1.3f GyroX=%1.3f\n",anglePitch,angleRoll,driftX,Acclx,CalcDriftX,GyroX);
//				hdlc_send_sep();
//}
	
			
		}
}

#endif

	//---------------------------------------------------------------------
	// MAIN LOOP
	//---------------------------------------------------------------------

	#define MAX_CONTROL	10.0					//maximum control in duty cycle units 
	#define MAX_THR  (100.0-MAX_CONTROL*2)		//maximum allowed throtle 
	int controlRoll,controlPitch, controlYaw;	//control for each axis
	float Roll,Pitch,Yaw;					
	float throtle = 0;
	int i;
	float RwTrg[2];								//target inclination
	float yawTrg;								//target Yaw rotation rate in  deg / ms

	float interval_ms;
	printf("Start Loop\n");
	while (1){
		unsigned char panic = 0;
		
	
		yaw.Kd = 15000.0;
        //Flight Note: 
        //6/21/11 : good results with yaw.kd=15k, pitch/roll kd=15-20k  , pitch/roll kp=500, but frame flimsy and broke outside

	//	unsigned long interval_us = adc_new_data();
	//	interval_us =  (long) interval_us+ timer_dt();
	

	

//		if(BATTERY_VOLTS < 9.0){ 
//			STATUS_PANIC_BATTERY;
//			panic = 1;
//		}


	//	printf(" %i \n",interval_us);
	//	if(interval_us > 0 ){		//loop running at 7.056ms see adcutil.h
			//we have fresh adc samples
			ITG3200_update();
			ADXL345_update();
		//	interval_ms =   timer_dt();
			//printf("------- %3.3fms ",interval_ms);
			imu_update();
			printf("Scanf ");
			char str [20];
			i=scanf("%s",&str);
			if (&str !="")
			{
			printf("----------------");
			}
			printf(" %s--%i %i\n ",&str,imu_sequence,i);
			if (throtle< 95){
			throtle++;
			}
			//development safety feature, decrease throtle if inclination is too big (imminent crash)

		//	Read_Uart();
			unsigned char w;
        float* K = dcmEst[2];                                           //K(body) vector from DCM matrix        
        float pitch_roll = acos(K[2]);                          //total pitch and roll, angle necessary to bring K to [0,0,1]
        
                                                                                                     //cos(K,K0) = [Kx,Ky,Kz].[0,0,1] = Kz
        //now allocate this angle proportionally between pitch and roll based on Kx, Ky
        float Kxy = sqrt(K[0]*K[0] + K[1]*K[1]);
		if(fabs(Kxy) < 0.01){                                           //avoid division by 0, stabilize values when Kxy is close to 0
                                pitch.value = 0;                                                //one of the two case when Kxy is close to 0 is when device is upside down
                                roll.value = pitch_roll;                                //default corrective action is roll 
                        }else{
			Roll =	(atan2(dcmEst[2][2],dcmEst[2][1]))*180/3.14;
			Pitch =	-(asin(dcmEst[2][0]))*180/3.14;
			}
			
			int print_interval=4;
 			pitch.value =  pitch_roll * asin(K[0]/Kxy) / (PI/2); 
            roll.value = -pitch_roll * asin(K[1]/Kxy) / (PI/2);
			for(w=0;w<2;w++){       //ROL = 0 , PTC = 1
			pid[w].target=0;
			pid[w].Kp=15;//1500
			pid[w].Kd=00;//500
			pid[w].error = low_pass_filter(pid[w].target  - pid[w].value,pid[w].error, 5.0 );
			//pid[w].error = pid[w].target  - pid[w].value;
			if (w == 0) {pid[w].dError =  -getGyroYOutput();}
			if (w == 1) {pid[w].dError =  getGyroXOutput();	}
			pid[w].control = control_pid(pid[w].error,0,pid[w].dError ,	pid[w].Kp,0,pid[w].Kd); //error*Kp + iError * Ki + dError * Kd
			//float control_pid(float error,float iError, float dError,   float Kp, float Ki, float Kd)	
			}
			cmd[YAW]=0;
			yaw.Kd=500;
			 yaw.target = map_to_range(cmd[YAW],-100.0,100.0, -0.002, 0.002);                                //target Yaw rotation rate in  rad / ms
             yaw.dError = low_pass_filter(yaw.target - getGyroZOutput(),yaw.dError, 5.0 );   //filter low_pass_filter(float vNew,float vPrev,float factor) (vPrev*factor + vNew) / ( 1 + factor);                        
             yaw.control =   control_pid(0,0,yaw.dError,                             0,0,yaw.Kd);
			motor_set_duty(0,(float) throtle + pid[0].control - pid[1].control);// 		//FRONT Left
			motor_set_duty(1,(float) throtle - pid[0].control - pid[1].control);// 		//Front Right
			motor_set_duty(2,(float) throtle + pid[0].control + pid[1].control);// 		//Back Left
			motor_set_duty(3,(float) throtle - pid[0].control + pid[1].control);// 		//Back RIGHT
			if(0 == imu_sequence % print_interval){
			
//#define Motor_Debug
#ifdef Motor_Debug			
                                //SerialChart file: PicQuadController_DEBUG1.scc
                                //hdlc_send_word(interval_us);
                                //hdlc_send_byte(float_to_int(errorPitch * 100));
                                hdlc_send_byte(motorDuty[0]);
                                hdlc_send_byte(motorDuty[1]);
                                hdlc_send_byte(motorDuty[2]);
                                hdlc_send_byte(motorDuty[3]);
                                hdlc_send_sep();
#endif                                

			}
			
	//		motor_apply_duty();
			
			
//				//send debug data every few cycles
//				//SerialChart file: PicQuadController_DEBUG_CONTROL.scc
//				hdlc_send_byte(float_to_int((pitch.value*100) ));
//				hdlc_send_byte(float_to_int((roll.value*100)));
//				hdlc_send_byte(float_to_int(pid[1].error*100));
////				hdlc_send_byte(float_to_int(anglePitch));
////				hdlc_send_byte(float_to_int(angleRoll));
////				hdlc_send_byte(float_to_int(angleYaw));
//////				hdlc_send_byte(float_to_int(getAcclXOutput()));
//////				hdlc_send_byte(float_to_int(getAcclYOutput()));
//////				hdlc_send_byte(float_to_int(getAcclZOutput()));
//				hdlc_send_sep();				
//	//		}


		

	
			

		
//			if(0 == imu_sequence % 4){
//				//send debug data every few cycles
//				//SerialChart file: PicQuadController_TX.scc
//				hdlc_send_byte(T_Pitch);
//				hdlc_send_byte(T_Roll);
//				hdlc_send_byte(T_Yaw);
//				hdlc_send_byte(T_Throttle);
//			
//				hdlc_send_sep();
//			}
//			
		

//		}
		

		
	
		
	}



	return 0;
}

//---------------------------------------------------------------------
// THE END
//---------------------------------------------------------------------
