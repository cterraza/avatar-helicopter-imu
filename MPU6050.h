#ifndef MPU6050_H
#define MPU6050_H
#define MPU6050_I2CADDR 0xD0

#define SMPLRT_DIV 0x19
#define CONFIG 0x1A
#define GYRO_CONFIG 0x1B
#define ACCEL_CONFIG 0x1C
#define ACCEL_XOUT_H 0x3B
#define ACCEL_XOUT_L 0x3C
#define ACCEL_YOUT_H 0x3D
#define ACCEL_YOUT_L 0x3E
#define ACCEL_ZOUT_H 0x3F
#define ACCEL_ZOUT_L 0x40
#define TEMP_OUT_H 0x41
#define TEMP_OUT_L 0x42
#define GYRO_XOUT_H 0x43
#define GYRO_XOUT_L 0x44
#define GYRO_YOUT_H 0x45
#define GYRO_YOUT_L 0x46
#define GYRO_ZOUT_H 0x47
#define GYRO_ZOUT_L 0x48
#define USER_CTRL 0x6A
#define PWR_MGMT_1 0x6B
#define PWR_MGMT_2 0x6C
#define WHO_AM_I 0x75
int t;
float fGyroX,fGyroY,fGyroZ,fAccelX,fAccelY,fAccelZ,fTemp;
unsigned char ID;

void MPU6050_begin() {
int ID;
	ID= i2c_read_reg(MPU6050_I2CADDR,WHO_AM_I);
	if (ID!=0x68) printf("Error MPU6050 not present");
printf("ID=,%x ",ID);
  i2c_write_reg(MPU6050_I2CADDR, PWR_MGMT_1, 0x80); // reset
  __delay_ms(500);
  i2c_write_reg(MPU6050_I2CADDR, PWR_MGMT_1, 0x00); // clk sel PLL gyro x	
__delay_ms(100);

//		i2c_write_reg(MPU6050_I2CADDR,0x37, 0x02);

/*  i2c_write_reg(MPU6050_I2CADDR, SMPLRT_DIV, 0x05); // sample rate 200Hz
__delay_ms(100);
  i2c_write_reg(MPU6050_I2CADDR, CONFIG, 0x03); // DLPF 42Hz
__delay_ms(100);
  i2c_write_reg(MPU6050_I2CADDR,GYRO_CONFIG, 0x18); // gyro scale 2000 deg/s
__delay_ms(100);
  i2c_write_reg(MPU6050_I2CADDR, ACCEL_CONFIG, 0x00); // accel scale 2g
__delay_ms(100); 	
 */ 
}

void MPU6050_update (void)
{
MPU6050_begin();
int GyroX,GyroY,GyroZ,AccelX,AccelY,AccelZ,ID,Temp;
#define AccelScaler 1/16384;
float Xoffset,Yoffset,Zoffset;
float Scaler;
unsigned char low;
char high;
	ID= i2c_read_reg(MPU6050_I2CADDR,WHO_AM_I);
	
	high= i2c_read_reg(MPU6050_I2CADDR,TEMP_OUT_H);
	low=  i2c_read_reg(MPU6050_I2CADDR,TEMP_OUT_L);
	printf(" g %x %x %i\n",high,low,t);
t++;
	Temp = (high<<8)|low;

	high= i2c_read_reg(MPU6050_I2CADDR,GYRO_XOUT_H);
	low= i2c_read_reg(MPU6050_I2CADDR,GYRO_XOUT_L);
	GyroX = (high<<8)|low;
	
	high= i2c_read_reg(MPU6050_I2CADDR,GYRO_YOUT_H);;
	low= i2c_read_reg(MPU6050_I2CADDR,GYRO_YOUT_L);
	GyroY = (high<<8)|low;

	high= i2c_read_reg(MPU6050_I2CADDR,GYRO_ZOUT_H);
	low= i2c_read_reg(MPU6050_I2CADDR,GYRO_ZOUT_L);
	GyroZ = (high<<8)|low;

	high= i2c_read_reg(MPU6050_I2CADDR,ACCEL_XOUT_H);
	low= i2c_read_reg(MPU6050_I2CADDR,ACCEL_XOUT_L);
	AccelX = (high<<8)|low;
	
	high= i2c_read_reg(MPU6050_I2CADDR,ACCEL_YOUT_H);;
	low= i2c_read_reg(MPU6050_I2CADDR,ACCEL_YOUT_L);
	AccelY = (high<<8)|low;
	printf(" g %x %x ",high,low);

	high= i2c_read_reg(MPU6050_I2CADDR,ACCEL_ZOUT_H);
	low= i2c_read_reg(MPU6050_I2CADDR,ACCEL_ZOUT_L);
	printf(" g %x %x ",high,low);

	AccelZ = (high<<8)|low;

	// Divide by 14.375 for Deg/S
	// Divide by 1000 for Deg/ms
	// Divide by PI/180 for Rad/ms
	//Equals 819375
#define GyroDivide 819375

	fGyroX = ((float) GyroX/GyroDivide)-Xoffset;
	fGyroY = ((float) GyroY/GyroDivide)-Yoffset;
	fGyroZ = ((float) GyroZ/GyroDivide)-Zoffset;

	fAccelX=(float)AccelX*AccelScaler;
	fAccelY=(float)AccelY*AccelScaler;
	fAccelZ=(float)AccelZ*AccelScaler;
	//Get the offset temp
	fTemp = (float)((Temp)/340)+36.53;	//Convert the offset to degree C
	fTemp += 35;	//Add 35 degrees C to compensate for the offset

//	xr=xr+0.004;
//	yr=yr+0.004;
//	zr=zr+0.004;
//	xr =(float)x/14.375;
	printf("ID %x",ID);
	printf("Gyro x %3.9f y %3.9f z %3.9f %x ",fAccelX,fAccelY,fAccelZ,ID);
	printf("Gyro x %3.9f y %3.9f z %3.9f %x ",fGyroX,fGyroY,fGyroZ,ID);

}
float getAcclXOutput(void)
{	
	return fAccelX;
}
float getAcclYOutput(void)
{
	return fAccelY;
}
float getAcclZOutput(void)
{
	return fAccelZ;
}
float getGyroXOutput(void)
{
return fGyroX;
}
float getGyroYOutput(void)
{
return fGyroY;
}
float getGyroZOutput(void)
{
return fGyroZ;
}
float getGyroTempOutput(void)
{
return fTemp;
}
int getGyroIDOutput(void)
{
return ID;
}
#endif
