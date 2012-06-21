#ifndef ITG3200_H
#define ITG3200_H	

/* ************************ Register map for the ITG3200 ****************************/
#define ITG_ADDR	0xD0 //0xD0 if tied low, 0xD2 if tied high

#define WHO_AM_I	0x00
#define SMPLRT_DIV	0x15
#define	DLPF_FS		0x16
#define INT_CFG		0x17
#define INT_STATUS	0x1A
#define	TEMP_OUT_H	0x1B
#define	TEMP_OUT_L	0x1C
#define GYRO_XOUT_H	0x1D
#define	GYRO_XOUT_L	0x1E
#define GYRO_YOUT_H	0x1F
#define GYRO_YOUT_L	0x20
#define GYRO_ZOUT_H	0x21
#define GYRO_ZOUT_L	0x22
#define	PWR_MGM		0x3E

//Sample Rate Divider
//Fsample = Fint / (divider + 1) where Fint is either 1kHz or 8kHz
//Fint is set to 1kHz
//Set divider to 9 for 100 Hz sample rate

//DLPF, Full Scale Register Bits
//FS_SEL must be set to 3 for proper operation
//Set DLPF_CFG to 3 for 1kHz Fint and 42 Hz Low Pass Filter
#define DLPF_CFG_0	(1<<0)
#define DLPF_CFG_1	(1<<1)
#define DLPF_CFG_2	(1<<2)
#define DLPF_FS_SEL_0	(1<<3)
#define DLPF_FS_SEL_1	(1<<4)

//Power Management Register Bits
//Recommended to set CLK_SEL to 1,2 or 3 at startup for more stable clock
#define PWR_MGM_CLK_SEL_0	(1<<0)
#define PWR_MGM_CLK_SEL_1	(1<<1)
#define PWR_MGM_CLK_SEL_2	(1<<2)
#define PWR_MGM_STBY_Z	(1<<3)
#define PWR_MGM_STBY_Y	(1<<4)
#define PWR_MGM_STBY_X	(1<<5)
#define PWR_MGM_SLEEP	(1<<6)
#define PWR_MGM_H_RESET	(1<<7)

//Interrupt Configuration Bits
#define INT_CFG_ACTL	(1<<7)
#define INT_CFG_OPEN	(1<<6)
#define INT_CFG_LATCH_INT_EN	(1<<5)
#define INT_CFG_INT_ANYRD	(1<<4)
#define INT_CFG_ITG_RDY_EN	(1<<2)
#define INT_CFG_RAW_RDY_EN	(1<<0)

static float xr;
static float yr;
static float zr;
static float tr;
int id;
void ITG3200_begin(void)
{

i2c_write_reg(ITG_ADDR,DLPF_FS,DLPF_FS_SEL_0| DLPF_FS_SEL_1|DLPF_CFG_2|DLPF_CFG_1);
i2c_write_reg(ITG_ADDR,SMPLRT_DIV, 9);
i2c_write_reg(ITG_ADDR,INT_CFG, INT_CFG_RAW_RDY_EN | INT_CFG_ITG_RDY_EN);
i2c_write_reg(ITG_ADDR,PWR_MGM, PWR_MGM_CLK_SEL_0);
printf("Init ITG3200\n");
id= i2c_read_reg(ITG_ADDR,WHO_AM_I);
}

void ITG3200_update (void)
{

int value;
char temp=0;
int x;
int y;
int z;
int t;
int status;


	//status= i2c_read_reg(ITG_ADDR,INT_STATUS);
//	if ((status & 1) == 1){
	//printf("Good Data");
	
	value= i2c_read_reg(ITG_ADDR,TEMP_OUT_H);
	temp=value;
	value= i2c_read_reg(ITG_ADDR,TEMP_OUT_L);
	t = (temp<<8)|value;
	

	value= i2c_read_reg(ITG_ADDR,GYRO_XOUT_H);
	temp=value;
	value= i2c_read_reg(ITG_ADDR,GYRO_XOUT_L);
	x = (temp<<8)|value;
	

	value= i2c_read_reg(ITG_ADDR,GYRO_YOUT_H);;
	temp=value;
	value= i2c_read_reg(ITG_ADDR,GYRO_YOUT_L);
	y = (temp<<8)|value;

	value= i2c_read_reg(ITG_ADDR,GYRO_ZOUT_H);
	temp=value;
	value= i2c_read_reg(ITG_ADDR,GYRO_ZOUT_L);
	z = (temp<<8)|value;

	// Divide by 14.375 for Deg/S
	// Divide by 1000 for Deg/ms
	// Divide by PI/180 for Rad/ms
	//Equals 819375
#define GyroDivide 819375

	xr = (float) x/GyroDivide;
	yr = (float) y/GyroDivide;
	zr = (float) z/GyroDivide;
	xr=Runger_Kutta(xr,3);
	yr=Runger_Kutta(yr,4);
	zr=Runger_Kutta(zr,5);
	temp = -13200-t;	//Get the offset temp
	tr = (float)temp/280;	//Convert the offset to degree C
	tr += 35;	//Add 35 degrees C to compensate for the offset

//	xr=xr+0.004;
//	yr=yr+0.004;
//	zr=zr+0.004;
//	xr =(float)x/14.375;
//	printf("Gyro x %3.3f y %3.3f z %3.3f %i",xr,yr,zr,id);
}


float getGyroXOutput(void)
{
return xr;
}
float getGyroYOutput(void)
{
return yr;
}
float getGyroZOutput(void)
{
return zr;
}
float getGyroTempOutput(void)
{
return tr;
}
int getGyroIDOutput(void)
{
return id;
}

#endif
