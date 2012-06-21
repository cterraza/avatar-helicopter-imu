
/* 
 * Attitude reference system (no heading for a ahrs ;-) )
 */

struct Gyro1DKalman
{
	/* These variables represent our state matrix x */
	float x_angle,
	      x_bias;

	/* Our error covariance matrix */
	float P_00,
	      P_01,
	      P_10,
	      P_11;	
	
	/* 
	 * Q is a 2x2 matrix of the covariance. Because we
	 * assuma the gyro and accelero noise to be independend
	 * of eachother, the covariances on the / diagonal are 0.
	 *
	 * Covariance Q, the process noise, from the assumption
	 *    x = F x + B u + w
	 * with w having a normal distribution with covariance Q.
	 * (covariance = E[ (X - E[X])*(X - E[X])' ]
	 * We assume is linair with dt
	 */
	float Q_angle, Q_gyro;
	/*
	 * Covariance R, our observation noise (from the accelerometer)
	 * Also assumed to be linair with dt
	 */
	float R_angle;
};

// Initializing the struct
void init_Gyro1DKalman(struct Gyro1DKalman *filterdata, float Q_angle, float Q_gyro, float R_angle);


// Kalman predict
void ars_predict(struct Gyro1DKalman *filterdata, const float gyro, const float dt);


// Kalman update
float ars_update(struct Gyro1DKalman *filterdata, const float angle_m);

