/***********************************************************************
 *                                                                     *
 * This file contains the code for the kalman filter that uses the     *
 * sensor data as inputs.                                              *
 *                                                                     *
 ***********************************************************************
 *                                                                     * 
 *    Author:         Tom Pycke                                        *
 *    Filename:       ars.c                                            *
 *    Date:           17/10/2007                                       *
 *    File Version:   1.00                                             *
 *                                                                     *
 ***********************************************************************
 *                                                                     *
 * Comments:                                                           *
 *   To help others to understand the kalman filter, I used one of     *
 *   the most accessible sources with information on it:               *
 *   http://en.wikipedia.org/wiki/Kalman_filter                        *
 *   The code is split into 2 parts: a Predict function and an         *
 *   Update function, just as the wikipedia page does.                 *
 *   The model I used in the kalman filter is the following:           *
 *   Our gyroscope measures the turnrate in degrees per second. This   *
 *   is the derivative of the angle, called dotAngle. The bias is the  *
 *   output of our gyro when the rotationrate is 0 degrees per second  *
 *   (not rotating). Because of drift it changes over time.            *
 *   So mathematically we just integrate the gyro (dt is timespan      *
 *   since last integration):                                          *
 *              angle = angle + (dotAngle - bias) * dt                 *
 *   When we include the bias in our model, the kalman filter will     *
 *   try to estimate the bias, thanks to our last input: the measured  *
 *   angle. This is just an estimate and comes from the accelerometer. *
 *   So the state used in our filter had 2 dimensions: [ angle, bias ] *
 *   Jump to the functions to read more about the actual               *
 *   implementation.                                                   *
 *                                                                     *
 ***********************************************************************/

#include "ars.h"

#include <math.h>



      

void init_Gyro1DKalman(struct Gyro1DKalman *filterdata, float Q_angle, float Q_gyro, float R_angle)
{
	filterdata->Q_angle = Q_angle;
	filterdata->Q_gyro  = Q_gyro;
	filterdata->R_angle = R_angle;
}

/*
 * The predict function. Updates 2 variables:
 * our model-state x and the 2x2 matrix P
 *     
 * x = [ angle, bias ]' 
 * 
 *   = F x + B u
 *
 *   = [ 1 -dt, 0 1 ] [ angle, bias ] + [ dt, 0 ] [ dotAngle 0 ]
 *
 *   => angle = angle + dt (dotAngle - bias)
 *      bias  = bias
 *
 *
 * P = F P transpose(F) + Q
 *
 *   = [ 1 -dt, 0 1 ] * P * [ 1 0, -dt 1 ] + Q
 *
 *  P(0,0) = P(0,0) - dt * ( P(1,0) + P(0,1) ) + dt² * P(1,1) + Q(0,0)
 *  P(0,1) = P(0,1) - dt * P(1,1) + Q(0,1)
 *  P(1,0) = P(1,0) - dt * P(1,1) + Q(1,0)
 *  P(1,1) = P(1,1) + Q(1,1)
 *
 *
 */
void ars_predict(struct Gyro1DKalman *filterdata, const float dotAngle, const float dt)
{
	filterdata->x_angle += dt * (dotAngle - filterdata->x_bias);

	filterdata->P_00 +=  - dt * (filterdata->P_10 + filterdata->P_01) + filterdata->Q_angle * dt;
	filterdata->P_01 +=  - dt * filterdata->P_11;
	filterdata->P_10 +=  - dt * filterdata->P_11;
	filterdata->P_11 +=  + filterdata->Q_gyro * dt;
}

/*
 *  The update function updates our model using 
 *  the information from a 2nd measurement.
 *  Input angle_m is the angle measured by the accelerometer.
 *
 *  y = z - H x
 *
 *  S = H P transpose(H) + R
 *    = [ 1 0 ] P [ 1, 0 ] + R
 *    = P(0,0) + R
 * 
 *  K = P transpose(H) S^-1
 *    = [ P(0,0), P(1,0) ] / S
 *
 *  x = x + K y
 *
 *  P = (I - K H) P
 *
 *    = ( [ 1 0,    [ K(0),
 *          0 1 ] -   K(1) ] * [ 1 0 ] ) P
 *
 *    = [ P(0,0)-P(0,0)*K(0)  P(0,1)-P(0,1)*K(0),
 *        P(1,0)-P(0,0)*K(1)  P(1,1)-P(0,1)*K(1) ]
 */
float ars_update(struct Gyro1DKalman *filterdata, const float angle_m)
{
	const float y = angle_m - filterdata->x_angle;
	
	const float S = filterdata->P_00 + filterdata->R_angle;
	const float K_0 = filterdata->P_00 / S;
	const float K_1 = filterdata->P_10 / S;
	
	filterdata->x_angle +=  K_0 * y;
	filterdata->x_bias  +=  K_1 * y;
	
	filterdata->P_00 -= K_0 * filterdata->P_00;
	filterdata->P_01 -= K_0 * filterdata->P_01;
	filterdata->P_10 -= K_1 * filterdata->P_00;
	filterdata->P_11 -= K_1 * filterdata->P_01;
	
	return filterdata->x_angle;
}
