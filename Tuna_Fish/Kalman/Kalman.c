#include "Kalman.h"
#include "Math.h"

bool cfilter_en=false,kfilter_en=false;

float Roll_est,Roll_predict;								//Estimated states and predicted states
float Pitch_est,Pitch_predict;
float Yaw_est,Yaw_predict;

float P_est[2][2][3],P_predict[2][2][3];												//Process covariance matrix

float Q_angle[3]={0.001,0.001,0},Q_bias[3]={0.003,0.003,0};			//Process Noise covariance matrix

float R_measurement[2]={0.003,0.003};														//Measurement noise covariance matrix

float Gyro_Bias[3];																							//Gyro bias

float Bomb[2];																									//Innovation

float K_gain[2][3];		//Kalman gain


void Attitude_c(float accel[3],float gyro[3],float rpy_c[3],float delt)
{
	if(!cfilter_en)
	{																									//http://www.nxp.com/files/sensors/doc/app_note/AN3461.pdf
		rpy_c[0]= atan2(accel[1],accel[2]) * rad_to_deg;
		rpy_c[1]= atan2(-accel[0],sqrt(accel[1]*accel[1] + accel[2]*accel[2])) * rad_to_deg;
		rpy_c[2]= 0;
		cfilter_en=true;
		PrintString("\nComplementry filter initiated");
	}
	
	float accel_roll=atan2(accel[1],accel[2]) * rad_to_deg;     //calculates from accelerometer readings
  float accel_pitch=atan2(-accel[0],sqrt(accel[1]*accel[1] + accel[2]*accel[2])) * rad_to_deg;
	
	rpy_c[0]= 0.93 * (rpy_c[0] + gyro[0]*delt) + 0.07*accel_roll;
  rpy_c[1]= 0.93 * (rpy_c[1] + gyro[1]*delt) + 0.07*accel_pitch;
	rpy_c[2]= 				rpy_c[2] + gyro[2]*delt;
	/*
	PrintString("\nPitch and Roll\t");
	PrintFloat(ypr_c[2]);
	PrintString("\t");
	PrintFloat(ypr_c[1]);*/
}

void Attitude_k(float accel[3],float gyro[3],float rpy_k[3], float delt)
{
	if(!kfilter_en)
	{
		rpy_k[0]= atan2(accel[1],accel[2]) * rad_to_deg;          //http://www.nxp.com/files/sensors/doc/app_note/AN3461.pdf
		rpy_k[1]= atan2(-accel[0],sqrt(accel[1]*accel[1] + accel[2]*accel[2])) * rad_to_deg;
		rpy_k[2]= 0;
		kfilter_en=true;
		PrintString("\nKalman filter initiated");
	}
	
	/*Prediction..of the state variables which are Roll and Gyro X bias and Pitch and Gyro Y Bias*/
																								//Previous_State + (Gyro_reading - Gyro_bias)delT..previous state updated with corrected gyro angular rate
	Roll_predict=  Roll_est  + (gyro[0]- Gyro_Bias[0])  * delt;
	Pitch_predict= Pitch_est + (gyro[1]- Gyro_Bias[1])  * delt;
	Yaw_predict= 	 Yaw_est   + (gyro[2]- Gyro_Bias[2])  * delt;
																								//No prediction for Gyro Bias..it'll be updated by measurement..no direct mesurement possible for gyro bias..it uses measured data to find the drift
	
	/*Prediction of the Process Covariance matrix [P_predict= A* P_prev * A transpose + Q ] where Q is Process Noise Covariance Matrix..
		we assume covariance to be zero so ther are only two elements Q angle and Q bias..variance in angle and variance in bias*/
	
	for(uint8_t i=0; i<2;i++)
	{
		P_predict[0][0][i]= P_est[0][0][i] + delt *( delt * P_est[1][1][i] - P_est[0][1][i] - P_est[1][0][i] + Q_angle[i]);
		P_predict[0][1][i]= P_est[0][1][i] - delt * P_est[1][1][i];
		P_predict[1][0][i]= P_est[1][0][i] - delt * P_est[1][1][i];
		P_predict[1][1][i]= P_est[1][1][i] + Q_bias[i] * delt;
	}
	
	/*Pitch and Roll computed using gravity i.e. via Accelerometers..
	this gets unreliable if there is acceleration in axis prependicular to gravitational axis*/
	
	float rpy_measured[2];
	
	float total_Accel = sqrt(accel[0]*accel[0] + accel[1]*accel[1] + accel[2]*accel[2]); //net acceleration vector.
	float inverted_Accel = 1/total_Accel;//this is because it doesn't make sense to divide twice as division takes more time
	
	if(abs(accel[1])<total_Accel-0.001)//just making sure the value of the acceleration isn't more than what the asin function can handle
	{
		rpy_measured[0]= asin(accel[1]*inverted_Accel) * rad_to_deg;//this method is faster and mathematically equivalent
	}
	if(abs(accel[0])<total_Accel-0.001)
	{
		rpy_measured[1]= asin(-accel[0]*inverted_Accel) * rad_to_deg;
	}
	/*
	The problem with the previous implementation was that if the system were to be put through a huge acceleration
	(say for example, a drone going in one direction and then suddenly going in the opposite direction, like in a racing drone),
	the measured value of the angles would be wayyy off. this would increase the "bomb" and since the R_measurement hasn't increased,
	The Kalman gain would throw off the estimates like a fat girl on a swing and not realize it. 
	I know this because it happened to me(not the fat girl on the swing).
	You need a variable R_measurement. To make the R_measurement variable, we must know when the Error is supposed to be high and when
	it is supposed to be low. If the total acceleration is equal to that of gravity, the error in the angle measurement is the least, 
	so the value of the least error is 0.003(set by pundoobvi) and the value of the max error (when the total acceleration is not 9.8)
	is..... IDK what it should be so I have currently kept it as 1+0.003. To find the R_measurement, I use a "notch filter" 
	Notch(x) =    
		x^2
	-------------------
	( 1 	 +     x^2)
	visualisation : http://www.wolframalpha.com/input/?i=x%5E2%2F(1%2Bx%5E2)
	The notch filter can be centered at x = 9.8 simply by subtracting 9.81 from x
	This way, the R_measurement will increase the moment the system accelerates and the system will never diverge too far away from
	the true value. 
	*/
	
	total_Accel = total_Accel - 9.81;//subtract gravity from the total acceleration to center the notch filter at 9.81
	R_measurement[0] = R_measurement[1] = ((total_Accel*total_Accel)/(1+total_Accel*total_Accel)) + 0.003;

	
	/*Computing Kalman Gain..which we can say is 
	             Error in prediction
	----------------------------------------------
	(Error in prediciton + Error in measurement)
	*/
	
	for(uint8_t i=0;i<2;i++)
	{
		K_gain[0][i]= P_predict[0][0][i]/( P_predict[0][0][i] + R_measurement[i] ) ;
		K_gain[1][i]= P_predict[1][0][i]/( P_predict[1][0][i] + R_measurement[i] ) ;
	}
	
	/*Innovation..Difference b/w Measured value and Predicted value*/
	
	Bomb[0]=rpy_measured[0] - Roll_predict;
	Bomb[1]=rpy_measured[1] - Pitch_predict;
	
	/*Updating predicted states with measured states..
	New_Estimate= Predicted_estimate + Kalman_gain * (Measured state - Predicted state)
	*/
	
	rpy_k[0]=Roll_est= Roll_predict +  K_gain[0][0] * Bomb[0];
	Gyro_Bias[0]= Gyro_Bias[0] + K_gain[1][0] * Bomb[0]; 
	
	rpy_k[1]=Pitch_est= Pitch_predict +  K_gain[0][1] * Bomb[1];
	Gyro_Bias[1]= Gyro_Bias[1] + K_gain[1][1] * Bomb[1];
	
	rpy_k[2]=Yaw_est= Yaw_predict;
	
	/*Updateing Process Covariance matrix Pk= (I- K*H)*Pkp  Pkp is predicted Process covariance matrix*/
	
	for(uint8_t i=0;i<2;i++)
	{
		P_est[0][0][i]= P_predict[0][0][i]*(1-K_gain[0][i]); 
		P_est[0][1][i]= P_predict[0][1][i]*(1-K_gain[0][i]);
		P_est[1][0][i]= P_predict[1][0][i]-(K_gain[1][i] * P_predict[0][0][i]); 
		P_est[1][1][i]= P_predict[1][1][i]-(K_gain[1][i] * P_predict[0][1][i]);
	}
	
	/*
	PrintString("\nGyro Biases\t");
	PrintFloat(Gyro_Bias[0]);
	PrintString("\t");
	PrintFloat(Gyro_Bias[1]);
	PrintString("\t");
	PrintFloat(Gyro_Bias[2]);
	PrintString("\t");
	*/
	
} /*End of fucntion*/

