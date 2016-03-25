#include "Quaternions.h"
#include "Math.h"

Quaternion CreateQ(float axis_vector[3], float rot_angle)						//Creates Quaternion. arguments- Axis vector and rotation angle
{
	Quaternion q;
	rot_angle=rot_angle*deg_to_rad/2;
	
	q.w= 			 cos(rot_angle);
	q.axis[0]= axis_vector[0] * sin(rot_angle);
	q.axis[1]= axis_vector[1] * sin(rot_angle);
	q.axis[2]= axis_vector[2] * sin(rot_angle);
	
	return q;
}

Quaternion MultiplyQ(Quaternion q1, Quaternion q2)
{
	Quaternion q;
	
	q.w			 = q1.w * q2.w       - q1.axis[0] * q2.axis[0] - q1.axis[1] * q2.axis[1] - q1.axis[2] * q2.axis[2]; 
	q.axis[0]= q1.w * q2.axis[0] + q1.axis[0] * q2.w	     + q1.axis[1] * q2.axis[2] - q1.axis[2] * q2.axis[1];
	q.axis[1]= q1.w * q2.axis[1] - q1.axis[0] * q2.axis[2] + q1.axis[1] * q2.w 			 + q1.axis[2] * q2.axis[0];
	q.axis[2]= q1.w * q2.axis[2] + q1.axis[0] * q2.axis[1] - q1.axis[1] * q2.axis[0] + q1.axis[2] * q2.w;
	return q;
}

Quaternion NormalizeQ(Quaternion q)
{
	float magnitude= sqrt(q.w*q.w + q.axis[0]*q.axis[0] + q.axis[1]*q.axis[1] + q.axis[2]*q.axis[2]);
	
	q.w=q.w/magnitude;
	q.axis[0]=q.axis[0]/magnitude;
	q.axis[1]=q.axis[1]/magnitude;
	q.axis[2]=q.axis[2]/magnitude;
	
	return q;
}

Quaternion QInverse(Quaternion q)
{
	Quaternion q1;
	q1.w=  q.w;
	q1.axis[0]= -q.axis[0];
	q1.axis[1]= -q.axis[1];
	q1.axis[2]= -q.axis[2];

	return q1;
}

void DisplayQ(Quaternion q)
{
	PrintString("<");
	PrintFloat(q.w);
	PrintString(",");
	PrintFloat(q.axis[0]);
	PrintString(",");
	PrintFloat(q.axis[1]);
	PrintString(",");
	PrintFloat(q.axis[2]);
	PrintString(">");;
}

Quaternion RotateVector(float vector[3],float axis[3], float rot_angle)
{
	Quaternion q,q_rotation,q_temp;
	
	q_rotation=CreateQ(axis,rot_angle);											//Ceates Quaternion to rotate around
	
	q_temp=CreateQ(vector,180);															//Creates Quaternion of the Vector to be rotated
	
	q= MultiplyQ(q_rotation,q_temp);												// Q_rotation * Vector
	q= MultiplyQ(q,QInverse(q_rotation));										// Q_Rotation * Vector * Inverse_Q_Rotation
	
	return q;
}

Quaternion RotateVectorX(float vector[3],float rot_angle)
{
	Quaternion q,q_rotation,q_temp;
	
	float axis[3]={1,0,0};
	q_rotation=CreateQ(axis,rot_angle);											//Ceates Quaternion to rotate around
	
	q_temp=CreateQ(vector,180);															//Creates Quaternion of the Vector to be rotated
	
	q= MultiplyQ(q_rotation,q_temp);												// Q_rotation * Vector
	q= MultiplyQ(q,QInverse(q_rotation));										// Q_Rotation * Vector * Inverse_Q_Rotation
	
	return q;
}


Quaternion RotateVectorY(float vector[3],float rot_angle)
{
	Quaternion q,q_rotation,q_temp;
	
	float axis[3]={0,1,0};
	q_rotation=CreateQ(axis,rot_angle);											//Ceates Quaternion to rotate around
	
	q_temp=CreateQ(vector,180);															//Creates Quaternion of the Vector to be rotated
	
	q= MultiplyQ(q_rotation,q_temp);												// Q_rotation * Vector
	q= MultiplyQ(q,QInverse(q_rotation));										// Q_Rotation * Vector * Inverse_Q_Rotation
	
	return q;
}

Quaternion RotateVectorZ(float vector[3],float rot_angle)
{
	Quaternion q,q_rotation,q_temp;
	
	float axis[3]={0,0,1};
	q_rotation=CreateQ(axis,rot_angle);											//Ceates Quaternion to rotate around
	
	q_temp=CreateQ(vector,180);															//Creates Quaternion of the Vector to be rotated
	
	q= MultiplyQ(q_rotation,q_temp);												// Q_rotation * Vector
	q= MultiplyQ(q,QInverse(q_rotation));										// Q_Rotation * Vector * Inverse_Q_Rotation
	
	return q;
}

Quaternion RotateVector3D(float vector[3],float yaw,float pitch,float roll)
{
	Quaternion q,q_rotation,q_rot_X,q_rot_Y,q_rot_Z,q_temp;
	
	float axis[3]={0,0,1};
	q_rot_Z=CreateQ(axis,yaw);											   //Ceates rotation Quaternion to rotate around Z axis..Yaw
	
	axis[2]=0;
	axis[1]=1;
	
	
	q_rot_Y=CreateQ(axis,pitch);											//Ceates Quaternion to rotate around Y axis..Pitch
	
	axis[1]=0;
	axis[0]=1;
	
	q_rot_X=CreateQ(axis,roll);											  //Ceates Quaternion to rotate around X axis..Roll
	
	q_rotation=MultiplyQ(q_rot_X,q_rot_Y);
	q_rotation=MultiplyQ(q_rotation,q_rot_Z);				  // q_rotation = q_rot_X * q_rot_Y * q_rot_Z
	
	q_temp=CreateQ(vector,180);															//Creates Quaternion of the Vector to be rotated
	
	q= MultiplyQ(q_rotation,q_temp);												// Q_rotation * Vector
	q= MultiplyQ(q,QInverse(q_rotation));										// Q_Rotation * Vector * Inverse_Q_Rotation
	
	return q;
}


