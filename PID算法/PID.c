#include "pid.h" 
extern double data[3];

struct _pid{
	float SetSpeed;                    //设定值
	float ActualSpeed;				   //实际值
	float err;						   //偏差值
	float err_last;                    //上一次偏差值
	float err_next;                    //下一次偏差值
	float Kp, Ki, Kd;           	   //设定值
	float voltage;                     //实际转换值
	float integral;					   //积分累积
	float umax;						   //偏差上限值
	float umin;						   //偏差下限值
}pid;

 void PID_Init()
{
	printf(" PID Init begin \r\n");
	pid.SetSpeed = 0.0;
	pid.ActualSpeed = 0.0;
	pid.err = 0.0;
	pid.err_last = 0.0;
	pid.err_next = 0.0;
	pid.voltage = 0.0;
	pid.integral = 0.0;
	pid.Kp = data[0];
    pid.Ki = data[1];
	pid.Kd = data[2];
	pid.umax = 400;
	pid.umin = -200;
	printf(" PID Init end \r\n");
	printf(" KP:%lf  ,KI:%lf,  KD:%lf\r\n",data[0],data[1],data[2]);
 
}
 //位置式PID公式
float PID_SpeedOut( float speed)
{
	pid.SetSpeed = speed;
	pid.err = pid.SetSpeed - pid.ActualSpeed;
	pid.integral += pid.err;
	pid.voltage = pid.Kp*pid.err + pid.Ki*pid.integral + pid.Kd*(pid.err - pid.err_last);//算法实现过程
	pid.err_last = pid.err;
	pid.ActualSpeed = pid.voltage * PID_CONV_REAL;
	return pid.ActualSpeed;
}
//增量PID公式
float PID_Speed_Incr( float speed)
{
	float increment; 
	pid.SetSpeed = speed;
	pid.err = pid.SetSpeed - pid.ActualSpeed;
    increment = pid.Kp*(pid.err - pid.err_next) + pid.Ki*pid.err + pid.Kd*(pid.err - 2*pid.err_next + pid.err_last);//算法实现过程
	pid.ActualSpeed += increment;
	pid.err_last = pid.err_next;
	pid.err_next = pid.err;
	return pid.ActualSpeed;
}
//积分分离pid
float PID_Integral1( float speed)
{
	float index;
	pid.SetSpeed = speed;
	pid.err = pid.SetSpeed - pid.ActualSpeed;
	if (abs(pid.err) > 200)
	{
		index = 0;
	}
	else
	{
		index = 1.0;
		pid.integral += pid.err;
	}
	pid.voltage = pid.Kp*pid.err + index * pid.Ki*pid.integral + pid.Kd*(pid.err - pid.err_last);//算法实现过程
	pid.err_last = pid.err;
	pid.ActualSpeed = pid.voltage * PID_CONV_REAL;
	return pid.ActualSpeed;
}

//抗积分饱和pid
float PID_Integral2(  float speed)
{
	float index;
	pid.SetSpeed = speed;
	pid.err = pid.SetSpeed - pid.ActualSpeed;
	if (pid.ActualSpeed > pid.umax)//算法实现过程
	{
		if (abs(pid.err) > 200)
		{
			index = 0;
		}
		else
		{
			index = 1.0;
			if (pid.err < 0)
				pid.integral += pid.err;
		}
	}
	else if (pid.ActualSpeed < pid.umin)//算法实现过程
	{
		if (abs(pid.err) > 200)
		{
			index = 0;
		}
		else
		{
			index = 1.0;
			if (pid.err > 0)
				pid.integral += pid.err;
		}
	}
	else
	{
		if (abs(pid.err) > 200)//算法实现过程
		{
			index = 0;
		}
		else
		{
			index = 1.0;
			pid.integral += pid.err;
		}
	}

	pid.voltage = pid.Kp*pid.err + index * pid.Ki*pid.integral + pid.Kd*(pid.err - pid.err_last);//算法实现过程
	pid.err_last = pid.err;
	pid.ActualSpeed = pid.voltage * PID_CONV_REAL;
	return pid.ActualSpeed;
}
//变积分PID公式 ： Ki*index
float PID_Integral3(  float speed)
{
	float index;
	pid.SetSpeed = speed;
	pid.err = pid.SetSpeed - pid.ActualSpeed;
		if (abs(pid.err) > 200)
		{
			index = 0;
		}
		else
		{
			if (abs(pid.err) < 180)
				index = 1.0;
			else
				index = (200 - abs(pid.err)) / 20;
			pid.integral += pid.err;
		}
	pid.voltage = pid.Kp*pid.err + index * pid.Ki*pid.integral + pid.Kd*(pid.err - pid.err_last);
	pid.err_last = pid.err;
	pid.ActualSpeed = pid.voltage * PID_CONV_REAL;
	return pid.ActualSpeed;
}
//抗积分饱和（设置上下限），积分分离，变积分Ki*index
float PID_Integral4( float speed)
{
	float index;
	pid.SetSpeed = speed;
	pid.err = pid.SetSpeed - pid.ActualSpeed;
	if (pid.ActualSpeed > pid.umax)
	{
		if (abs(pid.err) > 200)
		{
			index = 0;
		}
		else
		{
			if (abs(pid.err) < 180)
				index = 1.0;
			else
				index = (200 - abs(pid.err)) / 20;
			if (pid.err < 0)
				pid.integral += pid.err;
		}
	}
	else if (pid.ActualSpeed < pid.umin)
	{
		if (abs(pid.err) > 200)
		{
			index = 0;
		}
		else
		{
			if (abs(pid.err) < 180)
				index = 1.0;
			else
				index = (200 - abs(pid.err)) / 20;
			if (pid.err > 0)
				pid.integral += pid.err;
		}
	}
	else
	{
		if (abs(pid.err) > 200)
		{
			index = 0;
		}
		else
		{
			if (abs(pid.err) < 180)
				index = 1.0;
			else
				index = (200 - abs(pid.err)) / 20;
			pid.integral += pid.err;
		}
	}

	pid.voltage = pid.Kp*pid.err + index * pid.Ki*pid.integral + pid.Kd*(pid.err - pid.err_last);
	pid.err_last = pid.err;
	pid.ActualSpeed = pid.voltage * PID_CONV_REAL;
	return pid.ActualSpeed;
}
