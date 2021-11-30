#include "pid.h" 
extern double data[3];

struct _pid{
	float SetSpeed;                    //�趨ֵ
	float ActualSpeed;				   //ʵ��ֵ
	float err;						   //ƫ��ֵ
	float err_last;                    //��һ��ƫ��ֵ
	float err_next;                    //��һ��ƫ��ֵ
	float Kp, Ki, Kd;           	   //�趨ֵ
	float voltage;                     //ʵ��ת��ֵ
	float integral;					   //�����ۻ�
	float umax;						   //ƫ������ֵ
	float umin;						   //ƫ������ֵ
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
 //λ��ʽPID��ʽ
float PID_SpeedOut( float speed)
{
	pid.SetSpeed = speed;
	pid.err = pid.SetSpeed - pid.ActualSpeed;
	pid.integral += pid.err;
	pid.voltage = pid.Kp*pid.err + pid.Ki*pid.integral + pid.Kd*(pid.err - pid.err_last);//�㷨ʵ�ֹ���
	pid.err_last = pid.err;
	pid.ActualSpeed = pid.voltage * PID_CONV_REAL;
	return pid.ActualSpeed;
}
//����PID��ʽ
float PID_Speed_Incr( float speed)
{
	float increment; 
	pid.SetSpeed = speed;
	pid.err = pid.SetSpeed - pid.ActualSpeed;
    increment = pid.Kp*(pid.err - pid.err_next) + pid.Ki*pid.err + pid.Kd*(pid.err - 2*pid.err_next + pid.err_last);//�㷨ʵ�ֹ���
	pid.ActualSpeed += increment;
	pid.err_last = pid.err_next;
	pid.err_next = pid.err;
	return pid.ActualSpeed;
}
//���ַ���pid
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
	pid.voltage = pid.Kp*pid.err + index * pid.Ki*pid.integral + pid.Kd*(pid.err - pid.err_last);//�㷨ʵ�ֹ���
	pid.err_last = pid.err;
	pid.ActualSpeed = pid.voltage * PID_CONV_REAL;
	return pid.ActualSpeed;
}

//�����ֱ���pid
float PID_Integral2(  float speed)
{
	float index;
	pid.SetSpeed = speed;
	pid.err = pid.SetSpeed - pid.ActualSpeed;
	if (pid.ActualSpeed > pid.umax)//�㷨ʵ�ֹ���
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
	else if (pid.ActualSpeed < pid.umin)//�㷨ʵ�ֹ���
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
		if (abs(pid.err) > 200)//�㷨ʵ�ֹ���
		{
			index = 0;
		}
		else
		{
			index = 1.0;
			pid.integral += pid.err;
		}
	}

	pid.voltage = pid.Kp*pid.err + index * pid.Ki*pid.integral + pid.Kd*(pid.err - pid.err_last);//�㷨ʵ�ֹ���
	pid.err_last = pid.err;
	pid.ActualSpeed = pid.voltage * PID_CONV_REAL;
	return pid.ActualSpeed;
}
//�����PID��ʽ �� Ki*index
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
//�����ֱ��ͣ����������ޣ������ַ��룬�����Ki*index
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
