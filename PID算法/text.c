#include "stdio.h"
#include "PID.h"

#define ITERATION_NUM 50 //计算次数
#define SET_VALUE 300.0   //PID目标值
int count = 0;
double data[3]={0.3,0.2,0.1};//分别对应P，I，D值，改变参数以观察数据增益情况


void pid_analysis(void);

int main()
{
  PID_Init();
  pid_analysis();
  return 0; 
}

void pid_analysis()
{
   while (count < ITERATION_NUM)
	{
		float DATA0 = PID_SpeedOut(   SET_VALUE); 
		float DATA1 = PID_Speed_Incr( SET_VALUE); 
		float DATA2 = PID_Integral1(  SET_VALUE); 
		float DATA3 = PID_Integral2(  SET_VALUE); 
		float DATA4 = PID_Integral3(  SET_VALUE); 
		float DATA5 = PID_Integral4(  SET_VALUE);
		printf("DATA0:%f, DATA1:%f, DATA2:%f, DATA3:%f, DATA4:%f, DATA5:%f \r\n",DATA0,DATA1,DATA2,DATA3,DATA4,DATA5);
		count++;
	}
}