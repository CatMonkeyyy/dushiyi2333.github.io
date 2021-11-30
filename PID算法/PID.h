#ifndef __PID_H
#define __PID_H

#define PID_CONV_REAL 1.0 //定义pid输出倍数

#include <string.h> 
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>


void PID_Init(void);
float PID_SpeedOut( float speed);
float PID_Speed_Incr( float speed);
float PID_Integral1( float speed);
float PID_Integral2(  float speed);
float PID_Integral3(  float speed);
float PID_Integral4( float speed);

#endif

