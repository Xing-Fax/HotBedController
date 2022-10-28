#ifndef FUNCTION_H
#define FUNCTION_H
#include <Arduino.h>


static char EC11_A_Last;    //EC11的A引脚上一次的状态
static char EC11_B_Last;    //EC11的B引脚上一次的状态

void Initial_K(float Data);
float KalmanFilter(const float ResrcData,float ProcessNiose_Q,float MeasureNoise_R);
int16_t Encoder_EC11_Scan(int16_t key,int16_t EC11_A,int16_t EC11_B);
String to_String(int n);
String doubleToString(const double &val ,String Str);
#endif
