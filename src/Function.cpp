#include "Function.h"
#include <Arduino.h>
float CTemperatures;
//得到温度初始值
void Initial_K(float Data)
{
    CTemperatures = Data;
}

//卡尔曼滤波处理
float KalmanFilter(const float ResrcData,float ProcessNiose_Q,float MeasureNoise_R)
{

    float R = MeasureNoise_R;
    float Q = ProcessNiose_Q;

    static float x_last =  CTemperatures;
    float x_mid = x_last;
    float x_now;

    static float p_last;
    float p_mid ;
    float p_now;

    float kg;

    x_mid=x_last;                       //x_last=x(k-1|k-1),x_mid=x(k|k-1)
    p_mid=p_last+Q;                     //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=噪声

    /*
     *  卡尔曼滤波的五个重要公式
     */
    kg=p_mid/(p_mid+R);                 //kg为kalman filter，R 为噪声
    x_now=x_mid+kg*(ResrcData-x_mid);   //估计出的最优值
    p_now=(1-kg)*p_mid;                 //最优值对应的covariance
    p_last = p_now;                     //更新covariance 值
    x_last = x_now;                     //更新系统状态值

    return x_now;
}

//两脉冲一定位
int16_t Encoder_EC11_Scan(int16_t EC11_K_Now,int16_t EC11_A_Now,int16_t EC11_B_Now) 
{
    int16_t ScanResult = 0;
    if(EC11_A_Now != EC11_A_Last)       //当A发生跳变时采集B当前的状态，并将B与上一次的状态进行对比。
    {                                   //若A 0->1 时，B 1->0 正转；若A 1->0 时，B 0->1 正转；
                                        //若A 0->1 时，B 0->1 反转；若A 1->0 时，B 1->0 反转
        if(EC11_A_Now == 1)             //EC11_A和上一次状态相比，为上升沿
        {
            if((EC11_B_Last == 1)&&(EC11_B_Now == 0))           //EC11_B和上一次状态相比，为下降沿
                ScanResult = 1;                                 //正转
            if((EC11_B_Last == 0)&&(EC11_B_Now == 1))           //EC11_B和上一次状态相比，为上升沿               
                ScanResult = -1;                                //反转
            //>>>>>>>>>>>>>>>>下面为正转一次再反转或反转一次再正转处理<<<<<<<<<<<<<<<<//
            if((EC11_B_Last == EC11_B_Now)&&(EC11_B_Now == 0))  //A上升沿时，采集的B不变且为0
                ScanResult = 1;                                 //正转
            if((EC11_B_Last == EC11_B_Now)&&(EC11_B_Now == 1))  //A上升沿时，采集的B不变且为1
                ScanResult = -1;                                //反转
        }
        else                            //EC11_A和上一次状态相比，为下降沿
        {
            if((EC11_B_Last == 1)&&(EC11_B_Now == 0))           //EC11_B和上一次状态相比，为下降沿
                ScanResult = -1;                                //反转
            if((EC11_B_Last == 0)&&(EC11_B_Now == 1))           //EC11_B和上一次状态相比，为上升沿
                ScanResult = 1;                                 //正转
            //>>>>>>>>>>>>>>>>下面为正转一次再反转或反转一次再正转处理<<<<<<<<<<<<<<<<//
            if((EC11_B_Last == EC11_B_Now)&&(EC11_B_Now == 0))  //A上升沿时，采集的B不变且为0
                ScanResult = -1;                                //反转
            if((EC11_B_Last == EC11_B_Now)&&(EC11_B_Now == 1))  //A上升沿时，采集的B不变且为1   
                ScanResult = 1;                                 //正转
        }               
        EC11_A_Last = EC11_A_Now;   //更新编码器上一个状态暂存变量
        EC11_B_Last = EC11_B_Now;   //更新编码器上一个状态暂存变量
    }
    if(EC11_K_Now == 0)             //如果EC11的按键按下，并且没有EC11没有转动，
    {
        if(ScanResult == 1)       //按下按键时候正转
            ScanResult = 3;       //返回值为3
        if(ScanResult == -1)      //按下按键时候反转
            ScanResult = -3;      //返回值为-3
    }  
    return ScanResult; 
}

String to_String(int n)
{
    int m = n;
    char s[30];
    char ss[30];
    int i=0,j=0;
    if (n < 0)// 处理负数
    {
        m = 0 - m;
        j = 1;
        ss[0] = '-';
    }
    while (m>0)
    {
        s[i++] = m % 10 + '0';
        m /= 10;
    }
    s[i] = '\0';
    i = i - 1;
    while (i >= 0)
    {
        ss[j++] = s[i--];
    }
    ss[j] = '\0';
    return ss;
}

String doubleToString(const double &val ,String Str)
{
    char* chCode;
    chCode = new char[20];
    sprintf(chCode,(char*)Str.c_str(), val);
    String str(chCode);
    delete[]chCode;
    return str;
}