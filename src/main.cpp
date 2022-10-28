#include <Arduino.h>
#include <nvs.h>
#include <OneButton.h>
#include <PID_v1.h>
#include <max6675.h>
#include <U8g2lib.h>
#include "Function.h"
#include "Bitmap.h"

/* Pins */
//Max6675
#define Max6675_SO  19
#define Max6675_CS  5
#define Max6675_SCK 18
//旋转编码器
#define EC11_DATA_K 27
#define EC11_DATA_A 12
#define EC11_DATA_B 14
//0.91 OLED
#define OLED_SCL 22
#define OLED_SDA 21
//PWM控制
#define PwmPin 15
//按键
#define ON_PIN  2
#define SET_PIN 0
#define FN_PIN  4
//自恢复温度保险
#define FN_INS  13

//设定加热温度区间
#define TEM_MAX 300
#define TEM_MIN 30
//每~200ms读取一次数字温度传感器
#define TEMP_READ_DELAY 200 
//每~5m刷新一次屏幕
#define TEMP_READ_REFRE 5

/* 卡尔曼滤波 */
//Q:过程噪声，Q增大，动态响应变快，收敛稳定性变坏
float KALMAN_Q = 0.4;
//R:测量噪声，R增大，动态响应变慢，收敛稳定性变好 
float KALMAN_R = 3.0;

/* PID */
//要连接的变量
double Setpoint, Input, Output;
//初始调整参数
double KP = 3.0, KI = 1.0, KD = 0.5;

/* 基本 */
//设定默认加热温度
int Temperature = 200;
//保存当前温度
float CTemperature;
//保存滤波温度
float OTemperature;
//编码器状态
int16_t State;
//界面状态
int16_t interface;
//要设置的项
int16_t Item;
//温度单位
int16_t Unit;
//是否开启加热
bool Mode = false;
//是否初始化完毕
bool inits = false; 
//Mos是否过温
bool OverTemp = false; 
//过温保护使能
bool OverEnable = true;
//NVS对象
nvs_handle Handle;

/* 初始化MAX6675模块 */
MAX6675 Thermocouple(Max6675_SCK, Max6675_CS, Max6675_SO);
/* 初始化PID */
PID MYPID(&Input, &Output, &Setpoint, KP, KI, KD, DIRECT);
/*初始化0.91寸OLED屏幕*/
U8G2_SSD1306_128X32_UNIVISION_F_SW_I2C u8g2(U8G2_R0, OLED_SCL, OLED_SDA, U8X8_PIN_NONE); 

/*初始化按键检测*/
OneButton button_ON(ON_PIN,true);
OneButton button_SET(SET_PIN,true);
OneButton button_FN(FN_PIN,true);

//加载设置
void LoadConfig(){
    uint32_t T1 = 32;char T[32] = {0};uint32_t P1 = 32;char P[32] = {0};
    uint32_t I1 = 32;char I[32] = {0};uint32_t D1 = 32;char D[32] = {0};
    uint32_t Q1 = 32;char Q[32] = {0};uint32_t R1 = 32;char R[32] = {0};
    uint32_t U1 = 32;char U[32] = {0};uint32_t O1 = 32;char O[32] = {0};
    ESP_ERROR_CHECK( nvs_open("Customer data", NVS_READWRITE, &Handle) );
    ESP_ERROR_CHECK ( nvs_get_str(Handle, "ST", T , &T1) );
    ESP_ERROR_CHECK ( nvs_get_str(Handle, "KP", P , &P1) );
    ESP_ERROR_CHECK ( nvs_get_str(Handle, "KI", I , &I1) );
    ESP_ERROR_CHECK ( nvs_get_str(Handle, "KD", D , &D1) );
    ESP_ERROR_CHECK ( nvs_get_str(Handle, "Q", Q , &Q1) );
    ESP_ERROR_CHECK ( nvs_get_str(Handle, "R", R , &R1) );
    ESP_ERROR_CHECK ( nvs_get_str(Handle, "Unit", U , &U1) );
    ESP_ERROR_CHECK ( nvs_get_str(Handle, "Over", O , &O1) );
    nvs_close(Handle); OverEnable = atoi(O);
    Temperature = atoi(T); Unit = atoi(U);
    KP = atof(P); KI = atof(I); KD = atof(D);
    KALMAN_Q = atoff(Q); KALMAN_R = atoff(R);
}

//保存设置
void KeepSet(){
    ESP_ERROR_CHECK( nvs_open("Customer data", NVS_READWRITE, &Handle) );
    uint32_t P1 = 32;char P[32] = {0};uint32_t I1 = 32;char I[32] = {0};
    uint32_t D1 = 32;char D[32] = {0};uint32_t Q1 = 32;char Q[32] = {0};
    uint32_t R1 = 32;char R[32] = {0};uint32_t U1 = 32;char U[32] = {0};
    uint32_t O1 = 32;char O[32] = {0};
    ESP_ERROR_CHECK ( nvs_get_str(Handle, "KP", P , &P1) );
    ESP_ERROR_CHECK ( nvs_get_str(Handle, "KI", I , &I1) );
    ESP_ERROR_CHECK ( nvs_get_str(Handle, "KD", D , &D1) );
    ESP_ERROR_CHECK ( nvs_get_str(Handle, "Q", Q , &Q1) );
    ESP_ERROR_CHECK ( nvs_get_str(Handle, "R", R , &R1) );
    ESP_ERROR_CHECK ( nvs_get_str(Handle, "Unit", U , &U1) );
    ESP_ERROR_CHECK ( nvs_get_str(Handle, "Over", O , &O1) );
    if(doubleToString(KP,"%.1lf") != doubleToString(atoff(P),"%.1lf"))
        ESP_ERROR_CHECK( nvs_set_str( Handle, "KP",   (char*)doubleToString(KP,"%.1lf").c_str()) );
    if(doubleToString(KI,"%.1lf") != doubleToString(atoff(I),"%.1lf"))
        ESP_ERROR_CHECK( nvs_set_str( Handle, "KI",   (char*)doubleToString(KI,"%.1lf").c_str()) );
    if(doubleToString(KD,"%.1lf") != doubleToString(atoff(D),"%.1lf"))
        ESP_ERROR_CHECK( nvs_set_str( Handle, "KD",   (char*)doubleToString(KD,"%.1lf").c_str()) );
    if(doubleToString(KALMAN_Q,"%.1lf") != doubleToString(atoff(Q),"%.1lf"))
        ESP_ERROR_CHECK( nvs_set_str( Handle, "Q",    (char*)doubleToString(KALMAN_Q,"%.1lf").c_str()) );
    if(doubleToString(KALMAN_R,"%.1lf") != doubleToString(atoff(R),"%.1lf"))
        ESP_ERROR_CHECK( nvs_set_str( Handle, "R",    (char*)doubleToString(KALMAN_R,"%.1lf").c_str()) );
    if(doubleToString(Unit,"%.0lf") != doubleToString(atoi(U),"%.0lf"))
        ESP_ERROR_CHECK( nvs_set_str( Handle, "Unit", (char*)doubleToString(Unit,"%.0lf").c_str()) );       
    if(doubleToString(OverEnable,"%.0lf") != doubleToString(atoi(O),"%.0lf"))
        ESP_ERROR_CHECK( nvs_set_str( Handle, "Over", (char*)doubleToString(OverEnable,"%.0lf").c_str()) );
        
    ESP_ERROR_CHECK( nvs_commit(Handle) );
    nvs_close(Handle);
}

float TimeS;
unsigned long PlayMusic;
void PlayBuzzer(){
    if ((millis() - PlayMusic) > TimeS)
        ledcWrite(1, 0);
}

void Buzzer(int Frequency,float Time){
    PlayMusic= millis();
    TimeS = Time;
    ledcWrite(1, Frequency);
}

//单击按键1
void ONClick(){
    switch (interface)
    {
        case 0:
            if(Mode){
                Mode = false;
                Buzzer(150,20);
            }

            break;
        case 1 :
            Item --; if(Item < 0) Item = 2;
            Buzzer(150,20);
            break;
        case 2:
            Item --; if(Item < 0) Item = 1;
            Buzzer(150,20);
            break;
        case 3:
            Item --; if(Item < 0) Item = 1;
            Buzzer(150,20);
            break;
    }
}

int TemporaryTemp;
unsigned long TrouTempUpdate;
//长按按键1
void ONPress(){
    if(interface == 0 && CTemperature != 1024){
        if(Mode){
            Mode = false;
            Buzzer(150,50);
        }
        else{
            Buzzer(150,500);
            Mode = true;
            TrouTempUpdate = millis();
            TemporaryTemp = OTemperature;
        }
    }
}

//单击按键2
void SETClick(){
    //加热状态无法进入设置界面
    if(!Mode){
        Buzzer(150,20);
        Item = 0;
        interface ++;
        if(interface > 3){
            interface = 0;
            KeepSet();
        }
    }
}

//长按按键2
void SetPress(){
    if (interface != 0){
        Buzzer(150,20);
        interface = 0;
        KeepSet();
    }
}

//单击按键3
void FNClick(){
    switch(interface)
    {
        case 0:

            break;
        case 1:
            Item ++; if(Item > 2) Item = 0;
            Buzzer(150,20);
            break;
        case 2:
            Item ++; if(Item > 1) Item = 0;
            Buzzer(150,20);
            break;
        case 3:
            Item ++; if(Item > 1) Item = 0;
            Buzzer(150,20);
            break;
    }
}

void ButtonAttachLoop(){
    button_ON.tick();
    button_SET.tick();
    button_FN.tick();
}

void ButtonInitializat(){
    button_ON.reset();                      //清除一下按钮状态机的状态
    button_SET.reset();
    button_FN.reset();
    button_ON.setClickTicks(1);             //设置单击时长
    button_SET.setClickTicks(1);
    button_FN.setClickTicks(1);
    button_ON.setPressTicks(1500);          //设置长按时长
    button_SET.setPressTicks(400);          
    button_ON.attachClick(ONClick);         //初始化单击回调函数
    button_SET.attachClick(SETClick);
    button_FN.attachClick(FNClick);
    button_ON.attachLongPressStart(ONPress);    //初始化长按回调函数
    button_SET.attachLongPressStart(SetPress);  //初始化长按回调函数
}
unsigned long StorTempUpdate;
bool SaveSign = false;
void UpdateEC11(){
    //ledcWrite(1, 150);
    State = Encoder_EC11_Scan(digitalRead(EC11_DATA_K),digitalRead(EC11_DATA_A),digitalRead(EC11_DATA_B));
    if(State != 0 && inits){
        float Temporary;
        //微调
        if(State == 1)
            Temporary = 1;
        if(State == -1)
            Temporary = -1;

        //粗调
        if(State == 3)
            Temporary = 10;
        if(State == -3)
            Temporary = -10;
        switch (interface)
        {
            case 0:
                if(!Mode){
                    Temperature += Temporary;
                    if(Temperature > TEM_MAX)
                        Temperature = TEM_MAX;
                    if(Temperature < TEM_MIN)
                        Temperature = TEM_MIN;
                    Setpoint = Temperature;
                }
                SaveSign = true;
                StorTempUpdate = millis();
                break;

            case 1:
                switch (Item)
                {
                    case 0:
                        if(KP >= 100) Temporary *= 10;
                        KP += Temporary / 10;
                        if(KP > 999) KP = 999;
                        if(KP < 0) KP = 0;
                        break;
                    case 1:
                        if(KI >= 100) Temporary *= 10;
                        KI += Temporary / 10;
                        if(KI > 999) KI = 999;
                        if(KI < 0) KI = 0;
                        break;
                    case 2:
                        if(KD >= 100) Temporary *= 10;
                        KD += Temporary / 10;
                        if(KD > 999) KD = 999;
                        if(KD < 0) KD = 0;
                        break;
                }
                break;

            case 2:
                switch (Item)
                {
                    case 0:
                        if(KALMAN_Q >= 100) Temporary *= 10;
                        KALMAN_Q += Temporary / 10;
                        if(KALMAN_Q > 999) KALMAN_Q = 999;
                        if(KALMAN_Q < 0) KALMAN_Q = 0;
                        break;
                    case 1:
                        if(KALMAN_R >= 100)Temporary *= 10;
                        KALMAN_R += Temporary / 10;
                        if(KALMAN_R > 999)KALMAN_R = 999;
                        if(KALMAN_R < 0)KALMAN_R = 0;
                        break;
                }
                break;

            case 3:
                switch (Item)
                {
                    case 0:
                        Unit += Temporary;
                        if(Unit > 1) Unit = 0;
                        if(Unit < 0) Unit = 1;
                        break;
                    case 1:
                        if(OverEnable) 
                            OverEnable = false;
                        else 
                            OverEnable = true;
                        break;
                }
                break;
        }
    }
    //ledcWrite(1, 0);
}

void Insurance(){
    if(OverEnable){
        OverTemp = true;
        u8g2.clearBuffer();
        u8g2.setFont(u8g2_font_logisoso16_tf);
        u8g2.drawStr(13,16,"TEMP > 65C");
        u8g2.drawStr(0,32,"Please Reboot");
        u8g2.sendBuffer();
    }
}//过温保护

void StorageTempe() {
  if ((millis() - StorTempUpdate) > 10000 && SaveSign) {
    ESP_ERROR_CHECK( nvs_open("Customer data", NVS_READWRITE, &Handle) );
    uint32_t T1 = 32;char T[32] = {0};
    ESP_ERROR_CHECK ( nvs_get_str(Handle, "ST", T , &T1) );
    if(atoi(T) != Temperature){
        ESP_ERROR_CHECK( nvs_set_str( Handle, "ST", (char*)doubleToString(Temperature,"%.0lf").c_str()) );
        ESP_ERROR_CHECK( nvs_commit(Handle) );
    }
    SaveSign = false;
    nvs_close(Handle);
    StorTempUpdate = millis();
  }
}//自动保存设置温度

void setup() {
    Serial.begin(115200);                           //设置串口波特率
    u8g2.begin();                                   //启动OLED
    u8g2.clearBuffer();                             //显示开机画面
    u8g2.drawXBM(0, 0, 128, 32, StartupScreen);
    u8g2.sendBuffer();                              //写入屏幕
    setCpuFrequencyMhz(240);                        //以最高频率运行
    LoadConfig();
    pinMode(EC11_DATA_K,INPUT_PULLUP);                  //输入上拉
    pinMode(EC11_DATA_A,INPUT_PULLUP);
    pinMode(EC11_DATA_B,INPUT_PULLUP); 
    pinMode(FN_INS,INPUT_PULLUP); 
    EC11_A_Last = digitalRead(EC11_DATA_A);             //刷新编码器开机状态
    EC11_B_Last = digitalRead(EC11_DATA_B);
    OverTemp = digitalRead(FN_INS);                     //检测是否过热
    attachInterrupt(EC11_DATA_A, UpdateEC11, CHANGE);   //配置编码器引脚中断
    attachInterrupt(FN_INS, Insurance, HIGH);
    StorTempUpdate = millis();
    delay(500);                                     //等待MAX6675芯片稳定
    CTemperature =                                  //得到原始温度
    Thermocouple.readCelsius();
    Initial_K(CTemperature);                        //滤波器初始温度
    OTemperature =                                  //得到滤波温度
    KalmanFilter(CTemperature,KALMAN_Q,KALMAN_R);
    ledcSetup(0, 2000, 8);                          //配置PWM 功能,通道、频率、分辨率
    ledcAttachPin(PwmPin, 0);                       //将通道附加到指定IO,IO引脚,通道
    ledcSetup(1, 2000, 8);                          //配置PWM 功能,通道、频率、分辨率
    ledcAttachPin(32, 1);                           //将通道附加到指定IO,IO引脚,通道
    ButtonInitializat();                            //初始化按钮检测
    MYPID.SetMode(AUTOMATIC);                       //开启PID
    MYPID.SetTunings(KP,KI,KD);                     //应用到新参数
    Setpoint = Temperature;                         //设定加热到目标温度
    delay(500);
    Buzzer(127,50);
    inits = true;                                   //初始化完成标志
}//配置

void Troubleshoot(){                                //当温度检测发生故障时停止输出(阈值20秒)
  if ((millis() - TrouTempUpdate) > 20000) {
    if (Mode && Output > 10 && Temperature - OTemperature > 20){
        if (!(OTemperature > TemporaryTemp + 5)){
            Mode = false;
        }
        TemporaryTemp = OTemperature;
    }
    TrouTempUpdate = millis();
  }
}//检测故障

unsigned long lastTempUpdate;                       //跟踪上次临时更新的时钟时间
bool UpdateTemperature() {
  if ((millis() - lastTempUpdate) > TEMP_READ_DELAY) {
    CTemperature = Thermocouple.readCelsius();      //获取温度读数
    StorageTempe();
    Troubleshoot();
    Serial.println(OverEnable);
    lastTempUpdate = millis();
    return true;
  }
  return false;
}//更新温度

unsigned long RefTempUpdate;    //跟踪上次临时更新的时钟时间
//温度，状态，PID输出，设置温度
void RefreshScreen(int Tempe,bool Mod ,double PID ,int Set) {
    
  if ((millis() - RefTempUpdate) > TEMP_READ_REFRE) {
    String Temprature,OUT,Modes,Sets,Units;
    int static Counter1,Counter2;
    int Temperatures = 0;

    switch (Unit)
    {
        case 0:
            Temperatures = OTemperature;
            Set = Set;
            Units = "C";
            break;
        case 1:
            Temperatures = 1.8 * OTemperature + 32;
            Set = 1.8 * Set + 32;
            Units = "F";
            break;
    }
    
    if(Temperatures > 999)
        Temperatures = 999;

    //加入前导零
    if(to_String(Temperatures).length() <= 2)
        Temprature = "0" + to_String(Temperatures);
    else
        Temprature = to_String(Temperatures);

    Sets = to_String(Set);

    OUT = doubleToString(PID,"%.0lf");
    //断线检测
    if(CTemperature >= 1024){
        Mode =false;        //停止输出
        Temprature = " OL"; //无穷
    }

    if(Mod) Modes = "Start";
    else    Modes = "Close";

    u8g2.clearBuffer();                             //清除缓冲区
    u8g2.setFont(u8g2_font_logisoso28_tf);          //加载字库
    u8g2.drawStr(0,31,(char*)Temprature.c_str());   //写入温度
    u8g2.drawXBM(56, 2, 12, 12, Symbol);
    u8g2.drawStr(67,31,(char*)Units.c_str());

    if(Mode){
        Counter1 ++;Counter2 ++;
        if (Counter1 > 49) Counter1 = 0;
        if (Counter2 > 73) Counter2 = 24;
        if(Tempe < Set){
            u8g2.drawXBM(87, 32 - Counter1 , 7, 8, Rise); 
            u8g2.drawXBM(87, 40 - Counter1 , 7, 8, Rise); 
            u8g2.drawXBM(87, 56 - Counter2 , 7, 8, Rise); 
            u8g2.drawXBM(87, 64 - Counter2 , 7, 8, Rise); 
        }
        else if(Tempe > Set){
            u8g2.drawXBM(87, Counter1 - 8 , 7, 8, Decl); 
            u8g2.drawXBM(87, Counter1 - 16 , 7, 8, Decl); 
            u8g2.drawXBM(87,  Counter2 - 32 , 7, 8, Decl); 
            u8g2.drawXBM(87,  Counter2 - 40 , 7, 8, Decl); 
        }
    }
    else{Counter1 = 0;Counter2 = 0;}

    u8g2.setFont(u8g2_font_6x13B_tf);                //加载字库   
    u8g2.drawStr(121,20,"%");
    u8g2.drawStr(121,31,(char*)Units.c_str());
    u8g2.drawStr(97,9,(char*)Modes.c_str());         //写值
    u8g2.drawStr(97,20,(char*)OUT.c_str());
    u8g2.drawStr(97,31,(char*)Sets.c_str());
    u8g2.sendBuffer();                               //写入屏幕
    RefTempUpdate = millis();
  }
}//更新屏幕

unsigned long SetPIDUpdate; //跟踪上次临时更新的时钟时间
//P,I,D
void SetPID(float P,float I,float D) {
    
  if ((millis() - SetPIDUpdate) > TEMP_READ_REFRE) {
    String SP,SI,SD;

    if(P < 10) SP = "0" + doubleToString(P,"%.1lf");
    else if(P < 100) SP = doubleToString(P,"%.1lf");
    else             SP = doubleToString(P,"%.0lf");

    if(I < 10) SI = "0" + doubleToString(I,"%.1lf");
    else if(I < 100) SI = doubleToString(I,"%.1lf");
    else             SI = doubleToString(I,"%.0lf");

    if(D < 10) SD = "0" + doubleToString(D,"%.1lf");
    else if(D < 100) SD = doubleToString(D,"%.1lf");
    else             SD = doubleToString(D,"%.0lf");
    
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_6x13B_tf);

    switch (Item)
    {
        case 0:
            u8g2.drawStr(58,10,"I");
            u8g2.drawStr(104,10,"D");  
            u8g2.drawBox(11, 0, 8, 11);
            u8g2.setDrawColor(0);
            u8g2.drawStr(12,10,"P"); 
            break;
        case 1:
            u8g2.drawStr(12,10,"P"); 
            u8g2.drawStr(104,10,"D"); 
            u8g2.drawBox(57, 0, 8, 11);
            u8g2.setDrawColor(0);
            u8g2.drawStr(58,10,"I"); 
            break;
        case 2:
            u8g2.drawStr(12,10,"P"); 
            u8g2.drawStr(58,10,"I"); 
            u8g2.drawBox(103, 0, 8, 11);
            u8g2.setDrawColor(0);
            u8g2.drawStr(104,10,"D"); 
            break;
    }

    u8g2.setDrawColor(255);
    u8g2.setFont(u8g2_font_logisoso16_tf);
    u8g2.drawStr(0,30,(char*)SP.c_str());
    u8g2.drawStr(46,30,(char*)SI.c_str());
    u8g2.drawStr(92,30,(char*)SD.c_str());
    u8g2.sendBuffer();                      //写入屏幕

    MYPID.SetTunings(KP,KI,KD);
    SetPIDUpdate = millis();
  }
}//更新PID

unsigned long SetQRUpdate; //跟踪上次临时更新的时钟时间
void SetQR(float Q,float R) {
    
  if ((millis() - SetQRUpdate) > TEMP_READ_REFRE) {
        String SQ,SR;

        if(Q < 10) SQ = "0" + doubleToString(Q,"%.1lf");
        else if(Q < 100) SQ = doubleToString(Q,"%.1lf");
        else             SQ = doubleToString(Q,"%.0lf");

        if(R < 10) SR = "0" + doubleToString(R,"%.1lf");
        else if(R < 100) SR = doubleToString(R,"%.1lf");
        else             SR = doubleToString(R,"%.0lf");
        
        u8g2.clearBuffer();
        u8g2.setFont(u8g2_font_6x13B_tf);

        switch (Item)
        {
            case 0:
                u8g2.drawStr(92,10,"R");
                u8g2.drawBox(27, 0, 8, 11);
                u8g2.setDrawColor(0);
                u8g2.drawStr(28,10,"Q");
                break;
            case 1:
                u8g2.drawStr(28,10,"Q");
                u8g2.drawBox(91, 0, 8, 11);
                u8g2.setDrawColor(0);
                u8g2.drawStr(92,10,"R");
                break;
        }

        u8g2.setDrawColor(255);
        u8g2.setFont(u8g2_font_logisoso16_tf);
        u8g2.drawStr(16,30,(char*)SQ.c_str());
        u8g2.drawStr(80,30,(char*)SR.c_str());
        u8g2.sendBuffer();                      //写入屏幕

        SetQRUpdate = millis();
    }
}//更新QR

//°图标
unsigned char Symbolmini[] U8X8_PROGMEM = { 0x18,0x7e,0x66,0xc3,0xc3,0x66,0x7e,0x18 };
unsigned long UniUpdate; //跟踪上次临时更新的时钟时间
void SetUniversal(){
  if ((millis() - UniUpdate) > TEMP_READ_REFRE) {

        u8g2.clearBuffer();
        u8g2.setFont(u8g2_font_6x13B_tf);
        switch (Item)
        {
            case 0:
                u8g2.drawStr(60,10,"Over");
                u8g2.drawBox(16, 0, 20, 11);
                u8g2.setDrawColor(0);
                u8g2.drawStr(17,10,"C/F");
                break;
            case 1:
                u8g2.drawStr(17,10,"C/F");
                u8g2.drawBox(59, 0, 26, 11);
                u8g2.setDrawColor(0);
                u8g2.drawStr(60,10,"Over");
                break;
        }

        u8g2.setDrawColor(255);
        u8g2.setFont(u8g2_font_logisoso16_tf);
        u8g2.drawXBM(17, 14, 8, 8, Symbolmini);

        if (Unit == 0) u8g2.drawStr(26,30,"C");
        if (Unit == 1) u8g2.drawStr(26,30,"F");

        if (OverEnable)  u8g2.drawStr(62,30,"ON");
        if (!OverEnable) u8g2.drawStr(57,30,"OFF");

        u8g2.sendBuffer();                      //写入屏幕

        UniUpdate = millis();
  }

}//通用设置

void loop() {

    if(OverTemp && OverEnable){
        ledcWrite(0, 0); Insurance(); delay(1000);
        return;
    }

    if (Serial.available() > 0)     //从串口接收到数据
    {
        //ledcWrite(ledChannel, Serial.readStringUntil('\n').toInt());
        //Input = Serial.readStringUntil('\n').toInt();
        //Serial.println(OTemperature);
    }
    PlayBuzzer();
    if(UpdateTemperature()){
        OTemperature =
        KalmanFilter(CTemperature,KALMAN_Q,KALMAN_R);
        if(Mode){
            if(OTemperature < Temperature - 15)
                Output = 255;
            else if(OTemperature >= Temperature + 0.6)
                Output = 0;
            else{
                Input = OTemperature;
                MYPID.Compute();
            }
        }
        else{ Output = 0;}
        ledcWrite(0, Output);
    }

    switch (interface)
    {
        case 0:
            RefreshScreen(OTemperature,Mode,(Output / 255) * 100,Temperature);
            break;

        case 1:
            SetPID(KP,KI,KD);
            break;

        case 2:
            SetQR(KALMAN_Q,KALMAN_R);
            break;
        case 3:
            SetUniversal();
            break;
        default:
            break;
    }
    ButtonAttachLoop();
}