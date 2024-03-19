/**
 * @file multiServo.cpp
 * @author M1ck3y (mickeyevaii@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2024-03-19
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "Arduino.h"
#include "multiServo.h"

int servoArray[3][3] = {
    {A2, 10, 12},
    {A3, 5, 6},
    {A1, 3, 9}
};

struct miniServos
{
  int analogChannels;
  int hBridgeIA;
  int hBridgeIB;
};
miniServos lineServo1Pin =  {A3, 5, 6};
miniServos lineServo2Pin =  {A2, 3, 9};

multiServo::multiServo(int analogChannel, int L9110s_IA, int L9110s_IB) {
//构造函数
  internal_Potentiometer_ADC = analogChannel;
  hbridge_Ctrl_A = L9110s_IA;
  hbridge_Ctrl_B = L9110s_IB;
  pinMode(hbridge_Ctrl_A, OUTPUT);
  pinMode(hbridge_Ctrl_B, OUTPUT);
}

/// @brief 为了获得相对稳定的电位器模数转换值，默认读取10次，取中间8个的平均值
/// @param analogChannel 电位器读数引脚对应的MCU引脚，arduino的A0~A7
/// @param arraySize 自定义读取次数，默认读取10次
/// @return 默认取中间8个的平均值
int multiServo::readAnalogData(int analogChannel, int adcRound) {
  //internal_Potentiometer_ADC = analogChannel;
  int analogData[adcRound];
  for(int i=0;i<adcRound;i++) {
    analogData[i]=analogRead(analogChannel);
    //printf("Internal Poot data is %d", analogData[i]);
  }
  //bubblesort and math the average data
  int temp = 0;
  bool iSwap = true;
  while(iSwap) {
    iSwap = false;
    for(int i=0;i<adcRound-1;i++) {    
      if(analogData[i] > analogData[i+1]){
        iSwap = true;
        temp = analogData[i];               
        analogData[i] = analogData[i+1];               
        analogData[i+1] = temp;           
      }
    }
  }

}



/// @brief 根据外部输入ADC值或PWM转化来的ADC值，移动舵机摆臂到目的位置
/// @param destinationADC 外部输入的ADC值（可以是PWM值转化而来）{0，1023},数组，{adc1,adc2,adc3}
/// @param insidePotADC 内部反馈电位器ADC值，数组
/// @param adcRound ADC读取次数
/// @note destinationADC = middlePosition,即为上方的初始化无线舵机函数diyServo::initServo
void multiServo::ctrlServo(int destinationADC[],int insidePotADC[], int adcRound) {
  // cp0 = currentPosition1
  int cp1 = readAnalogData(insidePotADC[0], adcRound);
  int cp2 = readAnalogData(insidePotADC[1], adcRound);
  int cp3 = readAnalogData(insidePotADC[2], adcRound);


  while (cp1 > destinationADC[0] || cp1 < destinationADC[0]) {
    int val = currentPosition - destinationADC;
    if(abs(val)*2 > 1023) val = 1023;
    else val = abs(val)*2;
    byte speed = map(val,0,1023,0,255);
    //byte speed = map(abs(val),0,1023,0,255);

    if(val > 0) forward();
    if(val < 0) backward();
    if(speed < 10) speed = 0; //10 {0，255}相当于+40，-40{0，1023}，反馈电位器位于472与552之间认为是摆臂归中的。
    delayMicroseconds(speed * 30);
    standby();
    delayMicroseconds((255 - speed) * 30);
    currentPosition = readAnalogData(internal_Potentiometer_ADC, adcRound);
  }
}

//return direction And Speed
boolean multiServo::direction(int destadc, int insideadc,int adcRound) {
  boolean servoDirection;
  int currentPosition = readAnalogData(insideadc, adcRound);
  while (currentPosition > destadc || currentPosition < destadc) {
    int val = currentPosition - destadc;
    if(abs(val)*2 > 1023) val = 1023;
    else val = abs(val)*2;
    byte speed = map(val,0,1023,0,255);
    //byte speed = map(abs(val),0,1023,0,255);

    if(val > 0) servoDirection = true;
    if(val < 0) servoDirection = false;
    if(speed < 10) speed = 0; //10 {0，255}相当于+40，-40{0，1023}，反馈电位器位于472与552之间认为是摆臂归中的。
    delayMicroseconds(speed * 30);
    standby();
    delayMicroseconds((255 - speed) * 30);
    currentPosition = readAnalogData(insideadc, adcRound);
  }
  return servoDirection;
}

int multiServo::servoSpeed(int destadc, int insideadc,int adcRound) {
  boolean servoDirection;
  int currentPosition = readAnalogData(insideadc, adcRound);
  while (currentPosition > destadc || currentPosition < destadc) {
    int val = currentPosition - destadc;
    if(abs(val)*2 > 1023) val = 1023;
    else val = abs(val)*2;
    byte speed = map(val,0,1023,0,255);
    //byte speed = map(abs(val),0,1023,0,255);

    if(val > 0) servoDirection = true;
    if(val < 0) servoDirection = false;
    if(speed < 10) speed = 0; //10 {0，255}相当于+40，-40{0，1023}，反馈电位器位于472与552之间认为是摆臂归中的。
    delayMicroseconds(speed * 30);
    standby();
    delayMicroseconds((255 - speed) * 30);
    currentPosition = readAnalogData(insideadc, adcRound);
  }
  return servoDirection;
}



void multiServo::forward() {
  digitalWrite(hbridge_Ctrl_A, HIGH);
  digitalWrite(hbridge_Ctrl_B, LOW);
}
/// @brief IA=1,IB=0;后退
void multiServo::backward() {
  digitalWrite(hbridge_Ctrl_A, LOW);
  digitalWrite(hbridge_Ctrl_B, HIGH);
}
/// @brief IA=0,IB=0;待机
void multiServo::standby() {
  digitalWrite(hbridge_Ctrl_A, LOW);
  digitalWrite(hbridge_Ctrl_B, LOW);
}
/// @brief IA=1,IB=1;刹车
void multiServo::brake() {
  digitalWrite(hbridge_Ctrl_A, HIGH);
  digitalWrite(hbridge_Ctrl_B, HIGH);
}