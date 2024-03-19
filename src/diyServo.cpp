/**
 * @file diyServo.cpp
 * @author M1ck3y (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2024-03-18
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "diyServo.h"
#include <Arduino.h>

int arraySize = 10;
//int middlePosition = 512;

/// @brief 构造函数，初始化引脚模式，传递参数；是否需要初始化时使舵机摆臂归中？
/// @param analogChannel MCU读取内部反馈电位器ADC的引脚
/// @param L9110s_IA MCU驱动H桥的一个引脚
/// @param L9110s_IB MCU驱动H桥的另一个引脚
diyServo::diyServo(int analogChannel, int L9110s_IA, int L9110s_IB) {
  internal_Potentiometer_ADC = analogChannel;
  hbridge_Ctrl_A = L9110s_IA;
  hbridge_Ctrl_B = L9110s_IB;
  pinMode(hbridge_Ctrl_A, OUTPUT);
  pinMode(hbridge_Ctrl_B, OUTPUT);
}

/*
/// @brief 重新写一个初始化多个五线舵机的构造函数
/// @param analogChannels[] 若干个MCU读取内部反馈电位器ADC的引脚
/// @param L9110s_IAs[] 若干个MCU驱动H桥的一个引脚
/// @param L9110s_IBs[] 若干个MCU驱动H桥的另一个引脚
/// @note analogChannels[] = {A2,A0,A1}
/// @note L9110s_IAs[] = {D10,D5,D3}
/// @note L9110s_IBs[] = {D12,D6,D9}
//struct {int analogChannels[];int L9110s_IAs[];int L9110s_IBs[];};
diyServo::diyServo(int analogChannels[], int L9110s_IAs[],int L9110s_IBs[]){

}   
*/

/// @brief 析构函数，调用成员函数disattach()，拉低引脚电平并设置为INPUT模式
diyServo::~diyServo() {
  disattach(hbridge_Ctrl_A);
  disattach(hbridge_Ctrl_B);
}

/// @brief 为了获得相对稳定的电位器模数转换值，默认读取10次，取中间8个的平均值
/// @param analogChannel 电位器读数引脚对应的MCU引脚，arduino的A0~A7
/// @param arraySize 自定义读取次数，默认读取10次
/// @return 默认取中间8个的平均值
int diyServo::readAnalogData(int analogChannel, int arraySize) {
  //internal_Potentiometer_ADC = analogChannel;
  int analogData[arraySize];
  for(int i=0;i<arraySize;i++) {
    analogData[i]=analogRead(analogChannel);
    //printf("Internal Poot data is %d", analogData[i]);
  }
  //bubblesort and math the average data
  int temp = 0;
  bool iSwap = true;
  while(iSwap) {
    iSwap = false;
    for(int i=0;i<arraySize-1;i++) {    
      if(analogData[i] > analogData[i+1]){
        iSwap = true;
        temp = analogData[i];               
        analogData[i] = analogData[i+1];               
        analogData[i+1] = temp;           
      }
    }
  }

  temp = 0;
  for(int i=1;i<arraySize-1;i++) {
  // except analogData[0] and analogData[arraySize-1], average for 1,2,3...arrayData[arraySize-2]
    temp += analogData[i];
  }
  temp = temp/(arraySize-2); //求平均值
  return temp;
}

/*
/// @brief 初始化五线舵机，使摆臂归中
void diyServo::initServo() {
//the motor drive the internal pot, when the pot reach the middle position motor stop.
  int currentPosition = readAnalogData(arraySize);
  while (currentPosition > middlePosition || currentPosition < middlePosition) {
    int val = currentPosition - middlePosition; //512
    byte speed = map(abs(val),0,middlePosition,0,255);
    if(val > 0) forward();
    if(val < 0) backward();
    if(speed < 10) speed = 0; //10 {0，255}相当于+40，-40{0，1023}，反馈电位器位于472与552之间认为是摆臂归中的。
    delayMicroseconds(speed * 30);
    standby();
    delayMicroseconds((255 - speed) * 30);
    currentPosition = readAnalogData(arraySize);
  }
}
*/

/// @brief 根据外部输入ADC值或PWM转化来的ADC值，移动舵机摆臂到目的位置
/// @param destinationADC 外部输入的ADC值（可以是PWM值转化而来）{0，1023}
/// @note destinationADC = middlePosition,即为上方的初始化无线舵机函数diyServo::initServo
void diyServo::ctrlServo(int destinationADC) {
  int currentPosition = readAnalogData(internal_Potentiometer_ADC,arraySize);
  while (currentPosition > destinationADC || currentPosition < destinationADC) {
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
    currentPosition = readAnalogData(internal_Potentiometer_ADC, arraySize);
  }
}

/// @brief 释放引脚与hbridgePin的绑定，使得引脚可以控制其他的东西
/// @param hbridgePin 相应的H桥控制引脚
void diyServo::disattach(int hbridgePin) {
  digitalWrite(hbridgePin, LOW);
  pinMode(hbridgePin,INPUT);
}

/// @brief IA=0,IB=1;前进
void diyServo::forward() {
  digitalWrite(hbridge_Ctrl_A, HIGH);
  digitalWrite(hbridge_Ctrl_B, LOW);
}
/// @brief IA=1,IB=0;后退
void diyServo::backward() {
  digitalWrite(hbridge_Ctrl_A, LOW);
  digitalWrite(hbridge_Ctrl_B, HIGH);
}
/// @brief IA=0,IB=0;待机
void diyServo::standby() {
  digitalWrite(hbridge_Ctrl_A, LOW);
  digitalWrite(hbridge_Ctrl_B, LOW);
}
/// @brief IA=1,IB=1;刹车
void diyServo::brake() {
  digitalWrite(hbridge_Ctrl_A, HIGH);
  digitalWrite(hbridge_Ctrl_B, HIGH);
}


/*
void diyServo::forward(int L9110s_IA,int L9110s_IB) {
  hbridge_Ctrl_A = L9110s_IA;
  hbridge_Ctrl_B = L9110s_IB;
  digitalWrite(hbridge_Ctrl_A, HIGH);
  digitalWrite(hbridge_Ctrl_B, LOW);
}

void diyServo::backward(int L9110s_IA,int L9110s_IB) {
  hbridge_Ctrl_A = L9110s_IA;
  hbridge_Ctrl_B = L9110s_IB;
  digitalWrite(hbridge_Ctrl_A, LOW);
  digitalWrite(hbridge_Ctrl_B, HIGH);
}

void diyServo::standby(int L9110s_IA,int L9110s_IB) {
  hbridge_Ctrl_A = L9110s_IA;
  hbridge_Ctrl_B = L9110s_IB;
  digitalWrite(hbridge_Ctrl_A, LOW);
  digitalWrite(hbridge_Ctrl_B, LOW);
}

void diyServo::brake(int L9110s_IA,int L9110s_IB) {
  hbridge_Ctrl_A = L9110s_IA;
  hbridge_Ctrl_B = L9110s_IB;
  digitalWrite(hbridge_Ctrl_A, HIGH);
  digitalWrite(hbridge_Ctrl_B, HIGH);
}

*/