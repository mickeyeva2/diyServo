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
//servoA = servoArray[0]
//servoB = servoArray[1]
//servoC = servoArray[2]

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
    int val = cp1 - destinationADC[0];
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
    cp1 = readAnalogData(internal_Potentiometer_ADC, adcRound);
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
  digitalWrite(hbridge_Ctrl_A, 0x1);
  digitalWrite(hbridge_Ctrl_B, HIGH);
}

//motorRun
void motorRun(int pins[], int statsOfPins[]) {
  //
  PORTD |= 0x0E;
  //0x0E 二进制 00001110
  // Set D2, D3, and D4 to HIGH

  // Set D5, D6, and D7 to LOW
  PORTD &= ~0xF1;
  //       8 4 2 1 8 4 2 1
  //       11110001 ，00001110；
  //二进制 11110001
  //       D7 D6 D5 D4 D3 D2 D1 D0
}

// | 按位或运算，有1得1，全0得0，
// & 按位与运算,有0得0，全1得1.
/*
Arduino Pro Mini 的数字端口如下：
    **端口 D：**D0、D1、D2、D3、D4、D5、D6、D7
    **端口 B：**D8、D9、D10、D11、D12、D13

以下是 Arduino Pro Mini 数字端口的功能：
    **数字端口：**可以配置为输入或输出。
    **PWM 端口：**D3、D5、D6、D9、D10、D11 可以用于生成脉冲宽度调制信号。
    DDRD  PORTD的数据方向寄存器

    DDRD = 1 << DDD3;
    DDRD |= 0x04;
    等效


    PORTD = 1 << PD3;

以下是 Arduino Pro Mini 数字端口的详细信息：

端口 D

    **D0：**RX 引脚，用于串行通信。
    **D1：**TX 引脚，用于串行通信。
    **D2：**可配置为输入或输出。
    **D3：**可配置为输入或输出，也可用于 PWM 输出。
    **D4：**可配置为输入或输出。
    **D5：**可配置为输入或输出，也可用于 PWM 输出。
    **D6：**可配置为输入或输出，也可用于 PWM 输出。
    **D7：**可配置为输入或输出。

端口 B

    **D8：**可配置为输入或输出。
    **D9：**可配置为输入或输出，也可用于 PWM 输出。
    **D10：**可配置为输入或输出，也可用于 PWM 输出。
    **D11：**可配置为输入或输出，也可用于 PWM 输出。
    **D12：**可配置为输入或输出。
    **D13：**可配置为输入或输出。

  DDRB |= 0x05; //0b 0000 0101

  DDRB = 1 << DDB0;
  DDRB = 1 << DDB2;
*/

//配置为高电平时用按位或，有1得1，全0得0.

//配置为低电平时用按位与？ 