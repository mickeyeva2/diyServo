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

/*
 int 3diyservo[3][3] = {
    {A2, 10, 12},
    {A3, 5, 6},
    {A1, 3, 9}
  };
*/


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


}



/// @brief 为了获得相对稳定的电位器模数转换值，默认读取10次，取中间8个的平均值
/// @param analogChannel 电位器读数引脚对应的MCU引脚，arduino的A0~A7
/// @param arraySize 自定义读取次数，默认读取10次
/// @return 默认取中间8个的平均值
int multiServo::readAnalogData(int analogChannel, int arraySize) {
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