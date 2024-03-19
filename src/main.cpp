/**
 * @file main.cpp
 * @author M1ck3y (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2024-03-18
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include <Arduino.h>
#include "diyServo.h"

diyServo miniServo(A2,10,12);

void lightLED(byte speed, byte speedXus);

//char forwardStr[] = "Forward speed = ";
//char backwardStr[] = "Backward speed = ";


struct miniServos
{
  int analogChannels;
  int hBridgeIA;
  int hBridgeIB;
};
miniServos lineServo1Pin =  {A3, 5, 6};

//int lineServo1Pin[ 3 ] = {A3, 5, 6};
//int lineServo2[ 3 ] = {A4, 3, 9};

diyServo lineServo1(lineServo1Pin.analogChannels, lineServo1Pin.hBridgeIA, lineServo1Pin.hBridgeIB);
//diyServo lineServo1(lineServo1Pin[0], lineServo1Pin[1], lineServo1Pin.[2]);


void setup() {
   
  //diyServo microServo2();
  pinMode(LED_BUILTIN, OUTPUT);//build in led on PB5 (Arduino pin D13)
  //Serial.begin(9600);
  //initServo(greater, smaller);
  miniServo.ctrlServo(512);
  //microServo1(A2,10,12);
  lineServo1.ctrlServo(512);
}

void loop() {
  digitalWrite(LED_BUILTIN, HIGH);
  //repeat
  //int motorRunTime = 0;
  //int cwd = microServo1.readAnalogData(A0);
  //microServo1.ctrlServo(A2,10,12,512);
  int dest = miniServo.readAnalogData(A2, 10);
  int dest1 = lineServo1.readAnalogData(A4, 10);
  miniServo.ctrlServo(dest);
  lineServo1.ctrlServo(dest1);
//驱动以一个舵机后再驱动第二个舵机，延时 = 255*speed*30us
//

//重写一个能控制多个五线舵机的函数
/*
/// @brief 重新写一个初始化多个五线舵机的函数
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

  //currentPosition = analogRead(A2);
  //printDebug("currentPosition = ", currentPosition);
  //runMotor(currentPosition, greater, smaller);
  //lightLED(200);
  //int internal_pot = analogRead(A1); 
  //int purposePosition = readAnalogData(A0, 5);
  //int currentPosition = readAnalogData(A1, 5);
  //operateServo(purposePosition, currentPosition);
  //bool isFoward = external_pot <= offsetX;
  //Serial.print("isForward = ");
  //Serial.println(isFoward);
  //speed = external_pot==offsetX ? 0: (isFoward ? (255- map(external_pot,0,offsetX,0,255)): map(external_pot,0,1024,0,255));
  //Serial.println(speed);
  //isFoward ? forward() : backward();
  digitalWrite(LED_BUILTIN, LOW);
}

/// @brief 闪烁？
/// @param speed {0，255} 模拟PWM
/// @param speedXus 模拟点亮时间，默认30us
void lightLED(byte speed, byte speedXus) {
  digitalWrite(LED_BUILTIN, HIGH);
  delayMicroseconds(speed * speedXus);
  digitalWrite(LED_BUILTIN, LOW);
  delayMicroseconds((255 - speed) * speedXus);
}
