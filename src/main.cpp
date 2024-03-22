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
//#include <avr/io.h>
#include "diyServo.h"

diyServo miniServo(A2,10,12);
//diyServo mini2Servo(A2,PORTB0,PORTB2);
void lightLED(byte speed, byte speedXus);

//char forwardStr[] = "Forward speed = ";
//char backwardStr[] = "Backward speed = ";



void setup() {
  //PORTD = 1 << PORTD3; // 1<<PD3, 1左移3位得到3位得到二进制数 0000 0100

  pinMode(LED_BUILTIN, OUTPUT);//build in led on PB5 (Arduino pin D13)
  //Serial.begin(9600);
  //initServo(greater, smaller);
  miniServo.ctrlServo(512);
  //microServo1(A2,10,12);
}

void loop() {
  digitalWrite(LED_BUILTIN, HIGH);
  //repeat
  //int motorRunTime = 0;
  //int cwd = microServo1.readAnalogData(A0);
  //microServo1.ctrlServo(A2,10,12,512);
  int dest = miniServo.readAnalogData(A2, 10);

  miniServo.ctrlServo(dest);

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
