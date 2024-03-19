/**
 * @file multiServo.h
 * @author M1ck3y (mickeyevaii@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2024-03-19
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef  _MULTISERVO_H_
#define  _MULTISERVO_H_
#define  VERSION = 0.1

#include "Arduino.h"


struct miniServos
{
  int analogChannels;
  int hBridgeIA;
  int hBridgeIB;
};
miniServos lineServo1Pin =  {A3, 5, 6};

/// @brief 控制五线舵机的相应H桥引脚与内部相联的电位器ADC值
class multiServo {
  private:
    int internal_Potentiometer_ADC; //arduino用于获得舵机内部反馈电位器ADC值的引脚编号
    int hbridge_Ctrl_A; //arduino控制H桥的引脚编号
    int hbridge_Ctrl_B; //arduino控制H桥的引脚编号

  public:
    multiServo(int analogChannel, int L9110s_IA, int L9110s_IB); //构造函数
    ~multiServo(); //默认析构函数

    //diyServo(int analogChannels[], int L9110s_IAs[],int L9110s_IBs[]);
    //重新写一个初始化多个五线舵机的构造函数

    void disattach(int); //释放引脚与LED的绑定，使得引脚可以控制其他的东西
    int readAnalogData(int, int);
    //void initServo(); //上电后是舵机摆臂回到归中位置，是否可以不用传入参数
    
    void ctrlServo(int); //根据外部输入ADC值或PWM转化来的ADC值，移动舵机摆臂到目的位置

    void forward();
    void backward();
    void standby();
    void brake();
};

#endif