




#include <Arduino.h>
#include "regTest.h"


/// @brief 构造函数，初始化引脚模式，传递参数；是否需要初始化时使舵机摆臂归中？
/// @param analogChannel MCU读取内部反馈电位器ADC的引脚
/// @param L9110s_IA MCU驱动H桥的一个引脚 
/// @param L9110s_IB MCU驱动H桥的另一个引脚
regTest::regTest(int analogChannel, int L9110s_IA, int L9110s_IB) {
  internal_Potentiometer_ADC = analogChannel;
  hbridge_Ctrl_A = L9110s_IA;
  hbridge_Ctrl_B = L9110s_IB;
  int temp = 1 << DDD3;
  temp |= 1<< DDD5;
  DDRD = temp; //寄存器赋值，运行过程中是否应该用其他操作符，避免修改其他位的值。
  DDRD = 1 << hbridge_Ctrl_A;
  DDRD = 1 << hbridge_Ctrl_B;

}