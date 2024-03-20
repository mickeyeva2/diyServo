



#ifndef  _REGTEST_H_
#define  _REGTEST_H_
#define  VERSION = 0.1

#include "Arduino.h"

class regTest {
  public:
    int internal_Potentiometer_ADC; //arduino用于获得舵机内部反馈电位器ADC值的引脚编号
    int hbridge_Ctrl_A; //arduino控制H桥的引脚编号
    int hbridge_Ctrl_B; //arduino控制H桥的引脚编号
        
    regTest(int analogChannel, int L9110s_IA, int L9110s_IB); //构造函数
    ~regTest(); //默认析构函数

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

  private:
  
  };
#endif


//motorRun

  //
  //PORTD |= 0x0E;
  //0x0E 二进制 00001110
  // Set D2, D3, and D4 to HIGH

  // Set D5, D6, and D7 to LOW
  //PORTD &= ~0xF1;
  //       8 4 2 1 8 4 2 1
  //       11110001 ，00001110；
  //二进制 11110001
  //       D7 D6 D5 D4 D3 D2 D1 D0


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