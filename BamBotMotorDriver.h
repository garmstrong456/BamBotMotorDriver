#ifndef BamBotMotorDriver_h
#define BamBotMotorDriver_h

#include <Arduino.h>
#include <Adafruit_MCP23008.h>
#include "BamBotEncoder.h"

class BamBotMotorDriver
{
  public:
    static void init(byte M1PWM = 3,
    						byte M1DIR = 1, 
    						byte M2PWM = 17, 
    						byte M2DIR = 16);
    static void init(Adafruit_MCP23008 mcp,
    						byte M1PWM = 3,
    						byte M1DIR = 7, 
    						byte M2PWM = 1, 
    						byte M2DIR = 6);
    static void setM1Speed(int speed);
    static void setM2Speed(int speed);
    static void setSpeeds(int m1Speed, int m2Speed);
    static void flipM1(boolean flip);
    static void flipM2(boolean flip);
    static void attachEncoders(byte pin1A, byte pin1B, byte pin2A, byte pin2B);
    static BamBotEncoder encoder1;
    static BamBotEncoder encoder2;
  
  private:
    static void _setPWM(uint8_t channel, uint32_t value);
    static bool _USING_MCP;
    static Adafruit_MCP23008 _MCP;
    static byte _M1DIR;
    static byte _M2DIR;
    static byte _M1PWM;
    static byte _M2PWM;
    static byte _M1CHN;
    static byte _M2CHN;
    static int PWM_FREQUENCY;
    static byte PWM_PRECISION;
    static bool _flipM1;
    static bool _flipM2;
};

#endif
