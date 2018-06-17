#ifndef BamBotMotorDriver_h
#define BamBotMotorDriver_h

#include <Arduino.h>
#include <Adafruit_MCP23008.h>

//Default PWM channels, precision and frequency
#define M1CHN 1
#define M2CHN 2
#define PWM_PRECISION 13
#define PWM_FREQUENCY 5000

class BamBotMotorDriver
{
	public:	  
		void init(byte M1Pwm = 3,
						byte M1Dir = 1, 
						byte M2Pwm = 17, 
						byte M2Dir = 16);
		void init(Adafruit_MCP23008 mcp,
						byte M1Pwm = 3,
						byte M1Dir = 7, 
						byte M2Pwm = 1, 
						byte M2Dir = 6);
		void setM1Speed(int speed);
		void setM2Speed(int speed);
		void setSpeeds(int m1Speed, int m2Speed);
		void flipM1(boolean flip);
		void flipM2(boolean flip);
		 
		 //Encoder functions
		void attachEncoders(byte pin1A, byte pin1B, byte pin2A, byte pin2B);
		int motor1Position();
		int motor2Position();
		void motor1ResetPosition();
		void motor2ResetPosition();

	private:
		void _setPWM(uint8_t channel, uint32_t value);
		bool _USING_MCP;
		Adafruit_MCP23008 _MCP;
		byte _M1Dir;
		byte _M2Dir;
		byte _M1Pwm;
		byte _M2Pwm;
		bool _flipM1;
		bool _flipM2;

		static byte _enc1PinA;
		static byte _enc1PinB;
		static byte _enc2PinA;
		static byte _enc2PinB;

		//Anything referenced by an ISR must be declared volatile to prevent the compiler
		//from applying optimizations that will keep the value from being properly updated
		static volatile int _position1;
		static volatile int _position2;
		static volatile byte _state1;
		static volatile byte _state2;

		static portMUX_TYPE _mux;

		//Interrupt service routine
		static void IRAM_ATTR _updatePosition();
};

#endif
