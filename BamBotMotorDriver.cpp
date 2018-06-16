#include "Arduino.h"
#include "BamBotMotorDriver.h"
#include <Adafruit_MCP23008.h>

//Define the PWM Channels, frequency and precision
byte BamBotMotorDriver::_M1CHN = 1;
byte BamBotMotorDriver::_M2CHN = 2;
byte BamBotMotorDriver::PWM_PRECISION = 13;
int BamBotMotorDriver::PWM_FREQUENCY = 5000;

bool BamBotMotorDriver::_USING_MCP;
Adafruit_MCP23008 BamBotMotorDriver::_MCP;

//Motor pins and direction flags
byte BamBotMotorDriver::_M1DIR;
byte BamBotMotorDriver::_M2DIR;
byte BamBotMotorDriver::_M1PWM;
byte BamBotMotorDriver::_M2PWM;
bool BamBotMotorDriver::_flipM1 = false;
bool BamBotMotorDriver::_flipM2 = false;

//Encoders
BamBotEncoder BamBotMotorDriver::encoder1;
BamBotEncoder BamBotMotorDriver::encoder2;



//initialize the motor driver assuming all four pins are connected 
//directly to the M5Stack
void BamBotMotorDriver::init(byte M1PWM, 
										byte M1DIR, 
										byte M2PWM, 
										byte M2DIR)
{
	_USING_MCP = false;

	//Record the motor pins in private variables
	_M1PWM = M1PWM;
	_M1DIR = M1DIR;
	_M2PWM = M2PWM;
	_M2DIR = M2DIR;

  // I drive the pins low here over and over because the Pololu driver does
  //this to avoid ever driving the pins high by mistake. It seems there are
  //certain boards that will do this when pinmode changes. I could
  //probably omit this, but better to be safe than sorry. See the Pololu
  //DRV8835 library (initPinsAndMaybeTimer() function) for more info.
  digitalWrite(_M1PWM, LOW);
  pinMode(_M1PWM, OUTPUT);
  digitalWrite(_M1PWM, LOW);
  digitalWrite(_M2PWM, LOW);
  pinMode(_M2PWM, OUTPUT);
  digitalWrite(_M2PWM, LOW);
  digitalWrite(_M1DIR, LOW);
  pinMode(_M1DIR, OUTPUT);
  digitalWrite(_M1DIR, LOW);
  digitalWrite(_M2DIR, LOW);
  pinMode(_M2DIR, OUTPUT);
  digitalWrite(_M2DIR, LOW);
  
  //Initialize the PWM channels
  //use the ESP32 function ledc to output PWM
  ledcSetup(_M1CHN, PWM_FREQUENCY, PWM_PRECISION);
  ledcAttachPin(_M1PWM, _M1CHN);
  ledcSetup(_M2CHN, PWM_FREQUENCY, PWM_PRECISION);
  ledcAttachPin(_M2PWM, _M2CHN);
}

//initialize the motor driver assuming the direction pins are connected to the MCP
void BamBotMotorDriver::init(Adafruit_MCP23008 mcp,
										byte M1PWM, 
										byte M1DIR, 
										byte M2PWM, 
										byte M2DIR)
{
	_USING_MCP = true;
	_MCP = mcp;

	//Record the motor pins in private variables
	_M1PWM = M1PWM;
	_M1DIR = M1DIR;
	_M2PWM = M2PWM;
	_M2DIR = M2DIR;

  // I drive the pins low here over and over because the Pololu driver does
  //this to avoid ever driving the pins high by mistake. It seems there are
  //certain boards that will do this when pinmode changes. I could
  //probably omit this, but better to be safe than sorry. See the Pololu
  //DRV8835 library (initPinsAndMaybeTimer() function) for more info.
  digitalWrite(_M1PWM, LOW);
  pinMode(_M1PWM, OUTPUT);
  digitalWrite(_M1PWM, LOW);
  digitalWrite(_M2PWM, LOW);
  pinMode(_M2PWM, OUTPUT);
  digitalWrite(_M2PWM, LOW);
  
  //initialize the direction pins on the MCP
  _MCP.pinMode(_M1DIR, OUTPUT);
  _MCP.pinMode(_M2DIR, OUTPUT);
  
  //Initialize the PWM channels
  //use the ESP32 function ledc to output PWM
  ledcSetup(_M1CHN, PWM_FREQUENCY, PWM_PRECISION);
  ledcAttachPin(_M1PWM, _M1CHN);
  ledcSetup(_M2CHN, PWM_FREQUENCY, PWM_PRECISION);
  ledcAttachPin(_M2PWM, _M2CHN);
}

// speed should be a number between -400 and 400
void BamBotMotorDriver::setM1Speed(int speed)
{
	//set reverse to true if one, but not both of these conditions are true
	bool reverse = ((speed < 0) ^ _flipM1);

	//constrain speed to be between 0 and 400
	speed = abs(speed);
	if (speed = 400) {
		speed = 400;
	}

	//set the speed
	_setPWM(_M1CHN, speed);

	//set the direction
	if (_USING_MCP) {
		_MCP.digitalWrite(_M1DIR, reverse);
	} else {
		digitalWrite(_M1DIR, reverse);
	}
}

// speed should be a number between -400 and 400
void BamBotMotorDriver::setM2Speed(int speed)
{
	//set reverse to true if one, but not both of these conditions are true
	bool reverse = ((speed < 0) ^ _flipM2);

	//constrain speed to be between 0 and 400
	speed = abs(speed);
	if (speed > 400) {
		speed = 400;
	}

	//set the speed
	_setPWM(_M2CHN, speed);
	
	//set the direction
	if (_USING_MCP) {
		_MCP.digitalWrite(_M2DIR, reverse);
	} else {
		digitalWrite(_M2DIR, reverse);
	}
}

// set speed for both motors
// speed should be a number between -400 and 400
void BamBotMotorDriver::setSpeeds(int m1Speed, int m2Speed){
  setM1Speed(m1Speed);
  setM2Speed(m2Speed);
}

void BamBotMotorDriver::flipM1(bool flip)
{
  _flipM1 = flip;
}

void BamBotMotorDriver::flipM2(bool flip)
{
  _flipM2 = flip;
}

void BamBotMotorDriver::_setPWM(uint8_t channel, uint32_t value) {
	uint32_t duty = (8191/400)*value;
	ledcWrite(channel, duty);
}

void BamBotMotorDriver::attachEncoders(byte pin1A, byte pin1B, byte pin2A, byte pin2B) {
	encoder1.init(pin1A, pin1B);
	encoder2.init(pin2A, pin2B);
}
