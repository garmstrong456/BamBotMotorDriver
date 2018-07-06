#include "Arduino.h"
#include "BamBotMotorDriver.h"
#include <Adafruit_MCP23008.h>


/*******************
Motor Initialization
********************/

//initialize the motor driver assuming all four pins are connected 
//directly to the M5Stack
void BamBotMotorDriver::init(byte M1Pwm, 
										byte M1Dir, 
										byte M2Pwm, 
										byte M2Dir)
{
	_USING_MCP = false;

	//Record the motor pins in private variables
	_M1Pwm = M1Pwm;
	_M1Dir = M1Dir;
	_M2Pwm = M2Pwm;
	_M2Dir = M2Dir;
	
	_flipM1 = false;
	_flipM2 = false;

  // I drive the pins low here over and over because the Pololu driver does
  //this to avoid ever driving the pins high by mistake. It seems there are
  //certain boards that will do this when pinMode changes. I could
  //probably omit this, but better to be safe than sorry. See the Pololu
  //DRV8835 library (initPinsAndMaybeTimer() function) for more info.
  digitalWrite(_M1Pwm, LOW);
  pinMode(_M1Pwm, OUTPUT);
  digitalWrite(_M1Pwm, LOW);
  digitalWrite(_M2Pwm, LOW);
  pinMode(_M2Pwm, OUTPUT);
  digitalWrite(_M2Pwm, LOW);
  digitalWrite(_M1Dir, LOW);
  pinMode(_M1Dir, OUTPUT);
  digitalWrite(_M1Dir, LOW);
  digitalWrite(_M2Dir, LOW);
  pinMode(_M2Dir, OUTPUT);
  digitalWrite(_M2Dir, LOW);
  
  //Initialize the PWM channels
  //use the ESP32 function ledc to output PWM
  ledcSetup(M1CHN, PWM_FREQUENCY, PWM_PRECISION);
  ledcAttachPin(_M1Pwm, M1CHN);
  ledcSetup(M2CHN, PWM_FREQUENCY, PWM_PRECISION);
  ledcAttachPin(_M2Pwm, M2CHN);
}

//initialize the motor driver assuming the direction pins are connected to the MCP
void BamBotMotorDriver::init(Adafruit_MCP23008 mcp,
										byte M1Pwm, 
										byte M1Dir, 
										byte M2Pwm, 
										byte M2Dir)
{
	_USING_MCP = true;
	_MCP = mcp;

	//Record the motor pins in private variables
	_M1Pwm = M1Pwm;
	_M1Dir = M1Dir;
	_M2Pwm = M2Pwm;
	_M2Dir = M2Dir;
	
	_flipM1 = false;
	_flipM2 = false;

  // I drive the pins low here over and over because the Pololu driver does
  //this to avoid ever driving the pins high by mistake. It seems there are
  //certain boards that will do this when pinmode changes. I could
  //probably omit this, but better to be safe than sorry. See the Pololu
  //DRV8835 library (initPinsAndMaybeTimer() function) for more info.
  digitalWrite(_M1Pwm, LOW);
  pinMode(_M1Pwm, OUTPUT);
  digitalWrite(_M1Pwm, LOW);
  digitalWrite(_M2Pwm, LOW);
  pinMode(_M2Pwm, OUTPUT);
  digitalWrite(_M2Pwm, LOW);
  
  //initialize the direction pins on the MCP
  _MCP.pinMode(_M1Dir, OUTPUT);
  _MCP.pinMode(_M2Dir, OUTPUT);
  
  //Initialize the PWM channels
  //use the ESP32 function ledc to output PWM
  ledcSetup(M1CHN, PWM_FREQUENCY, PWM_PRECISION);
  ledcAttachPin(_M1Pwm, M1CHN);
  ledcSetup(M2CHN, PWM_FREQUENCY, PWM_PRECISION);
  ledcAttachPin(_M2Pwm, M2CHN);
}

/**********
Motor Drive
***********/

// speed should be a number between -400 and 400
void BamBotMotorDriver::setM1Speed(int speed)
{
	//set reverse to true if one, but not both of these conditions are true
	bool reverse = (!(speed < 0) != !_flipM1);

	//constrain speed to be between 0 and 400
	speed = abs(speed);
	if (speed > 400) {
		speed = 400;
	}

	//set the speed
	_setPWM(M1CHN, speed);

	//set the direction
	if (_USING_MCP) {
		_MCP.digitalWrite(_M1Dir, reverse);
	} else {
		digitalWrite(_M1Dir, reverse);
	}
}

// speed should be a number between -400 and 400
void BamBotMotorDriver::setM2Speed(int speed)
{
	//set reverse to true if one, but not both of these conditions are true
	bool reverse = (!(speed < 0) != !_flipM2);

	//constrain speed to be between 0 and 400
	speed = abs(speed);
	if (speed > 400) {
		speed = 400;
	}

	//set the speed
	_setPWM(M2CHN, speed);
	
	//set the direction
	if (_USING_MCP) {
		_MCP.digitalWrite(_M2Dir, reverse);
	} else {
		digitalWrite(_M2Dir, reverse);
	}
}

// set speed for both motors
// speed should be a number between -400 and 400
void BamBotMotorDriver::setSpeeds(int m1Speed, int m2Speed){
  setM1Speed(m1Speed);
  setM2Speed(m2Speed);
}

//Set the reverse flag for the motors
void BamBotMotorDriver::flipM1(bool flip)
{
  _flipM1 = flip;
}

void BamBotMotorDriver::flipM2(bool flip)
{
  _flipM2 = flip;
}

//Private PWM function
//Set the PWM output using the ledcWrite function
void BamBotMotorDriver::_setPWM(uint8_t channel, uint32_t value) {
	
	uint32_t duty;
	if (value == 0) {
		duty = 0;
	} else {
		duty = (8191 - DEADBAND)*value/400 + DEADBAND;
	}
	ledcWrite(channel, duty);
}


/****************
Encoder Functions
*****************/

byte BamBotMotorDriver::_enc1PinA;
byte BamBotMotorDriver::_enc1PinB;
byte BamBotMotorDriver::_enc2PinA;
byte BamBotMotorDriver::_enc2PinB;

volatile int BamBotMotorDriver::_position1;
volatile int BamBotMotorDriver::_position2;
volatile byte BamBotMotorDriver::_state1;
volatile byte BamBotMotorDriver::_state2;

float BamBotMotorDriver::_enc1Cal;
float BamBotMotorDriver::_enc2Cal;

portMUX_TYPE BamBotMotorDriver::_mux = portMUX_INITIALIZER_UNLOCKED;

//Initialize the encoders
void BamBotMotorDriver::attachEncoders(byte pin1A, byte pin1B, byte pin2A, byte pin2B) {

	//Record the encoder pins
	_enc1PinA = pin1A;
	_enc1PinB = pin1B;
	_enc2PinA = pin2A;
	_enc2PinB = pin2B;
	
	//Initialize the position and state variables
	_position1 = 0;
	_position2 = 0;
	_state1 = 0;
	_state2 = 0;
	
	//Initialize the encoder pins
	pinMode(_enc1PinA, INPUT_PULLUP);
	pinMode(_enc1PinB, INPUT_PULLUP);
	pinMode(_enc2PinA, INPUT_PULLUP);
	pinMode(_enc2PinB, INPUT_PULLUP);
	
	//Attach interrupts to moniter when the pins change state
	attachInterrupt(digitalPinToInterrupt(_enc1PinA), _updatePosition, CHANGE);
	attachInterrupt(digitalPinToInterrupt(_enc1PinB), _updatePosition, CHANGE);
	attachInterrupt(digitalPinToInterrupt(_enc2PinA), _updatePosition, CHANGE);
	attachInterrupt(digitalPinToInterrupt(_enc2PinB), _updatePosition, CHANGE);
}

//Return the position for each motor
float BamBotMotorDriver::motor1Position() {
	return _position1 * _enc1Cal;
}

float BamBotMotorDriver::motor2Position() {
	return _position2 * _enc2Cal;
}

//Reset the position to 0
void BamBotMotorDriver::motor1ResetPosition() {
	_position1 = 0;
}

void BamBotMotorDriver::motor2ResetPosition() {
	_position2 = 0;
}

//Set calibrations

void BamBotMotorDriver::setEncoderCalibrations(float enc1Ca, float enc2Ca){
	_enc1Cal = enc1Ca;
	_enc2Cal = enc2Ca;
}


//This is where the position of the encoders are calculated based on the current and
//previous states. There's some clever bit math here that I lifted from Paul
//Stoffregen's Arduino Encoder library.
//Check out Encoder.h from that library for more documentation on what this is doing
//
//In order to find out how much the encoder moved you need to compare the previous state
//(pinA and pinB: 2 bits) + the current state (2 more bits). There are 16 possible cases
//(4 bits total) and this code checks each one individually then records the current
//state for the next call.
void BamBotMotorDriver::_updatePosition() {
	portENTER_CRITICAL_ISR(&_mux);
	
	int s;
	
	
	//Update encoder 1
	s = _state1 & 3;
	
	if (digitalRead(_enc1PinA)) s |= 4;
	if (digitalRead(_enc1PinB)) s |= 8;
	
	switch (s) {
		case 0: case 5: case 10: case 15:	//No movement
			break;
		case 1: case 7: case 8: case 14:		//one position forward
			_position1++;
			break;
		case 2: case 4: case 11: case 13:	//one position backward
			_position1--;
			break;
		case 3: case 12:							//two positions forward
			_position1 += 2;
			break;
		default:										//two positions backward
			_position1 -= 2;
			break;
	}
	
	_state1 = (s >> 2);
	
	
	//Update encoder 2
	s = _state2 & 3;

	if (digitalRead(_enc2PinA)) s |= 4;
	if (digitalRead(_enc2PinB)) s |= 8;
	
	switch (s) {
		case 0: case 5: case 10: case 15:	//No movement
			break;
		case 1: case 7: case 8: case 14:		//one position forward
			_position2++;
			break;
		case 2: case 4: case 11: case 13:	//one position backward
			_position2--;
			break;
		case 3: case 12:							//two positions forward
			_position2 += 2;
			break;
		default:										//two positions backward
			_position2 -= 2;
			break;
	}

	_state2 = (s >> 2);

	
	portEXIT_CRITICAL_ISR(&_mux);
}






