#include "Arduino.h"
#include "BamBotMotorDriver.h"
#include <Adafruit_MCP23008.h>

/*
	Code here from https://github.com/espressif/arduino-esp32/blob/600f4c4130f2411e8ff33396d0ef40a7ec257cd6/cores/esp32/esp32-hal-ledc.c
*/

// Section copyright:
//
// Copyright 2015-2016 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "esp32-hal.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "rom/ets_sys.h"
#include "esp32-hal-matrix.h"
#include "soc/dport_reg.h"
#include "soc/ledc_reg.h"
#include "soc/ledc_struct.h"
#define LEDC_CHAN(g,c) LEDC.channel_group[(g)].channel[(c)]
#define LEDC_TIMER(g,t) LEDC.timer_group[(g)].timer[(t)]
static void aWrite(uint8_t chan, uint32_t duty)
{
    if(chan > 15) return;
    uint8_t group=(chan/8), channel=(chan%8);
    //LEDC_MUTEX_LOCK();
    LEDC_CHAN(group, channel).duty.duty = duty << 4;//25 bit (21.4)
    if(duty) {
        LEDC_CHAN(group, channel).conf0.sig_out_en = 1;//This is the output enable control bit for channel
        LEDC_CHAN(group, channel).conf1.duty_start = 1;//When duty_num duty_cycle and duty_scale has been configured. these register won't take effect until set duty_start. this bit is automatically cleared by hardware.
        if(group) {
            LEDC_CHAN(group, channel).conf0.val |= BIT(4);
        } else {
            LEDC_CHAN(group, channel).conf0.clk_en = 1;
        }
    } else {
        LEDC_CHAN(group, channel).conf0.sig_out_en = 0;//This is the output enable control bit for channel
        LEDC_CHAN(group, channel).conf1.duty_start = 0;//When duty_num duty_cycle and duty_scale has been configured. these register won't take effect until set duty_start. this bit is automatically cleared by hardware.
        if(group) {
            LEDC_CHAN(group, channel).conf0.val &= ~BIT(4);
        } else {
            LEDC_CHAN(group, channel).conf0.clk_en = 0;
        }
    }
    //LEDC_MUTEX_UNLOCK();
}

/*
	End section.
*/

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

#ifdef MCP_ASYNC
  _MCP.setAsync(true);
#endif

  //Initialize the PWM channels
  //use the ESP32 function ledc to output PWM
  ledcSetup(M1CHN, PWM_FREQUENCY, PWM_PRECISION);
  ledcAttachPin(_M1Pwm, M1CHN);
  ledcSetup(M2CHN, PWM_FREQUENCY, PWM_PRECISION);
  ledcAttachPin(_M2Pwm, M2CHN);
}

//Initialize the motors using the alternate (red) driver board
void BamBotMotorDriver::initRed(byte M1A,
										byte M1B,
										byte M2A,
										byte M2B)
{
	_M1Pwm = M1A;
	_M1Dir = M1B;
	_M2Pwm = M2A;
	_M2Dir = M2B;
	
	_flipM1 = false;
	_flipM2 = false;
	
	ledcSetup(M1CHN, PWM_FREQUENCY, PWM_PRECISION);
	ledcAttachPin(_M1Pwm, M1CHN);
	
	ledcSetup(M1CHNB, PWM_FREQUENCY, PWM_PRECISION);
	ledcAttachPin(_M1Dir, M1CHNB);
	
	ledcSetup(M2CHN, PWM_FREQUENCY, PWM_PRECISION);
	ledcAttachPin(_M2Pwm, M2CHN);
	
	ledcSetup(M2CHNB, PWM_FREQUENCY, PWM_PRECISION);
	ledcAttachPin(_M2Dir, M2CHNB);
}

/**********
Motor Drive
***********/

// speed should be a number between -400 and 400
#ifdef MCP_ASYNC
void BamBotMotorDriver::setM1Speed(int speed, bool sync) {
#else
void BamBotMotorDriver::setM1Speed(int speed) {
#endif
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

#ifdef MCP_ASYNC
	if(sync) _MCP.flush();
#endif
}

// speed should be a number between -400 and 400
#ifdef MCP_ASYNC
void BamBotMotorDriver::setM2Speed(int speed, bool sync) {
#else
void BamBotMotorDriver::setM2Speed(int speed) {
#endif
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

#ifdef MCP_ASYNC
	if(sync) _MCP.flush();
#endif
}

#ifdef MCP_ASYNC
void BamBotMotorDriver::setM1Speed(int speed) {
	BamBotMotorDriver::setM1Speed(speed, true);
}

void BamBotMotorDriver::setM2Speed(int speed) {
	BamBotMotorDriver::setM2Speed(speed, true);
}
#endif

// set speed for both motors
// speed should be a number between -400 and 400
void BamBotMotorDriver::setSpeeds(int m1Speed, int m2Speed){
#ifdef MCP_ASYNC
  setM1Speed(m1Speed, false);
  setM2Speed(m2Speed, false);
  _MCP.flush();
#else
  setM1Speed(m1Speed);
  setM2Speed(m2Speed);
#endif
}

void BamBotMotorDriver::setM1SpeedRed(int speed) {

	bool reverse = ((speed < 0) ^ _flipM1);
	speed = abs(speed);
	if (speed > 400) {
		speed = 400;
	}
	
	if (reverse) {
		_setPWM(M1CHN, 0);
		_setPWM(M1CHNB, speed);
	} else {
		_setPWM(M1CHN, speed);
		_setPWM(M1CHNB, 0);
	}
}

void BamBotMotorDriver::setM2SpeedRed(int speed) {

	bool reverse = ((speed < 0) ^ _flipM2);
	speed = abs(speed);
	if (speed > 400) {
		speed = 400;
	}
	
	if (reverse) {
		_setPWM(M2CHN, 0);
		_setPWM(M2CHNB, speed);
	} else {
		_setPWM(M2CHN, speed);
		_setPWM(M2CHNB, 0);
	}
}

void BamBotMotorDriver::setSpeedsRed(int m1Speed, int m2Speed) {
	setM1SpeedRed(m1Speed);
	setM2SpeedRed(m2Speed);
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
	aWrite(channel, duty);
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
int BamBotMotorDriver::motor1Position() {
	return _position1;
}

int BamBotMotorDriver::motor2Position() {
	return _position2;
}

//Reset the position to 0
void BamBotMotorDriver::motor1ResetPosition() {
	_position1 = 0;
}

void BamBotMotorDriver::motor2ResetPosition() {
	_position2 = 0;
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
