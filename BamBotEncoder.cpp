#include "Arduino.h"
#include "BamBotEncoder.h"

volatile int BamBotEncoder::_position = 0;
volatile uint8_t BamBotEncoder::_state = 0;
portMUX_TYPE BamBotEncoder::_mux = portMUX_INITIALIZER_UNLOCKED;
byte BamBotEncoder::_pinA;
byte BamBotEncoder::_pinB;

void BamBotEncoder::init(byte pinA, byte pinB) {
	_pinA = pinA;
	_pinB = pinB;
	
	pinMode(_pinA, INPUT_PULLUP);
	pinMode(_pinB, INPUT_PULLUP);
	attachInterrupt(digitalPinToInterrupt(_pinA), _interruptHandler, CHANGE);
	attachInterrupt(digitalPinToInterrupt(_pinB), _interruptHandler, CHANGE);
}

int BamBotEncoder::position() {
	return _position;
}

void BamBotEncoder::reset() {
	_position = 0;
}

void BamBotEncoder::_interruptHandler() {
	portENTER_CRITICAL_ISR(&_mux);
	
	uint8_t s = _state & 3;
	if (digitalRead(_pinA)) s |= 4;
	if (digitalRead(_pinB)) s |= 8;

	switch (s) {
		case 0: case 5: case 10: case 15:
			break;
		case 1: case 7: case 8: case 14:
			_position++;
			break;
		case 2: case 4: case 11: case 13:
			_position--;
			break;
		case 3: case 12:
			_position += 2;
			break;
		default:
			_position -= 2;
			break;
	}
	_state = (s >> 2);

	portEXIT_CRITICAL_ISR(&_mux);
}
