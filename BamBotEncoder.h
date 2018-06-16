#ifndef BamBotEncoder_h
#define BamBotEncoder_h

#include <Arduino.h>

class BamBotEncoder
{
	public:
		static void init(byte pinA, byte pinB);
		static int position();
		static void reset();
	
	private:
		static byte _pinA;
		static byte _pinB;
		static volatile int _position;
		static volatile uint8_t _state;
		static portMUX_TYPE _mux;
		static void IRAM_ATTR _interruptHandler();

};

#endif
