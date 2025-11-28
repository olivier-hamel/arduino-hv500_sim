#ifndef THROTTLE_H
#define THROTTLE_H

#include <Arduino.h>

class Throttle {
public:
	Throttle(uint8_t analogPin);
	int read();

private:
	uint8_t pin;
};

#endif // THROTTLE_H
