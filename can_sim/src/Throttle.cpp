
#include "Throttle.h"

Throttle::Throttle(uint8_t analogPin) : 
pin(analogPin) 
{
    pinMode(pin, INPUT);
}

int Throttle::read() {
    int value = analogRead(pin);

    return map(value, 0, 1023, 0, 100);
}
