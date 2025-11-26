#include <SerialInterface.h>

SerialInterface::SerialInterface(Hv500CanNode& hv500, uint16_t speed) :
    hv500_(hv500)
{
    Serial.begin(speed);
    while (!Serial)
    {
        // Wait for the port to open
    }

    if (!hv500_.begin(CAN_500KBPS, MCP_16MHZ))
    {
        Serial.println("Failed to initialize MCP2515.");
        while (true)
        {
            delay(1000);
        }
        
    }
    
    Serial.println("HV500 CAN ready.");
    
}

void SerialInterface::printHelloWorld() {
    Serial.println("Hello World");
}