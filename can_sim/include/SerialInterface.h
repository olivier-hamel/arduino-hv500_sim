#pragma once

#include <Arduino.h>
#include <Hv500CanNode.h>

class SerialInterface
{
    
public:
    SerialInterface(Hv500CanNode& hv500, uint16_t speed);

    void printHelloWorld();
    void showMainMenu(); // Show basic status. Two choice : (1 - Write a command) (2 - Read incomming can data)
    

private :
    Hv500CanNode hv500_;
};