#include <Arduino.h>

#include <Hv500CanNode.h>
#include <SerialInterface.h>

constexpr uint8_t CAN_CS_PIN = 10;
constexpr uint32_t TELEMETRY_PERIOD_MS = 100;
constexpr uint16_t SERIAL_SPEED = 115200;

Hv500CanNode hv500(CAN_CS_PIN);
SerialInterface iface(hv500, SERIAL_SPEED);

void setup() {

}