# HV500 Motor Simulator

Arduino-based simulator that mimics four HV500 inverters on CAN bus.

## Hardware Requirements

- Arduino
- MCP2515 + TJA1050 CAN transceiver module
- 120 ohms termnination resistors on CAN bus

### Wiring

| MCP2515 | Arduino Uno |
|---------|-------------|
| VCC     | 5V          |
| GND     | GND         |
| CS      | D10         |
| SCK     | D13         |
| MOSI    | D11         |
| MISO    | D12         |
| INT     | (not used)  |

## CAN Protocol

### Commands (TCS → Simulator)

| Frame                  | CAN ID              | DLC | Description                          |
|------------------------|---------------------|-----|--------------------------------------|
| `hv500_setdriveenable` | 0x18A + (node - 10) | 2   | Enable/disable drive (byte0 = 0/1)   |
| `hv500_setrelcurrent`  | 0x0AA + (node - 10) | 2   | Relative current (-1.0 to +1.0), scale 0.1 |
| `hv500_seterpm`        | 0x06A + (node - 10) | 4   | Target electrical RPM (int32)        |
| `read_write_param`     | 0x0C1               | 8   | Parameter read/write (addr 20 = fault reset) |

### Telemetry (Simulator → TCS)

| Frame                | CAN ID | DLC | Key signals                              |
|----------------------|--------|-----|------------------------------------------|
| `internal_states`    | 0x0AA  | 8   | VSM state, inverter state, enable flags  |
| `current_info`       | 0x0A6  | 8   | Phase currents A/B/C, DC bus current     |
| `voltage_info`       | 0x0A7  | 8   | DC bus voltage, output voltage           |
| `fault_codes`        | 0x0AB  | 8   | POST and RUN fault codes                 |
| `motor_position_info`| 0x0A5  | 8   | Electrical angle, speed, frequency       |


## Motor Simulation

Each motor simulates:
- Speed ramp toward `target_erpm × relative_current` (2000 RPM/s)
- Coast-down when disabled or faulted
- DC bus current proportional to torque command
- Electrical angle advancing based on speed

## Building & Flashing

Requires [PlatformIO](https://platformio.org/).

```bash
cd arduino/can_sender_sim
pio run -t upload
pio device monitor -b 115200
```