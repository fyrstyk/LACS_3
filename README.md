# LACS2 - Leveling And Control System 2

ESP32-S3 based control system for mobile hybrid power trailer with solar tracking capabilities. Controls multiple motors through H-bridges with web interface for manual operation and automatic solar panel alignment.

## Table of Contents
- [Features](#features)
- [Hardware Requirements](#hardware-requirements)
- [Pin Assignments](#pin-assignments)
- [Software Requirements](#software-requirements)
- [Building and Installation](#building-and-installation)
- [Usage](#usage)
- [Motor Configuration](#motor-configuration)
- [Calibration](#calibration)
- [Web Interface](#web-interface)
- [API Reference](#api-reference)
- [Solar Tracking](#solar-tracking)
- [Troubleshooting](#troubleshooting)
- [License](#license)

## Features

- **11 Motor Control Channels**: 7 closed-loop with position feedback, 4 open-loop
- **H-Bridge Sharing**: 4 H-bridges shared between motors via relay switching
- **Position Tracking**: Hall effect sensor feedback with pulse counting (closed-loop motors)
- **Web Interface**: Responsive control panel accessible at http://192.168.1.80
- **Solar Tracking**: NOAA solar position algorithm with automatic panel alignment
- **Synchronized Movement**: Coordinate multiple motors for level lifting
- **Persistent Calibration**: NVS storage retains calibration values
- **Real-time Monitoring**: Live position updates and system status
- **Safety Features**: Relay sequencing, E-stop support, limit switches

## Hardware Requirements

### Core Components
- **MCU**: ESP32-S3 development board
- **Ethernet**: W5500 module for network connectivity
- **I/O Expander**: MCP23017 for relay control
- **Motor Drivers**: 4x H-bridge modules (48V capable)
- **Sensors**: Hall effect sensors for position feedback
- **Power**: 48V main supply, 24V auxiliary supply

### Additional Components
- Relay modules for motor selection
- Emergency stop switches
- Limit switches for safety
- Current sensing on each H-bridge

## Pin Assignments

### I2C Bus (MCP23017)
| Pin | Function | Notes |
|-----|----------|-------|
| GPIO 0 | SCL | MCP23017 communication |
| GPIO 1 | SDA | MCP23017 communication |

### Ethernet (W5500)
| Pin | Function |
|-----|----------|
| GPIO 9 | RST |
| GPIO 10 | INT |
| GPIO 11 | MOSI |
| GPIO 12 | MISO |
| GPIO 13 | SCLK |
| GPIO 14 | CS |

### Motor Control PWM
| H-Bridge | PWM1 Pin | PWM2 Pin | Motors |
|----------|----------|----------|---------|
| HB1 | GPIO 21 | GPIO 17 | Leg A, Elev A |
| HB2 | GPIO 18 | GPIO 16 | Leg B, Elev B |
| HB3 | GPIO 34 | GPIO 35 | Leg C, Extensions |
| HB4 | GPIO 41 | GPIO 42 | Leg D, Slew, Top/Bottom |

### Position Sensors (Hall Effect)
| Motor | Sensor Pin |
|-------|------------|
| Leg A | GPIO 15 |
| Leg B | GPIO 36 |
| Leg C | GPIO 38 |
| Leg D | GPIO 47 |
| Elevation A | GPIO 33 |
| Elevation B | GPIO 37 |
| Slew | GPIO 2 |

### MCP23017 I/O Assignments
#### Port A (Outputs)
| Pin | Function |
|-----|----------|
| A0 | Slew motor select relay |
| A1 | Elevation/Expansion relay select |
| A2 | Legs select relay |
| A3 | 48V sense (input) |
| A4 | H-Bridge supply enable |
| A5 | Limit switch 1 (input) |
| A6 | Spare |
| A7 | Heartbeat to PIC (25Hz) |

#### Port B
| Pin | Function |
|-----|----------|
| B0-B2 | Spare relays (expansion motor selection) |
| B3-B4 | Alarm relays |
| B5 | E-stop (input) |
| B6 | Limit switch 2 (input) |
| B7 | H-Bridge enable |

## Software Requirements

- ESP-IDF v5.4.1 or later
- Python 3.x (for ESP-IDF tools)
- Git

## Building and Installation

### 1. Clone the Repository
```bash
git clone https://github.com/yourusername/lacs2-control-system.git
cd lacs2-control-system
```

### 2. Set Up ESP-IDF Environment
```bash
# Install ESP-IDF (if not already installed)
git clone -b v5.4.1 --recursive https://github.com/espressif/esp-idf.git ~/esp-idf
cd ~/esp-idf
./install.sh esp32s3
. ./export.sh
```

### 3. Configure and Build
```bash
cd /path/to/lacs2-control-system
idf.py set-target esp32s3
idf.py menuconfig  # Optional: adjust settings
idf.py build
```

### 4. Flash to Device
```bash
idf.py -p COM7 flash monitor  # Replace COM7 with your serial port
```

## Usage

### Initial Setup
1. Power on the system with 48V supply connected
2. Connect to the device via Ethernet (static IP: 192.168.1.80)
3. Open web browser and navigate to http://192.168.1.80
4. Verify heartbeat LED on MCP23017 A7 (25Hz blink)

### Basic Operation
1. **Select Motor Group**: Choose Legs, Elevation, Slew, Extensions, or Top/Bottom
2. **Manual Control**: Use UP/DOWN buttons for speed control
3. **Position Control** (closed-loop only): Enter target position and click "Go To Position"
4. **Stop**: Click "STOP ALL" for emergency stop

### Motor Groups

| Group | Motors | Type | Range |
|-------|---------|------|-------|
| Legs (1) | Leg A, B, C, D | Closed-loop | 0-615mm |
| Elevation (2) | Elevation A, B | Closed-loop | 0-65° |
| Slew (3) | Slew | Closed-loop | ±270° |
| Extensions (4) | Left/Right Extension | Open-loop | Manual only |
| Top/Bottom (5) | Top/Bottom Extension | Open-loop | Manual only |

## Motor Configuration

### Closed-Loop Motors (with position feedback)
- **Legs A-D**: Linear actuators for trailer leveling
- **Elevation A-B**: Panel tilt control
- **Slew**: Azimuth rotation for solar tracking

### Open-Loop Motors (manual control only)
- **Left/Right Extensions**: Paired motors on single outputs
- **Top/Bottom Extensions**: Additional positioning motors

### H-Bridge Allocation
Motors share H-bridges through relay switching:
- **HB1**: Leg A / Elevation A
- **HB2**: Leg B / Elevation B  
- **HB3**: Leg C / Extension motors
- **HB4**: Leg D / Slew / Top-Bottom motors

**Note**: Only one motor group can be active at a time to prevent H-bridge conflicts.

## Calibration

### Default Calibration Values

| Motor | Pulses per Unit | Unit |
|-------|-----------------|------|
| Leg A | 29.19 | pulses/mm |
| Leg B | 29.00 | pulses/mm |
| Leg C | 28.95 | pulses/mm |
| Leg D | 28.94 | pulses/mm |
| Elevation A | 892.6 | pulses/degree |
| Elevation B | 890.5 | pulses/degree |
| Slew | 50.0 | pulses/degree |

### Calibration Procedure
1. Move motor to known position
2. Note displayed position
3. Calculate correction factor: `actual_position / displayed_position`
4. Enter new calibration value: `current_cal × correction_factor`
5. Click "Set Calibration" to save

### Example
If Elevation A shows 126.4° but actual angle is 65°:
- Correction factor: 126.4 / 65 = 1.944
- New calibration: 459 × 1.944 = 892.6 pulses/degree

## Web Interface

### Status Display
- Real-time position for all motors
- Visual indicators for active/inactive motors
- Open-loop motors show "Open Loop" status
- Click motor status to toggle selection

### Control Panel
- **Motor Group Selection**: Buttons for each group
- **Individual Motor Selection**: Checkboxes for fine control
- **Manual Control**: Touch-friendly UP/DOWN buttons
- **Position Control**: Target position entry (closed-loop only)
- **Calibration**: Motor-specific calibration adjustment

### Solar Tracking
- Current sun position display
- Time synchronization button
- Auto-tracking enable/disable
- Target position indicators

## API Reference

### HTTP Endpoints

#### GET /
Returns the main web interface HTML

#### GET /status
Returns JSON system status:
```json
{
  "positions": [611.0, 615.0, ...],
  "calibrations": [29.19, 29.0, ...],
  "motor_names": ["Leg A", "Leg B", ...],
  "motor_types": ["closed", "closed", "open", ...],
  "active_group": 1,
  "active_motors": 15,
  "auto_tracking": false,
  "time_synced": true,
  "sun_azimuth": 182.5,
  "sun_elevation": 45.2,
  "time": "2025-06-10 14:30:00"
}
```

#### POST /command
Accepts JSON commands:

**Select Motor Group**
```json
{"cmd": "select_group", "params": {"group": 1}}
```
Groups: 0=None, 1=Legs, 2=Elevation, 3=Slew, 4=Extensions, 5=Top/Bottom

**Select Individual Motors**
```json
{"cmd": "select_motors", "params": {"mask": 15}}
```
Bitmask for motors 0-10 (e.g., 15 = 0x0F = motors 0-3)

**Set Motor Speed**
```json
{"cmd": "set_speed", "params": {"motor": 0, "speed": 150}}
```
Speed range: -250 to 250

**Go To Position** (closed-loop only)
```json
{"cmd": "go_to", "params": {"motor": 0, "position": 300.0}}
```

**Stop All Motors**
```json
{"cmd": "stop_all"}
```

**Zero Position** (closed-loop only)
```json
{"cmd": "zero_position", "params": {"motor": 0}}
```

**Set Calibration**
```json
{"cmd": "set_calibration", "params": {"motor": 0, "value": 29.19}}
```

**Sync Time**
```json
{"cmd": "sync_time", "params": {"timestamp": 1686408600}}
```

**Enable Auto Tracking**
```json
{"cmd": "auto_track", "params": {"enable": true}}
```

**Synchronized Lift**
```json
{"cmd": "sync_lift", "params": {"height": 400.0}}
```

## Solar Tracking

### Algorithm
Uses NOAA solar position algorithm to calculate sun azimuth and elevation based on:
- Location: 52.77°N, -0.38°E (Bourne, UK)
- Current date and time
- Updates every minute when enabled

### Operation
1. Click "Sync Time" to set system clock
2. Enable tracking when sun elevation > 10°
3. System automatically:
   - Moves slew motor to sun azimuth
   - Tilts panels to optimal angle (90° - sun elevation)
   - Updates position throughout the day

### Manual Override
Disable auto-tracking to regain manual control at any time.

## Troubleshooting

### No Network Connection
- Verify Ethernet cable connected
- Check W5500 module connections
- Confirm static IP not conflicting (192.168.1.80)
- Check router allows 192.168.1.x subnet

### Motors Not Moving
- Verify motor group selected
- Check 48V power supply connected
- Confirm H-bridge enable LED lit
- Test E-stop not activated
- Check current limit not triggered

### Position Reading Issues
- Verify hall sensor connections
- Check sensor alignment with magnets
- Test with manual movement while monitoring
- Recalibrate if readings drift

### Relay Clicking But No Movement
- Check motor wiring
- Verify H-bridge getting power
- Test individual H-bridge channels
- Confirm relay sequencing correct

### Web Interface Not Loading
- Clear browser cache
- Try different browser
- Check JavaScript console for errors
- Verify ESP32 fully booted (heartbeat LED)

### Calibration Not Saving
- Ensure valid motor ID (0-6)
- Check NVS partition not full
- Verify calibration value reasonable
- Power cycle after setting

## Safety Considerations

1. **Emergency Stop**: Wire E-stop switches to MCP23017 B5 (active low)
2. **Limit Switches**: Connect to A5 and B6 for end-of-travel protection
3. **Current Monitoring**: H-bridge current feedback prevents overload
4. **Relay Sequencing**: Automatic delays prevent relay welding
5. **48V Isolation**: Proper grounding and isolation required

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Contributing

1. Fork the repository
2. Create your feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add some amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

## Authors

- Your Name - Initial work

## Acknowledgments

- ESP-IDF framework by Espressif
- NOAA for solar position algorithm
- Bootstrap CSS framework (embedded in web interface)

## Version History

- 1.0.0 - Initial release with basic motor control
- 1.1.0 - Added solar tracking functionality
- 1.2.0 - Added open-loop motor support for extensions
- 1.3.0 - Individual motor calibration support

---

For more information or support, please open an issue on GitHub.