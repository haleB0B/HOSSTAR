# Gimbal-less Flightstick Project

This repository contains the necessary files to build a compact, 3D-printable, gimbal-less flightstick controller based on the Arduino Pro Micro, leveraging an MPU-6050 for pitch and roll sensing, along with traditional potentiometers and buttons for additional controls.

## Table of Contents

- [Features](#features)
- [Hardware](#hardware)
  - [PCB Shield (Gerber Files)](#pcb-shield-gerber-files)
  - [BOM (Bill of Materials)](#bom-bill-of-materials)
- [Firmware](#firmware)
  - [Compatibility](#compatibility)
  - [Dependencies](#dependencies)
  - [Configuration](#configuration)
  - [Uploading the Firmware](#uploading-the-firmware)
- [Assembly](#assembly)
- [Usage](#usage)
- [Contributing](#contributing)
- [License](#license)
- [Acknowledgements](#acknowledgements)

## Features

* **Gimbal-less Design:** Utilizes an MPU-6050 IMU for pitch and roll sensing, reducing mechanical complexity and size.
* **Arduino Pro Micro Based:** Easy to program and widely supported.
* **Custom PCB Shield:** Simplifies wiring and assembly for a cleaner build.
* **Configurable Controls:** Supports a thrust/rudder potentiometer, 5-way HAT switch, and multiple tactile buttons.
* **EEPROM Storage:** Saves calibration and configuration settings for persistence across power cycles.
* **USB HID Joystick:** Recognized as a standard joystick by your operating system, no special drivers needed.

## Hardware

The core of this project is a custom 3D-printed enclosure (design files not included in this repo, but mentioned for context) housing the electronics.

### PCB Shield (Gerber Files)

The `Yoke_Pro_Micro_-_new.zip` archive contains all the necessary manufacturing files for the custom PCB shield. This shield is designed to sit on top of the Arduino Pro Micro, simplifying the connections for the MPU-6050, potentiometer, and buttons.

**To order the PCB:**
1.  Download and extract the `Yoke_Pro_Micro_-_new.zip` file.
2.  Upload the contents of the extracted folder (the `.gbr` files) to your preferred PCB fabrication service (e.g., JLCPCB, PCBWay, OSH Park).
3.  Ensure you select appropriate manufacturing parameters (e.g., 2-layer board, FR-4 material).

The PCB is designed with footprints for:
* Arduino Pro Micro (via pin headers)
* MPU-6050 module (I2C connection)
* Pin headers for connecting the potentiometer and tactile buttons.

### BOM (Bill of Materials)

Here's a breakdown of the electronic components required for this project:

| Component                    | Quantity | Notes / Description                                                                   |
| :--------------------------- | :------- | :------------------------------------------------------------------------------------ |
| **Core Components** |          |                                                                                       |
| Arduino Pro Micro            | 1        | 5V/16MHz version is recommended. This is the brain of the controller.                 |
| MPU-6050 Module              | 1        | 3-axis Gyroscope & Accelerometer. Used for pitch and roll (via I2C).                  |
| **Inputs** |          |                                                                                       |
| 10kΩ Potentiometer           | 1        | A linear taper pot is best. Used for the Thrust/Rudder lever.                         |
| 5-Way Navigation Switch      | 1        | For the Coolie HAT switch (Up, Down, Left, Right, and Center Press).                  |
| Tactile Push Buttons         | 4        | For Activation, Rudder Modifiers (x2), and a general-purpose button.                  |
| **Assembly & Connection** |          |                                                                                       |
| Custom PCB Shield (from Gerbers) | 1      | The custom board to solder connectors, Arduino, and IMU I2C.                          |
| Pin headers/sockets          | As needed| Use pins on the PCB for pushbuttons, potentiometer, and sockets for the Arduino.      |
| Jumper Wires                 | 1 pack   | A female-to-female assortment is most useful if you use pin sockets on the PCB.       |
| Micro USB Cable              | 1        | To connect the Pro Micro to your computer for programming and use.                    |

## Firmware

The `Pro_Micro_Joystick_v2_8` folder contains the Arduino Sketch (firmware) for the gimbal-less flightstick.

### Compatibility

This firmware is designed specifically for the **Arduino Pro Micro (ATmega32U4)**. It leverages the native USB capabilities of this microcontroller to act as a HID (Human Interface Device) joystick.

### Dependencies

Before compiling, you need to install the following libraries in your Arduino IDE:

1.  **`Joystick.h`**: This library is usually included with the Arduino IDE for ATmega32U4 boards, but ensure it's available. It handles the USB HID joystick communication.
2.  **`Wire.h`**: Standard Arduino library for I2C communication (required for MPU-6050).
3.  **`Adafruit_ADS1015.h`**: Required if you use an external ADC for higher precision inputs. If your design relies solely on the Pro Micro's internal ADC, this might be optional or removed.
    * *Installation:* Go to `Sketch > Include Library > Manage Libraries...` and search for "Adafruit ADS1015".
4.  **`EEPROM.h`**: Standard Arduino library for reading and writing to the EEPROM (for configuration storage).
5.  **`MPU6050_light.h`** (or similar MPU-6050 library): A library to easily interface with the MPU-6050.
    * *Installation:* Go to `Sketch > Include Library > Manage Libraries...` and search for "MPU6050_light" or your preferred MPU6050 library.

### Configuration

Open the `Pro_Micro_Joystick_v2_8.ino` file in the Arduino IDE.

* **`FIRMWARE_VERSION`**: Defined on line 15, indicates the firmware version.
* **`JoystickConfig` Struct**: (Lines 18-22) Defines the structure for configuration data stored in EEPROM, including version, lean angle, handedness, and Kalman filter level.
* **Pin Definitions**: (Lines 31-41) Crucially, these lines define which physical pins on the Arduino Pro Micro are connected to your joystick's components. **Adjust these values to match your PCB and wiring if you deviate from the original design.**
    * `THRUST_UP_PIN`, `THRUST_DOWN_PIN`: For the thrust/rudder potentiometer.
    * `COOLIE_UP_PIN`, `COOLIE_DOWN_PIN`, `COOLIE_LEFT_PIN`, `COOLIE_RIGHT_PIN`, `COOLIE_PRESS_PIN`: For the 5-way navigation switch.
    * `SET_BUTTON_PIN`: A special button for entering configuration/calibration modes.
    * `RST_BUTTON_PIN`: Reset button (often tied to the Pro Micro's reset).
    * `BUTTON_1_PIN`, `BUTTON_2_PIN`, `BUTTON_3_PIN`, `BUTTON_4_PIN`: General-purpose buttons.

### Uploading the Firmware

1.  Connect your Arduino Pro Micro to your computer using a Micro USB cable.
2.  Open the `Pro_Micro_Joystick_v2_8.ino` file in the Arduino IDE.
3.  Go to `Tools > Board` and select `Arduino Leonardo` (the Pro Micro uses the same ATmega32U4 chip).
4.  Go to `Tools > Port` and select the serial port corresponding to your Pro Micro.
5.  Click the "Upload" button (right arrow icon).

Once uploaded, your Pro Micro should be recognized by your operating system as a standard USB joystick.

## Assembly

Detailed assembly instructions are outside the scope of this README, as they depend heavily on the 3D-printed enclosure design (not provided here). However, the general steps involve:

1.  **Solder components to the PCB shield:** Install the MPU-6050, pin headers for the Arduino Pro Micro, and other connection pins.
2.  **Mount the Arduino Pro Micro:** Plug the Pro Micro onto the soldered pin headers of the PCB shield.
3.  **Wire external components:** Connect the potentiometer, 5-way switch, and tactile buttons to the appropriate pins on the PCB shield using jumper wires.
4.  **Enclosure:** Integrate the assembled electronics into your 3D-printed flightstick handle.

Find some more details around this project here: https://www.instructables.com/The-Gimbal-less-Joystick-Redefining-Control-With-a/

## Usage

After successfully uploading the firmware and assembling the hardware:

1.  Plug the flightstick into your computer via USB.
2.  Open your operating system's joystick calibration utility (e.g., "Set up USB game controllers" on Windows) to verify functionality and calibrate if necessary.
3.  Launch your favorite flight simulator or game and configure the joystick axes and buttons.

## Contributing

Contributions are welcome! If you have improvements, bug fixes, or new features, please:

1.  Fork the repository.
2.  Create a new branch (`git checkout -b feature/AmazingFeature`).
3.  Make your changes.
4.  Commit your changes (`git commit -m 'Add some AmazingFeature'`).
5.  Push to the branch (`git push origin feature/AmazingFeature`).
6.  Open a Pull Request.

## License

This project is open-source and licensed under the [MIT License](LICENSE.md).

## Acknowledgements

* Thanks to the Arduino community for the IDE and core libraries.
* Thanks to Adafruit for the ADS1015 library.
* Inspiration drawn from various custom flightstick and Arduino HID projects.
