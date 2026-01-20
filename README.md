# Gimbal-less Flightstick Project (v4.4)

This repository contains the necessary files to build a compact, 3D-printable, gimbal-less flightstick controller based on the Arduino Pro Micro. It leverages an **MPU-6050** for pitch and roll sensing, reducing mechanical complexity, while offering advanced software features like dynamic rudder modes, ambidextrous support, and smart safety cutoffs.

## Table of Contents

- [Features](#features)
- [Hardware](#hardware)
  - [PCB Shield](#pcb-shield-gerber-files)
  - [BOM (Bill of Materials)](#bom-bill-of-materials)
- [Firmware](#firmware)
  - [Dependencies](#dependencies)
  - [Configuration (The Setup Menu)](#configuration-the-setup-menu)
  - [Advanced: Custom Device Name](#advanced-custom-device-name)
  - [Uploading](#uploading-the-firmware)
- [Usage Guide](#usage-guide)
  - [Flight Modes](#flight-modes)
  - [Safety Features](#safety-features)
  - [Hand-Swapping](#hand-swapping)
- [Contributing](#contributing)
- [License](#license)

---

## Features

* **Gimbal-less Design:** Utilizes an MPU-6050 IMU for pitch and roll sensing. No complex springs or gimbals required.
* **Dual Rudder Modes:**
    * *Pot Swap:* Potentiometer controls Throttle, but swaps to Rudder when a modifier is held.
    * *Roll-to-Yaw:* Leaning the stick controls Pitch/Roll, but swaps Roll to Rudder when a modifier is held (great for space sims).
* **Ambidextrous:** Runtime toggling between Left-Handed and Right-Handed operation (inverts throttle logic).
* **Smart Safety:**
    * **Cutoff:** Automatically centers controls if the stick falls over (>80° tilt).
    * **Rolling Average:** Optionally holds the average of the last 3 seconds of input instead of snapping to neutral.
    * **Hysteresis:** Requires the stick to be upright (<15°) to re-engage controls.
* **Hardware Agnostic:** "Learn Mode" allows you to map buttons to any pin via the software menu without recoding.
* **Full Customization:** Invert axes, tune sensitivity (Kalman filter), and adjust deadzones via the Serial Menu.

## Hardware

The core of this project is a custom 3D-printed enclosure housing the electronics.

### PCB Shield (Gerber Files)

The `Joystick_board.zip` archive contains the manufacturing files for the custom PCB shield. This shield sits on top of the Arduino Pro Micro to simplify wiring.

**To order:** Upload the `.gbr` files to a service like JLCPCB or PCBWay. Standard 2-layer FR-4 settings work fine.

### BOM (Bill of Materials)

| Component | Quantity | Description |
| :--- | :--- | :--- |
| **Arduino Pro Micro** | 1 | 5V/16MHz version (ATmega32U4). |
| **MPU-6050 Module** | 1 | GY-521 or similar breakout board (I2C). |
| **10kΩ Potentiometer** | 1 | Linear taper (B10K). Used for Throttle/Rudder. |
| **5-Way Navigation Switch** | 1 | "Coolie Hat" switch (Up, Down, Left, Right, Press). |
| **Tactile Buttons** | 4 | Activation, Modifiers (L/R), and Set/Reset buttons. |
| **PCB / Wires** | - | Custom PCB or perfboard + jumper wires. |

---

## Firmware

The code has been updated to **Version 4.4**.

### Dependencies

Install these libraries via the Arduino IDE Library Manager:

1.  **`Joystick`** by MHeironimus (Handles USB HID communication).
2.  **`Adafruit MPU6050`** by Adafruit (Sensor driver).
3.  **`Adafruit Unified Sensor`** by Adafruit (Base sensor class).
4.  **`Wire`** & **`EEPROM`** (Built-in standard libraries).

### Configuration (The Setup Menu)

You do not need to edit the code to change settings. This firmware features a **Serial Setup Menu**.

1.  Unplug the joystick.
2.  Hold the **Activation Button** (Pin 14 by default).
3.  Plug the joystick in while holding the button.
4.  Open **Serial Monitor** in Arduino IDE (Set to **115200 baud**).
5.  Release the button when the menu appears.

**Menu Options:**
* **1. Lean Angle:** Set physical tilt required for 100% input (20°-60°).
* **2. Handedness:** Toggles Throttle direction (Push-to-Thrust vs Pull-to-Thrust).
* **3. Responsiveness:** Tunes the Kalman filter (Smooth, Quick, Fastest).
* **4. Pickup Threshold:** Sensitivity for "catching" the throttle after a mode swap.
* **5. Rudder Mode:** Switch between "Pot Swap" and "Roll-to-Yaw".
* **6. Remap Buttons:** Interactive wizard to assign pins to functions. (Type 's' to skip buttons you haven't wired).
* **7. Invert Axes:** Invert Pitch, Roll, or Throttle direction.
* **8. Test Inputs:** Debug screen to verify wiring.
* **9. Smart Safety:** Toggle between "Snap to Neutral" or "Hold 3s Average" on cutoff.
* **10. Safety Angle:** Adjust the angle at which safety cutoff triggers (50°-89°).

### Advanced: Custom Device Name

To make Windows display "My Custom Stick" instead of "Arduino Leonardo", you must edit the `boards.txt` file in your Arduino hardware folder.
1.  Locate `boards.txt` for SparkFun AVR Boards.
2.  Find the `promicro` section.
3.  Change `build.pid` (increment the last digit, e.g., `0x9207`).
4.  Change `build.usb_product` to your desired name.
5.  Re-upload the firmware.

### Uploading the Firmware

1.  Select Board: **SparkFun Pro Micro** (or Arduino Leonardo).
2.  Select Port: Your COM port.
3.  Upload `Pro_Micro_Joystick_v4_4.ino`.

---

## Usage Guide

### Flight Modes (Rudder Logic)
* **Mode 0 (Pot Swap):**
    * *Standard:* Tilt = Pitch/Roll, Pot = Throttle.
    * *Modifier Held:* Pot = Rudder (Throttle freezes).
* **Mode 1 (Roll-to-Yaw):**
    * *Standard:* Tilt = Pitch/Roll, Pot = Throttle.
    * *Modifier Held:* Tilt Left/Right = Rudder (Roll freezes).

### Safety Features
* **Desk Mode:** If you lay the controller down (>80° tilt), outputs cut to Neutral (or 3s Average if "Smart Safety" is ON).
* **Wake Up:** To regain control, pick the controller up and hold it upright (<15°) for a moment.
* **Panic Reset:** Press the **Reset Button** (or Coolie Hat Press) to instantly center the Rudder trim.

### Hand-Swapping
* To switch hands mid-flight without unplugging: **Hold the SET Button for 3 seconds**.
* The LED will flash, and the throttle direction will invert.
* *Note:* The throttle output will freeze until you move the physical pot to "catch" the value, preventing sudden engine surges.

---

## Contributing

1.  Fork the repo.
2.  Create a branch (`git checkout -b feature/NewFeature`).
3.  Commit changes.
4.  Push and open a Pull Request.

## License

This project is open-source and licensed under the [MIT License](LICENSE.md).
