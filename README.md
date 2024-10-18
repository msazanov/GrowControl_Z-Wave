# Greenhouse Controller on Z-Uno G2

A greenhouse automation controller project designed for growing agricultural crops. The controller is based on the Z-Uno 2 platform and supports various sensors and devices for monitoring and controlling the microclimate of the greenhouse.

## Description

This project implements the following features:

- **Connection and data reading from various sensors**:
  - Temperature and humidity sensor **DHT-22**
  - Temperature sensors **DS18B20**
  - CO₂ level sensor **MH-Z19**
  - **Water leakage sensor**

- **Control of outputs**:
  - 4 relay outputs for controlling devices (e.g., lighting, ventilation)
  - 4 PWM outputs for controlling brightness or speed of devices (e.g., LED strips, fans)

- **Interface with OLED display** for information display

- **Operation with an encoder** for managing settings directly on the controller

- **Support for Z-Wave protocol** for integration with smart home systems

## Installation

1. **Clone** the repository to your computer.
2. **Install the Z-Uno 2 board** in the Arduino IDE:
   - Open the Arduino IDE.
   - Go to **File** > **Preferences**.
   - In the **Additional Boards Manager URLs** field, add the following URL:
     ```
     http://z-uno.z-wave.me/files/z-uno2/package_z-wave.me_index.json
     ```
   - Click **OK** to save the preferences.
   - Go to **Tools** > **Board** > **Boards Manager**.
   - Search for **Z-Uno 2** and install it.
3. **Install the necessary libraries**:
   - The required libraries (`EEPROM.h`, `ZUNO_DHT.h`, `ZUNO_DS18B20.h`, `U8g2lib.h`, `Wire.h`) are included with the Z-Uno 2 board installation.
   - No additional library installations are necessary beyond installing the Z-Uno 2 board.
4. **Upload** the sketch to the Z-Uno G2 controller using the Arduino IDE.

## Usage

- Connect the sensors and actuators according to the **wiring diagram** provided in the code comments.
- Configure the controller parameters if necessary.
- Power on the controller and ensure that sensor data is displayed on the **OLED display**.
- Control the outputs via the **Z-Wave** interface or directly using the **encoder**.

## Implemented

- **Connection and data reading from sensors**:
  - DHT-22 (temperature and humidity)
  - DS18B20 (temperature)
  - MH-Z19 (CO₂ level)
  - Water leakage sensor

- **Configured Z-Wave channels** for transmitting sensor data and controlling outputs.

- **Control of 4 relay outputs and 4 PWM outputs** via Z-Wave.

- **Information display on OLED screen**:
  - Current sensor values
  - Relay and PWM output statuses
  - Indication of water leakage sensor activation

- **Encoder handling** for navigation and settings management on the controller.

## Not Implemented and Needs Development

- **Automation of control processes** based on sensor readings:
  - Automatic switching on/off of relay and PWM outputs depending on parameters (temperature, humidity, CO₂ level, etc.)
  - Setting threshold values for automation

- **Enhancement of the user interface on the OLED display**:
  - Settings menu for configuring parameters without reflashing the controller
  - Graphical data representation (e.g., parameter change graphs)

- **Adding data logging** from sensors for subsequent analysis

- **Integration with external services** or systems for remote monitoring and control

- **Support for additional sensors** (e.g., pH, soil moisture, light intensity) if necessary

## Author

- Marat Sazanov

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.
