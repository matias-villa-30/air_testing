# Testing of SCD41 and BME690

## Components

* XIAO BLE nRF52840 Sense
* Sensirion SCD41 — Temperature, CO₂ and Humidity Sensor
* Bosch BME690 — Gas, Pressure, Temperature and Humidity Sensor

## How to build and test

### Linux / macOS

The build and flash process described below uses the Zephyr command-line tools and helper scripts. Follow the official Zephyr Getting Started Guide first to install the required dependencies and set up the Zephyr environment:

https://docs.zephyrproject.org/latest/develop/getting_started/index.html

1. Install the Zephyr SDK and create/activate the required Python virtual environment by following the Zephyr documentation above.

2. Open a terminal in the project directory and run:

   ```bash
   ./build_zephyr.sh
   ```

3. Put the XIAO BLE nRF52840 Sense into bootloader mode by pressing the **RESET** button twice.

4. Flash the firmware by running:

   ```bash
   ./flash_zephyr.sh
   ```

### Windows

The Zephyr Getting Started Guide also contains the Windows-specific setup instructions. If you are using Windows, follow the Windows section of the official documentation first, as the required environment and command-line tools are different from Linux/macOS.

https://docs.zephyrproject.org/latest/develop/getting_started/index.html

## How it works

The firmware continuously reads measurements from both sensors and reports the air conditions detected in the room.

The **SCD41** measures:

* Temperature
* Relative humidity
* CO₂ concentration (ppm)

The **BME690** measures:

* Temperature
* Relative humidity
* Gas resistance

The measurements can be used together to get a better picture of the air conditions in the room. For example, the SCD41 provides a direct measurement of CO₂ concentration, while the BME690's gas-resistance measurement can be used as an indicator of changes in air quality.

### ELI5: How does the BME690 measure gas?

The BME690 contains a tiny heating element and a gas-sensitive material.

Think of it like a tiny **electronic nose**:

1. The sensor heats up its gas-sensitive element to a specific temperature.
2. When different gases in the air come into contact with the element, its **electrical resistance changes**.
3. The sensor measures this resistance.
4. The BME690 reports the result as **gas resistance**, measured in ohms (Ω).
5. Bosch provides formulas and compensation data that can be used to convert the raw measurements into information about the gas-sensing element and air quality.

So, unlike the SCD41, the BME690 does **not directly tell you "the air quality is X"**. It measures how the resistance of its sensing element changes, and software can use those changes to detect variations in the surrounding air.

For example, if someone enters the room, cooking starts, or another source of volatile compounds appears, the gases in the air can change the sensing element's resistance. By observing these changes over time, the BME690 can be used to detect changes in indoor air quality.

The **SCD41 works differently**: it specifically measures CO₂ concentration and reports it in **parts per million (ppm)**. This makes it useful for monitoring ventilation and detecting when a room has accumulated more CO₂.
