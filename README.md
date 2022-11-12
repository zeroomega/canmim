# CAN Man In the Middle (CANMIM)

CANMIM is a DIY project which supports reading from vehicle's CAN bus, filtering and injecting data and sending the CAN data to another CAN bus using a ESP32C3 MCU and MP2515 CAN controller. The way it works is very similar to the Man In the Middle attack. The project uses ESP-IDF.


# Intended use case

For amateur car racer and track enthusists, it is very popular to use laptimer and data loggers like AIM Solo 2 DL to log GPS and CAN data. However, these comercial data loggers/lap timers usually cannot log any data that are not from the CAN bus. For example, aftermarket oil pressure sensor sends out 0-5V analog signals and there isn't any official accessories that can hook it up to an AIM Solo 2 DL device. This CANMIM device solved this issue, by reading the analog signal from the oil pressure sensor using MCU's ADC controller, send out the ADC readings using custom CAN IDs, combined with CAN data from the vehicle's CAN bus, to data logger's CAN input.

This project is currently designed for Toyota GR86/Subaru BRZ ZD8. But the code is simple and should be easy to port for other vehicles.

# Recommended parts list

For using the PCB provided in this repo, following parts will be needed to assemble a unit:

* [ESP32-C3-DevKitC-02](https://www.amazon.com/gp/product/B09D3S4RPZ)
* [MCP2515 CAN Bus Module TJA1050 Receiver SPI Module](https://www.amazon.com/gp/product/B01D0WSEWU)
* [SN65HVD230 CAN Bus Transceiver breakout board](https://www.amazon.com/gp/product/B0B82GJLH5)
* [Traco TSR-1-2450 5V1A buck converter](https://www.digikey.com/en/products/detail/traco-power/TSR-1-2450/9383780)
* [Through hole Resistors (recommend 10KOhm and 22KOhm)](https://www.amazon.com/gp/product/B072BL2VX1)
* [JST XH connectors](https://www.amazon.com/gp/product/B09DBGVX5C)
  - [Optional RJ45 connectors](https://lcsc.com/product-detail/Ethernet-Connectors-Modular-Connectors-RJ45-RJ11_CONNFLY-Elec-DS1129-05-S80BP-X_C86580.html)

TODO: Explain how to assemble a unit.

# Build the project

This projects requires ESP-IDF to build. Please fetch the latest version from [Espressif](https://dl.espressif.com/dl/esp-idf/).

## Fetch the source and setup the environment

```
git clone --recurse-submodules https://github.com/zeroomega/canmim
```

Please follow the guide at https://docs.espressif.com/projects/esp-idf/en/v4.3.1/esp32/get-started/index.html#get-started-set-up-env to setup the enviroment. On Windows, this can be done by opening up a ESP-IDF powershell windows.

## Setup and build the project

```
cd canmin
idf.py set-target=esp32c3
idf.py build
```

# Flash the image to the MCU

Connect the MCU to the computer using USB. Find the device name of the MCU serial device, e.g. COM8 on Windows. Run:

```
idf.py -p COM8 flash
```
