# Mailbox Sentry

Mailbox Mail Delivery Notification for HomeKit. Be notified when the mailman delivers your mail by attaching an accelerometer to the mailbox lid.

This project requires two ESP32 devices. The sensor element, in the mailbox, runs on a battery and sleeps most of the time. Sleeping is incompatible with HomeKit's requirement that WiFi devices always be powered. So a second ESP32 device is used as a wall powered hub that is always available on WiFi. The sensor communicates with the hub using Espressif's ESP-NOW mechanism. ESP-NOW bootstrapping is done manually, by editing the code.

Using available development boards, this is a fairly large sensor. See the included STL files to get an idea of the size of the sensor using the recommended hardware. In principal, it should be straightforward to design a very compact custom board with an ESP32 and a Bosch BMA400 accelerometer on board.

## Requirements

* Two Espressif ESP32 devices, one for the sensor element, and one for the hub element. The sensor element should ideally have a charging circuit to allow charging the LiPo that will power the sensor. [The SparkFun Qwiic Pocket Development Board](https://www.sparkfun.com/products/22925) has an onboard charger, and is easy to interface with a suitable accelerometer.
* Bosch BMA400 accelerometer. The [SparkFun BMA400 Qwicc board](https://www.sparkfun.com/products/21207) interfaces easily with the SparkFun Qwiic Pocket.
* LiPo Battery. If using the linked SparkFun board, use a battery with a JST PH 2.0mm connector. Ensure that the polarity of the battery leads match the polarity of the board connector. The SparkFun Qwicc Pocket's connector polarity is the opposite of most hobby LiPos sold. Battery connector pins can be easily pried out with a utility knife to reverse their position in the connector. [This LiPo](https://www.amazon.com/dp/B0CNLNZFQL) battery works well after the leads have been reversed.
* [Arduino IDE](https://www.arduino.cc/en/software)

## Building

This project was designed around the recommended SparkFun hardware. It is possible to use any ESP32 and BMA400 devices. But you will likely need to change the pin assignments for I2C and battery charge level to match the hardware you use. You will also likely need to tweak the battery charge level calculation in the hub to match the hardware you are using in the sensor.

### Prep

* Install [Arduino IDE](https://www.arduino.cc/en/software)
* In the Arduino IDE, Select 'Tools' -> 'Manage Libraries'
    * Install the (SparkFun) BMA400 library.
    * Install the HomeSpan library.
* In the Arduino IDE, Select 'Tools' -> 'Board' -> 'Board Manager'
    * Install support for the ESP32 boards you are using (Use the Espressif ESP32 library if using the recommended board).

### Hub

* Open, compile, and install the hub project onto an ESP32.
    * If there are compilation/upload failures because of space, select 'Tools' -> 'Partition Scheme', and select a partition scheme with a larger APP size (OTA is not supported so 'No OTA' is ok)
* Use the Arduino IDE's serial monitor and [HomeSpan's CLI](https://github.com/HomeSpan/HomeSpan/blob/master/docs/CLI.md) to configure the hub's WiFi, and add it to HomeKit.
* Write down the hub's MAC address reported in the serial monitor!

### Sensor

* Connect the BMA400 accelerometer's INT1 pin to the ESP32's GPIO 5 pin. If using different pins, make sure to modify the code to match.
* Open the sensor project.
* Find the MAC address string in the code `12:34:56:78:9A:BC`, and replace it with the hub's MAC address.
* Compile, and install the sensor project on an ESP32 with a BMA400 accelerometer.
    * If there are compilation/upload failures because of space, select 'Tools' -> 'Partition Scheme', and select a partition scheme with a larger APP size (OTA is not supported so 'No OTA' is ok)
* Write down the sensor's MAC address reported in the serial monitor!
* To reupload the sensor code onto an already running sensor; on the board, press and hold the `boot` button and press the `reset` button. Release both and the board will be ready for the upload. After upload, press the `reset` button again to start the code.
* To save a bit on power consumption, change the code to disable serial logging by defining `ENABLE_SERIAL` to `0`. Recompile, and upload.

### Hub Redux

* In the hub code, find the MAC address string `12:34:56:78:9A:BC`, and replace it with the sensor's MAC address.
* Recompile, and reinstall the code onto the hub.

## Test

With both the sensor and hub running, ensure that moving the accelerometer triggers a notification to HomeKit. The serial monitors may provide useful debug messages, if it doesn't immediately work. If the initial test works, try the same with the sensor operating solely on battery power (disconnect it from your computer).

## Packaging

If the linked hardware is used, the provided STL files are suitable for 3D printing case components for the sensor element (ESP32 board, and BMA400 board). Print without supports.



