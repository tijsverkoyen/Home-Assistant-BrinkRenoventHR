# Home Assistant Brink Renovent HR

Integrate your Brink Renovent HR into Home Assistant with the use of an
[Wemos D1 Mini](https://www.wemos.cc/en/latest/d1/d1_mini.html) and the
[Master OpenTherm Shield](https://diyless.com/product/master-opentherm-shield).

__Important Remark__: I don't take any responsibility for any damage to your
Brink Renovent HR or any other device. Use this software at your own risk.

The Wemos D1 Mini and Master OpenTherm Shield are the layer that allows the
communication between the Brink Renovent HR and Home Assistant. This project
uses the [MQTT integration](https://www.home-assistant.io/integrations/mqtt/)
to expose different entities.

## Installation

For this we will assume you have a MQTT broker running and the
[MQTT integration](https://www.home-assistant.io/integrations/mqtt/)
installed and configured in Home Assistant.

### Hardware

The hardware consists of a [Wemos D1 Mini](https://www.wemos.cc/en/latest/d1/d1_mini.html)
and a [Master OpenTherm Shield](https://diyless.com/product/master-opentherm-shield).

You will need to solder the headers to the Wemos D1 Mini and the Master
OpenTherm Shield, so they stack together.

### Software

1. Install and
   run [Arduino IDE](https://support.arduino.cc/hc/en-us/articles/360019833020-Download-and-install-Arduino-IDE)
2. Open [Brink_HR_bypass.ino](brink_renovent_hr_with_bypass/brink_renovent_hr.ino)
   in Arduino IDE
3. Connect the Wemos D1 Mini to your computer
4. Select the correct board and port in Arduino IDE
5. Include the following libraries through the Library Manager:
    * [OpenTherm Library by Ihor](https://github.com/ihormelnyk/opentherm_library)
    * [PubSubClient by Nick O'Leary](https://pubsubclient.knolleary.net)
    * [ArduinoJson by Benoit Blanchon](https://arduinojson.org)
6. Alter the OpenTherm library, see [Alter OpenTherm library](#alter-opentherm-library) below.
7. Alter the configuration of the Wifi Network and MQTT server in
   the [code](./brink_renovent_hr_with_bypass/brink_renovent_hr_with_bypass.ino#L8)
8. Verify and upload the code to the Wemos D1 Mini
9. Check the serial monitor for any errors.

## Alter OpenTherm library

There are several ways to alter the code of an library. Below are the steps
that I used.

For me the OpenTherm Library is located in `~/Documents/Arduino/libraries/OpenTherm_Library`.

1. Open the `src`-folder.
2. Open `OpenTherm.cpp` in your favorite editor.
3. Compare the code with the code in [OpenTherm.cpp](opentherm_library/OpenTherm.cpp). Apply the changes.
4. Open `OpenTherm.h` in your favorite editor.
5. Compare the code with the code in [OpenTherm.h](opentherm_library/OpenTherm.h). Apply the changes.

## Physical connection

1. Power of your Brink Renovent HR
2. Locate the OpenTherm connector on your Brink Renovent HR. Connect the Master
   OpenTherm Shield. There is no polarity, so you can connect it in any direction.
3. Connect a power source to the Wemos D1 Mini USB-C port.
4. Power on your Brink Renovent HR

## Home Assistant

As this project uses the [MQTT integration](https://www.home-assistant.io/integrations/mqtt/),
you will see a new device exposed: `Brink Renovent HR`, with several entities.

### Entities

* **Current position/outlet volume**, a fan entity that allows you to control
  the speed of the Brink Renovent HR.
* **Minimum atmospheric temperature bypass**, to control the minimum atmospheric
  temperature to open the bypass.
* **Minimum indoor temperature bypass**, to control the minimum indoor
  temperature to open the bypass.
* **Bypass status**, a sensor that indicates the status of the bypass. See the [FAQ](#the-bypass-is-always-0)
    * 0: bypass is closed
    * 1: bypass is open
* **Current input volume**
* **Current output volume**
* **Current pressure input duct**
* **Current pressure output duct**
* **Fault Code**
* **Fault Indication**
* **Filter Dirty**
* **Fixed imbalance**
* **Status frost protection**
* **Temperature from atmosphere**
* **Temperature from indoors**
* **Ventilation Mode**
    * 0: Standby
    * 1: Working
* **WiFi RSI**

## FAQ

### Do you support other Brink systems?

No, not at the moment. I only have a Brink Renovent HR and I don't know if the other systems are using the same system.

### Can I still use my physical switch in combination with this integration?

No, this integration will take over the control of your Brink Renovent HR.

### Why do I need to use a custom version of the OpenTherm library?

The [OpenTherm library by Ihor](https://github.com/ihormelnyk/opentherm_library)
does not support the Brink Renovent HR. Support was added by [Raf1000](https://github.com/raf1000/brink_openhab).
Which I have copied and altered to make it work for my setup.

### I get `Timed out waiting for packet header` when uploading the code to the Wemos D1 Mini

If you get the following error when uploading the code to the Wemos D1 Mini:

A fatal esptool.py error occurred: Timed out waiting for packet header

You can try the following:

1. Open `~/Library/Arduino15/packages/esp8266/hardware/esp8266/3.1.2/tools/esptool/esptool.py`
2. Change `MD5_TIMEOUT_PER_MB = 8` to `MD5_TIMEOUT_PER_MB = 12`

I don't know why this happens, but it seems to work.

### The bypass is always 0

I know, I have the same issue. I don't know why this happens. If you know, please let me know.

## Credits

* [raf1000/brink_openhab](https://github.com/raf1000/brink_openhab)
* [C. Portegies → Brink HRV OpenTherm Exploration](https://portegi.es/blog/opentherm-wtw-1)
* [C. Portegies → Brink HRV OpenTherm Control via Home-Assistant](https://portegi.es/blog/opentherm-wtw-2)
* [Renovent HR Medium/Large (5e druk mei 2006)](https://www.brinkclimatesystems.nl/documenten/renovent-medium-large-wellington-611175.pdf)
* [Renovent HR Medium/Large manual](https://www.ventilationsupplies.ie/uploadedfiles/BCSI-EN002-Renovent%20medium-large-installation-manual.pdf)