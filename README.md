# Puara Controller (Joystick converter/bridge)

This program converts game controller data into OSC.

- [Puara Controller (Joystick converter/bridge)](#puara-controller-joystick-converterbridge)
  - [To compile](#to-compile)
  - [Usage](#usage)
    - [Current OSC namespace](#current-osc-namespace)
      - [Messages sending value and duration of the last event:](#messages-sending-value-and-duration-of-the-last-event)
      - [The program also receives OSC messages (default port 9000)](#the-program-also-receives-osc-messages-default-port-9000)
    - [OSC Forwarders](#osc-forwarders)
  - [Troubleshoot / Extra info](#troubleshoot--extra-info)

## To compile

Dependencies are part of the repository as gitsubmodules:

- [SDL](https://github.com/libsdl-org/SDL.git)
- [liblo](https://github.com/radarsat1/liblo.git)
- [jsoncpp](https://github.com/open-source-parsers/jsoncpp.git)

Clone this repository (including the submodules) using `git clone --recurse-submodules https://github.com/Puara/puara-controller.git`

To compile puara-controller, navigate to the cloned folder and execute:

```bash
mkdir build && cd build
cmake ..
make
```
This will create a `puara-controller` executable in the build folder.

## Usage

To execute, navigate to the `build` folder and execute `./puara_controller`

Options:

* -h, --help      Show help message
* -c, --config    Provide a JSON config file

Alternatively, you can run `make install` to install Puara Controller system-wide.

### Current OSC namespace

The standard UDP port to send OSC messages is 9001. 
Puara Controller also receives messages through port 9000.
Both ports, along with other options, can be changed through the JSON config file.

#### Messages sending value and duration of the last event:

* Values can be either 0 or 1, except for triggerleft and triggerright that output integer values between 0 and 32768
* Timestamps are integer values in milliseconds
* X and Y are integer values between -32768 and 32768
* X, Y, and Z are float values between -40 and 40

#### The program also receives OSC messages (default port 9000)

| namespace                | values                                         |
|--------------------------|------------------------------------------------|
| /puaracontroller/rumble  |  iiff  (controllerID, time, low_freq, hi_freq) |

### OSC Forwarders

The JSON config file can also set custom OSC addresses.
This is useful when a specific message needs to send data to another application that doesn't have flexibility in defining namespaces and message ranges.

The [config.json](/config.json) file is set up to send all available game controller data. At this moment it is mandatory to load the file to run the program (using the `-c` option).  

OBS: The range parameter only modifies values and axes (X, Y, and Z). It does not range timestamps or event durations.

You can also add strings and hardcoded OSC arguments. Some examples are available in the [config.json](/config.json) file.

## Troubleshoot / Extra info

Note that you may need to add udev rules to allow accessing motion and force feedback on the controllers. You can add these rules manually in your `/usr/lib/udev/rules.d/` folder. 
One example of such rules for the DualShock3 controller:

```
# DualShock 4 over USB hidraw
KERNEL=="hidraw*", ATTRS{idVendor}=="054c", ATTRS{idProduct}=="05c4", MODE="0660", TAG+="uaccess"

# DualShock 4 wireless adapter over USB hidraw
KERNEL=="hidraw*", ATTRS{idVendor}=="054c", ATTRS{idProduct}=="0ba0", MODE="0660", TAG+="uaccess"

# DualShock 4 Slim over USB hidraw
KERNEL=="hidraw*", ATTRS{idVendor}=="054c", ATTRS{idProduct}=="09cc", MODE="0660", TAG+="uaccess"

# DualShock 4 over bluetooth hidraw
KERNEL=="hidraw*", KERNELS=="*054C:05C4*", MODE="0660", TAG+="uaccess"

# DualShock 4 Slim over bluetooth hidraw
KERNEL=="hidraw*", KERNELS=="*054C:09CC*", MODE="0660", TAG+="uaccess"
```
