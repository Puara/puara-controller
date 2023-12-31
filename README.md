# Puara Controller (Joystick converter/bridge)

This program converts game controller data into OSC.

- [Puara Controller (Joystick converter/bridge)](#puara-controller-joystick-converterbridge)
  - [To compile](#to-compile)
  - [Usage](#usage)
    - [Current OSC namespace](#current-osc-namespace)
      - [Messages sending value and duration of the last event:](#messages-sending-value-and-duration-of-the-last-event)
      - [Messages sending values (x and y) + duration of the last event:](#messages-sending-values-x-and-y--duration-of-the-last-event)
      - [Messages sending x, y, and z values:](#messages-sending-x-y-and-z-values)
      - [The program also receives (default port 9000)](#the-program-also-receives-default-port-9000)
    - [OSC Forwarders](#osc-forwarders)
  - [Troubloshoot / Extra info](#troubloshoot--extra-info)

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

Values can be either 0 or 1, except for triggerleft and triggerright that output integer values between 0 and 32768.
Timestamps are integer values in miliseconds.

| namespace                                     | values                 |
|-----------------------------------------------|------------------------|
| /puaracontroller/<JOYSTICK_ID>/A              | ii  (value, timestamp) |
| /puaracontroller/<JOYSTICK_ID>/B              | ii  (value, timestamp) |
| /puaracontroller/<JOYSTICK_ID>/X              | ii  (value, timestamp) |
| /puaracontroller/<JOYSTICK_ID>/Y              | ii  (value, timestamp) |
| /puaracontroller/<JOYSTICK_ID>/leftstick      | ii  (value, timestamp) |
| /puaracontroller/<JOYSTICK_ID>/rightstick     | ii  (value, timestamp) |
| /puaracontroller/<JOYSTICK_ID>/leftshoulder   | ii  (value, timestamp) |
| /puaracontroller/<JOYSTICK_ID>/rightshoulder  | ii  (value, timestamp) |
| /puaracontroller/<JOYSTICK_ID>/dpad_up        | ii  (value, timestamp) |
| /puaracontroller/<JOYSTICK_ID>/dpad_down      | ii  (value, timestamp) |
| /puaracontroller/<JOYSTICK_ID>/dpad_left      | ii  (value, timestamp) |
| /puaracontroller/<JOYSTICK_ID>/dpad_right     | ii  (value, timestamp) |
| /puaracontroller/<JOYSTICK_ID>/triggerleft    | ii  (value, timestamp) |
| /puaracontroller/<JOYSTICK_ID>/triggerright   | ii  (value, timestamp) |

#### Messages sending values (x and y) + duration of the last event:

X and Y are integer values between -32768 and 32768.
Timestamps are integer values in miliseconds.

| namespace                                   | values                  |
|---------------------------------------------|-------------------------|
| /puaracontroller/<JOYSTICK_ID>/analogleft   | iii  (X, Y, timestamp)  |
| /puaracontroller/<JOYSTICK_ID>/analogright  | iii  (X, Y, timestamp)  |

#### Messages sending x, y, and z values:

X, Y, and Z are float values between -40 and 40.
Timestamps are integer values in miliseconds.

| namespace                             | values                     |
|---------------------------------------|----------------------------|
| /puaracontroller/<JOYSTICK_ID>/accel  | fff  (X, Y, Z, timestamp)  |
| /puaracontroller/<JOYSTICK_ID>/gyro   | fff  (X, Y, Z, timestamp)  |

#### The program also receives (default port 9000)

| namespace                | values                                         |
|--------------------------|------------------------------------------------|
| /puaracontroller/rumble  |  iiff  (controllerID, time, low_freq, hi_freq) |

### OSC Forwarders

The JSON config file can also set OSC addresses for certain messages to be forwarded.
This is useful when a specific message needs to to sent to another application that doesn't have flexibility in defining namespaces and message ranges.

One example of the syntax to add a custom OSC forwarder is:

```json
"forwarders": [
        {
            "internal_address": "A",
            "controller_id": 0,
            "forward_namespace": "/button",
            "forward_arguments": ["value", "timestamp"],
            "range": {"min": 0, "max": 1}
        },
        {
            "internal_address": "triggerright",
            "controller_id": 1,
            "forward_namespace": "/control/volume",
            "forward_arguments": ["value"],
            "range": {"min": 0, "max": 127}
        },
        {
            "internal_address": "accel",
            "controller_id": 0,
            "forward_namespace": "/motion",
            "forward_arguments": ["X", "Z"],
            "range": {"min": -90, "max": 90}
        }
]
```

OBS: The range parameter only modify values and axis (X, Y, and Z). It does not range timestamps.

Keep in mind that is also necessary to set the **forward_address** and **forward_port** parameters in the config section of the JSON file to forward OSC messages.

## Troubloshoot / Extra info

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
