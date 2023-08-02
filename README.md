# Puara Controller (Joystick converter/bridge)

This program converts game controller data into OSC.

- [Puara Controller (Joystick converter/bridge)](#puara-controller-joystick-converterbridge)
  - [To compile](#to-compile)
  - [To run](#to-run)
  - [Current OSC namespace](#current-osc-namespace)
    - [Messages sending value and duration of the last event:](#messages-sending-value-and-duration-of-the-last-event)
    - [Messages sending values (x and y) + duration of the last event:](#messages-sending-values-x-and-y--duration-of-the-last-event)
    - [Messages sending x, y, and z values:](#messages-sending-x-y-and-z-values)
    - [The program also receives (port 9000)](#the-program-also-receives-port-9000)

## To compile

Dependencies:

- [libsdl2-dev](https://github.com/libsdl-org/SDL/releases)
- [liblo](https://github.com/radarsat1/liblo)

For Debian-based systems:

```bash
sudo apt install libsdl2-dev liblo-dev liblo-tools
```

Note: The `steam-devices` package adds udev rules to allow accessing motion and force feedback on the controllers. You can add these rules manually in your `/usr/lib/udev/rules.d/`folder. 
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

To compile puara-controller, navigate to the cloned folder and execute:

```bash
mkdir build && cd build
cmake ..
make
```

## To run

After [compilation](#to-compile), you can run the software by executing `./`.

## Current OSC namespace

Default port: 9001

### Messages sending value and duration of the last event:

* /puaracontroller/0/A ii
* /puaracontroller/0/A ii
* /puaracontroller/0/B ii
* /puaracontroller/0/B ii
* /puaracontroller/0/X ii
* /puaracontroller/0/X ii
* /puaracontroller/0/Y ii
* /puaracontroller/0/Y ii
* /puaracontroller/0/leftshoulder ii
* /puaracontroller/0/rightshoulder ii
* /puaracontroller/0/dpad_up ii
* /puaracontroller/0/dpad_down ii
* /puaracontroller/0/dpad_left ii
* /puaracontroller/0/dpad_right ii
* /puaracontroller/0/triggerleft ii
* /puaracontroller/0/triggerright ii

### Messages sending values (x and y) + duration of the last event:

* /puaracontroller/0/analogleft iii
* /puaracontroller/0/analogright iii

### Messages sending x, y, and z values:

* /puaracontroller/0/accel fff
* /puaracontroller/0/gyro fff

### The program also receives (port 9000)

* /puaracontroller/rumble iiff (controllerID, time, low_freq, hi_freq)
