
#ifndef DEFAULT_CONFIG_H
#define DEFAULT_CONFIG_H

const char* defaultConfig = R"(
{
    "config": {
        "polling_frequency": 1000,
        "analog_dead_zone": 2000,
        "disable_motion": false,
        "verbose": false,
        "print_events": false,
        "print_motion_data": false,
        "osc_namespace": "puaracontroller",
        "osc_server_port": 9000,
        "osc_client_address": "localhost",
        "osc_client_port": 9001
    },
    "osc": [
        {
            "controller_id": -1,
            "namespace": "puaracontroller/<ID>/button/A",
            "arguments": [
                {"action": "A", "value": "value", "min": 0, "max": 1},
                {"action": "A", "value": "duration"},
                {"action": "A", "value": "timestamp"},
                {"action": "A", "value": "state"},
                {"action": "A", "value": "custom_text"},
                {"action": "A", "value": "1"},
                {"action": "A", "value": "1.1"}
            ]
        },
        {
            "namespace": "puaracontroller/<ID>/button/B",
            "arguments": [
                {"action": "B", "value": "value", "min": 0, "max": 1},
                {"action": "B", "value": "duration"},
                {"action": "B", "value": "timestamp"}
            ]
        },
        {
            "namespace": "puaracontroller/<ID>/button/X",
            "arguments": [
                {"action": "X", "value": "value", "min": 0, "max": 1},
                {"action": "X", "value": "duration"},
                {"action": "X", "value": "timestamp"}
            ]
        },
        {
            "namespace": "puaracontroller/<ID>/button/Y",
            "arguments": [
                {"action": "Y", "value": "value", "min": 0, "max": 1},
                {"action": "Y", "value": "duration"},
                {"action": "Y", "value": "timestamp"}
            ]
        },
        {
            "namespace": "puaracontroller/<ID>/button/back",
            "arguments": [
                {"action": "back", "value": "value", "min": 0, "max": 1},
                {"action": "back", "value": "duration"},
                {"action": "back", "value": "timestamp"}
            ]
        },
        {
            "namespace": "puaracontroller/<ID>/button/guide",
            "arguments": [
                {"action": "guide", "value": "value", "min": 0, "max": 1},
                {"action": "guide", "value": "duration"},
                {"action": "guide", "value": "timestamp"}
            ]
        },
        {
            "namespace": "puaracontroller/<ID>/button/start",
            "arguments": [
                {"action": "start", "value": "value", "min": 0, "max": 1},
                {"action": "start", "value": "duration"},
                {"action": "start", "value": "timestamp"}
            ]
        },
        {
            "namespace": "puaracontroller/<ID>/button/leftstick",
            "arguments": [
                {"action": "leftstick", "value": "value", "min": 0, "max": 1},
                {"action": "leftstick", "value": "duration"},
                {"action": "leftstick", "value": "timestamp"}
            ]
        },
        {
            "namespace": "puaracontroller/<ID>/button/rightstick",
            "arguments": [
                {"action": "rightstick", "value": "value", "min": 0, "max": 1},
                {"action": "rightstick", "value": "duration"},
                {"action": "rightstick", "value": "timestamp"}
            ]
        },
        {
            "namespace": "puaracontroller/<ID>/button/leftshoulder",
            "arguments": [
                {"action": "leftshoulder", "value": "value", "min": 0, "max": 1},
                {"action": "leftshoulder", "value": "duration"},
                {"action": "leftshoulder", "value": "timestamp"}
            ]
        },
        {
            "namespace": "puaracontroller/<ID>/button/rightshoulder",
            "arguments": [
                {"action": "rightshoulder", "value": "value", "min": 0, "max": 1},
                {"action": "rightshoulder", "value": "duration"},
                {"action": "rightshoulder", "value": "timestamp"}
            ]
        },
        {
            "namespace": "puaracontroller/<ID>/button/dpad_up",
            "arguments": [
                {"action": "dpad_up", "value": "value", "min": 0, "max": 1},
                {"action": "dpad_up", "value": "duration"},
                {"action": "dpad_up", "value": "timestamp"}
            ]
        },
        {
            "namespace": "puaracontroller/<ID>/button/dpad_down",
            "arguments": [
                {"action": "dpad_down", "value": "value", "min": 0, "max": 1},
                {"action": "dpad_down", "value": "duration"},
                {"action": "dpad_down", "value": "timestamp"}
            ]
        },
        {
            "namespace": "puaracontroller/<ID>/button/dpad_left",
            "arguments": [
                {"action": "dpad_left", "value": "value", "min": 0, "max": 1},
                {"action": "dpad_left", "value": "duration"},
                {"action": "dpad_left", "value": "timestamp"}
            ]
        },
        {
            "namespace": "puaracontroller/<ID>/button/dpad_right",
            "arguments": [
                {"action": "dpad_right", "value": "value", "min": 0, "max": 1},
                {"action": "dpad_right", "value": "duration"},
                {"action": "dpad_right", "value": "timestamp"}
            ]
        },
        {
            "namespace": "puaracontroller/<ID>/button/misc1",
            "arguments": [
                {"action": "misc1", "value": "value", "min": 0, "max": 1},
                {"action": "misc1", "value": "duration"},
                {"action": "misc1", "value": "timestamp"}
            ]
        },
        {
            "namespace": "puaracontroller/<ID>/button/paddle1",
            "arguments": [
                {"action": "paddle1", "value": "value", "min": 0, "max": 1},
                {"action": "paddle1", "value": "duration"},
                {"action": "paddle1", "value": "timestamp"}
            ]
        },
        {
            "namespace": "puaracontroller/<ID>/button/paddle2",
            "arguments": [
                {"action": "paddle2", "value": "value", "min": 0, "max": 1},
                {"action": "paddle2", "value": "duration"},
                {"action": "paddle2", "value": "timestamp"}
            ]
        },
        {
            "namespace": "puaracontroller/<ID>/button/paddle3",
            "arguments": [
                {"action": "paddle3", "value": "value", "min": 0, "max": 1},
                {"action": "paddle3", "value": "duration"},
                {"action": "paddle3", "value": "timestamp"}
            ]
        },
        {
            "namespace": "puaracontroller/<ID>/button/paddle4",
            "arguments": [
                {"action": "paddle4", "value": "value", "min": 0, "max": 1},
                {"action": "paddle4", "value": "duration"},
                {"action": "paddle4", "value": "timestamp"}
            ]
        },
        {
            "namespace": "puaracontroller/<ID>/button/touchpadbutton",
            "arguments": [
                {"action": "touchpadbutton", "value": "value", "min": 0, "max": 1},
                {"action": "touchpadbutton", "value": "duration"},
                {"action": "touchpadbutton", "value": "timestamp"}
            ]
        },
        {
            "namespace": "puaracontroller/<ID>/button/analogleft",
            "arguments": [
                {"action": "analogleft", "value": "X", "min": -32768, "max": 32767},
                {"action": "analogleft", "value": "Y", "min": -32768, "max": 32767},
                {"action": "analogleft", "value": "duration"},
                {"action": "analogleft", "value": "timestamp"}
            ]
        },
        {
            "namespace": "puaracontroller/<ID>/button/analogright",
            "arguments": [
                {"action": "analogright", "value": "X", "min": -32768, "max": 32767},
                {"action": "analogright", "value": "Y", "min": -32768, "max": 32767},
                {"action": "analogright", "value": "duration"},
                {"action": "analogright", "value": "timestamp"}
            ]
        },
        {
            "namespace": "puaracontroller/<ID>/button/triggerleft",
            "arguments": [
                {"action": "triggerleft", "value": "value", "min": 0, "max": 1},
                {"action": "triggerleft", "value": "value", "min": 0, "max": 32767},
                {"action": "triggerleft", "value": "duration"},
                {"action": "triggerleft", "value": "timestamp"}
            ]
        },
        {
            "namespace": "puaracontroller/<ID>/button/triggerright",
            "arguments": [
                {"action": "triggerright", "value": "value", "min": 0, "max": 1},
                {"action": "triggerright", "value": "value", "min": 0, "max": 32767},
                {"action": "triggerright", "value": "duration"},
                {"action": "triggerright", "value": "timestamp"}
            ]
        },
        {
            "namespace": "puaracontroller/<ID>/button/accel",
            "arguments": [
                {"action": "accel", "value": "X", "min": -40, "max": 40},
                {"action": "accel", "value": "Y", "min": -40, "max": 40},
                {"action": "accel", "value": "Z", "min": -40, "max": 40},
                {"action": "accel", "value": "duration"},
                {"action": "accel", "value": "timestamp"}
            ]
        },
        {
            "namespace": "puaracontroller/<ID>/button/gyro",
            "arguments": [
                {"action": "gyro", "value": "X", "min": -40, "max": 40},
                {"action": "gyro", "value": "Y", "min": -40, "max": 40},
                {"action": "gyro", "value": "Z", "min": -40, "max": 40},
                {"action": "gyro", "value": "duration"},
                {"action": "gyro", "value": "timestamp"}
            ]
        },
        {
            "namespace": "puaracontroller/<ID>/button/touch",
            "arguments": [
                {"action": "touch", "value": "X", "min": 0.0, "max": 1.0},
                {"action": "touch", "value": "Y", "min": 0.0, "max": 1.0},
                {"action": "touch", "value": "touchpad"},
                {"action": "touch", "value": "finger"},
                {"action": "touch", "value": "duration"},
                {"action": "touch", "value": "timestamp"}
            ]
        }
    ],
    "libmapper": [
        {
            "direction": "in",
            "name": "rumble",
            "arguments": [
                {"action": "id", "min": 0, "max": 10},
                {"action": "time", "min": 0, "max": 10000},
                {"action": "low_freq", "min": 0.0, "max": 1.0},
                {"action": "hi_freq", "min": 0.0, "max": 1.0}
            ]
        },
        {
            "direction": "out",
            "name": "A",
            "arguments": [
                {"action": "A", "value": "value", "min": 0, "max": 1}
            ]
        },
        {
            "direction": "out",
            "name": "analogleft",
            "arguments": [
                {"action": "analogleft", "value": "X", "min": -1.0, "max": 1.0},
                {"action": "analogleft", "value": "Y", "min": -1.0, "max": 1.0}
            ]
        }
    ],
    "midi": [
        {
            "controller_id": -1,
            "type": "on",
            "channel": 0,
            "key": {"action": "int", "value": "60"},
            "velocity": {"action": "A", "value": "value", "min": 0, "max": 127}
        },
        {
            "controller_id": -1,
            "type": "off",
            "channel": 0,
            "key": {"action": "int", "value": "60"},
            "velocity": {"action": "B", "value": 0}
        },
        {
            "controller_id": 0,
            "type": "cc",
            "channel": 0,
            "controller": {"action": "int", "value": "15"},
            "value": {"action": "triggerleft", "value": "value", "min": 127, "max": 0}
            
        }
    ]
}
)";

#endif
