//****************************************************************************//
// Puara Controller module - connect with game controllers using SDL2 (cpp)   //
//                         Controller -> OSC/MIDI bridge                      //
// https://github.com/Puara/puara-controller                                  //
// Metalab - Société des Arts Technologiques (SAT)                            //
// Input Devices and Music Interaction Laboratory (IDMIL), McGill University  //
// Edu Meneses (2023) - https://www.edumeneses.com                            //
//****************************************************************************//

#include "puara_controller.hpp"


namespace puara_controller {

// JM: Internal data like this can be enclosed in an anonymous namespace
// to avoid them being visible to the public (and enabling more compiler optimizations)
//: namespace { ... }

    // Default values
    std::string identifier = "puaracontroller";
    bool verbose = true;
    int move_buffer_size = 10;
    int analogDeadZone = 1024;
    bool enableMotion = true;
    int polling_frequency = 100; // hertz
    bool print_events = false;
    bool print_motion_data = false;

    std::int32_t elapsed_time_;
    ControllerEvent currentEvent;
    std::atomic<bool> keep_running(true);
    std::condition_variable controller_event;
    std::mutex controller_event_mutex;
    std::vector<std::thread> threads;
    std::thread joiner;

    std::unordered_map<int, Controller> controllers;

    // JM: Okay, if this is actually all known before hand we can use something much better than 
    // a std::unordered_map : in traditional GNU code one would have used 
    // gperf to pregenerate a perfect hash table, but in C++ nowadays we can use something like 
    // https://github.com/serge-sans-paille/frozen/blob/master/include/frozen/unordered_map.h 
    // which does it automatically.
    // Also very important: if it's not going to move it should be const!
    // Otherwise you can risk adding a value you don't want by using the [] operator, e.g.
    // SDL2Name[someNewEvent] 
    std::unordered_map<std::string, std::unordered_map<int, std::string>> SDL2Name = {
        {"events",{ /* Supported Puara Controller SDL events */
            {SDL_EVENT_GAMEPAD_ADDED,"added"},
            {SDL_EVENT_GAMEPAD_REMOVED,"removed"},
            {SDL_EVENT_GAMEPAD_BUTTON_DOWN,"button"},
            {SDL_EVENT_GAMEPAD_BUTTON_UP,"button"},
            {SDL_EVENT_GAMEPAD_AXIS_MOTION,"axis"},
            {SDL_EVENT_GAMEPAD_SENSOR_UPDATE,"motion"},
            {SDL_EVENT_GAMEPAD_TOUCHPAD_DOWN,"touch"},
            {SDL_EVENT_GAMEPAD_TOUCHPAD_MOTION,"touch"},
            {SDL_EVENT_GAMEPAD_TOUCHPAD_UP,"touch"}
        }},
        {"button",{ /* This list is generated from SDL_GameControllerButton */
            {SDL_GAMEPAD_BUTTON_INVALID,"invalid"},
            {SDL_GAMEPAD_BUTTON_SOUTH,"A"},
            {SDL_GAMEPAD_BUTTON_EAST,"B"},
            {SDL_GAMEPAD_BUTTON_WEST,"X"},
            {SDL_GAMEPAD_BUTTON_NORTH,"Y"},
            {SDL_GAMEPAD_BUTTON_BACK,"back"},
            {SDL_GAMEPAD_BUTTON_GUIDE,"guide"},
            {SDL_GAMEPAD_BUTTON_START,"start"},
            {SDL_GAMEPAD_BUTTON_LEFT_STICK,"leftstick"},
            {SDL_GAMEPAD_BUTTON_RIGHT_STICK,"rightstick"},
            {SDL_GAMEPAD_BUTTON_LEFT_SHOULDER,"leftshoulder"},
            {SDL_GAMEPAD_BUTTON_RIGHT_SHOULDER,"rightshoulder"},
            {SDL_GAMEPAD_BUTTON_DPAD_UP,"dpad_up"},
            {SDL_GAMEPAD_BUTTON_DPAD_DOWN,"dpad_down"},
            {SDL_GAMEPAD_BUTTON_DPAD_LEFT,"dpad_left"},
            {SDL_GAMEPAD_BUTTON_DPAD_RIGHT,"dpad_right"},
            {SDL_GAMEPAD_BUTTON_MISC1, "misc1"},
            {SDL_GAMEPAD_BUTTON_RIGHT_PADDLE1,"paddle1"},
            {SDL_GAMEPAD_BUTTON_LEFT_PADDLE1,"paddle2"},
            {SDL_GAMEPAD_BUTTON_RIGHT_PADDLE2,"paddle3"},
            {SDL_GAMEPAD_BUTTON_LEFT_PADDLE2,"paddle4"},
            {SDL_GAMEPAD_BUTTON_TOUCHPAD,"touchpadbutton"},
            {SDL_GAMEPAD_BUTTON_MAX,"max_btn"}
        }},
        {"axis",{ /* This list is generated from SDL_GameControllerAxis */
            {SDL_GAMEPAD_AXIS_INVALID,"invalid"},
            {SDL_GAMEPAD_AXIS_LEFTX,"analogleft"},
            {SDL_GAMEPAD_AXIS_LEFTY,"analogleft"},
            {SDL_GAMEPAD_AXIS_RIGHTX,"analogright"},
            {SDL_GAMEPAD_AXIS_RIGHTY,"analogright"},
            {SDL_GAMEPAD_AXIS_LEFT_TRIGGER,"triggerleft"},
            {SDL_GAMEPAD_AXIS_RIGHT_TRIGGER,"triggerright"},
            {SDL_GAMEPAD_AXIS_MAX,"max_axis"}
        }},
        {"motion",{ /* This list is generated from SDL_SensorType */
            {SDL_SENSOR_INVALID,"invalid"},
            {SDL_SENSOR_UNKNOWN,"unknown"},
            {SDL_SENSOR_ACCEL,"accel"},
            {SDL_SENSOR_GYRO,"gyro"}
        }},
        {"touch",{ /* This list allows pushing the sensor name for the touchpad */
            {0,"touch"}
        }}
    };

    // JM: same (though for this size I'dd use a std::array and linear std::find, 
    // for less than 10 elements in general it's the fastest and shortest approach)
    std::unordered_map<std::string, int> State2int = {
        {"value",0},
        {"duration",1},
        {"timestamp",2},
        {"state",3},
        {"X",4},
        {"Y",5},
        {"Z",6},
        {"touchpad",7},
        {"finger",8},
        {"pressure",9}
    };

    int state2int(std::string state){
        auto it = State2int.find(state);
        if (it != State2int.end()) {
            return it->second;
        } else {
            return -1;
        }
    }

    int start() {
        // JM: In general you'd want a proper logging framework where you can easily
        // disable logging if you don't want it, spdlog is recommended (used already in Scenic, ossia, haptic floor...) 
        std::cout << "Starting Puara Controller..." << std::endl;
        if (SDL_Init(SDL_INIT_GAMEPAD | SDL_INIT_SENSOR) < 0) {
            std::cerr << "Could not initialize SDL: " << SDL_GetError() << std::endl;
            return 1;
        } else {
            if (verbose) std::cout << "SDL initialized successfully" << std::endl;
        }
        // JM: Okay, this design from a cursory glance necessitates a larger review :) I think you really want a proper task graph here
        // see e.g. cpp taskflow
        threads.emplace_back(pullControllerEventThread);
        if (print_events) {
            threads.emplace_back(printEventThread);
        }
        joiner = std::thread(joinAllThreads, std::ref(threads));
        std::cout << "Puara Controller started successfully" << std::endl;
        return 0;
    }

    // freq = floats from 0 to 1, time in ms
    int rumble(int controllerID, int time, float lowFreq, float hiFreq) {
        // JM: if controllers.size() == 0 then (controllers.size()-1) == 4 billion due to unsigned underflow
        // which can lead to "fun" behaviours
        int rangedControllerID = clip_(controllerID, 0, (controllers.size()-1));
        int rangedTime = clip_(time, 100, 10000);
        float rangedLowFreq = clip_(lowFreq, 0.0f, 1.0f) * 65535;
        float rangedHiFreq = clip_(hiFreq, 0.0f, 1.0f) * 65535;      
        SDL_RumbleGamepad(controllers[rangedControllerID].instance, rangedLowFreq, rangedHiFreq, time);
        if (verbose) std::cout << "Controller " << rangedControllerID << " rumble!" << std::endl;
        return 0;
    }

    int openController(int joy_index) {
        if (SDL_IsGamepad(joy_index)) {
            std::cout << "New game controller found. Opening..." << std::endl;
            controllers.emplace(joy_index, Controller());
            // JM: Instead of doing a hash lookup each time you can use the return value of emplace directly,
            // it's an iterator to the emplaced value
            controllers[joy_index].id = joy_index;
            controllers[joy_index].instance = SDL_OpenGamepad(joy_index);
            controllers[joy_index].is_open = true;
            std::cout << "\nController \""<< SDL_GetGamepadInstanceName(joy_index)
                      << "\" (" << joy_index << ") " << "opened successfully" << std::endl;
            if (enableMotion) {
                 // JM: spdlog wold allow to set log levels to avoid the if(verbose) :)
                if (SDL_SetGamepadSensorEnabled(controllers[joy_index].instance, SDL_SENSOR_GYRO, SDL_TRUE) < 0)
                    if (verbose) std::cout << "Could not enable the gyroscope for this controller" << std::endl;
                if (SDL_SetGamepadSensorEnabled(controllers[joy_index].instance, SDL_SENSOR_ACCEL, SDL_TRUE) < 0)
                    if (verbose) std::cout << "Could not enable the acclelerometer for this controller" << std::endl;
                switch (SDL_GetGamepadInstanceType(joy_index)){
                    case SDL_GAMEPAD_TYPE_XBOX360: case SDL_GAMEPAD_TYPE_XBOXONE:
                        SDL_SetHint(SDL_HINT_JOYSTICK_HIDAPI_XBOX, "1");
                        break;
                    case SDL_GAMEPAD_TYPE_PS4:
                        SDL_SetHint(SDL_HINT_JOYSTICK_HIDAPI_PS4_RUMBLE, "1");
                        break;
                    case SDL_GAMEPAD_TYPE_PS5:
                        SDL_SetHint(SDL_HINT_JOYSTICK_HIDAPI_PS5_RUMBLE, "1");
                        break;
                }
            }
            std::cout << "This controller has "
                    << SDL_GetNumGamepadTouchpads(controllers[joy_index].instance)
                    << " touchpad(s) and can use up to "
                    << SDL_GetNumGamepadTouchpadFingers(controllers[joy_index].instance, 0)
                    << " finger(s) simultaneously" << std::endl;
            return 0;
        } else {
            std::cerr << "Error: the controller is not supported by the game controller interface" << std::endl;
            return 1;
        }
    }

    // JM: C++ has std::clamp which is a generic implementation of this.
    // See also a tiny bit more optimized version in ossia/detail/math.hpp (ossia::clamp) which uses std::fma
    float clip_(float n, float lower, float upper) {
    return std::max(lower, std::min(n, upper));
    }

    int clip_(int n, int lower, int upper) {
    return std::max(lower, std::min(n, upper));
    }

    double applyDeadZone_(double in, double in_min, double in_max, double out_min, double out_max, double dead_zone_min, double dead_zone_max, double dead_zone_value) {
        // JM: in_max - in_min == 0 -> crash
        double mappedValue = ((in - in_min) / (in_max - in_min)) * (out_max - out_min) + out_min;
        if (mappedValue >= dead_zone_min && mappedValue <= dead_zone_max)
            mappedValue = dead_zone_value;
        return mappedValue;
    }

    int applyAnalogDeadZone(int in) {
        // JM: I remember that to have something centered properly when you have 16 bit values
        // you need to add a -0.5f factor somewhere but I forgot, will investigate.
        // Doesn' matter too much with the deadzone though I think... 
        return applyDeadZone_(static_cast<double>(in), -32768.0, 32768.0, -32768.0, 32768.0, static_cast<double>(analogDeadZone*-1), static_cast<double>(analogDeadZone), 0.0);
    }

    int pullSDLEvent(SDL_Event event){
        if (event.type == SDL_EVENT_QUIT) {
            quit();
        } else if (event.type == SDL_EVENT_GAMEPAD_ADDED) {
            openController(event.gdevice.which);
        }
        // JM: are we sure controllers isn't being used in another thread in this function?
        // I recommend running with -fsanitize=address -fsanitize=undefined first, and then making another build 
        // with -fsanitize=thread. First with gcc and then with clang.
        // Also look up clang thread-safety annotations. 
        switch (event.type) {
            case SDL_EVENT_GAMEPAD_REMOVED: case SDL_EVENT_JOYSTICK_REMOVED:
                    SDL_CloseGamepad(controllers[event.gdevice.which].instance);
                    controllers.erase(event.gdevice.which);
                    std::cout << "Controller " << event.gdevice.which << " vanished!" << std::endl;
                    return 1;
                break;
            case SDL_EVENT_GAMEPAD_BUTTON_DOWN: case SDL_EVENT_GAMEPAD_BUTTON_UP:
                controllers[event.gdevice.which].state[SDL2Name["button"][event.gbutton.button]].value = event.gbutton.state;
                currentEvent.eventAction = event.gbutton.button;
                currentEvent.eventName = SDL2Name["button"][event.gbutton.button];
                // That's a lot of lookup, I'd just take a reference to   controllers[event.gdevice.which].state[SDL2Name["button"][event.gbutton.button]] 
                // and work on that
                controllers[event.gdevice.which].state[SDL2Name["button"][event.gbutton.button]].event_duration = nano2mili(event.gbutton.timestamp - controllers[event.gdevice.which].state[SDL2Name["button"][event.gbutton.button]].event_timestamp);
                controllers[event.gdevice.which].state[SDL2Name["button"][event.gbutton.button]].event_timestamp = event.gbutton.timestamp;
                break;
            case SDL_EVENT_GAMEPAD_AXIS_MOTION:
                switch (event.gaxis.axis) {
                    case SDL_GAMEPAD_AXIS_LEFTX:
                        if (applyAnalogDeadZone(event.gaxis.value) == controllers[event.gdevice.which].state[SDL2Name["axis"][SDL_GAMEPAD_AXIS_LEFTX]].X) return 1;
                        controllers[event.gdevice.which].state[SDL2Name["axis"][SDL_GAMEPAD_AXIS_LEFTX]].X = controllers[event.gdevice.which].state[SDL2Name["axis"][SDL_GAMEPAD_AXIS_LEFTY]].X = applyAnalogDeadZone(event.gaxis.value);
                        if ( isSensorChanged(event.gdevice.which, event.gaxis.axis, "analog") ) {
                            controllers[event.gdevice.which].state[SDL2Name["axis"][SDL_GAMEPAD_AXIS_LEFTX]].event_duration = controllers[event.gdevice.which].state[SDL2Name["axis"][SDL_GAMEPAD_AXIS_LEFTY]].event_duration = nano2mili(event.gaxis.timestamp - controllers[event.gdevice.which].state[SDL2Name["axis"][SDL_GAMEPAD_AXIS_LEFTX]].event_timestamp);
                            controllers[event.gdevice.which].state[SDL2Name["axis"][SDL_GAMEPAD_AXIS_LEFTX]].event_timestamp = controllers[event.gdevice.which].state[SDL2Name["axis"][SDL_GAMEPAD_AXIS_LEFTY]].event_timestamp = event.gaxis.timestamp;
                        }
                        break;
                    case SDL_GAMEPAD_AXIS_LEFTY:
                        if (applyAnalogDeadZone(event.gaxis.value) == controllers[event.gdevice.which].state[SDL2Name["axis"][SDL_GAMEPAD_AXIS_LEFTX]].Y) return 1;
                        controllers[event.gdevice.which].state[SDL2Name["axis"][SDL_GAMEPAD_AXIS_LEFTX]].Y = controllers[event.gdevice.which].state[SDL2Name["axis"][SDL_GAMEPAD_AXIS_LEFTY]].Y = applyAnalogDeadZone(event.gaxis.value);
                        if ( isSensorChanged(event.gdevice.which, event.gaxis.axis, "analog") ) {
                            controllers[event.gdevice.which].state[SDL2Name["axis"][SDL_GAMEPAD_AXIS_LEFTX]].event_duration = controllers[event.gdevice.which].state[SDL2Name["axis"][SDL_GAMEPAD_AXIS_LEFTY]].event_duration = nano2mili(event.gaxis.timestamp - controllers[event.gdevice.which].state[SDL2Name["axis"][SDL_GAMEPAD_AXIS_LEFTX]].event_timestamp);
                            controllers[event.gdevice.which].state[SDL2Name["axis"][SDL_GAMEPAD_AXIS_LEFTX]].event_timestamp = controllers[event.gdevice.which].state[SDL2Name["axis"][SDL_GAMEPAD_AXIS_LEFTY]].event_timestamp = event.gaxis.timestamp;
                        }
                        break;
                    case SDL_GAMEPAD_AXIS_RIGHTX:
                        if (applyAnalogDeadZone(event.gaxis.value) == controllers[event.gdevice.which].state[SDL2Name["axis"][SDL_GAMEPAD_AXIS_RIGHTX]].X) return 1;
                        controllers[event.gdevice.which].state[SDL2Name["axis"][SDL_GAMEPAD_AXIS_RIGHTX]].X = controllers[event.gdevice.which].state[SDL2Name["axis"][SDL_GAMEPAD_AXIS_RIGHTY]].X = applyAnalogDeadZone(event.gaxis.value);
                        if ( isSensorChanged(event.gdevice.which, event.gaxis.axis, "analog") ) {
                            controllers[event.gdevice.which].state[SDL2Name["axis"][SDL_GAMEPAD_AXIS_RIGHTX]].event_duration = controllers[event.gdevice.which].state[SDL2Name["axis"][SDL_GAMEPAD_AXIS_RIGHTY]].event_duration = nano2mili(event.gaxis.timestamp - controllers[event.gdevice.which].state[SDL2Name["axis"][SDL_GAMEPAD_AXIS_RIGHTX]].event_timestamp);
                            controllers[event.gdevice.which].state[SDL2Name["axis"][SDL_GAMEPAD_AXIS_RIGHTX]].event_timestamp = controllers[event.gdevice.which].state[SDL2Name["axis"][SDL_GAMEPAD_AXIS_RIGHTY]].event_timestamp = event.gaxis.timestamp;
                        }
                        break;
                    case SDL_GAMEPAD_AXIS_RIGHTY:
                        if (applyAnalogDeadZone(event.gaxis.value) == controllers[event.gdevice.which].state[SDL2Name["axis"][SDL_GAMEPAD_AXIS_RIGHTX]].Y) return 1;
                        controllers[event.gdevice.which].state[SDL2Name["axis"][SDL_GAMEPAD_AXIS_RIGHTX]].Y = controllers[event.gdevice.which].state[SDL2Name["axis"][SDL_GAMEPAD_AXIS_RIGHTY]].Y = applyAnalogDeadZone(event.gaxis.value);
                        if ( isSensorChanged(event.gdevice.which, event.gaxis.axis, "analog") ) {
                            controllers[event.gdevice.which].state[SDL2Name["axis"][SDL_GAMEPAD_AXIS_RIGHTX]].event_duration = controllers[event.gdevice.which].state[SDL2Name["axis"][SDL_GAMEPAD_AXIS_RIGHTY]].event_duration = nano2mili(event.gaxis.timestamp - controllers[event.gdevice.which].state[SDL2Name["axis"][SDL_GAMEPAD_AXIS_RIGHTX]].event_timestamp);
                            controllers[event.gdevice.which].state[SDL2Name["axis"][SDL_GAMEPAD_AXIS_RIGHTX]].event_timestamp = controllers[event.gdevice.which].state[SDL2Name["axis"][SDL_GAMEPAD_AXIS_RIGHTY]].event_timestamp = event.gaxis.timestamp;
                        }
                        break;
                    case SDL_GAMEPAD_AXIS_LEFT_TRIGGER:
                        if (applyAnalogDeadZone(event.gaxis.value) == controllers[event.gdevice.which].state[SDL2Name["axis"][SDL_GAMEPAD_AXIS_LEFT_TRIGGER]].value) return 1;
                        controllers[event.gdevice.which].state[SDL2Name["axis"][SDL_GAMEPAD_AXIS_LEFT_TRIGGER]].value = event.gaxis.value;
                        if ( isSensorChanged(event.gdevice.which, event.gaxis.axis, "trigger") ) {
                            controllers[event.gdevice.which].state[SDL2Name["axis"][SDL_GAMEPAD_AXIS_LEFT_TRIGGER]].event_duration = nano2mili(event.gaxis.timestamp - controllers[event.gdevice.which].state[SDL2Name["axis"][SDL_GAMEPAD_AXIS_LEFT_TRIGGER]].event_timestamp);
                            controllers[event.gdevice.which].state[SDL2Name["axis"][SDL_GAMEPAD_AXIS_LEFT_TRIGGER]].event_timestamp = event.gaxis.timestamp;
                        }
                        break;
                    case SDL_GAMEPAD_AXIS_RIGHT_TRIGGER:
                        if (applyAnalogDeadZone(event.gaxis.value) == controllers[event.gdevice.which].state[SDL2Name["axis"][SDL_GAMEPAD_AXIS_RIGHT_TRIGGER]].value) return 1;
                        controllers[event.gdevice.which].state[SDL2Name["axis"][SDL_GAMEPAD_AXIS_RIGHT_TRIGGER]].value = event.gaxis.value;
                        if ( isSensorChanged(event.gdevice.which, event.gaxis.axis, "trigger") ) {
                            controllers[event.gdevice.which].state[SDL2Name["axis"][SDL_GAMEPAD_AXIS_RIGHT_TRIGGER]].event_duration = nano2mili(event.gaxis.timestamp - controllers[event.gdevice.which].state[SDL2Name["axis"][SDL_GAMEPAD_AXIS_RIGHT_TRIGGER]].event_timestamp);
                            controllers[event.gdevice.which].state[SDL2Name["axis"][SDL_GAMEPAD_AXIS_RIGHT_TRIGGER]].event_timestamp = event.gaxis.timestamp;
                        }
                        break;
                }
                currentEvent.eventAction = event.gaxis.axis;
                currentEvent.eventName = SDL2Name["axis"][event.gaxis.axis];
                break;
            case SDL_EVENT_GAMEPAD_TOUCHPAD_DOWN: case SDL_EVENT_GAMEPAD_TOUCHPAD_MOTION: case SDL_EVENT_GAMEPAD_TOUCHPAD_UP:
                currentEvent.eventAction = 0;
                currentEvent.touchID = event.gtouchpad.touchpad;
                currentEvent.eventName = SDL2Name["touch"][currentEvent.eventAction];
                // JM: std::to_string can be pretty slow, it touches locale and all this. 
                // Check out std::from_chars / to_chars (and do it only once, even then it's a complex algorithm.
                // Or better, precompute a small table of numbers from 0 to 255? :)
                controllers[event.gdevice.which].state[std::to_string(event.gtouchpad.touchpad)].action = event.gtouchpad.type;
                controllers[event.gdevice.which].state[std::to_string(event.gtouchpad.touchpad)].touchpad = event.gtouchpad.touchpad;
                controllers[event.gdevice.which].state[std::to_string(event.gtouchpad.touchpad)].finger = event.gtouchpad.finger;
                controllers[event.gdevice.which].state[std::to_string(event.gtouchpad.touchpad)].X = event.gtouchpad.x;
                controllers[event.gdevice.which].state[std::to_string(event.gtouchpad.touchpad)].Y = event.gtouchpad.y;
                controllers[event.gdevice.which].state[std::to_string(event.gtouchpad.touchpad)].pressure = event.gtouchpad.pressure;
                if ( isSensorChanged(event.gdevice.which, event.gtouchpad.touchpad, "touch") ) {
                    controllers[event.gdevice.which].state[std::to_string(event.gtouchpad.touchpad)].event_duration = nano2mili(event.gtouchpad.timestamp - controllers[event.gdevice.which].state[std::to_string(event.gtouchpad.touchpad)].event_timestamp);
                    controllers[event.gdevice.which].state[std::to_string(event.gtouchpad.touchpad)].event_timestamp = event.gtouchpad.timestamp;
                }
                controllers[event.gdevice.which].state[std::to_string(event.gtouchpad.touchpad)].last_X = event.gtouchpad.x;
                controllers[event.gdevice.which].state[std::to_string(event.gtouchpad.touchpad)].last_Y = event.gtouchpad.y;
                break;
            case SDL_EVENT_GAMEPAD_SENSOR_UPDATE:
                if (event.gsensor.sensor == SDL_SENSOR_ACCEL) {
                    controllers[event.gdevice.which].state[SDL2Name["motion"][SDL_SENSOR_ACCEL]].X = event.gsensor.data[0];
                    controllers[event.gdevice.which].state[SDL2Name["motion"][SDL_SENSOR_ACCEL]].Y = event.gsensor.data[1];
                    controllers[event.gdevice.which].state[SDL2Name["motion"][SDL_SENSOR_ACCEL]].Z = event.gsensor.data[2];
                };
                if (event.gsensor.sensor == SDL_SENSOR_GYRO) {
                    controllers[event.gdevice.which].state[SDL2Name["motion"][SDL_SENSOR_GYRO]].X = event.gsensor.data[0];
                    controllers[event.gdevice.which].state[SDL2Name["motion"][SDL_SENSOR_GYRO]].Y = event.gsensor.data[1];
                    controllers[event.gdevice.which].state[SDL2Name["motion"][SDL_SENSOR_GYRO]].Z = event.gsensor.data[2];
                };
                currentEvent.eventAction = event.gsensor.sensor;
                currentEvent.eventName = SDL2Name["motion"][event.gsensor.sensor];
                break;
            default:
                currentEvent.eventAction = -1;
                return 1;
                break;
        }
        currentEvent.controller = event.gdevice.which;
        currentEvent.eventType = event.type;
        currentEvent.eventAction = 0;
        currentEvent.touchID = -1;
        return 0;
    }

    void printEvent(bool printSensor) {
        switch (currentEvent.eventType) {
            case SDL_EVENT_GAMEPAD_BUTTON_DOWN: case SDL_EVENT_GAMEPAD_BUTTON_UP:
                std::cout << "Event on controller " << currentEvent.controller
                        << ": " << SDL2Name[SDL2Name["events"][currentEvent.eventType]][currentEvent.eventAction]
                        << " " << controllers[currentEvent.controller].state[SDL2Name["button"][currentEvent.eventAction]].value
                        << " (duration: " << controllers[currentEvent.controller].state[SDL2Name["button"][currentEvent.eventAction]].event_duration
                        << ")" << std::endl;
                break;
            case SDL_EVENT_GAMEPAD_AXIS_MOTION:
                std::cout << "Event on controller " << currentEvent.controller << ": ";
                switch (currentEvent.eventAction) {
                    case SDL_GAMEPAD_AXIS_LEFTX: case SDL_GAMEPAD_AXIS_LEFTY:
                        std::cout << " Analog_left " << controllers[currentEvent.controller].state[SDL2Name["axis"][SDL_GAMEPAD_AXIS_LEFTX]].X << " "
                                << controllers[currentEvent.controller].state[SDL2Name["axis"][SDL_GAMEPAD_AXIS_LEFTX]].Y
                                << " (duration: " << controllers[currentEvent.controller].state[SDL2Name["axis"][SDL_GAMEPAD_AXIS_LEFTX]].event_duration
                                << ")" << std::endl;
                        break;
                    case SDL_GAMEPAD_AXIS_RIGHTX: case SDL_GAMEPAD_AXIS_RIGHTY:
                        std::cout << " Analog_right " << controllers[currentEvent.controller].state[SDL2Name["axis"][SDL_GAMEPAD_AXIS_RIGHTX]].X << " "
                                << controllers[currentEvent.controller].state[SDL2Name["axis"][SDL_GAMEPAD_AXIS_RIGHTX]].Y
                                << " (duration: " << controllers[currentEvent.controller].state[SDL2Name["axis"][SDL_GAMEPAD_AXIS_RIGHTX]].event_duration
                                << ")" << std::endl;
                        break;
                    case SDL_GAMEPAD_AXIS_LEFT_TRIGGER:
                        std::cout << " trigger_left " << controllers[currentEvent.controller].state[SDL2Name["axis"][SDL_GAMEPAD_AXIS_LEFT_TRIGGER]].value
                                << " (duration: " << controllers[currentEvent.controller].state[SDL2Name["axis"][SDL_GAMEPAD_AXIS_LEFT_TRIGGER]].event_duration
                                << ")" << std::endl;
                        break;
                    case SDL_GAMEPAD_AXIS_RIGHT_TRIGGER:
                        std::cout << " trigger_right " << controllers[currentEvent.controller].state[SDL2Name["axis"][SDL_GAMEPAD_AXIS_RIGHT_TRIGGER]].value
                                << " (duration: " << controllers[currentEvent.controller].state[SDL2Name["axis"][SDL_GAMEPAD_AXIS_RIGHT_TRIGGER]].event_duration
                                << ")" << std::endl;
                        break;
                }
                break;
            case SDL_EVENT_GAMEPAD_TOUCHPAD_DOWN: case SDL_EVENT_GAMEPAD_TOUCHPAD_MOTION: case SDL_EVENT_GAMEPAD_TOUCHPAD_UP:
                std::cout << "Event on controller " << currentEvent.controller << ": ";
                    std::cout << " Touchpad " << currentEvent.touchID << ":"
                        << " finger: " << controllers[currentEvent.controller].state[std::to_string(currentEvent.touchID)].finger
                        << "  X: " << controllers[currentEvent.controller].state[std::to_string(currentEvent.touchID)].X
                        << "  Y: " << controllers[currentEvent.controller].state[std::to_string(currentEvent.touchID)].Y
                        << " pressure: " << controllers[currentEvent.controller].state[std::to_string(currentEvent.touchID)].pressure
                        << " duration: " << controllers[currentEvent.controller].state[std::to_string(currentEvent.touchID)].event_duration
                        << std::endl;
                break;
            case SDL_EVENT_GAMEPAD_SENSOR_UPDATE:
                if (printSensor) {
                    std::cout << "Event on controller " << currentEvent.controller
                              << ": " << SDL2Name[SDL2Name["events"][currentEvent.eventType]][currentEvent.eventAction];
                    if (currentEvent.eventAction == SDL_SENSOR_ACCEL) {
                        std::cout << " " << controllers[currentEvent.controller].state[SDL2Name["motion"][SDL_SENSOR_ACCEL]].X
                                  << " " << controllers[currentEvent.controller].state[SDL2Name["motion"][SDL_SENSOR_ACCEL]].Y
                                  << " " << controllers[currentEvent.controller].state[SDL2Name["motion"][SDL_SENSOR_ACCEL]].Z << std::endl;
                    } else if (currentEvent.eventAction == SDL_SENSOR_GYRO) {
                        std::cout << " " << controllers[currentEvent.controller].state[SDL2Name["motion"][SDL_SENSOR_GYRO]].X
                                  << " " << controllers[currentEvent.controller].state[SDL2Name["motion"][SDL_SENSOR_GYRO]].Y
                                  << " " << controllers[currentEvent.controller].state[SDL2Name["motion"][SDL_SENSOR_GYRO]].Z << std::endl;
                    }
                }
                break;
        }
    }

    void printEvent() {
        printEvent(false);
    }

    void pullControllerEventThread() {
        SDL_Event sdlEvent;
        while (keep_running.load()) {
            if (SDL_PollEvent( &sdlEvent ) != 0) {
                if (!pullSDLEvent(sdlEvent)) {
                    std::lock_guard<std::mutex> lock(controller_event_mutex);
                    controller_event.notify_all();
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1000/polling_frequency));
        }
    }

    void printEventThread() {
        while (keep_running.load()) {
            std::unique_lock<std::mutex> lock(controller_event_mutex);
            controller_event.wait(lock);
            printEvent(print_motion_data);
        }
    }

    void joinAllThreads(std::vector<std::thread>& threads) {
        for (auto& thread : threads) {
            if (thread.joinable()) {
                thread.join();
            }
        }
    }

    void quit(){
        SDL_Quit();
        keep_running.store(false);
        controller_event.notify_all();
        joiner.join();
    }

    template<typename T>
    CircularBuffer<T>::CircularBuffer(size_t capacity)
        : capacity_(capacity), size_(0), read_index_(0), write_index_(0) {
        buffer_.resize(capacity_);
    }

    template<typename T>
    bool CircularBuffer<T>::isEmpty() const {
        return size_ == 0;
    }

    template<typename T>
    bool CircularBuffer<T>::isFull() const {
        return size_ == capacity_;
    }

    template<typename T>
    void CircularBuffer<T>::push(const T& item) {
        if ( isFull() ) pop();
        buffer_[write_index_] = item;
        write_index_ = (write_index_ + 1) % capacity_;
        size_++;
    }

    template<typename T>
    T CircularBuffer<T>::pop() {
        if (isEmpty()) {
            std::cerr << "Circular buffer is empty. Unable to pop item." << std::endl;
            return T();
        }
        T item = buffer_[read_index_];
        read_index_ = (read_index_ + 1) % capacity_;
        size_--;
        return item;
    }

    Controller::Controller(int id, SDL_Gamepad* instance, int move_buffer_size)
        : id(id), instance(instance), discrete_buffer(move_buffer_size), is_open(true) {
            for (auto const& i: SDL2Name["button"]) {
                state.emplace(i.second, ControllerState());
            }
            for (auto const& i: SDL2Name["motion"]) {
                state.emplace(i.second, ControllerState());
            }
            state.emplace(SDL2Name["axis"][SDL_GAMEPAD_AXIS_LEFTX], ControllerState());
            state.emplace(SDL2Name["axis"][SDL_GAMEPAD_AXIS_LEFTY], ControllerState());
            state.emplace(SDL2Name["axis"][SDL_GAMEPAD_AXIS_RIGHTX], ControllerState());
            state.emplace(SDL2Name["axis"][SDL_GAMEPAD_AXIS_RIGHTY], ControllerState());
            state.emplace(SDL2Name["axis"][SDL_GAMEPAD_AXIS_LEFT_TRIGGER], ControllerState());
            state.emplace(SDL2Name["axis"][SDL_GAMEPAD_AXIS_RIGHT_TRIGGER], ControllerState());
            //touch
            if (SDL_GetNumGamepadTouchpads(controllers[id].instance) != 0 ) {
                for (int i=0; i<SDL_GetNumGamepadTouchpads(controllers[id].instance); i++) {
                    state.emplace(std::to_string(i),ControllerState());
                }
            }
    }

    bool isSensorChanged(int joy_index, int axis, std::string sensorType) {
        bool convertedValue = false;
        bool answer;
        if (sensorType == "trigger") {
            if (controllers[joy_index].state[SDL2Name["axis"][axis]].value != 0) {convertedValue = true;}
            answer = convertedValue != controllers[joy_index].state[SDL2Name["axis"][axis]].state;
            if (answer) {controllers[joy_index].state[SDL2Name["axis"][axis]].state = !controllers[joy_index].state[SDL2Name["axis"][axis]].state;}
        } else if (sensorType == "analog") {
            if (controllers[joy_index].state[SDL2Name["axis"][axis]].X == 0 && controllers[joy_index].state[SDL2Name["axis"][axis]].Y ==0) {
                convertedValue = false;
            } else {
                convertedValue = true;
            }
            answer = convertedValue != controllers[joy_index].state[SDL2Name["axis"][axis]].state;
            if (answer) {controllers[joy_index].state[SDL2Name["axis"][axis]].state = !controllers[joy_index].state[SDL2Name["axis"][axis]].state;}
        } else if (sensorType == "touch") {
            if (controllers[joy_index].state[std::to_string(axis)].X != controllers[joy_index].state[std::to_string(axis)].last_X ||
                controllers[joy_index].state[std::to_string(axis)].Y != controllers[joy_index].state[std::to_string(axis)].last_Y) {
                    convertedValue = true;
            }
            answer = convertedValue != controllers[joy_index].state[std::to_string(axis)].state;
            if (answer) {controllers[joy_index].state[std::to_string(axis)].state = !controllers[joy_index].state[std::to_string(axis)].state;}
        }
        return answer;
    }

    int nano2mili(Uint64 ns) {
        return static_cast<int>(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::nanoseconds(ns)).count());
    }

    std::string replaceID(std::string str, int newID) {
        std::string oldID = "<ID>";
        size_t pos = str.find(oldID);
        if (pos != std::string::npos) {
            std::stringstream newIDstr;
            newIDstr << newID;
            return str.replace(pos, oldID.length(), newIDstr.str());
        }
        return str;
    }
}