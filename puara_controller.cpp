//****************************************************************************//
// Puara Controller module - connect with game controllers using SDL3 (cpp)   //
//                         Controller -> OSC/MIDI bridge                      //
// https://github.com/Puara/puara-controller                                  //
// Metalab - Société des Arts Technologiques (SAT)                            //
// Input Devices and Music Interaction Laboratory (IDMIL), McGill University  //
// Edu Meneses (2023) - https://www.edumeneses.com                            //
//****************************************************************************//

#include "puara_controller.hpp"


namespace puara_controller {

    // Default values
    std::string identifier = "puaracontroller";
    bool verbose = true;
    int move_buffer_size = 10;
    int analogDeadZone = 1024;
    bool enableMotion = true;
    int polling_frequency = 100; // in hertz
    bool print_events = false;
    bool print_motion_data = false;

    std::int32_t elapsed_time;
    ControllerEvent currentEvent;
    std::atomic<bool> keep_running(true);
    std::condition_variable controller_event;
    std::mutex controller_event_mutex;
    std::vector<std::thread> threads;
    std::thread joiner;

    std::unordered_map<int, Controller> controllers;

    int state2int(const std::string& state) {
        auto it = std::find(State2intArray.begin(), State2intArray.end(), state);
        if (it != State2intArray.end()) {
            return it - State2intArray.begin();
        }
        return -1;
    }

    // int state2int(std::string state){
    //     auto it = State2int.find(state);
    //     if (it != State2int.end()) {
    //         return it->second;
    //     } else {
    //         return -1;
    //     }
    // }

    int start() {
        std::cout << "Starting Puara Controller..." << std::endl;
        if (SDL_Init(SDL_INIT_GAMEPAD | SDL_INIT_SENSOR) < 0) {
            std::cerr << "Could not initialize SDL: " << SDL_GetError() << std::endl;
            return 1;
        } else {
            if (verbose) std::cout << "SDL initialized successfully" << std::endl;
        }
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
        int rangedControllerID = clip(controllerID, 0, (controllers.size()-1));
        int rangedTime = clip(time, 100, 10000);
        float rangedLowFreq = clip(lowFreq, 0.0f, 1.0f) * 65535;
        float rangedHiFreq = clip(hiFreq, 0.0f, 1.0f) * 65535;      
        SDL_RumbleGamepad(controllers[rangedControllerID].instance, rangedLowFreq, rangedHiFreq, time);
        if (verbose) std::cout << "Controller " << rangedControllerID << " rumble!" << std::endl;
        return 0;
    }

    int openController(int joy_index) {
        if (SDL_IsGamepad(joy_index)) {
            std::cout << "New game controller found. Opening..." << std::endl;
            auto it = controllers.emplace(joy_index, Controller());
            it.first->second.id = joy_index;
            it.first->second.instance = SDL_OpenGamepad(joy_index);
            it.first->second.is_open = true;
            // elements in the state map are generated dynamically then pulled from SDLEvent
            std::cout << "\nController \""<< SDL_GetGamepadInstanceName(joy_index)
                      << "\" (" << joy_index << ") " << "opened successfully" << std::endl;
            if (enableMotion) {
                if (SDL_SetGamepadSensorEnabled(it.first->second.instance, SDL_SENSOR_GYRO, SDL_TRUE) < 0)
                    if (verbose) std::cout << "Could not enable the gyroscope for this controller" << std::endl;
                if (SDL_SetGamepadSensorEnabled(it.first->second.instance, SDL_SENSOR_ACCEL, SDL_TRUE) < 0)
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
            bool hasRumble = SDL_GamepadHasRumble(it.first->second.instance);
            bool hasTriggerRumble = SDL_GamepadHasRumbleTriggers(it.first->second.instance);
            std::cout <<"This controller has "
                    << SDL_GetNumGamepadTouchpads(it.first->second.instance) << " touchpad(s) " 
                    << "(can read up to " << SDL_GetNumGamepadTouchpadFingers(it.first->second.instance, 0)
                    << " finger(s) simultaneously), ";
                    if  (SDL_GamepadHasRumble(it.first->second.instance)) {
                        std::cout << "has rumble function, and ";
                    } else {
                        std::cout << "does not have rumble function, and ";
                    }
                    if  (SDL_GamepadHasRumbleTriggers(it.first->second.instance)) {
                        std::cout << "has rumble on triggers." << std::endl;
                    } else {
                        std::cout << "does not have rumble on triggers." << std::endl;
                    }
            return 0;
        } else {
            std::cerr << "Error: the controller is not supported by the game controller interface" << std::endl;
            return 1;
        }
    }

    float clip(float n, float lower, float upper) {
    return std::max(lower, std::min(n, upper));
    }

    int clip(int n, int lower, int upper) {
    return std::max(lower, std::min(n, upper));
    }

    double applyDeadZone(double in, double in_min, double in_max, double out_min, double out_max, double dead_zone_min, double dead_zone_max, double dead_zone_value) {
        if (in_max - in_min == 0) return in;
        double mappedValue = ((in - in_min) / (in_max - in_min)) * (out_max - out_min) + out_min;
        if (mappedValue >= dead_zone_min && mappedValue <= dead_zone_max)
            mappedValue = dead_zone_value;
        return mappedValue;
    }

    int applyAnalogDeadZone(int in) {
        return applyDeadZone(static_cast<double>(in), -32768.0, 32768.0, -32768.0, 32768.0, static_cast<double>(analogDeadZone*-1), static_cast<double>(analogDeadZone), 0.0);
    }

    int pullSDLEvent(SDL_Event event){
        if (event.type == SDL_EVENT_GAMEPAD_ADDED) {
            openController(event.gdevice.which);
            return 0;
        } else if (controllers.count(event.gdevice.which)<=0) {
            return 1;
        }
        auto& controllerState = controllers.at(event.gdevice.which).state;
        switch (event.type) {
            case SDL_EVENT_QUIT:
                quit();
                return 0;
            case SDL_EVENT_GAMEPAD_REMOVED:
                    SDL_CloseGamepad(controllers[event.gdevice.which].instance);
                    controllers.erase(event.gdevice.which);
                    std::cout << "Controller " << event.gdevice.which << " vanished!" << std::endl;
                    return 1;
                break;
            case SDL_EVENT_GAMEPAD_BUTTON_DOWN: case SDL_EVENT_GAMEPAD_BUTTON_UP:
                currentEvent.eventAction = event.gbutton.button;
                currentEvent.eventName = button2Name.at(event.gbutton.button);
                controllerState[currentEvent.eventName].value = event.gbutton.state;
                controllerState[currentEvent.eventName].event_duration = nano2mili(event.gbutton.timestamp - controllerState[currentEvent.eventName].event_timestamp);
                controllerState[currentEvent.eventName].event_timestamp = event.gbutton.timestamp;
                break;
            case SDL_EVENT_GAMEPAD_AXIS_MOTION:
                {
                std::string currentAxisEvent = axis2Name.at(event.gaxis.axis);
                switch (event.gaxis.axis) {
                    case SDL_GAMEPAD_AXIS_LEFTX:
                        if (applyAnalogDeadZone(event.gaxis.value) == controllerState[currentAxisEvent].X) return 1;
                        controllerState[currentAxisEvent].X = applyAnalogDeadZone(event.gaxis.value);
                        controllerState[axis2Name.at(SDL_GAMEPAD_AXIS_LEFTY)].X = controllerState[currentAxisEvent].X;
                        if ( isSensorChanged(event.gdevice.which, event.gaxis.axis, 1) ) {
                            controllerState[currentAxisEvent].event_duration = nano2mili(event.gaxis.timestamp - controllerState[currentAxisEvent].event_timestamp);
                            controllerState[axis2Name.at(SDL_GAMEPAD_AXIS_LEFTY)].event_duration = controllerState[currentAxisEvent].event_duration;
                            controllerState[currentAxisEvent].event_timestamp = event.gaxis.timestamp;
                            controllerState[axis2Name.at(SDL_GAMEPAD_AXIS_LEFTY)].event_timestamp = event.gaxis.timestamp;
                        }
                        break;
                    case SDL_GAMEPAD_AXIS_LEFTY:
                        if (applyAnalogDeadZone(event.gaxis.value) == controllerState[currentAxisEvent].Y) return 1;
                        controllerState[currentAxisEvent].Y = applyAnalogDeadZone(event.gaxis.value);
                        controllerState[axis2Name.at(SDL_GAMEPAD_AXIS_LEFTX)].Y = controllerState[currentAxisEvent].Y;
                        if ( isSensorChanged(event.gdevice.which, event.gaxis.axis, 1) ) {
                            controllerState[currentAxisEvent].event_duration = nano2mili(event.gaxis.timestamp - controllerState[currentAxisEvent].event_timestamp);
                            controllerState[axis2Name.at(SDL_GAMEPAD_AXIS_LEFTX)].event_duration = controllerState[currentAxisEvent].event_duration;
                            controllerState[currentAxisEvent].event_timestamp = event.gaxis.timestamp;
                            controllerState[axis2Name.at(SDL_GAMEPAD_AXIS_LEFTX)].event_timestamp = event.gaxis.timestamp;
                        }
                        break;
                    case SDL_GAMEPAD_AXIS_RIGHTX:
                        if (applyAnalogDeadZone(event.gaxis.value) == controllerState[currentAxisEvent].X) return 1;
                        controllerState[currentAxisEvent].X = applyAnalogDeadZone(event.gaxis.value);
                        controllerState[axis2Name.at(SDL_GAMEPAD_AXIS_RIGHTY)].X = controllerState[currentAxisEvent].X;
                        if ( isSensorChanged(event.gdevice.which, event.gaxis.axis, 1) ) {
                            controllerState[currentAxisEvent].event_duration = nano2mili(event.gaxis.timestamp - controllerState[currentAxisEvent].event_timestamp);
                            controllerState[axis2Name.at(SDL_GAMEPAD_AXIS_RIGHTY)].event_duration = controllerState[currentAxisEvent].event_duration;
                            controllerState[currentAxisEvent].event_timestamp = event.gaxis.timestamp;
                            controllerState[axis2Name.at(SDL_GAMEPAD_AXIS_RIGHTY)].event_timestamp = event.gaxis.timestamp;
                        }
                        break;
                    case SDL_GAMEPAD_AXIS_RIGHTY:
                        if (applyAnalogDeadZone(event.gaxis.value) == controllerState[currentAxisEvent].Y) return 1;
                        controllerState[currentAxisEvent].Y = applyAnalogDeadZone(event.gaxis.value);
                        controllerState[axis2Name.at(SDL_GAMEPAD_AXIS_RIGHTX)].Y = controllerState[currentAxisEvent].Y;
                        if ( isSensorChanged(event.gdevice.which, event.gaxis.axis, 1) ) {
                            controllerState[currentAxisEvent].event_duration = nano2mili(event.gaxis.timestamp - controllerState[currentAxisEvent].event_timestamp);
                            controllerState[axis2Name.at(SDL_GAMEPAD_AXIS_RIGHTX)].event_duration = controllerState[currentAxisEvent].event_duration;
                            controllerState[currentAxisEvent].event_timestamp = event.gaxis.timestamp;
                            controllerState[axis2Name.at(SDL_GAMEPAD_AXIS_RIGHTX)].event_timestamp = event.gaxis.timestamp;
                        }
                        break;
                    case SDL_GAMEPAD_AXIS_LEFT_TRIGGER: case SDL_GAMEPAD_AXIS_RIGHT_TRIGGER:
                        if (applyAnalogDeadZone(event.gaxis.value) == controllerState[currentAxisEvent].value) return 1;
                        controllerState[currentAxisEvent].value = event.gaxis.value;
                        if ( isSensorChanged(event.gdevice.which, event.gaxis.axis, 0) ) {
                            controllerState[currentAxisEvent].event_duration = nano2mili(event.gaxis.timestamp - controllerState[currentAxisEvent].event_timestamp);
                            controllerState[currentAxisEvent].event_timestamp = event.gaxis.timestamp;
                        }
                        break;
                }
                currentEvent.eventAction = event.gaxis.axis;
                currentEvent.eventName = currentAxisEvent;
                break;
                }
            case SDL_EVENT_GAMEPAD_TOUCHPAD_DOWN: case SDL_EVENT_GAMEPAD_TOUCHPAD_MOTION: case SDL_EVENT_GAMEPAD_TOUCHPAD_UP:
                {
                currentEvent.eventAction = 0;
                currentEvent.touchID = event.gtouchpad.touchpad;
                currentEvent.eventName = "touch";
                std::string touchID(1,touchId2Name.at(event.gtouchpad.touchpad));
                currentEvent.touchID = touchID;
                controllerState[touchID].action = event.gtouchpad.type;
                controllerState[touchID].touchpad = event.gtouchpad.touchpad;
                controllerState[touchID].finger = event.gtouchpad.finger;
                controllerState[touchID].X = event.gtouchpad.x;
                controllerState[touchID].Y = event.gtouchpad.y;
                controllerState[touchID].pressure = event.gtouchpad.pressure;
                if ( isSensorChanged(event.gdevice.which, event.gtouchpad.touchpad, 2) ) {
                    controllerState[touchID].event_duration = nano2mili(event.gtouchpad.timestamp - controllerState[touchID].event_timestamp);
                    controllerState[touchID].event_timestamp = event.gtouchpad.timestamp;
                }
                controllerState[touchID].last_X = event.gtouchpad.x;
                controllerState[touchID].last_Y = event.gtouchpad.y;
                break;
                }
            case SDL_EVENT_GAMEPAD_SENSOR_UPDATE:
                currentEvent.eventName = motion2Name.at(event.gsensor.sensor);
                controllerState[currentEvent.eventName].X = event.gsensor.data[0];
                controllerState[currentEvent.eventName].Y = event.gsensor.data[1];
                controllerState[currentEvent.eventName].Z = event.gsensor.data[2];
                currentEvent.eventAction = event.gsensor.sensor;
                break;
            default:
                currentEvent.eventAction = -1;
                return 1;
                break;
        }
        currentEvent.controller = event.gdevice.which;
        currentEvent.eventType = event.type;
        //currentEvent.eventAction = 0;
        // currentEvent.touchID = -1;
        return 0;
    }

    void printEvent(bool printSensor) {
        switch (currentEvent.eventType) {
            case SDL_EVENT_GAMEPAD_BUTTON_DOWN: case SDL_EVENT_GAMEPAD_BUTTON_UP:
                std::cout << "Event on controller " << currentEvent.controller
                        << ": " << button2Name.at(currentEvent.eventAction) 
                        << " " << controllers.at(currentEvent.controller).state.at(currentEvent.eventName).value
                        << " (duration: " << controllers.at(currentEvent.controller).state.at(currentEvent.eventName).event_duration 
                        << ")" << std::endl;
                break;
            case SDL_EVENT_GAMEPAD_AXIS_MOTION:
                std::cout << "Event on controller " << currentEvent.controller << ": ";
                switch (currentEvent.eventAction) {
                    case SDL_GAMEPAD_AXIS_LEFTX: case SDL_GAMEPAD_AXIS_LEFTY:
                        std::cout << " Analog_left " << controllers.at(currentEvent.controller).state.at(currentEvent.eventName).X << " "
                                  << controllers.at(currentEvent.controller).state.at(currentEvent.eventName).Y
                                  << " (duration: " << controllers.at(currentEvent.controller).state.at(currentEvent.eventName).event_duration
                                  << ")" << std::endl;
                        break;
                    case SDL_GAMEPAD_AXIS_RIGHTX: case SDL_GAMEPAD_AXIS_RIGHTY:
                        std::cout << " Analog_right " << controllers.at(currentEvent.controller).state.at(currentEvent.eventName).X << " "
                                << controllers.at(currentEvent.controller).state.at(currentEvent.eventName).Y
                                << " (duration: " << controllers.at(currentEvent.controller).state.at(currentEvent.eventName).event_duration
                                << ")" << std::endl;
                        break;
                    case SDL_GAMEPAD_AXIS_LEFT_TRIGGER:
                        std::cout << " trigger_left " << controllers.at(currentEvent.controller).state.at(currentEvent.eventName).value
                                << " (duration: " << controllers.at(currentEvent.controller).state.at(currentEvent.eventName).event_duration
                                << ")" << std::endl;
                        break;
                    case SDL_GAMEPAD_AXIS_RIGHT_TRIGGER:
                        std::cout << " trigger_right " << controllers.at(currentEvent.controller).state.at(currentEvent.eventName).value
                                << " (duration: " << controllers.at(currentEvent.controller).state.at(currentEvent.eventName).event_duration
                                << ")" << std::endl;
                        break;
                }
                break;
            case SDL_EVENT_GAMEPAD_TOUCHPAD_DOWN: case SDL_EVENT_GAMEPAD_TOUCHPAD_MOTION: case SDL_EVENT_GAMEPAD_TOUCHPAD_UP:
                std::cout << "Event on controller " << currentEvent.controller << ": ";
                    std::cout << " Touchpad " << currentEvent.touchID << ":"
                        << " finger: " << controllers.at(currentEvent.controller).state.at(currentEvent.touchID).finger
                        << "  X: " << controllers.at(currentEvent.controller).state.at(currentEvent.touchID).X
                        << "  Y: " << controllers.at(currentEvent.controller).state.at(currentEvent.touchID).Y
                        << " pressure: " << controllers.at(currentEvent.controller).state.at(currentEvent.touchID).pressure
                        << " duration: " << controllers.at(currentEvent.controller).state.at(currentEvent.touchID).event_duration
                        << std::endl;
                break;
            case SDL_EVENT_GAMEPAD_SENSOR_UPDATE:
                if (printSensor) {
                    std::cout << "Event on controller " << currentEvent.controller
                              << ": " << currentEvent.eventName 
                              << " " << controllers.at(currentEvent.controller).state.at(currentEvent.eventName).X
                              << " " << controllers.at(currentEvent.controller).state.at(currentEvent.eventName).Y
                              << " " << controllers.at(currentEvent.controller).state.at(currentEvent.eventName).Z << std::endl;
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
            for (auto const& i: button2Name) {
                state.emplace(i.second, ControllerState());
            }
            for (auto const& i: motion2Name) {
                state.emplace(i.second, ControllerState());
            }
            state.emplace(axis2Name.at(SDL_GAMEPAD_AXIS_LEFTX), ControllerState());
            state.emplace(axis2Name.at(SDL_GAMEPAD_AXIS_LEFTY), ControllerState());
            state.emplace(axis2Name.at(SDL_GAMEPAD_AXIS_RIGHTX), ControllerState());
            state.emplace(axis2Name.at(SDL_GAMEPAD_AXIS_RIGHTY), ControllerState());
            state.emplace(axis2Name.at(SDL_GAMEPAD_AXIS_LEFT_TRIGGER), ControllerState());
            state.emplace(axis2Name.at(SDL_GAMEPAD_AXIS_RIGHT_TRIGGER), ControllerState());
            //touch
            if (SDL_GetNumGamepadTouchpads(controllers[id].instance) != 0 ) {
                for (int i=0; i<SDL_GetNumGamepadTouchpads(controllers[id].instance); i++) {
                    state.emplace(std::to_string(i),ControllerState());
                }
            }
    }

    // sensorType: 0=trigger, 1=analog, 2=touch
    bool isSensorChanged(int joy_index, int axis, int sensorType) {
        bool convertedValue = false;
        bool answer;
        std::string axisName = axis2Name.at(axis);
        std::string touchID(1,touchId2Name.at(axis));

        // auto& controller = controllers.at(joy_index).state[axis2Name.at(axis)];

        switch (sensorType) {
            case 0: // trigger
                convertedValue = (controllers.at(joy_index).state[axis2Name.at(axis)].value != 0) ? true : false;
                answer = (convertedValue != controllers.at(joy_index).state[axis2Name.at(axis)].state);
                if (answer) {controllers.at(joy_index).state[axis2Name.at(axis)].state = !controllers.at(joy_index).state[axis2Name.at(axis)].state;}
                break;
            case 1: // analog
                convertedValue = (controllers.at(joy_index).state[axis2Name.at(axis)].X == 0 && controllers.at(joy_index).state[axis2Name.at(axis)].Y == 0) ? true : false;
                answer = (convertedValue != controllers.at(joy_index).state[axis2Name.at(axis)].state);
                break;
            case 2: // touch
                if (controllers.at(joy_index).state[touchID].X != controllers.at(joy_index).state[touchID].last_X ||
                    controllers.at(joy_index).state[touchID].Y != controllers.at(joy_index).state[touchID].last_Y) {
                        convertedValue = true;
                }
                answer = convertedValue != controllers[joy_index].state[std::to_string(axis)].state;
                if (answer) {controllers.at(joy_index).state[touchID].state = !controllers.at(joy_index).state[touchID].state;}
                break;
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