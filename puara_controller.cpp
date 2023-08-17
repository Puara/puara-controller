//****************************************************************************//
// Puara Controller module - connect with game controllers using SDL2 (cpp)   //
//                         Controller -> OSC/MIDI bridge                      //
// https://github.com/Puara/puara-controller                                  //
// Metalab - Société des Arts Technologiques (SAT)                            //
// Input Devices and Music Interaction Laboratory (IDMIL), McGill University  //
// Edu Meneses (2023) - https://www.edumeneses.com                            //
//****************************************************************************//

#include "puara_controller.hpp"

std::unordered_map<std::string, std::unordered_map<int, std::string>> PuaraController::SDL2Name = {
    {"events",{ /* Supported Puara Controller SDL events */
        {SDL_EVENT_GAMEPAD_ADDED,"added"},
        {SDL_EVENT_GAMEPAD_REMOVED,"removed"},
        {SDL_EVENT_GAMEPAD_BUTTON_DOWN,"button"},
        {SDL_EVENT_GAMEPAD_BUTTON_UP,"button"},
        {SDL_EVENT_GAMEPAD_AXIS_MOTION,"axis"},
        {SDL_EVENT_GAMEPAD_SENSOR_UPDATE,"motion"},
        {SDL_EVENT_GAMEPAD_TOUCHPAD_DOWN,"touch"},
        {SDL_EVENT_GAMEPAD_TOUCHPAD_MOTION,"touch"},
        {SDL_EVENT_GAMEPAD_TOUCHPAD_UP,"touch"},
        //{SDL_MULTIGESTURE,"touch"}
    }},
    {"button",{ /* This list is generated from SDL_GameControllerButton */
        {SDL_GAMEPAD_BUTTON_INVALID,"invalid"},
        {SDL_GAMEPAD_BUTTON_A,"A"},
        {SDL_GAMEPAD_BUTTON_B,"B"},
        {SDL_GAMEPAD_BUTTON_X,"X"},
        {SDL_GAMEPAD_BUTTON_Y,"Y"},
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

int PuaraController::start() {
    std::cout << "Starting Puara Controller..." << std::endl;
    if (SDL_Init(SDL_INIT_GAMEPAD | SDL_INIT_SENSOR) < 0) {
        std::cerr << "Could not initialize sdl2: " << SDL_GetError() << std::endl;
        return 1;
    } else {
        if (verbose) std::cout << "SDL2 initialized successfully" << std::endl;
    }
    std::cout << "Puara Controller started successfully" << std::endl;
    return 0;
};

int PuaraController::rumble(int controllerID, int time=1000, float lowFreq=1.0, float hiFreq=1.0f) {
    int rangedControllerID = clip_(controllerID, 0, (controllers.size()-1));
    int rangedTime = clip_(time, 10, 10000);
    float rangedLowFreq = clip_(lowFreq, 0.0f, 1.0f) * 65535;
    float rangedHiFreq = clip_(hiFreq, 0.0f, 1.0f) * 65535;
    SDL_RumbleGamepad(controllers[rangedControllerID].instance, rangedLowFreq, rangedHiFreq, time);
    if (verbose) std::cout << "Controller " << rangedControllerID << " rumble!" << std::endl;
    return 0;
}

int PuaraController::openController(int joy_index) {
    if (SDL_IsGamepad(joy_index)) {
        std::cout << "New game controller found. Opening...\n" << std::endl;
        Controller controller_instance(joy_index, SDL_OpenGamepad(joy_index), move_buffer_size);
        controllers.insert({joy_index, controller_instance});
        if (controllers[joy_index].isOpen) {
            std::cout << "\nController \""<< SDL_GetGamepadInstanceName(joy_index) 
                      << "\" (" << joy_index << ") " << "opened successfully" << std::endl;
        }
        if (enableMotion) {
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

float PuaraController::clip_(float n, float lower, float upper) {
  return std::max(lower, std::min(n, upper));
}

int PuaraController::clip_(int n, int lower, int upper) {
  return std::max(lower, std::min(n, upper));
}

double PuaraController::applyDeadZone_(double in, double in_min, double in_max, double out_min, double out_max, double dead_zone_min, double dead_zone_max, double dead_zone_value) {
    double mappedValue = ((in - in_min) / (in_max - in_min)) * (out_max - out_min) + out_min;
    if (mappedValue >= dead_zone_min && mappedValue <= dead_zone_max)
        mappedValue = dead_zone_value;
    return mappedValue;
}

int PuaraController::applyAnalogDeadZone_(int in) {
    return applyDeadZone_(static_cast<double>(in), -32768.0, 32768.0, -32768.0, 32768.0, static_cast<double>(analogDeadZone*-1), static_cast<double>(analogDeadZone), 0.0);
}

PuaraController::EventResume PuaraController::pullSDLEvent(SDL_Event event){
    EventResume answer = {
        .controller = event.gdevice.which,
        .eventType = event.type,
        .eventAction = 0
    };
    if (event.type == SDL_EVENT_QUIT) {
        sdl_quit_ = true;
    } else if (event.type == SDL_EVENT_GAMEPAD_ADDED) {
        openController(event.gdevice.which);
    }
    switch (event.type) {
        case SDL_EVENT_GAMEPAD_REMOVED:
                SDL_CloseGamepad(controllers[event.gdevice.which].instance);
                controllers.erase(event.gdevice.which);
                if (verbose) std::cout << "Controller " << event.gdevice.which << " vanished!" << std::endl;
            break;
        case SDL_EVENT_GAMEPAD_BUTTON_DOWN: case SDL_EVENT_GAMEPAD_BUTTON_UP:
            controllers[event.gdevice.which].state.button[event.gbutton.button].value = event.gbutton.state;
            answer.eventAction = event.gbutton.button;
            answer.eventName = SDL2Name["button"][event.gbutton.button];
            controllers[event.gdevice.which].state.button[event.gbutton.button].event_duration = event.gbutton.timestamp - controllers[event.gdevice.which].state.button[event.gbutton.button].event_timestamp;
            controllers[event.gdevice.which].state.button[event.gbutton.button].event_timestamp = event.gbutton.timestamp;
            break;
        case SDL_EVENT_GAMEPAD_AXIS_MOTION:
            switch (event.gaxis.axis) {
                case SDL_GAMEPAD_AXIS_LEFTX:
                    controllers[event.gdevice.which].state.analogL.X = applyAnalogDeadZone_(event.gaxis.value);
                    if ( isSensorChanged_(event.gdevice.which, "analog", "l") ) {
                        controllers[event.gdevice.which].state.analogL.event_duration = event.gaxis.timestamp - controllers[event.gdevice.which].state.analogL.event_timestamp;
                        controllers[event.gdevice.which].state.analogL.event_timestamp = event.gaxis.timestamp;
                    }
                    break;
                case SDL_GAMEPAD_AXIS_LEFTY:
                    controllers[event.gdevice.which].state.analogL.Y = applyAnalogDeadZone_(event.gaxis.value);
                    if ( isSensorChanged_(event.gdevice.which, "analog", "l") ) {
                        controllers[event.gdevice.which].state.analogL.event_duration = event.gaxis.timestamp - controllers[event.gdevice.which].state.analogL.event_timestamp;
                        controllers[event.gdevice.which].state.analogL.event_timestamp = event.gaxis.timestamp;
                    }
                    break;
                case SDL_GAMEPAD_AXIS_RIGHTX:
                    controllers[event.gdevice.which].state.analogR.X = applyAnalogDeadZone_(event.gaxis.value);
                    if ( isSensorChanged_(event.gdevice.which, "analog", "r") ) {
                        controllers[event.gdevice.which].state.analogR.event_duration = event.gaxis.timestamp - controllers[event.gdevice.which].state.analogR.event_timestamp;
                        controllers[event.gdevice.which].state.analogR.event_timestamp = event.gaxis.timestamp;
                    }
                    break;
                case SDL_GAMEPAD_AXIS_RIGHTY:
                    controllers[event.gdevice.which].state.analogR.Y = applyAnalogDeadZone_(event.gaxis.value);
                    if ( isSensorChanged_(event.gdevice.which, "analog", "r") ) {
                        controllers[event.gdevice.which].state.analogR.event_duration = event.gaxis.timestamp - controllers[event.gdevice.which].state.analogR.event_timestamp;
                        controllers[event.gdevice.which].state.analogR.event_timestamp = event.gaxis.timestamp;
                    }
                    break;
                case SDL_GAMEPAD_AXIS_LEFT_TRIGGER:
                    controllers[event.gdevice.which].state.triggerL.value = event.gaxis.value;
                    if ( isSensorChanged_(event.gdevice.which, "trigger", "l") ) {
                        controllers[event.gdevice.which].state.triggerL.event_duration = event.gaxis.timestamp - controllers[event.gdevice.which].state.triggerL.event_timestamp;
                        controllers[event.gdevice.which].state.triggerL.event_timestamp = event.gaxis.timestamp;
                    }
                    break;
                case SDL_GAMEPAD_AXIS_RIGHT_TRIGGER:
                    controllers[event.gdevice.which].state.triggerR.value = event.gaxis.value;
                    if ( isSensorChanged_(event.gdevice.which, "trigger", "r") ) {
                        controllers[event.gdevice.which].state.triggerR.event_duration = event.gaxis.timestamp - controllers[event.gdevice.which].state.triggerR.event_timestamp;
                        controllers[event.gdevice.which].state.triggerR.event_timestamp = event.gaxis.timestamp;
                    }
                    break;
            }
            answer.eventAction = event.gaxis.axis;
            answer.eventName = SDL2Name["axis"][event.gaxis.axis];
            break;
        case SDL_EVENT_GAMEPAD_TOUCHPAD_DOWN: case SDL_EVENT_GAMEPAD_TOUCHPAD_MOTION: case SDL_EVENT_GAMEPAD_TOUCHPAD_UP:
            answer.eventAction = 0;
            answer.eventName = SDL2Name["touch"][answer.eventAction];
            controllers[event.gdevice.which].state.touch.action = event.gtouchpad.type;
            controllers[event.gdevice.which].state.touch.touchpad = event.gtouchpad.touchpad;
            controllers[event.gdevice.which].state.touch.finger = event.gtouchpad.finger;
            controllers[event.gdevice.which].state.touch.X = event.gtouchpad.x;
            controllers[event.gdevice.which].state.touch.Y = event.gtouchpad.y;
            controllers[event.gdevice.which].state.touch.pressure = event.gtouchpad.pressure;
            break;
        // case SDL_MULTIGESTURE:
        //     controllers[event.gdevice.which].state.multitouch.touchId = event.mgesture.touchId;
        //     controllers[event.gdevice.which].state.multitouch.dTheta = event.mgesture.dTheta;
        //     controllers[event.gdevice.which].state.multitouch.dDist = event.mgesture.dDist;
        //     controllers[event.gdevice.which].state.multitouch.X = event.mgesture.x;
        //     controllers[event.gdevice.which].state.multitouch.Y = event.mgesture.y;
        //     controllers[event.gdevice.which].state.multitouch.numFingers = event.mgesture.numFingers;
        //     break;
        case SDL_EVENT_GAMEPAD_SENSOR_UPDATE:
            if (event.gsensor.sensor == SDL_SENSOR_ACCEL) {
                controllers[event.gdevice.which].state.accel.X = event.gsensor.data[0];
                controllers[event.gdevice.which].state.accel.Y = event.gsensor.data[1];
                controllers[event.gdevice.which].state.accel.Z = event.gsensor.data[2];
            };
            if (event.gsensor.sensor == SDL_SENSOR_GYRO) {
                controllers[event.gdevice.which].state.gyro.X = event.gsensor.data[0];
                controllers[event.gdevice.which].state.gyro.Y = event.gsensor.data[1];
                controllers[event.gdevice.which].state.gyro.Z = event.gsensor.data[2];
            };
            answer.eventAction = event.gsensor.sensor;
            answer.eventName = SDL2Name["motion"][event.gsensor.sensor];
            break;
        default:
            answer.eventAction = -1;
            break;
    }
    return answer;
}

void PuaraController::printEvent(PuaraController::EventResume eventResume, bool printSensor) {
    switch (eventResume.eventType) {
        case SDL_EVENT_GAMEPAD_BUTTON_DOWN: case SDL_EVENT_GAMEPAD_BUTTON_UP:
            std::cout << "Event on controller " << eventResume.controller
                      << ": " << SDL2Name[SDL2Name["events"][eventResume.eventType]][eventResume.eventAction]
                      << " " << controllers[eventResume.controller].state.button[eventResume.eventAction].value
                      << " " << controllers[eventResume.controller].state.button[eventResume.eventAction].event_duration << std::endl;
            break;
        case SDL_EVENT_GAMEPAD_AXIS_MOTION:
            std::cout << "Event on controller " << eventResume.controller << ": ";
            switch (eventResume.eventAction) {
                case SDL_GAMEPAD_AXIS_LEFTX: case SDL_GAMEPAD_AXIS_LEFTY:
                    std::cout << " Analog_left " << controllers[eventResume.controller].state.analogL.X << " "
                              << controllers[eventResume.controller].state.analogL.Y
                              << " " << controllers[eventResume.controller].state.analogL.event_duration << std::endl;
                    break;
                case SDL_GAMEPAD_AXIS_RIGHTX: case SDL_GAMEPAD_AXIS_RIGHTY:
                    std::cout << " Analog_right " << controllers[eventResume.controller].state.analogR.X << " "
                              << controllers[eventResume.controller].state.analogR.Y
                              << " " << controllers[eventResume.controller].state.analogR.event_duration << std::endl;
                    break;
                case SDL_GAMEPAD_AXIS_LEFT_TRIGGER:
                    std::cout << " trigger_left " << controllers[eventResume.controller].state.triggerL.value
                              << " " << controllers[eventResume.controller].state.triggerL.event_duration << std::endl;
                    break;
                case SDL_GAMEPAD_AXIS_RIGHT_TRIGGER:
                    std::cout << " trigger_right " << controllers[eventResume.controller].state.triggerR.value
                              << " " << controllers[eventResume.controller].state.triggerR.event_duration << std::endl;
                    break;
            }
            break;
        case SDL_EVENT_GAMEPAD_TOUCHPAD_DOWN: case SDL_EVENT_GAMEPAD_TOUCHPAD_MOTION: case SDL_EVENT_GAMEPAD_TOUCHPAD_UP:
        std::cout << "Event on controller " << eventResume.controller << ": ";
            std::cout << " Touchpad " << controllers[eventResume.controller].state.touch.touchpad << ":"
                      << " ID: " << controllers[eventResume.controller].state.touch.finger
                      << "  X: " << controllers[eventResume.controller].state.touch.X
                      << "  Y: " << controllers[eventResume.controller].state.touch.Y
                    //   << " pr: " << controllers[eventResume.controller].state.touch.pressure
                      << std::endl;
            break;
        // case SDL_MULTIGESTURE:
        // std::cout << "Event on controller " << eventResume.controller << ": ";
        //     std::cout << " Multigesture " << controllers[eventResume.controller].state.touch.touchId << ":"
        //               << " ID: " << controllers[eventResume.controller].state.multitouch.touchId
        //               << "  X: " << controllers[eventResume.controller].state.multitouch.dTheta
        //               << "  Y: " << controllers[eventResume.controller].state.multitouch.dDist
        //               << " dX: " << controllers[eventResume.controller].state.multitouch.X
        //               << " dY: " << controllers[eventResume.controller].state.multitouch.Y
        //               << " pr: " << controllers[eventResume.controller].state.multitouch.numFingers
        //               << std::endl;
        //     break;
        case SDL_EVENT_GAMEPAD_SENSOR_UPDATE:
            if (printSensor) {
                std::cout << "Event on controller " << eventResume.controller
                          << ": " << SDL2Name[SDL2Name["events"][eventResume.eventType]][eventResume.eventAction];
                if (eventResume.eventAction == SDL_SENSOR_ACCEL) {
                    std::cout << " " << controllers[eventResume.controller].state.accel.X
                            << " " << controllers[eventResume.controller].state.accel.Y 
                            << " " << controllers[eventResume.controller].state.accel.Z << std::endl;
                } else if (eventResume.eventAction == SDL_SENSOR_GYRO) {
                    std::cout << " " << controllers[eventResume.controller].state.gyro.X
                            << " " << controllers[eventResume.controller].state.gyro.Y 
                            << " " << controllers[eventResume.controller].state.gyro.Z << std::endl;
                }
            }
            break;
    }
}

void PuaraController::printEvent(PuaraController::EventResume eventResume) {
    printEvent(eventResume, false);
}

bool PuaraController::doQuit(){
    return sdl_quit_; 
}

template<typename T>
PuaraController::CircularBuffer<T>::CircularBuffer(size_t capacity)
    : capacity_(capacity), size_(0), read_index_(0), write_index_(0) {
    buffer_.resize(capacity_);
}

template<typename T>
bool PuaraController::CircularBuffer<T>::isEmpty() const {
    return size_ == 0;
}

template<typename T>
bool PuaraController::CircularBuffer<T>::isFull() const {
    return size_ == capacity_;
}

template<typename T>
void PuaraController::CircularBuffer<T>::push(const T& item) {
    if ( isFull() ) pop();
    buffer_[write_index_] = item;
    write_index_ = (write_index_ + 1) % capacity_;
    size_++;
}

template<typename T>
T PuaraController::CircularBuffer<T>::pop() {
    if (isEmpty()) {
        std::cerr << "Circular buffer is empty. Unable to pop item." << std::endl;
        return T();
    }
    T item = buffer_[read_index_];
    read_index_ = (read_index_ + 1) % capacity_;
    size_--;
    return item;
}

PuaraController::Controller::Controller(int id, SDL_Gamepad* instance, int move_buffer_size) 
    : id(id), instance(instance), buffer(move_buffer_size) {
        for (auto const& i: PuaraController::SDL2Name["button"]) {
            Button btnInstance;
            state.button.insert({i.first, btnInstance});
        }
        isOpen = true;
}

bool PuaraController::isSensorChanged_(int joy_index, std::string sensor, std::string side) {
    bool convertedValue = false;
    bool answer;
    if (sensor == "trigger") {
        if (side == "l") {
            if (controllers[joy_index].state.triggerL.value != 0) {
                convertedValue = true;
            }
            answer = convertedValue != controllers[joy_index].state.triggerL.state;
            if (answer)
                controllers[joy_index].state.triggerL.state = !controllers[joy_index].state.triggerL.state;
        } else if (side == "r") {
            if (controllers[joy_index].state.triggerR.value != 0) {
                convertedValue = true;
            }
            answer = convertedValue != controllers[joy_index].state.triggerR.state;
            if (answer)
                controllers[joy_index].state.triggerR.state = !controllers[joy_index].state.triggerR.state;
        }
    } else if (sensor == "analog") {
        if (side == "l") {
            if (controllers[joy_index].state.analogL.X != 0 || controllers[joy_index].state.analogL.Y !=0) {
                convertedValue = true;
            }
            answer = convertedValue != controllers[joy_index].state.analogL.state;
            if (answer)
                controllers[joy_index].state.analogL.state = !controllers[joy_index].state.analogL.state;
        } else if (side == "r") {
            if (controllers[joy_index].state.analogR.X != 0 || controllers[joy_index].state.analogR.Y !=0) {
                convertedValue = true;
            }
            answer = convertedValue != controllers[joy_index].state.analogR.state;
            if (answer)
                controllers[joy_index].state.analogR.state = !controllers[joy_index].state.analogR.state;
        }
    } 
    return answer; 
}

double PuaraController::mapRange(double in, double inMin, double inMax, double outMin, double outMax) {
    return (in - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}

int PuaraController::mapRange(int in, int inMin, int inMax, float outMin, float outMax) {
    return round((in - inMin) * (outMax - outMin) / (inMax - inMin) + outMin);
}

float PuaraController::mapRange(float in, float inMin, float inMax, float outMin, float outMax) {
    return (in - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}