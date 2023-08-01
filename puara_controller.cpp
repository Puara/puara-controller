//****************************************************************************//
// Puara Joystick module - connect with game controllers using SDL2 (cpp)     //
//                         Controller -> OSC/MIDI bridge                      //
// https://github.com/Puara/puara-joystick                                    //
// Metalab - Société des Arts Technologiques (SAT)                            //
// Input Devices and Music Interaction Laboratory (IDMIL), McGill University  //
// Edu Meneses (2023) - https://www.edumeneses.com                            //
//****************************************************************************//

#include "puara_controller.hpp"

std::unordered_map<std::string, std::unordered_map<int, std::string>> PuaraController::SDL2Name_ = {
    {"events",{ /* Supported Puara Controller SDL events */
        {SDL_CONTROLLERDEVICEADDED,"added"},
        {SDL_CONTROLLERDEVICEREMOVED,"removed"},
        {SDL_CONTROLLERBUTTONDOWN,"button"},
        {SDL_CONTROLLERBUTTONUP,"button"},
        {SDL_CONTROLLERAXISMOTION,"axis"},
        {SDL_CONTROLLERSENSORUPDATE,"sensor"}
    }},
    {"button",{ /* This list is generated from SDL_GameControllerButton */
        {SDL_CONTROLLER_BUTTON_INVALID,"invalid"},
        {SDL_CONTROLLER_BUTTON_A,"A"},
        {SDL_CONTROLLER_BUTTON_B,"B"},
        {SDL_CONTROLLER_BUTTON_X,"X"},
        {SDL_CONTROLLER_BUTTON_Y,"Y"},
        {SDL_CONTROLLER_BUTTON_BACK,"back"},
        {SDL_CONTROLLER_BUTTON_GUIDE,"guide"},
        {SDL_CONTROLLER_BUTTON_START,"start"},
        {SDL_CONTROLLER_BUTTON_LEFTSTICK,"leftstick"},
        {SDL_CONTROLLER_BUTTON_RIGHTSTICK,"rightstick"},
        {SDL_CONTROLLER_BUTTON_LEFTSHOULDER,"leftshoulder"},
        {SDL_CONTROLLER_BUTTON_RIGHTSHOULDER,"rightshoulder"},
        {SDL_CONTROLLER_BUTTON_DPAD_UP,"dpad_up"},
        {SDL_CONTROLLER_BUTTON_DPAD_DOWN,"dpad_down"},
        {SDL_CONTROLLER_BUTTON_DPAD_LEFT,"dpad_left"},
        {SDL_CONTROLLER_BUTTON_DPAD_RIGHT,"dpad_right"},
        {SDL_CONTROLLER_BUTTON_MISC1, "misc1"},
        {SDL_CONTROLLER_BUTTON_PADDLE1,"paddle1"},
        {SDL_CONTROLLER_BUTTON_PADDLE2,"paddle2"},
        {SDL_CONTROLLER_BUTTON_PADDLE3,"paddle3"},
        {SDL_CONTROLLER_BUTTON_PADDLE4,"paddle4"},
        {SDL_CONTROLLER_BUTTON_TOUCHPAD,"touchpadbutton"},
        {SDL_CONTROLLER_BUTTON_MAX,"max_btn"}
    }},
    {"axis",{ /* This list is generated from SDL_GameControllerAxis */
        {SDL_CONTROLLER_AXIS_INVALID,"invalid"},
        {SDL_CONTROLLER_AXIS_LEFTX,"leftx"},
        {SDL_CONTROLLER_AXIS_LEFTY,"lefty"},
        {SDL_CONTROLLER_AXIS_RIGHTX,"rightx"},
        {SDL_CONTROLLER_AXIS_RIGHTY,"righty"},
        {SDL_CONTROLLER_AXIS_TRIGGERLEFT,"triggerleft"},
        {SDL_CONTROLLER_AXIS_TRIGGERRIGHT,"triggerright"},
        {SDL_CONTROLLER_AXIS_MAX,"max_axis"}
    }},
    {"sensor",{ /* This list is generated from SDL_SensorType */
        {SDL_SENSOR_INVALID,"invalid"},
        {SDL_SENSOR_UNKNOWN,"unknown"},
        {SDL_SENSOR_ACCEL,"accel"},
        {SDL_SENSOR_GYRO,"gyro"}
    }},
};

PuaraController::PuaraController() {
    std::cout << "Starting Puara Joystick..." << std::endl;
    if (SDL_Init(SDL_INIT_GAMECONTROLLER | SDL_INIT_SENSOR) < 0) {
        std::cerr << "Could not initialize sdl2: " << SDL_GetError() << std::endl;
        exit(EXIT_FAILURE);
    } else {if (verbose) std::cout << "SDL2 initialized successfully" << std::endl;}
    std::cout << "Puara Joystick started successfully" << std::endl;
};

int PuaraController::rumble(int controllerID, int time=1000, float lowFreq=1.0, float hiFreq=1.0f) {
    int rangedControllerID = clip_(controllerID, 0, (controllers_.size()-1));
    int rangedTime = clip_(time, 10, 10000);
    float rangedLowFreq = clip_(lowFreq, 0.0f, 1.0f) * 65535;
    float rangedHiFreq = clip_(hiFreq, 0.0f, 1.0f) * 65535;
    SDL_GameControllerRumble(controllers_[rangedControllerID].instance, rangedLowFreq, rangedHiFreq, time);
    if (verbose) std::cout << "Controller " << rangedControllerID << " rumble!" << std::endl;
    return 0;
}

int PuaraController::openController(int joy_index) {
    if (SDL_IsGameController(joy_index)) {
        std::cout << "New game controller found. Opening...\n" << std::endl;
        Controller controller_instance(joy_index, SDL_GameControllerOpen(joy_index), move_buffer_size);
        controllers_.insert({joy_index, controller_instance});
        if (controllers_[joy_index].isOpen) {
            std::cout << "\nController \""<< SDL_GameControllerNameForIndex(joy_index) 
                      << "\" (" << joy_index << ") " << "opened successfully" << std::endl;
        }
        if (SDL_GameControllerSetSensorEnabled(controllers_[joy_index].instance, SDL_SENSOR_GYRO, SDL_TRUE) < 0)
            if (verbose) std::cout << "Could not enable the gyroscope for this controller" << std::endl;
        if (SDL_GameControllerSetSensorEnabled(controllers_[joy_index].instance, SDL_SENSOR_ACCEL, SDL_TRUE) < 0)
            if (verbose) std::cout << "Could not enable the acclelerometer for this controller" << std::endl;
        switch (SDL_GameControllerTypeForIndex(joy_index)){
            case SDL_CONTROLLER_TYPE_XBOX360: case SDL_CONTROLLER_TYPE_XBOXONE:
                SDL_SetHint(SDL_HINT_JOYSTICK_HIDAPI_XBOX, "1");
                break;
            case SDL_CONTROLLER_TYPE_PS4:
                SDL_SetHint(SDL_HINT_JOYSTICK_HIDAPI_PS4_RUMBLE, "1");
                break;
            case SDL_CONTROLLER_TYPE_PS5:
                SDL_SetHint(SDL_HINT_JOYSTICK_HIDAPI_PS5_RUMBLE, "1");
                break;
        }
        std::cout << "This controleer has "
                  << SDL_GameControllerGetNumTouchpads(controllers_[joy_index].instance)
                  << " touchpads and can use up to "
                  << SDL_GameControllerGetNumTouchpadFingers(controllers_[joy_index].instance, 0)
                  << " fingers simultaneusly" << std::endl;
        return 0;
    } else {
        std::cerr << "Error: the controller is not supported by the game controller interface" << std::endl;
        return 1;
    } 
}

int PuaraController::openAllControllers() {
    if (SDL_NumJoysticks() == 0) {
        std::cerr << "Warning: could not find any controller" << std::endl;
        return 1;
    }
    for (int i = 0; i < SDL_NumJoysticks(); i++)
        openController(i);
    return 0;
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
        .controller = event.cdevice.which,
        .eventType = event.type,
        .eventAction = 0
    };
    if (event.type == SDL_QUIT) {
        sdl_quit_ = true;
    } else if (event.type == SDL_CONTROLLERDEVICEADDED) {
        openController(event.cdevice.which);
    }
    switch (event.type) {
        case SDL_CONTROLLERDEVICEREMOVED:
                SDL_GameControllerClose(controllers_[event.cdevice.which].instance);
                controllers_.erase(event.cdevice.which);
                if (verbose) std::cout << "Controller " << event.cdevice.which << " vanished!" << std::endl;
            break;
        case SDL_CONTROLLERBUTTONDOWN: case SDL_CONTROLLERBUTTONUP:
            controllers_[event.cdevice.which].state.button[event.cbutton.button].value = event.cbutton.state;
            answer.eventAction = event.cbutton.button;
            controllers_[event.cdevice.which].state.button[event.cbutton.button].event_duration = event.cbutton.timestamp - controllers_[event.cdevice.which].state.button[event.cbutton.button].event_timestamp;
            controllers_[event.cdevice.which].state.button[event.cbutton.button].event_timestamp = event.cbutton.timestamp;
            break;
        case SDL_CONTROLLERAXISMOTION:
            switch (event.caxis.axis) {
                case SDL_CONTROLLER_AXIS_LEFTX:
                    controllers_[event.cdevice.which].state.analogL.X = applyAnalogDeadZone_(event.caxis.value);
                    if ( isSensorChanged_(event.cdevice.which, "analog", "l") ) {
                        controllers_[event.cdevice.which].state.analogL.event_duration = event.caxis.timestamp - controllers_[event.cdevice.which].state.analogL.event_timestamp;
                        controllers_[event.cdevice.which].state.analogL.event_timestamp = event.caxis.timestamp;
                    }
                    break;
                case SDL_CONTROLLER_AXIS_LEFTY:
                    controllers_[event.cdevice.which].state.analogL.Y = applyAnalogDeadZone_(event.caxis.value);
                    if ( isSensorChanged_(event.cdevice.which, "analog", "l") ) {
                        controllers_[event.cdevice.which].state.analogL.event_duration = event.caxis.timestamp - controllers_[event.cdevice.which].state.analogL.event_timestamp;
                        controllers_[event.cdevice.which].state.analogL.event_timestamp = event.caxis.timestamp;
                    }
                    break;
                case SDL_CONTROLLER_AXIS_RIGHTX:
                    controllers_[event.cdevice.which].state.analogR.X = applyAnalogDeadZone_(event.caxis.value);
                    if ( isSensorChanged_(event.cdevice.which, "analog", "r") ) {
                        controllers_[event.cdevice.which].state.analogR.event_duration = event.caxis.timestamp - controllers_[event.cdevice.which].state.analogR.event_timestamp;
                        controllers_[event.cdevice.which].state.analogR.event_timestamp = event.caxis.timestamp;
                    }
                    break;
                case SDL_CONTROLLER_AXIS_RIGHTY:
                    controllers_[event.cdevice.which].state.analogR.Y = applyAnalogDeadZone_(event.caxis.value);
                    if ( isSensorChanged_(event.cdevice.which, "analog", "r") ) {
                        controllers_[event.cdevice.which].state.analogR.event_duration = event.caxis.timestamp - controllers_[event.cdevice.which].state.analogR.event_timestamp;
                        controllers_[event.cdevice.which].state.analogR.event_timestamp = event.caxis.timestamp;
                    }
                    break;
                case SDL_CONTROLLER_AXIS_TRIGGERLEFT:
                    controllers_[event.cdevice.which].state.triggerL.value = event.caxis.value;
                    if ( isSensorChanged_(event.cdevice.which, "trigger", "l") ) {
                        controllers_[event.cdevice.which].state.triggerL.event_duration = event.caxis.timestamp - controllers_[event.cdevice.which].state.triggerL.event_timestamp;
                        controllers_[event.cdevice.which].state.triggerL.event_timestamp = event.caxis.timestamp;
                    }
                    break;
                case SDL_CONTROLLER_AXIS_TRIGGERRIGHT:
                    controllers_[event.cdevice.which].state.triggerR.value = event.caxis.value;
                    if ( isSensorChanged_(event.cdevice.which, "trigger", "r") ) {
                        controllers_[event.cdevice.which].state.triggerR.event_duration = event.caxis.timestamp - controllers_[event.cdevice.which].state.triggerR.event_timestamp;
                        controllers_[event.cdevice.which].state.triggerR.event_timestamp = event.caxis.timestamp;
                    }
                    break;
            }
            answer.eventAction = event.caxis.axis;
            break;
        case SDL_CONTROLLERTOUCHPADDOWN: case SDL_CONTROLLERTOUCHPADMOTION: case SDL_CONTROLLERTOUCHPADUP:
            controllers_[event.cdevice.which].state.touch.action = event.type;
            controllers_[event.cdevice.which].state.touch.touchId = event.tfinger.touchId;
            controllers_[event.cdevice.which].state.touch.fingerId = event.tfinger.fingerId;
            controllers_[event.cdevice.which].state.touch.X = event.tfinger.x;
            controllers_[event.cdevice.which].state.touch.Y = event.tfinger.y;
            controllers_[event.cdevice.which].state.touch.dX = event.tfinger.dx;
            controllers_[event.cdevice.which].state.touch.dY = event.tfinger.dy;
            controllers_[event.cdevice.which].state.touch.pressure = event.tfinger.pressure;
            break;
        case SDL_MULTIGESTURE:
            controllers_[event.cdevice.which].state.multitouch.touchId = event.mgesture.touchId;
            controllers_[event.cdevice.which].state.multitouch.dTheta = event.mgesture.dTheta;
            controllers_[event.cdevice.which].state.multitouch.dDist = event.mgesture.dDist;
            controllers_[event.cdevice.which].state.multitouch.X = event.mgesture.x;
            controllers_[event.cdevice.which].state.multitouch.Y = event.mgesture.y;
            controllers_[event.cdevice.which].state.multitouch.numFingers = event.mgesture.numFingers;
            break;
        case SDL_CONTROLLERSENSORUPDATE:
            if (event.csensor.sensor == SDL_SENSOR_ACCEL) {
                controllers_[event.cdevice.which].state.accel.X = event.csensor.data[0];
                controllers_[event.cdevice.which].state.accel.Y = event.csensor.data[1];
                controllers_[event.cdevice.which].state.accel.Z = event.csensor.data[2];
            };
            if (event.csensor.sensor == SDL_SENSOR_GYRO) {
                controllers_[event.cdevice.which].state.gyro.X = event.csensor.data[0];
                controllers_[event.cdevice.which].state.gyro.Y = event.csensor.data[1];
                controllers_[event.cdevice.which].state.gyro.Z = event.csensor.data[2];
            };
            answer.eventAction = event.csensor.sensor;
            break;
    }
    return answer;
}

void PuaraController::printEvent(PuaraController::EventResume eventResume, bool printSensor) {
    switch (eventResume.eventType) {
        case SDL_CONTROLLERBUTTONDOWN: case SDL_CONTROLLERBUTTONUP:
            std::cout << "Event on controller " << eventResume.controller
                      << ": " << SDL2Name_[SDL2Name_["events"][eventResume.eventType]][eventResume.eventAction]
                      << " " << controllers_[eventResume.controller].state.button[eventResume.eventAction].value
                      << " " << controllers_[eventResume.controller].state.button[eventResume.eventAction].event_duration << std::endl;
            break;
        case SDL_CONTROLLERAXISMOTION:
            std::cout << "Event on controller " << eventResume.controller << ": ";
            switch (eventResume.eventAction) {
                case SDL_CONTROLLER_AXIS_LEFTX: case SDL_CONTROLLER_AXIS_LEFTY:
                    std::cout << " Analog_left " << controllers_[eventResume.controller].state.analogL.X << " "
                              << controllers_[eventResume.controller].state.analogL.Y
                              << " " << controllers_[eventResume.controller].state.analogL.event_duration << std::endl;
                    break;
                case SDL_CONTROLLER_AXIS_RIGHTX: case SDL_CONTROLLER_AXIS_RIGHTY:
                    std::cout << " Analog_right " << controllers_[eventResume.controller].state.analogR.X << " "
                              << controllers_[eventResume.controller].state.analogR.Y
                              << " " << controllers_[eventResume.controller].state.analogR.event_duration << std::endl;
                    break;
                case SDL_CONTROLLER_AXIS_TRIGGERLEFT:
                    std::cout << " trigger_left " << controllers_[eventResume.controller].state.triggerL.value
                              << " " << controllers_[eventResume.controller].state.triggerL.event_duration << std::endl;
                    break;
                case SDL_CONTROLLER_AXIS_TRIGGERRIGHT:
                    std::cout << " trigger_right " << controllers_[eventResume.controller].state.triggerR.value
                              << " " << controllers_[eventResume.controller].state.triggerR.event_duration << std::endl;
                    break;
            }
            break;
        case SDL_CONTROLLERTOUCHPADDOWN: case SDL_CONTROLLERTOUCHPADMOTION: case SDL_CONTROLLERTOUCHPADUP:
        std::cout << "Event on controller " << eventResume.controller << ": ";
            std::cout << " Touchpad " << controllers_[eventResume.controller].state.touch.touchId << ":"
                      << " ID: " << controllers_[eventResume.controller].state.touch.fingerId
                      << "  X: " << controllers_[eventResume.controller].state.touch.X
                      << "  Y: " << controllers_[eventResume.controller].state.touch.Y
                      << " dX: " << controllers_[eventResume.controller].state.touch.dX
                      << " dY: " << controllers_[eventResume.controller].state.touch.dY
                      << " pr: " << controllers_[eventResume.controller].state.touch.pressure
                      << std::endl;
            break;
        case SDL_MULTIGESTURE:
        std::cout << "Event on controller " << eventResume.controller << ": ";
            std::cout << " Multigesture " << controllers_[eventResume.controller].state.touch.touchId << ":"
                      << " ID: " << controllers_[eventResume.controller].state.multitouch.touchId
                      << "  X: " << controllers_[eventResume.controller].state.multitouch.dTheta
                      << "  Y: " << controllers_[eventResume.controller].state.multitouch.dDist
                      << " dX: " << controllers_[eventResume.controller].state.multitouch.X
                      << " dY: " << controllers_[eventResume.controller].state.multitouch.Y
                      << " pr: " << controllers_[eventResume.controller].state.multitouch.numFingers
                      << std::endl;
            break;
        case SDL_CONTROLLERSENSORUPDATE:
            if (printSensor) {
                std::cout << "Event on controller " << eventResume.controller
                          << ": " << SDL2Name_[SDL2Name_["events"][eventResume.eventType]][eventResume.eventAction];
                if (eventResume.eventAction == SDL_SENSOR_ACCEL) {
                    std::cout << " " << controllers_[eventResume.controller].state.accel.X
                            << " " << controllers_[eventResume.controller].state.accel.Y 
                            << " " << controllers_[eventResume.controller].state.accel.Z << std::endl;
                } else if (eventResume.eventAction == SDL_SENSOR_GYRO) {
                    std::cout << " " << controllers_[eventResume.controller].state.gyro.X
                            << " " << controllers_[eventResume.controller].state.gyro.Y 
                            << " " << controllers_[eventResume.controller].state.gyro.Z << std::endl;
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

PuaraController::Controller::Controller(int id, SDL_GameController* instance, int move_buffer_size) 
    : id(id), instance(instance), buffer(move_buffer_size) {
        for (auto const& i: PuaraController::SDL2Name_["button"]) {
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
            if (controllers_[joy_index].state.triggerL.value != 0) {
                convertedValue = true;
            }
            answer = convertedValue != controllers_[joy_index].state.triggerL.state;
            if (answer)
                controllers_[joy_index].state.triggerL.state = !controllers_[joy_index].state.triggerL.state;
        } else if (side == "r") {
            if (controllers_[joy_index].state.triggerR.value != 0) {
                convertedValue = true;
            }
            answer = convertedValue != controllers_[joy_index].state.triggerR.state;
            if (answer)
                controllers_[joy_index].state.triggerR.state = !controllers_[joy_index].state.triggerR.state;
        }
    } else if (sensor == "analog") {
        if (side == "l") {
            if (controllers_[joy_index].state.analogL.X != 0 || controllers_[joy_index].state.analogL.Y !=0) {
                convertedValue = true;
            }
            answer = convertedValue != controllers_[joy_index].state.analogL.state;
            if (answer)
                controllers_[joy_index].state.analogL.state = !controllers_[joy_index].state.analogL.state;
        } else if (side == "r") {
            if (controllers_[joy_index].state.analogR.X != 0 || controllers_[joy_index].state.analogR.Y !=0) {
                convertedValue = true;
            }
            answer = convertedValue != controllers_[joy_index].state.analogR.state;
            if (answer)
                controllers_[joy_index].state.analogR.state = !controllers_[joy_index].state.analogR.state;
        }
    } 
    return answer; 
}
