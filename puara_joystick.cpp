//****************************************************************************//
// Puara Joystick module - connect with game controllers using SDL2 (cpp)     //
//                         Controller -> OSC/MIDI bridge                      //
// https://github.com/Puara/puara-joystick                                    //
// Metalab - Société des Arts Technologiques (SAT)                            //
// Input Devices and Music Interaction Laboratory (IDMIL), McGill University  //
// Edu Meneses (2023) - https://www.edumeneses.com                            //
//****************************************************************************//

#include "puara_joystick.hpp"

std::unordered_map<std::string, std::unordered_map<int, std::string>> PuaraJoystick::SDL2Name = {
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
        {SDL_CONTROLLER_BUTTON_TOUCHPAD,"touchpad"},
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

PuaraJoystick::PuaraJoystick() {
    std::cout << "Starting Puara Joystick..." << std::endl;
    if (SDL_Init(SDL_INIT_GAMECONTROLLER | SDL_INIT_SENSOR) < 0) {
        std::cerr << "Could not initialize sdl2: " << SDL_GetError() << std::endl;
        exit(EXIT_FAILURE);
    } else {if (verbose) std::cout << "SDL2 initialized successfully" << std::endl;}
    std::cout << "Puara Joystick started successfully" << std::endl;
};

int PuaraJoystick::rumble(int controllerID, int time=1000, float lowFreq=1.0, float hiFreq=1.0f) {
    int rangedControllerID = clip(controllerID, 0, (controllers.size()-1));
    int rangedTime = clip(time, 10, 10000);
    float rangedLowFreq = clip(lowFreq, 0.0f, 1.0f) * 65535;
    float rangedHiFreq = clip(hiFreq, 0.0f, 1.0f) * 65535;
    SDL_GameControllerRumble(controllers[rangedControllerID].instance, rangedLowFreq, rangedHiFreq, time);
    if (verbose) std::cout << "Controller " << rangedControllerID << " rumble!" << std::endl;
    return 0;
}

int PuaraJoystick::openController(int joy_index) {
    if (SDL_IsGameController(joy_index)) {
        std::cout << "New game controller found. Opening...\n" << std::endl;
        Controller controller_instance(joy_index, SDL_GameControllerOpen(joy_index), move_buffer_size);
        controllers.insert({joy_index, controller_instance});
        if (controllers[joy_index].isOpen) {
            std::cout << "\nController \""<< SDL_GameControllerNameForIndex(joy_index) 
                      << "\" (" << joy_index << ") " << "opened successfully\n"
                      << "Controller type: " << SDL_GameControllerTypeForIndex(joy_index) << std::endl;
        }
        if (SDL_GameControllerSetSensorEnabled(controllers[joy_index].instance, SDL_SENSOR_GYRO, SDL_TRUE) < 0)
            if (verbose) std::cout << "Could not enable the gyroscope for this controller" << std::endl;
        if (SDL_GameControllerSetSensorEnabled(controllers[joy_index].instance, SDL_SENSOR_ACCEL, SDL_TRUE) < 0)
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
        return 0;
    } else {
        std::cerr << "Error: the controller is not supported by the game controller interface" << std::endl;
        return 1;
    } 
}

int PuaraJoystick::openAllControllers() {
    if (SDL_NumJoysticks() == 0) {
        std::cerr << "Warning: could not find any controller" << std::endl;
        return 1;
    }
    for (int i = 0; i < SDL_NumJoysticks(); i++)
        openController(i);
    return 0;
}

float PuaraJoystick::clip(float n, float lower, float upper) {
  return std::max(lower, std::min(n, upper));
}

int PuaraJoystick::clip(int n, int lower, int upper) {
  return std::max(lower, std::min(n, upper));
}

PuaraJoystick::EventResume PuaraJoystick::pullSDLEvent(SDL_Event event){
    EventResume answer = {
        .controller = event.cdevice.which,
        .eventType = event.type,
        .eventAction = 0
    };
    if (event.type == SDL_QUIT) {
        sdl_quit = true;
    } else if (event.type == SDL_CONTROLLERDEVICEADDED) {
        openController(event.cdevice.which);
    }
    switch (event.type) {
        case SDL_CONTROLLERDEVICEREMOVED:
            for (auto controller : controllers) {
                SDL_GameControllerClose(controllers[event.cdevice.which].instance);
                controllers.erase(event.cdevice.which);
                if (verbose) std::cout << "Controller " << event.cdevice.which << "vanished!" << std::endl;
            }
            break;
        case SDL_CONTROLLERBUTTONDOWN: case SDL_CONTROLLERBUTTONUP:
            // elapsed_time = event.cbutton.timestamp - last_button_event_time[event.cbutton.button];
            // last_button_event_time[event.cbutton.button] = event.cbutton.timestamp;
            controllers[event.cdevice.which].state.button[event.cbutton.button] = event.cbutton.state;
            answer.eventAction = event.cbutton.button;
            break;
        case SDL_CONTROLLERAXISMOTION:
            switch (event.caxis.axis) {
                case SDL_CONTROLLER_AXIS_LEFTX:
                    controllers[event.cdevice.which].state.analogL.X = event.caxis.value;
                    break;
                case SDL_CONTROLLER_AXIS_LEFTY:
                    controllers[event.cdevice.which].state.analogL.Y = event.caxis.value;
                    break;
                case SDL_CONTROLLER_AXIS_RIGHTX:
                    controllers[event.cdevice.which].state.analogR.X = event.caxis.value;
                    break;
                case SDL_CONTROLLER_AXIS_RIGHTY:
                    controllers[event.cdevice.which].state.analogR.Y = event.caxis.value;
                    break;
                case SDL_CONTROLLER_AXIS_TRIGGERLEFT:
                    controllers[event.cdevice.which].state.triggerL = event.caxis.value;
                    break;
                case SDL_CONTROLLER_AXIS_TRIGGERRIGHT:
                    controllers[event.cdevice.which].state.triggerR = event.caxis.value;
                    break;
            }
            answer.eventAction = event.caxis.axis;
            break;
        case SDL_CONTROLLERSENSORUPDATE:
            if (event.csensor.sensor == SDL_SENSOR_ACCEL) {
                controllers[event.cdevice.which].state.accel.X = event.csensor.data[0];
                controllers[event.cdevice.which].state.accel.Y = event.csensor.data[1];
                controllers[event.cdevice.which].state.accel.Z = event.csensor.data[2];
            };
            if (event.csensor.sensor == SDL_SENSOR_GYRO) {
                controllers[event.cdevice.which].state.gyro.X = event.csensor.data[0];
                controllers[event.cdevice.which].state.gyro.Y = event.csensor.data[1];
                controllers[event.cdevice.which].state.gyro.Z = event.csensor.data[2];
            };
            answer.eventAction = event.csensor.sensor;
            break;
    }
    return answer;
}

void PuaraJoystick::printEvent(PuaraJoystick::EventResume eventResume, bool printSensor) {
    switch (eventResume.eventType) {
        case SDL_CONTROLLERBUTTONDOWN: case SDL_CONTROLLERBUTTONUP:
            std::cout << "Event on controller " << eventResume.controller
                      << ": " << SDL2Name[SDL2Name["events"][eventResume.eventType]][eventResume.eventAction]
                      << " " << controllers[eventResume.controller].state.button[eventResume.eventAction] << std::endl;
            break;
        case SDL_CONTROLLERAXISMOTION:
            std::cout << "Event on controller " << eventResume.controller
                      << ": " << SDL2Name[SDL2Name["events"][eventResume.eventType]][eventResume.eventAction];
            switch (eventResume.eventAction) {
                case SDL_CONTROLLER_AXIS_LEFTX: case SDL_CONTROLLER_AXIS_LEFTY:
                    std::cout << " " << controllers[eventResume.controller].state.analogL.X << " "
                              << controllers[eventResume.controller].state.analogL.Y << std::endl;
                    break;
                case SDL_CONTROLLER_AXIS_RIGHTX: case SDL_CONTROLLER_AXIS_RIGHTY:
                    std::cout << " " << controllers[eventResume.controller].state.analogR.X << " "
                              << controllers[eventResume.controller].state.analogR.Y << std::endl;
                    break;
                case SDL_CONTROLLER_AXIS_TRIGGERLEFT:
                    std::cout << " " << controllers[eventResume.controller].state.triggerL << std::endl;
                    break;
                case SDL_CONTROLLER_AXIS_TRIGGERRIGHT:
                    std::cout << " " << controllers[eventResume.controller].state.triggerR << std::endl;
                    break;
            }
            break;
        case SDL_CONTROLLERSENSORUPDATE:
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

void PuaraJoystick::printEvent(PuaraJoystick::EventResume eventResume) {
    printEvent(eventResume, false);
}

bool PuaraJoystick::doQuit(){
    return sdl_quit; 
}

template<typename T>
PuaraJoystick::CircularBuffer<T>::CircularBuffer(size_t capacity)
    : capacity_(capacity), size_(0), read_index_(0), write_index_(0) {
    buffer_.resize(capacity_);
}

template<typename T>
bool PuaraJoystick::CircularBuffer<T>::isEmpty() const {
    return size_ == 0;
}

template<typename T>
bool PuaraJoystick::CircularBuffer<T>::isFull() const {
    return size_ == capacity_;
}

template<typename T>
void PuaraJoystick::CircularBuffer<T>::push(const T& item) {
    if ( isFull() ) pop();
    buffer_[write_index_] = item;
    write_index_ = (write_index_ + 1) % capacity_;
    size_++;
}

template<typename T>
T PuaraJoystick::CircularBuffer<T>::pop() {
    if (isEmpty()) {
        std::cerr << "Circular buffer is empty. Unable to pop item." << std::endl;
        return T();
    }
    T item = buffer_[read_index_];
    read_index_ = (read_index_ + 1) % capacity_;
    size_--;
    return item;
}

PuaraJoystick::Controller::Controller(int id, SDL_GameController* instance, int move_buffer_size) 
    : id(id), instance(instance), buffer(move_buffer_size) {
        for (auto const& i: PuaraJoystick::SDL2Name["button"]) {
            state.button.insert({i.first,0});
            state.last_button_event_duration.insert({i.first,0});
        }
        for (auto const& i: PuaraJoystick::SDL2Name["axis"])
            state.axis.insert({i.first,0});
        isOpen = true;
}
