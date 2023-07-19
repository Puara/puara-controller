//****************************************************************************//
// Puara Joystick module - connect with game controllers using SDL2 (cpp)     //
//                         Controller -> OSC/MIDI bridge                      //
// https://github.com/Puara/puara-joystick                                    //
// Metalab - Société des Arts Technologiques (SAT)                            //
// Input Devices and Music Interaction Laboratory (IDMIL), McGill University  //
// Edu Meneses (2023) - https://www.edumeneses.com                            //
//****************************************************************************//

#include "puara_joystick.hpp"


PuaraJoystick::PuaraJoystick(...) : osc_server(osc_server_port) {
    std::cout << "Starting Puara Joystick..." << std::endl;
    if (SDL_Init(SDL_INIT_GAMECONTROLLER | SDL_INIT_SENSOR) < 0) {
        std::cerr << "Could not initialize sdl2: " << SDL_GetError() << std::endl;
        exit(EXIT_FAILURE);
    } else {if (verbose) std::cout << "SDL2 initialized successfully" << std::endl;}
    full_namespace = OSC_namespace + "/rumble";
    libloServerMethods();
    osc_server.start();
    std::cout << "Puara Joystick started successfully" << std::endl;
};

void PuaraJoystick::libloServerMethods(){
    osc_server.add_method(full_namespace, "iiff",
        [&](lo_arg **argv, int) {
            int id = clip(argv[0]->i, 0, (controllers_temp.size()-1));
            int time = clip(argv[1]->i, 10, 10000);
            float low_freq = clip(argv[2]->f, 0.0f, 1.0f) * 65535;
            float hi_freq = clip(argv[3]->f, 0.0f, 1.0f) * 65535;
            SDL_GameControllerRumble(controllers_temp[id].instance, low_freq, hi_freq, time);
            if (verbose) std::cout << "Controller " << id << " rumble!" << std::endl;
        });
    osc_server.add_method(full_namespace, "i",
        [&](lo_arg **argv, int) {
            int id = clip(argv[0]->i, 0, (controllers_temp.size()-1));
            SDL_GameControllerRumble(controllers_temp[id].instance, 65535, 65535, 1000);
            if (verbose) std::cout << "Controller " << id << " rumble!" << std::endl;
        });
}

int PuaraJoystick::openController(int joy_index) {
    if (SDL_IsGameController(joy_index)) {
        std::cout << "New game controller found. Opening...\n" << std::endl;
        //controllers_temp.insert({joy_index, SDL_GameControllerOpen(joy_index)});
        Controller controller_instance(move_buffer_size);
        controllers_temp.insert({joy_index, controller_instance});
        controllers_temp[joy_index].id = joy_index;
        controllers_temp[joy_index].instance = SDL_GameControllerOpen(joy_index);
        std::cout << "\nController \""<< SDL_GameControllerNameForIndex(joy_index) 
                    << "\" (" << joy_index << ") " << "opened successfully" << std::endl;
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
        if (SDL_GameControllerSetSensorEnabled(controllers_temp[joy_index].instance, SDL_SENSOR_GYRO, SDL_TRUE) < 0)
            if (verbose) std::cout << "Could not enable the gyroscope for this controller" << std::endl;
        if (SDL_GameControllerSetSensorEnabled(controllers_temp[joy_index].instance, SDL_SENSOR_ACCEL, SDL_TRUE) < 0)
            if (verbose) std::cout << "Could not enable the acclelerometer for this controller" << std::endl;
        return 0;
    } else {
        std::cerr << "Error: the controller is not supported by the game controller interface" << std::endl;
        return 1;
    } 
}

// int PuaraJoystick::openController(std::vector<SDL_GameController*> &controller_container, int joy_index) {
//     if (SDL_IsGameController(joy_index)) {
//         std::cout << "New game controller found. Opening...\n" << std::endl;
//         controller_container.push_back(SDL_GameControllerOpen(joy_index));
//         std::cout << "\nController \""<< SDL_GameControllerNameForIndex(joy_index) 
//                     << "\" (" << joy_index << ") " << "opened successfully" << std::endl;
//         switch (SDL_GameControllerTypeForIndex(joy_index)){
//             case SDL_CONTROLLER_TYPE_XBOX360: case SDL_CONTROLLER_TYPE_XBOXONE:
//                 SDL_SetHint(SDL_HINT_JOYSTICK_HIDAPI_XBOX, "1");
//                 break;
//             case SDL_CONTROLLER_TYPE_PS4:
//                 SDL_SetHint(SDL_HINT_JOYSTICK_HIDAPI_PS4_RUMBLE, "1");
//                 break;
//             case SDL_CONTROLLER_TYPE_PS5:
//                 SDL_SetHint(SDL_HINT_JOYSTICK_HIDAPI_PS5_RUMBLE, "1");
//                 break;
//         }
//         if (SDL_GameControllerSetSensorEnabled(controller_container.back(), SDL_SENSOR_GYRO, SDL_TRUE) < 0)
//             if (verbose) std::cout << "Could not enable the gyroscope for this controller" << std::endl;
//         if (SDL_GameControllerSetSensorEnabled(controller_container.back(), SDL_SENSOR_ACCEL, SDL_TRUE) < 0)
//             if (verbose) std::cout << "Could not enable the acclelerometer for this controller" << std::endl;
//         return 0;
//     } else {
//         std::cerr << "Error: the controller is not supported by the game controller interface" << std::endl;
//         return 1;
//     } 
// }

// int PuaraJoystick::openController(int joy_index) {
//     openController(controllers, joy_index);
//     return 0;
// }

int PuaraJoystick::openAllControllers() {
    if (SDL_NumJoysticks() == 0) {
        std::cerr << "Warning: could not find any controller" << std::endl;
        return 1;
    }
    for (int i = 0; i < SDL_NumJoysticks(); i++)
        openController(i);
    return 0;
}

// std::vector<SDL_GameController*> PuaraJoystick::openAllControllers(std::vector<SDL_GameController*> &controller_container) {
//     if (SDL_NumJoysticks() == 0) {
//         std::cerr << "Warning: could not find any controller" << std::endl;
//         return controller_container;
//     }
//     for (int i = 0; i < SDL_NumJoysticks(); i++)
//         openController(controller_container, i);
//     return controller_container;
// }

// int PuaraJoystick::openAllControllers() {
//     openAllControllers(controllers);
//     return 0;
// }

float PuaraJoystick::clip(float n, float lower, float upper) {
  return std::max(lower, std::min(n, upper));
}

int PuaraJoystick::clip(int n, int lower, int upper) {
  return std::max(lower, std::min(n, upper));
}

void PuaraJoystick::pullSDLEvent(SDL_Event event){
    if (event.type == SDL_QUIT) {
        sdl_quit = true;
    } else if (event.type == SDL_CONTROLLERDEVICEADDED) {
        openController(event.cdevice.which);
    }
    switch (event.type) {
        case SDL_CONTROLLERDEVICEREMOVED:
            for (auto controller : controllers_temp) {
                SDL_GameControllerClose(controllers_temp[event.cdevice.which].instance);
                controllers_temp.erase(event.cdevice.which);
                if (verbose) std::cout << "Controller " << event.cdevice.which << "vanished!" << std::endl;
            }
            break;
        case SDL_CONTROLLERBUTTONDOWN: case SDL_CONTROLLERBUTTONUP:
            full_namespace = OSC_namespace + "_" + std::to_string(event.cdevice.which) + "/" + SDL_GameControllerButton_list[event.cbutton.button];
            elapsed_time = event.cbutton.timestamp - last_button_event_time[event.cbutton.button];
            last_button_event_time[event.cbutton.button] = event.cbutton.timestamp;
            //controllers_temp[event.cdevice.which].state.
            if (verbose) std::cout << "OSC message: " << full_namespace << " " << event.cbutton.state << elapsed_time << std::endl;
            break;
        case SDL_CONTROLLERAXISMOTION:
            full_namespace = OSC_namespace + std::to_string(event.cdevice.which) + "/" + SDL_GameControllerAxis_list[event.caxis.axis];
            if (verbose) std::cout << "OSC message: " << full_namespace << " " << event.caxis.value << std::endl;
            break;
        case SDL_CONTROLLERSENSORUPDATE:
            full_namespace = OSC_namespace + std::to_string(event.cdevice.which) + "/" + SDL_SensorType_list[event.csensor.sensor];
            if (verbose) std::cout << "OSC message to port " << osc_port << ": " << full_namespace << " " << event.csensor.data[0] << " " << event.csensor.data[1]  << " " << event.csensor.data[2] << "\r" << std::flush;
            break;
    }
}

bool PuaraJoystick::getQuit(){
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

template <typename Key, typename Value>
void PuaraJoystick::ReversibleMap<Key, Value>::insert(const Key& key, const Value& value) {
    forwardMap[key] = value;
    reverseMap[value] = key;
}

template <typename Key, typename Value>
const Value& PuaraJoystick::ReversibleMap<Key, Value>::operator[](const Key& key) const {
    return forwardMap.at(key);
}

template <typename Key, typename Value>
const Key& PuaraJoystick::ReversibleMap<Key, Value>::reverseLookup(const Value& value) const {
    return reverseMap.at(value);
}