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
    } else {std::cout << "SDL2 initialized successfully" << std::endl;}
    full_namespace = OSC_namespace + "/rumble";
    osc_server.add_method(full_namespace, "iiff",
        [&](lo_arg **argv, int) {
            int controller = clip(argv[0]->i, 0, (controllers.size()-1));
            int time = clip(argv[1]->i, 10, 10000);
            float low_freq = clip(argv[2]->f, 0.0f, 1.0f) * 65535;
            float hi_freq = clip(argv[3]->f, 0.0f, 1.0f) * 65535;
            std::cout << "args: " 
            << " " << controller 
            << " " << time
            << " " << low_freq
            << " " << hi_freq
            << std::endl;
            SDL_GameControllerRumble(controllers.at(controller), low_freq, hi_freq, time);
            //std::cout << "rumble! (" << "): " << argv[0]->i << std::endl;
        });
    osc_server.start();
};

int PuaraJoystick::openController(std::vector<SDL_GameController*> &controller_container, int joy_index) {
    if (SDL_IsGameController(joy_index)) {
        std::cout << "Game controller found. Opening...\n" << std::endl;
        controller_container.push_back(SDL_GameControllerOpen(joy_index));
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
        if (SDL_GameControllerSetSensorEnabled(controller_container.back(), SDL_SENSOR_GYRO, SDL_TRUE) < 0)
            std::cout << "Could not enable the gyroscope for this controller" << std::endl;
        if (SDL_GameControllerSetSensorEnabled(controller_container.back(), SDL_SENSOR_ACCEL, SDL_TRUE) < 0)
            std::cout << "Could not enable the acclelerometer for this controller" << std::endl;
        return 0;
    } else {
        std::cerr << "Error: the controller is not supported by the game controller interface" << std::endl;
        return 1;
    } 
}

int PuaraJoystick::openController(int joy_index) {
    openController(controllers, joy_index);
    return 0;
}

float PuaraJoystick::clip(float n, float lower, float upper) {
  return std::max(lower, std::min(n, upper));
}

int PuaraJoystick::clip(int n, int lower, int upper) {
  return std::max(lower, std::min(n, upper));
}

std::vector<SDL_GameController*> PuaraJoystick::openAllControllers(std::vector<SDL_GameController*> &controller_container) {
    if (SDL_NumJoysticks() == 0) {
        std::cerr << "Warning: could not find any controller" << std::endl;
        return controller_container;
    }
    for (int i = 0; i < SDL_NumJoysticks(); i++)
        openController(controller_container, i);
    return controller_container;
}

int PuaraJoystick::openAllControllers() {
    openAllControllers(controllers);
    return 0;
}

void PuaraJoystick::processSDLEvent(SDL_Event event){
    if (event.type == SDL_QUIT) {
        sdl_quit = true;
    } else if (event.type == SDL_CONTROLLERDEVICEADDED) {
        std::cout << "New controller connected, switching to the new controller..." << std::endl;
        openController(controllers,event.cdevice.which);
    }
    switch (event.type) {
        case SDL_CONTROLLERDEVICEREMOVED:
            for (auto controller : controllers) {
                if (SDL_GameControllerFromInstanceID(event.cdevice.which) == controller) {
                    SDL_GameControllerClose(controller);
                    std::cout << "Controller vanished!" << std::endl;
                }
            }
            break;
        case SDL_CONTROLLERBUTTONDOWN: case SDL_CONTROLLERBUTTONUP:
            full_namespace = OSC_namespace + "_" + std::to_string(event.cdevice.which) + "/" + SDL_GameControllerButton_list[event.cbutton.button];
            elapsed_time = event.cbutton.timestamp - last_button_event_time[event.cbutton.button];
            last_button_event_time[event.cbutton.button] = event.cbutton.timestamp;
            std::cout << "OSC message: " << full_namespace << " " << event.cbutton.state << elapsed_time << std::endl;
            break;
        case SDL_CONTROLLERAXISMOTION:
            full_namespace = OSC_namespace + std::to_string(event.cdevice.which) + "/" + SDL_GameControllerAxis_list[event.caxis.axis];
            std::cout << "OSC message: " << full_namespace << " " << event.caxis.value << std::endl;
            break;
        case SDL_CONTROLLERSENSORUPDATE:
            full_namespace = OSC_namespace + std::to_string(event.cdevice.which) + "/" + SDL_SensorType_list[event.csensor.sensor];
            std::cout << "OSC message to port " << osc_port << ": " << full_namespace << " " << event.csensor.data[0] << " " << event.csensor.data[1]  << " " << event.csensor.data[2] << std::endl;
            break;
    }
}

bool PuaraJoystick::getQuit(){
    return sdl_quit; 
}