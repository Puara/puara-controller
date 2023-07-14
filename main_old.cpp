/* 
 * Puara Deck
 * Controller -> OSC/MIDI bridge
 *
 * SAT/Metalab & IDMIL
 * Edu Meneses (2023)
*/


#include <SDL2/SDL.h>
#include <iostream>
#include <vector>

#include <lo/lo.h>
#include <lo/lo_cpp.h>

#include <chrono>
#include <thread>

std::string OSC_namespace = "/puaradeck_";
std::string full_namespace;
std::int32_t elapsed_time;

std::string SDL_GameControllerButton_list[] = {"A","B","X","Y","back","guide","start","leftstick","rightstick","leftshoulder","rightshoulder","dpad_up","dpad_down","dpad_left","dpad_right","misc1","paddle1","paddle2","paddle3","paddle4","touchpad","max"};
std::string SDL_GameControllerAxis_list[] = {"leftx","lefty","rightx","righty","triggerleft","triggerright","max"};

std::string SDL_SensorType_list[] = {"unknown","accel","gyro"};

std::uint32_t last_button_event_time[sizeof(SDL_GameControllerButton_list)/sizeof(std::uint32_t)] = {0};

int openController(std::vector<SDL_GameController*> controller_container, int joy_index) {
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

std::vector<SDL_GameController*> openAllControllers() {
    std::vector<SDL_GameController*> controller_container;
    if (SDL_NumJoysticks() == 0) {
        std::cerr << "Warning: could not find any controller during initialization" << std::endl;
        return controller_container;
    }
    for (int i = 0; i < SDL_NumJoysticks(); i++)
        openController(controller_container, i);
    return controller_container;
}

inline SDL_JoystickID getControllerInstanceID(SDL_GameController *controller) {
    return SDL_JoystickInstanceID(
            SDL_GameControllerGetJoystick(controller));
}

lo::ServerThread osc_server(9000);

int initOSC () {
    osc_server.add_method("/example", "i",
                  [](lo_arg **argv, int)
                  {std::cout << "example (" << "): "
                             << argv[0]->i << std::endl;});
    osc_server.start();
    return 1;
}

int main(int argc, char* args[]) {

    initOSC ();

    // Init SDL
    std::cout << "Starting Puara Joystick..." << std::endl;

    if (SDL_Init(SDL_INIT_GAMECONTROLLER | SDL_INIT_SENSOR) < 0) {
        std::cerr << "Could not initialize sdl2: " << SDL_GetError() << std::endl;
        return 1;
    } else {std::cout << "SDL2 initialized successfully" << std::endl;}

    bool quit = false;

    //SDL_GameController *controller = openFirstController();
    std::vector<SDL_GameController*> controllers = openAllControllers();

    SDL_Event event;

    // Loop
    while (!quit) {
        if (SDL_PollEvent( &event ) != 0) {
            if (event.type == SDL_QUIT) {
                quit = true;
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
                    full_namespace = OSC_namespace + std::to_string(event.cdevice.which) + "/" + SDL_GameControllerButton_list[event.cbutton.button];
                    elapsed_time = event.cbutton.timestamp - last_button_event_time[event.cbutton.button];
                    last_button_event_time[event.cbutton.button] = event.cbutton.timestamp;
                    std::cout << "To be sent: " << full_namespace << " " << event.cbutton.state << elapsed_time << std::endl;
                    break;
                case SDL_CONTROLLERAXISMOTION:
                    full_namespace = OSC_namespace + std::to_string(event.cdevice.which) + "/" + SDL_GameControllerAxis_list[event.caxis.axis];
                    std::cout << "To be sent: " << full_namespace << " " << event.caxis.value << std::endl;
                    break;
                case SDL_CONTROLLERSENSORUPDATE:
                    full_namespace = OSC_namespace + std::to_string(event.cdevice.which) + "/" + SDL_SensorType_list[event.csensor.sensor];
                    std::cout << "To be sent: " << full_namespace << " " << event.csensor.data[0] << " " << event.csensor.data[1]  << " " << event.csensor.data[2] << std::endl;
                    break;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    
    SDL_Quit();

    return 0;
}