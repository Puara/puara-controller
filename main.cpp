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

std::string OSC_namespace = "/puaradeck/";

std::string SDL_GameControllerButton_list[] = {"A","B","X","Y","back","guide","start","leftstick","rightstick","leftshoulder","rightshoulder","dpad_up","dpad_down","dpad_left","dpad_right","misc1","paddle1","paddle2","paddle3","paddle4","touchpad","max"};
std::string SDL_GameControllerAxis_list[] = {"leftx","lefty","rightx","righty","triggerleft","triggerright","max"};

std::string SDL_SensorType_list[] = {"unknown","accel","gyro"};

std::uint32_t last_button_event_time[sizeof(SDL_GameControllerButton_list)/sizeof(std::uint32_t)] = {0};

SDL_GameController *openFirstController() {
    for (int i = 0; i < SDL_NumJoysticks(); i++) {
        if (SDL_IsGameController(i)) {
            std::cout << "Game controller found. Opening...\n" << std::endl;
            SDL_GameController *controller = SDL_GameControllerOpen(i);
            std::cout << "\nController \""<< SDL_GameControllerNameForIndex(i) 
                      << "\" (" << i << ") " << "opened successfully" << std::endl;
            return controller;
        }
    }
    return nullptr;
}

inline SDL_JoystickID getControllerInstanceID(SDL_GameController *controller) {
    return SDL_JoystickInstanceID(
            SDL_GameControllerGetJoystick(controller));
}

//lo::ServerThread osc_server(9000);

// int init_OSC () {
//     osc_server.add_method("example", "i",
//                   [](lo_arg **argv, int)
//                   {std::cout << "example (" << "): "
//                              << argv[0]->i << std::endl;});
//     osc_server.start();
//     return 1;
// }

int main(int argc, char* args[]) {

    // Init
    std::cout << "Starting Puara Joystick..." << std::endl;

    if (SDL_Init(SDL_INIT_GAMECONTROLLER | SDL_INIT_SENSOR) < 0) {
        std::cerr << "Could not initialize sdl2: " << SDL_GetError() << std::endl;
        //fprintf(stderr, "could not initialize sdl2: %s\n", SDL_GetError());
        return 1;
    } else {std::cout << "SDL2 initialized successfully" << std::endl;}

    bool quit = false;

    SDL_GameController *controller = openFirstController();

    if (SDL_GameControllerSetSensorEnabled(controller, SDL_SENSOR_GYRO, SDL_TRUE) < 0) {
		printf("Could not enable gyro for controller");
	}
    if (SDL_GameControllerSetSensorEnabled(controller, SDL_SENSOR_ACCEL, SDL_TRUE) < 0) {
		printf("Could not enable gyro for controller");
	}
    SDL_SetHint(SDL_HINT_JOYSTICK_HIDAPI_PS4_RUMBLE, "1");
    SDL_SetHint(SDL_HINT_JOYSTICK_HIDAPI_PS5_RUMBLE, "1");
    SDL_SetHint(SDL_HINT_JOYSTICK_HIDAPI_STEAM, "1");
    SDL_SetHint(SDL_HINT_JOYSTICK_HIDAPI_XBOX, "1");

    SDL_Event event;

    // Loop
    while (!quit) {
        if (SDL_PollEvent( &event ) != 0) {
            if (event.type == SDL_QUIT) {
                quit = true;
            } else if (event.type == SDL_CONTROLLERDEVICEADDED && !controller) {
                std::cout << "New controller connected, switching to the new controller..." << std::endl;
                controller = SDL_GameControllerOpen(event.cdevice.which);
            }
            if (controller && event.cdevice.which == getControllerInstanceID(controller)) {
                std::string full_namespace;
                std::int32_t elapsed_time;
                switch (event.type) {
                    case SDL_CONTROLLERDEVICEREMOVED:
                        SDL_GameControllerClose(controller);
                        std::cout << "Controller vanished! Trying to find new controller..." << std::endl;
                        controller = openFirstController();
                        break;
                    case SDL_CONTROLLERBUTTONDOWN:
                    case SDL_CONTROLLERBUTTONUP:
                        full_namespace = OSC_namespace + SDL_GameControllerButton_list[event.cbutton.button];
                        elapsed_time = event.cbutton.timestamp - last_button_event_time[event.cbutton.button];
                        last_button_event_time[event.cbutton.button] = event.cbutton.timestamp;
                        std::cout << "To be sent: " << full_namespace << " " << event.cbutton.state << elapsed_time << std::endl;
                        break;
                    case SDL_CONTROLLERAXISMOTION:
                        full_namespace = OSC_namespace + SDL_GameControllerAxis_list[event.caxis.axis];
                        std::cout << "To be sent: " << full_namespace << " " << event.caxis.value << std::endl;
                        break;
                    case SDL_CONTROLLERSENSORUPDATE:
                        full_namespace = OSC_namespace + SDL_SensorType_list[event.csensor.sensor];
                        std::cout << "To be sent: " << full_namespace << " " << event.csensor.data[0] << " " << event.csensor.data[1]  << " " << event.csensor.data[2] << std::endl;
                        // SDL_Log("Controller %d sensor %s: %.2f, %.2f, %.2f\n",
                        // event.csensor.which,
                        // event.csensor.sensor == SDL_SENSOR_ACCEL ? "accelerometer" :
                        // event.csensor.sensor == SDL_SENSOR_GYRO ? "gyro" : "unknown",
                        // event.csensor.data[0],
                        // event.csensor.data[1],
                        // event.csensor.data[2]);
                        // break;
                }
            }  
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    
    SDL_Quit();

    return 0;
}