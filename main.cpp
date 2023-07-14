//****************************************************************************//
// Puara Joystick standalone - connect with game controllers using SDL2       //
//                             Controller -> OSC/MIDI bridge                  //
// https://github.com/Puara/puara-joystick                                    //
// Metalab - Société des Arts Technologiques (SAT)                            //
// Input Devices and Music Interaction Laboratory (IDMIL), McGill University  //
// Edu Meneses (2023) - https://www.edumeneses.com                            //
//****************************************************************************//

#include "puara_joystick.hpp"
#include <chrono>
#include <thread>

// Initialize Puara's jostick module
PuaraJoystick puara;

int main(int argc, char* args[]) {

    puara.initSDL2();

    puara.initOSC ();

    puara.openAllControllers();

    SDL_Event event;

    // Loop
    while (!puara.getQuit()) {
        if (SDL_PollEvent( &event ) != 0) {
            puara.processSDLEvent(event);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    
    SDL_Quit();

    return 0;
}