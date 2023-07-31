//****************************************************************************//
// Puara Joystick standalone - connect with game controllers using SDL2       //
//                             Controller -> OSC/MIDI bridge                  //
// https://github.com/Puara/puara-joystick                                    //
// Metalab - Société des Arts Technologiques (SAT)                            //
// Input Devices and Music Interaction Laboratory (IDMIL), McGill University  //
// Edu Meneses (2023) - https://www.edumeneses.com                            //
//****************************************************************************//

#include "puara_controller.hpp"
#include <chrono>
#include <thread>

// Initialize Puara's jostick module
PuaraController puara;

int main(int argc, char* args[]) {

    SDL_Event event;

    // Loop
    while (!puara.doQuit()) {
        if (SDL_PollEvent( &event ) != 0) {
            puara.printEvent(puara.pullSDLEvent(event));
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    
    SDL_Quit();

    return 0;
}