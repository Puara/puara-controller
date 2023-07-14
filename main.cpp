/* 
 * Puara Deck
 * Controller -> OSC/MIDI bridge
 *
 * SAT/Metalab & IDMIL
 * Edu Meneses (2023)
*/

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