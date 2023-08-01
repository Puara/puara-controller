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

#include <lo/lo.h>
#include <lo/lo_cpp.h>

int polling_interval = 2; // milliseconds
bool verbose = false;
bool print_events = false;
int osc_server_port = 9000;
std::string osc_client_address = "localhost";
int osc_client_port = 9000;

PuaraController puaracontroller;

lo::ServerThread osc_server(osc_server_port);
lo::Address osc_sender(osc_client_address, osc_client_port);

int libloServerMethods(){
    osc_server.add_method("/puaracontroller/rumble", "iiff",
        [&](lo_arg **argv, int) {
            int id = argv[0]->i;
            int time = argv[1]->i;
            float low_freq = argv[2]->f;
            float hi_freq = argv[3]->f;
            puaracontroller.rumble(id, time, low_freq, hi_freq);
            if (verbose) std::cout << "Controller " << id << " rumble!" << std::endl;
        });
    osc_server.add_method("/puaracontroller/rumble", "i",
        [&](lo_arg **argv, int) {
            int id = argv[0]->i;
            puaracontroller.rumble(id, 65535, 65535, 1000);
            if (verbose) std::cout << "Controller " << id << " rumble!" << std::endl;
        });
        return 0;
}

SDL_Event event;

int sendOSC(int joy_index, int button) {
    osc_sender.send(puaracontroller.identifier, "i", puaracontroller.controllers[joy_index].state.button[button].value);
    return 0;
}

void readSDL() {
    
    while (!puaracontroller.doQuit()) {
        PuaraController::EventResume puaraEvent;
        if (SDL_PollEvent( &event ) != 0)
            if (print_events)
                puaracontroller.printEvent(puaracontroller.pullSDLEvent(event));
            else
                puaraEvent = puaracontroller.pullSDLEvent(event);
        if  (puaraEvent.eventAction == SDL_CONTROLLER_BUTTON_A)
            sendOSC(puaraEvent.controller, puaraEvent.eventAction);
        std::this_thread::sleep_for(std::chrono::milliseconds(polling_interval));
    }
}



int main() {

    libloServerMethods();

    if (!osc_server.is_valid()) {
        std::cerr << "Error, liblo OSC server did not initialized correctly" << std::endl;
        return 1;
    } else {
        osc_server.start();
        std::cout << "Liblo server started" << std::endl;
        return 0;
    }

    puaracontroller.verbose = verbose;

    std::thread readSDLThread(readSDL);

    readSDLThread.join();
    
    SDL_Quit();

    return 0;
}