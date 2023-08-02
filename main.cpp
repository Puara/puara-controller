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
#include <atomic>
#include <csignal>
#include <vector>
#include <condition_variable>

#include <lo/lo.h>
#include <lo/lo_cpp.h>

int polling_interval = 2; // milliseconds
bool verbose = false;
bool print_events = false;
bool print_motion_data = false;
int osc_server_port = 9000;
std::string osc_client_address = "localhost";
int osc_client_port = 9001;
bool disableMotion = false;

std::atomic<bool> keepRunning(true);
std::condition_variable cv;
std::mutex mtx;
PuaraController puaracontroller;
lo::ServerThread osc_server(osc_server_port);
lo::Address osc_sender(osc_client_address, osc_client_port);
std::vector<std::thread> threads;

void killlHandler(int signum) {
    std::cout << "\nClosing Puara Controller..." << std::endl;
    SDL_Quit();
    keepRunning.store(false);
    cv.notify_all();
}

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

int sendOSC(PuaraController::EventResume puaraEvent) {
    if (puaraEvent.eventAction == -1)
        return 1;

    std::string full_namespace = 
        "/" + puaracontroller.identifier + 
        "/" + std::to_string(puaraEvent.controller) + 
        "/" + puaracontroller.SDL2Name[puaracontroller.SDL2Name["events"][puaraEvent.eventType]][puaraEvent.eventAction];
    lo::Message msg;
    switch (puaraEvent.eventType) {
        case SDL_CONTROLLERBUTTONDOWN: case SDL_CONTROLLERBUTTONUP:
            msg.add(puaracontroller.controllers[puaraEvent.controller].state.button[puaraEvent.eventAction].value);
            msg.add(puaracontroller.controllers[puaraEvent.controller].state.button[puaraEvent.eventAction].event_duration);
            break;
        case SDL_CONTROLLERAXISMOTION:
            switch (puaraEvent.eventAction) {
                case SDL_CONTROLLER_AXIS_LEFTX: case SDL_CONTROLLER_AXIS_LEFTY:
                    msg.add(puaracontroller.controllers[puaraEvent.controller].state.analogL.X);
                    msg.add(puaracontroller.controllers[puaraEvent.controller].state.analogL.Y);
                    msg.add(puaracontroller.controllers[puaraEvent.controller].state.analogL.event_duration);
                    break;
                case SDL_CONTROLLER_AXIS_RIGHTX: case SDL_CONTROLLER_AXIS_RIGHTY:
                    msg.add(puaracontroller.controllers[puaraEvent.controller].state.analogR.X);
                    msg.add(puaracontroller.controllers[puaraEvent.controller].state.analogR.Y);
                    msg.add(puaracontroller.controllers[puaraEvent.controller].state.analogR.event_duration);
                    break;
                case SDL_CONTROLLER_AXIS_TRIGGERLEFT:
                    msg.add(puaracontroller.controllers[puaraEvent.controller].state.triggerL.value);
                    msg.add(puaracontroller.controllers[puaraEvent.controller].state.triggerL.event_duration);
                    break;
                case SDL_CONTROLLER_AXIS_TRIGGERRIGHT:
                    msg.add(puaracontroller.controllers[puaraEvent.controller].state.triggerR.value);
                    msg.add(puaracontroller.controllers[puaraEvent.controller].state.triggerR.event_duration);
                    break;
            }
            break;
        case SDL_CONTROLLERTOUCHPADDOWN: case SDL_CONTROLLERTOUCHPADMOTION: case SDL_CONTROLLERTOUCHPADUP:
            msg.add(puaracontroller.controllers[puaraEvent.controller].state.touch.touchId);
            msg.add(puaracontroller.controllers[puaraEvent.controller].state.touch.fingerId);
            msg.add(puaracontroller.controllers[puaraEvent.controller].state.touch.X);
            msg.add(puaracontroller.controllers[puaraEvent.controller].state.touch.Y);
            break;
        case SDL_CONTROLLERSENSORUPDATE:
            if (puaraEvent.eventAction == SDL_SENSOR_ACCEL) {
                    msg.add(puaracontroller.controllers[puaraEvent.controller].state.accel.X);
                    msg.add(puaracontroller.controllers[puaraEvent.controller].state.accel.Y);
                    msg.add(puaracontroller.controllers[puaraEvent.controller].state.accel.Z);
                } else if (puaraEvent.eventAction == SDL_SENSOR_GYRO) {
                    msg.add(puaracontroller.controllers[puaraEvent.controller].state.gyro.X);
                    msg.add(puaracontroller.controllers[puaraEvent.controller].state.gyro.Y);
                    msg.add(puaracontroller.controllers[puaraEvent.controller].state.gyro.Z);
                }
            break;
    }
    osc_sender.send(full_namespace, msg);
    return 0;
}

void readSDL() {
    SDL_Event sdlEvent;
    PuaraController::EventResume puaraEvent;
    while (keepRunning) {
        if (SDL_PollEvent( &sdlEvent ) != 0) {
            if (print_events)
                puaracontroller.printEvent(puaracontroller.pullSDLEvent(sdlEvent), print_motion_data);
            else
                puaraEvent = puaracontroller.pullSDLEvent(sdlEvent);
            sendOSC(puaraEvent);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(polling_interval));
    }
}

int main() {

    std::signal(SIGINT, killlHandler);

    libloServerMethods();

    if (!osc_server.is_valid()) {
        std::cerr << "Error, liblo OSC server did not initialized correctly."
                  << " This program cannot accept OSC messages" << std::endl;
    } else {
        osc_server.start();
    }

    puaracontroller.verbose = verbose;
    puaracontroller.enableMotion = !disableMotion;

    threads.emplace_back(readSDL);

    {
        std::unique_lock<std::mutex> lock(mtx);
        cv.wait(lock, []{ return !keepRunning; });
    }

    for (auto& thread : threads) {
        thread.join();
    }

    return 0;
}