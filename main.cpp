//****************************************************************************//
// Puara Controller standalone - connect with game controllers using SDL2     //
//                             Controller -> OSC/MIDI bridge                  //
// https://github.com/Puara/puara-controller                                  //
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
#include "json/json.h"
//#include <jsoncpp/json/json.h>
#include <fstream>
#include <unordered_map>
#include <iterator>

#include <lo/lo.h>
//#include <lo/lo_cpp.h>

int polling_interval = 2; // milliseconds
bool verbose = false;
bool print_events = true;
bool print_motion_data = false;
int osc_server_port = 9000;
std::string osc_client_address = "localhost";
int osc_client_port = 9001;
bool disableMotion = false;

std::atomic<bool> keepRunning(true);
std::condition_variable cv;
std::mutex mtx;
PuaraController puaracontroller;
// lo::ServerThread osc_server(osc_server_port);
// lo::Address osc_sender(osc_client_address, osc_client_port);
std::vector<std::thread> threads;

struct Mapping {
    std::string internal_address;
    std::string forward_address;
    std::vector<std::string> forward_arguments;
    float min_range, max_range;
};
std::unordered_map<std::string, Mapping> puaraMappings;

struct Midi {
    std::string type;
    int channel;
    std::string note_value;
    std::string velocity_controller;
};
std::vector<Midi> puaraMidi;

void printHelp(const char* programName) {
    std::cout << " Puara Controller standalone - connect with game controllers using SDL2     \n"
              << "                             Controller -> OSC/MIDI bridge                  \n"
              << " https://github.com/Puara/puara-controller                                  \n"
              << " Metalab - Société des Arts Technologiques (SAT)                            \n"
              << " Input Devices and Music Interaction Laboratory (IDMIL), McGill University  \n"
              << " Edu Meneses (2023)                                                         \n"
              << "\nUsage: " << programName << " [options] <json_file>\n"
              << "Options:\n"
              << "  -h, --help      Show help message\n"
              << "  -c, --config    Provide a JSON config file"
              << std::endl;
}

// int readJson(std::string jsonFileName) {
//     std::ifstream jsonFile(jsonFileName);
//     if (!jsonFile.is_open()) {
//         std::cerr << "Failed to open JSON file: " << jsonFileName << std::endl;
//         return 1;
//     }
//     Json::Value root;
//     //Json::Value mappings = root["mappings"];
//     jsonFile >> root;
//     // Reading config
//     polling_interval = root["config"]["polling_interval"].asInt();
//     disableMotion = root["config"]["disableMotion"].asBool();
//     verbose = root["config"]["verbose"].asBool();
//     print_events = root["config"]["print_events"].asBool();
//     print_motion_data = root["config"]["print_motion_data"].asBool();
//     puaracontroller.identifier = root["config"]["osc_namespace"].asString();
//     osc_server_port = root["config"]["osc_server_port"].asInt();
//     osc_client_address = root["config"]["osc_client_address"].asString();
//     osc_client_port = root["config"]["osc_client_port"].asInt();
//     // Reading mappings
//     for (const Json::Value& mapInfo : root["mappings"]) {
//         puaraMappings.emplace(
//             mapInfo["internal_address"].asString(), 
//             Mapping{
//                 .internal_address = mapInfo["internal_address"].asString(),
//                 .forward_address = mapInfo["forward_address"].asString(),
//                 .min_range = mapInfo["min_range"].asFloat(),
//                 .max_range = mapInfo["max_range"].asFloat()
//             }
//         );
//         if (mapInfo["forward_arguments"].isArray()) {
//             for (const Json::Value& element : mapInfo["forward_arguments"]) {
//                 puaraMappings[mapInfo["internal_address"].asString()].forward_arguments.push_back(element.asString());
//             }
//         }
//     }
//     // Reading MIDI
//     for (const Json::Value& midiInfo : root["midi"]) {
//         puaraMidi.emplace_back(
//             Midi{
//                 .type = midiInfo["type"].asString() ,
//                 .channel = midiInfo["channel"].asInt(),
//                 .note_value = midiInfo["note_value"].asString() ,
//                 .velocity_controller = midiInfo["velocity_controller"].asString() 
//             }
//         );
//     }
//     return 0;
// }

void killlHandler(int signum) {
    std::cout << "\nClosing Puara Controller..." << std::endl;
    SDL_Quit();
    keepRunning.store(false);
    cv.notify_all();
}

// int libloServerMethods(){
//     osc_server.add_method("/puaracontroller/rumble", "iiff",
//         [&](lo_arg **argv, int) {
//             int id = argv[0]->i;
//             int time = argv[1]->i;
//             float low_freq = argv[2]->f;
//             float hi_freq = argv[3]->f;
//             puaracontroller.rumble(id, time, low_freq, hi_freq);
//             if (verbose) std::cout << "Controller " << id << " rumble!" << std::endl;
//         });
//     osc_server.add_method("/puaracontroller/rumble", "i",
//         [&](lo_arg **argv, int) {
//             int id = argv[0]->i;
//             puaracontroller.rumble(id, 65535, 65535, 1000);
//             if (verbose) std::cout << "Controller " << id << " rumble!" << std::endl;
//         });
//         return 0;
// }

// int sendOSC(PuaraController::EventResume puaraEvent) {
//     if (puaraEvent.eventAction == -1)
//         return 1;

//     std::string full_namespace = 
//         "/" + puaracontroller.identifier + 
//         "/" + std::to_string(puaraEvent.controller) + 
//         "/" + puaracontroller.SDL2Name[puaracontroller.SDL2Name["events"][puaraEvent.eventType]][puaraEvent.eventAction];
//     lo::Message msg;
//     switch (puaraEvent.eventType) {
//         case SDL_CONTROLLERBUTTONDOWN: case SDL_CONTROLLERBUTTONUP:
//             msg.add(puaracontroller.controllers[puaraEvent.controller].state.button[puaraEvent.eventAction].value);
//             msg.add(puaracontroller.controllers[puaraEvent.controller].state.button[puaraEvent.eventAction].event_duration);
//             break;
//         case SDL_CONTROLLERAXISMOTION:
//             switch (puaraEvent.eventAction) {
//                 case SDL_CONTROLLER_AXIS_LEFTX: case SDL_CONTROLLER_AXIS_LEFTY:
//                     msg.add(puaracontroller.controllers[puaraEvent.controller].state.analogL.X);
//                     msg.add(puaracontroller.controllers[puaraEvent.controller].state.analogL.Y);
//                     msg.add(puaracontroller.controllers[puaraEvent.controller].state.analogL.event_duration);
//                     break;
//                 case SDL_CONTROLLER_AXIS_RIGHTX: case SDL_CONTROLLER_AXIS_RIGHTY:
//                     msg.add(puaracontroller.controllers[puaraEvent.controller].state.analogR.X);
//                     msg.add(puaracontroller.controllers[puaraEvent.controller].state.analogR.Y);
//                     msg.add(puaracontroller.controllers[puaraEvent.controller].state.analogR.event_duration);
//                     break;
//                 case SDL_CONTROLLER_AXIS_TRIGGERLEFT:
//                     msg.add(puaracontroller.controllers[puaraEvent.controller].state.triggerL.value);
//                     msg.add(puaracontroller.controllers[puaraEvent.controller].state.triggerL.event_duration);
//                     break;
//                 case SDL_CONTROLLER_AXIS_TRIGGERRIGHT:
//                     msg.add(puaracontroller.controllers[puaraEvent.controller].state.triggerR.value);
//                     msg.add(puaracontroller.controllers[puaraEvent.controller].state.triggerR.event_duration);
//                     break;
//             }
//             break;
//         case SDL_CONTROLLERTOUCHPADDOWN: case SDL_CONTROLLERTOUCHPADMOTION: case SDL_CONTROLLERTOUCHPADUP:
//             msg.add(puaracontroller.controllers[puaraEvent.controller].state.touch.touchId);
//             msg.add(puaracontroller.controllers[puaraEvent.controller].state.touch.fingerId);
//             msg.add(puaracontroller.controllers[puaraEvent.controller].state.touch.X);
//             msg.add(puaracontroller.controllers[puaraEvent.controller].state.touch.Y);
//             break;
//         case SDL_CONTROLLERSENSORUPDATE:
//             if (puaraEvent.eventAction == SDL_SENSOR_ACCEL) {
//                     msg.add(puaracontroller.controllers[puaraEvent.controller].state.accel.X);
//                     msg.add(puaracontroller.controllers[puaraEvent.controller].state.accel.Y);
//                     msg.add(puaracontroller.controllers[puaraEvent.controller].state.accel.Z);
//                 } else if (puaraEvent.eventAction == SDL_SENSOR_GYRO) {
//                     msg.add(puaracontroller.controllers[puaraEvent.controller].state.gyro.X);
//                     msg.add(puaracontroller.controllers[puaraEvent.controller].state.gyro.Y);
//                     msg.add(puaracontroller.controllers[puaraEvent.controller].state.gyro.Z);
//                 }
//             break;
//     }
//     osc_sender.send(full_namespace, msg);
//     return 0;
// }

void readSDL() {
    SDL_Event sdlEvent;
    PuaraController::EventResume puaraEvent;
    while (keepRunning) {
        if (SDL_PollEvent( &sdlEvent ) != 0) {
            if (print_events)
                puaracontroller.printEvent(puaracontroller.pullSDLEvent(sdlEvent), print_motion_data);
            else
                puaraEvent = puaracontroller.pullSDLEvent(sdlEvent);
            //sendOSC(puaraEvent);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(polling_interval));
    }
}

int main(int argc, char* argv[]) {

    bool useConfig = false;
    std::string jsonFileName;

    int option;
    while ((option = getopt(argc, argv, "hc::")) != -1) {
        switch (option) {
            case 'h':
                printHelp(argv[0]);
                return 0;
            case 'c':
                useConfig = true;
                jsonFileName = optarg;
                break;
            case '?':
                if (optopt == 'c') {
                    std::cerr << "Option -c requires a JSON config file." << std::endl;
                } else {
                    std::cerr << "Unknown option: " << static_cast<char>(optopt) << std::endl;
                }
                return 1;
            default:
                return 1;
        }
    }

    if ( puaracontroller.start() ) {
        exit(EXIT_FAILURE);
    }

    std::signal(SIGINT, killlHandler);

    // if (useConfig) readJson(jsonFileName);

    // libloServerMethods();

    // if (!osc_server.is_valid()) {
    //     std::cerr << "Error, liblo OSC server did not initialized correctly."
    //               << " This program cannot accept OSC messages" << std::endl;
    // } else {
    //     osc_server.start();
    // }

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