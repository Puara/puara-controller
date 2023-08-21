//****************************************************************************//
// Puara Controller standalone - Connect with game controllers using SDL3     //
//                               Controller -> OSC/MIDI bridge                //
// https://github.com/Puara/puara-controller                                  //
// Metalab - Société des Arts Technologiques (SAT)                            //
// Input Devices and Music Interaction Laboratory (IDMIL), McGill University  //
// Edu Meneses (2023) - https://www.edumeneses.com                            //
//****************************************************************************//

#include "puara_controller.hpp"
#include <iostream>
#include <chrono>
#include <thread>
#include <atomic>
#include <csignal>
#include <vector>
#include <condition_variable>
#include "json/json.h"
#include <fstream>
#include <unordered_map>
#include <iterator>
#include "lo/lo.h"
#include "lo/lo_cpp.h"

#include <cmath>

int polling_interval = 2; // milliseconds
bool verbose = false;
bool print_events = false;
bool print_motion_data = false;
int osc_server_port = 9000;
std::string osc_client_address = "localhost";
int osc_client_port = 9001;
bool disable_motion = false;
std::string forward_address;
int forward_port;

std::atomic<bool> keep_running(true);
std::condition_variable cv;
std::mutex mtx;
PuaraController puaracontroller;
lo::ServerThread osc_server(osc_server_port);
lo::Address osc_sender(osc_client_address, osc_client_port);
std::vector<std::thread> threads;

struct Mapping {
    std::string internal_address;
    int controller_id;
    std::string forward_namespace;
    std::string forward_port;
    std::vector<std::string> forward_arguments;
    float min_range, max_range;
};
std::unordered_map<std::string, Mapping> custom_mappings;

struct Midi {
    std::string type;
    int channel;
    std::string note_value;
    std::string velocity_controller;
};
std::vector<Midi> puara_midi;

void printHelp(const char* programName) {
    std::cout << "Puara Controller standalone - Connect with game controllers using SDL3     \n"
              << "                              Controller -> OSC/MIDI bridge                \n"
              << "https://github.com/Puara/puara-controller                                  \n"
              << "Metalab - Société des Arts Technologiques (SAT)                            \n"
              << "Input Devices and Music Interaction Laboratory (IDMIL), McGill University  \n"
              << "Edu Meneses (2023)                                                         \n"
              << "\nUsage: " << programName << " [options] <json_file>\n"
              << "Options:\n"
              << "  -h,       Show help message\n"
              << "  -c,       Provide a JSON config file"
              << std::endl;
}

int readJson(std::string jsonFileName) {
    std::ifstream jsonFile(jsonFileName);
    if (!jsonFile.is_open()) {
        std::cerr << "Failed to open JSON file: " << jsonFileName << std::endl;
        return 1;
    }
    Json::Value root;
    //Json::Value mappings = root["mappings"];
    jsonFile >> root;
    // Reading config
    polling_interval = root["config"]["polling_interval"].asInt();
    disable_motion = root["config"]["disable_motion"].asBool();
    verbose = root["config"]["verbose"].asBool();
    print_events = root["config"]["print_events"].asBool();
    print_motion_data = root["config"]["print_motion_data"].asBool();
    puaracontroller.identifier = root["config"]["osc_namespace"].asString();
    osc_server_port = root["config"]["osc_server_port"].asInt();
    osc_client_address = root["config"]["osc_client_address"].asString();
    osc_client_port = root["config"]["osc_client_port"].asInt();
    forward_address = root["config"]["forward_address"].asString();
    forward_port = root["config"]["forward_port"].asInt();
    // Reading mappings
    for (const Json::Value& mapInfo : root["forwarders"]) {
        custom_mappings.emplace(
            mapInfo["internal_address"].asString(), 
            Mapping{
                .internal_address = mapInfo["internal_address"].asString(),
                .controller_id = mapInfo["controller_id"].asInt(),
                .forward_namespace = mapInfo["forward_namespace"].asString()
            }
        );
        if (!mapInfo["range"]["min"].isNull()) {
            custom_mappings[mapInfo["internal_address"].asString()].min_range = mapInfo["range"]["min"].asFloat();
        } else {
            custom_mappings[mapInfo["internal_address"].asString()].min_range = 0;
        }
        if (!mapInfo["range"]["max"].isNull()) {
            custom_mappings[mapInfo["internal_address"].asString()].max_range = mapInfo["range"]["max"].asFloat();
        } else {
            custom_mappings[mapInfo["internal_address"].asString()].max_range = 0;
        }
        
        
        if (mapInfo["forward_arguments"].isArray()) {
            for (const Json::Value& element : mapInfo["forward_arguments"]) {
                custom_mappings[mapInfo["internal_address"].asString()].forward_arguments.push_back(element.asString());
            }
        }
    }
    // Reading MIDI
    for (const Json::Value& midiInfo : root["midi"]) {
        puara_midi.emplace_back(
            Midi{
                .type = midiInfo["type"].asString() ,
                .channel = midiInfo["channel"].asInt(),
                .note_value = midiInfo["note_value"].asString() ,
                .velocity_controller = midiInfo["velocity_controller"].asString() 
            }
        );
    }
    return 0;
}

void killlHandler(int signum) {
    std::cout << "\nClosing Puara Controller..." << std::endl;
    SDL_Quit();
    keep_running.store(false);
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
    
    std::string puara_namespace = "/" + puaracontroller.identifier + "/" + std::to_string(puaraEvent.controller) + "/" + puaraEvent.eventName;

    lo::Message msg;
    switch (puaraEvent.eventType) {
        case SDL_EVENT_GAMEPAD_BUTTON_DOWN: case SDL_EVENT_GAMEPAD_BUTTON_UP:
            msg.add(puaracontroller.controllers[puaraEvent.controller].state.button[puaraEvent.eventAction].value);
            msg.add(puaracontroller.controllers[puaraEvent.controller].state.button[puaraEvent.eventAction].event_duration);
            break;
        case SDL_EVENT_GAMEPAD_AXIS_MOTION:
            switch (puaraEvent.eventAction) {
                case SDL_GAMEPAD_AXIS_LEFTX: case SDL_GAMEPAD_AXIS_LEFTY:
                    msg.add(puaracontroller.controllers[puaraEvent.controller].state.analogL.X);
                    msg.add(puaracontroller.controllers[puaraEvent.controller].state.analogL.Y);
                    msg.add(puaracontroller.controllers[puaraEvent.controller].state.analogL.event_duration);
                    break;
                case SDL_GAMEPAD_AXIS_RIGHTX: case SDL_GAMEPAD_AXIS_RIGHTY:
                    msg.add(puaracontroller.controllers[puaraEvent.controller].state.analogR.X);
                    msg.add(puaracontroller.controllers[puaraEvent.controller].state.analogR.Y);
                    msg.add(puaracontroller.controllers[puaraEvent.controller].state.analogR.event_duration);
                    break;
                case SDL_GAMEPAD_AXIS_LEFT_TRIGGER:
                    msg.add(puaracontroller.controllers[puaraEvent.controller].state.triggerL.value);
                    msg.add(puaracontroller.controllers[puaraEvent.controller].state.triggerL.event_duration);
                    break;
                case SDL_GAMEPAD_AXIS_RIGHT_TRIGGER:
                    msg.add(puaracontroller.controllers[puaraEvent.controller].state.triggerR.value);
                    msg.add(puaracontroller.controllers[puaraEvent.controller].state.triggerR.event_duration);
                    break;
            }
            break;
        case SDL_EVENT_GAMEPAD_TOUCHPAD_DOWN: case SDL_EVENT_GAMEPAD_TOUCHPAD_MOTION: case SDL_EVENT_GAMEPAD_TOUCHPAD_UP:
            msg.add(puaracontroller.controllers[puaraEvent.controller].state.touch.touchpad);
            msg.add(puaracontroller.controllers[puaraEvent.controller].state.touch.finger);
            msg.add(puaracontroller.controllers[puaraEvent.controller].state.touch.X);
            msg.add(puaracontroller.controllers[puaraEvent.controller].state.touch.Y);
            break;
        case SDL_EVENT_GAMEPAD_SENSOR_UPDATE:
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
    osc_sender.send(puara_namespace, msg);

    return 0;
}

// High-level gesture draft for azimuth
int calculateAngle(int X, int Y) {
    double x = puaracontroller.mapRange(X, -32768, 32767, -1.0f, 1.0f);
    double y = puaracontroller.mapRange((-1*Y), -32768, 32767, -1.0f, 1.0f);
    double azimuth = std::atan2(x, y) * 180 / M_PI;
    return static_cast<int>(azimuth);
}

int sendCustomOSC(PuaraController::EventResume puaraEvent) {
    
    if (
        puaraEvent.eventAction == -1 ||
        puaraEvent.controller != custom_mappings[puaraEvent.eventName].controller_id ||
        forward_address.empty() ||
        forward_port == 0
    ) return 1;

    lo::Address custom_osc_sender(forward_address, forward_port);
    lo::Message msg;
    float min_range = custom_mappings[puaraEvent.eventName].min_range;
    float max_range = custom_mappings[puaraEvent.eventName].max_range;

    switch (puaraEvent.eventType) {
        case SDL_EVENT_GAMEPAD_BUTTON_DOWN: case SDL_EVENT_GAMEPAD_BUTTON_UP:
            for (auto& argument : custom_mappings[puaraEvent.eventName].forward_arguments) {
                if (argument == "value") {
                    if (min_range != max_range) {
                        msg.add(puaracontroller.mapRange(
                            puaracontroller.controllers[puaraEvent.controller].state.button[puaraEvent.eventAction].value,
                            0, 1, min_range, max_range));
                    } else {
                        msg.add(puaracontroller.controllers[puaraEvent.controller].state.button[puaraEvent.eventAction].value);
                    }
                } else if (argument == "timestamp") {
                    msg.add(puaracontroller.controllers[puaraEvent.controller].state.button[puaraEvent.eventAction].event_duration);
                } else if (!argument.empty() && argument.front() == '$') {
                    msg.add_string(argument.substr(1));
                } else if (!argument.empty() && argument.front() == '#') {
                    msg.add(std::stof(argument.substr(1)));
                } else if (!argument.empty() && argument.front() == '!') {
                    msg.add(std::stoi(argument.substr(1)));
                }
            }
            break;
        case SDL_EVENT_GAMEPAD_AXIS_MOTION:
            switch (puaraEvent.eventAction) {
                case SDL_GAMEPAD_AXIS_LEFTX: case SDL_GAMEPAD_AXIS_LEFTY:
                    for (auto& argument : custom_mappings[puaraEvent.eventName].forward_arguments) {
                        if (argument == "X") {
                            if (min_range != max_range) {
                                msg.add(puaracontroller.mapRange(
                                    puaracontroller.controllers[puaraEvent.controller].state.analogL.X,
                                    -32768, 32768, min_range, max_range));
                            } else {
                                msg.add(puaracontroller.controllers[puaraEvent.controller].state.analogL.X);
                            };
                        } else if (argument == "Y") {
                            if (min_range != max_range) {
                                msg.add(puaracontroller.mapRange(
                                    puaracontroller.controllers[puaraEvent.controller].state.analogL.Y,
                                    -32768, 32768, min_range, max_range));
                            } else {
                                msg.add(puaracontroller.controllers[puaraEvent.controller].state.analogL.Y);
                            };
                        } else if (argument == "azimuth") {
                            msg.add(calculateAngle(
                                puaracontroller.controllers[puaraEvent.controller].state.analogL.X,
                                puaracontroller.controllers[puaraEvent.controller].state.analogL.Y));
                        } else if (argument == "timestamp") {
                            msg.add(puaracontroller.controllers[puaraEvent.controller].state.analogL.event_duration);
                        } else if (!argument.empty() && argument.front() == '$') {
                            msg.add_string(argument.substr(1));;
                        }
                    }
                    break;
                case SDL_GAMEPAD_AXIS_RIGHTX: case SDL_GAMEPAD_AXIS_RIGHTY:
                    for (auto& argument : custom_mappings[puaraEvent.eventName].forward_arguments) {
                        if (argument == "X") {
                            if (min_range != max_range) {
                                msg.add(puaracontroller.mapRange(
                                    puaracontroller.controllers[puaraEvent.controller].state.analogR.X,
                                    -32768, 32768, min_range, max_range));
                            } else {
                                msg.add(puaracontroller.controllers[puaraEvent.controller].state.analogR.X);
                            };
                        } else if (argument == "Y") {
                            if (min_range != max_range) {
                                msg.add(puaracontroller.mapRange(
                                    puaracontroller.controllers[puaraEvent.controller].state.analogR.Y,
                                    -32768, 32768, min_range, max_range));
                            } else {
                                msg.add(puaracontroller.controllers[puaraEvent.controller].state.analogR.Y);
                            };
                        } else if (argument == "azimuth") {
                            msg.add(calculateAngle(
                                puaracontroller.controllers[puaraEvent.controller].state.analogR.X,
                                puaracontroller.controllers[puaraEvent.controller].state.analogR.Y));
                        } else if (argument == "timestamp") {
                            msg.add(puaracontroller.controllers[puaraEvent.controller].state.analogR.event_duration);
                        } else if (!argument.empty() && argument.front() == '$') {
                            msg.add_string(argument.substr(1));
                        }
                    }
                    break;
                case SDL_GAMEPAD_AXIS_LEFT_TRIGGER:
                    for (auto& argument : custom_mappings[puaraEvent.eventName].forward_arguments) {
                        if (argument == "value") {
                            if (min_range != max_range) {
                                msg.add(puaracontroller.mapRange(
                                    puaracontroller.controllers[puaraEvent.controller].state.triggerL.value,
                                    0, 32768, min_range, max_range));
                            } else {
                                msg.add(puaracontroller.controllers[puaraEvent.controller].state.triggerL.value);
                            };
                        } else if (argument == "timestamp") {
                            msg.add(puaracontroller.controllers[puaraEvent.controller].state.triggerL.event_duration);
                        } else if (!argument.empty() && argument.front() == '$') {
                            msg.add_string(argument.substr(1));
                        }
                    }
                    break;
                case SDL_GAMEPAD_AXIS_RIGHT_TRIGGER:
                    for (auto& argument : custom_mappings[puaraEvent.eventName].forward_arguments) {
                        if (argument == "value") {
                            if (min_range != max_range) {
                                msg.add(puaracontroller.mapRange(
                                    puaracontroller.controllers[puaraEvent.controller].state.triggerR.value,
                                    0, 32768, min_range, max_range));
                            } else {
                                msg.add(puaracontroller.controllers[puaraEvent.controller].state.triggerR.value);
                            };
                        } else if (argument == "timestamp") {
                            msg.add(puaracontroller.controllers[puaraEvent.controller].state.triggerR.event_duration);
                        } else if (!argument.empty() && argument.front() == '$') {
                            msg.add_string(argument.substr(1));
                        }
                    }
                    break;
            }
            break;
        case SDL_EVENT_GAMEPAD_TOUCHPAD_DOWN: case SDL_EVENT_GAMEPAD_TOUCHPAD_MOTION: case SDL_EVENT_GAMEPAD_TOUCHPAD_UP:
            for (auto& argument : custom_mappings[puaraEvent.eventName].forward_arguments) {
                if (argument == "touchId") {
                    msg.add(puaracontroller.controllers[puaraEvent.controller].state.touch.touchpad);
                } else if (argument == "fingerId") {
                    msg.add(puaracontroller.controllers[puaraEvent.controller].state.touch.finger);
                } else if (argument == "X") {
                    if (min_range != max_range) {
                            msg.add(puaracontroller.mapRange(
                                puaracontroller.controllers[puaraEvent.controller].state.touch.X,
                                0.0f, 1.0f, min_range, max_range));
                    } else {
                        msg.add(puaracontroller.controllers[puaraEvent.controller].state.touch.X);
                    };
                } else if (argument == "Y") {
                    if (min_range != max_range) {
                            msg.add(puaracontroller.mapRange(
                                puaracontroller.controllers[puaraEvent.controller].state.touch.Y,
                                0.0f, 1.0f, min_range, max_range));
                    } else {
                        msg.add(puaracontroller.controllers[puaraEvent.controller].state.touch.Y);
                    };
                } else if (!argument.empty() && argument.front() == '$') {
                    msg.add_string(argument.substr(1));
                }
            }
            break;
        case SDL_EVENT_GAMEPAD_SENSOR_UPDATE:
            if (puaraEvent.eventAction == SDL_SENSOR_ACCEL) {
                for (auto& argument : custom_mappings[puaraEvent.eventName].forward_arguments) {
                    if (argument == "X") {
                        if (min_range != max_range) {
                            msg.add(puaracontroller.mapRange(
                                puaracontroller.controllers[puaraEvent.controller].state.accel.X,
                                -40.0f, 40.0f, min_range, max_range));
                        } else {
                            msg.add(puaracontroller.controllers[puaraEvent.controller].state.accel.X);
                        };
                    } else if (argument == "Y") {
                        if (min_range != max_range) {
                            msg.add(puaracontroller.mapRange(
                                puaracontroller.controllers[puaraEvent.controller].state.accel.Y,
                                -40.0f, 40.0f, min_range, max_range));
                        } else {
                            msg.add(puaracontroller.controllers[puaraEvent.controller].state.accel.Y);
                        };
                    } else if (argument == "Z") {
                        if (min_range != max_range) {
                            msg.add(puaracontroller.mapRange(
                                puaracontroller.controllers[puaraEvent.controller].state.accel.Z,
                                -40.0f, 40.0f, min_range, max_range));
                        } else {
                            msg.add(puaracontroller.controllers[puaraEvent.controller].state.accel.Z);
                        };
                    } else if (!argument.empty() && argument.front() == '$') {
                        msg.add_string(argument.substr(1));
                    }
                }
            } else if (puaraEvent.eventAction == SDL_SENSOR_GYRO) {
                for (auto& argument : custom_mappings[puaraEvent.eventName].forward_arguments) {
                    if (argument == "X") {
                        if (min_range != max_range) {
                            msg.add(puaracontroller.mapRange(
                                puaracontroller.controllers[puaraEvent.controller].state.gyro.X,
                                -40.0f, 40.0f, min_range, max_range));
                        } else {
                            msg.add(puaracontroller.controllers[puaraEvent.controller].state.gyro.X);
                        };
                    } else if (argument == "Y") {
                        if (min_range != max_range) {
                            msg.add(puaracontroller.mapRange(
                                puaracontroller.controllers[puaraEvent.controller].state.gyro.Y,
                                -40.0f, 40.0f, min_range, max_range));
                        } else {
                            msg.add(puaracontroller.controllers[puaraEvent.controller].state.gyro.Y);
                        };
                    } else if (argument == "Z") {
                        if (min_range != max_range) {
                            msg.add(puaracontroller.mapRange(
                                puaracontroller.controllers[puaraEvent.controller].state.gyro.Z,
                                -40.0f, 40.0f, min_range, max_range));
                        } else {
                            msg.add(puaracontroller.controllers[puaraEvent.controller].state.gyro.Z);
                        };
                    } else if (!argument.empty() && argument.front() == '$') {
                        msg.add_string(argument.substr(1));
                    }
                }
            }
            break;
    }
    custom_osc_sender.send(custom_mappings[puaraEvent.eventName].forward_namespace, msg);

    return 0;
}

void readSDL() {
    SDL_Event sdlEvent;
    PuaraController::EventResume puaraEvent;
    while (keep_running) {
        if (SDL_PollEvent( &sdlEvent ) != 0) {
            if (print_events) {
                puaracontroller.printEvent(puaracontroller.pullSDLEvent(sdlEvent), print_motion_data);
            } else {
                puaraEvent = puaracontroller.pullSDLEvent(sdlEvent);
                sendOSC(puaraEvent);
                if (custom_mappings.find(puaraEvent.eventName) != custom_mappings.end()) {
                    sendCustomOSC(puaraEvent);
                }
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(polling_interval));
    }
}

int main(int argc, char* argv[]) {

    std::cout << "Puara Controller standalone - Connect with game controllers using SDL3\n"
              << "                              Controller -> OSC/MIDI bridge           \n"
              << std::endl;
    
    bool useConfig = false;
    std::string jsonFileName;

    int option;
    while ((option = getopt(argc, argv, "hc: ")) != -1) {
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

    if (useConfig) readJson(jsonFileName);

    libloServerMethods();

    if (!osc_server.is_valid()) {
        std::cerr << "Error, liblo OSC server did not initialized correctly."
                  << " This program cannot accept OSC messages" << std::endl;
    } else {
        osc_server.start();
    }

    puaracontroller.verbose = verbose;
    puaracontroller.enableMotion = !disable_motion;

    threads.emplace_back(readSDL);

    {
        std::unique_lock<std::mutex> lock(mtx);
        cv.wait(lock, []{ return !keep_running; });
    }

    for (auto& thread : threads) {
        thread.join();
    }

    return 0;
}