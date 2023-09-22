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
#include <stdexcept>
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

int osc_server_port = 9000;
std::string osc_client_address = "localhost";
int osc_client_port = 9001;
std::string forward_address;
int forward_port;

std::condition_variable cv;
std::mutex mtx;
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
    if (!root["config"]["polling_frequency"].isNull()) puara_controller::polling_frequency = root["config"]["polling_frequency"].asInt();
    if (!root["config"]["analog_dead_zone"].isNull()) puara_controller::analogDeadZone = root["config"]["analog_dead_zone"].asInt();
    if (!root["config"]["disable_motion"].isNull()) puara_controller::enableMotion = !root["config"]["disable_motion"].asBool();
    if (!root["config"]["verbose"].isNull()) puara_controller::verbose = root["config"]["verbose"].asBool();
    if (!root["config"]["print_events"].isNull()) puara_controller::print_events = root["config"]["print_events"].asBool();
    if (!root["config"]["print_motion_data"].isNull()) puara_controller::print_motion_data = root["config"]["print_motion_data"].asBool();
    if (!root["config"]["osc_namespace"].isNull()) puara_controller::identifier = root["config"]["osc_namespace"].asString();
    if (!root["config"]["osc_server_port"].isNull()) osc_server_port = root["config"]["osc_server_port"].asInt();
    if (!root["config"]["osc_client_address"].isNull()) osc_client_address = root["config"]["osc_client_address"].asString();
    if (!root["config"]["osc_client_port"].isNull()) osc_client_port = root["config"]["osc_client_port"].asInt();
    if (!root["config"]["forward_address"].isNull()) forward_address = root["config"]["forward_address"].asString();
    if (!root["config"]["forward_port"].isNull()) forward_port = root["config"]["forward_port"].asInt();
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
    puara_controller::keep_running.store(false);
    cv.notify_all();
    puara_controller::controller_event.notify_all();
    for (auto& thread : threads) {
        thread.join();
    }
}

int libloServerMethods(){
    osc_server.add_method("/puaracontroller/rumble", "iiff",
        [&](lo_arg **argv, int) {
            int id = argv[0]->i;
            int time = argv[1]->i;
            float low_freq = argv[2]->f;
            float hi_freq = argv[3]->f;
            puara_controller::rumble(id, time, low_freq, hi_freq);
            if (puara_controller::verbose) std::cout << "Controller " << id << " rumble!" << std::endl;
        });
    osc_server.add_method("/puaracontroller/rumble", "i",
        [&](lo_arg **argv, int) {
            int id = argv[0]->i;
            puara_controller::rumble(id, 65535, 65535, 1000);
            if (puara_controller::verbose) std::cout << "Controller " << id << " rumble!" << std::endl;
        });
        return 0;
}

int sendOSC(puara_controller::ControllerEvent puaraEvent) {
    if (puaraEvent.eventAction == -1)
        return 1;
    
    std::string puara_namespace = "/" + puara_controller::identifier + "/" + std::to_string(puaraEvent.controller) + "/" + puaraEvent.eventName;

    lo::Message msg;
    switch (puaraEvent.eventType) {
        case SDL_EVENT_GAMEPAD_BUTTON_DOWN: case SDL_EVENT_GAMEPAD_BUTTON_UP:
            msg.add(puara_controller::controllers[puaraEvent.controller].state.button[puaraEvent.eventAction].value);
            msg.add(puara_controller::controllers[puaraEvent.controller].state.button[puaraEvent.eventAction].event_duration);
            break;
        case SDL_EVENT_GAMEPAD_AXIS_MOTION:
            switch (puaraEvent.eventAction) {
                case SDL_GAMEPAD_AXIS_LEFTX: case SDL_GAMEPAD_AXIS_LEFTY:
                    msg.add(puara_controller::controllers[puaraEvent.controller].state.analog[SDL_GAMEPAD_AXIS_LEFTX].X);
                    msg.add(puara_controller::controllers[puaraEvent.controller].state.analog[SDL_GAMEPAD_AXIS_LEFTX].Y);
                    msg.add(puara_controller::controllers[puaraEvent.controller].state.analog[SDL_GAMEPAD_AXIS_LEFTX].event_duration);
                    break;
                case SDL_GAMEPAD_AXIS_RIGHTX: case SDL_GAMEPAD_AXIS_RIGHTY:
                    msg.add(puara_controller::controllers[puaraEvent.controller].state.analog[SDL_GAMEPAD_AXIS_RIGHTX].X);
                    msg.add(puara_controller::controllers[puaraEvent.controller].state.analog[SDL_GAMEPAD_AXIS_RIGHTX].Y);
                    msg.add(puara_controller::controllers[puaraEvent.controller].state.analog[SDL_GAMEPAD_AXIS_RIGHTX].event_duration);
                    break;
                case SDL_GAMEPAD_AXIS_LEFT_TRIGGER:
                    msg.add(puara_controller::controllers[puaraEvent.controller].state.trigger[SDL_GAMEPAD_AXIS_LEFT_TRIGGER].value);
                    msg.add(puara_controller::controllers[puaraEvent.controller].state.trigger[SDL_GAMEPAD_AXIS_LEFT_TRIGGER].event_duration);
                    break;
                case SDL_GAMEPAD_AXIS_RIGHT_TRIGGER:
                    msg.add(puara_controller::controllers[puaraEvent.controller].state.trigger[SDL_GAMEPAD_AXIS_RIGHT_TRIGGER].value);
                    msg.add(puara_controller::controllers[puaraEvent.controller].state.trigger[SDL_GAMEPAD_AXIS_RIGHT_TRIGGER].event_duration);
                    break;
            }
            break;
        case SDL_EVENT_GAMEPAD_TOUCHPAD_DOWN: case SDL_EVENT_GAMEPAD_TOUCHPAD_MOTION: case SDL_EVENT_GAMEPAD_TOUCHPAD_UP:
            msg.add(puara_controller::controllers[puaraEvent.controller].state.touch[puaraEvent.touchID].touchpad);
            msg.add(puara_controller::controllers[puaraEvent.controller].state.touch[puaraEvent.touchID].finger);
            msg.add(puara_controller::controllers[puaraEvent.controller].state.touch[puaraEvent.touchID].X);
            msg.add(puara_controller::controllers[puaraEvent.controller].state.touch[puaraEvent.touchID].Y);
            break;
        case SDL_EVENT_GAMEPAD_SENSOR_UPDATE:
            if (puaraEvent.eventAction == SDL_SENSOR_ACCEL) {
                    msg.add(puara_controller::controllers[puaraEvent.controller].state.motion[SDL_SENSOR_ACCEL].X);
                    msg.add(puara_controller::controllers[puaraEvent.controller].state.motion[SDL_SENSOR_ACCEL].Y);
                    msg.add(puara_controller::controllers[puaraEvent.controller].state.motion[SDL_SENSOR_ACCEL].Z);
                } else if (puaraEvent.eventAction == SDL_SENSOR_GYRO) {
                    msg.add(puara_controller::controllers[puaraEvent.controller].state.motion[SDL_SENSOR_GYRO].X);
                    msg.add(puara_controller::controllers[puaraEvent.controller].state.motion[SDL_SENSOR_GYRO].Y);
                    msg.add(puara_controller::controllers[puaraEvent.controller].state.motion[SDL_SENSOR_GYRO].Z);
                }
            break;
    }
    osc_sender.send(puara_namespace, msg);

    return 0;
}

// High-level gesture draft for azimuth
int calculateAngle(int X, int Y) {
    double x = puara_controller::mapRange(static_cast<double>(X), -32768.0, 32767.0, -1.0, 1.0);
    double y = puara_controller::mapRange(static_cast<double>(-1*Y), -32768.0, 32767.0, -1.0, 1.0);
    double azimuth = std::atan2(x, y) * 180 / M_PI;
    return static_cast<int>(azimuth);
}

enum ArgumentType { Integer, Float, NotANumber };

int getType(const std::string& input) {
    std::istringstream iss(input);
    float temp;
    if ((iss >> temp >> std::ws).eof()) {
        return (static_cast<int>(temp) == temp) ? ArgumentType::Integer : ArgumentType::Float;
    } else {
        return ArgumentType::NotANumber;
    }
}

float extractNumberFromString(const std::string& input) {
    std::istringstream iss(input);
    float number;
    if (iss >> number) {
        return number;
    } else {
        throw std::invalid_argument("Invalid number format");
    }
}

int sendCustomOSC(puara_controller::ControllerEvent puaraEvent) {
    if (puaraEvent.eventAction == -1 ||
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
                    msg.add(puara_controller::mapRange(
                            puara_controller::controllers[puaraEvent.controller].state.button[puaraEvent.eventAction].value,
                            0, 1, min_range, max_range));
                } else if (argument == "timestamp") {
                    msg.add(puara_controller::controllers[puaraEvent.controller].state.button[puaraEvent.eventAction].event_duration);
                } else {
                    switch (getType(argument)) {
                        case ArgumentType::Integer:
                            msg.add(static_cast<int>(extractNumberFromString(argument)));
                            break;
                        case ArgumentType::Float:
                            msg.add(extractNumberFromString(argument));
                            break;
                        case ArgumentType::NotANumber:
                            msg.add_string(argument);
                            break;
                    }
                }
            }
            break;
        case SDL_EVENT_GAMEPAD_AXIS_MOTION:
            switch (puaraEvent.eventAction) {
                case SDL_GAMEPAD_AXIS_LEFTX: case SDL_GAMEPAD_AXIS_LEFTY: case SDL_GAMEPAD_AXIS_RIGHTX: case SDL_GAMEPAD_AXIS_RIGHTY:
                    for (auto& argument : custom_mappings[puaraEvent.eventName].forward_arguments) {
                        if (argument == "X") {
                            msg.add(puara_controller::mapRange(
                                    puara_controller::controllers[puaraEvent.controller].state.analog[puaraEvent.eventAction].X,
                                    -32768, 32768, min_range, max_range));
                        } else if (argument == "Y") {
                            msg.add(puara_controller::mapRange(
                                    puara_controller::controllers[puaraEvent.controller].state.analog[puaraEvent.eventAction].Y,
                                    -32768, 32768, min_range, max_range));
                        } else if (argument == "azimuth") {
                            msg.add(calculateAngle(
                                puara_controller::controllers[puaraEvent.controller].state.analog[puaraEvent.eventAction].X,
                                puara_controller::controllers[puaraEvent.controller].state.analog[puaraEvent.eventAction].Y));
                        } else if (argument == "timestamp") {
                            msg.add(puara_controller::controllers[puaraEvent.controller].state.analog[puaraEvent.eventAction].event_duration);
                        } else {
                            switch (getType(argument)) {
                                case ArgumentType::Integer:
                                    msg.add(static_cast<int>(extractNumberFromString(argument)));
                                    break;
                                case ArgumentType::Float:
                                    msg.add(extractNumberFromString(argument));
                                    break;
                                case ArgumentType::NotANumber:
                                    msg.add_string(argument);
                                    break;
                            }
                        }
                    }
                    break;
                case SDL_GAMEPAD_AXIS_LEFT_TRIGGER: case SDL_GAMEPAD_AXIS_RIGHT_TRIGGER:
                    for (auto& argument : custom_mappings[puaraEvent.eventName].forward_arguments) {
                        if (argument == "value") {
                            msg.add(puara_controller::mapRange(
                                    puara_controller::controllers[puaraEvent.controller].state.trigger[puaraEvent.eventAction].value,
                                    0, 32767, min_range, max_range));
                        } else if (argument == "timestamp") {
                            msg.add(puara_controller::controllers[puaraEvent.controller].state.trigger[puaraEvent.eventAction].event_duration);
                        } else {
                            switch (getType(argument)) {
                                case ArgumentType::Integer:
                                    msg.add(static_cast<int>(extractNumberFromString(argument)));
                                    break;
                                case ArgumentType::Float:
                                    msg.add(extractNumberFromString(argument));
                                    break;
                                case ArgumentType::NotANumber:
                                    msg.add_string(argument);
                                    break;
                            }
                        }
                    }
                    break;
            }
            break;
        case SDL_EVENT_GAMEPAD_TOUCHPAD_DOWN: case SDL_EVENT_GAMEPAD_TOUCHPAD_MOTION: case SDL_EVENT_GAMEPAD_TOUCHPAD_UP:
            for (auto& argument : custom_mappings[puaraEvent.eventName].forward_arguments) {
                if (argument == "touchId") {
                    msg.add(puara_controller::controllers[puaraEvent.controller].state.touch[puaraEvent.touchID].touchpad);
                } else if (argument == "fingerId") {
                    msg.add(puara_controller::controllers[puaraEvent.controller].state.touch[puaraEvent.touchID].finger);
                } else if (argument == "X") {
                    msg.add(puara_controller::mapRange(
                            puara_controller::controllers[puaraEvent.controller].state.touch[puaraEvent.touchID].X,
                            0.0f, 1.0f, min_range, max_range));
                } else if (argument == "Y") {
                    msg.add(puara_controller::mapRange(
                            puara_controller::controllers[puaraEvent.controller].state.touch[puaraEvent.touchID].Y,
                            0.0f, 1.0f, min_range, max_range));
                } else if (argument == "pressure") {
                    msg.add(puara_controller::controllers[puaraEvent.controller].state.touch[puaraEvent.touchID].pressure);
                } else {
                    switch (getType(argument)) {
                        case ArgumentType::Integer:
                            msg.add(static_cast<int>(extractNumberFromString(argument)));
                            break;
                        case ArgumentType::Float:
                            msg.add(extractNumberFromString(argument));
                            break;
                        case ArgumentType::NotANumber:
                            msg.add_string(argument);
                            break;
                    }
                }
            }
            break;
        case SDL_EVENT_GAMEPAD_SENSOR_UPDATE:
            for (auto& argument : custom_mappings[puaraEvent.eventName].forward_arguments) {
                if (argument == "X") {
                    msg.add(puara_controller::mapRange(
                            puara_controller::controllers[puaraEvent.controller].state.motion[puaraEvent.eventAction].X,
                            -40.0f, 40.0f, min_range, max_range));
                } else if (argument == "Y") {
                    msg.add(puara_controller::mapRange(
                            puara_controller::controllers[puaraEvent.controller].state.motion[puaraEvent.eventAction].Y,
                            -40.0f, 40.0f, min_range, max_range));
                } else if (argument == "Z") {
                    msg.add(puara_controller::mapRange(
                            puara_controller::controllers[puaraEvent.controller].state.motion[puaraEvent.eventAction].Z,
                            -40.0f, 40.0f, min_range, max_range));
                } else {
                    switch (getType(argument)) {
                        case ArgumentType::Integer:
                            msg.add(static_cast<int>(extractNumberFromString(argument)));
                            break;
                        case ArgumentType::Float:
                            msg.add(extractNumberFromString(argument));
                            break;
                        case ArgumentType::NotANumber:
                            msg.add_string(argument);
                            break;
                    }
                }
            }
            break;
    }
    custom_osc_sender.send(
        puara_controller::replaceID(custom_mappings[puaraEvent.eventName].forward_namespace, puara_controller::controllers[puaraEvent.controller].id), 
        msg
    );

    return 0;
}

void sendOSCThread() {
    while (puara_controller::keep_running.load()) {
        std::unique_lock<std::mutex> lock(puara_controller::controller_event_mutex);
        puara_controller::controller_event.wait(lock);
        sendOSC(puara_controller::currentEvent);
        if (custom_mappings.find(puara_controller::currentEvent.eventName) != custom_mappings.end()) {
            sendCustomOSC(puara_controller::currentEvent);
        }
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

    std::signal(SIGINT, killlHandler);

    if (useConfig) readJson(jsonFileName);

    if ( puara_controller::start() ) {
        exit(EXIT_FAILURE);
    }

    libloServerMethods();

    if (!osc_server.is_valid()) {
        std::cerr << "Error, liblo OSC server did not initialized correctly."
                  << " This program cannot accept OSC messages" << std::endl;
    } else {
        osc_server.start();
    }

    threads.emplace_back(sendOSCThread);

    {
        std::unique_lock<std::mutex> lock(mtx);
        cv.wait(lock, []{ return !puara_controller::keep_running; });
    }

    puara_controller::quit();
    
    return 0;
}