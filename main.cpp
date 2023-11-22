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
#include <algorithm>
#include <iterator>
#include "lo/lo.h"
#include "lo/lo_cpp.h"

#include <cmath>

int osc_server_port = 9000;
std::string osc_client_address = "localhost";
int osc_client_port = 9001;

std::condition_variable cv;
std::mutex mtx;
lo::ServerThread osc_server(osc_server_port);
lo::Address osc_sender(osc_client_address, osc_client_port);
std::vector<std::thread> threads;

struct OSCMapping {
    int controller_id;
    std::string osc_namespace;
    struct OSCArguments {
        std::string action;
        std::string value;
        float min;
        float max;
    };
    std::vector<OSCArguments> arguments;
};
std::vector <OSCMapping> osc_mappings;
std::unordered_map<std::string, std::vector<int>> osc_trigger;

struct LibmapperMapping {
    std::string direction;
    std::string name;
    struct LibmapperArguments {
        std::string action;
        std::string value;
        float min;
        float max;
    };
    std::vector<LibmapperArguments> arguments;
};
std::vector<LibmapperMapping> libmapper_mappings;
std::unordered_map<std::string, std::vector<int>> libmapper_trigger;

struct MidiMapping {
    int controller_id;
    std::string type;
    int channel;
    struct MidiArguments {
        std::string action;
        std::string value;
        float min;
        float max;
    };
    MidiArguments key_controller;
    MidiArguments velocity_value;
};
std::vector<MidiMapping> midi_mappings;
std::unordered_map<std::string, std::vector<int>> midi_trigger;

void printHelp(const char* programName) {
    std::cout << "Puara Controller standalone - Connect with game controllers using SDL3     \n"
              << "                              Controller -> OSC/MIDI bridge                \n"
              << "https://github.com/Puara/puara-controller                                  \n"
              << "Metalab - Société des Arts Technologiques (SAT)                            \n"
              << "Input Devices and Music Interaction Laboratory (IDMIL), McGill University  \n"
              << "Edu Meneses (2023)                                                         \n"
              << "\nUsage: " << programName << " [options] <json_file>\n"
              << "Options:\n"
              << "  -h,       Show this help message\n"
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
    // Reading osc
    for (const Json::Value& mapOSC : root["osc"]) {
        osc_mappings.emplace(osc_mappings.end(), 
            OSCMapping{
                .controller_id = (mapOSC["controller_id"].isNull()) ? -1 : mapOSC["controller_id"].asInt(),
                .osc_namespace = (mapOSC["namespace"].isNull()) ? "puaracontroller" : mapOSC["namespace"].asString(),
            }
        );
        for (const Json::Value& argumentOSC : mapOSC["arguments"]) {
            osc_mappings.back().arguments.emplace(
                osc_mappings.back().arguments.end(),
                OSCMapping::OSCArguments{
                    .action = (argumentOSC["action"].isNull()) ? "text" : argumentOSC["action"].asString(),
                    .value = (argumentOSC["value"].isNull()) ? "none" : argumentOSC["value"].asString(),
                    .min = (argumentOSC["min"].isNull()) ? 0 : argumentOSC["min"].asFloat(),
                    .max = (argumentOSC["max"].isNull()) ? 0 : argumentOSC["max"].asFloat(),
                }
            );
            if (osc_trigger.find(osc_mappings.back().arguments.back().action) == osc_trigger.end()) {
                osc_trigger.emplace(osc_mappings.back().arguments.back().action, std::vector<int>{static_cast<int>(osc_mappings.size())-1});
            } else {
                int mappingIndex = static_cast<int>(osc_mappings.size())-1;
                auto checkDuplicate = std::find(osc_trigger[osc_mappings.back().arguments.back().action].begin(), osc_trigger[osc_mappings.back().arguments.back().action].end(), mappingIndex);
                if (checkDuplicate == osc_trigger[osc_mappings.back().arguments.back().action].end()) {
                    osc_trigger[osc_mappings.back().arguments.back().action].push_back(mappingIndex);
                }
            };
        };
    };
    // Reading libmapper
    for (const Json::Value& mapLibmapper : root["libmapper"]) {
        libmapper_mappings.emplace(libmapper_mappings.end(), 
            LibmapperMapping{
                .direction = (mapLibmapper["direction"].isNull()) ? "out" : mapLibmapper["direction"].asString(),
                .name = (mapLibmapper["name"].isNull()) ? "puaracontroller" : mapLibmapper["name"].asString(),
            }
        );
        for (const Json::Value& argumentLibmapper : mapLibmapper["arguments"]) {
            libmapper_mappings.back().arguments.emplace(
                libmapper_mappings.back().arguments.end(),
                LibmapperMapping::LibmapperArguments{
                    .action = (argumentLibmapper["action"].isNull()) ? "text" : argumentLibmapper["action"].asString(),
                    .value = (argumentLibmapper["value"].isNull()) ? "none" : argumentLibmapper["value"].asString(),
                    .min = (argumentLibmapper["min"].isNull()) ? 0 : argumentLibmapper["min"].asFloat(),
                    .max = (argumentLibmapper["max"].isNull()) ? 0 : argumentLibmapper["max"].asFloat(),
                }
            );
            if (libmapper_trigger.find(libmapper_mappings.back().arguments.back().action) == libmapper_trigger.end()) {
                libmapper_trigger.emplace(libmapper_mappings.back().arguments.back().action, std::vector<int>{static_cast<int>(libmapper_mappings.size())-1});
            } else {
                int mappingIndex = static_cast<int>(libmapper_mappings.size())-1;
                auto checkDuplicate = std::find(libmapper_trigger[libmapper_mappings.back().arguments.back().action].begin(), libmapper_trigger[libmapper_mappings.back().arguments.back().action].end(), mappingIndex);
                if (checkDuplicate == libmapper_trigger[libmapper_mappings.back().arguments.back().action].end()) {
                    libmapper_trigger[libmapper_mappings.back().arguments.back().action].push_back(mappingIndex);
                }
            };
        };
    };
    // Reading midi
    for (const Json::Value& mapMidi : root["midi"]) {
        midi_mappings.emplace(midi_mappings.end(), 
            MidiMapping{
                .controller_id = (mapMidi["controller_id"].isNull()) ? -1 : mapMidi["controller_id"].asInt(),
                .type = (mapMidi["type"].isNull()) ? "on" : mapMidi["type"].asString(),
                .channel = (mapMidi["channel"].isNull()) ? 0 : mapMidi["channel"].asInt(),
            }
        );
        if (midi_mappings.back().type == "cc") {
            midi_mappings.back().key_controller.action = (mapMidi["controller"]["action"].isNull()) ? "none" : mapMidi["controller"]["action"].asString();
            midi_mappings.back().velocity_value.action = (mapMidi["value"]["action"].isNull()) ? "none" : mapMidi["value"]["action"].asString();
        } else {
            midi_mappings.back().key_controller.action = (mapMidi["key"]["action"].isNull()) ? "none" : mapMidi["key"]["action"].asString();
            midi_mappings.back().velocity_value.action = (mapMidi["velocity"]["action"].isNull()) ? "none" : mapMidi["velocity"]["action"].asString();
        };
        if (midi_trigger.find(midi_mappings.back().key_controller.action) == midi_trigger.end()) {
                midi_trigger.emplace(midi_mappings.back().key_controller.action, std::vector<int>{static_cast<int>(midi_mappings.size())-1});
        } else {
            int mappingIndex = static_cast<int>(midi_mappings.size())-1;
            auto checkDuplicate = std::find(midi_trigger[midi_mappings.back().key_controller.action].begin(), midi_trigger[midi_mappings.back().key_controller.action].end(), mappingIndex);
            if (checkDuplicate == midi_trigger[midi_mappings.back().key_controller.action].end()) {
                midi_trigger[midi_mappings.back().key_controller.action].push_back(mappingIndex);
            }
        };
        if (midi_trigger.find(midi_mappings.back().velocity_value.action) == midi_trigger.end()) {
                midi_trigger.emplace(midi_mappings.back().velocity_value.action, std::vector<int>{static_cast<int>(midi_mappings.size())-1});
        } else {
            int mappingIndex = static_cast<int>(midi_mappings.size())-1;
            auto checkDuplicate = std::find(midi_trigger[midi_mappings.back().velocity_value.action].begin(), midi_trigger[midi_mappings.back().velocity_value.action].end(), mappingIndex);
            if (checkDuplicate == midi_trigger[midi_mappings.back().velocity_value.action].end()) {
                midi_trigger[midi_mappings.back().velocity_value.action].push_back(mappingIndex);
            }
        };
    };
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
            puara_controller::rumble(id, 200, 1.0, 1.0);
            if (puara_controller::verbose) std::cout << "Controller " << id << " rumble!" << std::endl;
        });
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

std::string replaceString(const std::string& initial, const std::string& target, const std::string& replacement) {
    std::string result = initial;
    size_t pos = result.find(target);
    while (pos != std::string::npos) {
        result.replace(pos, target.length(), replacement);
        pos = result.find(target, pos + replacement.length());
    }
    return result;
}

int sendOSC(puara_controller::ControllerEvent currentEvent) {
    if (osc_trigger.find(currentEvent.eventName) != osc_trigger.end()) {
        lo::Address osc_sender(osc_client_address, osc_client_port);
        for (int mappingID : osc_trigger[currentEvent.eventName]) {
            std::vector<int> controllerIDs;
            if (osc_mappings[mappingID].controller_id == -1) {
                for (const auto& pair : puara_controller::controllers) {
                    controllerIDs.push_back(pair.first);
                }
            } else {
                controllerIDs.push_back(osc_mappings[mappingID].controller_id);
            }
            for (int controllerID : controllerIDs) {
                lo::Message msg;
                for (OSCMapping::OSCArguments argument : osc_mappings[mappingID].arguments) {
                    switch (puara_controller::state2int(argument.value)) {
                        case 0:
                            msg.add(puara_controller::mapRange(
                                puara_controller::controllers[controllerID].state[argument.action].value,
                                0, 1, argument.min, argument.max));
                            break;
                        case 1:
                            msg.add(puara_controller::controllers[controllerID].state[argument.action].event_duration);
                            break;
                        case 2:
                            msg.add(static_cast<int64_t>(puara_controller::controllers[controllerID].state[argument.action].event_timestamp));
                            break;
                        case 3:
                            msg.add(static_cast<int>(puara_controller::controllers[controllerID].state[argument.action].state));
                            break;
                        case 4:
                            msg.add(puara_controller::mapRange(
                                puara_controller::controllers[controllerID].state[argument.action].X,
                                -32768.0f, 32767.0f, argument.min, argument.max));
                            break;
                        case 5:
                            msg.add(puara_controller::mapRange(
                                puara_controller::controllers[controllerID].state[argument.action].Y,
                                -32768.0f, 32767.0f, argument.min, argument.max));
                            break;
                        case 6:
                            msg.add(puara_controller::mapRange(
                                puara_controller::controllers[controllerID].state[argument.action].Z,
                                -32768.0f, 32767.0f, argument.min, argument.max));
                            break;
                        case 7:
                            msg.add(puara_controller::controllers[controllerID].state[argument.action].touchpad);
                            break;
                        case 8:
                            msg.add(puara_controller::controllers[controllerID].state[argument.action].finger);
                            break;
                        case 9:
                            msg.add(puara_controller::mapRange(
                                puara_controller::controllers[controllerID].state[argument.action].pressure,
                                0.0f, 1.0f, argument.min, argument.max));
                            break;
                        default:
                            switch (getType(argument.value)) {
                                case ArgumentType::Integer:
                                    msg.add(static_cast<int>(extractNumberFromString(argument.value)));
                                    break;
                                case ArgumentType::Float:
                                    msg.add(extractNumberFromString(argument.value));
                                    break;
                                case ArgumentType::NotANumber:
                                    msg.add_string(argument.value);
                                    break;
                            };   
                            break;
                    };
                };
                std::string final_namespace = replaceString(osc_mappings[mappingID].osc_namespace, "<ID>", std::to_string(currentEvent.controller));
                osc_sender.send(final_namespace, msg);
            };
        };
    };
    return 0;
};

void sendOSCThread() {
    while (puara_controller::keep_running.load()) {
        std::unique_lock<std::mutex> lock(puara_controller::controller_event_mutex);
        puara_controller::controller_event.wait(lock);
        sendOSC(puara_controller::currentEvent);
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