//****************************************************************************//
// Puara Controller module - connect with game controllers using SDL2 (hpp)   //
//                         Controller -> OSC/MIDI bridge                      //
// https://github.com/Puara/puara-controller                                  //
// Metalab - Société des Arts Technologiques (SAT)                            //
// Input Devices and Music Interaction Laboratory (IDMIL), McGill University  //
// Edu Meneses (2023) - https://www.edumeneses.com                            //
//****************************************************************************//


#ifndef PUARA_CONTROLLER_H
#define PUARA_CONTROLLER_H

#include "SDL.h"
#include <iostream>
#include <sstream>
#include <thread>
#include <atomic>
#include <condition_variable>
#include <vector>
#include <unordered_map>
#include <cmath>
#include <cstdint>
#include <chrono>


namespace puara_controller {
        
    struct ControllerEvent {
        unsigned int controller; 
        unsigned int eventType; 
        int eventAction; 
        int touchID;
        std::string eventName;
    };

    extern ControllerEvent currentEvent;

    extern std::atomic<bool> keep_running;
    extern std::condition_variable controller_event;
    extern std::mutex controller_event_mutex;
    extern std::vector<std::thread> threads;
    extern std::thread joiner;
    void joinAllThreads(std::vector<std::thread>& threads);

    int start();
    int openController(int joy_index);
    int pullSDLEvent(SDL_Event event);
    void pullControllerEventThread();
    int rumble(int controllerID, int time, float low_freq, float hi_freq);
    void printEvent();
    void printEvent(bool printSensor);
    void printEventThread();
    void quit();
    extern std::string identifier;
    extern bool verbose;
    extern int move_buffer_size;
    extern int analogDeadZone;
    extern bool enableMotion;
    extern int polling_interval;
    extern bool print_events;
    extern bool print_motion_data;

    int mapRange(int in, int inMin, int inMax, float outMin, float outMax);
    float mapRange(float in, float inMin, float inMax, float outMin, float outMax);
    double mapRange(double in, double inMin, double inMax, double outMin, double outMax);

    template<typename T>
    class CircularBuffer {
        public:
            CircularBuffer() : CircularBuffer(10) {};
            explicit CircularBuffer(size_t capacity);
            bool isEmpty() const;
            bool isFull() const;
            void push(const T& item);
            T pop();
        private:
            std::vector<T> buffer_;
            size_t capacity_;
            size_t size_;
            size_t read_index_;
            size_t write_index_;
    };

    struct Button {
        int value = 0; 
        int event_duration = 0; 
        Uint64 event_timestamp = 0;
    };
    struct Trigger {
        bool state = false; 
        int value = 0; 
        int event_duration = 0; 
        Uint64 event_timestamp = 0;
    };
    struct Sensor {
        float X = 0.0;
        float Y = 0.0;
        float Z = 0.0;
        int event_duration = 0; 
        Uint64 event_timestamp = 0;
    };
    struct Analog {
        bool state = false; 
        int X = 0; 
        int Y = 0; 
        int event_duration = 0; 
        Uint64 event_timestamp = 0;
    };
    struct Touch {
        int action = 0;
        int touchpad = 0;
        int finger = 0;
        float X = 0.0;
        float Y = 0.0;
        float pressure = 0.0;
        int event_duration = 0; 
        Uint64 event_timestamp = 0;
    };
    struct ControllerState {
        std::unordered_map<int, Button> button;
        std::unordered_map<int, Sensor> motion;
        std::unordered_map<int, Analog> analog;
        std::unordered_map<int, Trigger> trigger;
        std::unordered_map<int, Touch> touch;
        // Sensor accel, gyro;
        // Axis analogR, analogL;
        // Trigger triggerL, triggerR;
        // Touch touch;
        bool rumble;
    };

    class Controller {
        public:
            Controller() = default;
            explicit Controller(int move_buffer_size) : discrete_buffer(move_buffer_size) {};
            Controller(int id, SDL_Gamepad* instance, int move_buffer_size);
            ControllerState state;
            CircularBuffer<ControllerState> discrete_buffer;
            bool is_open = false;
            int id;
            SDL_Gamepad* instance;
            
    };

    extern std::unordered_map<int, Controller> controllers;
    extern std::unordered_map<std::string, std::unordered_map<int, std::string>> SDL2Name;

    extern std::int32_t elapsed_time_;
    float clip_(float n, float lower, float upper);
    int clip_(int n, int lower, int upper);
    int nano2mili(Uint64 ns);
    std::string replaceID(std::string str, int newID);
    double applyDeadZone_(double in, double in_min, double in_max, double out_min, double out_max, double dead_zone_min, double dead_zone_max, double dead_zone_value);
    int applyAnalogDeadZone_(int in);
    bool isSensorChanged_(int joy_index, std::string sensor, std::string side);
};

#endif