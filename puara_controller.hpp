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
    // JM: I recommend always initializing these types's values to 0, e.g. unsigned int controller = 0;     
    struct ControllerEvent {
        unsigned int controller; 
        unsigned int eventType; 
        int eventAction; 
        int touchID;
        std::string eventName;
    };

    // JM: Do all these variable need to be publically visible in the header ? What is using them?
    extern ControllerEvent currentEvent;

    extern std::atomic<bool> keep_running;
    extern std::condition_variable controller_event;
    extern std::mutex controller_event_mutex;
    extern std::vector<std::thread> threads;
    extern std::thread joiner;

    // JM: If you can I recommend you use std::jthread, it handles joining automatically and 
    // in a very elegant way: 
    // https://www.modernescpp.com/index.php/an-improved-thread-with-c-20/
    // https://meetingcpp.com/mcpp/slides/2022/introduction_to_multithreading_cpp204850.pdf
    // 
    // Then of course the big question in 2023 is having an actual async runtime (for instance 
    // in C++, Boost.asio, in Rust it's Tokio, etc) which handles all of this for you and 
    // avoids you from reinventing an event loop
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
    extern int polling_frequency;
    extern bool print_events;
    extern bool print_motion_data;

    // JM: If the range is settable from the outside (e.g. not just in code but through some protocol)
    // it's important to check the (inMax == inMin) case which will crash with SIGFPE otherwise...
    // I see you have an if(outMin != outMax), isn't it the opposite check since it's inMax - inMin which 
    // may lead to a division by zero?
    template <typename InType, typename OutType>
    OutType mapRange(InType in, InType inMin, InType inMax, OutType outMin, OutType outMax) {
        if (outMin != outMax) {
            return static_cast<OutType>((in - inMin) * (outMax - outMin) / (inMax - inMin) + outMin);
        } else {
            return static_cast<OutType>(in);
        }
    }

    // JM: I'd tend to use an existing implementation here e.g. boost's
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
            size_t capacity_; // Same, zero-init
            size_t size_;
            size_t read_index_;
            size_t write_index_;
    };

    // JM:  yes :) 
    struct ControllerState {
        int value = 0; 
        int event_duration = 0; 
        Uint64 event_timestamp = 0; // why Uint64 here where all the other types are standard C++ oens?
        bool state = false;
        float X = 0.0;
        float Y = 0.0;
        float Z = 0.0;
        int action = 0;
        int touchpad = 0;
        int finger = 0;
        float last_X = 0.0;
        float last_Y = 0.0;
        float pressure = 0.0;
    };

    class Controller {
        public:
            // JM: Note that this will not initialize is_open, id and instance  
            Controller() = default; 

            Controller(int id, SDL_Gamepad* instance, int move_buffer_size);
            //ControllerState state;
            std::unordered_map<std::string, ControllerState> state;
            CircularBuffer<ControllerState> discrete_buffer;
            bool is_open;
            int id;
            SDL_Gamepad* instance;
            
    };

    extern std::unordered_map<int, Controller> controllers;
    // JM: What is this map for? are you sure you don't want a map: pair<string, int> -> string ?
    extern std::unordered_map<std::string, std::unordered_map<int, std::string>> SDL2Name;

    // JM: Again this looks like an implementatino detail of state2int below, not sure it needs to be shown 
    // to the world with extern
    extern std::unordered_map<std::string, int> State2int;
    int state2int(std::string state);
    extern std::int32_t elapsed_time_;

    // JM: Why the _'s ?
    float clip_(float n, float lower, float upper);
    int clip_(int n, int lower, int upper);

    // JM: milli
    // (but in modern C++ design, you'd use std::chrono types instead anyways
    // as they carry their SI prefix as part of their type and will convert automatically
    int nano2mili(Uint64 ns);

    // JM: if you pass std::stirng by value in general you can pas std::string_view instead, it will avoid a copy 
    // (std::string like this copies the whole content, while string_view is just (const char*, size))
    std::string replaceID(std::string str, int newID);
    double applyDeadZone_(double in, double in_min, double in_max, double out_min, double out_max, double dead_zone_min, double dead_zone_max, double dead_zone_value);
    int applyAnalogDeadZone(int in);

    // same as above
    bool isSensorChanged(int joy_index, int axis, std::string sensor);
};

#endif