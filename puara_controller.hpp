//****************************************************************************//
// Puara Controller module - connect with game controllers using SDL3 (hpp)   //
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
#include <iomanip>
#include <sstream>
#include <thread>
#include <atomic>
#include <condition_variable>
#include <vector>
#include <unordered_map>
#include <cmath>
#include <cstdint>
#include <chrono>
#include <frozen/unordered_map.h>
#include <frozen/string.h>


namespace puara_controller {
    
    constexpr frozen::unordered_map<int, const char*, 9> event2Name = { /* Supported Puara Controller SDL events */
        {SDL_EVENT_GAMEPAD_ADDED,"added"},
        {SDL_EVENT_GAMEPAD_REMOVED,"removed"},
        {SDL_EVENT_GAMEPAD_BUTTON_DOWN,"button"},
        {SDL_EVENT_GAMEPAD_BUTTON_UP,"button"},
        {SDL_EVENT_GAMEPAD_AXIS_MOTION,"axis"},
        {SDL_EVENT_GAMEPAD_SENSOR_UPDATE,"motion"},
        {SDL_EVENT_GAMEPAD_TOUCHPAD_DOWN,"touch"},
        {SDL_EVENT_GAMEPAD_TOUCHPAD_MOTION,"touch"},
        {SDL_EVENT_GAMEPAD_TOUCHPAD_UP,"touch"}
    };
    constexpr frozen::unordered_map<int, const char*, 23> button2Name = { /* This list is generated from SDL_GameControllerButton */
        {SDL_GAMEPAD_BUTTON_INVALID,"invalid"},
        {SDL_GAMEPAD_BUTTON_SOUTH,"A"},
        {SDL_GAMEPAD_BUTTON_EAST,"B"},
        {SDL_GAMEPAD_BUTTON_WEST,"X"},
        {SDL_GAMEPAD_BUTTON_NORTH,"Y"},
        {SDL_GAMEPAD_BUTTON_BACK,"back"},
        {SDL_GAMEPAD_BUTTON_GUIDE,"guide"},
        {SDL_GAMEPAD_BUTTON_START,"start"},
        {SDL_GAMEPAD_BUTTON_LEFT_STICK,"leftstick"},
        {SDL_GAMEPAD_BUTTON_RIGHT_STICK,"rightstick"},
        {SDL_GAMEPAD_BUTTON_LEFT_SHOULDER,"leftshoulder"},
        {SDL_GAMEPAD_BUTTON_RIGHT_SHOULDER,"rightshoulder"},
        {SDL_GAMEPAD_BUTTON_DPAD_UP,"dpad_up"},
        {SDL_GAMEPAD_BUTTON_DPAD_DOWN,"dpad_down"},
        {SDL_GAMEPAD_BUTTON_DPAD_LEFT,"dpad_left"},
        {SDL_GAMEPAD_BUTTON_DPAD_RIGHT,"dpad_right"},
        {SDL_GAMEPAD_BUTTON_MISC1, "misc1"},
        {SDL_GAMEPAD_BUTTON_RIGHT_PADDLE1,"paddle1"},
        {SDL_GAMEPAD_BUTTON_LEFT_PADDLE1,"paddle2"},
        {SDL_GAMEPAD_BUTTON_RIGHT_PADDLE2,"paddle3"},
        {SDL_GAMEPAD_BUTTON_LEFT_PADDLE2,"paddle4"},
        {SDL_GAMEPAD_BUTTON_TOUCHPAD,"touchpadbutton"},
        {SDL_GAMEPAD_BUTTON_MAX,"max_btn"}
    };
    constexpr frozen::unordered_map<int, const char*, 8> axis2Name = { /* This list is generated from SDL_GamepadAxis */
        {SDL_GAMEPAD_AXIS_INVALID,"invalid"},
        {SDL_GAMEPAD_AXIS_LEFTX,"analogleft"},
        {SDL_GAMEPAD_AXIS_LEFTY,"analogleft"},
        {SDL_GAMEPAD_AXIS_RIGHTX,"analogright"},
        {SDL_GAMEPAD_AXIS_RIGHTY,"analogright"},
        {SDL_GAMEPAD_AXIS_LEFT_TRIGGER,"triggerleft"},
        {SDL_GAMEPAD_AXIS_RIGHT_TRIGGER,"triggerright"},
        {SDL_GAMEPAD_AXIS_MAX,"max_axis"}
    };
    constexpr frozen::unordered_map<int, const char*, 4> motion2Name = { /* This list is generated from SDL_SensorType */
        {SDL_SENSOR_INVALID,"invalid"},
        {SDL_SENSOR_UNKNOWN,"unknown"},
        {SDL_SENSOR_ACCEL,"accel"},
        {SDL_SENSOR_GYRO,"gyro"}
    };
    // constexpr frozen::unordered_map<int, const char*, 1> touch2Name = { /* This list allows pushing the sensor name for the touchpad */
    //         {0,"touch"}
    // };

    constexpr std::array<char,10> touchId2Name = {'0','1','2','3','4','5','6','7','8','9'};

    const std::array<std::string,10> State2intArray = {
        "value",
        "duration",
        "timestamp",
        "state",
        "X",
        "Y",
        "Z",
        "touchpad",
        "finger",
        "pressure"
    };
    
    // std::unordered_map<std::string, int> State2int = {
    //     {"value",0},
    //     {"duration",1},
    //     {"timestamp",2},
    //     {"state",3},
    //     {"X",4},
    //     {"Y",5},
    //     {"Z",6},
    //     {"touchpad",7},
    //     {"finger",8},
    //     {"pressure",9}
    // };

    struct ControllerEvent {
        unsigned int controller; 
        unsigned int eventType; 
        int eventAction;
        std::string touchID;
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
    extern int polling_frequency;
    extern bool print_events;
    extern bool print_motion_data;

    template <typename InType, typename OutType>
    OutType mapRange(InType in, InType inMin, InType inMax, OutType outMin, OutType outMax) {
        if (outMin != outMax) {
            return static_cast<OutType>((in - inMin) * (outMax - outMin) / (inMax - inMin) + outMin);
        } else {
            return static_cast<OutType>(in);
        }
    }

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

    struct ControllerState {
        int value = 0; 
        int event_duration = 0; 
        Uint64 event_timestamp = 0;
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
            Controller() = default;
            Controller(int id, SDL_Gamepad* instance, int move_buffer_size);
            std::unordered_map<std::string, ControllerState> state;
            CircularBuffer<ControllerState> discrete_buffer;
            bool is_open = false;
            int id = -1;
            SDL_Gamepad* instance;
            
    };

    extern std::unordered_map<int, Controller> controllers;
    // inline constexpr frozen::unordered_map<int, const char*, 9> event2Name;
    // inline constexpr frozen::unordered_map<int, const char*, 23> button2Name;
    // inline constexpr frozen::unordered_map<int, const char*, 8> axis2Name;
    // inline constexpr frozen::unordered_map<int, const char*, 4> motion2Name;
    // inline constexpr frozen::unordered_map<int, const char*, 1> touch2Name;
    // extern std::unordered_map<std::string, int> State2int;
    //int state2int(std::string state);
    int state2int(const std::string& state);

    // Auxiliary functions and declarations
    extern std::int32_t elapsed_time;
    float clip(float n, float lower, float upper);
    int clip(int n, int lower, int upper);
    int nano2mili(Uint64 ns);
    std::string replaceID(std::string str, int newID);
    double applyDeadZone(double in, double in_min, double in_max, double out_min, double out_max, double dead_zone_min, double dead_zone_max, double dead_zone_value);
    int applyAnalogDeadZone(int in);
    bool isSensorChanged(int joy_index, int axis, int sensor);
};

#endif