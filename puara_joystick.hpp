//****************************************************************************//
// Puara Joystick module - connect with game controllers using SDL2 (hpp)     //
//                         Controller -> OSC/MIDI bridge                      //
// https://github.com/Puara/puara-joystick                                    //
// Metalab - Société des Arts Technologiques (SAT)                            //
// Input Devices and Music Interaction Laboratory (IDMIL), McGill University  //
// Edu Meneses (2023) - https://www.edumeneses.com                            //
//****************************************************************************//


#ifndef PUARA_JOYSTICK_H
#define PUARA_JOYSTICK_H


#include <SDL2/SDL.h>
#include <iostream>
#include <vector>
#include <unordered_map>

// #include <lo/lo.h>
// #include <lo/lo_cpp.h>

// #include <chrono>
// #include <thread>

class PuaraJoystick {
    public:
        PuaraJoystick();
        int openController(int joy_index);
        int openAllControllers();
        struct EventResume {int controller; unsigned int eventType; int eventAction;};
        EventResume pullSDLEvent(SDL_Event event);
        int rumble(int controllerID, int time, float low_freq, float hi_freq);
        void printEvent(EventResume eventResume);
        void printEvent(EventResume eventResume, bool printSensor);
        bool doQuit();
        std::string identifier = "puaracontroller";
        bool verbose = true;
        int move_buffer_size = 10;
    private:
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

        struct Sensor {float X; float Y; float Z;};
        struct Axis {int X; int Y;};

        struct ControllerState {
            std::unordered_map<int, int> button;
            //std::unordered_map<int, int> axis;
            std::unordered_map<int, int> last_button_event_duration;
            Sensor accel, gyro;
            Axis analogR, analogL;
            int triggerL, triggerR;
            bool rumble;
        };

        std::string full_namespace;
        std::int32_t elapsed_time;
        bool sdl_quit = false;
        static std::unordered_map<std::string, std::unordered_map<int, std::string>> SDL2Name;

        class Controller {
            public:
                Controller() = default;
                explicit Controller(int move_buffer_size) : buffer(move_buffer_size) {};
                Controller(int id, SDL_GameController* instance, int move_buffer_size);
            private:
                friend class PuaraJoystick;
                bool isOpen = false;
                int id;
                SDL_GameController* instance;
                ControllerState state;
                CircularBuffer<ControllerState> buffer;
        };

        std::unordered_map<int, Controller> controllers;

        float clip(float n, float lower, float upper);
        int clip(int n, int lower, int upper);
};

#endif