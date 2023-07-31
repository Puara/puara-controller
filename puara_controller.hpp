//****************************************************************************//
// Puara Joystick module - connect with game controllers using SDL2 (hpp)     //
//                         Controller -> OSC/MIDI bridge                      //
// https://github.com/Puara/puara-joystick                                    //
// Metalab - Société des Arts Technologiques (SAT)                            //
// Input Devices and Music Interaction Laboratory (IDMIL), McGill University  //
// Edu Meneses (2023) - https://www.edumeneses.com                            //
//****************************************************************************//


#ifndef PUARA_CONTROLLER_H
#define PUARA_CONTROLLER_H


#include <SDL2/SDL.h>
#include <iostream>
#include <vector>
#include <unordered_map>

// #include <lo/lo.h>
// #include <lo/lo_cpp.h>

// #include <chrono>
// #include <thread>

class PuaraController {
    public:
        PuaraController();
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
        int analogDeadZone = 128;
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

        struct Button {
            int value = 0; 
            int event_duration = 0; 
            long int event_timestamp = 0;
        };
        struct Trigger {
            bool state = false; 
            int value = 0; 
            int event_duration = 0; 
            long int event_timestamp = 0;
        };
        struct Sensor {
            float X = 0;
            float Y = 0;
            float Z = 0;
            int event_duration = 0; 
            long int event_timestamp = 0;
        };
        struct Axis {
            bool state = false; 
            int X = 0; 
            int Y = 0; 
            int event_duration = 0; 
            long int event_timestamp = 0;
        };

        struct ControllerState {
            std::unordered_map<int, Button> button;
            Sensor accel, gyro;
            Axis analogR, analogL;
            Trigger triggerL, triggerR;
            bool rumble;
        };

        std::string full_namespace_;
        std::int32_t elapsed_time_;
        bool sdl_quit_ = false;
        static std::unordered_map<std::string, std::unordered_map<int, std::string>> SDL2Name_;

        class Controller {
            public:
                Controller() = default;
                explicit Controller(int move_buffer_size) : buffer(move_buffer_size) {};
                Controller(int id, SDL_GameController* instance, int move_buffer_size);
            private:
                friend class PuaraController;
                bool isOpen = false;
                int id;
                SDL_GameController* instance;
                ControllerState state;
                CircularBuffer<ControllerState> buffer;
        };

        std::unordered_map<int, Controller> controllers_;

        float clip_(float n, float lower, float upper);
        int clip_(int n, int lower, int upper);
        double applyDeadZone_(double in, double in_min, double in_max, double out_min, double out_max, double dead_zone_min, double dead_zone_max, double dead_zone_value);
        int applyAnalogDeadZone_(int in);
        bool isTriggerLinUse_(int joy_index);
        bool isSensorChanged_(int joy_index, std::string sensor, std::string side);
};

#endif