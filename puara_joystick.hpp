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

#include <lo/lo.h>
#include <lo/lo_cpp.h>

#include <chrono>
#include <thread>

class PuaraJoystick {
    public:
        PuaraJoystick(...);
        int openController(int joy_index);
        int openAllControllers();
        void pullSDLEvent(SDL_Event event);
        bool getQuit();
    private:
        template<typename T>
        class CircularBuffer {
            public:
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
        template <typename Key, typename Value>
        class ReversibleMap {
            public:
                void insert(const Key& key, const Value& value);
                const Value& operator[](const Key& key) const;
                const Key& reverseLookup(const Value& value) const;

            private:
                std::unordered_map<Key, Value> forwardMap;
                std::unordered_map<Value, Key> reverseMap;
        };


        std::string OSC_namespace = "/puarajoystick";
        bool verbose = true;
        std::string full_namespace;
        std::int32_t elapsed_time;
        int osc_port = 8000;
        int osc_server_port = 8000;
        lo::ServerThread osc_server;
        //lo::Address a("localhost", "9000");
        int move_buffer_size = 10;

        struct ControllerState {
            int A, B, X, Y, back, guide, start, leftStick, rightStick, leftShoulder, rightShoulder, dpadUp, dpadDown, dpadLeft, dpadRight, misc1, paddle1, paddle2, paddle3, paddle4, touchpad, triggerLeft, triggerRight, maxBtn, maxAxis;
            struct Sensor {float X;float Y;float Z;};
            struct Axis {int X;int Y;};
            bool rumble;
            ControllerState () {
                Sensor accel, gyro;
                Axis analogLeft, analogRight;
            };
        };

        class Controller {
            friend class PuaraJoystick;
            int id;
            SDL_GameController* instance;
            ControllerState state;
            CircularBuffer<ControllerState> buffer;
            Controller(int move_buffer_size) : buffer(move_buffer_size) {};
        };

        bool sdl_quit = false;

        /* This list is generated from SDL_GameControllerButton */
        std::string SDL_GameControllerButton_list[22] = {"A","B","X","Y","back","guide","start","leftstick","rightstick","leftshoulder","rightshoulder","dpad_up","dpad_down","dpad_left","dpad_right","misc1","paddle1","paddle2","paddle3","paddle4","touchpad","max_btn"};
        /* This list is generated from SDL_GameControllerAxis */
        std::string SDL_GameControllerAxis_list[7] = {"leftx","lefty","rightx","righty","triggerleft","triggerright","max_axis"};
        /* This list is generated from SDL_SensorType */
        std::string SDL_SensorType_list[3] = {"unknown","accel","gyro"};

        std::uint32_t last_button_event_time[sizeof(SDL_GameControllerButton_list)/sizeof(std::uint32_t)] = {0};

        //std::unordered_map<int, SDL_GameController*> controllers;
        std::unordered_map<int, Controller> controllers_temp;
        //std::unordered_map<int, ControllerState> controller_state;
        //std::unordered_map<int, CircularBuffer<ControllerState>> move_buffer;

        float clip(float n, float lower, float upper);
        int clip(int n, int lower, int upper);
        void libloServerMethods();
};

#endif