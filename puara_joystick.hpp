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
                CircularBuffer() : CircularBuffer(10) {}; // Default constructor with default capacity
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
            std::unordered_map<int, int> button;
            std::unordered_map<int, int> last_button_event_duration;
            //int A, B, X, Y, back, guide, start, leftStick, rightStick, leftShoulder, rightShoulder, dpadUp, dpadDown, dpadLeft, dpadRight, misc1, paddle1, paddle2, paddle3, paddle4, touchpad, triggerLeft, triggerRight, maxBtn, maxAxis;
            struct Sensor {float X;float Y;float Z;};
            struct Axis {int X;int Y;};
            bool rumble;
            ControllerState () {
                Sensor accel, gyro;
                Axis analogLeft, analogRight;
            };
        };

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
                //std::vector<std::uint32_t> last_buttons_event_time;
        };

        bool sdl_quit = false;

        static std::unordered_map<std::string, std::unordered_map<int, std::string>> SDL2Name;
        //  = {
        //     {"button",{ /* This list is generated from SDL_GameControllerButton */
        //         {SDL_CONTROLLER_BUTTON_INVALID,"invalid"},
        //         {SDL_CONTROLLER_BUTTON_A,"A"},
        //         {SDL_CONTROLLER_BUTTON_B,"B"},
        //         {SDL_CONTROLLER_BUTTON_X,"X"},
        //         {SDL_CONTROLLER_BUTTON_Y,"Y"},
        //         {SDL_CONTROLLER_BUTTON_BACK,"back"},
        //         {SDL_CONTROLLER_BUTTON_GUIDE,"guide"},
        //         {SDL_CONTROLLER_BUTTON_START,"start"},
        //         {SDL_CONTROLLER_BUTTON_LEFTSTICK,"leftstick"},
        //         {SDL_CONTROLLER_BUTTON_RIGHTSTICK,"rightstick"},
        //         {SDL_CONTROLLER_BUTTON_LEFTSHOULDER,"leftshoulder"},
        //         {SDL_CONTROLLER_BUTTON_RIGHTSHOULDER,"rightshoulder"},
        //         {SDL_CONTROLLER_BUTTON_DPAD_UP,"dpad_up"},
        //         {SDL_CONTROLLER_BUTTON_DPAD_DOWN,"dpad_down"},
        //         {SDL_CONTROLLER_BUTTON_DPAD_LEFT,"dpad_left"},
        //         {SDL_CONTROLLER_BUTTON_DPAD_RIGHT,"dpad_right"},
        //         {SDL_CONTROLLER_BUTTON_MISC1, "misc1"},
        //         {SDL_CONTROLLER_BUTTON_PADDLE1,"paddle1"},
        //         {SDL_CONTROLLER_BUTTON_PADDLE2,"paddle2"},
        //         {SDL_CONTROLLER_BUTTON_PADDLE3,"paddle3"},
        //         {SDL_CONTROLLER_BUTTON_PADDLE4,"paddle4"},
        //         {SDL_CONTROLLER_BUTTON_TOUCHPAD,"touchpad"},
        //         {SDL_CONTROLLER_BUTTON_MAX,"max_btn"}
        //     }},
        //     {"axis",{ /* This list is generated from SDL_GameControllerAxis */
        //         {SDL_CONTROLLER_AXIS_INVALID,"invalid"},
        //         {SDL_CONTROLLER_AXIS_LEFTX,"leftx"},
        //         {SDL_CONTROLLER_AXIS_LEFTY,"lefty"},
        //         {SDL_CONTROLLER_AXIS_RIGHTX,"rightx"},
        //         {SDL_CONTROLLER_AXIS_RIGHTY,"righty"},
        //         {SDL_CONTROLLER_AXIS_TRIGGERLEFT,"triggerleft"},
        //         {SDL_CONTROLLER_AXIS_TRIGGERRIGHT,"triggerright"},
        //         {SDL_CONTROLLER_AXIS_MAX,"max_axis"}
        //     }},
        //     {"sensor",{ /* This list is generated from SDL_SensorType */
        //         {SDL_SENSOR_INVALID,"invalid"},
        //         {SDL_SENSOR_UNKNOWN,"unknown"},
        //         {SDL_SENSOR_ACCEL,"accel"},
        //         {SDL_SENSOR_GYRO,"gyro"}
        //     }},
        // };

        // /* This list is generated from SDL_GameControllerButton */
        // std::string SDL_GameControllerButton_list[22] = {"A","B","X","Y","back","guide","start","leftstick","rightstick","leftshoulder","rightshoulder","dpad_up","dpad_down","dpad_left","dpad_right","misc1","paddle1","paddle2","paddle3","paddle4","touchpad","max_btn"};
        // /* This list is generated from SDL_GameControllerAxis */
        // std::string SDL_GameControllerAxis_list[7] = {"leftx","lefty","rightx","righty","triggerleft","triggerright","max_axis"};
        // /* This list is generated from SDL_SensorType */
        // std::string SDL_SensorType_list[3] = {"unknown","accel","gyro"};

        //std::vector<std::uint32_t> last_button_event_time;

        //std::unordered_map<int, SDL_GameController*> controllers;
        std::unordered_map<int, Controller> controllers_temp;
        //std::unordered_map<int, ControllerState> controller_state;
        //std::unordered_map<int, CircularBuffer<ControllerState>> move_buffer;

        float clip(float n, float lower, float upper);
        int clip(int n, int lower, int upper);
        void libloServerMethods();
        int createControllerInstance(int joy_index);
};

#endif