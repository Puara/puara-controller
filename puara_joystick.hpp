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

#include <lo/lo.h>
#include <lo/lo_cpp.h>

#include <chrono>
#include <thread>

class PuaraJoystick {
    private:
        std::string OSC_namespace = "/puarajoystick";
        std::string full_namespace;
        std::int32_t elapsed_time;
        int osc_port = 8000;
        int osc_server_port = 8000;
        lo::ServerThread osc_server;
        //lo::Address a("localhost", "9000");
        bool sdl_quit = false;

        /* This list is created from SDL_GameControllerButton */
        std::string SDL_GameControllerButton_list[22] = {"A","B","X","Y","back","guide","start","leftstick","rightstick","leftshoulder","rightshoulder","dpad_up","dpad_down","dpad_left","dpad_right","misc1","paddle1","paddle2","paddle3","paddle4","touchpad","max"};
        /* This list is created from SDL_GameControllerAxis */
        std::string SDL_GameControllerAxis_list[7] = {"leftx","lefty","rightx","righty","triggerleft","triggerright","max"};
        /* This list is created from SDL_SensorType */
        std::string SDL_SensorType_list[3] = {"unknown","accel","gyro"};

        std::uint32_t last_button_event_time[sizeof(SDL_GameControllerButton_list)/sizeof(std::uint32_t)] = {0};

        std::vector<SDL_GameController*> controllers;

        float clip(float n, float lower, float upper);
        int clip(int n, int lower, int upper);

    public:
        PuaraJoystick(...);
        int openController(std::vector<SDL_GameController*> &controller_container, int joy_index);
        int openController(int joy_index);
        std::vector<SDL_GameController*> openAllControllers(std::vector<SDL_GameController*> &controller_container);
        int openAllControllers();
        void processSDLEvent(SDL_Event event);
        bool getQuit();
};

#endif