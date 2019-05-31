#ifndef STRATEGY
#define STRATEGY

#include <iostream>
#include "MinotaureLib/L6470Driver.h"
#include <unistd.h>
#include "asservissement.h"
#include "rplidar.h"
#include "actions.h"

using namespace rp::standalone::rplidar;

class Strategy
    {
        public:
        Asservissement asservissement;
        Strategy();
        void beginTimer(bool isJaune);
        void mainLoop();
        int idAction=0;
        double tinitial;
        RPlidarDriver* lidar;
        u_result res;
        bool lidarOK;
        actions action;
        bool coteJaune;
      	double targetX;
        double targertY;
        bool evitement=false;

        Droite * SimpleForward(double x_arr, double y_arr);
        Droite * Strategy::SimpleBackward(double x_arr, double y_arr);
        Rotation * SimpleRotation(Angle theta_arr);
    };

#endif
