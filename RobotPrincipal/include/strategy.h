#ifndef STRATEGY
#define STRATEGY

#include <iostream>
#include "MinotaureLib/L6470Driver.h"
#include <unistd.h>
#include "asservissement.h"
#include "rplidar.h"

using namespace rp::standalone::rplidar;    

class Strategy
    {
        public:
        Asservissement asservissement;
        Strategy();
        void beginTimer();
        void mainLoop();
        int idAction=0;
        double tinitial;
        RPlidarDriver* lidar;
        u_result res;
        bool lidarOK;
    };

#endif
