#ifndef STRATEGY
#define STRATEGY

#include <iostream>
#include "MinotaureLib/L6470Driver.h"
#include <unistd.h>
#include "asservissement.h"

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
    };

#endif