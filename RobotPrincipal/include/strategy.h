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
        void mainLoop();
        int idAction=0;
    };

#endif