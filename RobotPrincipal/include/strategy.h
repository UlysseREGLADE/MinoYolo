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
        bool etaitDroite=false;
        bool evitement=true;
        bool marcheArriere;
        bool evitementOn=false;
      	double targetX;
        double targetY;

        Droite *SimpleForward(double x_arr, double y_arr);
        Droite *SimpleBackward(double x_arr, double y_arr);
        Rotation *SimpleRotation(double theta_arr);
	Attente *SimpleAttente(double temps);
    };

#endif
