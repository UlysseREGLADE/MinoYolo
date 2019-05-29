#include <iostream>
#include "strategy.h"
#include <unistd.h>

Strategy strategy;

int main(int argc, char **argv)
{
    bool done = false;
    bool hasStarted = false;
    //RPi_enablePorts();
    while(!hasStarted)
    {
        usleep(10000);
        //ici coder l'interrupteur
        hasStarted=true;
    }
    strategy.mainLoop();
    return 0;
}