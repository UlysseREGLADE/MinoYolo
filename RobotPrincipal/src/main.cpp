#include <iostream>
#include "strategy.h"
#include <unistd.h>
#include "uCListener.h"

Strategy strategy;

double getTime()
{
    struct timespec currentTime;  
    clock_gettime(CLOCK_MONOTONIC, &currentTime);   
    return (double)currentTime.tv_sec + (double)(currentTime.tv_nsec)/1e9;
}


int main(int argc, char **argv)
{
    bool hasStarted = false;
    //RPi_enablePorts();
    while(!hasStarted)
    {
        usleep(10000);
        //ici coder l'interrupteur
        hasStarted=true;
    }
    if(!uCListener_start("/dev/arduinoUno"))
    {
        std::cout<<"échec de démarrage de l'arduino"<<std::endl;
    }
    while (true)
    {
        std::cout<<getTime()<<std::endl;
    }
    
    uCData test;
    test = uCListener_getData();
    //std::cout<<test.encoderValues[0]<<std::endl;
    strategy.mainLoop();
    return 0;
}