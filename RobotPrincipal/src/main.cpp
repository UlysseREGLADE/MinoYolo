#include <iostream>
#include "strategy.h"
#include <unistd.h>
#include <wiringPi.h>



#include "uCListener.h"


int main(int argc, char **argv)
{
    int pinTirette = 28;
    int pinCote = 27;
    pinMode(pinTirette, INPUT);
    pinMode(pinCote, INPUT);
    bool hasStarted = false;
    bool coteJaune = false;
    //RPi_enablePorts();
    while(!hasStarted)
    {
        if(digitalRead(pinTirette))
        {
            hasStarted=true;
            coteJaune = digitalRead(pinCote);
        }
        //ici coder l'interrupteur
        usleep(10);
    }
    if(!uCListener_start("/dev/arduinoUno"))
    {
        std::cout<<"échec de démarrage de l'arduino"<<std::endl;
    }
    uCData test;
    test = uCListener_getData();



    
    //std::cout<<test.encoderValues[0]<<std::endl;
    Strategy strategy;
    strategy.mainLoop();
    return 0;
}