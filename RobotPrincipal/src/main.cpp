#include <iostream>
#include "strategy.h"
#include <unistd.h>
#include <sys/socket.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/rfcomm.h>
#include <wiringPi.h>
#include "uCListener.h"


int main(int argc, char **argv)
{
	wiringPiSetup();
    int pinTirette = 28;
    int pinCote = 27;
    pinMode(pinTirette, INPUT);
    pinMode(pinCote, INPUT);
    
    bool coteJaune = false;
    bool hasStarted = false;

    if(!uCListener_start("/dev/arduinoUno"))
    {
        std::cout<<"échec de démarrage de l'arduino"<<std::endl;
    }
        struct sockaddr_rc addr = { 0 };
    int s, status;
    char dest[18] = "00:14:03:06:58:1A"; //adresse du module bluetooth de dédale
    // allocate a socket
    s = socket(AF_BLUETOOTH, SOCK_STREAM, BTPROTO_RFCOMM);

    // set the connection parameters (who to connect to)
    addr.rc_family = AF_BLUETOOTH;
    addr.rc_channel = (uint8_t) 1;
    str2ba( dest, &addr.rc_bdaddr );
    // connect to server
        status = connect(s, (struct sockaddr *)&addr, sizeof(addr));

    Strategy strategy;

    while(!hasStarted)
    {
        if(digitalRead(pinTirette))
        {
            hasStarted=true;
            coteJaune = digitalRead(pinCote);
        }
        else
        usleep(1);
    }
    strategy.beginTimer(coteJaune);
    if(coteJaune)
    std::cout<<"cote jaune detecte"<<std::endl;
    else
    std::cout<<"cote violet detecte"<<std::endl;
        // send a message
        if(coteJaune)
            status = write(s, "piche", 6);  //Une chaine de charactere donne 247->côté jaune
        else
	 status = write(s, "1", 6);  //Un nombre donne 246->côté violet
            std::cout<<"writing"<<std::endl;

    close(s);
    /*bool hasStarted = false;
    while(!hasStarted)
    {
        usleep(10);
        hasStarted=true;
    }*/

    strategy.mainLoop();
    return 0;
}
