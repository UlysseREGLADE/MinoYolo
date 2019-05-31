#include "pca9685.h"
#include <wiringPi.h>
#include<iostream>

class actions
{
public:
    actions();
    void baisserBras();
    void leverBras();
    void claqueGauche();
    void claqueDroite();
    void ventouseAvantOn();
    void ventouseAvantOff();
    void ventouseArriereOn();
    void ventouseArriereOff();
void sortirVentouse();
void rentrerVentouse();
    ~actions();
};
