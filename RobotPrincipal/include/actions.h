#include "pca9685.h"
#include <wiringPi.h>
#include<iostream>

class actions
{
public:
    actions();
    void baisserBras();
    void leverBras();
    void ventouseAvantOn();
    void ventouseAvantOff();
    void ventouseArriereOn();
    void ventouseArriereOff();//...
    ~actions();
};