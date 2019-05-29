#include "strategy.h"

using namespace std;


// Maximum motor speed and acceleration.
int maxSpeed = 200;
int maxAcceleration = 200;

// Motor current input value.
// These are the default value of the L6470.
const int MOTOR_KVAL_HOLD = 0x29;
const int MOTOR_BEMF[4] = {0x29, 0x0408, 0x19, 0x29};

void Strategy::mainLoop()
{
    cout<<"Démarrage du robot"<<endl;
    std::cout << "L6470 Stepper motor test." << std::endl;
    std::cout << "Requierments: X-NUCLEO-IHM02A1 wired to SPI port 0 of Raspberry Pi." << std::endl;

    // Init connection with driver.
    miam::L6470 stepperMotors("/dev/spidev0.0", 2);
    bool areMotorsInit = stepperMotors.init(maxSpeed, maxAcceleration, MOTOR_KVAL_HOLD, MOTOR_BEMF[0], MOTOR_BEMF[1], MOTOR_BEMF[2], MOTOR_BEMF[3], false);
    if(areMotorsInit)
        std::cout << "Connection with driver successful." << std::endl;
    else
    {
        std::cout << "Failed to communicate with L6470 board" << std::endl;
        return;
    }

    std::cout << "Move both motors clockwise." << std::endl;
    std::vector<int> positions;
    positions.push_back(200);
    positions.push_back(200);
    stepperMotors.moveNSteps(positions);
    while(stepperMotors.isBusy())
        usleep(50000);
    std::cout << "Move motor 1 anti-clockwise." << std::endl;
    positions[1] = -200;
    stepperMotors.moveNSteps(positions);
    while(stepperMotors.isBusy())
        usleep(50000);

    std::cout << "Move both motors at constant velocity." << std::endl;
    std::vector<double> velocities;
    velocities.push_back(maxSpeed);
    velocities.push_back(maxSpeed);
    stepperMotors.setSpeed(velocities);
    usleep(5000000);
    stepperMotors.softStop();

}