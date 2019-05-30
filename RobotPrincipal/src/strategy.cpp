#include "strategy.h"
#include "uCListener.h"
#include "rplidar.h"
using namespace std;
#define PI 3.14159

using namespace rp::standalone::rplidar;

// Maximum motor speed and acceleration.
int maxSpeed = 200;
int maxAcceleration = 200;

// Motor current input value.
// These are the default value of the L6470.
const int MOTOR_KVAL_HOLD = 0x29;
const int MOTOR_BEMF[4] = {0x29, 0x0408, 0x19, 0x29};

double obtaintime()
{
    struct timespec currentTime;  
    clock_gettime(CLOCK_MONOTONIC, &currentTime);   
    return (double)currentTime.tv_sec + (double)(currentTime.tv_nsec)/1e9;
}


Strategy::Strategy()
{
    asservissement = Asservissement(0.,0., Angle());
}
void Strategy::mainLoop()
{

    RPlidarDriver* lidar = RPlidarDriver::CreateDriver();

            u_result res = lidar->connect("/dev/ttyUSB0", 115200);

    if (IS_OK(res))
    {
            lidar->startMotor();


while (true)
{
  if(asservissement.trajFinie() && idAction<1)
  {
      std::cout<<idAction<<std::endl;
    idAction++;
    delete(asservissement.traj);
    if(idAction==2)
      asservissement.traj = new Rotation(0., 0, Angle(0), Angle(PI/2), 2,2, obtaintime());
    if(idAction==1)
      asservissement.traj = new Droite(0.0, 0, 0.5, 0., 0.5,1, obtaintime());
    if(idAction==3)
      asservissement.traj = new Rotation(0.5, 0.5, Angle(PI/2), Angle(PI), 2,1, obtaintime());
    if(idAction==4)
      asservissement.traj = new Droite(0.5, 0.5, 0, 0.5, 0.5,1, obtaintime());
    if(idAction==5)
      asservissement.traj = new Rotation(0, 0.5, Angle(PI), Angle(-PI/2), 2,1, obtaintime());
    if(idAction==6)
      asservissement.traj = new Droite(0, 0.5, 0, 0, 0.5,1, obtaintime());
    if(idAction==7)
      asservissement.traj = new Rotation(0, 0, Angle(-PI/2), Angle(0), 2,1, obtaintime());
  }
  asservissement.actualise();
  usleep(100);

}
      lidar->stopMotor();
        lidar->disconnect();
    }
    else
    {
        
    }
        RPlidarDriver::DisposeDriver(lidar);

}
