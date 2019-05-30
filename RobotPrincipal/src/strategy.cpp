#include "strategy.h"
#include "uCListener.h"
#include "rplidar.h"
#include "rplidar_driver.h"
using namespace std;
#define PI 3.14159

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

    RPlidarDriver lidar = RPlidarDriver::CreateDriver(DRIVER_TYPE_SERIALPORT);

    u_result res = lidar->connect("/dev/ttyUSB0", 115200);
    lidar->startMotor();
    std::vector<RplidarScanMode> scanModes;
    lidar->getAllSupportedScanModes(scanModes);
    lidar->startScanExpress(false, scanModes[0].id);
    int compteur = 0;
    int distLimite = 300;
    int Nlimite = 20;

    while (true)
    {
        compteur = 0;
        rplidar_response_measurement_node_hq_t nodes[8192];
        size_t nodeCount = sizeof(nodes)/sizeof(rplidar_response_measurement_node_hq_t);
        res = lidar->grabScanDataHq(nodes, nodeCount);
        
        for(int i = 0; i < nodeCount; i++)
        {
            float ang = nodes[i].angle_z_q14* 90.f / (1 << 14); //On convertit en degre
            if(ang >= 325 && ang <= 25)
            {
                if(nodes[i].dist_mm_q2 <= distLimite)
                {
                    compteur++;
                }
            }
        }
        if(compteur > Nlimite)
        {
            std::cout<<"Obstacle"<<std::endl;
        }
        else{
            std::cout<<"Rien"<<std::endl;
        }
    }
    lidar->stopMotor();
    lidar->disconnect();
    RPlidarDriver::DisposeDriver(lidar);




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

}
