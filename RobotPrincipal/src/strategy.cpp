#include "strategy.h"
#include "uCListener.h"
#define PI 3.14159
#define KILLTIME 100
#define iVMAX 0.25
#define iACCMAX 1


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
    lidar = RPlidarDriver::CreateDriver();

    res = lidar->connect("/dev/ttyUSB0", 115200);

    if (IS_OK(res))
    {
    lidar->startMotor();
    lidarOK=true;
    }
    else
    {
      std::cout<<"Lidar échoué"<<std::endl;
      lidarOK=false;
    }

}

void Strategy::beginTimer()
{
tinitial = obtaintime();
}

void Strategy::mainLoop()
{
  if(lidarOK)
  {
  std::vector<RplidarScanMode> scanModes;
    lidar->getAllSupportedScanModes(scanModes);
    lidar->startScanExpress(false, scanModes[0].id);


  }
    int compteur = 0;
    int distLimite = 1200;
    int Nlimite = 8;

int obstacles=0;
while (obtaintime()-tinitial<KILLTIME)
{
compteur = 0;
  if(lidarOK)
  {
        rplidar_response_measurement_node_hq_t nodes[8192];
        size_t nodeCount = sizeof(nodes)/sizeof(rplidar_response_measurement_node_hq_t);
        res = lidar->grabScanDataHq(nodes, nodeCount);
        
        for(int i = 0; i < nodeCount; i++)
        {
            float ang = nodes[i].angle_z_q14* 90.f / (1 << 14); //On convertit en degre
            if(true)
            {
                if(nodes[i].dist_mm_q2 <= distLimite && nodes[i].dist_mm_q2>300)
                {
                    compteur++;
                }
            }
        }
        if(compteur > Nlimite)
        {
            std::cout<<"Obstacle"<<std::endl;
            obstacles++;
            if (obstacles>3)
            {
              //TODO coder un évitement
            }
        }
        else{
            obstacles=0;
        }

  }




  if(asservissement.trajFinie() && idAction<5)
  {
    idAction++;
    delete(asservissement.traj);
    if(idAction==1)
      asservissement.traj = new Droite(0.0, 0, 0.3, 0., iVMAX,iACCMAX, obtaintime());
    if(idAction==2)
      asservissement.traj = new Rotation(0.3, 0, Angle(0), Angle(PI/2), 2,2, obtaintime());
    if(idAction==3)
      asservissement.traj = new Droite(0.3, 0, 0.3, 0.3, iVMAX,iACCMAX, obtaintime());
    if(idAction==4)
      asservissement.traj = new Rotation(0.3, 0.3, Angle(PI/2), Angle(PI), 2,2, obtaintime());
    if(idAction==5)
      asservissement.traj = new Droite(0.3, 0.3, 0, 0.3, iVMAX,iACCMAX, obtaintime());
  }
  asservissement.actualise();
  usleep(1);

}
    asservissement.stop();
    lidar->stopMotor();
    lidar->disconnect();
    RPlidarDriver::DisposeDriver(lidar);

}

