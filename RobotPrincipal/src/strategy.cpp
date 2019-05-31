#include "strategy.h"
#include "uCListener.h"
#define PI 3.141592653589793
#define KILLTIME 80
#define iVMAX 0.25
#define iACCMAX 0.25


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
    //action = actions();

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

Droite *Strategy::SimpleForward(double x_arr, double y_arr)
{
  double signe = -1;
  if(coteJaune)
    signe  = 1;
  return new Droite(asservissement.getX(), asservissement.getY(), x_arr, signe*y_arr, iVMAX, iACCMAX, obtaintime());
}

Droite *Strategy::SimpleBackward(double x_arr, double y_arr)
{
  double signe = -1;
  if(coteJaune)
    signe  = 1;
  return new Droite(asservissement.getX(), asservissement.getY(), x_arr, signe*y_arr, -iVMAX, iACCMAX, obtaintime());
}

Rotation *Strategy::SimpleRotation(double theta_arr)
{
  double signe = -1;
  if(coteJaune)
    signe  = 1;
  return new Rotation(asservissement.getX(), asservissement.getY(), Angle(asservissement.getTheta()), Angle(signe*theta_arr), 1,1, obtaintime());
}

void Strategy::beginTimer(bool isJaune)
{
tinitial = obtaintime();
coteJaune=isJaune;
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
    int distLimite = 1500;
    int Nlimite = 4;

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
            if(ang>=45 && ang <= 135)
            {
                if(nodes[i].dist_mm_q2 <= distLimite && nodes[i].dist_mm_q2>300)
                {
                    compteur++;
                }
            }
        }
        if(compteur > Nlimite)
        {
            //std::cout<<"Obstacle"<<std::endl;
            obstacles++;
            /*if (obstacles==4&&evitement&&asservissement.traj->TYPE==DROITE)
            {
targetX = asservissement.traj->getArriveeX();
targetY = asservissement.traj->getArriveeY();
std::cout<<"évitement"<<std::endl;
asservissement.stop();
delete(asservissement.traj);
      asservissement.traj = new Attente(100, obtaintime());
etaitDroite=true;
            }*/
        }
else
{
/*if(obstacles>3&&etaitDroite)
{

asservissement.traj = new Droite(asservissement.getX(), asservissement.getY(), targetX, targetY, iVMAX,iACCMAX, obtaintime());
}*/
etaitDroite=false;
 obstacles=0;
        }

  }
if(asservissement.trajFinie() && idAction<12)
{
    idAction++;
    delete(asservissement.traj);
   if(idAction==1)
   {
     //asservissement.traj = new Attente(100, obtaintime());
asservissement.traj = SimpleForward(0.7,0);
   }
   if(idAction==2)
   {
asservissement.traj = SimpleRotation(PI/2);
   }
      if(idAction==3)
   {
  asservissement.traj = SimpleForward(0.7,0.2);
   }
      if(idAction==4)
   {
  asservissement.traj = SimpleRotation(PI);
   }
      if(idAction==5)
   {
  asservissement.traj = SimpleForward(-0.1,0.2);
   }
    if(idAction==6)
   {
    asservissement.traj = SimpleBackward(0.1,0.2);
   }
    if(idAction==7)
   {
      asservissement.traj = SimpleRotation(PI/2);
   }
    if(idAction==8)
   {
    asservissement.traj = SimpleForward(0.1,0.4);
   }
    if(idAction==9)
   {
    asservissement.traj = SimpleRotation(PI);
   }
    if(idAction==10)
   {
        asservissement.traj = SimpleForward(-0.2,0.4);
   }
    if(idAction==11)
   {
    asservissement.traj = SimpleRotation(-PI/2);
   }
    if(idAction==12)
   {
        asservissement.traj = SimpleForward(-0.2,0);
   }
  }

//lidar->stopMotor();
  asservissement.actualise();
  //usleep(1);
}
    asservissement.stop();
if(lidarOK)
{
action.ventouseAvantOff();
action.ventouseArriereOff();
action.leverBras();
    lidar->stopMotor();
    lidar->disconnect();
    RPlidarDriver::DisposeDriver(lidar);
}
}
