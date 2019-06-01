#include "strategy.h"
#include "uCListener.h"
#define PI 3.141592653589793
#define KILLTIME 88
#define iVMAX 0.28
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
  return new Rotation(asservissement.getX(), asservissement.getY(), Angle(asservissement.getTheta()), Angle(signe*theta_arr), 1.5,1, obtaintime());
}

Attente *Strategy::SimpleAttente(double temps)
{
 return new Attente(temps, obtaintime());
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
            if(ang>=35 && ang <= 145)
            {
                if(nodes[i].dist_mm_q2 <= distLimite && nodes[i].dist_mm_q2>100)
                {
                    compteur++;
                }
            }
        }
        if(compteur > Nlimite)
        {
            //std::cout<<"Obstacle"<<std::endl;
            obstacles++;
            if (obstacles==4&&evitement&&asservissement.traj->TYPE==DROITE)
            {
targetX = asservissement.traj->getArriveeX();
targetY = asservissement.traj->getArriveeY();
std::cout<<"évitement"<<std::endl;
asservissement.stop();
delete(asservissement.traj);
      asservissement.traj = new Attente(100, obtaintime());
etaitDroite=true;
            }
        }
else
{
if(obstacles>3&&etaitDroite)
{

asservissement.traj = new Droite(asservissement.getX(), asservissement.getY(), targetX, targetY, iVMAX,iACCMAX, obtaintime());
}
etaitDroite=false;
 obstacles=0;
        }

  }
  if(asservissement.trajFinie())
  {
    idAction++;
switch (idAction)
{
  case 1: 
  asservissement.traj = SimpleForward(0.65,0);
action.ventouseAvantOff();
action.ventouseArriereOff();
action.leverBras();
action.rentrerDroite();
action.rentrerVentouse();
  break;
case 2: 
  asservissement.traj = SimpleRotation(PI/2);
  break;
  case 3: 
  asservissement.traj = SimpleForward(0.65,0.2);
  break;
  case 4: 
  asservissement.traj = SimpleRotation(PI);
  break;
  case 5: 
  asservissement.traj = SimpleForward(0,0.2);
  break;
  case 6: 
    asservissement.traj = SimpleBackward(0.12,0.2);
  break;
  case 7: 
      asservissement.traj = SimpleRotation(PI/2);
  break;
  case 8: 
    asservissement.traj = SimpleForward(0.12,0.5);
  break;
  case 9: 
    asservissement.traj = SimpleRotation(PI);
  break;
  case 10: 
        asservissement.traj = SimpleForward(-0.2,0.5);
  break;
  case 11: 
    asservissement.traj = SimpleRotation(-PI/2);
  break;
  case 12: 
        asservissement.traj = SimpleForward(-0.2,0.1  );
  break;
    case 13: 
        asservissement.traj = SimpleBackward(-0.2,1.38);
  break;
    case 14: 
  asservissement.traj = SimpleRotation(PI);
  break;
      case 15: 
  asservissement.traj = SimpleForward(-0.395,1.38);
    action.pencherBras();
      action.claqueDroite();
  break;
        case 16: 
  asservissement.traj = SimpleAttente(0.25);
  action.claqueGauche();
  break;
  case 17: 
  asservissement.traj = SimpleBackward(-0.2,1.38);
  action.leverBras();
  break;
    case 18: 
  asservissement.traj = SimpleRotation(PI/2);
  break;
    case 19: 
  asservissement.traj = SimpleForward(-0.2,1.91);
  break;
      case 20: 
  asservissement.traj = SimpleRotation(0);
  break;
      case 21: 
  asservissement.traj = SimpleBackward(-0.565,1.91);
  action.ventouseArriereOn();
  break;
    case 22: 
  asservissement.traj = SimpleAttente(0.5);
  action.sortirVentouse();
  break;
  case 23: 
  asservissement.traj = SimpleAttente(0.5);
  action.rentrerVentouse();
  break;
    case 24:
    asservissement.traj = SimpleForward(-0.36,1.91);
  break;
  case 25:
    asservissement.traj = SimpleRotation(-PI/4);
  break;
    case 26:
    asservissement.traj = SimpleForward(0.57,1.04);
  break;
  case 27:
    asservissement.traj = SimpleRotation(PI);
  break;
    case 28:
    asservissement.traj = SimpleBackward(0.7, 1.04);
  break;
      case 29:
    asservissement.traj = SimpleAttente(1);
    action.ventouseArriereOff();
  break;
default:
    asservissement.stop();
if(lidarOK)
{
action.ventouseAvantOff();
action.ventouseArriereOff();
action.leverBras();
action.rentrerDroite();
action.rentrerVentouse();
    lidar->stopMotor();
    lidar->disconnect();
    RPlidarDriver::DisposeDriver(lidar);
    std::exit(0);
}
  break;
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
