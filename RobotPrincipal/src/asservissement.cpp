#include "asservissement.h"
#include <cmath>
#include <iostream>

#define PI 3.14159

#define MAXSPEED 400

double gettime()
{
    struct timespec currentTime;
    clock_gettime(CLOCK_MONOTONIC, &currentTime);
    return (double)currentTime.tv_sec + (double)(currentTime.tv_nsec)/1e9;
}

Asservissement::Asservissement()
{

}

Asservissement::Asservissement(double m_xInit, double m_yInit, Angle m_thetaInit)
{
  std::cout<<"Démarrage du robot"<<std::endl;
    std::cout << "L6470 Stepper motor test." << std::endl;
    std::cout << "Requierments: X-NUCLEO-IHM02A1 wired to SPI port 0 of Raspberry Pi." << std::endl;

int maxAcceleration = 800;

// Motor current input value.
// These are the default value of the L6470.
const int MOTOR_KVAL_HOLD = 0x29;
const int MOTOR_BEMF[4] = {0x29, 0x0408, 0x19, 0x29};
    // Init connection with driver.
    stepperMotors = miam::L6470("/dev/spidev0.0", 2);
    bool areMotorsInit = stepperMotors.init(MAXSPEED, maxAcceleration, MOTOR_KVAL_HOLD, MOTOR_BEMF[0], MOTOR_BEMF[1], MOTOR_BEMF[2], MOTOR_BEMF[3], false);
    if(areMotorsInit)
        std::cout << "Connection with driver successful." << std::endl;
    else
    {
        std::cout << "Failed to communicate with L6470 board" << std::endl;
        //return;
    }


  erreurRot[0] = 0;
  erreurRot[1] = 0;
  erreurRot[2] = 0;

  erreurPos[0] = 0;
  erreurPos[1] = 0;
  erreurPos[2] = 0;

  x=m_xInit;
  y=m_yInit;
  theta=m_thetaInit;

  uCData donneesCapteur = uCListener_getData();
  incAvantD=donneesCapteur.encoderValues[0];
  incAvantG=donneesCapteur.encoderValues[1];
  tempsAvant = 0;
  traj = new Rotation(x, y, Angle(theta), Angle(theta), 0.5,0.5, gettime());
  //=====CONSTANTES ROBOT DEPENDANT=====
      K_INC = 0.00722;
      LARGEUR = 0.195;
      COEFF_ERREUR_ROT_P = 400;
      COEFF_ERREUR_ROT_I = 20;
      COEFF_ERREUR_ROT_D = 0;
      COEFF_ERREUR_POS_P = 2800;
      COEFF_ERREUR_POS_I = 100;
      COEFF_ERREUR_POS_D = 0;
}

Asservissement::~Asservissement()
{

}

void Asservissement::stop()
{
std::cout<<"stop moteur"<<std::endl;
std::vector<double> velocities;
    velocities.push_back(0);
    velocities.push_back(0);
    stepperMotors.setSpeed(velocities);
}

void Asservissement::actualise()
{
  //On integre la position du robot

  uCData donneesCapteur = uCListener_getData();
  double NowD = donneesCapteur.encoderValues[0];
  double NowG = donneesCapteur.encoderValues[1];
  x += cos(theta.versFloat())*(NowD+NowG-incAvantD-incAvantG)*K_INC/2;
  y += sin(theta.versFloat())*(NowD+NowG-incAvantD-incAvantG)*K_INC/2;
  theta = Angle(theta.versFloat() + atan((NowD-NowG-incAvantD+incAvantG)*K_INC/LARGEUR));
  incAvantD = NowD;
  incAvantG = NowG;


  if(traj->marcheArriere())
 {
    theta = theta+Angle(PI);
 }

  //On calcule l'erreur associee a cette position
  double erreurPosCour;
  double erreurRotCour;


  //Sinon, on met la consigne des moteurs a zero
  double temps=gettime();
  erreurPosCour = traj->erreurPos(x, y, theta, temps);
  erreurRotCour = traj->erreurRot(x, y, theta, temps);
  //Calcul du PID sur la trajectoire
  erreurPos[D] = (erreurPosCour-erreurPos[P])/((temps-tempsAvant));
  erreurPos[P] = erreurPosCour;
  erreurPos[I] += erreurPos[P]*(temps-tempsAvant);
  if(fabs(erreurPos[I])>KI_POS_SAT)
    erreurPos[I]=KI_POS_SAT*fabs(erreurPos[I])/erreurPos[I];

  erreurRot[D] = (erreurRotCour-erreurRot[P])/((temps-tempsAvant));
  erreurRot[P] = erreurRotCour;
  erreurRot[I] += erreurRot[P]*(temps-tempsAvant);
  if(fabs(erreurRot[I])>KI_ROT_SAT)
    erreurRot[I]=KI_ROT_SAT*fabs(erreurRot[I])/erreurRot[I];

  //Puis on asservit
  double consigneR = erreurRot[P]*COEFF_ERREUR_ROT_P+erreurRot[I]*COEFF_ERREUR_ROT_I+erreurRot[D]*COEFF_ERREUR_ROT_D;
  double consigneP = erreurPos[P]*COEFF_ERREUR_POS_P+erreurPos[I]*COEFF_ERREUR_POS_I+erreurPos[D]*COEFF_ERREUR_POS_D;

  if(!traj->marcheArriere())
  {
    double consigneD = +consigneP+consigneR;
    double consigneG = +consigneP-consigneR;
    if(fabs(consigneD)>MAXSPEED)
      consigneD=consigneD*MAXSPEED/fabs(consigneD);
    if(fabs(consigneG)>MAXSPEED)
      consigneG=consigneG*MAXSPEED/fabs(consigneG);
    std::vector<double> velocities;
    velocities.push_back(consigneG);
    velocities.push_back(consigneD);
    stepperMotors.setSpeed(velocities);
  }
  else
  {
    double consigneD = -consigneP+consigneR;
    double consigneG = -consigneP-consigneR;
    if(fabs(consigneD)>MAXSPEED)
      consigneD=consigneD*MAXSPEED/fabs(consigneD);
    if(fabs(consigneG)>MAXSPEED)
      consigneG=consigneG*MAXSPEED/fabs(consigneG);
    std::vector<double> velocities;
    velocities.push_back(consigneG);
    velocities.push_back(consigneD);
    stepperMotors.setSpeed(velocities);
    theta = theta+Angle(PI);
  }

  //On actualise les variables de memoire
  tempsAvant=temps;

}

double Asservissement::getX()
{
	return x;
}

double Asservissement::getY()
{
	return y;
}

Angle Asservissement::getTheta()
{
	return theta;
}

void Asservissement::nouvelleTrajectoire(Trajectoire *iTraj)
{
  delete(traj);
  traj = iTraj;
}

bool Asservissement::trajFinie()
{
  return traj->estFinie(gettime());
}
