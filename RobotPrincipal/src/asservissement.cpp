#include "asservissement.h"
#include <cmath>
#include <iostream>

#define PI 3.15159

#define ROBOT_PRINCIPAL 0
#define ROBOT_SECONDAIRE 1
#define ROBOT_TEST 2
#define MAXSPEED 200

Asservissement::Asservissement()
{
std::cout<<"vide"<<std::endl;
  
}

Asservissement::Asservissement(double m_xInit, double m_yInit, Angle m_thetaInit)
{
  std::cout<<"Démarrage du robot"<<std::endl;
    std::cout << "L6470 Stepper motor test." << std::endl;
    std::cout << "Requierments: X-NUCLEO-IHM02A1 wired to SPI port 0 of Raspberry Pi." << std::endl;
int maxAcceleration = 200;

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
  incAvantD=donneesCapteur.encoderValues[0]; //TODO : vérifier 0 et 1 = D G
  incAvantG=donneesCapteur.encoderValues[1];
  tempsAvant = 0;
  traj = new Rotation(x, y, Angle(theta), Angle(theta), 0.5,0.5);
  traj ->commence();
  //=====CONSTANTES ROBOT DEPENDANT=====
      K_INC = 0.0000905052;
      LARGEUR = 0.206;
      COEFF_ERREUR_ROT_P = 1200;
      COEFF_ERREUR_ROT_I = 800;
      COEFF_ERREUR_ROT_D = 70;
      COEFF_ERREUR_POS_P = 12000;
      COEFF_ERREUR_POS_I = 30000;
      COEFF_ERREUR_POS_D = 2000;
}

Asservissement::~Asservissement()
{
  
}

/*void Asservissement::init(double m_xInit, double m_yInit, Angle m_thetaInit)
{
  x=m_xInit;
  y=m_yInit;
  theta=m_thetaInit;

  erreurRot[0] = 0;
  erreurRot[1] = 0;
  erreurRot[2] = 0;
  
  erreurPos[0] = 0;
  erreurPos[1] = 0;
  erreurPos[2] = 0;

  incAvantD=0;
  incAvantG=0;
std::cout << "Failed to communicate with L6470 board" << std::endl;
std::cout << "Failed to communicate with L6470 board" << std::endl;
std::cout << "Failed to communicate with L6470 board" << std::endl;
std::cout << "Failed to communicate with L6470 board" << std::endl;
std::cout << "Failed to communicate with L6470 board" << std::endl;
std::cout << "Failed to communicate with L6470 board" << std::endl;
  nouvelleTrajectoire(new Rotation(x, y, Angle(theta), Angle(theta), 0.5,0.5));
}*/

void Asservissement::actualise(double temps)
{
  //On integre la position du robot
  
  uCData donneesCapteur = uCListener_getData();
  double NowD = donneesCapteur.encoderValues[0]; //TODO : vérifier 0 et 1 = D G
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
    double consigneD = +consigneP-consigneR; //TODO vérifier D G
    double consigneG = +consigneP+consigneR;
    if(fabs(consigneD)>MAXSPEED)
      consigneD=consigneD*MAXSPEED/fabs(consigneD);
    if(fabs(consigneG)>MAXSPEED)
      consigneG=consigneG*MAXSPEED/fabs(consigneG);
    std::vector<double> velocities;
    velocities.push_back(+consigneD);
    velocities.push_back(+consigneG);
    stepperMotors.setSpeed(velocities);
  }
  else
  {
    double consigneD = -consigneP-consigneR; //TODO vérifier D G
    double consigneG = -consigneP+consigneR;
    if(fabs(consigneD)>MAXSPEED)
      consigneD=consigneD*MAXSPEED/fabs(consigneD);
    if(fabs(consigneG)>MAXSPEED)
      consigneG=consigneG*MAXSPEED/fabs(consigneG);
    std::vector<double> velocities;
    velocities.push_back(+consigneD);
    velocities.push_back(+consigneG);
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
  traj->commence();
}

bool Asservissement::trajFinie(double iTemps)
{
  return traj->estFinie(iTemps);
}

