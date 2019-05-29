#include "Arduino.h"
#include "asservissement.h"
#include "ecran.h" 

#define ROBOT_PRINCIPAL 0
#define ROBOT_SECONDAIRE 1
#define ROBOT_TEST 2

Asservissement::Asservissement(double m_xInit, double m_yInit, Angle m_thetaInit, int const typeRobot)
{
  erreurRot[0] = 0;
  erreurRot[1] = 0;
  erreurRot[2] = 0;
  
  erreurPos[0] = 0;
  erreurPos[1] = 0;
  erreurPos[2] = 0;

  x=m_xInit;
  y=m_yInit;
  theta=m_thetaInit;

  incAvantD=0;
  incAvantG=0;

  tempsAvant = 0;
  CapteursInc::init();
  Moteurs::init();

  //=====CONSTANTES ROBOT DEPENDANT=====
  switch(typeRobot)
  {
    case ROBOT_PRINCIPAL :
      K_INC = 0.0000905052;
      LARGEUR = 0.206;
      COEFF_ERREUR_ROT_P = 1200;
      COEFF_ERREUR_ROT_I = 800;
      COEFF_ERREUR_ROT_D = 70;
      COEFF_ERREUR_POS_P = 12000;
      COEFF_ERREUR_POS_I = 30000;
      COEFF_ERREUR_POS_D = 2000;
      Serial.println("Constantes robot principal");
      break;
    case ROBOT_SECONDAIRE :
      K_INC = 0.0000905052;
      LARGEUR = 0.220 ;
      COEFF_ERREUR_ROT_P = 1200;
      COEFF_ERREUR_ROT_I = 800;
      COEFF_ERREUR_ROT_D = 70;
      COEFF_ERREUR_POS_P = 12000;
      COEFF_ERREUR_POS_I = 3000;
      COEFF_ERREUR_POS_D = 1300;
      Serial.println("Constantes robot secondaire");
      break;
    case ROBOT_TEST :
      K_INC = 0.000268;
      LARGEUR = 0.315;
      COEFF_ERREUR_ROT_P = 600;
      COEFF_ERREUR_ROT_I = 400;
      COEFF_ERREUR_ROT_D = 70;
      COEFF_ERREUR_POS_P = 5000;
      COEFF_ERREUR_POS_I = 1000;
      COEFF_ERREUR_POS_D = 600;
      Serial.println("Constantes robot test");
      break;
    default :
      Serial.println("Grosse merde");
      break;
  }
}

Asservissement::~Asservissement()
{
  
}

void Asservissement::init(double m_xInit, double m_yInit, Angle m_thetaInit)
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

  CapteursInc::d = 0;
  CapteursInc::g = 0;

  tempsAvant = 0;

  Serial3.println("Théorie : x " + String(x) + " y " + String(y));

  nouvelleTrajectoire(new Rotation(x, y, Angle(theta), Angle(theta), 0.5,0.5));
}

void Asservissement::actualise(long temps)
{
  //Serial.println("g " + String(CapteursInc::g) + " d " + String(CapteursInc::d));
  //On integre la position du robot
  
  int NowD = CapteursInc::d;
  int NowG = CapteursInc::g;
  x += cos(theta.versFloat())*(NowD+NowG-incAvantD-incAvantG)*K_INC/2;
  y += sin(theta.versFloat())*(NowD+NowG-incAvantD-incAvantG)*K_INC/2;
  theta = Angle(theta.versFloat() + atan((NowD-NowG-incAvantD+incAvantG)*K_INC/LARGEUR));
  incAvantD = NowD;
  incAvantG = NowG;


/*  
  int capteurD = CapteursInc::d;
  int capteurG = CapteursInc::g;
  
  double d_theta = (capteurD-capteurG-incAvantD+incAvantG)*K_INC/LARGEUR;
  if(d_theta != 0)
  {
    double rayon = (LARGEUR/2)*(capteurD+capteurG-incAvantD-incAvantG)/(capteurD-capteurG-incAvantD+incAvantG);
    double d_xx = cos(d_theta) - 1;
    if(abs(capteurD-incAvantD) < abs(capteurG-incAvantG))
      d_xx = -d_xx;
    x += cos(theta.versFloat())*rayon*sin(d_theta) - sin(theta.versFloat())*rayon*d_xx;
    y += sin(theta.versFloat())*rayon*sin(d_theta) + cos(theta.versFloat())*rayon*d_xx;
    theta = Angle(theta.versFloat()) + d_theta;
  }
  else
  {
    x += cos(theta.versFloat())*(capteurD+capteurG-incAvantD-incAvantG)*K_INC/2;
    y += sin(theta.versFloat())*(capteurD+capteurG-incAvantD-incAvantG)*K_INC/2;
  }
*/
  
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

  //Serial3.println("x " + String(x) + " y " + String(y) + " t " + String(theta.versFloat()/PI*180) + " ep " + String(erreurPosCour) + " er " + String(erreurRotCour));

  //Serial.println(erreurRotCour);

  //Calcul du PID sur la trajectoire
  erreurPos[D] = (erreurPosCour-erreurPos[P])/((temps-tempsAvant)*0.000001);
  erreurPos[P] = erreurPosCour;
  erreurPos[I] += erreurPos[P]*(temps-tempsAvant)*0.000001;
  if(abs(erreurPos[I])>KI_POS_SAT)
    erreurPos[I]=KI_POS_SAT*abs(erreurPos[I])/erreurPos[I];

  erreurRot[D] = (erreurRotCour-erreurRot[P])/((temps-tempsAvant)*0.000001);
  erreurRot[P] = erreurRotCour;
  erreurRot[I] += erreurRot[P]*(temps-tempsAvant)*0.000001;
  if(abs(erreurRot[I])>KI_ROT_SAT)
    erreurRot[I]=KI_ROT_SAT*abs(erreurRot[I])/erreurRot[I];

  //Puis on asservit
  int consigneR = erreurRot[P]*COEFF_ERREUR_ROT_P+erreurRot[I]*COEFF_ERREUR_ROT_I+erreurRot[D]*COEFF_ERREUR_ROT_D;
  int consigneP = erreurPos[P]*COEFF_ERREUR_POS_P+erreurPos[I]*COEFF_ERREUR_POS_I+erreurPos[D]*COEFF_ERREUR_POS_D;
 
  if(!traj->marcheArriere())
  {
    Moteurs::avancent(+consigneP-consigneR, +consigneP+consigneR);
  }
  else
  {
    Moteurs::avancent(-consigneP-consigneR, -consigneP+consigneR);
    theta = theta+Angle(PI);
  }

  //On actualise les variables de memoire
  tempsAvant=temps;

  
  /*Serial.print("X:");
  Serial.println(x);

  Serial.print("Y:");
  Serial.println(y);

  Serial.print("thheta:");
  Serial.println(theta.versFloat());

  Serial.print("erreurPosCour:");
  Serial.println(erreurPosCour);

   Serial.print("erreurRotCour:");
  Serial.println(erreurRotCour);
  */
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

bool Asservissement::trajFinie(long iTemps)
{
  return traj->estFinie(iTemps);
}

