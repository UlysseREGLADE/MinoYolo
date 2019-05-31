#ifndef ASSERVISSEMENT
#define ASSERVISSEMENT

#include "trajectoire.h"
#include "uCListener.h"
#include "MinotaureLib/L6470Driver.h"
//=====CONSTANTES NON ROBOT DEPENDANT=====

//Contantes d'asservissement
#define KI_ROT_SAT 5
#define KI_POS_SAT 5

//Constantes de tableau
#define D 0
#define P 1
#define I 2

class Asservissement
{
  private:
    //Position et orientation du robot
    double x, y;
    Angle theta;

    //Variables des PIDs
    double erreurRot[3];
    double erreurPos[3];
    int consigneP, consigneR;

    //Temps au precedant tour de boucle
    double tempsAvant;
    double incAvantD, incAvantG;

    double K_INC;
    double LARGEUR;
    double COEFF_ERREUR_ROT_P;
    double COEFF_ERREUR_ROT_I;
    double COEFF_ERREUR_ROT_D;
    double COEFF_ERREUR_POS_P;
    double COEFF_ERREUR_POS_I;
    double COEFF_ERREUR_POS_D;

  public:
    Asservissement(double m_xInit, double m_yInit, Angle m_thetaInit);
    Asservissement();
    ~Asservissement();

    miam::L6470 stepperMotors;
    //void init(double m_xInit, double m_yInit, Angle m_thetaInit);
        //Trajectoire courrante du robot
    Trajectoire* traj;
    void actualise(bool iIsFreez);
	  double getX();
	  double getY();
	  Angle getTheta();
    void nouvelleTrajectoire(Trajectoire *iTraj);
    bool trajFinie();
    void stop();

    inline Trajectoire const* getTrajectoireCourante(){return traj;}
};

#endif
