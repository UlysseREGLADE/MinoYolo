#ifndef ASSERVISSEMENT
#define ASSERVISSEMENT

#include "trajectoire.h"
//#include "moteurs.h"
//#include "capteursInc.h"
//=====CONSTANTES NON ROBOT DEPENDANT=====

//Contantes d'asservissement
#define KI_ROT_SAT 0.05
#define KI_POS_SAT 0.05

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

    //Trajectoire courrante du robot
    Trajectoire* traj;

    //Temps au precedant tour de boucle
    long tempsAvant;
    int incAvantD, incAvantG;

    double K_INC;
    double LARGEUR;
    int COEFF_ERREUR_ROT_P;
    int COEFF_ERREUR_ROT_I;
    int COEFF_ERREUR_ROT_D;
    long COEFF_ERREUR_POS_P;
    long COEFF_ERREUR_POS_I;
    long COEFF_ERREUR_POS_D;

  public:
    Asservissement(double m_xInit, double m_yInit, Angle m_thetaInit, int const typeRobot);
    ~Asservissement();

    void init(double m_xInit, double m_yInit, Angle m_thetaInit);
    void actualise(long temps);
	  double getX();
	  double getY();
	  Angle getTheta();
    void nouvelleTrajectoire(Trajectoire *iTraj);
    bool trajFinie(long iTemps);

    inline Trajectoire const* getTrajectoireCourante(){return traj;}
};

#endif
