#ifndef TRAJECTOIRE
#define TRAJECTOIRE

#include "angle.h"

//Constantes de tableau
#define X 0
#define Y 1
#define TRIGO 1
#define A_TRIGO -1

enum TypeTrajectoire{INDEFINI, ROTATION, DROITE};

class Trajectoire
{
  protected:
    long int mDateDep, mDateAcc, mDateDec, mDateArr;
    boolean mMarcheArriere = false;

  public:
    const TypeTrajectoire TYPE;

  public:
    Trajectoire(TypeTrajectoire type = INDEFINI);
    virtual ~Trajectoire();

    virtual double erreurPos(double x, double y, Angle theta, long temps);
    virtual double erreurRot(double x, double y, Angle theta, long temps);

    void commence();
    bool estFinie(long temps);

    bool marcheArriere();
};

class Rotation : public Trajectoire
{
  private:
    double mVitesseMax, mSens, mAccMax;
    double mAngleAcc, mAngleConst;

    double mX, mY;
    Angle mThetaDep, mThetaArr;

  public:
    Rotation(double iX, double iY, Angle iThetaDep, Angle iThetaArr, double iVitesseMax, double iAccMax);
    virtual ~Rotation();

    double erreurPos(double iX, double iY, Angle iTheta, long iTemps);
    double erreurRot(double iX, double iY, Angle iTheta, long iTemps);
};

class Droite : public Trajectoire
{
  private:
    double mVitesseMax, mAccMax;
    double mVecteur[2];
    double mDistAcc, mDistConst;
    Angle mTheta;
    double mDepart[2];
    double mArrivee[2];

  public:
    Droite(double iXDepart, double iYDepart, double iXArrivee, double iYArrivee, double iVitesseMax, double iAccMax);
    virtual ~Droite();

    double erreurPos(double iX, double iY, Angle iTheta, long iTemps);
    double erreurRot(double iX, double iY, Angle iTheta, long iTemps);

    inline double getArriveeX(){return mArrivee[X];}
    inline double getArriveeY(){return mArrivee[Y];}
};

#endif
