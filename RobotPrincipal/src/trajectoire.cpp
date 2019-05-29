#include "trajectoire.h"
#include <cmath>
#define PI 3.15159


Trajectoire::Trajectoire(TypeTrajectoire type):TYPE(type){mDateDep=0;}
Trajectoire::~Trajectoire(){}
double Trajectoire::erreurPos(double iX, double iY, Angle iTheta, long iTemps){return 0;}
double Trajectoire::erreurRot(double iX, double iY, Angle iTheta, long iTemps){return 0;}
bool Trajectoire::marcheArriere(){return mMarcheArriere;}

void Trajectoire::commence()
{
	mDateDep=0;
}

bool Trajectoire::estFinie(long iTemps)
{
  iTemps = iTemps-mDateDep;
  if(iTemps > mDateArr || mDateArr == 0)
  {
    return true;
  }
  else
    return false;
}


Rotation::Rotation(double iX, double iY, Angle iThetaDep, Angle iThetaArr, double iVitesseMax, double iAccMax):Trajectoire(ROTATION)
{
  mX = iX;
  mY = iY;
  mThetaDep = iThetaDep;
  mThetaArr = iThetaArr;
  mVitesseMax = iVitesseMax;
  mAccMax = iAccMax;
  if((mThetaArr-mThetaDep).versFloat()<0)
    mSens = A_TRIGO;
  else
    mSens = TRIGO;

  double dureeAcc = mVitesseMax/mAccMax;
  double angleAcc = 0.5*mAccMax*pow(dureeAcc, 2);
  double angleTot = abs((mThetaArr-mThetaDep).versFloat());

  if(2*angleAcc>angleTot)
  {
    angleAcc = angleTot/2;
    dureeAcc = sqrt(2*angleAcc/mAccMax);

    mAngleAcc = angleAcc;
    mAngleConst = 0;
    mDateAcc = 1000000*dureeAcc;
    mDateDec = mDateAcc + 0;
    mDateArr = mDateDec + 1000000*dureeAcc;
  }
  else
  {
    mAngleAcc = angleAcc;
    mAngleConst = angleTot-2*mAngleAcc;
    mDateAcc = 1000000*dureeAcc;
    mDateDec = mDateAcc + 1000000*mAngleConst/mVitesseMax;
    mDateArr = mDateDec + 1000000*dureeAcc;
  }
}

Rotation::~Rotation(){}

double Rotation::erreurPos(double iX, double iY, Angle iTheta, long iTemps)
{
  double vecteur[2] = {cos(iTheta.versFloat()), sin(iTheta.versFloat())};
  double projX = (mX-iX)*vecteur[X]+(mY-iY)*vecteur[Y];
  return projX;
}

double Rotation::erreurRot(double iX, double iY, Angle iTheta, long iTemps)
{
  iTemps = iTemps-mDateDep;
  if(iTemps<0)
  {
    return (mThetaDep-iTheta).versFloat();
  }
  else if(0<iTemps && iTemps<mDateAcc)
  {
    Angle theta = mThetaDep + 0.5*mSens*mAccMax*pow((iTemps)*0.000001, 2);
    return (theta-iTheta).versFloat();
  }
  else if(mDateAcc<iTemps && iTemps<mDateDec)
  {
    Angle theta = mThetaDep + mSens*(mAngleAcc + (iTemps-mDateAcc)*0.000001*mVitesseMax);
    return (theta-iTheta).versFloat();
  }
  else if(mDateDec<iTemps && iTemps<mDateArr)
  {
    Angle theta = mThetaArr - 0.5*mSens*mAccMax*pow((mDateArr-iTemps)*0.000001, 2);
    return (theta-iTheta).versFloat();
  }
  else
  {
    return (mThetaArr-iTheta).versFloat();
  }
}




Droite::Droite(double iXDepart, double iYDepart, double iXArrivee, double iYArrivee,  double iVitesseMax, double iAccMax):Trajectoire(DROITE)
{
  mVitesseMax = iVitesseMax;
  mAccMax = iAccMax;
  mDateDep =  0;
  mDepart[X] = iXDepart;
  mDepart[Y] = iYDepart;
  mArrivee[X] = iXArrivee;
  mArrivee[Y] = iYArrivee;

  if(mVitesseMax < 0)
  {
    mVitesseMax = -mVitesseMax;
    mMarcheArriere = true;
  }

  double dist = sqrt(pow(mDepart[X]-mArrivee[X], 2)+pow(mDepart[Y]-mArrivee[Y], 2));

  double dureeAcc = mVitesseMax/mAccMax;
  double distAcc = 0.5*pow(dureeAcc, 2)*mAccMax;

  if(dist==0)
  {
    mVecteur[X]=1;
    mVecteur[Y]=0;
    mTheta = 0;
  }
  else
  {
    mVecteur[X] = (mArrivee[X]-mDepart[X])/dist;
    mVecteur[Y] = (mArrivee[Y]-mDepart[Y])/dist;
    mTheta = acos(mVecteur[X]);
    if(mVecteur[Y]<0)
      mTheta = -mTheta.versFloat()+2*PI;

    if(2*distAcc>dist)
    {
      mDistAcc = dist/2;
      mDistConst=0;
      dureeAcc = sqrt(2*mDistAcc/mAccMax);
      mDateAcc = mDateDep + 1000000*dureeAcc;
      mDateDec = mDateAcc + 0;
      mDateArr = mDateDec + 1000000*dureeAcc;
    }
    else
    {
      mDistAcc = distAcc;
      mDistConst = dist - 2*distAcc;
      mDateAcc = mDateDep + 1000000*dureeAcc;
      mDateDec = mDateAcc + 1000000*mDistConst/mVitesseMax;
      mDateArr = mDateDec + 1000000*dureeAcc;
    }
  }
}

Droite::~Droite(){}

double Droite::erreurPos(double iX, double iY, Angle iTheta, long iTemps)
{
  iTemps = iTemps - mDateDep;
  //On se place dans la base ou le vecteur x colineaire a la trajectoire
  double projeteX = (iX-mDepart[X])*mVecteur[X]+(iY-mDepart[Y])*mVecteur[Y];

  if(iTemps<0)
  {
    double vecteurX = cos(iTheta.versFloat());
    double vecteurY = sin(iTheta.versFloat());
    projeteX = (mDepart[X]-iX)*vecteurX+(mDepart[Y]-iY)*vecteurY;
    return projeteX;
  }
  else if(0<iTemps && iTemps<mDateAcc)
  {
    double x = 0.5*mAccMax*pow((iTemps)*0.000001, 2);
    return x-projeteX;
  }
  else if(mDateAcc<iTemps && iTemps<mDateDec)
  {
    double x = mDistAcc + (iTemps-mDateAcc)*0.000001*mVitesseMax;
    return x-projeteX;
  }
  else if(mDateDec<iTemps && iTemps<mDateArr)
  {
    double x = 2*mDistAcc+mDistConst - 0.5*mAccMax*pow((mDateArr-iTemps)*0.000001, 2);
    return x-projeteX;
  }
  else
  {
    double vecteurX = cos(iTheta.versFloat());
    double vecteurY = sin(iTheta.versFloat());
    projeteX = (mArrivee[X]-iX)*vecteurX+(mArrivee[Y]-iY)*vecteurY;
    return projeteX;
  }
}

double Droite::erreurRot(double iX, double iY, Angle iTheta, long iTemps)
{
  iTemps = iTemps - mDateDep;
  //La, c'est une simple operation de projection sur la droite de la trajectoire
  double projeteY = -(iX-mDepart[X])*mVecteur[Y]+(iY-mDepart[Y])*mVecteur[X];
  //return 0;

  if(iTemps<0)
  {
    return (mTheta-iTheta).versFloat();
  }
  else if(0<iTemps && iTemps<mDateAcc)
  {
    return (Angle(-projeteY)-(iTheta-mTheta)).versFloat()*iTemps*0.000001*mAccMax*10;
  }
  else if(mDateAcc<iTemps && iTemps<mDateDec)
  {
    return (Angle(-projeteY)-(iTheta-mTheta)).versFloat()*mVitesseMax*10;
  }
  else if(mDateDec<iTemps && iTemps<mDateArr)
  {
    return (Angle(-projeteY)-(iTheta-mTheta)).versFloat()*0.000001*(mDateArr-iTemps)*mAccMax*10;
  }
  else
  {
    return (mTheta-iTheta).versFloat();
  }
}
