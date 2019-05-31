#include "trajectoire.h"
#include <cmath>
#include <iostream>
#define PI 3.14159
#define TIMEMARGIN 0.5

double gettimetraj()
{
    struct timespec currentTime;
    clock_gettime(CLOCK_MONOTONIC, &currentTime);
    return (double)currentTime.tv_sec + (double)(currentTime.tv_nsec)/1e9;
}


Trajectoire::Trajectoire(TypeTrajectoire type):TYPE(type)
{
  mDateDep=0;
  mRetard=0;
  mLastTime = gettimetraj();
}
Trajectoire::~Trajectoire(){}
double Trajectoire::erreurPos(double iX, double iY, Angle iTheta, double iTemps, bool iIsFreez){return 0;}
double Trajectoire::erreurRot(double iX, double iY, Angle iTheta, double iTemps, bool iIsFreez){return 0;}
bool Trajectoire::marcheArriere(){return mMarcheArriere;}

bool Trajectoire::estFinie(double iTemps)
{
 // iTemps = iTemps-mDateDep;
  if(iTemps > mDateArr + TIMEMARGIN || mDateArr == 0)
  {
    return true;
  }
  else
    return false;
}


Rotation::Rotation(double iX, double iY, Angle iThetaDep, Angle iThetaArr, double iVitesseMax, double iAccMax, double temps):Trajectoire(ROTATION)
{
  mX = iX;
  mY = iY;
  mThetaDep = iThetaDep;
  mThetaArr = iThetaArr;
  mVitesseMax = iVitesseMax;
  mAccMax = iAccMax;
  mDateDep = temps;
  if((mThetaArr-mThetaDep).versFloat()<0)
    mSens = A_TRIGO;
  else
    mSens = TRIGO;

  double dureeAcc = mVitesseMax/mAccMax;
  double angleAcc = 0.5*mAccMax*pow(dureeAcc, 2);
  double angleTot = fabs((mThetaArr-mThetaDep).versFloat());

  if(2*angleAcc>angleTot)
  {
    angleAcc = angleTot/2;
    dureeAcc = sqrt(2*angleAcc/mAccMax);

    mAngleAcc = angleAcc;
    mAngleConst = 0;
    mDateAcc = mDateDep + dureeAcc;
    mDateDec = mDateAcc + 0;
    mDateArr = mDateDec + dureeAcc;
  }
  else
  {
    mAngleAcc = angleAcc;
    mAngleConst = angleTot-2*mAngleAcc;
    mDateAcc = mDateDep + dureeAcc;
    mDateDec = mDateAcc + mAngleConst/mVitesseMax;
    mDateArr = mDateDec + dureeAcc;
  }
}

Rotation::~Rotation(){}

double Rotation::erreurPos(double iX, double iY, Angle iTheta, double iTemps, bool iIsFreez)
{
  if(iIsFreez)
  {
    mRetard += iTemps - mLastTime;
  }
  mLastTime = iTemps;
  iTemps -= mRetard;
  double vecteur[2] = {cos(iTheta.versFloat()), sin(iTheta.versFloat())};
  double projX = (mX-iX)*vecteur[X]+(mY-iY)*vecteur[Y];
  return projX;
}

double Rotation::erreurRot(double iX, double iY, Angle iTheta, double iTemps, bool iIsFreez)
{
  if(iIsFreez)
  {
    mRetard += iTemps - mLastTime;
  }
  mLastTime = iTemps;
  iTemps -= mRetard;


  if(iTemps<mDateDep)
  {
    return (mThetaDep-iTheta).versFloat();
  }
  else if(mDateDep<iTemps && iTemps<mDateAcc)
  {
    Angle theta = mThetaDep + 0.5*mSens*mAccMax*pow((iTemps-mDateDep), 2);
    return (theta-iTheta).versFloat();
  }
  else if(mDateAcc<iTemps && iTemps<mDateDec)
  {
    Angle theta = mThetaDep + mSens*(mAngleAcc + (iTemps-mDateAcc)*mVitesseMax);
    return (theta-iTheta).versFloat();
  }
  else if(mDateDec<iTemps && iTemps<mDateArr)
  {
    Angle theta = mThetaArr - 0.5*mSens*mAccMax*pow((mDateArr-iTemps), 2);
    return (theta-iTheta).versFloat();
  }
  else
  {
    return (mThetaArr-iTheta).versFloat();
  }
}




Droite::Droite(double iXDepart, double iYDepart, double iXArrivee, double iYArrivee,  double iVitesseMax, double iAccMax, double temps):Trajectoire(DROITE)
{
  mVitesseMax = iVitesseMax;
  mAccMax = iAccMax;
  mDateDep =  temps;
  mDepart[X] = iXDepart;
  mDepart[Y] = iYDepart;
  mArrivee[X] = iXArrivee;
  mArrivee[Y] = iYArrivee;
  mDateDep = temps;

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
      mDateAcc = mDateDep + dureeAcc;
      mDateDec = mDateAcc + 0;
      mDateArr = mDateDec + dureeAcc;
    }
    else
    {
      mDistAcc = distAcc;
      mDistConst = dist - 2*distAcc;
      mDateAcc = mDateDep + dureeAcc;
      mDateDec = mDateAcc + mDistConst/mVitesseMax;
      mDateArr = mDateDec + dureeAcc;
    }
  }
}

Droite::~Droite(){}

double Droite::erreurPos(double iX, double iY, Angle iTheta, double iTemps, bool iIsFreez)
{
  if(iIsFreez)
  {
    mRetard += iTemps - mLastTime;
  }
  mLastTime = iTemps;
  iTemps -= mRetard;
  //On se place dans la base ou le vecteur x colineaire a la trajectoire
//if(iIsFreez)
//return 0;
  double projeteX = (iX-mDepart[X])*mVecteur[X]+(iY-mDepart[Y])*mVecteur[Y];
  if(iTemps<mDateDep)
  {
    double vecteurX = cos(iTheta.versFloat());
    double vecteurY = sin(iTheta.versFloat());
    projeteX = (mDepart[X]-iX)*vecteurX+(mDepart[Y]-iY)*vecteurY;
    return projeteX;
  }
  else if(mDateDep<iTemps && iTemps<mDateAcc)
  {
    double x = 0.5*mAccMax*pow((iTemps - mDateDep), 2);
    return x-projeteX;
  }
  else if(mDateAcc<iTemps && iTemps<mDateDec)
  {
    double x = mDistAcc + (iTemps-mDateAcc)*mVitesseMax;
    return x-projeteX;
  }
  else if(mDateDec<iTemps && iTemps<mDateArr)
  {
    double x = 2*mDistAcc+mDistConst - 0.5*mAccMax*pow((mDateArr-iTemps), 2);
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

double Droite::erreurRot(double iX, double iY, Angle iTheta, double iTemps, bool iIsFreez)
{
  if(iIsFreez)
  {
    mRetard += iTemps - mLastTime;
  }
  mLastTime = iTemps;
  iTemps -= mRetard;
if(iIsFreez)
return 0;
  //La, c'est une simple operation de projection sur la droite de la trajectoire
  double projeteY = -(iX-mDepart[X])*mVecteur[Y]+(iY-mDepart[Y])*mVecteur[X];
  //return 0;

  if(iTemps<mDateDep)
  {
    return (mTheta-iTheta).versFloat();
  }
  else if(mDateDep<iTemps && iTemps<mDateAcc)
  {
    return (Angle(-projeteY)-(iTheta-mTheta)).versFloat()*(iTemps - mDateDep)*mAccMax*10;
  }
  else if(mDateAcc<iTemps && iTemps<mDateDec)
  {
    return (Angle(-projeteY)-(iTheta-mTheta)).versFloat()*mVitesseMax*10;
  }
  else if(mDateDec<iTemps && iTemps<mDateArr)
  {
    return (Angle(-projeteY)-(iTheta-mTheta)).versFloat()*(mDateArr-iTemps)*mAccMax*10;
  }
  else
  {
    return (mTheta-iTheta).versFloat();
  }
}

Attente::Attente(double attente, double temps):Trajectoire(ATTENTE)
{
  mDateArr=temps+attente;
}

Attente::~Attente(){}
