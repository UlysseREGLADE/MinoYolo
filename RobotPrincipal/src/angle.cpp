#include "angle.h"
#include <cmath>

#define PI 3.15159

Angle::Angle()
{
  mAngle = 0;
  ajuste();
}

Angle::Angle(double iAngle)
{
  mAngle = iAngle;
  ajuste();
}

Angle::Angle(double iX, double iY)
{
  double norme = sqrt(pow(iX, 2) + pow(iY, 2));
  // Code pas logique pour des raisons de précisions
  // Voir Ulysse pour plus de détails
  if(iX!=0 && abs(iY/iX)<1)
  {
    mAngle = asin(iY/norme);
    if(iX<0)
      mAngle = PI - asin(iY/norme);
  }
  else
  {
    mAngle = acos(iX/norme);
    if(iY<0)
      mAngle = - acos(iX/norme) - PI/2;
  }
}

Angle::~Angle(){}

void Angle::ajuste()
{
  while(mAngle>PI)
    mAngle -= 2*PI;
  while(mAngle<-PI)
    mAngle += 2*PI;
}

double Angle::versFloat() const{return mAngle;}
double Angle::versFloatPositif() const
{
  if(mAngle<0)
  {
    return mAngle + 2*PI;
  }
  else
  {
    return mAngle;
  }
}
Angle Angle::operator+(Angle iAngle) {return Angle(mAngle + iAngle.mAngle);}
Angle Angle::operator-(Angle iAngle) {return Angle(mAngle - iAngle.mAngle);}
Angle Angle::operator+(double iAngle) {return Angle(mAngle + iAngle);}
Angle Angle::operator-(double iAngle) {return Angle(mAngle - iAngle);}
Angle Angle::operator=(Angle iAngle) {mAngle=iAngle.mAngle; ajuste();}
Angle Angle::operator=(double iAngle) {mAngle=iAngle; ajuste();}
