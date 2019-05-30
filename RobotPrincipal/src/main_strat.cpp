/*#include "asservissement.h"

//Varaibles de mesure du temps
long temps=0;

int idAction  = 0;

//Asservissement* aserv;

void setup()
{
  Asservissement::init();

}

//Boucle principale
void loop()
{
  temps = micros();

  if(Asservissement::traj->estFinie(temps) && idAction<7)
  {
    idAction+=1;
    Serial.println(idAction);
    delete(Asservissement::traj);
    if(idAction==1)
      Asservissement::traj = new Rotation(0.5, 0, Angle(0), Angle(PI/2), 2);
    if(idAction==2)
      Asservissement::traj = new Droite(0.5, 0, 0.5, 0.5, 0.5);
    if(idAction==3)
      Asservissement::traj = new Rotation(0.5, 0.5, Angle(PI/2), Angle(PI), 2);
    if(idAction==4)
      Asservissement::traj = new Droite(0.5, 0.5, 0, 0.5, 0.5);
    if(idAction==5)
      Asservissement::traj = new Rotation(0, 0.5, Angle(PI), Angle(-PI/2), 2);
    if(idAction==6)
      Asservissement::traj = new Droite(0, 0.5, 0, 0, 0.5);
    if(idAction==7)
      Asservissement::traj = new Rotation(0, 0, Angle(-PI/2), Angle(0), 2);
  }


  Asservissement::actualise(temps);
  delay(10);
}

*/
