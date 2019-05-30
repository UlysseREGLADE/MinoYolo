#include "asservissement.h"

//Varaibles de mesure du temps
long temps=0;

int idAction  = 0;

//Asservissement* aserv;

void setup()
{
  //Initialisations des pin des capteurs incrementaux
  
  Serial.begin(9600);
  Serial.println("Je suis un super Robot !");

  Asservissement::init();

  double a=0, b=0;
  for(int i=0; i<4000; i++)
    a+=K_INC/LARGEUR;
  for(int i=0; i<1000; i++)
    b+=4*K_INC/LARGEUR;
}

//Boucle principale
void loop()
{
  temps = micros();
  //Moteurs::avancent(255, 255);
  //while(true)
  //  delay(10);

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

  while (Serial.available() > 0)
  {
    //aserv->a=Serial.parseInt();
    //aserv->b=Serial.parseInt();
    //aserv->c=Serial.parseInt();
    //Serial.print(aserv->a);Serial.print(" ");Serial.print(aserv->b);Serial.print(" ");Serial.println(aserv->c);
  }
  Asservissement::actualise(temps);
  //moteursAvancent(40, 60);
  //On dort pendant 0.01 s
  delay(10);
}


