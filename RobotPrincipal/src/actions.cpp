#include "actions.h"
#define PIN_BASE 300
#define MAX_PWM 4096
#define HERTZ 50
#include <iostream>

float map(float input, float min, float max)
{
	return (input * max) + (1 - input) * min;
}

actions::actions(/* args */)
{
    //wiringPiSetup();
    int fd = pca9685Setup(PIN_BASE, 0x40, HERTZ);
	if (fd < 0)
	{
		std::cout<<"Erreur de préparation des actionneurs"<<std::endl;
	}
	// Reset all output
	pca9685PWMReset(fd);
}

actions::~actions(){}

void actions::baisserBras()
{
	pwmWrite(PIN_BASE , 233);
}

void actions::leverBras()
{
	pwmWrite(PIN_BASE , 393);
}

void actions::ventouseAvantOn()
{
int en = 21;
int in1=22;
int in2=26;

pinMode(in1,OUTPUT);
pinMode(in2,OUTPUT);
pinMode(en,OUTPUT);
digitalWrite(in1, LOW);
digitalWrite(in2, LOW);
digitalWrite(en, LOW);

//lancer le moteur
digitalWrite(in1, HIGH);
digitalWrite(in2, LOW);
digitalWrite(en, HIGH);
}

void actions::ventouseAvantOff()
{
int en = 21;
int in1=22;
int in2=26;

pinMode(in1,OUTPUT);
pinMode(in2,OUTPUT);
pinMode(en,OUTPUT);
digitalWrite(in1, LOW);
digitalWrite(in2, LOW);
digitalWrite(en, LOW);

//Arrêter le moteur
digitalWrite(in1, LOW);
digitalWrite(in2, LOW);
digitalWrite(en, LOW);
}

void actions::ventouseArriereOn()
{
int en = 24;
int in1=25;
int in2=23;

pinMode(in1,OUTPUT);
pinMode(in2,OUTPUT);
pinMode(en,OUTPUT);
digitalWrite(in1, LOW);
digitalWrite(in2, LOW);
digitalWrite(en, LOW);

//lancer le moteur
digitalWrite(in1, HIGH);
digitalWrite(in2, LOW);
digitalWrite(en, HIGH);
}

void actions::ventouseArriereOff()
{
int en = 24;
int in1=25;
int in2=23;

pinMode(in1,OUTPUT);
pinMode(in2,OUTPUT);
pinMode(en,OUTPUT);
digitalWrite(in1, LOW);
digitalWrite(in2, LOW);
digitalWrite(en, LOW);

//Arrêter le moteur
digitalWrite(in1, LOW);
digitalWrite(in2, LOW);
digitalWrite(en, LOW);
}

void actions::claqueGauche()
{
	pwmWrite(PIN_BASE +15, 154);
}
void actions::claqueDroite()
{
	pwmWrite(PIN_BASE +15, 454);
}

void actions::sortirVentouse()
{
	pwmWrite(PIN_BASE +14, 424);
}
void actions::rentrerVentouse()
{
	pwmWrite(PIN_BASE +14, 220);
}

