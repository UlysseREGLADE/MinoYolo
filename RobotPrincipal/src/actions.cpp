#include "actions.h"
#define PIN_BASE 300
#define MAX_PWM 4096
#define HERTZ 50

int calcTicks(float impulseMs, int hertz)
{
	float cycleMs = 1000.0f / hertz;
	return (int)(MAX_PWM * impulseMs / cycleMs + 0.5f);
}

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
    float millis = map(0.14, 1, 2);
	int	tick = calcTicks(millis, HERTZ);
		pwmWrite(PIN_BASE , tick);
}

void actions::leverBras()
{
    float millis = map(0.92, 1, 2);
	int	tick = calcTicks(millis, HERTZ);
		pwmWrite(PIN_BASE , tick);
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
