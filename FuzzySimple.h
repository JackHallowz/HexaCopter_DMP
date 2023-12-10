#pragma once
#ifndef FUZZYSIMPLE_H_
#define FUZZYSIMPLE_H_
#include "Arduino.h"
#define LENGTH 5 //number of columns change if needed
class SimpleFuzzy
{
public:
	SimpleFuzzy(int, double*, double, double, double=0);
	double Priority();
	double LeftShoulder();
	double Triangle();
	double RightShoulder();
private:
	double left, right, top;
	double* currentvalue;
	int select;
};




class sugeno
{
public:
  sugeno(int, int);
	void initiate(int);
	double* defuzzication(double[], double[][LENGTH], int, int);
private:
	const int nMember = 3;
  double Length,Out_length;
  
};

#endif // !FUZZYSIMPLE_H_
