#include "FuzzySimple.h"
#define arraylength 3

SimpleFuzzy::SimpleFuzzy(int n, double* x, double L, double R, double C )
{
	select = n;
	switch (select)
	{
	case 1: //Left
		left = L;
		right = R;
		currentvalue = x;
		break;
	case 2: //Right
		left = L;
		right = R;
		currentvalue = x;
		break;
	case 3: //Triangle
		left = L;
		right = R;
		top = C;
    currentvalue = x;
		break;
	default:
		break;
	}
}
double SimpleFuzzy::Priority()
{
	switch (select)
	{
	case 1:
		return LeftShoulder();
		break;
	case 2:
		return RightShoulder();
		break;
	case 3:
		return Triangle();
		break;
	default:
    return 0;
		break;
	}
}

double SimpleFuzzy::LeftShoulder()
{
	double current = *currentvalue;
	if (current < left)
	{
		return 1;
	}
	else if (left <= current && current <= right)
	{
		return  (right - current) / (right - left);
	}
	else if (current > right)
	{
		return  0;
	}
}

double SimpleFuzzy::Triangle()
{
	double current = *currentvalue;
	if (left <= current && current <= top)
	{
		return (current - left) / (top - left);
	}
	else if (top < current && current <= right)
	{
		return  (right - current) / (right - top);
	}
	else if (current > right || current < left)
	{
		return  0;
	}
}

double SimpleFuzzy::RightShoulder()
{
	double current = *currentvalue;
	if (current < left)
	{
		return 0;
	}
	else if (left <= current && current <= right)
	{
		return (current - left) / (right - left);
	}
	else if (current > right)
	{
		return 1;
	}
}





sugeno::sugeno(int Length, int Out_length)
{

}
void sugeno::initiate(int Input)
{
  
	// typedef struct Data_int
	// {
	// 	double nuy[sth];
	// 	double outMember[sth];
	// }Data_int;
}
double* sugeno::defuzzication(double a[], double b[][LENGTH], int Length, int Out_length)
{
  double numo = 0;
  double deno = 0;
  double* sth = new double [Out_length];
  for(int i=0;i<Out_length;i++)
  {
    for(int j=0;j<Length;j++)
    {
      numo += a[j]*b[i][j];
      deno += a[j];
    }
    sth[i] = (numo/deno);
    numo = 0;
    deno = 0;
  }
  return sth;
}
