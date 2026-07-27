#include "alldata.h"

#define ESPILON 0.01




void PID_Init(PID_t *p)
{	p->Target = 0;
	p->Actual = 0;
    p->Out=0;
	p->Error0 = 0;
	p->Error1 = 0;
	p->ErrorInt = 0;
}



void PID_Update(PID_t *p)
{
	p->Error1 = p->Error0;
	p->Error0 = p->Target - p->Actual;
	
	

	if(fabs(p->Ki)>ESPILON)
	{
		p->ErrorInt += p->Error0;
		
		if (p->ErrorInt > p->ErrorIntMax) {p->ErrorInt = p->ErrorIntMax;}
		if (p->ErrorInt < p->ErrorIntMin) {p->ErrorInt = p->ErrorIntMin;}
	}
	else
	{
	p->ErrorInt=0;
	}
	
	p->Out = p->Kp * p->Error0 + p->Ki * p->ErrorInt + p->Kd * (p->Error0 - p->Error1);
	
	if (p->Out > 0) {p->Out += p->OutOffset;}
	if (p->Out < 0) {p->Out -= p->OutOffset;}
	
	if (p->Out > p->OutMax) {p->Out = p->OutMax;}
	if (p->Out < p->OutMin) {p->Out = p->OutMin;}
	
	
	
	
}