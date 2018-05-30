#include "PID.h"
#include<cmath>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double _Kp, double _Ki, double _Kd) {

	Kp = _Kp;
	Ki = _Ki;
	Kd = _Kd;
}

void PID::UpdateError(double cte) {

	//calculte each kind of error. 
	// the proportion part;
	p_error = -Kp*cte;

	// the intergal part;
	itg_cte += cte ;
	i_error = -Ki*itg_cte;

	// the differential part;
	d_error = -Kd*(cte - prev_cte);

	//update the 'last time cte value' in order to calculate the differential part next time-step
	prev_cte = cte;

	// if the cte near 0, then set the intergal part to 0, in case of avoiding the intergal part so high that changing the errors. 
	if(abs(cte)<0.001){

		itg_cte = 0;
	}
	// if some cte value too high ,in case of avoiding the intergal part wind up that cause a big coefficient to the errors .
	//if(abs(cte)>2){

	//	itg_cte = 0;
	//}
	
	//the code below is used to print the debug informations
	total_cte+=abs(cte);
	step ++;
	mean_cte = total_cte /step ;
	if(cte > max_cte){
		max_cte = cte;
	}
	if(abs(cte) < min_cte){
		min_cte = abs(cte);
	}
}

double PID::TotalError() {

	// sum up the errors
	double totle_err = p_error + i_error +d_error;
	return totle_err;
}



