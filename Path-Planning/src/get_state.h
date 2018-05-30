#ifndef GET_STATE_H
#define GET_STATE_H

#include <iostream>
#include <vector>
#include <string>
#define DIST_COST_W 1.0
#define SPEED_COST_W 0.3
#define TURN_COST_W 1.0
// here I defined 3 wights initial coefficients

using namespace std;

 struct vehicle {

	    int vehicle_id ; // car's id ,not used in this code implement 
		double vehicle_speed;// car's speed
		double vehicle_d; // d in frenet coord
		double vehicle_check_s; // s in frenet coord
		int lane_n ; // car driving in which lane
		double car_dist_s ;// the distance in s value that ego car and other cars.
	};
 // defined  a struct called 'vehicle' to record the car's information 

 // this information is for the cycle end in future .

 struct parameters 
 {
	 double MAX_SAFE_SEC ;    // safety buffer time for car to prepare next actions.
	 double MAX_SPEED_LIMIT;  //max speed limit
	 double MIN_SAFETY_DIST;  // min safety distance that between ego car and the car in front of ego car, to avoid rear-end.
	 double MAX_DIST_DIFF;   // when the front car come in this scope,the ego car would keep a proper velcity and keep a safety distance between them. 
	 double MIN_SAFETY_GAP_FRONT; // a safety gap between the front car in adjacent lane and ego car indicating whether ego car can change lane .
	 double MIN_SAFETY_GAP_BEHIND;// a safety gap between the behind car in adjacent lane and ego car indicating whether ego car can change lane .
	 double BALANCE_SAFETY_GAP_FRONT; 
	 // a gap between the front car in edge lane(lane 0 or lane 2) and ego car(in this situation ego car was in lane 2 or lane 0)
	 // that was used in judging the ego car whether change lane from lane 0 to lane 2 or lane 2 to lane 0. 
	 // the parameter was used in the situation like this: assume the traffic in lane 0,1 was heavy,and ego car in lane 0, 
	 // but the traffic in lane 2 was free, then ego car need to judge whether to change lane to lane 1 in order to gain more
	 // chance to change lane to lane 2.
	 double BALANCE_SAFETY_GAP_BEHIND;// same as above , different is the distance beweent ego car and the car behind ego car.
	 double COST_DIFF_TURN_MID; 
	 // a cost difference using in judging go straight in edge lane(lane 0,2) or turn to middle lane(lane 1).
	 // consider in the situation that both driving in edge lanes and in  middle lane have closely cost, and the cost is very low,
	 // 
	 double COST_EXTRA_1; // some extra cost like successive changing lane punishment.
 };
 // the initial parameters were defined as a struct.


class get_state{

public :

	parameters  init_para ;
	vector <vehicle>  car_info ; // a container record the gather car's information from sensor fusion.
	vehicle ego; // ego car.
	int prev_size; // the count of steps that has not excuted yet from last cycle.
	//vector<vehicle> nearest_car; // cars that near ego car.
	double front_car_vel ; // the speed of the car in front of ego car.
	vector<double> cost_weights ={DIST_COST_W,SPEED_COST_W,TURN_COST_W};
	vector<vector<vehicle>> near_cars;// a container record the nearest cars information that calculated from method 'get_nearest()'

	// cost weights.
	
	vector<string> state ={"KLDEC","LCL","LCR","KL"};
	//
	// I  defined 4 states :
	// KL  :Keep Lane 
	// KLDEC  :Keep Lane and Deceleration
	// LCL :Change Lane : Left
	// LCR: Chante Lane :Right
	get_state();


	virtual ~get_state();
	string next_state();// calculate the cost of each option ,and return the minimize cost result. 
	vector<vector<vehicle>> get_nearest(); 
	// get the nearest cars to ego car.
	// here nearest means 2 cars in each lane, one in front ego car, the other behind ego car.

	
private:

	double logistic(double x);// sigmod funtion to calculate cost.
	double dist_cost(double dist_s); // calculate the distance cost.
	double speed_cost(double car_speed,double dist_s); // calculate the speed cost.
	double turning_cost (double front_dist,double behind_dist); // calculate turning cost.

	//the method below are to calculate the turning cost when ego car in edge lanes.
	double middle_lane_prior (double kldcost,double turncost,double lanecost); 
	double lane_cost(int lane_num);
	double ajust_turn_cost(double kldcost,double turncost,string mode);
};
#endif