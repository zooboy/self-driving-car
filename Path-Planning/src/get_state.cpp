# include "get_state.h"
#include <math.h>
#include <iostream>
#include <vector>
#include<algorithm>


using namespace std;

get_state::get_state(){


	this->init_para.MAX_SAFE_SEC = 6;
	this->init_para.MAX_SPEED_LIMIT =49.5;
	this->init_para.MIN_SAFETY_DIST = 30;
	this->init_para.MAX_DIST_DIFF =100;
	this->init_para.MIN_SAFETY_GAP_FRONT =15;
	this->init_para.MIN_SAFETY_GAP_BEHIND = -15;
	this->init_para.BALANCE_SAFETY_GAP_FRONT =40;
	this->init_para.BALANCE_SAFETY_GAP_BEHIND = -40;
	this->init_para.COST_DIFF_TURN_MID =0.014;
	this->init_para.COST_EXTRA_1 = 0;
	// initial the parameter
}


get_state::~get_state() {}



string get_state::next_state()
{
	
	string state_n = state[3] ;
	
	this->near_cars = get_nearest();
	// get the nearest cars to ego car in each lane.


	this->front_car_vel = near_cars[ego.lane_n][0].vehicle_speed;
	//record the speed of the car in front of ego car.
	

	double dist_diff = init_para.MAX_DIST_DIFF;
	if(near_cars[ego.lane_n][0].car_dist_s<dist_diff)
	{
		dist_diff = 3*abs((ego.vehicle_speed - front_car_vel))/2.24;
		if(dist_diff<init_para.MIN_SAFETY_DIST)
			dist_diff =init_para.MIN_SAFETY_DIST ;
		// calculate the safety distance.
		// the safety distance is related to the relative velocity of two cars and the buffer time
		// here I set the buffer time as 3 seconds, it handled most situation.

	}
	if(near_cars[ego.lane_n][0].car_dist_s<dist_diff)
	{
		vector<double> cost_vec ;
		
		//if some car in reaction distance scope, ego car should calculate the cost of each option:
		// deceleration, change lane to right or change lane to left. 
		
		double kld_cost = 0.0;
		double lcl_cost = 0.0;
		double lcr_cost = 0.0;
		// kld_cost to record the 'KLDEC' state cost;
		// lcl_cost to record the 'LCL' state cost;
		// lcr_cost to record the 'LCR' state cost;


		kld_cost += dist_cost(near_cars[ego.lane_n][0].car_dist_s)*this->cost_weights[0];
		kld_cost += speed_cost(near_cars[ego.lane_n][0].vehicle_speed,near_cars[ego.lane_n][0].car_dist_s)*this->cost_weights[1];
		// for KLEDC state , I calculated two kind of cost:
		// 1. the distance to front car.
		// 2. the speed of front car.

		if((ego.lane_n -1)>-1){
			// turning left
			if((near_cars[ego.lane_n-1][0].car_dist_s<init_para.MIN_SAFETY_GAP_FRONT)||(near_cars[ego.lane_n-1][1].car_dist_s>init_para.MIN_SAFETY_GAP_BEHIND)){
				// set a safety gap 
				// if the car has not enough safety gap to turning ,the turning cost should be a big number to prevent ego car to do that.
				// here I set it to 1000.
				lcl_cost = 1000;

			}
			else
			{
				//else  means the has a safety gap to turning ,then we need to calculate the turning cost;
				lcl_cost += dist_cost(near_cars[ego.lane_n-1][0].car_dist_s)*this->cost_weights[0];
				lcl_cost += speed_cost(near_cars[ego.lane_n-1][0].vehicle_speed,near_cars[ego.lane_n-1][0].car_dist_s)*this->cost_weights[1];
				lcl_cost += turning_cost(abs(near_cars[ego.lane_n-1][0].car_dist_s),abs(near_cars[ego.lane_n-1][1].car_dist_s))* this->cost_weights[2];
				lcl_cost += init_para.COST_EXTRA_1;
				// lcl_cost are composed of 4 costs:
				// 1. the distance to front car.
				// 2. the speed of front car.
				// 3. cost of turning action.
				// 4. other cost like continuously lane changing.most of time this value is 0.
			}
		}
		else
		{
			// if(ego.lane_n -1)<=-1 means the car would be out of lane if turning left,so
			// I add a big number to avoid the car turning left.
			lcl_cost =1000;
		}
		
		if((ego.lane_n+1)<3){
			if((near_cars[ego.lane_n+1][0].car_dist_s<init_para.MIN_SAFETY_GAP_FRONT)||(near_cars[ego.lane_n+1][1].car_dist_s>init_para.MIN_SAFETY_GAP_BEHIND)){
				// set a safety gap 
				lcr_cost = 1000;
			}
			else
			{
				lcr_cost += dist_cost(near_cars[ego.lane_n+1][0].car_dist_s)*this->cost_weights[0];
				lcr_cost += speed_cost(near_cars[ego.lane_n+1][0].vehicle_speed,near_cars[ego.lane_n+1][0].car_dist_s )*this->cost_weights[1];
				lcr_cost += turning_cost(abs(near_cars[ego.lane_n+1][0].car_dist_s),abs(near_cars[ego.lane_n+1][1].car_dist_s))* this->cost_weights[2];
				lcr_cost += init_para.COST_EXTRA_1;
			}
		}
		else
		{
			lcr_cost =1000;
		}
		
		double ajustc;	
		//the code below in 'swith' segment is mainly consider in some situation
		//the car could be 'stucked' in edge lane(lane 0 or lane 2),in this situation
		//the car should change to middle lane if it meet some criterias
		//like:go straight and turning has closely cost ,and the third lane is free.
		// it is mainly to prevent the car was 'locked' to following the front car on the edge lane,
		//even if the third lane was free and has a chance to turn to middle lane. 
		switch (this->ego.lane_n){
			case 0:
				{
					ajustc = this->ajust_turn_cost(kld_cost,lcr_cost,"RIGHT");
					lcr_cost = ajustc;
					break;
				}
			case 2:
				{
					ajustc = this->ajust_turn_cost(kld_cost,lcl_cost,"LEFT");
					lcl_cost = ajustc;
					break;
				}
			default:
				{
					break;
				}
		}
		cost_vec.push_back(kld_cost);
		cost_vec.push_back(lcl_cost);
		cost_vec.push_back(lcr_cost);


		// pick up the smallest cost state .
		auto smallest = std::min_element(std::begin(cost_vec), std::end(cost_vec));
		int k = std::distance(std::begin(cost_vec), smallest);
		state_n = state[k];
	}
	else
	{
		// if the front car not in reaction distance scope.
		// that means the car can drive straight.
		// but if the middle lane is free too, and the car drive in edge lane,
		// I prefer to choose the middle lane,because in middle lane we have more options, it is more efficient drive in middle lane
		// in same circumstance.
		double middlecost = 0;
		middlecost += dist_cost(near_cars[1][0].car_dist_s)*this->cost_weights[0];;
		middlecost += speed_cost(near_cars[1][0].vehicle_speed,near_cars[1][0].car_dist_s )*this->cost_weights[1];
		middlecost += turning_cost(abs(near_cars[1][0].car_dist_s),abs(near_cars[1][1].car_dist_s))* this->cost_weights[2]; 
		if(middlecost<init_para.COST_DIFF_TURN_MID)
		{
			if(ego.lane_n ==0){
				state_n ="LCR";
			}
			else if(ego.lane_n==2){
				state_n ="LCL";
			}
		}
	}

	return state_n;

}
vector<vector<vehicle>> get_state::get_nearest()
{
	vector<vector<vehicle>> nears ;
	// the near_cars max length is  6 ,that means each lane have 2 cars which one is in front of ego car, 
	//the other one behind the ego car;
	// first of all , initialize the ego_car ;
	// vehicle_id = -2 means that was a virtual car.
	for(int i = 0 ;i< 3 ;i++)
	{
		vector<vehicle> lane_v;
		vehicle front_car;
		vehicle behind_car;

		front_car.lane_n = i;
		front_car.vehicle_id = -2 ;
		front_car.car_dist_s = 1000;
		front_car.vehicle_speed =init_para.MAX_SPEED_LIMIT;
		// I set a initial max value for distance and speed.
		// so ,if there is no car information from sensor detected, it would be set as a max value.
		// that is convenient for coding. 


		behind_car.lane_n = i;
		behind_car.vehicle_id = -2;
		behind_car.car_dist_s = -1000;
		behind_car.vehicle_speed = init_para.MAX_SPEED_LIMIT;

		lane_v.push_back(front_car);
		lane_v.push_back(behind_car);
		nears.push_back(lane_v);
	}
	
		for (int i = 0;i< car_info.size();i++)
		{
			double dist_s = car_info[i].car_dist_s;
			int lane_idx = car_info[i].lane_n;
			if(dist_s>0){
				if( nears[lane_idx][0].car_dist_s >dist_s)
				{
					nears[lane_idx][0] = car_info[i];
				}
			}
			else
			{
				if( nears[lane_idx][1].car_dist_s <dist_s)
				{
					nears[lane_idx][1] = car_info[i];
				}
			}
		}

	return nears;
}


double get_state::dist_cost(double dist_s){

	int secs =	this->init_para.MAX_SAFE_SEC;
	double max_distance = secs * ego.vehicle_speed/2.24;
	if (max_distance <init_para.MIN_SAFETY_DIST)
		max_distance =init_para.MIN_SAFETY_DIST;
	if (dist_s > max_distance)
		dist_s = max_distance;
	return logistic((max_distance-dist_s)/max_distance);
	//calculate the distance cost. the far front car from ego car ,the smaller the cost .
	// the max distance I defined as ego car would be get there after 6 seconds with current speed.
}

double get_state::speed_cost(double car_speed,double dist_s){

	double max_speed = this->init_para.MAX_SPEED_LIMIT;
	int secs =	this->init_para.MAX_SAFE_SEC;
	double spd = car_speed;
	double max_distance = secs * max_speed/2.24;

	if((car_speed>max_speed)||(dist_s>max_distance))
		spd =max_speed;
	return logistic((max_speed-spd)/max_speed);
	// in speed cost part I consider two factors ,one is the speed ,the other is distance
	// that because if front car has a long distance to ego car,even if the car's speed is slow,but the distance is large enough
	// to do other options ,we should leave out of account for its speed.

}

double get_state:: turning_cost (double front_dist,double behind_dist){

	// since the lane changing has some  potential safety hazard , I add some punish on it.
	
	return (logistic(1/front_dist)+logistic(1/behind_dist))*0.5;

}

double get_state :: middle_lane_prior (double kldcost,double turncost,double lane_cost)
{

	
	if(kldcost <turncost)
	{
		// here I set some boundary cost difference to ensure the cost was used in proper situation.
		// if the cost of go straight and turn are close ,and the third lane was free ,then I pick
		// the smaller one of them as new turning cost.
		if ((abs(kldcost-turncost)<0.05)&&(lane_cost<0.3)){
			
			turncost = min(turncost,(turncost+lane_cost)*0.5);
		}
	}

	return turncost ;
}

double get_state:: lane_cost(int lane_num)
{
	// calculate the third lane's cost.
	double cost = 0.0;
	if((near_cars[lane_num][0].car_dist_s>init_para.BALANCE_SAFETY_GAP_FRONT)&&(near_cars[lane_num][1].car_dist_s<init_para.BALANCE_SAFETY_GAP_BEHIND))
	{
		cost += dist_cost(near_cars[lane_num][0].car_dist_s)*this->cost_weights[0];
		cost += speed_cost(near_cars[lane_num][0].vehicle_speed,near_cars[lane_num][0].car_dist_s )*this->cost_weights[1];
		cost += turning_cost(abs(near_cars[lane_num][0].car_dist_s),abs(near_cars[lane_num][1].car_dist_s))* this->cost_weights[2];
	}
	else
	{
		cost =1000;
	}
	return cost ;
}

double get_state :: ajust_turn_cost(double kldcost,double turncost,string mode){
	// according to the turn left or right to call the method 'middle_lane_prior()'
	int lane_num ;
	if(mode =="LEFT"){
		lane_num = 0;
	}
	else if(mode =="RIGHT"){

		lane_num = 2;
	}
	double lcost = lane_cost(lane_num);
	double ajust_c = middle_lane_prior(kldcost,turncost,lcost);
	return ajust_c;
}


double get_state:: logistic(double x){

	// sigmoid function for generating cost.
	return 2.0/(1+std::exp(-x))-1 ;

}

