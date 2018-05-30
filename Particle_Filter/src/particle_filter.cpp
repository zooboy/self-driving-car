/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h> 
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>

#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
	//set the particles number
	num_particles = 15;
	default_random_engine gen;
	double std_x, std_y, std_theta;
	std_x = std[0];
	std_y = std[1];
	std_theta = std[2];
	
	normal_distribution<double> dist_x(x, std_x);
	normal_distribution<double> dist_y(y, std_y);
	normal_distribution<double> dist_theta(theta, std_theta);
	for (int i = 0; i < num_particles; i++) {
		// generate the particles with the GPS location as mean value and add measurment noise.  
		Particle pt ;
		pt.x = dist_x(gen);
		pt.y = dist_y(gen);
		pt.theta =  dist_theta(gen);
		pt.id = i;
		particles.push_back(pt);
	}
	is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
	default_random_engine gen;
	normal_distribution<double> n_x(0, std_pos[0]);
	normal_distribution<double> n_y(0, std_pos[1]);
	normal_distribution<double> n_t(0, std_pos[2]);

	for (int i = 0; i<num_particles; i++)
	{
		double new_x, new_y, new_t;
		new_x = particles[i].x;
		new_y = particles[i].y;
		new_t = particles[i].theta;
		//judge the yaw angle whether equal to 0
		// make prediction by the motion model.
		if (abs(yaw_rate)<0.001){
				new_x += velocity*delta_t*cos(new_t);
				new_y += velocity*delta_t*sin(new_t);
			}
		else{
				new_x += (velocity/yaw_rate)*(sin(new_t+yaw_rate*delta_t)-sin(new_t));
				new_y += (velocity/yaw_rate)*(cos(new_t)-cos(new_t+yaw_rate*delta_t));
				new_t += yaw_rate*delta_t;
			}
		// add noise to px ,py and yaw angle
		new_x+= n_x(gen);
		new_y+= n_y(gen);
		new_t+= n_t(gen);

		// update the particle.
		particles[i].x = new_x;
		particles[i].y = new_y;
		particles[i].theta = new_t;
	}
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.
	

	/*
	The code frame function "dataAssociation()" seems not convenient to use. it defined a "void" function return type
	 that means we can only modify the origin parameters value to get the results,besides this, the function would generate
	 a modified vector variable that need to be called by updateWeight function ,and it will need to excuted another loop to
	 calculate the weight and make the code poor readability and low efficiency.
	*/		



}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html

		double x_std,y_std ;
		x_std = std_landmark[0];
		y_std = std_landmark[1];
		
		for(int i=0; i < particles.size(); i++) {
	    Particle p = particles[i];
		vector<double> sense_x;
		vector<double> sense_y;
		vector<int> associations;

		//  set the weight to 1 in order to calculate the product of each measurement's Multivariate-Gaussian probability
		p.weight  =1.0;



		for (int j = 0;j<observations.size();j++){

			LandmarkObs global_coord;
			LandmarkObs obs ;
			obs = observations[j];
			Map::single_landmark_s nearest_landmark;
			// transform the measurment data points that from lidar senor to the global system coordinate 
			//in one particular particle'sight.
			global_coord.x = p.x + obs.x * cos(p.theta) - obs.y * sin(p.theta);
			global_coord.y = p.y + obs.x * sin(p.theta) + obs.y * cos(p.theta);
			
			double nearest_dist = sensor_range;
			int nearest_id = 0;
			for (int k = 0; k < map_landmarks.landmark_list.size(); k++){
				double x_f = map_landmarks.landmark_list[k].x_f;
				double y_f = map_landmarks.landmark_list[k].y_f;

				// calculate distance from the transformed observations to landmarks 
				// and find the nearest landmark for each observation.
				double distance = dist(global_coord.x,global_coord.y,x_f,y_f);
			
				// update closer landmark information each time.
				if (distance < nearest_dist) {
					nearest_dist = distance;
                    nearest_id = map_landmarks.landmark_list[k].id_i;
					nearest_landmark =map_landmarks.landmark_list[k];
				}
			}
			// after this loop above ,we get the nearest landmark for one particular observation of a particle.
			// For all particles ,we should calculate :
			//P(particle numbers)*O(observations for one particles)*L(landmarks in sensor range) times
			
			
			//mark the association informations for each observation.mainly for code debug.
			associations.push_back(nearest_id);
			sense_x.push_back(global_coord.x);
			sense_y.push_back(global_coord.y);
			p = SetAssociations(p, associations, sense_x, sense_y);

			//update the weight by Multivariate-Gaussian probability formula.
			// I think the weight does not  need to normalize to (0,1), because in resample step 
			// the function "iscrete_distribution()" will choose the particle by Wi/¦²W,that means
			// the function will do the job.
			
			double dx = global_coord.x - nearest_landmark.x_f;
			double dy = global_coord.y - nearest_landmark.y_f;
			
			double y_std2 = 2*y_std*y_std;
			double x_std2 = 2*x_std*x_std;
			double dx2 = dx*dx/x_std2;
			double dy2 = dy*dy/y_std2;
			double epow = dx2+dy2;
			double coef = 1.0/(2*M_PI*x_std*y_std);

			double each_prob = coef * exp(-epow);
			if(each_prob>0)
				p.weight*=each_prob;
			//get the product of each probability .
		}
		weights.push_back(p.weight);
		particles[i] = p;		
	}
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
	std::default_random_engine gen;

	// construct a discrete_distribution object that use all particles weights as input.
	std::discrete_distribution<int>d(weights.begin(),weights.end());

	// construct a particles container to receive the resampled particles.
	std::vector<Particle> resampled_particles;
	
	for (int i = 0;i< num_particles;i++){
		int k = d(gen);
		resampled_particles.push_back(particles[k]);
	}
	//update the particles and clear the weights vector(the weights will be push_back in the update weight step )
	particles = resampled_particles;
	weights.clear();

}

Particle ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations, 
                                     const std::vector<double>& sense_x, const std::vector<double>& sense_y)
{
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates


    particle.associations= associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;

	return particle;
}

string ParticleFilter::getAssociations(Particle best)
{
	vector<int> v = best.associations;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseX(Particle best)
{
	vector<double> v = best.sense_x;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseY(Particle best)
{
	vector<double> v = best.sense_y;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
