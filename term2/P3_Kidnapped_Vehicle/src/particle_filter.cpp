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
#include <cmath> 
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>

#include "particle_filter.h"

using namespace std;



void ParticleFilter::init(double x, double y, double theta, double std[]) {
    num_particles = 20;
    
    particles.resize(num_particles);
    weights.resize(num_particles);
    
    std::default_random_engine gen;
    std::normal_distribution<double> norm_x (x, std[0]);
    std::normal_distribution<double> norm_y (y, std[1]);
    std::normal_distribution<double> norm_theta (theta, std[2]);
    
    int id = 0;
    for (Particle& p : particles){
        p.x = norm_x(gen);
        p.y = norm_y(gen);
        p.theta = norm_theta(gen);
        p.id = id;
        p.weight = 1.0 / static_cast<double>(num_particles);
        id++;
    }
    is_initialized = true;

}


void ParticleFilter::prediction(double dt, double std_pos[], double velocity, double yaw_rate) {
    
    std::default_random_engine gen;
    std::normal_distribution<double> norm_x (0, std_pos[0]);
    std::normal_distribution<double> norm_y (0, std_pos[1]);
    std::normal_distribution<double> norm_theta (0, std_pos[2]);
    
    for (Particle &p : particles){
        if (std::fabs(yaw_rate) < 0.001){
            p.x += velocity*dt*std::cos(p.theta) + norm_x(gen);
            p.y += velocity*dt*std::sin(p.theta) + norm_y(gen);
            p.theta += norm_theta(gen);
        }
        else{
            p.x += (velocity/yaw_rate) * ( std::sin(p.theta +yaw_rate*dt) - std::sin(p.theta) ) + norm_x(gen);
            p.y += (velocity/yaw_rate)*(-std::cos(p.theta +yaw_rate*dt)
                               + std::cos(p.theta)) + norm_y(gen);
            p.theta += yaw_rate*dt + norm_theta(gen);
        }
    }
    
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		std::vector<LandmarkObs> observations, Map map_landmarks) {
	
    double std_x = std_landmark[0];
    double std_y = std_landmark[1];
    
    double weight_sum = 0.0;
    
    
    for (Particle& p : particles){
        double total_prob = 1.0;
        for (LandmarkObs& z : observations){
            
            //Coordinates of the observation from the point of view of the particle, in the map coordinates
            double meas_par_x = p.x + z.x*std::cos(p.theta) - z.y*std::sin(p.theta);
            double meas_par_y = p.y + z.x*std::sin(p.theta) + z.y*std::cos(p.theta);
            
            //Association part
            int near_land_id = -1;
            double d_min = sensor_range;
            for (int k = 0; k < map_landmarks.landmark_list.size(); ++k)
            {
                Map::single_landmark_s& mark  = map_landmarks.landmark_list[k];
                double dist_to_landmark = dist(p.x, p.y, mark.x_f, mark.y_f);
                if (dist_to_landmark < sensor_range){
                    double meas_error = dist(meas_par_x, meas_par_y, mark.x_f, mark.y_f);
                    
                    if (meas_error < d_min){
                        d_min = meas_error;
                        near_land_id = k;
                    }
                }
            }
            if (near_land_id < 0)
                continue;

            double m_x = map_landmarks.landmark_list[near_land_id].x_f;
            double m_y = map_landmarks.landmark_list[near_land_id].y_f;

            double a = std::pow(meas_par_x - m_x, 2) / std::pow(std_x,2);
            double b = std::pow(meas_par_y - m_y, 2) / std::pow(std_y,2);
            total_prob *= std::exp(-0.5*(a + b))/(2.0*M_PI*std_x*std_y);
        }
    p.weight = (total_prob != 1.0 ? total_prob : 0.0);  
    weight_sum += total_prob;
    }
    for (Particle& p : particles)
        p.weight /= weight_sum;

}


void ParticleFilter::resample() {

    for (int i = 0; i < particles.size(); ++i){
        weights[i] = particles[i].weight;
    }

    std::vector<Particle> copy_particles = particles;
    std::default_random_engine gen;
    std::discrete_distribution<int> d(weights.begin(), weights.end());

    for (Particle& p : particles)
        p = copy_particles[d(gen)];

}


Particle ParticleFilter::SetAssociations(Particle particle, std::vector<int> associations, std::vector<double> sense_x, std::vector<double> sense_y)
{
	//particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
	// associations: The landmark id that goes along with each listed association
	// sense_x: the associations x mapping already converted to world coordinates
	// sense_y: the associations y mapping already converted to world coordinates

	//Clear the previous associations
	particle.associations.clear();
	particle.sense_x.clear();
	particle.sense_y.clear();

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

