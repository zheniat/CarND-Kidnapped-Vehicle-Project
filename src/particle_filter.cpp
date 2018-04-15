/*
 * particle_filter.cpp
 *
 *  Created on: April 14, 2018
 *      Author: Eugene Citrisky
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
static default_random_engine gen;


void ParticleFilter::init(double x, double y, double theta, double std[]) {
    num_particles = 300;
    // Create a normal (Gaussian) distribution for x, y, theta
    normal_distribution<double> dist_x(x, std[0]);
    normal_distribution<double> dist_y(y, std[1]);
    normal_distribution<double> dist_theta(theta, std[2]);
    
    for (int i = 0; i < num_particles; ++i) {
        Particle p;
        p.id = i;
        p.x = dist_x(gen);
        p.y = dist_y(gen);
        p.theta = dist_theta(gen);
        p.weight = 1.0;
        particles.push_back(p);
    }
    
    is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
    normal_distribution<double> dist_x(0, std_pos[0]);
    normal_distribution<double> dist_y(0, std_pos[1]);
    normal_distribution<double> dist_theta(0, std_pos[2]);

    for (int i = 0; i < num_particles; ++i) {
        if (fabs(yaw_rate) < 0.00001){
            particles[i].x += velocity * delta_t *cos(particles[i].theta);
            particles[i].y += velocity * delta_t *sin(particles[i].theta);

        } else {
            particles[i].x += velocity/yaw_rate * (sin(particles[i].theta + yaw_rate * delta_t) - sin(particles[i].theta));
            particles[i].y += velocity/yaw_rate * (cos(particles[i].theta) - cos(particles[i].theta + yaw_rate * delta_t));
            particles[i].theta += yaw_rate * delta_t;
        }
        //noise
        particles[i].x += dist_x(gen);
        particles[i].y += dist_y(gen);
        particles[i].theta += dist_theta(gen);
    }
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
    if(predicted.size()>0){
        for(int i=0; i<observations.size();i++){
            LandmarkObs o = observations[i];
            
            //find the nearest neighbor landmark
            LandmarkObs min_landmark = predicted[0];
            double min_dist = dist(o.x,o.y,min_landmark.x,min_landmark.y);
            
            for (int j=1; j<predicted.size(); j++) {
                LandmarkObs l = predicted[j];
                double current_dist = dist(o.x,o.y,l.x,l.y);
                if(current_dist < min_dist){
                    min_dist = current_dist;
                    min_landmark = l;
                }
            }
            observations[i].id = min_landmark.id;
        }
    }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
    
    if(std_landmark[0] == 0 || std_landmark[1]==0){
        return;
    }
    
    double sig_x = std_landmark[0];
    double sig_y = std_landmark[1];
    double sig_x2 = pow(sig_x,2);
    double sig_y2 = pow(sig_y,2);

    for (int i = 0; i < num_particles; ++i) {
        double px = particles[i].x;
        double py = particles[i].y;
        double ptheta = particles[i].theta;
        
        particles[i].weight = 1;
        
        //Transform observations from car coordinates to map coordinates
        vector<LandmarkObs> transformed_obs;
        for (int j = 0; j < observations.size(); ++j) {
            LandmarkObs o = observations[j];
            LandmarkObs t_o;
            t_o.id = o.id;
            t_o.x = px + cos(ptheta) * o.x - sin(ptheta) * o.y;
            t_o.y = py + sin(ptheta) * o.x + cos(ptheta) * o.y;
            
            transformed_obs.push_back(t_o);
        }
        
        //create vector of landmark objects within the particle range
        vector<LandmarkObs> landmarks;
        for (int j = 0; j < map_landmarks.landmark_list.size(); ++j) {
            Map::single_landmark_s lm = map_landmarks.landmark_list[j];
            
            //filter out landmarks which are outside of the particle range
            if (fabs(px -lm.x_f) <= sensor_range && fabs(py - lm.y_f) <= sensor_range) {
                landmarks.push_back(LandmarkObs{lm.id_i,lm.x_f,lm.y_f});
            }
        }
        
        dataAssociation(landmarks,transformed_obs);
        
        //calculate weights
        for(int j=0; j<transformed_obs.size();j++)
        {
            LandmarkObs o = transformed_obs[j];
            LandmarkObs min_landmark;
            for(int k=0; k<landmarks.size();k++){
                if(o.id == landmarks[k].id){
                    min_landmark = landmarks[k];
                    break;
                }
            }
            
            //calculate observation weight
            double o_exp = exp(-(pow(o.x-min_landmark.x,2)/(2*sig_x2) + pow(o.y-min_landmark.y,2)/(2*sig_y2)));
            double o_w = (1/(2*M_PI*sig_x*sig_y)) * o_exp;
            
            //update particle weight
            particles[i].weight *= o_w;
        }
    }
}

void ParticleFilter::resample() {
    uniform_int_distribution<int> indexDist (0,num_particles-1);
    
    vector<Particle> resampled_particles;
    vector<double> weights;

    for(int i=0; i<num_particles;i++){
        weights.push_back(particles[i].weight);
    }

    
    int index = indexDist(gen);
    double beta = 0.0;
    uniform_real_distribution<double> weightDist(0.0, *max_element(weights.begin(),weights.end()));
    
    for(int i=0; i<num_particles; i++){
        beta += weightDist(gen) * 2.0;
        while(beta > weights[index]){
            beta -= weights[index];
            index = (index + 1) % num_particles;
        }
        resampled_particles.push_back(particles[index]);
    }
    
    particles = resampled_particles;
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
