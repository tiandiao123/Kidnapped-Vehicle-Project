#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h> 
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>
#include <climits>

#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
    
    num_particles=80;
	default_random_engine gen;
	double std_x, std_y, std_psi; // Standard deviations for x, y, and psi
	std_x = std[0];
	std_y = std[1];
	std_psi = std[2];


	// These 3 lines create normal (Gaussian) distributions for x,y and theta
	normal_distribution<double> dist_x(x, std_x);
	normal_distribution<double> dist_y(y, std_y);
	normal_distribution<double> dist_psi(theta, std_psi);
    
	for(int i=0;i<num_particles;i++){
		weights.push_back(1.0);
	}

	for(int i=0;i<num_particles;i++){
         Particle temp;
         double sample_x, sample_y, sample_psi;
		 sample_x = dist_x(gen);
		 sample_y = dist_y(gen);
		 sample_psi = dist_psi(gen);

		 temp.x=sample_x;
		 temp.y=sample_y;
		 temp.theta=sample_psi;
		 temp.id=i;
		 temp.weight=1;

		 particles.push_back(temp);
	}
	is_initialized=true;

}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/


	double cosval=cos(std_pos[2]);
    double sinval=sin(std_pos[2]);
    double ro=std_pos[2];

    default_random_engine gen;
    
    std::cout<<particles.size();
    for(int i=0;i<num_particles;i++){
    	//Particle particle=particles[i];
    	
    	double p_x=particles[i].x;
    	double p_y=particles[i].y;
    	double yaw=particles[i].theta;
    	double px_p=0;
    	double py_p=0;

    	if (fabs(yaw_rate) > 0.001) {
            px_p = p_x + velocity/yaw_rate * ( sin (yaw + yaw_rate*delta_t) - sin(yaw));
            py_p = p_y + velocity/yaw_rate * ( cos(yaw) - cos(yaw+yaw_rate*delta_t) );
        }else {
            px_p = p_x + velocity*delta_t*cos(yaw);
            py_p = p_y + velocity*delta_t*sin(yaw);
        }
        
        double yaw_p=yaw+yaw_rate*delta_t;

        normal_distribution<double> dist_x(0, std_pos[0]*delta_t);
	    normal_distribution<double> dist_y(0, std_pos[1]*delta_t);
	    normal_distribution<double> dist_psi(0, std_pos[2]*delta_t);

	    double sample_x, sample_y, sample_psi;
		sample_x = dist_x(gen);
		sample_y = dist_y(gen);
		sample_psi = dist_psi(gen);

		px_p=px_p+sample_x;
		py_p=py_p+sample_y;
		yaw_p=yaw_p+sample_psi;
        
        particles[i].x=px_p;
        particles[i].y=py_p;
        particles[i].theta=yaw_p;
    }



}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.

}

void ParticleFilter::dataLink(std::vector<LandmarkObs> &predictions, std::vector<LandmarkObs>& observations,std::vector<LandmarkObs> &transformed_landmark_list){
          for(int i = 0; i < observations.size(); i++){
          	   
          	   LandmarkObs observedmark = observations[i];
               
               int min_dist=INT_MAX;
               int min_index=0;
               
               for(int j = 0; j < transformed_landmark_list.size(); j++){
               	   int temp=dist(observedmark.x,observedmark.y,transformed_landmark_list[j].x,transformed_landmark_list[j].y);
               	   if(temp<min_dist){
                        min_index=j;
                        min_dist=temp;
               	   }
               }

               //const LandmarkObs &t=transformed_landmark_list[min_index];

               predictions.push_back(transformed_landmark_list[min_index]);
          }      

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		std::vector<LandmarkObs> observations, Map map_landmarks) {
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
    

    for(int i=0;i<particles.size();i++){
          std::vector<LandmarkObs> transformed_landmark_list;
          for(int j=0;j<map_landmarks.landmark_list.size();j++){
          	    double theta=particles[i].theta;

          	    LandmarkObs temp;
          	    double cos_value=cos(-theta);
          	    double sin_value=sin(-theta);

          	    double x=cos_value*(map_landmarks.landmark_list[j].x_f-particles[i].x)-sin_value*(map_landmarks.landmark_list[j].y_f-particles[i].y);
          	    double y=sin_value*(map_landmarks.landmark_list[j].x_f-particles[i].x)+cos_value*(map_landmarks.landmark_list[j].y_f-particles[i].y);

          	    temp.x=x;
          	    temp.y=y;
          	    temp.id=j;
          	    transformed_landmark_list.push_back(temp);

          
          }

          std::vector<LandmarkObs> predictions;
          dataLink(predictions,observations,transformed_landmark_list);
          
          double temp_weight=1;
          for(int j=0;j<observations.size();j++){
               double single_weight=1;
               double denominator=sqrt(2*M_PI)*std_landmark[0]*std_landmark[1];

               double delta_x=observations[j].x-predictions[j].x;
               double delta_y=observations[j].y-predictions[j].y;
               double numerator=exp(0.5*((delta_x*delta_x)/(std_landmark[0]*std_landmark[0])+delta_y*delta_y/(std_landmark[1]*std_landmark[1])));
               
               single_weight=numerator/denominator;
               temp_weight*=single_weight;     
          }

          particles[i].weight=temp_weight;         
          weights[i]=temp_weight;
    }
    
    
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
    double max_weight=0;
    for(int i=0;i<particles.size();i++){
          max_weight=max(max_weight,weights[i]);
    }

    int max2_weight=2*max_weight;

    std::vector<Particle> resampled_particles;

    std :: random_device  seed_gen ; 
    std :: default_random_engine  engine ( seed_gen () );

    std::uniform_real_distribution<double> distribution(0, max2_weight);
    std::uniform_int_distribution<int>  dist(0,particles.size());

    int start=dist(engine);

    for(int i=0;i<particles.size();i++){
         double genvalue=distribution(engine);
         while(genvalue-weights[start]>0){
         	genvalue-=weights[start];
         	start=(start+1)%weights.size();
         }

         Particle temp;
         temp.x=particles[start].x;
         temp.y=particles[start].y;
         temp.theta=particles[start].theta;

         temp.id=i;
         temp.weight=particles[start].weight;
         resampled_particles.push_back(temp);
    }

    particles=resampled_particles;


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
