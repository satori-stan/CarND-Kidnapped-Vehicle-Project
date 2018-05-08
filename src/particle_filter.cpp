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

  // Number of particles is defined as part of the class construction in the
  // header file.

  // Create a normal (Gaussian) distribution for the initialization parameters'
  // noise
  default_random_engine gen;
  normal_distribution<double> dist_x(x, std[0]);
  normal_distribution<double> dist_y(y, std[1]);
  normal_distribution<double> dist_theta(theta, std[2]);

  // Initializing all particles to first position (which are the
  // x, y and theta estimates delivered by the parameters), adding some
  // gaussian noise (based on the uncertainty of GPS measurements) and setting
  // all weights to 1.
  for (int i = 0; i < num_particles; ++i) {
    Particle p {
      i,  // id
      dist_x(gen),  // x
      dist_y(gen),  // y
      dist_theta(gen),  // theta
      1.0  // weight
    };
    particles.push_back(p);
    weights.push_back(1.0);
  }
  is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
  default_random_engine gen;
  // Predicting the next possition by adding velocity and yaw rate measurements
  // to each particle with random Gaussian noise.
  int i = 0;
  for (std::vector<Particle>::iterator particle = particles.begin();
      particle != particles.end(); ++particle) {

    // Avoid having to look for them too much
    double x_0 = particle->x;
    double y_0 = particle->y;
    double theta_0 = particle->theta;

    // Apply the prediction to each particle
    double theta_f;
    double x_f;
    double y_f;
    if (abs(yaw_rate) < 0.0001) {
      theta_f = theta_0;
      //theta_f = atan2(sin(theta_f), cos(theta_f));  // Normalize the angle
      x_f = x_0 + (velocity * delta_t * cos(theta_f));
      y_f = y_0 + (velocity * delta_t * sin(theta_f));
    } else {
      theta_f = theta_0 + yaw_rate * delta_t;
      //theta_f = atan2(sin(theta_f), cos(theta_f));  // Normalize the angle
      x_f = x_0 + (velocity * (sin(theta_f) - sin(theta_0))) / yaw_rate;
      y_f = y_0 + (velocity * (cos(theta_0) - cos(theta_f))) / yaw_rate;
    }

    // Create a normal (Gaussian) distribution for the control inputs' noise
    // using the predicted position as the mean.
    normal_distribution<double> dist_x(x_f, std_pos[0]);
    normal_distribution<double> dist_y(y_f, std_pos[1]);
    normal_distribution<double> dist_theta(theta_f, std_pos[2]);

    // Set the final prediction to a random point in the distribution's space.
    particle->id = i++;
    particle->x = dist_x(gen);
    particle->y = dist_y(gen);
    particle->theta = dist_theta(gen);
  }

}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
  // TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
  //   observed measurement to this particular landmark.
  for (std::vector<LandmarkObs>::iterator observation = observations.begin();
      observation != observations.end(); ++observation) {
    
    double min_distance = std::numeric_limits<double>::max();

    for (std::vector<LandmarkObs>::iterator landmark = predicted.begin();
        landmark != predicted.end(); ++landmark) {
      double distance = dist(observation->x, observation->y, landmark->x, landmark->y);
      if (distance < min_distance) {
        min_distance = distance;
        observation->id = landmark->id;
      }
    }
  }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
    const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
  // TODO: Review the option of implementing using Eigen lib.
  // 1. Update the weight of every particle
  for (std::vector<Particle>::iterator particle = particles.begin();
      particle != particles.end(); ++particle) {

    // 2. Use unified coordinate system: Observations are in vehicle coordinates
    //   while the landmarks are in map coordinates, so transform observations
    //   with the Homogeneous transformation using the particle predicted position
    //   as the reference coordinate system, since we are assuming that the
    //   particles are at the car's position and thus the measurements are from
    //   the particle's position with the particle's heading.
    std::vector<LandmarkObs> transformed_observations;
    double x_p = particle->x;
    double y_p = particle->y;
    double theta = particle->theta;

    for (std::vector<LandmarkObs>::const_iterator observation = observations.begin();
        observation != observations.end(); ++observation) {

      LandmarkObs o;
      o.id = observation->id;
      double x_c = observation->x;
      double y_c = observation->y;
      /* x_m */ o.x = x_p + cos(theta) * x_c - sin(theta) * y_c;
      /* y_m */ o.y = y_p + sin(theta) * x_c + cos(theta) * y_c;
      transformed_observations.push_back(o);
    }

    // 3. Discard landmarks that are beyond the sensor's range to make the rest
    //   of the process more efficient.
    std::vector<LandmarkObs> predicted_landmarks;
    for (std::vector<Map::single_landmark_s>::const_iterator landmark =
          map_landmarks.landmark_list.begin();
        landmark != map_landmarks.landmark_list.end(); ++landmark) {
      // For the association only use landmarks that are in the sensor's range
      double x_l = landmark->x_f;
      double y_l = landmark->y_f;
      if (dist(x_l, y_l, x_p, y_p) <= 2 * sensor_range) {  // XXX: Plus some noise (of the sensor or the landmark?)
        // TODO: Should be able to do this by copying directly.
        LandmarkObs o;
        o.id = static_cast<int>(landmark->id_i);
        o.x = x_l;
        o.y = y_l;
        predicted_landmarks.push_back(o);
      }
    }
    
    // 4. Associate the observations with a specific landmark.
    dataAssociation(predicted_landmarks, transformed_observations);

    // 5. Calculate the probability of each observation
    double prob = 1;
    double sigma_x = std_landmark[0];
    double sigma_y = std_landmark[1];

    for (std::vector<LandmarkObs>::iterator observation = transformed_observations.begin();
        observation != transformed_observations.end(); ++observation) {

      double x;
      double y;
      double mu_x;
      double mu_y;

      // TODO: Do it without a loop. The landmark id = landmark index + 1
      for (std::vector<LandmarkObs>::iterator landmark = predicted_landmarks.begin();
          landmark != predicted_landmarks.end(); ++landmark) {
        if (landmark->id == observation->id) {
          x = observation->x;
          y = observation->y;
          mu_x = landmark->x;
          mu_y = landmark->y;
          break;
        }
      }
      /*
      x = observation->x;
      y = observation->y;
      mu_x = predicted_landmarks[observation->id - 1].x;
      mu_y = predicted_landmarks[observation->id - 1].y;
      */

      prob *= exp( - (pow(x-mu_x, 2) / pow(sigma_x, 2) / 2 + pow(y-mu_y, 2) / pow(sigma_y, 2) / 2)) / (2 * M_PI * sigma_x * sigma_y);
    }

    particle->weight = prob;
    weights[static_cast<unsigned>(particle->id)] = prob;
  }
}

void ParticleFilter::resample() {
  // TODO: Resample particles with replacement with probability proportional to their weight. 
  // NOTE: You may find std::discrete_distribution helpful here.
  //   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
  
  /*
  The resampling wheel as used in the lectures
  int index = (rand() % num_particles); // random.randint(0, N-1)
  int beta = 0;
  double two_max_weight = 2 *
      std::max_element(particles.begin(), particles.end(), 
          [](const Particle& p1, const Particle& p2) {
        return p1.weight < p2.weight;
      })->weight;
 double two_max_weight = 2 * (*std::max_element(weights.begin(), weights.end()));
  for (std::vector<Particle>::iterator particle = particles.begin();
      particle != particles.end(); ++particle) {
    beta += random_uniform(gen);
    while w[index] < beta:
        beta = beta - w[index]
        index = 0 if index == N-1 else index + 1

    p3.append(p[index])
  }
  */
  /*
  Original
  std::vector<Particle> new_particles;
  // Instead of doing the resampling wheel, use a discrete distribution to
  // get random items from the list using the assigned weights.
  default_random_engine gen;
  std::vector<double>::iterator begin = weights.begin();
  discrete_distribution<unsigned int> random_uniform(begin, weights.end());
  for (int i = 0; i < num_particles; ++i) {
    // Calculate the index from the iterator in order to find the correct particle
    //int index = std::distance(begin, random_uniform(gen));
    //new_particles.push_back(particles[static_cast<unsigned>(index)]);

    unsigned int index = random_uniform(gen);
    new_particles.push_back(particles[index]);
    //new_particles.push_back(particles[random_uniform(gen)]);
  }
  */
  std::vector<Particle> new_particles;
  unsigned int index = static_cast<unsigned>(rand() % num_particles);
  double beta = 0;
  default_random_engine gen;
  double two_max_weight = 2 * (*std::max_element(weights.begin(), weights.end()));
  uniform_real_distribution<double> random_uniform(0, two_max_weight);
  for (std::vector<Particle>::iterator particle = particles.begin();
      particle != particles.end(); ++particle) {
    beta = beta + random_uniform(gen);
    while (weights[index] < beta) {
      beta -= weights[index];
      index = index == static_cast<unsigned>(num_particles - 1) ? 0 : index + 1;
    }
    new_particles.push_back(particles[index]);
  }
  /*
  for (std::vector<Particle>::iterator particle = particles.begin();
      particle != particles.end(); ++particle) {
    beta += random_uniform(gen);
    while w[index] < beta:
        beta = beta - w[index]
        index = 0 if index == N-1 else index + 1

    p3.append(p[index])
  }
  */
  particles = new_particles;
}

Particle ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations, 
                                     const std::vector<double>& sense_x, const std::vector<double>& sense_y) {
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates

    particle.associations= associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;
    return particle;
}

string ParticleFilter::getAssociations(Particle best) {
  vector<int> v = best.associations;
  stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}

string ParticleFilter::getSenseX(Particle best) {
  vector<double> v = best.sense_x;
  stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}

string ParticleFilter::getSenseY(Particle best) {
  vector<double> v = best.sense_y;
  stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
