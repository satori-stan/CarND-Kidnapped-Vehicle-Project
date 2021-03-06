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

void ParticleFilter::prediction(double delta_t, double std_pos[],
                                double velocity, double yaw_rate) {
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

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted,
                                     std::vector<LandmarkObs>& observations) {
  // Find the predicted measurement that is closest to each observed measurement
  // and assign the observed measurement to this particular landmark.
  for (std::vector<LandmarkObs>::iterator observation = observations.begin();
      observation != observations.end(); ++observation) {
    
    double min_distance = std::numeric_limits<double>::max();

    for (std::vector<LandmarkObs>::iterator landmark = predicted.begin();
        landmark != predicted.end(); ++landmark) {

      double distance = dist(observation->x, observation->y, landmark->x,
          landmark->y);

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
    //   with the Homogeneous transformation using the particle predicted
    // position as the reference coordinate system, since we are assuming that
    // the particles are at the car's position and thus the measurements are
    // from the particle's position with the particle's heading.
    std::vector<LandmarkObs> transformed_observations;
    double x_p = particle->x;
    double y_p = particle->y;
    double theta = particle->theta;

    for (std::vector<LandmarkObs>::const_iterator observation =
        observations.begin();
        observation != observations.end();
        ++observation) {

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
    // Take into account an extension to the range consisting on the max
    // deviation of the landmarks plus the max deviation of the sensor.
    // TODO: Add the sensor deviation. Until such a time, the extension will be
    //       duplicated.
    double range_extension = sqrt(pow(std_landmark[0], 2) +
        pow(std_landmark[1], 2)) * 2;

    for (std::vector<Map::single_landmark_s>::const_iterator landmark =
          map_landmarks.landmark_list.begin();
        landmark != map_landmarks.landmark_list.end(); ++landmark) {

      // For the association only use landmarks that are in the sensor's range
      double x_l = landmark->x_f;
      double y_l = landmark->y_f;

      if (dist(x_l, y_l, x_p, y_p) <= sensor_range + range_extension) {
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

      // Match the observation with its landmark
      double x = observation->x;
      double y = observation->y;
      const unsigned int index = static_cast<unsigned>(observation->id - 1);
      double mu_x = map_landmarks.landmark_list[index].x_f;
      double mu_y = map_landmarks.landmark_list[index].y_f;

      // Calculate and aggregate the multivariate Gaussian distribution
      prob *= exp( - (pow(x-mu_x, 2) / pow(sigma_x, 2) / 2 +
                      pow(y-mu_y, 2) / pow(sigma_y, 2) / 2)) /
              (2 * M_PI * sigma_x * sigma_y);
    }

    // TODO: A couple of times, for reasons yet unknown all weights ended up as
    //       zero, even though the previous sample had a reasonable probability.
    //       Figure out the best course of action, possibilities include:
    //         a) Avoid having zero probability (i.e. assign the minimum
    //            fractional value allowed by the data type).
    //         b) Repeat the prediction process.
    //         c) Increase number of particles.
    particle->weight = prob;
    weights[static_cast<unsigned>(particle->id)] = prob;
  }
}

void ParticleFilter::resample() {
  // Resample particles with replacement with probability proportional to their weight. 

  // Currently, resamplig with the discrete distribution directly. It showed a
  // 4% improvement in performance over 5 runs without degradation in the
  // accuracy of the particle filter.
  ResampleWithDiscreteDistribution();
}

void ParticleFilter::ResampleWithWheel() {
  // Placeholder variable.
  std::vector<Particle> new_particles;
  // Initial index selection.
  unsigned int index = static_cast<unsigned>(rand() % num_particles);
  // The measure of how much we move forward in the wheel.
  double beta = 0;
  // Variables for randomization: a generator, the maximum limit, the distribution.
  default_random_engine gen;
  double two_max_weight = 2 * (*std::max_element(weights.begin(), weights.end()));
  uniform_real_distribution<double> random_uniform(0, two_max_weight);
  // Our resampling wheel:
  for (std::vector<Particle>::iterator particle = particles.begin();
      particle != particles.end(); ++particle) {
    // 1) Choose how much we are going to move forward.
    beta = beta + random_uniform(gen);
    // 2) Identify which slice we landed on.
    while (weights[index] < beta) {
      beta -= weights[index];
      // 2.5) Circle back to the beggining of the vector when we reach the end.
      index = index == static_cast<unsigned>(num_particles - 1) ? 0 : index + 1;
    }
    // 3) Add the slice we landed on to the new list.
    new_particles.push_back(particles[index]);
  }
  particles = new_particles;
}

void ParticleFilter::ResampleWithDiscreteDistribution() {
  // Placeholder variable.
  std::vector<Particle> new_particles;
  // Instead of doing the resampling wheel, use a discrete distribution to
  // get random items from the list using the assigned weights.
  default_random_engine gen;
  std::vector<double>::iterator begin = weights.begin();
  discrete_distribution<unsigned int> random_weighted(begin, weights.end());
  // For each of the particle slots...
  for (int i = 0; i < num_particles; ++i) {
    // Get a random index from the weighted distribution,
    unsigned int index = random_weighted(gen);
    // and add the incumbent particle to the new list.
    new_particles.push_back(particles[index]);
  }
  particles = new_particles;
}

Particle ParticleFilter::SetAssociations(
    // particle: the particle to assign each listed association, and
    //           association's (x,y) world coordinates mapping to
    Particle& particle,
    // associations: The landmark id that goes along with each listed association
    const std::vector<int>& associations,
    // sense_x: the associations x mapping already converted to world coordinates
    const std::vector<double>& sense_x,
    // sense_y: the associations y mapping already converted to world coordinates
    const std::vector<double>& sense_y) {

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
