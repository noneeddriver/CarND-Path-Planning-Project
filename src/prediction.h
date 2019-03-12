// Autor: Pengmian Yan
// Last modifified on: March 11, 2019

#ifndef PREDICTION_H
#define PREDICTION_H

#include <math.h>
#include <vector>
#include <iostream>
#include "helpers.h"

// for convenience
using std::vector;


struct Prediction {
  vector<double> car_fl;
  vector<double> car_fm;
  vector<double> car_fr;
  vector<double> car_rl;
  vector<double> car_rm;
  vector<double> car_rr;
  unsigned int num_left;
  unsigned int num_middle;
  unsigned int num_right;
};


class Environment {
 public:
  // six cars in the three lanes, one behind ego and one in front of ego car for each lane
  vector<double> car_fl;
  vector<double> car_fm;
  vector<double> car_fr;
  vector<double> car_rl;
  vector<double> car_rm;
  vector<double> car_rr;
  // number of detected cars in the three lanes
  unsigned int num_left;
  unsigned int num_middle;
  unsigned int num_right;

  // constructor
  Environment(const vector<vector<double> > &sensor_fusion, const double &car_s); // find the six cars from the sensor fusion
  Prediction predict_environment(const vector<vector<double> > &map, const double &T);
  };

vector<double> predict_ego(const vector<double> &ego, const vector<vector<double> > &map, const double &T);

#endif  // PREDICTION_H
