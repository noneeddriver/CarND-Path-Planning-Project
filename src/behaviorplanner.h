// Autor: Pengmian Yan
// Last modifified on: March 11, 2019

#ifndef BEHAVIORPLANNER_H_
#define BEHAVIORPLANNER_H_

#include <iostream>
#include <vector>
#include "prediction.h"
#include "helpers.h"

using std::vector;

enum BehaviorType {start_car, keep_lane, lane_change_left, lane_change_right};

class BehaviorPlanner {
 public:
  BehaviorType behavior;
  unsigned int target_lane;
  bool lane_change_left_safe;
  bool lane_change_right_safe;
  double last_lane_change_time;
  double left_lane_speed;
  double middle_lane_speed;
  double right_lane_speed;
  BehaviorPlanner();
  void update_behavior(const Prediction &prediction, const vector<double> &ego, const double &timestample);
};

#endif // BEHAVIORPLANNER_H_
