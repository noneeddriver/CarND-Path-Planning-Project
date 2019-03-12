// Autor: Pengmian Yan
// Last modifified on: March 11, 2019

#include "behaviorplanner.h"
#include <iostream>
#include <vector>
#include <math.h>

using std::cout;
using std::endl;
using std::vector;

// constructor
BehaviorPlanner::BehaviorPlanner() {
  behavior = start_car;
  target_lane = 1;
  lane_change_left_safe = true;
  lane_change_right_safe = true;
  last_lane_change_time = -10000.0;
  left_lane_speed = 0.0;
  middle_lane_speed = 0.0;
  right_lane_speed = 0.0;
}

void BehaviorPlanner::update_behavior(const Prediction &prediction, const vector<double> &prediction_ego, const double &timestample) {
  double ego_x = prediction_ego[0];
  double ego_y = prediction_ego[1];
  double ego_s = prediction_ego[2];
  double ego_d = prediction_ego[3];
  double ego_yaw = prediction_ego[4];
  double ego_speed = prediction_ego[5];
  double ego_lane = prediction_ego[6];

  // 1. check the safty of the lane change
  if (timestample > 10.0) {
    lane_change_left_safe = true;
    lane_change_right_safe = true;
  }
  else {
    lane_change_left_safe = false;
    lane_change_right_safe = false;
  }

  if (ego_lane == 0.0) {               // left lane
    lane_change_left_safe = false;
    if ((prediction.car_fm[5] != -999.0 && prediction.car_fm[5] < ego_s + 20.0) || (prediction.car_rm[5] != -999.0 && prediction.car_rm[5] > ego_s - 30.0)) {
      lane_change_right_safe = false;
    }
  }
  else if (ego_lane == 1.0) {         // middle lane
    if ((prediction.car_fl[5] != -999.0 && prediction.car_fl[5] < ego_s + 20.0) || (prediction.car_rl[5] != -999.0 && prediction.car_rl[5] > ego_s - 30.0)) {
      lane_change_left_safe = false;
    }
    if ((prediction.car_fr[5] != -999.0 && prediction.car_fr[5] < ego_s + 20.0) || (prediction.car_rr[5] != -999.0 && prediction.car_rr[5] > ego_s - 30.0)) {
      lane_change_right_safe = false;
    }
  }
  else if (ego_lane == 2.0) {                              // right lane
    lane_change_right_safe = false;
    if ((prediction.car_fm[5] != -999.0 && prediction.car_fm[5] < ego_s + 20.0) || (prediction.car_rm[5] != -999.0 && prediction.car_rm[5] > ego_s - 30.0)) {
      lane_change_left_safe = false;
    }
  }
  else {
    cout << "Error: ego_lane = " << ego_lane << " out of range" <<  endl;
    abort();
  }

  // 2. compute the speed limit of each lane
  // 2.1 speed limit for left lane
  if (prediction.car_fl[5] == -999.0 || prediction.car_fl[5] > ego_s + kForwardlookingRange) {
    left_lane_speed = kSpeedLimit;
  }
  else {
    if (prediction.car_rl[5] == -999.0 || ego_lane == 0.0 || prediction.car_rl[5] < ego_s - kForwardlookingRange) {
      left_lane_speed = sqrt(pow(prediction.car_fl[3],2.0) + pow(prediction.car_fl[4],2.0));
    }
    else {
      left_lane_speed = (sqrt(pow(prediction.car_fl[3],2.0) + pow(prediction.car_fl[4],2.0)) + sqrt(pow(prediction.car_rl[3],2.0) + pow(prediction.car_rl[4],2.0))) * 0.5;
    }
  }
  // 2.2 speed limit for middle lane
  if (prediction.car_fm[5] == -999.0 || prediction.car_fm[5] > ego_s + kForwardlookingRange) {
    middle_lane_speed = kSpeedLimit;
  }
  else {
    if (prediction.car_rm[5] == -999.0 || ego_lane == 1.0 || prediction.car_rm[5] < ego_s - kForwardlookingRange) {
      middle_lane_speed = sqrt(pow(prediction.car_fm[3],2.0) + pow(prediction.car_fm[4],2.0));
    }
    else {
      middle_lane_speed = (sqrt(pow(prediction.car_fm[3],2.0) + pow(prediction.car_fm[4],2.0)) + sqrt(pow(prediction.car_rm[3],2.0) + pow(prediction.car_rm[4],2.0))) * 0.5;
    }
  }
  // 2.3 speed limit for right lane
  if (prediction.car_fr[5] == -999.0 || prediction.car_fr[5] > ego_s + kForwardlookingRange) {
    right_lane_speed = kSpeedLimit;
  }
  else {
    if (prediction.car_rr[5] == -999.0 || ego_lane == 2.0  || prediction.car_rl[5] < ego_s - kForwardlookingRange) {
      right_lane_speed = sqrt(pow(prediction.car_fr[3],2.0) + pow(prediction.car_fr[4],2.0));
    }
    else {
      right_lane_speed = (sqrt(pow(prediction.car_fr[3],2.0) + pow(prediction.car_fr[4],2.0)) + sqrt(pow(prediction.car_rr[3],2.0) + pow(prediction.car_rr[4],2.0))) * 0.5;
    }
  }

  // 3.0 compute the factor to remind lane change frequently
  double time_gap_lane_change = timestample - last_lane_change_time;
  double factor = 1.2;
  if (time_gap_lane_change < 4.0) {
    factor = 4.0 - 0.7 * time_gap_lane_change;
  }

  // 4.0 decide the next behavior
  target_lane = int(ego_lane);
  behavior = keep_lane;
  if (ego_lane == 0.0) {
    if (lane_change_right_safe) {
      if ((middle_lane_speed > left_lane_speed * factor && (prediction.car_fm[5] == -999.0 || prediction.car_fm[5] > 12.0 * factor + ego_s)) ||
          prediction.num_right == 0) {
        if (prediction.car_fl[5] - ego_s < 60.0) {
          behavior = lane_change_right;
          last_lane_change_time = timestample;
          target_lane = 1;
        }
      }
    }
  }
  else if (ego_lane == 1.0) {
    if (lane_change_left_safe) {
      if ((left_lane_speed > middle_lane_speed * factor && (prediction.car_fl[5] == -999.0 || prediction.car_fl[5] > 12.0 * factor + ego_s) && left_lane_speed > right_lane_speed) ||
          (prediction.num_left == 0 && ego_speed < 0.8 * kSpeedLimit)) {
        if (prediction.car_fm[5] - ego_s < 60.0) {
          behavior = lane_change_left;
          last_lane_change_time = timestample;
          target_lane = 0;
        }
      }
    }
    else if (lane_change_right_safe) {
      if ((right_lane_speed > middle_lane_speed * factor && (prediction.car_fr[5] == -999.0 || prediction.car_fr[5] > 12.0 * factor + ego_s) && right_lane_speed > left_lane_speed) ||
          (prediction.num_right == 0 && ego_speed < 0.8 * kSpeedLimit)) {
        if (prediction.car_fm[5] - ego_s < 60.0) {
          behavior = lane_change_right;
          last_lane_change_time = timestample;
          target_lane = 2;
        }
      }
    }
  }
  else {
    if (lane_change_left_safe) {
      if ((middle_lane_speed > left_lane_speed * factor && (prediction.car_fm[5] == -999.0 || prediction.car_fm[5] > 12.0 * factor + ego_s)) ||
          prediction.num_left == 0) {
        if (prediction.car_fr[5] - ego_s < 60.0) {
          behavior = lane_change_left;
          last_lane_change_time = timestample;
          target_lane = 1;
        }
      }
    }
  }
}
