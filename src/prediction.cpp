// Autor: Pengmian Yan
// Last modifified on: March 11, 2019

#include "prediction.h"
#include "helpers.h"

using std::cout;
using std::endl;

// find the six cars around ego
Environment::Environment(const vector<vector<double> > &sensor_fusion, const double &car_s) {
  // i is the sequence number in the sensor_fusion vector instead of the id
  unsigned int i_fl = 999;
  unsigned int i_fm = 999;
  unsigned int i_fr = 999;
  unsigned int i_rl = 999;
  unsigned int i_rm = 999;
  unsigned int i_rr = 999;
  double s_fl = 1000000.0;
  double s_fm = 1000000.0;
  double s_fr = 1000000.0;
  double s_rl = -1000000.0;
  double s_rm = -1000000.0;
  double s_rr = -1000000.0;

  num_left = 0;
  num_middle = 0;
  num_right = 0;

  unsigned int num_vehicle = sensor_fusion.size();
  for (unsigned int i = 0; i < num_vehicle; i++) {
    unsigned int obj_id = sensor_fusion[i][0];
    double obj_x = sensor_fusion[i][1];
    double obj_y = sensor_fusion[i][2];
    double obj_vx = sensor_fusion[i][3];
    double obj_vy = sensor_fusion[i][4];
    double obj_s = sensor_fusion[i][5];
    double obj_d = sensor_fusion[i][6];

    if (obj_d > 0.0 && obj_d < 4.0) {       // left lane
      num_left += 1;
      if (obj_s > car_s) {                  // front left
        if (obj_s < s_fl) {
          s_fl = obj_s;
          i_fl = i;
        }
      }
      else {                                // rear left
        if (obj_s > s_rl) {
          s_rl = obj_s;
          i_rl = i;
        }
      }
    }
    else if (obj_d >= 4.0 && obj_d < 8.0) { // middle lane
      num_middle += 1;
      if (obj_s > car_s) {                  // front middle
        if (obj_s < s_fm) {
          s_fm = obj_s;
          i_fm = i;
        }
      }
      else {                                // rear middle
        if (obj_s > s_rm) {
          s_rm = obj_s;
          i_rm = i;
        }
      }
    }
    else if (obj_d < 12.0) {                // right lane
      num_right += 1;
      if (obj_s > car_s) {                  // front right
        if (obj_s < s_fr) {
          s_fr = obj_s;
          i_fr = i;
        }
      }
      else {                                // rear right
        if (obj_s > s_rr) {
          s_rr = obj_s;
          i_rr = i;
        }
      }
    }
  } // end for
//  std::cout << "obj_num_all_lane " << num_left << "\t" << num_middle << "\t" << num_right << std::endl;

  // Validation
  if (num_left != 0 && i_fl == 999 && i_rl == 999) {
    throw "num_left error";
  }
  if (num_middle != 0 && i_fm == 999 && i_rm == 999) {
    throw "num_middle error";
  }
  if (num_right != 0 && i_fr == 999 && i_rr == 999) {
    throw "num_right error";
  }
//  cout << "Checked " << endl;
  // assign the six cars if exist
  vector<double> Foocar(7, -999.0); // fake car to hold the place
  auto get_car = [sensor_fusion, Foocar](unsigned int &i) -> vector<double> {return ((i != 999) ? (sensor_fusion[i]) : Foocar);};
  car_fl = get_car(i_fl);
  car_fm = get_car(i_fm);
  car_fr = get_car(i_fr);
  car_rl = get_car(i_rl);
  car_rm = get_car(i_rm);
  car_rr = get_car(i_rr);
}



// predict the cars around ego-car in time T second
Prediction Environment::predict_environment(const vector<vector<double> > &map, const double &T) {
  // lambda function to preict a single car
  auto predict_single_car = [map, T](const vector<double> &car) -> vector<double> {
    if (car[0] != -999.0) { // it is no Foorcar
      vector<double> predicted_car;
      predicted_car.push_back(car[0]);                 // id
      predicted_car.push_back(car[1] + T * car[3]);    // x
      predicted_car.push_back(car[2] + T * car[4]);    // y
      predicted_car.push_back(car[3]);                 // dx
      predicted_car.push_back(car[4]);                 // dy
      double car_yaw = atan2(car[4], car[3]);
      vector<double> predicted_car_sd = getFrenet(car[1], car[2], car_yaw, map[0], map[1]);
      predicted_car.push_back(predicted_car_sd[0]);    // s
      predicted_car.push_back(predicted_car_sd[1]);    // d
      return predicted_car;
    }
    else {
      return car;
    }
  };

  Prediction prediction;
  prediction.car_fl = predict_single_car(car_fl);
  prediction.car_fm = predict_single_car(car_fm);
  prediction.car_fr = predict_single_car(car_fr);
  prediction.car_rl = predict_single_car(car_rl);
  prediction.car_rm = predict_single_car(car_rm);
  prediction.car_rr = predict_single_car(car_rr);
  prediction.num_left = num_left;
  prediction.num_middle = num_middle;
  prediction.num_right = num_right;

  return prediction;
  }

// predict the state of ego-car in T seconds
// assuming: the speed and the yaw dont change much
vector<double> predict_ego(const vector<double> &ego, const vector<vector<double> > &map, const double &T) {
  double car_x = ego[0];
  double car_y = ego[1];
  double car_s = ego[2];
  double car_d = ego[3];
  double car_yaw = ego[4]; // degree
  double car_speed = ego[5]; // m/s
  double car_lane = ego[6];

  double car_x_prediction = car_x + car_speed * T * cos(deg2rad(car_yaw));
  double car_y_prediction = car_y + car_speed * T * sin(deg2rad(car_yaw));
  vector<double> car_sd = getFrenet(car_x_prediction, car_y_prediction, deg2rad(car_yaw), map[0], map[1]);
  double car_s_prediction = car_sd[0];
  double car_d_prediction = car_sd[1];

  vector<double> prediction_ego;
  prediction_ego.push_back(car_x_prediction);
  prediction_ego.push_back(car_y_prediction);
  prediction_ego.push_back(car_s_prediction);
  prediction_ego.push_back(car_d_prediction);
  prediction_ego.push_back(car_yaw);
  prediction_ego.push_back(car_speed);
  prediction_ego.push_back(car_lane);

  return prediction_ego;
}

