// Autor: Pengmian Yan
// Last modifified on: March 11, 2019

#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "prediction.h"
#include "jmt.h"
#include "behaviorplanner.h"
#include <ctime>

// for convenience
using nlohmann::json;
using std::string;
using std::vector;
using std::cout;
using std::endl;

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map_extended.csv";

  BehaviorPlanner behaviorplanner;

  double ego_lane = 1.0;
  double timestample = 0.0;
  State saved_state_s;
  State saved_state_d;
  bool car_started = false;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }
  vector<vector<double> > map_waypoints;
  map_waypoints.push_back(map_waypoints_x);
  map_waypoints.push_back(map_waypoints_y);
  map_waypoints.push_back(map_waypoints_s);
  map_waypoints.push_back(map_waypoints_dx);
  map_waypoints.push_back(map_waypoints_dy);

  tk::spline x_s;
  tk::spline y_s;
  tk::spline dx_s;
  tk::spline dy_s;

  x_s.set_points(map_waypoints_s, map_waypoints_x);
  y_s.set_points(map_waypoints_s, map_waypoints_y);
  dx_s.set_points(map_waypoints_s, map_waypoints_dx);
  dy_s.set_points(map_waypoints_s, map_waypoints_dy);

  h.onMessage([&x_s,&y_s,&dx_s,&dy_s, &map_waypoints,
               &car_started, &timestample, &ego_lane, &saved_state_s, &saved_state_d, &behaviorplanner]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event

    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object

          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"]; // degree
          double car_speed = j[1]["speed"]; // mph
          car_speed = car_speed / 2.237; // m/s
          if (!car_started) {
            saved_state_s = {car_s, 0.0, 0.0};
            saved_state_d = {car_d, 0.0, 0.0};
            car_started = true;
          }
          if (car_d > 0.0 && car_d < 4.0) {
            ego_lane = 0;
          }
          else if (car_d >= 4.0 && car_d < 8.0) {
            ego_lane = 1;
          }
          else if (car_d >= 8 && car_d < 12.0) {
            ego_lane = 2;
          }
          else {
            cout << "out of lane" << endl;
            abort();
          }
          vector<double> ego = {car_x, car_y, car_s, car_d, car_yaw, car_speed, ego_lane};

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          unsigned int num_path_points = 50;
          unsigned int num_rest_prev_path_points = previous_path_x.size();
          timestample += 0.02 * (num_path_points - num_rest_prev_path_points);

          // Previous path's end s and d values
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          // 1. Prediction

          double prediction_time = 1.0; // prediction time

          // Find the six vehicles around car and make the prediction in T second
          Environment environment(sensor_fusion, car_s);
          Prediction prediction_1s = environment.predict_environment(map_waypoints, prediction_time);

          // predict the ego-car in T second
          vector<double> prediction_ego = predict_ego(ego, map_waypoints, T);


          // 2. Behavior planning
          if (num_rest_prev_path_points < 10) {
            behaviorplanner.update_behavior(prediction_1s, prediction_ego, timestample);
//            cout << "car_speed: " << 2.237 * ego[5] << endl;
            cout << "car_s: " << car_s << endl;
            string behavior_for_print;
            switch (behaviorplanner.behavior) {
              case 0:
                behavior_for_print = "start_car";
                break;
              case 1:
                behavior_for_print = "keep_lane";
                break;
              case 2:
                behavior_for_print = "lane_change_left";
                break;
              case 3:
                behavior_for_print = "lane_change_right";
                break;
              default: cout << "Unknown Behavior: " << behaviorplanner.behavior << endl;
            }
            cout << "behavior: " << behavior_for_print << endl;
            cout << "safe to change lane?  left: " << ((behaviorplanner.lane_change_left_safe) ? "Yes": "No") << "\t" << "right: "<< ((behaviorplanner.lane_change_right_safe) ? "Yes": "No") << endl;
            cout << "speed limit in three lanes:" << endl;
            cout << "left lane: " << behaviorplanner.left_lane_speed << "\t" <<
                    "middle lane: " << behaviorplanner.middle_lane_speed << "\t" <<
                    "right lane: " << behaviorplanner.right_lane_speed << endl;
          }


          // 3. Trajectorie planning
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          for(unsigned int i = 0; i < num_rest_prev_path_points; i++) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          double speed_limit_real;

          if (num_rest_prev_path_points < 10) {
            // speed control
            if ((car_s > 195.0 && car_s < 240.0) ||
              (car_s > 5750.0 && car_s < 5900.0) ||
              (car_s > 6670.0 && car_s < 6900.0)) {
              speed_limit_real = kSpeedLimit * 0.9;
            }
            else if ((car_s > 1040.0 && car_s < 1300.0) || (car_s > 2880.0 && car_s < 3200.0)) {
            speed_limit_real = kSpeedLimit * 0.8;
            }
            else {
              speed_limit_real = kSpeedLimit;
            }

            // target state for ego-car
            double trajectory_time = 1.0;
            double acc_max = 3.0;
            State target_state_s;
            State target_state_d;
            if (behaviorplanner.behavior == start_car) {
              trajectory_time = 5.0;
              double target_v = 20.0;
              double target_s = saved_state_s.p + target_v * 0.5 * trajectory_time;
              target_state_s = {target_s, target_v, 0.0};
              target_state_d = {saved_state_d.p, 0.0, 0.0};
              std::cout << "start" << std::endl;
            }
            else if (behaviorplanner.behavior == keep_lane) {
              double front_v;
              bool safe = false;
              if (ego_lane == 0.0) {
                front_v = behaviorplanner.left_lane_speed;
                if (front_v >= kSpeedLimit || prediction_1s.car_fl[5] - prediction_ego[2] > kDistanceBuffer) {
                  safe = true;
                }
              }
              else if (ego_lane == 1.0) {
                front_v = behaviorplanner.middle_lane_speed;
                if (front_v >= kSpeedLimit || prediction_1s.car_fm[5] - prediction_ego[2] > kDistanceBuffer) {
                  safe = true;
                }
              }
              else if (ego_lane == 2.0) {
                front_v = behaviorplanner.right_lane_speed;
                if (front_v >= kSpeedLimit || prediction_1s.car_fr[5] - prediction_ego[2] > kDistanceBuffer) {
                  safe = true;
                }
              }

              double target_v =  safe ? speed_limit_real : (front_v - kSpeedBuffer);

              // But if the car in front is too slow, let's go a little faster
              target_v = target_v > kMinSpeed ? target_v : kMinSpeed;

              // acc limiter
              target_v = (target_v > saved_state_s.v + trajectory_time * acc_max) ? (saved_state_s.v + trajectory_time * acc_max) : target_v;
              target_v = (target_v < saved_state_s.v - trajectory_time * acc_max) ? (saved_state_s.v - trajectory_time * acc_max) : target_v;

              // Estimate a safe target distance based on our selected speed
              double target_s =  saved_state_s.p + trajectory_time * 0.5 * (saved_state_s.v + target_v);
              target_state_s = {target_s, target_v, 0.0};
              target_state_d = {saved_state_d.p, 0.0, 0.0};
            }

            else if (behaviorplanner.behavior == lane_change_left) {
              trajectory_time = 2.0;
              double target_v = saved_state_s.v * 0.9;
              double target_s = saved_state_s.p + (target_v + saved_state_s.v) * 0.5 * trajectory_time;
              target_state_s = {target_s, target_v, 0.0};
              target_state_d = {2.0 + behaviorplanner.target_lane * 4.0, 0.0, 0.0};
            }
            else if (behaviorplanner.behavior == lane_change_right) {
              trajectory_time = 2.0;
              double target_v = saved_state_s.v * 0.9;
              double target_s = saved_state_s.p + (target_v + saved_state_s.v) * 0.5 * trajectory_time;
              target_state_s = {target_s, target_v, 0.0};
              target_state_d = {2.0 + behaviorplanner.target_lane * 4.0, 0.0, 0.0};
            }

            JMT jmt_s(saved_state_s, target_state_s, trajectory_time);
            JMT jmt_d(saved_state_d, target_state_d, trajectory_time);
            saved_state_s = target_state_s;
            saved_state_d = target_state_d;
            ego_lane = behaviorplanner.target_lane;

            for(unsigned int i = 1; i <= int(trajectory_time / 0.02); i++) {
              double s_add = jmt_s.get(i * 0.02);
              s_add = fmod(s_add, kMaxS);
              double d_add = jmt_d.get(i * 0.02);

//              vector<double> XY_add = getXY(s_add, d_add, map_waypoints_s_fine, map_waypoints_x_fine,map_waypoints_y_fine);
              double x_add = x_s(s_add) + d_add * dx_s(s_add);
              double y_add = y_s(s_add) + d_add * dy_s(s_add);
              // debug: std::cout << "add on s, d, x, y:  " << s_add <<"\t"  << d_add << "\t"  << x_add << "\t"   << y_add << std::endl;
              next_x_vals.push_back(x_add);
              next_y_vals.push_back(y_add);
            }

          }
          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */


          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }

  h.run();
}
