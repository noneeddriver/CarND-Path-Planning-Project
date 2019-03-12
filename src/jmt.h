// Autor: Pengmian Yan
// Last modifified on: March 11, 2019

#ifndef JMT_H_
#define JMT_H_

#include <iostream>
#include "../src/Eigen-3.3/Eigen/Core"
#include "../src/Eigen-3.3/Eigen/QR"
#include "../src/Eigen-3.3/Eigen/Dense"


// State - stores three doubles p, v, a
// intended to store position, velocity, and acceleration components in the s, or d axis

struct State {
  double p;
  double v;
  double a;
};


class JMT {

  public:
    Eigen::VectorXd c;
    JMT(const State& start, const State& end, const double t);
    double get(const double t) const;
};

#endif // JMT_H_
