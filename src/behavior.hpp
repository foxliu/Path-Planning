#ifndef BEHAVIOR_HPP
#define BEHAVIOR_HPP

#include <vector>
#include <string>
#include <cmath>
#include <numeric>
#include "config.hpp"

using namespace std;

class BehaviorPlanner {
public:
  /** The current vehicle lane **/
  int currnet_lane;

  /** The speed of the vehicle in front of the current vehicle **/
  double current_lead_vehicle_speed;

  /** Current vehicle speed **/
  double current_vehicle_speed;

  double target_vehicle_speed;
  vector<vector<double>> lane_scores;
  vector<double> average_scores;

  BehaviorPlanner();

  /** Plan to change lane **/
  int lanePlanner(double s, double d, vector<vector<double> > sensor_fusion);

  /** Calculate the lane according to the value of d **/
  int laneCalc(double d);

  /** Get the distance and speed of the nearest vehicle on the designated lane **/
  vector<double> closestVehicle(double s, int lane, vector<vector<double> > sensor_fusion, bool direction);

  /** Calculate the max score that can be changed to the lane **/
  int laneScore(double s, int lane, vector<vector<double> > sensor_fusion);
};

#endif