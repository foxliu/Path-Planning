#include "behavior.hpp"
#include <iostream>

BehaviorPlanner::BehaviorPlanner() {
  this->current_lead_vehicle_speed = VELOCITY;
  this->current_vehicle_speed = 0;
  this->average_scores = {0, 0, 0};
  vector<double> lane_1_score(10);
  vector<double> lane_2_score(10);
  vector<double> lane_3_score(10);
  this->lane_scores = {lane_1_score, lane_2_score, lane_3_score};
}

/*
 * Plan to change lane
 * sensor_fusion: [[car_id, car_map_x, car_map_y, car_x_v, car_y_v, car_s, car_d],...]
 */
int BehaviorPlanner::lanePlanner(double s, double d, vector<vector<double>> sensor_fusion) {
  int lane;
  int new_lane;
  double distance;

  lane = this->laneCalc(d);
  distance = this->closestVehicle(s, lane, sensor_fusion, true)[0];

  this->currnet_lane = lane;

  if (distance > CHANGE_LANE_DISTANCE) {
    new_lane = lane;
    this->target_vehicle_speed = VELOCITY;
  } else {
    new_lane = this->laneScore(s, lane, sensor_fusion);
    vector<double> vehicle = this->closestVehicle(s, new_lane, sensor_fusion, true);
    target_vehicle_speed = vehicle[1];
  }

  if (new_lane == lane) {
    return 0;
  } else if (new_lane < lane) {
    return -LANE_WIDTH;
  } else {
    return LANE_WIDTH;
  }
}

/*
 * Calculate the lane according to the value of d
 */
int BehaviorPlanner::laneCalc(double d) {
  int lane;
  if (d < LANE_WIDTH) {
    lane = 0;
  } else if (d < 2 * LANE_WIDTH) {
    lane = 1;
  } else {
    lane = 2;
  }
  return lane;
}

/*
 * Calculate the max score that can be changed to the lane
 */
int BehaviorPlanner::laneScore(double s, int lane, vector<vector<double>> sensor_fusion) {
  vector<double> scores = {0, 0, 0};
  vector<double> front_vehicle;
  vector<double> back_vehicle;
  this->average_scores = {0, 0, 0};

  for (int i = 0; i < 3; ++i) {
    if (i == lane) {
      scores[i] += 0.5;
    }
    front_vehicle = closestVehicle(s, i, sensor_fusion, true);
    back_vehicle = closestVehicle(s, i, sensor_fusion, false);
    if (front_vehicle[0] > 1000 && back_vehicle[0] > 1000) {
      scores[i] += 5;
    } else {
      scores[i] += 2 / (1 + exp((10 - front_vehicle[0]) / 5)) - 1;
      scores[i] += 2 / (1 + exp((10 - back_vehicle[0]) / 5)) - 1;
      scores[i] += 2 / (1 + exp((3 + this->current_vehicle_speed - front_vehicle[1]) / 3)) - 1;
      scores[i] += 2 / (1 + exp((3 + this->current_vehicle_speed - back_vehicle[1]) / 3)) - 1;
    }
    this->lane_scores[i].push_back(scores[i]);
    if (this->lane_scores[i].size() > 10) {
      this->lane_scores[i].erase(this->lane_scores[i].begin());
    }
    this->average_scores[i] = accumulate(
            this->lane_scores[i].begin(), this->lane_scores[i].end(), (double)0.0) / this->lane_scores[i].size();
  }

  switch (lane) {
    case 0:
      return this->average_scores[0] > this->average_scores[1] ? 0 : 1;
    case 1:
      int z;
      z = this->average_scores[0] > this->average_scores[1] ? 0 : 1;
      return this->average_scores[z] > this->average_scores[2] ? z : 2;
    case 2:
      return this->average_scores[1] > this->average_scores[2] ? 1 : 2;
    default:
      return this->currnet_lane;
  }
}

/*
 * Get the distance and speed of the nearest vehicle on the designated lane
 * bool direction: is it the front vehicle
 * sensor_fusion: [[car_id, car_map_x, car_map_y, car_x_v, car_y_v, car_s, car_d],...]
 */
vector<double>
        BehaviorPlanner::closestVehicle(double s, int lane, vector<vector<double>> sensor_fusion, bool direction) {
  double dist = 10000;
  double velocity = VELOCITY;
  double vehicle_s, vehicle_d, vehicle_v;
  int vehicle_lane;

  for (auto &vehicle : sensor_fusion) {
    vehicle_s = vehicle[5];
    vehicle_d = vehicle[6];
    vehicle_v = sqrt(pow(vehicle[3], 2) + pow(vehicle[4], 2));
    vehicle_lane = laneCalc(vehicle_d);

    if (vehicle_lane == lane) {
      if (direction) {
        if (vehicle_s >= s && vehicle_s - s < dist) {
          dist = vehicle_s - s;
          velocity = vehicle_v;
        }
      } else {
        if (s >= vehicle_s && s - vehicle_s < dist) {
          dist = s - vehicle_s;
          velocity = vehicle_v;
        }
      }
    }
  }

  if (lane == currnet_lane && direction) {
    current_lead_vehicle_speed = velocity;
  }
  return {dist, velocity};
}
