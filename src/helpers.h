#ifndef HELPERS_H
#define HELPERS_H

#include <math.h>
#include <string>
#include <vector>
#include <map>
#include "vehicle.h"

// for convenience
using std::string;
using std::vector;
using std::map;



// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
//   else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

//
// Helper functions related to waypoints and converting from XY to Frenet
//   or vice versa
//

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Calculate distance between two points
double distance(double x1, double y1, double x2, double y2) {
  return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

// Calculate closest waypoint to current x, y position
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, 
                    const vector<double> &maps_y) {
  double closestLen = 100000; //large number
  int closestWaypoint = 0;

  for (int i = 0; i < maps_x.size(); ++i) {
    double map_x = maps_x[i];
    double map_y = maps_y[i];
    double dist = distance(x,y,map_x,map_y);
    if (dist < closestLen) {
      closestLen = dist;
      closestWaypoint = i;
    }
  }

  return closestWaypoint;
}

// Returns next waypoint of the closest waypoint
int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, 
                 const vector<double> &maps_y) {
  int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

  double map_x = maps_x[closestWaypoint];
  double map_y = maps_y[closestWaypoint];

  double heading = atan2((map_y-y),(map_x-x));

  double angle = fabs(theta-heading);
  angle = std::min(2*pi() - angle, angle);

  if (angle > pi()/2) {
    ++closestWaypoint;
    if (closestWaypoint == maps_x.size()) {
      closestWaypoint = 0;
    }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, 
                         const vector<double> &maps_x, 
                         const vector<double> &maps_y) {
  int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

  int prev_wp;
  prev_wp = next_wp-1;
  if (next_wp == 0) {
    prev_wp  = maps_x.size()-1;
  }

  double n_x = maps_x[next_wp]-maps_x[prev_wp];
  double n_y = maps_y[next_wp]-maps_y[prev_wp];
  double x_x = x - maps_x[prev_wp];
  double x_y = y - maps_y[prev_wp];

  // find the projection of x onto n
  double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
  double proj_x = proj_norm*n_x;
  double proj_y = proj_norm*n_y;

  double frenet_d = distance(x_x,x_y,proj_x,proj_y);

  //see if d value is positive or negative by comparing it to a center point
  double center_x = 1000-maps_x[prev_wp];
  double center_y = 2000-maps_y[prev_wp];
  double centerToPos = distance(center_x,center_y,x_x,x_y);
  double centerToRef = distance(center_x,center_y,proj_x,proj_y);

  if (centerToPos <= centerToRef) {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for (int i = 0; i < prev_wp; ++i) {
    frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
  }

  frenet_s += distance(0,0,proj_x,proj_y);

  return {frenet_s,frenet_d};
}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, 
                     const vector<double> &maps_x, 
                     const vector<double> &maps_y) {
  int prev_wp = -1;

  while (s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1))) {
    ++prev_wp;
  }

  int wp2 = (prev_wp+1)%maps_x.size();

  double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),
                         (maps_x[wp2]-maps_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s-maps_s[prev_wp]);

  double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
  double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

  double perp_heading = heading-pi()/2;

  double x = seg_x + d*cos(perp_heading);
  double y = seg_y + d*sin(perp_heading);

  return {x,y};
}

// Helper function to Uses the current state to return a vector of possible successor states for the finite state machine.
vector<string> successor_states(int lane, string state) {
  // Provides the possible next states given the current state for the FSM 
  //   discussed in the course, with the exception that lane changes happen 
  //   instantaneously, so LCL and LCR can only transition back to KL.
  vector<string> states;
  states.push_back("KL");
  if(state.compare("KL") == 0) {
    if (lane == 1){
      states.push_back("PLCL");
      states.push_back("PLCR");
    }
    else if (lane == 0){
      states.push_back("PLCR");
    }
    else if (lane == 2){
      states.push_back("PLCL");
    }
  } 
  else if (state.compare("PLCL") == 0) {
    if (lane - 1 >= 0) {
      states.push_back("PLCL");
      states.push_back("LCL");
    }
  } 
  else if (state.compare("PLCR") == 0) {
    if (lane != 2) {
      states.push_back("PLCR");
      states.push_back("LCR");
    }
  }
    
  // If state is "LCL" or "LCR", then just return "KL"
  return states;
}

vector<double> find_lane_speeds(vector<Vehicle> surrond_vehicles){
  vector<double> lane_speeds = {55.0, 55.0, 55.0};
  map<int, vector<int>> lane_vehicles;
  for(int i = 0; i < surrond_vehicles.size(); i++){
    // Car is in my lane
    double d = surrond_vehicles[i].d;
    double vx = surrond_vehicles[i].vx;
    double vy = surrond_vehicles[i].vy;
    double check_speed = sqrt(vx*vx + vy*vy);
    int lane_find = -1;
    
    // Find which lane the vehicle is at
    for (int lane = 0; lane < 3; lane++){
      if ( d < (2 + 4*lane + 2) && d >= (2 + 4*lane - 2)){
        lane_vehicles[lane].push_back(i);
        lane_find = lane;  
        std::cout << "check speed" << check_speed << std::endl;
      }
    }
    
    // The slowest vehicle in each lane represent the lane speed
    if (check_speed < lane_speeds[lane_find]){
      lane_speeds[lane_find] = check_speed;
    }
    
  }
  
  
  // debug
  std::cout << "lane_speeds:" << lane_speeds[0]  << "  " << lane_speeds[1] << "  "  << lane_speeds[2] << std::endl;
  
  return lane_speeds;
}

double inefficiency_cost(string possible_successor_state, int cur_lane, vector<double> lane_speeds, double target_speed){
  double intended_lane_spd;
  double final_lane_spd;
  if (possible_successor_state.compare("KL") == 0){
    intended_lane_spd = lane_speeds[cur_lane];
    final_lane_spd = lane_speeds[cur_lane];
  }
  else if (possible_successor_state.compare("PLCL") == 0){
    intended_lane_spd = lane_speeds[cur_lane - 1];
    final_lane_spd = lane_speeds[cur_lane];
  }
  else if (possible_successor_state.compare("PLCR") == 0){
    intended_lane_spd = lane_speeds[cur_lane + 1];
    final_lane_spd = lane_speeds[cur_lane];
  }
  else if (possible_successor_state.compare("LCR") == 0){
    intended_lane_spd = lane_speeds[cur_lane + 1];
    final_lane_spd = lane_speeds[cur_lane + 1];
  }
  else if (possible_successor_state.compare("LCL") == 0){
    intended_lane_spd = lane_speeds[cur_lane - 1];
    final_lane_spd = lane_speeds[cur_lane - 1];
  }

  double cost = (2.0 * target_speed - intended_lane_spd - final_lane_spd) / target_speed;
  
  return cost;
}

double calculate_cost(string possible_successor_state, vector<Vehicle> surrond_vehicles, int cur_lane, double target_speed){
  vector<double> lane_speeds = find_lane_speeds(surrond_vehicles);
  double speed_cost = inefficiency_cost(possible_successor_state, cur_lane, lane_speeds, target_speed);
  std::cout<< "cost:" << speed_cost << std::endl;
  return speed_cost;
}
#endif  // HELPERS_H