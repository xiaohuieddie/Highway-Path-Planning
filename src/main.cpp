#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"
// #include "vehicle.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

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
  
  // start at lane 1, which is the middle lane
  int lane = 1;
  
  // Have a reference velocity to target
  double ref_vel = 0; // mph, very clost to the speed limit 50 mph
  
  // starting state is "KL" - Keep Lane
  string state = "KL";

  h.onMessage([&state, &ref_vel, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy, &lane]
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
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner, previous path is not the fully previous path, it only contains the remaining points from previous path that were not eaten up by the car
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          int prev_size = previous_path_x.size();
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];
          vector<Vehicle> surrond_vehicles;
          for(int i = 0; i < sensor_fusion.size(); i++){
            double x = sensor_fusion[i][1];
            double y = sensor_fusion[i][2];
            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];
            double s = sensor_fusion[i][5];
            double d = sensor_fusion[i][6];
            Vehicle vehicle{x, y, vx, vy, s, d};
            surrond_vehicles.push_back(vehicle);
          }
          std::cout << surrond_vehicles.size() << std::endl;

          json msgJson;

          // ---------------------------------------------------------------------------------------------

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
          if (prev_size > 0){
            car_s = end_path_s;  
          }
          // Prediction : Analysing other cars positions.
          bool car_ahead = false;
          bool car_left = false;
          bool car_righ = false;
          bool too_close = false;
          bool too_far = true;

          for ( int i = 0; i < sensor_fusion.size(); i++ ) {
            float d = sensor_fusion[i][6];
            int car_lane = -1;
            // is it on the same lane we are
            if ( d > 0 && d < 4 ) {
              car_lane = 0;
            } else if ( d > 4 && d < 8 ) {
              car_lane = 1;
            } else if ( d > 8 && d < 12 ) {
              car_lane = 2;
            }
            if (car_lane < 0) {
              continue;
            }
            // Find car speed.
            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];
            double check_speed = sqrt(vx*vx + vy*vy);
            double check_car_s = sensor_fusion[i][5];
            // Estimate car s position after executing previous trajectory.
            check_car_s += ((double)prev_size*0.02*check_speed);

            if ( car_lane == lane ) {
              // Car in our lane.
              car_ahead |= check_car_s > car_s && check_car_s - car_s < 30;
              too_close |= check_car_s > car_s && check_car_s - car_s < 15;
              if ((check_car_s > car_s) && ((check_car_s - car_s) < 200)){
                too_far = false; 
              }
            } else if ( car_lane - lane == -1 ) {
              // Car left
              car_left |= car_s - 30 < check_car_s && car_s + 30 > check_car_s;
            } else if ( car_lane - lane == 1 ) {
              // Car right
              car_righ |= car_s - 30 < check_car_s && car_s + 30 > check_car_s;
            }
          }

          // Behavior : Let's see what to do.

          if ( car_ahead ) { // Car ahead
            if ( !car_left && lane > 0 ) {
              // if there is no car left and there is a left lane.
              lane--; // Change lane left.
            } else if ( !car_righ && lane != 2 ){
              // if there is no car right and there is a right lane.
              lane++; // Change lane right.
            } else {
              if (too_close){
                ref_vel -= 0.4; 
              } else{
                ref_vel -= 0.224;
              }
            }
          } else {
            if ( lane != 1 ) { // if we are not on the center lane.
              if ( ( lane == 0 && !car_righ ) || ( lane == 2 && !car_left ) ) {
                lane = 1; // Back to center.
              }
            }
            if ( ref_vel < 49.5) {
              if (too_far){
                ref_vel += 0.5;
              } else{
                ref_vel += 0.224;
              }
            }
          }
          /* Cost Function approach
          // Find next possible states
          vector<string> possible_successor_states = successor_states(lane, state);
          
          // Create a cost list to include the cost for each possible state tranfer
          vector<double> costs;
          float cost;
          for ( int i = 0; i < possible_successor_states.size(); ++i){
            std::cout << possible_successor_states[i]<<std::endl;
            
          }
          for ( int i = 0; i < possible_successor_states.size(); ++i){
            std::cout << possible_successor_states[i]<<std::endl;
            cost = calculate_cost(possible_successor_states[i], surrond_vehicles, lane, 49.5);
            costs.push_back(cost);
          }
          
          // Find the best successor state, which has the least cost value
          float min_cost = 9999999.9;
          string best_successor_state;
          for (int i = 0; i < costs.size(); ++i) {
            if(costs[i] < min_cost){
              min_cost = costs[i];
              best_successor_state = possible_successor_states[i];
            }
          }
          std::cout << "best state" << best_successor_state << "cost" << min_cost;
          
          // execute the best successor state
          if (best_successor_state.compare("LCR") == 0){
            lane += 1;
          }
          else if (best_successor_state.compare("LCL") == 0){
            lane -= 1;
          }
          
          //update state variable
          state = best_successor_state;
          */

           
          // Create a list of widely spaced (x, y) waypoints, evenly spaced at 30m
          // Later we will interpolate these waypoints with a spline and fill it in with more points that control speed
          vector<double> ptsx;
          vector<double> ptsy;
          
          // reference x, y, yaw_rate
          // either we will reference the starting point as where the car is OR at the previous paths end point
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);
          
          // if previous size is almost empty when we are starting the vehicle, use the car as the staring reference
          if (prev_size < 2){
            // Use two points that make the path tangent to the current car position
            double prev_car_x = car_x - 1 * cos(car_yaw);
            double prev_car_y = car_y - 1 * sin(car_yaw);
            
            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);
            
            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);
          }
          // Else use the previous path end point as the starting reference
          else{
            //redefine the reference point as previous path end point
            ref_x = previous_path_x[prev_size - 1];
            ref_y = previous_path_y[prev_size - 1];
            
            double prev_ref_x = previous_path_x[prev_size - 2];
            double prev_ref_y = previous_path_y[prev_size - 2];
            ref_yaw = atan2(ref_y - prev_ref_y, ref_x - prev_ref_x);
            
            // Use two points that make the path tangent to the previous path's end points
            ptsx.push_back(prev_ref_x);
            ptsx.push_back(ref_x);

            ptsy.push_back(prev_ref_y);
            ptsy.push_back(ref_y);
            
          }
          
          // In Frenet add evenly 30m spaced points ahead of the starting reference (0 --> 30m --> 60m --> 90m)
          vector<double> next_wp0 = getXY(car_s+30, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s+60, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s+90, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          
          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);
          
          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);
          
          // To make math easier, Shift car reference angle to 0 degrees. It means we are consider the car as the origin of our new coordinate 
          for (int i = 0; i < ptsx.size(); ++i){
            double shift_x = ptsx[i] - ref_x;
            double shift_y = ptsy[i] - ref_y;
            
            ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
            ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
          }
          
          // create a spline
          tk::spline s;
          
          for (int i = 0; i< ptsx.size(); i++){
            std::cout << ptsx[i] << "  ";
          }
          std::cout << std::endl;
          
          // set (x,y) points to the spline
          s.set_points(ptsx, ptsy);
          
          // Define the actual (x,y) points we will use for the planner
          vector<double> next_x_vals;
          vector<double> next_y_vals;
          
          // Start with all of the previous path points from last time
          for (int i = 0; i < previous_path_x.size(); ++i){
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }
          
          // Calculate how to break up spline points so that we travel at our desired reference velocity
          double target_x = 30.0;
          double target_y = s(target_x);
          double target_dist = sqrt((target_x) * (target_x) + (target_y) * (target_y));
          
          double x_add_on = 0;
          
          // after filling it with previous points, we need to fill up the rest of our path planner. Here we will always output 50 points each cycle
          for (int i = 1; i < 50 - previous_path_x.size(); i++){
            double N = (target_dist / (0.02 * ref_vel / 2.24));  // From reference point to target point, the distance is evenly spaced into N parts; 2.24 is to convert mph to m/s
            double x_point = x_add_on + target_x / N;
            double y_point = s(x_point);
            
            x_add_on = x_point;
            
            double x_ref = x_point;
            double y_ref = y_point;
            
            // rotate back to normal because we rotated it eariler
            x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
            y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));
            
            x_point += ref_x;
            y_point += ref_y;
            
            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }
          
          // -----------------------------------------------------------------------------------


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