#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include <string>

#include "json.hpp"
#include "map.hpp"
#include "sensor_fusion.hpp"
#include "agent.hpp"
#include "behavior.hpp"
#include "trajectory.hpp"




using namespace std;
using json = nlohmann::json;


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


int main() {
  uWS::Hub h;

  string map_file_ = "../data/highway_map.csv";
  Map map(map_file_);
  Agent car;
  BehaviorPlanner behavior_planner;
  Generator generator;
  

  h.onMessage([&map, &car , &behavior_planner, &generator](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
   
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
            
            car.set_state(j[1]["x"] ,j[1]["y"] ,j[1]["s"] ,j[1]["d"] ,j[1]["speed"] ,j[1]["yaw"]);
            
          	vector<vector<double> >  sensor_fusion_data = j[1]["sensor_fusion"];
            
            Vehicles vehicles (sensor_fusion_data);
            
            Behavior new_behavior = behavior_planner.eval_behavior(map, sensor_fusion_data, car);

            vector<double> next_x_vals;
          	vector<double> next_y_vals;
            vector<double> previous_x = j[1]["previous_path_x"];
           	vector<double> previous_y = j[1]["previous_path_y"];
            

            generator.generate_trajectory(new_behavior, car, map, previous_x,
                                          previous_y ,next_x_vals, next_y_vals);
            
            json msgJson;
            msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	//this_thread::sleep_for(chrono::milliseconds(1000));
          	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

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
















































































