#include <uWS/uWS.h>
#include <iostream>
#include <math.h>
#include <fstream>

#include "json.hpp"
#include "fusion.h"
#include "tools.h"
#include "dynamics.h"
#include "filter.h"

#include "Eigen/Dense"

std::ofstream output("../data/output_radar.txt"); 

using namespace Eigen;
using namespace std;

// for convenience
using json = nlohmann::json;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main()
{
  uWS::Hub h;
  SensorFusion<UFK, CTRV> fusion;

  Tools tools;
  vector<VectorXd> estimations;
  vector<VectorXd> ground_truth;

  h.onMessage([&fusion,&tools,&estimations,&ground_truth](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event

    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {

      auto s = hasData(std::string(data));
      if (s != "") {
      	
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        
        if (event == "telemetry") {
            
          string sensor_measurement = j[1]["sensor_measurement"];
          MeasurementPackage meas_package;
          tools.EncodeLine(meas_package, ground_truth, sensor_measurement);
          cout << sensor_measurement << endl << endl;
          double x = 0, y = 0, epsilon;
          VectorXd RMSE = VectorXd::Zero(4);
          
          if (meas_package.sensor_type_ == MeasurementPackage::RADAR){
          //if (true){
            fusion.ProcessMeasurement(meas_package);    	  
            
            x = fusion.X_(0);
            y = fusion.X_(1);
            epsilon = fusion.epsilon;
            double vx  = fusion.X_(2)*std::cos(fusion.X_(3));
            double vy = fusion.X_(2)*std::sin(fusion.X_(3));
            VectorXd estimate(4);
            estimate << x , y , vx, vy;
            estimations.push_back(estimate);
            RMSE = tools.CalculateRMSE(estimations, ground_truth);
            tools.write_output(output, meas_package.raw_measurements_,ground_truth[ground_truth.size()-1], estimate, RMSE, epsilon);
            
          }
          else{
            ground_truth.pop_back();
          }
              
          json msgJson;
          msgJson["estimate_x"] = x;
          msgJson["estimate_y"] = y;
          msgJson["rmse_x"] =  RMSE(0);
          msgJson["rmse_y"] =  RMSE(1);
          msgJson["rmse_vx"] = RMSE(2);
          msgJson["rmse_vy"] = RMSE(3);
          
          auto msg = "42[\"estimate_marker\"," + msgJson.dump() + "]";
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);


        }
      } else {
        
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }

  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}























































































