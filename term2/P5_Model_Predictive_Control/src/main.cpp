#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <vector>
#include <chrono>

#include "Eigen"

#include "MPC.h"
#include "json.hpp"
#include "tools.h"

// for convenience
using json = nlohmann::json;

string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}


int main() {
  uWS::Hub h;

  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    string sdata = string(data).substr(0, length);
    cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        json j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          
          // Extracts variables from input json message
          Eigen::VectorXd x_traj, y_traj;
          tools::manage_input(j, x_traj, y_traj);

          // Fits the trajectory points to a polynomial
          Eigen::VectorXd coeffs = tools::polyfit(x_traj, y_traj, 3);

          // Initializes state in vehicle frame
          Eigen::VectorXd state = tools::build_init_state(j, coeffs);
         
          // Solves the mpc optimization and extracts optimal actuator values
          std::vector<double> mpc_x_traj, mpc_y_traj;
          auto result = mpc.Solve(state, coeffs, mpc_x_traj, mpc_y_traj);
          double steer_value = result[0]/ tools::deg2rad(25);
          double throttle_value = result[1];
          
          // Builds answer message and sends it to the simulator
          json msgJson = tools::manage_output(steer_value, throttle_value, x_traj, y_traj, mpc_x_traj, mpc_y_traj);
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
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
