#include <uWS/uWS.h>
#include <iostream>
#include <vector>
#include "json.hpp"
#include "PID.h"
#include <math.h>

using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

const double constant_speed = 20;
const double max_turn_angle = deg2rad(25);

const int twiddler_steps = 125;

int main()
{
  uWS::Hub h;

  PID pid_speed(0.2, 0.004, 6.0);

  int run_type = 0;
  int which_param = 0; // 0: kp, 1: ki, 2: kd
  /* Run type: 
     0: first run
     1: dp[which_param] increased previously
     2: dp[which_param] decreased previously
   */

  int n = 0;
  double curr_err = 0;
  double best_err;

  vector<double> p = {0, 0, 0};
  vector<double> dp = {1, 1, 1};
  PID pid_steer(p[0], p[1], p[2]);

  h.onMessage([&]
    (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (!(length && length > 2 && data[0] == '4' && data[1] == '2')) {
      return;
    }
    
    auto s = hasData(std::string(data).substr(0, length));
    if (s == "") {
      // Manual driving
      std::string msg = "42[\"manual\",{}]";
      ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      return;
    }

    auto j = json::parse(s);
    std::string event = j[0].get<std::string>();
    if (event != "telemetry") {
      return;
    }

    // j[1] is the data JSON object
    const double cte = std::stod(j[1]["cte"].get<std::string>());
    const double speed = std::stod(j[1]["speed"].get<std::string>());
    const double angle = std::stod(j[1]["steering_angle"].get<std::string>());
    
    pid_steer.UpdateError(cte);
    double steering_value = -pid_steer.TotalError();
    // Cap steering value to the maxium turing angle of 25 degrees.
    if (abs(steering_value) > max_turn_angle) {
      steering_value = steering_value > 0 ? max_turn_angle : -max_turn_angle;
    }
    // Scale it back to [-1, 1]
    steering_value /= max_turn_angle;

    pid_speed.UpdateError(speed - constant_speed);
    double throttle_value = -pid_speed.TotalError();
    
    // DEBUG
    // std::cout << "CTE: " << cte << " Angle: " << angle << " Speed: " << speed << std::endl;

    n += 1;
    // Let the simulator run for the first twidder_steps
    // Accumulate errors for the second twiddler_steps
    if (n > twiddler_steps) {
      curr_err += cte * cte;
    }

    std::string msg;
    // At 2 * twiddler_steps, we make a decision and reset the simulator for another run.
    if (n > 2 * twiddler_steps) {
      cout <<"------------BEFORE-----------\n";
      cout << "best_error: " << best_err << " curr_error: " << curr_err << endl;
      cout << "run_type: " << run_type << " which_param: " << which_param << endl;
      cout << "p: ";
      for (auto x : p) cout << x << ", ";
      cout << endl << "dp: ";
      for (auto x : dp) cout << x << ", ";
      cout << endl;


      if (run_type == 0) {
        best_err = curr_err;
        run_type = 1;
        which_param = 0;
        p[which_param] += dp[which_param];
      } else if (run_type == 1) {
        if (curr_err < best_err) {
          best_err = curr_err;
          dp[which_param] *= 1.1;

          // move to the next param
          run_type = 1;
          which_param = (which_param + 1) % p.size();
          p[which_param] += dp[which_param];
        } else {
          run_type = 2;
          p[which_param] -= 2.0 * dp[which_param];
        }
      } else if (run_type == 2) {
        if (curr_err < best_err) {
          best_err = curr_err;
          dp[which_param] *= -1.1;
        } else {
          // reset p[which_param] and dp[which_param]
          p[which_param] += dp[which_param];
          dp[which_param] /= 1.1;
        }

        // move to the next param
        run_type = 1;
        which_param = (which_param + 1) % p.size();
        p[which_param] += dp[which_param];
      }

      cout <<"------------AFTER-----------\n";
      cout << "best_error: " << best_err << " curr_error: " << curr_err << endl;
      cout << "run_type: " << run_type << " which_param: " << which_param << endl;
      cout << "p: ";
      for (auto x : p) cout << x << ", ";
      cout << endl << "dp: ";
      for (auto x : dp) cout << x << ", ";
      cout << endl;

      // Reset simulator and update controllers.
      curr_err = 0;
      n = 0;
      pid_steer.ClearErrors();
      pid_steer.SetParams(p[0], p[1], p[2]);
      pid_speed.ClearErrors();
      msg = "42[\"reset\",{}]";
    } else {
      json msgJson;
      msgJson["steering_angle"] = steering_value;
      msgJson["throttle"] = throttle_value;
      msg = "42[\"steer\"," + msgJson.dump() + "]";
    }
    // std::cout << n << ": msg=" << msg << std::endl;
    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
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
