#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>

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

int main()
{
  uWS::Hub h;

  PID pid_steer(0.2, 0.002, 8.0);
  PID pid_speed(0.2, 0.004, 6.0);

  h.onMessage([&pid_steer, &pid_speed](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
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
    std::cout << "CTE: " << cte << " Angle: " << angle << " Speed: " << speed << std::endl;

    json msgJson;
    msgJson["steering_angle"] = steering_value;
    msgJson["throttle"] = throttle_value;
    auto msg = "42[\"steer\"," + msgJson.dump() + "]";
    std::cout << msg << std::endl;
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
