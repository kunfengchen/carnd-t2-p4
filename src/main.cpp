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

int main()
{
  uWS::Hub h;

  PID pid;
  // DONE: Initialize the pid variable.

  /**
   * How to setup pid (proportional-integral-derivative)
   * coefficient Kp, Ki, and Kd values
   * First time I set (Kp, Ki, Kd) to like (0.5, 0.3. 0.3),
   * The car just drive in a circle, after tried different
   * values, it still ran on the circle.
   *
   * Then I decided to use Kp only first (returned only p_error)
   * and observed all k_error, i_error, and k_error)
   *
   * Used Kp = 0.5 to have the car run in a sin wave. Then I applied
   * d_error (returned p_erro + d_error) to reduce the oscillations.
   *
   * Used too big of d_error, like 0.3, it reduced the path oscillations,
   * but the car tire is turning left and right like craze.
   * Used too small of de_error, like 0.001, the car path still oscillated.
   * Kd = 0.003 is the right value after sereral tries.
   *
   * There are observations on fine-tuning the Kp: If the Kp is too big, the
   * path oscillation is to strong. On the other hand, if the Kp is too small,
   * the car will fail to make the sharp turns.
   *
   * By observing i_error, I used a small Ki to make sure i_error won't
   * affect TotalError too much, since there is no car bias to correct
   * in the simulator.
   *
   * That was tuning pid variable with throttle 0.3 (25 - 30 miles/hours)
   *
   * Have tried throttle 0.6 and with differ pid values. The car is harder to
   * stay on the track this time with higher speed.
   *
   * The pid control in the simulator works as expected.
   */

  // finishing loop with dancing around, Kd is too big! pid.Init(0.3, 0.003, 0.05);
  // good1 with big turn pid.Init(0.3, 0.006, 0.02);
  // good2 Kd a bit small pid.Init(0.3, 0.003, 0.01);
  // ok pid.Init(0.35, 0.003, 0.03);
  pid.Init(0.30, 0.003, 0.03);

  h.onMessage([&pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
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
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value;
          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */
          pid.UpdateError(cte);
          steer_value = pid.TotalError();
          // DEBUG
          std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = 0.3;
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
