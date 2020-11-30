#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include "json.hpp"
#include "PID.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;
using std::cout;
using std::endl;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != string::npos) {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main( int argc, char *argv[] ){
  uWS::Hub h;

  PID pid;
  /**
   * TODO: Initialize the pid variable.
   */

  double init_Kp = atof( argv[1] ); // 1;
  double init_Ki = atof( argv[2] ); // 0;
  double init_kd = atof( argv[3] ); //0;

  pid.Init( init_Kp, init_Ki, init_kd);


  int numberOfRuns = 0;
  int maxRuns = 5;
  std::vector<int> stepsCount(maxRuns, 0); /// vector of length maxRuns initilized with 0
  bool inTrack = true;
  h.onMessage([&pid, &stepsCount, &inTrack, &numberOfRuns, &maxRuns](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, 
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(string(data).substr(0, length));

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<string>());
          double speed = std::stod(j[1]["speed"].get<string>());
          double angle = std::stod(j[1]["steering_angle"].get<string>());
          double steer_value;
          /**
           * TODO: Calculate steering value here, remember the steering value is
           *   [-1, 1].
           * NOTE: Feel free to play around with the throttle and speed.
           *   Maybe use another PID controller to control the speed!
           */
          

          if ( inTrack )
          {
            if( abs(cte) < 2 )
            {
              pid.UpdateError(cte);
              steer_value = pid.TotalError();

              //double controlled_speed = 25 - 5 * abs(cte);
              

              stepsCount[numberOfRuns]++;

            }
            else if ( stepsCount[numberOfRuns] > 0 ) // Car considered outside of track only if actual steps were run
            {
              std::cout << "---- Outside of track -----" << cte << std::endl;

              //inTrack = false;
              std::cout << "Car went out of the track after " << stepsCount[numberOfRuns] << " steps ... " << std::endl;
              std::string reset_msg = "42[\"reset\",{}]";
              ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
              std::cout << "Restarting car from initial position ... sleeping" << std::endl;

              // apply twiddle algo to change pid parameters ...
              pid.Twiddle();

              numberOfRuns ++;

              if (numberOfRuns == maxRuns)
              {
                inTrack = false;
                std::cout << "Car finished all runs" << std::endl;
                for (int x : stepsCount) 
                  std::cout << x << " ";
              }
            }
            
            json msgJson;
            msgJson["steering_angle"] = steer_value;
            msgJson["throttle"] = 0.4;

            // DEBUG
            std::cout << "CTE: " << cte << " Steering Value: " << steer_value << " Run: " << numberOfRuns
                      << std::endl;

            auto msg = "42[\"steer\"," + msgJson.dump() + "]";
            std::cout << msg << std::endl;
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          }


        } // end "telemetry" if
      } else {
        // Manual driving
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket message if
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