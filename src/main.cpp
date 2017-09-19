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

//char Twiddle_check(PID& pid, double& error, char& flag){
//
//  double tol = 0.00001;
//
//  if(pid.d_Kp+pid.d_Ki+pid.d_Kd < tol){
//    return 'e';
//  }
//
//  if (flag == 'd'){
//    if(error < pid.best_error){
//      pid.best_error = error;
//      pid.d_Kd *= 1.005;
//    }
//    else{
//      pid.Kd += pid.d_Kd;
//      pid.d_Kd *= 0.095 ;
//    }
//    return 'p';
//  }
//  else if (flag == 'i'){
//    if(error < pid.best_error){
//      pid.best_error = error;
//      pid.d_Ki *= 1.005;
//    }
//    else{
//      pid.Ki += pid.d_Ki;
//      pid.d_Ki *= 0.095 ;
//    }
//    return 'd';
//  }
//  else if (flag == 'p'){
//    if(error < pid.best_error){
//      pid.best_error = error;
//      pid.d_Kp *= 1.005;
//    }
//    else{
//      pid.Kp += pid.d_Kp;
//      pid.d_Kp *= 0.095 ;
//    }
//    return 'i';
//  }
//}

void FirstIncrement(PID &pid, char flag){
  switch (flag){
    case 'p':
      pid.Kp += pid.dKp;
      break;
    case  'i':
      pid.Ki += pid.dKi;
      break;
    case 'd':
      pid.Kd += pid.dKd;
      break;
    default :
      std::cout << "Invalid flag - FirstIncrement" << std::endl;
  }
}

void SmallIncrement(PID &pid, char flag){
  switch (flag){
      case 'p':
        if(pid.error < pid.best_error){
          pid.best_error = pid.error;
          pid.dKp *= 1.005;
        }
        else{
          pid.Kp += pid.dKp;
          pid.dKp *= 0.095;
        }
        break;
      case  'i':
        if(pid.error < pid.best_error){
          pid.best_error = pid.error;
          pid.dKi *= 1.005;
        }
        else{
          pid.Ki += pid.dKi;
          pid.dKi *= 0.095;
        }
        break;
      case 'd':
        if(pid.error < pid.best_error){
          pid.best_error = pid.error;
          pid.dKd *= 1.005;
        }
        else{
          pid.Kd += pid.dKd;
          pid.dKd *= 0.095;
        }
        break;
      default :
        std::cout << "Invalid flag - SmallIncrement" << std::endl;
  }
}
int main()
{
  uWS::Hub h;

  PID pid;

  // TODO: Initialize the pid variable.
  //pid.Kp = 0.2;
  //pid.Ki = 0.004;
  //pid.Kd = 3.0;

  pid.Init(0.2, 0.004, 3.0, 0.01, 0.0001, 0.1, 1, 'p');
  //pid_t.Init(0.0, 0.0, 0.0, 0.1, 0.1, 0.1);

  h.onMessage([&pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value;
          double prev_error = 0.0;
          double throttle = 0.6;
          double tol = 0.00001;

          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */

          pid.UpdateError(cte);
          steer_value = pid.TotalError();

          if(pid.avg_count >= 100){
            pid.error /= pid.avg_count;

            if((pid.dKp+pid.dKi+pid.dKd) > tol){
              switch(pid.param){
                case 'p':
                  //TODO
                  switch(pid.process_step){
                    case 1:
                      FirstIncrement(pid, 'p');
                      pid.process_step = 2;
                      break;
                    case 2:
                      if(pid.error < pid.best_error){  //IsImproved?
                        pid.best_error = pid.error;
                        pid.dKp *= 1.05;
                        pid.param = 'i';
                        break;
                      }
                      else{
                        //TODO - Not Improved steps
                        pid.Kp -= 2*pid.dKp;
                        pid.process_step = 3;
                        break;
                      }
                      std::cout << "Neither 'Is Improved' Steps Activated - debug" << std::endl;
                    case 3:
                      //TODO
                      SmallIncrement(pid, 'p');
                      pid.param = 'i';
                      pid.process_step = 1;
                      break;
                  }
                  break;
                case 'i':
                  //TODO
                  switch(pid.process_step){
                    case 1:
                      FirstIncrement(pid, 'i');
                      pid.process_step = 2;
                      break;
                    case 2:
                      if(pid.error < pid.best_error){  //IsImproved?
                        pid.best_error = pid.error;
                        pid.dKi *= 1.05;
                        pid.param = 'd';
                        break;
                      }
                      else{
                        //TODO - Not Improved steps
                        pid.Ki -= 2*pid.dKi;
                        pid.process_step = 3;
                        break;
                      }
                      std::cout << "Neither 'Is Improved' Steps Activated - debug" << std::endl;
                    case 3:
                      //TODO
                      SmallIncrement(pid, 'i');
                      pid.param = 'd';
                      pid.process_step = 1;
                      break;
                  }
                  break;
                case 'd':
                  //TODO
                  switch(pid.process_step){
                    case 1:
                      FirstIncrement(pid, 'd');
                      pid.process_step = 2;
                      break;
                    case 2:
                      if(pid.error < pid.best_error){  //IsImproved?
                        pid.best_error = pid.error;
                        pid.dKd *= 1.05;
                        pid.param = 'p';
                        break;
                      }
                      else{
                        //TODO - Not Improved steps
                        pid.Kd -= 2*pid.dKd;
                        pid.process_step = 3;
                        break;
                      }
                      std::cout << "Neither 'Is Improved' Steps Activated - debug" << std::endl;
                    case 3:
                      //TODO
                      SmallIncrement(pid, 'd');
                      pid.param = 'p';
                      pid.process_step = 1;
                      break;
                  }
                  break;
              }
            }

            pid.error = 0;
            pid.avg_count = 0;
          }
          else{
            pid.error += pow(cte,2);
            pid.avg_count += 1;
          }


          // DEBUG
          std::cout << "Error: " << pid.error << "Best Error: " << pid.best_error << std::endl;
          std::cout << "P: " << pid.Kp << "I: " << pid.Ki << "D: " << pid.Kd <<std::endl;
          std::cout << " Steering Value: " << steer_value << std::endl;


          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle;
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
