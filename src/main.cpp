#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include "json.hpp"
#include "PID.h"

// for convenience
using nlohmann::json;
using std::string;

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

void changemode(int);
int kbhit(void);
void changemode(int dir)
{
  static struct termios oldt, newt;
  if ( dir == 1 ){
    tcgetattr( STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~( ICANON | ECHO );
    tcsetattr( STDIN_FILENO, TCSANOW, &newt);
  }
  else
    tcsetattr( STDIN_FILENO, TCSANOW, &oldt);
}

int kbhit (void)
{
  struct timeval tv;
  fd_set rdfs;
  tv.tv_sec = 0;
  tv.tv_usec = 0;
  FD_ZERO(&rdfs);
  FD_SET (STDIN_FILENO, &rdfs);
  select(STDIN_FILENO+1, &rdfs, NULL, NULL, &tv);
  return FD_ISSET(STDIN_FILENO, &rdfs);

}



int main() {
  uWS::Hub h;

  PID pid, pid_throttle;
  /**
   * TODO: Initialize the pid variable.
   */
  //pid.Init(0, 0.0000143423, 0);
  //pid.Init(0.0233613, 0.000553171, 0.0000143423);
  //pid.Init(0.0235769, 0.000553171, 0.0000143423);
  //pid.Init(0.0235769, 0.000553171, 0.0000143423);
  //pid.Init(0.02994, 0, 6.55727);
  pid_throttle.Init(0.003, 0.0, 0.00002);
  //pid.Init(0.10479, 0.000529, 7.9343);
  pid.Init(0.08982, 0.000529, 4.68512);
  h.onMessage([&pid, &pid_throttle](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, 
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    static double error = 0.0;
    static int steps = 0;
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
	  double throttle_value;
          /**
           * TODO: Calculate steering value here, remember the steering value is
           *   [-1, 1].
           * NOTE: Feel free to play around with the throttle and speed.
           *   Maybe use another PID controller to control the speed!
           */

	  pid.UpdateError(cte);
          steer_value = pid.TotalError();
	  pid_throttle.UpdateError(fabs(cte));
	  throttle_value = 0.4 + pid_throttle.TotalError();
          //std::cout << "Kp: " << pid.getKp() << std::endl;
	        //std::cout << "Ki: " << pid.getKi() << std::endl;
	        //std::cout << "Kd: " << pid.getKd() << std::endl;
	  std::cout << "Kp: " << pid.Kp[0] << " Ki: " << pid.Kp[1] << " Kd: " << pid.Kp[2] << std::endl;
          if (pid.enableTwiddle()) {
            if (steps > 7000) {
              pid.Twiddle(error/7000, 2);
              std::string reset_msg = "42[\"reset\",{}]";
              ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
              steps = 0;
              error = 0.0;
              return;
            } else {
              error += pow(cte, 2);
            }
            steps++;
          }
          
          // DEBUG
	  std::cout << "CTE: " << cte << " Steering Value: " << steer_value << " Kp: " << pid.Kp[0] << " Ki: " << pid.Kp[1] << " Kd: " << pid.Kp[2] 
		                      << std::endl;
          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value * 0.7;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket message if
    char ch; 
    changemode(1);
    if ( kbhit() ){
      ch = getchar();
      printf("%c\tFrom: P: %2.2f\tI: %f\tD:%2.2f", ch, pid.Kp[0] , pid.Kp[1], pid.Kp[2]);
      switch (ch){
        case 'T':
	  pid.twiddle = true; 
	  break;
	case 't':
	  pid.twiddle = false;
	  break;
	case 'P':
	  pid.Kp[0] += pid.Kp[0]*0.1; // Or += 0.01; 
	  break;
	case 'p':
	  pid.Kp[0] -= pid.Kp[0]*0.1; // Or -= 0.01;
	  break;
	case 'I':
	  pid.Kp[1] += pid.Kp[1]*0.1; // Or += 0.001; 
	  break;
	case 'i':
	  pid.Kp[1] -= pid.Kp[1]*0.1; // Or -= 0.001; 
	  break;
	case 'D':
	  pid.Kp[2] += pid.Kp[2]*0.1; // Or += 0.1; 
	  break;
	case 'd':
	  pid.Kp[2] -= pid.Kp[2]*0.1; // Or -=0.1;
	  break;
      }
      printf("\tto: P: %2.2f\tI: %f\tD:%2.2f\n", pid.Kp[0] , pid.Kp[1], pid.Kp[2]);
    }
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