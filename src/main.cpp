#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>
#include "cxxopts.hpp"

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

int main(int argc, char* argv[])
{
 
    uWS::Hub h;

    PID pid_steer;
    int step = 0;
    float SSE = 0; //sum of square error for twiddle

    try
    {
        cxxopts::Options options(argv[0], "");
        options
            .positional_help("[optional args]")
            .show_positional_help();

        options.add_options()
            ("h,help",                 "Print Usage")
            ("t,twiddle",              "enable twiddle to tune Kp, Ki, Kd", cxxopts::value<bool>())
            ("p,proportional_gain",    "set/initialize proportional gain, Kp", cxxopts::value<float>())
            ("i,integral_gain",        "set/initialize integral gain, Kp", cxxopts::value<float>())
            ("d,derivative_gain",      "set/initialize derivative, Kp", cxxopts::value<float>())
            ;

        auto args = options.parse(argc, argv);

        if (args["twiddle"].as<bool>()) {
            std::cout << "-I- Twiddle Tuning Enabled" << std::endl;
            pid_steer.is_twiddle = true;
        }

        if (args.count("proportional_gain") + args.count("integral_gain") + args.count("derivative_gain") == 3) {
            std::cout << "-I- Use user-specified Kp, Ki, Kd" << std::endl;
            
            if (args.count("proportional_gain")) 
                pid_steer.gain[0] = args["p"].as<float>();

            if (args.count("integral_gain"))
                pid_steer.gain[1] = args["i"].as<float>();

            if (args.count("derivative_gain"))
                pid_steer.gain[2] = args["d"].as<float>();

        } else {
            std::cout << "-I- Use pretuned Kp, Ki, Kd" << std::endl;
            pid_steer.gain[0] = 4.35924;
            pid_steer.gain[1] = 0;
            pid_steer.gain[2] = 9.3;
        }
        
        std::cout << "Initializing PID for steering with Kp: " << 
            pid_steer.gain[0] << 
            ", Ki: " << pid_steer.gain[1] << 
            ", Kd: " << pid_steer.gain[2] << 
            std::endl;

        pid_steer.Init(pid_steer.gain[0], pid_steer.gain[1], pid_steer.gain[2]);

    } catch (const cxxopts::OptionException& e)
    {
        std::cout << "error parsing options: " << e.what() << std::endl;
        exit(1);
    }

/*
    int step = 0;
    float SSE = 0; //sum of square error for twiddle

    if (pid_steer.is_twiddle) {
        pid_steer.Init(pid_steer.gain[0], pid_steer.gain[1], pid_steer.gain[2]);  
        //pid_steer.Init(4.35924, 0, 9.33881);
    } else {
        // Tuned Kp, Ki, Kd
        // 4.35924, 0, 9.33881
      pid_steer.Init(4.35924, 0, 9.33881);
    }
*/
  h.onMessage([&pid_steer, &step, &SSE](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    step++;
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
          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */

          if (pid_steer.is_twiddle) 
          {

            if ((step > pid_steer.twiddle_endstep) || (SSE > pid_steer.twiddle_best_sse)) {
                std::string reset_msg = "42[\"reset\",{}]";
                ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);

                //next iteration
                pid_steer.Twiddle(SSE);
                //pid_steer.Init(pid_steer.gain[0], pid_steer.gain[1], pid_steer.gain[2]);
                step = 0;
                SSE = 0;
                
                //iter++;
            }

            SSE += cte*cte;
            std::cout << "iter: " << pid_steer.twiddle_iter << ", step: " << step << ", curr_param:" << pid_steer.curr_param 
                << ", state: " << pid_steer.state <<
               ", gain: " << pid_steer.Kp << ", " << pid_steer.Ki << ", " << pid_steer.Kd << 
               ", d_gain: " << pid_steer.d_gain[0] << ", " << pid_steer.d_gain[1] << ", " << pid_steer.d_gain[2] <<
               ", SSE: " << std::setw(15) << SSE << "\n";

          } 

          pid_steer.UpdateError(cte);
          steer_value = pid_steer.TotalError();
        
          // DEBUG
          // std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;
          

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = 0.3;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          // std::cout << step << ": "<< msg << std::endl;
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
