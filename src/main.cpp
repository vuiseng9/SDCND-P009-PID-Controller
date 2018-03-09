#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>
#include "args.hxx"

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

    args::ArgumentParser parser("an PID controller app that drives Udacity SDC Simulator Lake Track", "Running ./pid without any argument invokes pre-tuned gain.");
    args::HelpFlag help(parser, "help", "Display help menu", {'h', "help"});
    args::Group gain_grp(parser, "kp, ki, kd need to coexist", args::Group::Validators::AllOrNone);
    args::ValueFlag<float>  kp(gain_grp, "float", "set/initialize proportional gain", {"kp"});
    args::ValueFlag<float>  ki(gain_grp, "float", "set/initialize integral gain", {"ki"});
    args::ValueFlag<float>  kd(gain_grp, "float", "set/initialize derivative gain", {"kd"});
    args::Flag              twiddle(parser, "twiddle", "enable twiddle mode to tune gain", {'t', "twiddle"});
    args::ValueFlag<int>    n_step(parser, "int", "set number of step per twiddle iteration", {"n_step"});
    args::Group dgain_grp(parser, "dp, di, dd need to coexist", args::Group::Validators::AllOrNone);
    args::ValueFlag<float>  dp(dgain_grp, "float", "kp max tunable range", {"dp"});
    args::ValueFlag<float>  di(dgain_grp, "float", "ki max tunable range", {"di"});
    args::ValueFlag<float>  dd(dgain_grp, "float", "kd max tunable range", {"dd"});

    try
    {
        parser.ParseCLI(argc, argv);
    }
    catch (args::Help)
    {
        std::cout << parser;
        return 0;
    }
    catch (args::ParseError e)
    {
        std::cerr << e.what() << std::endl;
        std::cerr << parser;
        return 1;
    }
    catch (args::ValidationError e)
    {
        std::cerr << e.what() << std::endl;
        std::cerr << parser;
        return 1;
    }

    if (twiddle) {
        std::cout << "[Info] Twiddle Tuning Enabled" << std::endl;
        pid_steer.is_twiddle = true;
    }

    if (n_step) {
        if (twiddle) {
            pid_steer.twiddle_endstep = args::get(n_step);
            std::cout << "[Info] Setting twiddle n_step to " << pid_steer.twiddle_endstep << std::endl;
        } else {
            std::cout << "[Error] n_steps only works when twiddle tuning is enabled." << std::endl;
            exit(1);
        }
    }


    if (kp && ki && kd) {
        std::cout << "[Info] Use user-specified kp, ki, kd" << std::endl;
        pid_steer.gain[0] = args::get(kp);
        pid_steer.gain[1] = args::get(ki);
        pid_steer.gain[2] = args::get(kd);
    } else {
        std::cout << "[Info] Use pretuned kp, ki, kd" << std::endl;
        pid_steer.gain[0] = 0.15; 
        pid_steer.gain[1] = 0.001;
        pid_steer.gain[2] = 0.6;
    }
    
    std::cout << "[Info] Initializing PID for steering with kp: " << 
        pid_steer.gain[0] << 
        ", ki: " << pid_steer.gain[1] << 
        ", kd: " << pid_steer.gain[2] << 
        std::endl;

    pid_steer.Init(pid_steer.gain[0], pid_steer.gain[1], pid_steer.gain[2]);

    if (dp && di && dd) {
        if (twiddle) {
            std::cout << "[Info] Use user-specified dp, di, dd" << std::endl;
            pid_steer.d_gain[0] = args::get(dp);
            pid_steer.d_gain[1] = args::get(di);
            pid_steer.d_gain[2] = args::get(dd);
        } else {
            std::cout << "[Error] dp, di, dd are to be used when twiddle tuning is enabled." << std::endl;
            exit(1);
        }
    }
    
    if (twiddle) {
        std::cout << "[Info] Twiddle with initial dp: " 
				<< pid_steer.d_gain[0] <<
            	", di: " << pid_steer.d_gain[1] <<
            	", dd: " << pid_steer.d_gain[2] <<
            	std::endl; 
    }
 
  	h.onMessage([&pid_steer, &step, &SSE](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    static double previous_angle = 0;
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
          double throttle_value;

          double d_angle;
          d_angle = previous_angle - angle;
          previous_angle = angle;

          std::cout.precision(3);
 
          if (!pid_steer.is_twiddle) {         
          std::cout << "cte: "      << std::setw(8) << cte 
                    << ", speed: "  << std::setw(8) << speed 
                    << ", angle: "  << std::setw(8) << angle 
                    << ", d_angle: "  << std::setw(8) << d_angle 
                    << std::endl;
          }
          // Sum of square error - cost function for twiddle
          SSE += cte*cte;               // minimize the car gap to the reference line
          SSE += angle*angle;           // penalize large angle so that car takes small angle overall
          SSE += pow((40 - speed),2);   //reference speed is 40, ensure car is moving, also as close to reference speed

          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */

          if (pid_steer.is_twiddle) 
          {
            // Triggle twiddle loop when number of step reaching threshold or 
            // when accumulated SSE is already over best SSE 
            if ((step > pid_steer.twiddle_endstep) || (SSE > pid_steer.twiddle_best_sse)) {
                std::cout << std::endl;
                std::string reset_msg = "42[\"reset\",{}]";
                ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);

                // Call to twiddle loop - 1 to terminate, 0 continue twiddle tuning
                if (pid_steer.Twiddle(SSE)) exit(1);

                // reset step count and SSE accumulator
                step = 0;
                SSE = 0;
            } 
           
           //Print out during tuning operation 
            std::cout   << "\repoch: "      << std::setw(3) << pid_steer.twiddle_cnt/3 
                        << ", step: "       << std::setw(4) << step 
                        << ", gain_idx: "   << std::setw(1) << pid_steer.gain_idx 
                        << ", state: "      << std::setw(1) << pid_steer.state 
                        << ", kp: "         << std::setw(6) << pid_steer.Kp 
                        << ", ki: "         << std::setw(6) << pid_steer.Ki 
                        << ", kd:"          << std::setw(6) << pid_steer.Kd 
                        << ", dp: "         << std::setw(6) << pid_steer.d_gain[0] 
                        << ", di: "         << std::setw(6) << pid_steer.d_gain[1] 
                        << ", dd: "         << std::setw(6) << pid_steer.d_gain[2] 
                        << ", Best SSE: "   << std::setw(10) << pid_steer.twiddle_best_sse
                        << ", SSE: "        << std::setw(10) << SSE ;

          } 


          pid_steer.UpdateError(cte); // call to update p, i, d error term corresponding to cte
          steer_value = pid_steer.TotalError(); // call to calculate (-Kp*p_error) + (-Kd*d_error) + (-Ki*i_error)

		  /* throttle is reduced proportionally to the change of steering angle
 		   * the idea is when change of steering angle is large, it signifies a huge turn
 		   * hence we need to slow down or brake */
		  throttle_value = 0.5 - 0.3 * fabs(d_angle);
    
		  std::cout << "Actuations: throttle: " << throttle_value
          		    << ", steer: " << steer_value << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;
         
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
    //std::cout << "Connected!!!" << std::endl;
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
