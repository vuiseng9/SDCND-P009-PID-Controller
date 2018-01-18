#include "PID.h"
#include <iostream>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {
    is_twiddle = false;
    twiddle_tol = 0.005;
    twiddle_endstep = 50;
    twiddle_iter = 0;
    twiddle_best_sse = 999999999;

    gain[0] = 1.0f;
    gain[1] = 0.0f;
    gain[2] = 0.0f;
   
    gain[0] = 4.4f;
    gain[1] = 0.0f;
    gain[2] = 9.3f;
 
    d_gain[0] = 1.0f;
    d_gain[1] = 1.0f;
    d_gain[2] = 1.0f;
    
    curr_param = 0;
	state = 'a';
}

PID::~PID() {}

void PID::Init(double _Kp, double _Ki, double _Kd) {
    p_error = 0;
    i_error = 0;
    d_error = 0;

    Kp = _Kp;
    Ki = _Ki;
    Kd = _Kd;
}

void PID::UpdateError(double cte) {
    d_error = cte - p_error;
    i_error += cte;
    p_error = cte;
}

double PID::TotalError() {
    return (-Kp * p_error) + (-Kd * d_error) + (-Ki * i_error);
}

void PID::Twiddle(double SSE) {
    std::cout << "I got this: " << SSE << std::endl;

	switch (state) {
		case 'a':
			if (SSE < twiddle_best_sse) {
        		twiddle_best_sse = SSE;
        		d_gain[curr_param] *= 1.1;

        		curr_param = (curr_param + 1) % 3;
				gain[curr_param] += d_gain[curr_param];
				Init(gain[0], gain[1], gain[2]);
				state = 'a';
				twiddle_iter++;
    		} else {
        		gain[curr_param] -= 2 * d_gain[curr_param];
        		Init(gain[0], gain[1], gain[2]);
				state = 'b';
    		}
			break;

		case 'b':
		    if (SSE < twiddle_best_sse) {
        		twiddle_best_sse = SSE;
        		d_gain[curr_param] *= 1.1;
    		} else {
        		gain[curr_param] += d_gain[curr_param];
        		d_gain[curr_param] *= 0.9;
    		}
    		curr_param = (curr_param + 1) % 3;
			gain[curr_param] += d_gain[curr_param];
        	Init(gain[0], gain[1], gain[2]);
			state = 'a';
			twiddle_iter++;
			break;

		default:
			std::cout << "[BUG!!!]" << std::endl;
			break;
	}
}

