#include "PID.h"
#include <iostream>

using namespace std;

enum STATE
{
    ASCENT = 0,
    DESCENT
};
/*
* TODO: Complete the PID class.
*/

PID::PID() {
    is_twiddle        = false;
    is_twiddle_init   = false;
    twiddle_tol       = 0.005;
    twiddle_endstep   = 800;
    twiddle_cnt       = 0;
    twiddle_best_sse  = 9999999;

    gain[0] = 0.0f;
    gain[1] = 0.0f;
    gain[2] = 0.0f;
 
    d_gain[0] = 1.0f;
    d_gain[1] = 1.0f;
    d_gain[2] = 1.0f;
    
    gain_idx = 0;   //0: P, 1: I, 2: D
	state = ASCENT;
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
    d_error =   cte - p_error;
    i_error +=  cte;
    p_error =   cte;
}

double PID::TotalError() {
    return (-Kp * p_error) + (-Kd * d_error) + (-Ki * i_error);
}

int PID::Twiddle(double SSE) {
    
    if (!is_twiddle_init) {
        twiddle_best_sse = SSE;
        best_gain[0]=gain[0];
        best_gain[1]=gain[1];
        best_gain[2]=gain[2];

        gain[gain_idx] += d_gain[gain_idx];
        Init(gain[0], gain[1], gain[2]);
        
        state = ASCENT;
        is_twiddle_init = true;
        return 0;
    }

    // Exit twiddle if total of all delta is lower than tolerance
    if (d_gain[0] + d_gain[1] + d_gain[2] < twiddle_tol) {
        std::cout << "[Info] Twiddle Tuning Complete! Best SSE: "<< twiddle_best_sse
            << ", kp: "  << best_gain[0] 
            << ", ki: "  << best_gain[1]
            << ", kd: "  << best_gain[2]
            << std::endl;
        return 1;
    }

	switch (state) {
		case ASCENT:
			if (SSE < twiddle_best_sse) {
        		twiddle_best_sse = SSE;
				best_gain[0]=gain[0];
		        best_gain[1]=gain[1];
        		best_gain[2]=gain[2];

        		d_gain[gain_idx] *= 1.1;
        		gain_idx = (gain_idx + 1) % 3;
				gain[gain_idx] += d_gain[gain_idx];
				Init(gain[0], gain[1], gain[2]);

				state = ASCENT;
				twiddle_cnt++;
    		} else {
        		gain[gain_idx] -= 2 * d_gain[gain_idx];
        		Init(gain[0], gain[1], gain[2]);
				state = DESCENT;
    		}
			break;

		case DESCENT:
		    if (SSE < twiddle_best_sse) {
        		twiddle_best_sse = SSE;
				best_gain[0]=gain[0];
                best_gain[1]=gain[1];
                best_gain[2]=gain[2];
        		d_gain[gain_idx] *= 1.1;
    		} else {
        		gain[gain_idx] += d_gain[gain_idx];
        		d_gain[gain_idx] *= 0.9;
    		}
    		gain_idx = (gain_idx + 1) % 3;
			gain[gain_idx] += d_gain[gain_idx];
        	Init(gain[0], gain[1], gain[2]);
			state = ASCENT;
			twiddle_cnt++;
			break;

		default:
			std::cout << "[ERROR] - BUG!!!" << std::endl;
			break;
	}
    return 0;
}

