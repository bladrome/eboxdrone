#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

#include "ebox.h"
#include "math.h"

class EPID
{
#define	GETVERBOSE 1
		public:
				EPID(float setpoint, float kp, float ki, float kd);
				EPID(float kp, float ki, float kd);

				void compute(float input, float& output, uint64_t deltatime = 0);
				void compute(float setpoint, float input, float& output, uint64_t deltatime = 0);

				void set_pid(float kp, float ki, float kd);
				void set_point(float setpoint);
				void set_error_weight(float error_weight);
				void set_output_limits(float min, float max);
				void set_integral_limit(float min, float max);
				void set_minsample_time(uint64_t sampletime);

				float get_output();

				// these could be commited for optimize.
#ifdef GETVERBOSE
				float get_kp();
				float get_ki();
				float get_kd();
				float get_setpoint();
				float get_deltatime();
				float get_minsample_time();
				float get_error_weight();
				float get_proportional();
				float get_integral();
				float get_derivative();
#endif


		private:
				float KP;
				float KI;
				float KD;

				float input;
				float setpoint;
				float output;

				float error;
				float pre_error;
				float prepre_error;

				float proportional;
				float integral;
				float derivative;

				float pre_error_weight;
				float error_weight;

				float integ_min_limit;
				float integ_max_limit;
				
				float min_output;
				float max_output;

				uint64_t last_time;	//in us;
				uint64_t delta_time;

				uint64_t min_sample_time;

};


#endif
