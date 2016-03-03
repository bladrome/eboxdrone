#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

#include "ebox.h"
#include "math.h"

class EPID
{
		public:
				EPID(double setpoint, double kp, double ki, double kd);
				EPID(double kp, double ki, double kd);
				void compute(double input, double& output, uint64_t deltatime = 0);
				double compute(double input, uint64_t deltatime = 0);

				// This may be funny.
				void compute(double setpoint, double input, double& output);

				void set_pid(double kp, double ki, double kd);
				void set_point(double setpoint);
				void set_output_limits(double min, double max);
				void set_minsample_time(uint64_t sampletime);
				void set_integral_limit(double limit);

				double get_output();

				// these could be commited for optimize.
				double get_kp();
				double get_ki();
				double get_kd();
				double get_setpoint();
				double get_deltatime();
				double get_minsample_time();
				double get_proportional();
				double get_integral();
				double get_derivative();

		private:
				double KP;
				double KI;
				double KD;

				double input;
				double setpoint;
				double output;

				double error;
				double pre_error;
				double prepre_error;

				double proportional;
				double integral;
				double derivative;

				double integ_limit;
				
				double min_output;
				double max_output;

				uint64_t last_time;	//in us;
				uint64_t delta_time;

				uint64_t min_sample_time;

};


#endif
