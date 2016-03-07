#include "pidcontroller.h"

// EPID::constructor.
EPID::EPID(float setpoint, float kp, float ki, float kd)
{
		set_pid(kp, ki, kd);
		this->setpoint = setpoint;
		this->input = 0.0f;
		this->last_time = micros();
		this->delta_time = 0;
		this->output = input;

		this->error = this->pre_error = this->prepre_error = 0.0f;
		this->proportional = this->integral = this->derivative = 0.0f;
		
		this->min_output = 0;
		this->max_output = 1000;
		this->integ_limit  = fabs(this->max_output);

		this->min_sample_time = 0;
}

EPID::EPID(float kp, float ki, float kd)
{
		set_pid(kp, ki, kd);
		this->setpoint = 0;
		this->input = 0.0f;
		this->last_time = micros();
		this->delta_time = 0;
		this->output = this->input;

		this->error = this->pre_error = this->prepre_error = 0.0f;
		this->proportional = this->integral = this->derivative = 0.0f;
		
		this->min_output = 0;
		this->max_output = 1000;
		this->integ_limit  = fabs(this->max_output);

		this->min_sample_time = 0;
}

float EPID::get_output()
{
		return this->output;
}


void EPID::compute(float setpoint, float input, float& output)
{
		EPID::set_point(setpoint);
		EPID::compute(input, output, (uint64_t)0);
}

// compute output.
void EPID::compute(float input, float& output, uint64_t deltatime)
{
		uint64_t now = micros();

		this->input = input;
		this->delta_time = deltatime == 0 ? now - this->last_time : deltatime;
		this->last_time = now;

		if(this->delta_time < this->min_sample_time)
		{
				output = this->output;

				return ;
		}

		this->prepre_error = this->pre_error;
		this->pre_error = this->error;
		this->error = this->setpoint - this->input;

		float inttmp = (this->integral) + (this->delta_time) * this->KI * (this->error + this->pre_error) / 2;
		float intTerm = fabs(inttmp) > this->integ_limit ? this->integral : inttmp;
		this->proportional = this->KP * this->error;
		this->integral = intTerm;
		this->derivative = this->KD * (((this->prepre_error - this->pre_error) + (this->pre_error - this->error)) / 2) / this->delta_time;

		this->output = this->proportional + this->integral + this->derivative;

		if(this->output > this->max_output)
				this->output = this->max_output;
		if(this->output < this->min_output)
				this->output = this->min_output;
		output = this->output;

		return ;
}

// compute output.
float EPID::compute(float input, uint64_t deltatime)
{
		
		float output;
		uint64_t now = micros();

		this->input = input;
		this->delta_time = deltatime == 0 ? now - this->last_time : deltatime;
		this->last_time = now;

		if(this->delta_time < this->min_sample_time)
		{
				output = this->output;

				return output;
		}

		this->prepre_error = this->pre_error;
		this->pre_error = this->error;
		this->error = this->setpoint - this->input;

		float inttmp = (this->integral) + (this->delta_time) * this->KI * (this->error + this->pre_error) / 2;
		float intTerm = fabs(inttmp) > this->integ_limit ? this->integral : inttmp;
		this->proportional = this->KP * this->error;
		this->integral = intTerm;
		this->derivative = this->KD * (((this->prepre_error - this->pre_error) + (this->pre_error - this->error)) / 2) / this->delta_time;

		this->output = this->proportional + this->integral + this->derivative;

		if(this->output > this->max_output)
				this->output = this->max_output;
		if(this->output < this->min_output)
				this->output = this->min_output;
		output = this->output;

		return output;
}

// reset PID.
void EPID::set_pid(float kp, float ki, float kd)
{
		if(kp < 0 || ki < 0 || kd < 0)
				return;
		this->KP = kp;
		this->KI = ki;
		this->KD = kd;

}

// reset setpoint.
void EPID::set_point(float setpoint)
{
		this->setpoint = setpoint;
}

// constrain output between min and max.
void EPID::set_output_limits(float min, float max)
{
		if(min > max)
				return ;
		this->min_output = min;
		this->max_output = max;
}

// set integral limit.
void EPID::set_integral_limit(float limit)
{
		this->integ_limit = fabs(limit);
}

// set min sample time.
void EPID::set_minsample_time(uint64_t sampletime)
{
		this->min_sample_time = sampletime;
}


// return KP.
float EPID::get_kp()
{
		return this->KP;
}

// return KI.
float EPID::get_ki()
{
		return this->KI;
}

// return KD.
float EPID::get_kd()
{
		return this->KD;
}

// return setpoint.
float EPID::get_setpoint()
{
		return this->setpoint;
}

// return delta time.
float EPID::get_deltatime()
{
		return this->delta_time;
}

// return proportional.
float EPID::get_proportional()
{
		return this->proportional;
}

// return integral.
float EPID::get_integral()
{
		return this->integral;
}

// return derivative.
float EPID::get_derivative()
{
		return this->derivative;
}

// return min sample time.
float EPID::get_minsample_time()
{
		return this->min_sample_time;
}

