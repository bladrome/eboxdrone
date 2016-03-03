#include "ebox.h"
#include <math.h>
#include "filter.h"

ELPF::ELPF(double sample_freq, double cutoff_freq)
{
		this->sample_frequency = sample_freq;
		this->cutoff_frequency = cutoff_freq;
		this->delay_element_0 = 0.0f;
		this->delay_element_1 = 0.0f;
		this->delay_element_2 = 0.0f;
		this->output = 0;

		double fr = 0;
		double ohm = 0;
		double c = 0;

		fr = sample_freq / cutoff_freq;
		ohm = tanf(PI / fr);
		c = 1.0f + 2.0f * cosf(PI / 4.0f) * ohm + ohm * ohm;

		if(this->cutoff_frequency > 0.0f)
		{
				this->b0 = ohm * ohm / c;
				this->b1 = 2.0f * b0;
				this->b2 = this->b0;
				this->a1 = 2.0f * (ohm * ohm - 1.0f) / c;
				this->a2 = (1.0f - 2.0f * cosf(PI / 4.0f) * ohm + ohm * ohm) / c;
		}

}

void ELPF::reset(double sample_freq, double cutoff_freq)
{
		if( sample_freq < 0 || cutoff_freq < 0 || sample_freq < cutoff_freq )
				return ;
		this->sample_frequency = sample_freq;
		this->cutoff_frequency = cutoff_freq;
}

double ELPF::apply(double sample)
{
		if(this->cutoff_frequency <= 0.0f)
		{
				return sample;
		}
		else
		{
				this->delay_element_0 = sample - this->delay_element_1 * this->a1 - delay_element_2 * this->a2;
				if(isnan(this->delay_element_0) || isinf(this->delay_element_0))
				{
						this->delay_element_0 = sample;
				}
				output = this->delay_element_0 * this->b0 + this->delay_element_1 * this->b1 + this->delay_element_2 * this->b2;
				this->delay_element_2 = this->delay_element_1;
				this->delay_element_1 = this->delay_element_0;

				return output;
		}

}


void ELPF::set_sample_frequency(double sample_freq)
{
		this->sample_frequency = sample_freq;

}


void ELPF::set_cutoff_frequency(double cutoff_freq)
{
		this->cutoff_frequency = cutoff_freq;

}


