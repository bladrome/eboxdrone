#ifndef FILTER_H
#define FILTER_H

class ELPF
{
		public:
				// Easy to know what these parameter is.
				ELPF(double sample_freq = 100, double cutoff_freq = 30);
				
				void reset(double sample_freq, double cutoff_freq);

				void set_sample_frequency(double sample_freq);

				void set_cutoff_frequency(double cutoff_freq);

				double apply(double sample);

		private:
				double sample_frequency;
				double cutoff_frequency;
				double a1;
				double a2;
				double b0;
				double b1;
				double b2;
				double delay_element_0;
				double delay_element_1;
				double delay_element_2;
				double output;

};

#endif
