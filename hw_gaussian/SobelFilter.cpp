#include <cmath>
#ifndef NATIVE_SYSTEMC
#include "stratus_hls.h"
#endif

#include "SobelFilter.h"

SobelFilter::SobelFilter( sc_module_name n ): sc_module( n )
{
#ifndef NATIVE_SYSTEMC
	HLS_FLATTEN_ARRAY(val);
#endif
	SC_THREAD( do_filter );
	sensitive << i_clk.pos();
	dont_initialize();
	reset_signal_is(i_rst, false);
        
#ifndef NATIVE_SYSTEMC
	i_rgb.clk_rst(i_clk, i_rst);
  o_result.clk_rst(i_clk, i_rst);
#endif
}

SobelFilter::~SobelFilter() {}

// sobel mask
const int mask[MASK_N][MASK_X][MASK_Y] = {{{1, 2, 1}, {2, 4, 2}, {1, 2, 1}}};

void SobelFilter::do_filter() {
	{
#ifndef NATIVE_SYSTEMC
		HLS_DEFINE_PROTOCOL("main_reset");
		i_rgb.reset();
		o_result.reset();
#endif
		wait();
	}
	while (true) {
		unsigned int val_r = 0, val_g = 0, val_b = 0;

		for (unsigned int i = 0; i<MASK_N; ++i) {
			HLS_CONSTRAIN_LATENCY(0, 1, "lat00");
			val[i] = 0;
		}
		for (unsigned int v = 0; v<MASK_Y; ++v) {
			for (unsigned int u = 0; u<MASK_X; ++u) {
				sc_dt::sc_uint<24> rgb;
#ifndef NATIVE_SYSTEMC
#ifdef UNROLL1
				HLS_UNROLL_LOOP( ON, "fill loop" );
#endif
#ifdef PIPELINE1
				HLS_PIPELINE_LOOP(SOFT_STALL, 1, "pipe loop");
#endif
				{
					HLS_DEFINE_PROTOCOL("input");
					rgb = i_rgb.get();
					wait();
				}
#else
				rgb = i_rgb.read();
#endif
				val_r += rgb.range(7, 0) * mask[0][u][v];
				val_g += rgb.range(15, 8) * mask[0][u][v];
				val_b += rgb.range(23, 16) * mask[0][u][v];
			}
		}
		val_r >>= 4;
		val_g >>= 4;
		val_b >>= 4;

		sc_dt::sc_uint<24> total;
		total.range(7, 0) = val_r;
		total.range(15, 8) = val_g;
		total.range(23, 16) = val_b;


#ifndef NATIVE_SYSTEMC
		{
			HLS_DEFINE_PROTOCOL("output");
			o_result.put(total);
			wait();
		}
#else
		o_result.write(total);
#endif
	}
}
