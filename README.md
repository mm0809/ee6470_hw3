# EE6470 hw3

## gaussian blur (HLS)

### How to run
```shell
$ cd hw_gaussian
$ cd stratus
$ make sim_V_@
```
@ can be {BASIC, DPA, UNROLL, PIPE, UNROLLDPA, PIPEDPA}

### essential code

```cpp
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
```
The inner loop will unroll or pipeline.

### Area and time

|              | time     | area |
| ------------ | -------- | ---- |
| BASIC        | 30801910 | 1702 |
| DPA          | 17039350 | 2887 |
| UNROLL       | 22282230 | 1866 |
| PIPE         | 11141110 | 3248 |
| DPA + UNROLL | 17039350 | 2887 |
| DPA + PIPE   | 11141110 | 3154 |

If we want a better area, the `BASIC` or `UNROLL` version are good choice.
If we want a better throughput `DPA + PIPE` it the best one.

## Timing annotation of Gaussian Blur module(TLM)

### annotate the latency

The total time of HLS version is 30801910ns = 3080191 cycles.
Total cycels = 256 * 256 * {9 * In_time + 1 * Out_time + Compute time} = 3080191
From the code of HLS, we can know that the input and ouput cycles is 1, so we can get the **compute time is about 37 cycles**.


### how to count pipeline verison
![](https://i.imgur.com/OrnsEBp.png)
According to the graph, the total time of the module is **In * num + compute + out**.
### code

In `SimpleBus.h`.
```cpp
  sc_core::sc_time delay(transaction_type &trans) {
    // Note that 4 means bus width is 4 bytes; not good enough coding.
    return (1 + trans.get_data_length() / 4) * clock_period; //model interconnect delay
    //return 0 * clock_period; // no interconnect delay
  }
```
Each input and output through the bus will have delay = `(1 + trans.get_data_length() / 4) * clock_period`.

But we don't need to count all the output delay, so in `SobelFilter.cpp` I set the delay time of output to 0. (Line：11)
```cpp=
void SobelFilter::blocking_transport(tlm::tlm_generic_payload &payload,
                                     sc_core::sc_time &delay) {
  sc_dt::uint64 addr = payload.get_address();
  addr -= base_offset;
  unsigned char *mask_ptr = payload.get_byte_enable_ptr();
  unsigned char *data_ptr = payload.get_data_ptr();
  //word buffer;
  word o_buffer[3];
  switch (payload.get_command()) {
  case tlm::TLM_READ_COMMAND:
    delay = sc_core::sc_time(0, SC_NS);
    switch (addr) {
    case SOBEL_FILTER_RESULT_ADDR:
      o_buffer[0].uint = o_result_r.read();
      o_buffer[1].uint = o_result_g.read();
      o_buffer[2].uint = o_result_b.read();
      break;
    case SOBEL_FILTER_CHECK_ADDR:
      if (o_result_r.num_available() && o_result_g.num_available() &&
          o_result_b.num_available())
          o_buffer[0].uint = 1;
      else 
        o_buffer[0].uint = 0;
    break;
    default:
      std::cerr << "Error! SobelFilter::blocking_transport: address 0x"
                << std::setfill('0') << std::setw(8) << std::hex << addr
                << std::dec << " is not valid" << std::endl;
    }
    // r
    data_ptr[0] = o_buffer[0].uc[0];
    data_ptr[1] = o_buffer[0].uc[1];
    data_ptr[2] = o_buffer[0].uc[2];
    data_ptr[3] = o_buffer[0].uc[3];
    // g
    data_ptr[4] = o_buffer[1].uc[0];
    data_ptr[5] = o_buffer[1].uc[1];
    data_ptr[6] = o_buffer[1].uc[2];
    data_ptr[7] = o_buffer[1].uc[3];
    // b
    data_ptr[8]  = o_buffer[2].uc[0];
    data_ptr[9]  = o_buffer[2].uc[1];
    data_ptr[10] = o_buffer[2].uc[2];
    data_ptr[11] = o_buffer[2].uc[3];
    break;
  case tlm::TLM_WRITE_COMMAND:
    switch (addr) {
    case SOBEL_FILTER_R_ADDR:
      if (mask_ptr[0] == 0xff) {
        i_r.write(data_ptr[0]);
      }
      if (mask_ptr[1] == 0xff) {
        i_g.write(data_ptr[1]);
      }
      if (mask_ptr[2] == 0xff) {
        i_b.write(data_ptr[2]);
      }
      break;
    default:
      std::cerr << "Error! SobelFilter::blocking_transport: address 0x"
                << std::setfill('0') << std::setw(8) << std::hex << addr
                << std::dec << " is not valid" << std::endl;
    }
    break;
  case tlm::TLM_IGNORE_COMMAND:
    payload.set_response_status(tlm::TLM_GENERIC_ERROR_RESPONSE);
    return;
  default:
    payload.set_response_status(tlm::TLM_GENERIC_ERROR_RESPONSE);
    return;
  }
  payload.set_response_status(tlm::TLM_OK_RESPONSE); // Always OK
}
```

In the end of `Testbench.cpp` I add one unit of cimpute time and output time.

```cpp
  wait(10 * CLOCK_PERIOD, SC_NS);		// delay for last compute
  wait(4 * CLOCK_PERIOD, SC_NS);		// delay for last output
  sc_stop();
```

total = 256 * 256 * 9 * 2 + 10 + 4 = 1179662 (cycles)
If each cycles is 10ns, then total time is 11796620ns whis is close to 11141110ns(PIPE in HLS).
SO, I can guess the compute time of the synthesis result of PIPLINE version is 10 cycles.

![](https://i.imgur.com/QJeRLGb.png)

