#include <cmath>
#include <iomanip>

#include "SobelFilter.h"

SobelFilter::SobelFilter(sc_module_name n)
    : sc_module(n), t_skt("t_skt"), base_offset(0) {
  SC_THREAD(do_filter);

  t_skt.register_b_transport(this, &SobelFilter::blocking_transport);
}

SobelFilter::~SobelFilter() = default;

void SobelFilter::do_filter() {
  while (true) {
    unsigned int i = 0;
    val_r = val_g = val_b = 0;
    for (unsigned int v = 0; v < MASK_Y; ++v) {
      for (unsigned int u = 0; u < MASK_X; ++u) {
        val_r += i_r.read() * mask[i][u][v];
        val_g += i_g.read() * mask[i][u][v];
        val_b += i_b.read() * mask[i][u][v];
      }
    }
	//val_r = std::min(abs(int(factor * val_r)), 255);
	//val_g = std::min(abs(int(factor * val_g)), 255);
	//val_b = std::min(abs(int(factor * val_b)), 255);
	val_r = val_r >> 4;
	val_g = val_g >> 4;
	val_b = val_b >> 4;
    //printf("sobeil: %d %d %d\n", val_r, val_g, val_b);
    o_result_r.write(val_r);
    o_result_g.write(val_g);
    o_result_b.write(val_b);
    //wait(10); //emulate module delay
    // why can't wait?
  }
}

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
