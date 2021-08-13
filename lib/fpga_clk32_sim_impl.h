/* -*- c++ -*- */
/* 
 * Copyright 2016 <+YOU OR YOUR COMPANY+>.
 * 
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 * 
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this software; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#ifndef INCLUDED_DME_FPGA_CLK32_SIM_IMPL_H
#define INCLUDED_DME_FPGA_CLK32_SIM_IMPL_H

#include <dme/fpga_clk32_sim.h>

#define TIME_MSK    0x3FFFFFFF
#define DME1_DETEC  0x80000000
#define DME2_DETEC  0x40000000

namespace gr {
  namespace dme {

    class fpga_clk32_sim_impl : public fpga_clk32_sim
    {
     private:
      int d_inc;
      int d_counter;
      bool d_is_dme1;
      bool d_is_dme2;

     public:
      fpga_clk32_sim_impl(int inc);
      ~fpga_clk32_sim_impl();

      // Where all the action really happens
      int work(int noutput_items,
         gr_vector_const_void_star &input_items,
         gr_vector_void_star &output_items);
    };

  } // namespace dme
} // namespace gr

#endif /* INCLUDED_DME_FPGA_CLK32_SIM_IMPL_H */

