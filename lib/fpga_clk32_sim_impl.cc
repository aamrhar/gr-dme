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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gnuradio/io_signature.h>
#include "fpga_clk32_sim_impl.h"

namespace gr {
  namespace dme {

    fpga_clk32_sim::sptr
    fpga_clk32_sim::make(int inc)
    {
      return gnuradio::get_initial_sptr
        (new fpga_clk32_sim_impl(inc));
    }

    /*
     * The private constructor
     */
    fpga_clk32_sim_impl::fpga_clk32_sim_impl(int inc)
      : gr::sync_block("fpga_clk32_sim",
              gr::io_signature::make(2, 2, sizeof(float)),
              gr::io_signature::make(1, 1, sizeof(int)))
    {
        d_inc=inc;
        d_counter=0;
        d_is_dme1=false;
        d_is_dme2=false;

    }

    /*
     * Our virtual destructor.
     */
    fpga_clk32_sim_impl::~fpga_clk32_sim_impl()
    {
    }

    int
    fpga_clk32_sim_impl::work(int noutput_items,
        gr_vector_const_void_star &input_items,
        gr_vector_void_star &output_items)
    {
      const float *in_dme1 = (const float *) input_items[0];
      const float *in_dme2 = (const float *) input_items[1];
      int *out = (int *) output_items[0];


      // Do <+signal processing+>
      for(int i = 0; i < noutput_items; i++)
      {
        // if rising edge dme1 set bit
          if (in_dme1[i]>0)
          {
              if(!d_is_dme1)
              {
                  d_counter+=DME1_DETEC;
                  d_is_dme1=true;
              }
          } else
          {
              d_is_dme1=false;
          }

          if (in_dme2[i]>0)
          {
              if(!d_is_dme2)
              {
                  d_counter+=DME2_DETEC;
                  d_is_dme2=true;
              }
          } else
          {
              d_is_dme2=false;
          }

        out[i]=d_counter;
        d_counter=(d_counter+d_inc) & TIME_MSK;
      }
      // Tell runtime system how many output items we produced.
      return noutput_items;
    }

  } /* namespace dme */
} /* namespace gr */

