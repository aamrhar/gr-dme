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
#include "dme_time_impl.h"

namespace gr {
  namespace dme {

    dme_time::sptr
    dme_time::make()
    {
      return gnuradio::get_initial_sptr
        (new dme_time_impl());
    }

    /*
     * The private constructor
     */
    dme_time_impl::dme_time_impl()
      : gr::sync_block("dme_time",
              gr::io_signature::make(1, 1, sizeof(int)),
              gr::io_signature::make(1, 1, sizeof(uint64_t)))
    {
        message_port_register_out(pmt::mp("dme1"));
        message_port_register_out(pmt::mp("dme2"));
        d_buffer=0;
        d_old=0;
        d_hold=0;
    }

    /*
     * Our virtual destructor.
     */
    dme_time_impl::~dme_time_impl()
    {
    }

    int
    dme_time_impl::work(int noutput_items,
        gr_vector_const_void_star &input_items,
        gr_vector_void_star &output_items)
    {
        const int *in = (const int *) input_items[0];
        int64_t *out = (int64_t *) output_items[0];

      // Do <+signal processing+>
        for(int i = 0; i < noutput_items; i++)
        {
            int mskd_in=in[i] & TIME_MSK;
            int diff=mskd_in-d_old;


            if (diff > 0 || diff < 2*NORMAL_STEP) //good
            {
                d_hold=mskd_in;
            }
            if (mskd_in < 10*NORMAL_STEP && diff < 0.5) //loop
            {
                d_hold=mskd_in;
                d_buffer+=TIME_LAP;
            }

            out[i]=(d_hold+d_buffer)/TIME_SCL;
            if(mskd_in != d_old)
            {
                if(in[i] & DME1_MSK)
                {
                    message_port_pub(pmt::mp("dme1"),pmt::from_uint64(out[i]));
                }
                if(in[i] & DME2_MSK)
                {
                    message_port_pub(pmt::mp("dme2"),pmt::from_uint64(out[i]));
                }
            }

            d_old=mskd_in;
        }

      // Tell runtime system how many output items we produced.
      return noutput_items;
    }

  } /* namespace dme */
} /* namespace gr */

